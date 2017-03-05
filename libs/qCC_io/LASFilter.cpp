//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifdef CC_LAS_SUPPORT

#include "LASFilter.h"

//Local
#include "LASOpenDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include "ccColorScalesManager.h"

//CCLib
#include <CCPlatform.h>

//Liblas
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/factory.hpp>	// liblas::ReaderFactory
Q_DECLARE_METATYPE(liblas::SpatialReference)

//Qt
#include <QFileInfo>
#include <QSharedPointer>
#include <QInputDialog>

//Qt gui
#include <ui_saveLASFileDlg.h>

//System
#include <string.h>
#include <fstream>				// std::ifstream
#include <iostream>				// std::cout

static const char s_LAS_SRS_Key[] = "LAS.spatialReference";

//! LAS Save dialog
class LASSaveDlg : public QDialog, public Ui::SaveLASFileDialog
{
public:
	explicit LASSaveDlg(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::SaveLASFileDialog()
	{
		setupUi(this);
	}
};

bool LASFilter::canLoadExtension(QString upperCaseExt) const
{
	return (	upperCaseExt == "LAS"
			||	upperCaseExt == "LAZ");
}

bool LASFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POINT_CLOUD)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

//! Custom ("Extra bytes") field
struct ExtraLasField : LasField
{
	//! Extra field type
	enum Type {	EXTRA_INVALID	= 0,
				EXTRA_UINT8		= 1,
				EXTRA_INT8		= 2,
				EXTRA_UINT16	= 3,
				EXTRA_INT16		= 4,
				EXTRA_UINT32	= 5,
				EXTRA_INT32		= 6,
				EXTRA_UINT64	= 7,
				EXTRA_INT64		= 8,
				EXTRA_FLOAT		= 9,
				EXTRA_DOUBLE	= 10 };

	//! Default constructor
	ExtraLasField(QString name, Type type, int off, double defaultVal = 0, double min = 0.0, double max = -1.0)
		: LasField(LAS_EXTRA,defaultVal,min,max)
		, fieldName(name)
		, valType(type)
		, dataOffset(off)
		, scale(1.0)
		, offset(0.0)
	{}

	//reimplemented from LasField
	virtual inline QString getName() const	{ return fieldName; }

	//! Returns the size (in bytes) of the specified type
	static size_t GetSizeBytes(Type type)
	{
		switch(type)
		{
		case EXTRA_INVALID:
			return 0;
		case EXTRA_UINT8:
		case EXTRA_INT8:
			return 1;
			break;
		case EXTRA_UINT16:
		case EXTRA_INT16:
			return 2;
			break;
		case EXTRA_UINT32:
		case EXTRA_INT32:
		case EXTRA_FLOAT:
			return 4;
			break;
		case EXTRA_UINT64:
		case EXTRA_INT64:
		case EXTRA_DOUBLE:
			return 8;
			break;
		default:
			assert(false);
			break;
		}

		return 0;
	}

	QString fieldName;
	Type valType;
	int dataOffset;
	double scale;
	double offset;
};

//! Semi persistent save dialog
QSharedPointer<LASSaveDlg> s_saveDlg(0);

//! Extended LAS writer
class LASWriter
{
	public:

		LASWriter()
			: w(0)
			, writeCounter(0)
		{}

		virtual ~LASWriter()
		{
			close();
		}

		bool open(const QString& _filename, liblas::Header const& header)
		{
			if (w)
			{
				//already opened?!
				assert(false);
				return true;
			}
			
			ofs.open(qPrintable(_filename), std::ios::out | std::ios::binary); //DGM: warning, toStdString doesn't preserve "local" characters
			if (ofs.fail())
			{
				return false;
			}

			w = new liblas::Writer(ofs, header);
			filename = _filename;
			writeCounter = 0;
			return true;
		};

		inline const liblas::Writer* writer() const { return w; }

		void write(const liblas::Point& P)
		{
			if (w)
			{
				w->WritePoint(P);
				++writeCounter;
			}
		}

		void close()
		{
			if (w)
			{
				delete w;
				w = 0;
				ofs.close();
			}
		}

		inline size_t writtenCount() const { return writeCounter; }

		const QString& getFilename() const { return filename; };

protected:

	liblas::Writer* w;
	std::ofstream ofs;
	QString filename;
	size_t writeCounter;
};

CC_FILE_ERROR LASFilter::saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(entity);
	if (!theCloud)
	{
		ccLog::Warning("[LAS] This filter can only save one cloud at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	unsigned numberOfPoints = theCloud->size();
	if (numberOfPoints == 0)
	{
		ccLog::Warning("[LAS] Cloud is empty!");
		return CC_FERR_NO_SAVE;
	}

	//colors
	bool hasColor = theCloud->hasColors();

	//additional fields (as scalar fields)
	std::vector<LasField> fieldsToSave;

	if (theCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(theCloud);
		
		//match cloud SFs with official LAS fields
		LasField::GetLASFields(pc, fieldsToSave);
	}

	LASWriter lasWriter;
	try
	{
		liblas::Header header;

		//LAZ support based on extension!
		if (QFileInfo(filename).suffix().toUpper() == "LAZ")
		{
			header.SetCompressed(true);
		}

		//header.SetDataFormatId(liblas::ePointFormat3);
		CCVector3d bbMin, bbMax;
		if (!theCloud->getGlobalBB(bbMin, bbMax))
		{
			return CC_FERR_NO_SAVE;
		}
		
		header.SetMin(bbMin.x, bbMin.y, bbMin.z);
		header.SetMax(bbMax.x, bbMax.y, bbMax.z);
		CCVector3d diag = bbMax - bbMin;

		//Set offset & scale, as points will be stored as boost::int32_t values (between 0 and 4294967296)
		//int_value = (double_value-offset)/scale
		header.SetOffset(bbMin.x, bbMin.y, bbMin.z);

		//let the user choose between the original scale and the 'optimal' one (for accuracy, not for compression ;)
		bool hasScaleMetaData = false;
		CCVector3d lasScale(0, 0, 0);
		lasScale.x = theCloud->getMetaData(LAS_SCALE_X_META_DATA).toDouble(&hasScaleMetaData);
		if (hasScaleMetaData)
		{
			lasScale.y = theCloud->getMetaData(LAS_SCALE_Y_META_DATA).toDouble(&hasScaleMetaData);
			if (hasScaleMetaData)
			{
				lasScale.z = theCloud->getMetaData(LAS_SCALE_Z_META_DATA).toDouble(&hasScaleMetaData);
			}
		}

		//optimal scale (for accuracy) --> 1e-9 because the maximum integer is roughly +/-2e+9
		CCVector3d optimalScale(1.0e-9 * std::max<double>(diag.x, ZERO_TOLERANCE),
								1.0e-9 * std::max<double>(diag.y, ZERO_TOLERANCE),
								1.0e-9 * std::max<double>(diag.z, ZERO_TOLERANCE));

		if (parameters.alwaysDisplaySaveDialog)
		{
			if (!s_saveDlg)
				s_saveDlg = QSharedPointer<LASSaveDlg>(new LASSaveDlg(0));
			s_saveDlg->bestAccuracyLabel->setText(QString("(%1, %2, %3)").arg(optimalScale.x).arg(optimalScale.y).arg(optimalScale.z));

			if (hasScaleMetaData)
			{
				s_saveDlg->origAccuracyLabel->setText(QString("(%1, %2, %3)").arg(lasScale.x).arg(lasScale.y).arg(lasScale.z));
			}
			else
			{
				s_saveDlg->origAccuracyLabel->setText("none");
				if (s_saveDlg->origRadioButton->isChecked())
					s_saveDlg->bestRadioButton->setChecked(true);
				s_saveDlg->origRadioButton->setEnabled(false);
			}

			s_saveDlg->exec();

			if (s_saveDlg->bestRadioButton->isChecked())
			{
				lasScale = optimalScale;
			}
			else if (s_saveDlg->customRadioButton->isChecked())
			{
				double s = s_saveDlg->customScaleDoubleSpinBox->value();
				lasScale = CCVector3d(s, s, s);
			}
			//else
			//{
			//	lasScale = lasScale;
			//}
		}
		else if (!hasScaleMetaData)
		{
			lasScale = optimalScale;
		}

		header.SetScale(lasScale.x,
						lasScale.y,
						lasScale.z);
		header.SetPointRecordsCount(numberOfPoints);

		if (theCloud->hasMetaData(s_LAS_SRS_Key))
		{
			//restore the SRS if possible
			liblas::SpatialReference srs = theCloud->getMetaData(s_LAS_SRS_Key).value<liblas::SpatialReference>();
			header.SetSRS(srs);
		}

		//DGM FIXME: doesn't seem to do anything;)
		//if (!hasColor) //we must remove the colors dimensions!
		//{
		//	liblas::Schema schema = header.GetSchema();
		//	boost::optional< liblas::Dimension const& > redDim = schema.GetDimension("Red");
		//	if (redDim)
		//		schema.RemoveDimension(redDim.get());
		//	boost::optional< liblas::Dimension const& > greenDim = schema.GetDimension("Green");
		//	if (greenDim)
		//		schema.RemoveDimension(greenDim.get());
		//	boost::optional< liblas::Dimension const& > blueDim = schema.GetDimension("Blue");
		//	if (blueDim)
		//		schema.RemoveDimension(blueDim.get());
		//	header.SetSchema(schema);
		//}
		//header.SetDataFormatId(Header::ePointFormat1);

		//open binary file for writing
		if (!lasWriter.open(filename, header))
		{
			return CC_FERR_WRITING;
		}
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}

	//progress dialog
	ccProgressDialog pdlg(true, parameters.parentWidget); //cancel available
	CCLib::NormalizedProgress nProgress(&pdlg, numberOfPoints);
	if (parameters.parentWidget)
	{
		pdlg.setMethodTitle(QObject::tr("Save LAS file"));
		pdlg.setInfo(QObject::tr("Points: %1").arg(numberOfPoints));
		pdlg.start();
	}

	assert(lasWriter.writer());
	//liblas::Point point(boost::shared_ptr<liblas::Header>(new liblas::Header(lasWriter.writer->GetHeader())));
	liblas::Point point(&lasWriter.writer()->GetHeader());
	liblas::Classification classif = point.GetClassification();

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	for (unsigned i = 0; i < numberOfPoints; i++)
	{
		const CCVector3* P = theCloud->getPoint(i);
		{
			CCVector3d Pglobal = theCloud->toGlobal3d<PointCoordinateType>(*P);
			point.SetCoordinates(Pglobal.x, Pglobal.y, Pglobal.z);
		}
		
		if (hasColor)
		{
			const ColorCompType* rgb = theCloud->getPointColor(i);
			//DGM: LAS colors are stored on 16 bits!
			point.SetColor(liblas::Color(	static_cast<uint32_t>(rgb[0]) << 8,
											static_cast<uint32_t>(rgb[1]) << 8,
											static_cast<uint32_t>(rgb[2]) << 8
										)
							);
		}

		//additional fields
		for (std::vector<LasField>::const_iterator it = fieldsToSave.begin(); it != fieldsToSave.end(); ++it)
		{
			assert(it->sf);
			switch(it->type)
			{
			case LAS_X:
			case LAS_Y:
			case LAS_Z:
				assert(false);
				break;
			case LAS_INTENSITY:
				point.SetIntensity(static_cast<boost::uint16_t>(it->sf->getValue(i)));
				break;
			case LAS_RETURN_NUMBER:
				point.SetReturnNumber(static_cast<boost::uint16_t>(it->sf->getValue(i)));
				break;
			case LAS_NUMBER_OF_RETURNS:
				point.SetNumberOfReturns(static_cast<boost::uint16_t>(it->sf->getValue(i)));
				break;
			case LAS_SCAN_DIRECTION:
				point.SetScanDirection(static_cast<boost::uint16_t>(it->sf->getValue(i)));
				break;
			case LAS_FLIGHT_LINE_EDGE:
				point.SetFlightLineEdge(static_cast<boost::uint16_t>(it->sf->getValue(i)));
				break;
			case LAS_CLASSIFICATION:
				{
					boost::uint32_t val = static_cast<boost::uint32_t>(it->sf->getValue(i));
					classif.SetClass(val & 31);		//first 5 bits
					classif.SetSynthetic(val & 32); //6th bit
					classif.SetKeyPoint(val & 64);	//7th bit
					classif.SetWithheld(val & 128);	//8th bit
				}
				break;
			case LAS_SCAN_ANGLE_RANK:
				point.SetScanAngleRank(static_cast<boost::uint8_t>(it->sf->getValue(i)));
				break;
			case LAS_USER_DATA:
				point.SetUserData(static_cast<boost::uint8_t>(it->sf->getValue(i)));
				break;
			case LAS_POINT_SOURCE_ID:
				point.SetPointSourceID(static_cast<boost::uint16_t>(it->sf->getValue(i)));
				break;
			case LAS_RED:
			case LAS_GREEN:
			case LAS_BLUE:
				assert(false);
				break;
			case LAS_TIME:
				point.SetTime(static_cast<double>(it->sf->getValue(i)) + it->sf->getGlobalShift());
				break;
			case LAS_CLASSIF_VALUE:
				classif.SetClass(static_cast<boost::uint32_t>(it->sf->getValue(i)));
				break;
			case LAS_CLASSIF_SYNTHETIC:
				classif.SetSynthetic(static_cast<boost::uint32_t>(it->sf->getValue(i)));
				break;
			case LAS_CLASSIF_KEYPOINT:
				classif.SetKeyPoint(static_cast<boost::uint32_t>(it->sf->getValue(i)));
				break;
			case LAS_CLASSIF_WITHHELD:
				classif.SetWithheld(static_cast<boost::uint32_t>(it->sf->getValue(i)));
				break;
			case LAS_INVALID:
			default:
				assert(false);
				break;
			}
		}

		//set classification (it's mandatory anyway ;)
		point.SetClassification(classif);

		try
		{
			lasWriter.write(point);
		}
		catch (...)
		{
			result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
			break;
		}

		if (parameters.parentWidget && !nProgress.oneStep())
		{
			break;
		}
	}

	lasWriter.close();

	return result;
}

QSharedPointer<LASOpenDlg> s_lasOpenDlg(0);

//! LAS 1.4 EVLR record
struct EVLR
{
	unsigned char reserved[2]; // 2 bytes 
	unsigned char data_type; // 1 byte 
	unsigned char options; // 1 byte 
	static const unsigned NAME_MAX_LENGTH = 32;
	char name[NAME_MAX_LENGTH]; // 32 bytes 
	unsigned char unused[4]; // 4 bytes 
	double no_data[3]; // 24 = 3*8 bytes 
	double min[3]; // 24 = 3*8 bytes 
	double max[3]; // 24 = 3*8 bytes 
	double scale[3]; // 24 = 3*8 bytes 
	double offset[3]; // 24 = 3*8 bytes 
	static const unsigned DESC_MAX_LENGTH = 32;
	char description[DESC_MAX_LENGTH]; // 32 bytes 

	//! Returns the field name
	QString getName() const
	{
		//if the name uses the full record length (32 bytes)
		//we must add a 0 at the end so as to make a valid string!
		char tempName[NAME_MAX_LENGTH+1];
		memcpy(tempName,name,NAME_MAX_LENGTH);
		tempName[NAME_MAX_LENGTH] = 0;
		return QString(tempName);
	}

	//! Returns the field description
	QString getDescription() const
	{
		//if the name uses the full record length (32 bytes)
		//we must add a 0 at the end so as to make a valid string!
		char tempDesc[DESC_MAX_LENGTH+1];
		memcpy(tempDesc, description, DESC_MAX_LENGTH);
		tempDesc[DESC_MAX_LENGTH] = 0;
		return QString(tempDesc);
	}
}; // total of 192 bytes 

//! Structure describing the current tiling process
struct TilingStruct
{
	TilingStruct()
		: w(1)
		, h(1)
		, X(0)
		, Y(1)
		, Z(2)
	{}

	~TilingStruct()
	{
		closeAll();
	}
	
	inline size_t tileCount() const { return tileFiles.size(); }

	bool init(	unsigned width,
				unsigned height,
				unsigned Zdim,
				QString absoluteBaseFilename,
				const CCVector3d& bbMin,
				const CCVector3d& bbMax,
				const liblas::Header& header)
	{
		//init tiling dimensions
		assert(Zdim < 3);
		Z = Zdim;
		X = (Z == 2 ? 0 : Z+1);
		Y = (X == 2 ? 0 : X+1);

		bbMinCorner = bbMin;
		tileDiag = bbMax - bbMin;
		tileDiag.u[X] /= width;
		tileDiag.u[Y] /= height;


		unsigned count = width * height;
		try
		{
			tileFiles.resize(count);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return false;
		}

		w = width;
		h = height;

		//File extension
		QString ext = (header.Compressed() ? "laz" : "las");
		
		for (unsigned i = 0; i < width; ++i)
		{
			for (unsigned j = 0; j < height; ++j)
			{
				unsigned ii = index(i, j);
				QString filename = absoluteBaseFilename + QString("_%1_%2.%3").arg(QString::number(i), QString::number(j), ext);

				//local version of the header (i.e. = base header with updated bounding-box)
				liblas::Header h = header;
				CCVector3d m, M;
				m.u[X] = bbMin.u[X] + i * tileDiag.u[X];
				m.u[Y] = bbMin.u[Y] + j * tileDiag.u[Y];
				m.u[Z] = bbMin.u[Z];

				M.u[X] = m.u[X] + tileDiag.u[X];
				M.u[Y] = m.u[Y] + tileDiag.u[Y];
				M.u[Z] = bbMax.u[Z];
				
				h.SetMin(m.x, m.y, m.z);
				h.SetMax(M.x, M.y, M.z);

				LASWriter* lw = new LASWriter;
				if (!lw->open(filename, h))
				{
					delete lw;
					closeAll();
					return false;
				}
				tileFiles[ii] = lw;
			}
		}

		return true;
	}

	size_t closeAll()
	{
		size_t nonEmptyCount = 0;
		for (LASWriter*& lw : tileFiles)
		{
			if (lw)
			{
				lw->close();
				if (lw->writtenCount() == 0)
				{
					//get rid of the file if it's empty!
					QFile(lw->getFilename()).remove();
				}
				else
				{
					++nonEmptyCount;
				}
				delete lw;
				lw = 0;
			}
		}
		
		tileFiles.clear();

		return nonEmptyCount;
	}

	void writePoint(const liblas::Point& P)
	{
		//determine the right tile
		CCVector3d Prel = CCVector3d(P.GetX(), P.GetY(), P.GetZ()) - bbMinCorner;
		int ii = static_cast<int>(floor(Prel.u[X] / tileDiag.u[X]));
		int ji = static_cast<int>(floor(Prel.u[Y] / tileDiag.u[Y]));
		unsigned i = std::min( static_cast<unsigned>(std::max(ii, 0)), w-1);
		unsigned j = std::min( static_cast<unsigned>(std::max(ji, 0)), h-1);

		LASWriter* lw = tileFiles[index(i,j)];
		assert(lw);
		lw->write(P);
	}

protected:

	inline unsigned index(unsigned i, unsigned j) const { return i + j * w; }
	
	unsigned w, h;
	unsigned X, Y, Z;
	CCVector3d bbMinCorner, tileDiag;
	std::vector<LASWriter*> tileFiles;
};

CC_FILE_ERROR LASFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	//opening file
	std::ifstream ifs;
	ifs.open(qPrintable(filename), std::ios::in | std::ios::binary); //DGM: warning, toStdString doesn't preserve "local" characters

	if (ifs.fail())
		return CC_FERR_READING;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	try
	{
		liblas::Reader reader(liblas::ReaderFactory().CreateWithStream(ifs));

		//handling of compressed/uncompressed files
		liblas::Header const& header = reader.GetHeader();

		ccLog::Print(QString("[LAS] %1 - signature: %2").arg(filename).arg(header.GetFileSignature().c_str()));

		//get the bounding-box (for tiling)
		CCVector3d bbMin(header.GetMinX(), header.GetMinY(), header.GetMinZ());
		CCVector3d bbMax(header.GetMaxX(), header.GetMaxY(), header.GetMaxZ());

		const liblas::Schema& schema = header.GetSchema();

		//save the 'Spatial Reference' as meta-data
		//QVariant<liblas::SpatialReference> 

		CCVector3d lasScale =  CCVector3d(header.GetScaleX(),  header.GetScaleY(),  header.GetScaleZ());
		CCVector3d lasShift = -CCVector3d(header.GetOffsetX(), header.GetOffsetY(), header.GetOffsetZ());

		//get fields present in file
		//DGM: strangely, on the 32 bits windows version, calling GetDimensionNames makes CC crash?!
		//DGM: so we call the same code as the function does... and it works?! (DIRTY)
		std::vector<std::string> dimensions;
		//dimensions = schema.GetDimensionNames();
		const liblas::Dimension* extraDimension = 0;
		std::vector<EVLR> evlrs;
		{
			liblas::IndexMap const& map = schema.GetDimensions();
			liblas::index_by_position const& position_index = map.get<liblas::position>();
			liblas::index_by_position::const_iterator it = position_index.begin();
			while (it != position_index.end())
			{
				//specific case: 'extra bytes' field
				if (it->GetName() == "extra")
				{
					//look for the corresponding EVLRs
					const std::vector<liblas::VariableRecord>& vlrs = header.GetVLRs();
					{
						for (size_t i = 0; i < vlrs.size(); ++i)
						{
							const liblas::VariableRecord& vlr = vlrs[i];
							if (vlr.GetUserId(false) == "LASF_Spec" && vlr.GetRecordId() == 4)
							{
								//EXTRA BYTES record length is 192
								static unsigned EB_RECORD_SIZE = 192;

								assert((vlr.GetData().size() % EB_RECORD_SIZE) == 0);
								size_t count = vlr.GetData().size() / EB_RECORD_SIZE;
								const uint8_t* vlrData = &(vlr.GetData()[0]);
								for (size_t j = 0; j < count; ++j)
								{
									const EVLR* evlr = reinterpret_cast<const EVLR*>(vlrData + j*EB_RECORD_SIZE);
									evlrs.push_back(*evlr);
									ccLog::PrintDebug(QString("[LAS] Extra bytes VLR found: %1 (%2)").arg(evlr->getName()).arg(evlr->getDescription()));
								}
							}
						}
					}

					if (!evlrs.empty())
					{
						//we'll handle this one separately!
						extraDimension = &(*it);
					}
				}
				else
				{
					dimensions.push_back(it->GetName());
				}
				ccLog::PrintDebug(QString("\tDimension: %1 (size: %2 - type: %3)").arg(QString::fromStdString(it->GetName())).arg(it->GetByteSize()).arg(it->IsNumeric() ? (it->IsInteger() ? "integer" : "Float") : "Non numeric"));
				++it;
			}
		}

		//and of course the number of points
		unsigned nbOfPoints = header.GetPointRecordsCount();
		if (nbOfPoints == 0)
		{
			//strange file ;)
			ifs.close();
			return CC_FERR_NO_LOAD;
		}

		//dialog to choose the fields to load
		if (!s_lasOpenDlg)
		{
			s_lasOpenDlg = QSharedPointer<LASOpenDlg>(new LASOpenDlg());
		}
		s_lasOpenDlg->setDimensions(dimensions);
		s_lasOpenDlg->clearEVLRs();
		s_lasOpenDlg->setInfos(filename, nbOfPoints, bbMin, bbMax);
		if (extraDimension)
		{
			assert(!evlrs.empty());
			for (const EVLR& evlr : evlrs)
			{
				s_lasOpenDlg->addEVLR(QString("%1 (%2)").arg(evlr.getName()).arg(evlr.getDescription()));
			}
		}

		if (parameters.alwaysDisplayLoadDialog && !s_lasOpenDlg->autoSkipMode() && !s_lasOpenDlg->exec())
		{
			ifs.close();
			return CC_FERR_CANCELED_BY_USER;
		}
		bool ignoreDefaultFields = s_lasOpenDlg->ignoreDefaultFieldsCheckBox->isChecked();

		TilingStruct tiler;
		bool tiling = s_lasOpenDlg->tileGroupBox->isChecked();
		if (tiling)
		{
			//tiling (vertilca) dimension
			unsigned vertDim = 2;
			switch (s_lasOpenDlg->tileDimComboBox->currentIndex())
			{
			case 0: //XY
				vertDim = 2;
				break;
			case 1: //XZ
				vertDim = 1;
				break;
			case 2: //YZ
				vertDim = 0;
				break;
			default:
				assert(false);
				break;
			}

			unsigned w = static_cast<unsigned>(s_lasOpenDlg->wTileSpinBox->value());
			unsigned h = static_cast<unsigned>(s_lasOpenDlg->hTileSpinBox->value());

			QString outputBaseName = s_lasOpenDlg->outputPathLineEdit->text() + "/" + QFileInfo(filename).baseName();

			if (!tiler.init(w, h, vertDim, outputBaseName, bbMin, bbMax, header))
			{
				//failed to open at least one file!
				return CC_FERR_WRITING;
			}
		}

		//RGB color
		liblas::Color rgbColorMask; //(0,0,0) on construction
		if (s_lasOpenDlg->doLoad(LAS_RED))
			rgbColorMask.SetRed(~0);
		if (s_lasOpenDlg->doLoad(LAS_GREEN))
			rgbColorMask.SetGreen(~0);
		if (s_lasOpenDlg->doLoad(LAS_BLUE))
			rgbColorMask.SetBlue(~0);
		bool loadColor = (rgbColorMask[0] || rgbColorMask[1] || rgbColorMask[2]);

		//progress dialog
		ccProgressDialog pdlg(true, parameters.parentWidget); //cancel available
		CCLib::NormalizedProgress nProgress(&pdlg, nbOfPoints);
		if (parameters.parentWidget)
		{
			pdlg.setMethodTitle(QObject::tr("Open LAS file"));
			pdlg.setInfo(QObject::tr("Points: %1").arg(nbOfPoints));
			pdlg.start();
		}

		//number of points read from the beginning of the current cloud part
		unsigned pointsRead = 0;
		CCVector3d Pshift(0, 0, 0);

		//by default we read colors as triplets of 8 bits integers but we might dynamically change this
		//if we encounter values using 16 bits (16 bits is the standard!)
		unsigned char colorCompBitShift = 0;
		bool forced8bitRgbMode = s_lasOpenDlg->forced8bitRgbMode();
		ColorCompType rgb[3] = { 0, 0, 0 };

		ccPointCloud* loadedCloud = 0;
		std::vector< LasField::Shared > fieldsToLoad;

		//if the file is too big, we will chunck it in multiple parts
		unsigned int fileChunkPos = 0;
		unsigned int fileChunkSize = 0;

		while (true)
		{
			//if we reach the end of the file, or the max. cloud size limit (in which case we cerate a new chunk)
			bool newPointAvailable = false;
			try
			{
				newPointAvailable = ((!parameters.parentWidget || nProgress.oneStep()) && reader.ReadNextPoint());
			}
			catch (...)
			{
				result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
				break;
			}

			//special operation: tiling mode
			if (tiling)
			{
				if (newPointAvailable)
				{
					const liblas::Point& p = reader.GetPoint();
					tiler.writePoint(p);
				}
				else
				{
					break; //end of the file (or cancel requested)
				}

				continue;
			}

			if (!newPointAvailable || pointsRead == fileChunkPos + fileChunkSize)
			{
				if (loadedCloud)
				{
					assert(!tiling);
					if (loadedCloud->size())
					{
						bool thisChunkHasColors = loadedCloud->hasColors();
						loadedCloud->showColors(thisChunkHasColors);
						if (loadColor && !thisChunkHasColors)
						{
							ccLog::Warning("[LAS] Color field was all black! We ignored it...");
						}

						while (!fieldsToLoad.empty())
						{
							LasField::Shared& field = fieldsToLoad.back();
							if (field && field->sf)
							{
								field->sf->computeMinAndMax();

								if (	field->type == LAS_CLASSIFICATION
									||	field->type == LAS_CLASSIF_VALUE
									||	field->type == LAS_CLASSIF_SYNTHETIC
									||	field->type == LAS_CLASSIF_KEYPOINT
									||	field->type == LAS_CLASSIF_WITHHELD
									||	field->type == LAS_RETURN_NUMBER
									||	field->type == LAS_NUMBER_OF_RETURNS)
								{
									int cMin = static_cast<int>(field->sf->getMin());
									int cMax = static_cast<int>(field->sf->getMax());
									field->sf->setColorRampSteps(std::min<int>(cMax - cMin + 1, 256));
									//classifSF->setMinSaturation(cMin);
								}
								else if (field->type == LAS_INTENSITY)
								{
									field->sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
								}

								int sfIndex = loadedCloud->addScalarField(field->sf);
								if (!loadedCloud->hasDisplayedScalarField())
								{
									loadedCloud->setCurrentDisplayedScalarField(sfIndex);
									loadedCloud->showSF(!thisChunkHasColors);
								}
								field->sf->release();
								field->sf = 0;
							}
							else
							{
								ccLog::Warning(QString("[LAS] All '%1' values were the same (%2)! We ignored them...").arg(field->type == LAS_EXTRA ? field->getName() : QString(LAS_FIELD_NAMES[field->type])).arg(field->firstValue));
							}

							fieldsToLoad.pop_back();
						}

						//if we have reserved too much memory
						if (loadedCloud->size() < loadedCloud->capacity())
						{
							loadedCloud->resize(loadedCloud->size());
						}

						QString chunkName("unnamed - Cloud");
						unsigned n = container.getChildrenNumber();
						if (n != 0) //if we have more than one cloud, we append an index
						{
							if (n == 1)  //we must also update the first one!
							{
								container.getChild(0)->setName(chunkName + QString(" #1"));
							}
							chunkName += QString(" #%1").arg(n + 1);
						}
						loadedCloud->setName(chunkName);

						loadedCloud->setMetaData(LAS_SCALE_X_META_DATA, QVariant(lasScale.x));
						loadedCloud->setMetaData(LAS_SCALE_Y_META_DATA, QVariant(lasScale.y));
						loadedCloud->setMetaData(LAS_SCALE_Z_META_DATA, QVariant(lasScale.z));

						container.addChild(loadedCloud);
						loadedCloud = 0;
					}
					else
					{
						//empty cloud?!
						delete loadedCloud;
						loadedCloud = 0;
					}
				}

				if (!newPointAvailable)
				{
					break; //end of the file (or cancel requested)
				}

				//otherwise, we must create a new cloud
				fileChunkPos = pointsRead;
				fileChunkSize = std::min(nbOfPoints - pointsRead, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
				loadedCloud = new ccPointCloud();
				if (!loadedCloud->reserveThePointsTable(fileChunkSize))
				{
					ccLog::Warning("[LAS] Not enough memory!");
					delete loadedCloud;
					ifs.close();
					return CC_FERR_NOT_ENOUGH_MEMORY;
				}
				loadedCloud->setGlobalShift(Pshift);

				//save the Spatial reference as meta-data
				loadedCloud->setMetaData(s_LAS_SRS_Key, QVariant::fromValue(header.GetSRS()));

				//DGM: from now on, we only enable scalar fields when we detect a valid value!
				if (s_lasOpenDlg->doLoad(LAS_CLASSIFICATION))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIFICATION, 0, 0, 255))); //unsigned char: between 0 and 255
				if (s_lasOpenDlg->doLoad(LAS_CLASSIF_VALUE))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_VALUE, 0, 0, 31))); //5 bits: between 0 and 31
				if (s_lasOpenDlg->doLoad(LAS_CLASSIF_SYNTHETIC))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_SYNTHETIC, 0, 0, 1))); //1 bit: 0 or 1
				if (s_lasOpenDlg->doLoad(LAS_CLASSIF_KEYPOINT))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_KEYPOINT, 0, 0, 1))); //1 bit: 0 or 1
				if (s_lasOpenDlg->doLoad(LAS_CLASSIF_WITHHELD))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_CLASSIF_WITHHELD, 0, 0, 1))); //1 bit: 0 or 1
				if (s_lasOpenDlg->doLoad(LAS_INTENSITY))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_INTENSITY, 0, 0, 65535))); //16 bits: between 0 and 65536
				if (s_lasOpenDlg->doLoad(LAS_TIME))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_TIME, 0, 0, -1.0))); //8 bytes (double) --> we use global shift!
				if (s_lasOpenDlg->doLoad(LAS_RETURN_NUMBER))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_RETURN_NUMBER, 1, 1, 7))); //3 bits: between 1 and 7
				if (s_lasOpenDlg->doLoad(LAS_NUMBER_OF_RETURNS))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_NUMBER_OF_RETURNS, 1, 1, 7))); //3 bits: between 1 and 7
				if (s_lasOpenDlg->doLoad(LAS_SCAN_DIRECTION))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_SCAN_DIRECTION, 0, 0, 1))); //1 bit: 0 or 1
				if (s_lasOpenDlg->doLoad(LAS_FLIGHT_LINE_EDGE))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_FLIGHT_LINE_EDGE, 0, 0, 1))); //1 bit: 0 or 1
				if (s_lasOpenDlg->doLoad(LAS_SCAN_ANGLE_RANK))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_SCAN_ANGLE_RANK, 0, -90, 90))); //signed char: between -90 and +90
				if (s_lasOpenDlg->doLoad(LAS_USER_DATA))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_USER_DATA, 0, 0, 255))); //unsigned char: between 0 and 255
				if (s_lasOpenDlg->doLoad(LAS_POINT_SOURCE_ID))
					fieldsToLoad.push_back(LasField::Shared(new LasField(LAS_POINT_SOURCE_ID, 0, 0, 65535))); //16 bits: between 0 and 65536

				//Extra fields
				if (s_lasOpenDlg->doLoad(LAS_EXTRA))
				{
					if (extraDimension)
					{
						assert(!evlrs.empty());
						const size_t extraBytesOffset = extraDimension->GetByteOffset();
						size_t localOffset = 0;
						for (size_t i=0; i<evlrs.size(); ++i)
						{
							unsigned char data_type = evlrs[i].data_type;
							//We split the fields with mutliple values in multiple scalar fields!
							unsigned subFieldCount = 1;
							if (evlrs[i].data_type > 20)
							{
								subFieldCount = 3;
								data_type -= 20;
							}
							else if (evlrs[i].data_type > 10)
							{
								subFieldCount = 2;
								data_type -= 10;
							}

							for (unsigned j = 0; j < subFieldCount; ++j)
							{
								size_t dataOffset = extraBytesOffset + localOffset;

								//move forward (and check that the byte count is ok!)
								assert(data_type <= ExtraLasField::EXTRA_DOUBLE);
								ExtraLasField::Type type = static_cast<ExtraLasField::Type>(data_type);
								localOffset += ExtraLasField::GetSizeBytes(type);
							
								if (localOffset <= extraDimension->GetByteSize())
								{
									if (s_lasOpenDlg->doLoadEVLR(i))
									{
										QString fieldName(evlrs[i].getName());
										if (subFieldCount > 1)
											fieldName += QString(".%1").arg(j+1);

										const unsigned char options = evlrs[i].options;

										//read the first optional informations
										double defaultVal = 0;
										double minVal = 0;
										double maxVal = -1.0;
										//DGM: the first 3 (no_data, min and max) are a bit
										//dangerous to use because we don't know if they have
										//been saved as double values (as Laspy do!) or if
										//the same type as ExtraLasField::Type is used!
										if (false)
										{
											if (options & 1) //1st bit = no_data_bit
												defaultVal = evlrs[i].no_data[j];
											if (options & 2) //2nd bit = min_bit
												minVal = evlrs[i].min[j];
											if (options & 3) //3rd bit = max_bit
												maxVal = evlrs[i].max[j];
										}

										ExtraLasField* eField = new ExtraLasField(fieldName,type,static_cast<int>(dataOffset),defaultVal,minVal,maxVal);

										//read the other optional information (scale and offset)
										{
											if (options & 4) //4th bit = scale_bit
												eField->scale = evlrs[i].scale[j];
											if (options & 5) //5th bit = offset_bit
												eField->offset = evlrs[i].offset[j];
										}
										fieldsToLoad.push_back(LasField::Shared(eField));
									}
								}
								else
								{
									ccLog::Warning("[LAS] Internal consistency of extra fields is broken! (more values defined that available types...)");
									break;
								}
							}
						}
					}
					else
					{
						//shouldn't happen:
						assert(false);
					}
				}
			}

			assert(newPointAvailable);
			const liblas::Point& p = reader.GetPoint();

			//first point: check for 'big' coordinates
			if (pointsRead == 0)
			{
				CCVector3d P(p.GetX(), p.GetY(), p.GetZ());
				//backup input global parameters
				ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
				bool useLasShift = false;
				//set the LAS shift as default shift (if none was provided)
				if (lasShift.norm2() != 0 && (!parameters.coordinatesShiftEnabled || !*parameters.coordinatesShiftEnabled))
				{
					useLasShift = true;
					Pshift = lasShift;
					if (	csModeBackup != ccGlobalShiftManager::NO_DIALOG
						&&	csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
					{
						parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
					}
				}
				if (HandleGlobalShift(P, Pshift, parameters, useLasShift))
				{
					loadedCloud->setGlobalShift(Pshift);
					ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}

				//restore previous parameters
				parameters.shiftHandlingMode = csModeBackup;
			}

			CCVector3 P(static_cast<PointCoordinateType>(p.GetX() + Pshift.x),
						static_cast<PointCoordinateType>(p.GetY() + Pshift.y),
						static_cast<PointCoordinateType>(p.GetZ() + Pshift.z));
			loadedCloud->addPoint(P);

			//color field
			if (loadColor)
			{
				//Warning: LAS colors are stored on 16 bits!
				liblas::Color col = p.GetColor();
				col[0] &= rgbColorMask[0];
				col[1] &= rgbColorMask[1];
				col[2] &= rgbColorMask[2];

				//if we don't have reserved a color field yet, we check first that color is not black
				bool pushColor = true;
				if (!loadedCloud->hasColors())
				{
					//if the color is not black, we are sure it's a valid color field!
					if (col[0] || col[1] || col[2])
					{
						if (loadedCloud->reserveTheRGBTable())
						{
							//we must set the color (black) of all the previously skipped points
							for (unsigned i = 0; i < loadedCloud->size() - 1; ++i)
							{
								loadedCloud->addRGBColor(ccColor::black.rgba);
							}
						}
						else
						{
							ccLog::Warning("[LAS] Not enough memory: color field will be ignored!");
							loadColor = false; //no need to retry with the other chunks anyway
							pushColor = false;
						}
					}
					else //otherwise we ignore it for the moment (we'll add it later if necessary)
					{
						pushColor = false;
					}
				}

				//do we need to push this color?
				if (pushColor)
				{
					//we test if the color components are on 16 bits (standard) or only on 8 bits (it happens ;)
					if (!forced8bitRgbMode && colorCompBitShift == 0)
					{
						if (	(col[0] & 0xFF00)
							||	(col[1] & 0xFF00)
							||	(col[2] & 0xFF00))
						{
							//the color components are on 16 bits!
							ccLog::Print("[LAS] Color components are coded on 16 bits");
							colorCompBitShift = 8;
							//we fix all the previously read colors
							for (unsigned i = 0; i < loadedCloud->size() - 1; ++i)
							{
								loadedCloud->setPointColor(i, ccColor::black.rgba); //255 >> 8 = 0!
							}
						}
					}

					rgb[0] = static_cast<ColorCompType>(col[0] >> colorCompBitShift);
					rgb[1] = static_cast<ColorCompType>(col[1] >> colorCompBitShift);
					rgb[2] = static_cast<ColorCompType>(col[2] >> colorCompBitShift);

					loadedCloud->addRGBColor(rgb);
				}
			}
		
			//additional fields
			for (std::vector<LasField::Shared>::iterator it = fieldsToLoad.begin(); it != fieldsToLoad.end(); ++it)
			{
				LasField::Shared field = *it;
			
				double value = 0.0;
				switch (field->type)
				{
				case LAS_INTENSITY:
					value = static_cast<double>(p.GetIntensity());
					break;
				case LAS_RETURN_NUMBER:
					value = static_cast<double>(p.GetReturnNumber());
					break;
				case LAS_NUMBER_OF_RETURNS:
					value = static_cast<double>(p.GetNumberOfReturns());
					break;
				case LAS_SCAN_DIRECTION:
					value = static_cast<double>(p.GetScanDirection());
					break;
				case LAS_FLIGHT_LINE_EDGE:
					value = static_cast<double>(p.GetFlightLineEdge());
					break;
				case LAS_CLASSIFICATION:
					value = static_cast<double>(p.GetClassification().GetClass());
					break;
				case LAS_SCAN_ANGLE_RANK:
					value = static_cast<double>(p.GetScanAngleRank());
					break;
				case LAS_USER_DATA:
					value = static_cast<double>(p.GetUserData());
					break;
				case LAS_POINT_SOURCE_ID:
					value = static_cast<double>(p.GetPointSourceID());
					break;
				case LAS_EXTRA:
					{
						//we must dynamically extract the value in the right format
						ExtraLasField* extraField = static_cast<ExtraLasField*>((*it).data());
						assert(extraDimension && extraField->dataOffset < static_cast<int>(p.GetData().size()));
						const uint8_t* v = &(p.GetData()[extraField->dataOffset]);

						switch(extraField->valType)
						{
						case ExtraLasField::EXTRA_UINT8:
							value = static_cast<double>(*(reinterpret_cast<const uint8_t*>(v)));
							break;
						case ExtraLasField::EXTRA_INT8:
							value = static_cast<double>(*(reinterpret_cast<const int8_t*>(v)));
							break;
						case ExtraLasField::EXTRA_UINT16:
							value = static_cast<double>(*(reinterpret_cast<const uint16_t*>(v)));
							break;
						case ExtraLasField::EXTRA_INT16:
							value = static_cast<double>(*(reinterpret_cast<const int16_t*>(v)));
							break;
						case ExtraLasField::EXTRA_UINT32:
							value = static_cast<double>(*(reinterpret_cast<const uint32_t*>(v)));
							break;
						case ExtraLasField::EXTRA_INT32:
							value = static_cast<double>(*(reinterpret_cast<const int32_t*>(v)));
							break;
						case ExtraLasField::EXTRA_UINT64:
							value = static_cast<double>(*(reinterpret_cast<const uint64_t*>(v)));
							break;
						case ExtraLasField::EXTRA_INT64:
							value = static_cast<double>(*(reinterpret_cast<const int64_t*>(v)));
							break;
						case ExtraLasField::EXTRA_FLOAT:
							value = static_cast<double>(*(reinterpret_cast<const float*>(v)));
							break;
						case ExtraLasField::EXTRA_DOUBLE:
							value = static_cast<double>(*(reinterpret_cast<const double*>(v)));
							break;
						default:
							assert(false);
							break;
						}

						value = extraField->offset + extraField->scale * value;
					}
					break;
				case LAS_TIME:
					value = p.GetTime();
					if (field->sf)
					{
						//shift time values (so as to avoid losing accuracy)
						value -= field->sf->getGlobalShift();
					}
					break;
				case LAS_CLASSIF_VALUE:
					value = static_cast<double>(p.GetClassification().GetClass() & 31); //5 bits
					break;
				case LAS_CLASSIF_SYNTHETIC:
					value = static_cast<double>(p.GetClassification().GetClass() & 32); //bit #6
					break;
				case LAS_CLASSIF_KEYPOINT:
					value = static_cast<double>(p.GetClassification().GetClass() & 64); //bit #7
					break;
				case LAS_CLASSIF_WITHHELD:
					value = static_cast<double>(p.GetClassification().GetClass() & 128); //bit #8
					break;
				default:
					//ignored
					assert(false);
					continue;
				}

				if (field->sf)
				{
					ScalarType s = static_cast<ScalarType>(value);
					field->sf->addElement(s);
				}
				else
				{
					//first point? we track its value
					if (loadedCloud->size() == 1)
					{
						field->firstValue = value;
					}
				
					if (	!ignoreDefaultFields
						||	value != field->firstValue
						||	(field->firstValue != field->defaultValue && field->firstValue >= field->minValue))
					{
						field->sf = new ccScalarField(qPrintable(field->getName()));
						if (field->sf->reserve(fileChunkSize))
						{
							field->sf->link();

							if (field->type == LAS_TIME)
							{
								//we use the first value as 'global shift' (otherwise we will lose accuracy)
								field->sf->setGlobalShift(field->firstValue);
								value -= field->firstValue;
								ccLog::Warning("[LAS] Time SF has been shifted to prevent a loss of accuracy (%.2f)",field->firstValue);
								field->firstValue = 0;
							}

							//we must set the value of all the previously skipped points
							ScalarType firstValue = static_cast<ScalarType>(field->firstValue);
							for (unsigned i = 0; i < loadedCloud->size() - 1; ++i)
							{
								field->sf->addElement(firstValue);
							}

							ScalarType s = static_cast<ScalarType>(value);
							field->sf->addElement(s);
						}
						else
						{
							ccLog::Warning(QString("[LAS] Not enough memory: '%1' field will be ignored!").arg(LAS_FIELD_NAMES[field->type]));
							field->sf->release();
							field->sf = 0;
						}
					}
				}
			}

			++pointsRead;
		}

		if (tiling)
		{
			size_t tileCount = tiler.tileCount();
			size_t nonEmptyCount = tiler.closeAll();
			ccLog::Print(QString("[LAS I/O filter] %1 tile file(s) written").arg(nonEmptyCount));
			if (nonEmptyCount < tileCount)
			{
				ccLog::Warning(QString("[LAS I/O filter] %1 skipped empty tile(s)").arg(tileCount - nonEmptyCount));
			}
		}
	}
	catch (const std::exception& e)
	{
		ccLog::Error(QString("Liblas exception: '%1'").arg(e.what()));
		result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch (...)
	{
		result = CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	ifs.close();

	return result;
}

#endif
