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

#include "LASFilter.h"

//Local
#include "LASOpenDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccHObjectCaster.h>
#include "ccColorScalesManager.h"

//CCLib
#include <CCPlatform.h>

//Qt
#include <QFileInfo>
#include <QSharedPointer>
#include <QInputDialog>
#include <QFuture>
#include <QtConcurrent>

//pdal
#include <memory>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/LasVLR.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

Q_DECLARE_METATYPE(pdal::SpatialReference)

using namespace pdal::Dimension;
using namespace pdal;

//Qt gui
#include <ui_saveLASFileDlg.h>

//System
#include <string.h>
#include <bitset>

static const char s_LAS_SRS_Key[] = "LAS.spatialReference.nosave"; //DGM: added the '.nosave' suffix because this custom type can't be streamed properly

//! Custom ("Extra bytes") field (EVLR)
struct ExtraLasField : LasField
{
	//! Default constructor
	ExtraLasField(QString name, Id id, double defaultVal = 0.0, double min = 0.0, double max = -1.0)
		: LasField(LAS_EXTRA, defaultVal, min, max)
		, fieldName(SanitizeString(name))
		, pdalId(id)
		, scale(1.0)
		, offset(0.0)
	{
		if (fieldName != name)
		{
			ccLog::Warning(QString("Extra field '%1' renamed '%2' to comply to LAS specifications").arg(name).arg(fieldName));
		}
	}

	typedef QSharedPointer<ExtraLasField> Shared;

	inline QString getName() const override { return fieldName; }

	QString fieldName;
	Id pdalId;
	double scale;
	double offset;
};

//! LAS Save dialog
class LASSaveDlg : public QDialog, public Ui::SaveLASFileDialog
{
public:
	explicit LASSaveDlg(QWidget* parent = nullptr)
		: QDialog(parent)
		, Ui::SaveLASFileDialog()
	{
		setupUi(this);
		clearEVLRs();
	}

	void clearEVLRs()
	{
		evlrListWidget->clear();
		extraFieldGroupBox->setEnabled(false);
		extraFieldGroupBox->setChecked(false);
	}

	void addEVLR(const QString &description)
	{
		QListWidgetItem* item = new QListWidgetItem(description);
		evlrListWidget->addItem(item);
		//auto select the entry
		item->setSelected(true);
		//auto enable the extraFieldGroupBox
		extraFieldGroupBox->setEnabled(true);
		extraFieldGroupBox->setChecked(false);
	}

	bool doSaveEVLR(size_t index) const
	{
		if (!extraFieldGroupBox->isChecked())
			return false;

		QListWidgetItem* item = evlrListWidget->item(static_cast<int>(index));
		return item && item->isSelected();
	}
};


LASFilter::LASFilter()
    : FileIOFilter( {
                    "_PDAL LAS Filter",
                    3.0f,	// priority
                    QStringList{ "las", "laz" },
                    "las",
                    QStringList{ "LAS cloud (*.las *.laz)" },
                    QStringList{ "LAS cloud (*.las *.laz)" },
                    Import | Export
                    } )
{
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

//! Semi persistent save dialog
QSharedPointer<LASSaveDlg> s_saveDlg(nullptr);
pdal::Dimension::Id typeToId(LAS_FIELDS sfType, uint8_t pointFormat)
{
	switch (sfType)
	{
	case LAS_FIELDS::LAS_X:
		return pdal::Dimension::Id::X;
	case LAS_FIELDS::LAS_Y:
		return pdal::Dimension::Id::Y;
	case LAS_FIELDS::LAS_Z:
		return pdal::Dimension::Id::Z;
	case LAS_FIELDS::LAS_INTENSITY:
		return pdal::Dimension::Id::Intensity;
	case LAS_FIELDS::LAS_RETURN_NUMBER:
		return pdal::Dimension::Id::ReturnNumber;
	case LAS_FIELDS::LAS_NUMBER_OF_RETURNS:
		return pdal::Dimension::Id::NumberOfReturns;
	case LAS_FIELDS::LAS_SCAN_DIRECTION:
		return pdal::Dimension::Id::ScanDirectionFlag;
	case LAS_FIELDS::LAS_FLIGHT_LINE_EDGE:
		return pdal::Dimension::Id::EdgeOfFlightLine;
	case LAS_FIELDS::LAS_CLASSIFICATION:
		return  pdal::Dimension::Id::Classification;
	case LAS_FIELDS::LAS_SCAN_ANGLE_RANK:
		return pdal::Dimension::Id::ScanAngleRank;
	case LAS_FIELDS::LAS_USER_DATA:
		return pdal::Dimension::Id::UserData;
	case LAS_FIELDS::LAS_POINT_SOURCE_ID:
		return pdal::Dimension::Id::PointSourceId;
	case LAS_FIELDS::LAS_RED:
		return pdal::Dimension::Id::Red;
	case LAS_FIELDS::LAS_GREEN:
		return pdal::Dimension::Id::Green;
	case LAS_FIELDS::LAS_BLUE:
		return pdal::Dimension::Id::Blue;
	case LAS_FIELDS::LAS_TIME:
		return pdal::Dimension::Id::GpsTime;
	case LAS_FIELDS::LAS_EXTRA:
		return pdal::Dimension::Id::Unknown;
		//Sub fields
	case LAS_FIELDS::LAS_CLASSIF_VALUE:
		return pdal::Dimension::Id::Classification;
	case LAS_FIELDS::LAS_CLASSIF_SYNTHETIC:
	case LAS_FIELDS::LAS_CLASSIF_KEYPOINT:
	case LAS_FIELDS::LAS_CLASSIF_WITHHELD:
			return pointFormat < 6 ? pdal::Dimension::Id::Classification : pdal::Dimension::Id::ClassFlags;
	case LAS_FIELDS::LAS_CLASSIF_OVERLAP:
		return pointFormat < 6 ? pdal::Dimension::Id::Unknown : pdal::Dimension::Id::ClassFlags; //only for point formats >= 6
		//Invalid flag
	case LAS_FIELDS::LAS_INVALID:
	default:
		return pdal::Dimension::Id::Unknown;
	};
}

CC_FILE_ERROR LASFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(entity);
	if (!theCloud)
	{
		ccLog::Warning("[LAS] This filter can only save one cloud at a time");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	unsigned int numberOfPoints = theCloud->size();
	if (numberOfPoints == 0)
	{
		ccLog::Warning("[LAS] Cloud is empty!");
		//return CC_FERR_NO_SAVE;
	}

	//colors
	bool hasColors = theCloud->hasColors();

	//standard las fields (as scalar fields)
	std::vector<LasField> fieldsToSave;
	//extra las fields (as scalar fields)
	std::vector<ExtraLasField::Shared> extraFields;

	uint8_t minPointFormat = 0;

	if (theCloud->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(theCloud);

		LasField::GetLASFields(pc, fieldsToSave, minPointFormat);

		for (unsigned i = 0; i < pc->getNumberOfScalarFields(); ++i)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(pc->getScalarField(i));
			//find an equivalent in official LAS fields
			QString sfName = QString(sf->getName()).toUpper();

			auto name_matches = [&sfName](const LasField &field) { return sfName == field.getName().toUpper(); };
			auto pos = std::find_if(fieldsToSave.begin(), fieldsToSave.end(), name_matches);
			if (pos == fieldsToSave.end())
			{
				ExtraLasField::Shared extraField(new ExtraLasField(QString(sf->getName()), Id::Unknown));
				extraFields.emplace_back(extraField);
				extraFields.back()->sf = sf;
			}
		}
	}

	bool hasClassification = false;
	bool hasClassifFlags = false;
	for (const LasField& field : fieldsToSave)
	{
		switch (field.type)
		{
		case LAS_CLASSIFICATION:
		case LAS_CLASSIF_VALUE:
			hasClassification = true;
			break;
		
		case LAS_CLASSIF_SYNTHETIC:
		case LAS_CLASSIF_KEYPOINT:
		case LAS_CLASSIF_WITHHELD:
			if (minPointFormat >= 6)
				hasClassifFlags = true;
			else
				hasClassification = true;
			break;
		
		case LAS_CLASSIF_OVERLAP:
			if (minPointFormat >= 6)
				hasClassifFlags = true;
			else
				assert(false);
			break;
		}
	}

	//progress dialog
	QScopedPointer<ccProgressDialog> pDlg(nullptr);
	if (parameters.parentWidget)
	{
		pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
		pDlg->setMethodTitle(QObject::tr("Save LAS file"));
		pDlg->setInfo(QObject::tr("Points: %L1").arg(numberOfPoints));
		pDlg->start();
	}
	CCLib::NormalizedProgress nProgress(pDlg.data(), numberOfPoints);

	CCVector3d bbMin, bbMax;
	if (!theCloud->getGlobalBB(bbMin, bbMax))
	{
		if (theCloud->size() != 0)
		{
			//it can only be acceptable if the cloud is empty
			//(yes, some people expect to save empty clouds!)
			return CC_FERR_NO_SAVE;
		}
		else
		{
			bbMax = bbMin = CCVector3d(0.0, 0.0, 0.0);
		}
	}

	//let the user choose between the original scale and the 'optimal' one (for accuracy, not for compression ;)
	bool hasScaleMetaData = false;
	CCVector3d originalLasScale(0, 0, 0);
	originalLasScale.x = theCloud->getMetaData(LAS_SCALE_X_META_DATA).toDouble(&hasScaleMetaData);
	if (hasScaleMetaData)
	{
		originalLasScale.y = theCloud->getMetaData(LAS_SCALE_Y_META_DATA).toDouble(&hasScaleMetaData);
		if (hasScaleMetaData)
		{
			originalLasScale.z = theCloud->getMetaData(LAS_SCALE_Z_META_DATA).toDouble(&hasScaleMetaData);
		}
	}

	bool hasOffsetMetaData = false;
	CCVector3d lasOffset(0, 0, 0);
	lasOffset.x = theCloud->getMetaData(LAS_OFFSET_X_META_DATA).toDouble(&hasOffsetMetaData);
	if (hasOffsetMetaData)
	{
		lasOffset.y = theCloud->getMetaData(LAS_OFFSET_Y_META_DATA).toDouble(&hasOffsetMetaData);
		if (hasOffsetMetaData)
		{
			lasOffset.z = theCloud->getMetaData(LAS_OFFSET_Z_META_DATA).toDouble(&hasOffsetMetaData);
		}
	}

	//Set offset & scale, as points will be stored as boost::int32_t values (between -2147483648 and 2147483647)
	//int_value = (double_value-offset)/scale
	if (hasOffsetMetaData & ccGlobalShiftManager::NeedShift(bbMax - lasOffset))
	{
		//the previous offset can't be used
		hasOffsetMetaData = false;
		lasOffset = CCVector3d(0, 0, 0);
	}
	if (!hasOffsetMetaData && ccGlobalShiftManager::NeedShift(bbMax))
	{
		//we have no choice, we'll use the min bounding box
		lasOffset = bbMin;
	}

	//optimal scale (for accuracy) --> 1e-8 because the maximum integer is roughly +/-2e+9
	CCVector3d diag = bbMax - lasOffset;
	CCVector3d optimalScale(1.0e-9 * std::max<double>(diag.x, ZERO_TOLERANCE),
	                        1.0e-9 * std::max<double>(diag.y, ZERO_TOLERANCE),
	                        1.0e-9 * std::max<double>(diag.z, ZERO_TOLERANCE));

	bool canUseOriginalScale = false;
	if (hasScaleMetaData)
	{
		//we may not be able to use the previous LAS scale
		canUseOriginalScale = (		originalLasScale.x >= optimalScale.x
								&&	originalLasScale.y >= optimalScale.y
								&&	originalLasScale.z >= optimalScale.z );
	}

	//uniformize the value to make it less disturbing to some lastools users ;)
	{
		double maxScale = std::max(optimalScale.x, std::max(optimalScale.y, optimalScale.z));
		double n = ceil(log10(maxScale)); //ceil because n should be negative
		maxScale = pow(10.0, n);
		optimalScale.x = optimalScale.y = optimalScale.z = maxScale;
	}

	CCVector3d lasScale = (canUseOriginalScale ? originalLasScale : optimalScale);

	if (parameters.alwaysDisplaySaveDialog)
	{
		if (!s_saveDlg)
			s_saveDlg = QSharedPointer<LASSaveDlg>(new LASSaveDlg(nullptr));
		
		s_saveDlg->bestAccuracyLabel->setText(QString("(%1, %2, %3)").arg(optimalScale.x).arg(optimalScale.y).arg(optimalScale.z));

		if (hasScaleMetaData)
		{
			s_saveDlg->origAccuracyLabel->setText(QString("(%1, %2, %3)").arg(originalLasScale.x).arg(originalLasScale.y).arg(originalLasScale.z));

			if (!canUseOriginalScale)
			{
				s_saveDlg->labelOriginal->setText(QObject::tr("Original scale is too small for this cloud  ")); //add two whitespaces to avoid issues with italic characters justification
				s_saveDlg->labelOriginal->setStyleSheet("color: red;");
			}
		}
		else
		{
			s_saveDlg->origAccuracyLabel->setText("none");
		}

		if (!hasScaleMetaData || !canUseOriginalScale)
		{
			if (s_saveDlg->origRadioButton->isChecked())
				s_saveDlg->bestRadioButton->setChecked(true);
			s_saveDlg->origRadioButton->setEnabled(false);
		}

		s_saveDlg->clearEVLRs();

		for (const ExtraLasField::Shared &extraField : extraFields)
		{
			s_saveDlg->addEVLR(QString("%1").arg(extraField->getName()));
		}

		s_saveDlg->exec();

		if (s_saveDlg->bestRadioButton->isChecked())
		{
			lasScale = optimalScale;
		}
		else if (s_saveDlg->origRadioButton->isChecked())
		{
			lasScale = originalLasScale;
		}
		else if (s_saveDlg->customRadioButton->isChecked())
		{
			double s = s_saveDlg->customScaleDoubleSpinBox->value();
			lasScale = CCVector3d(s, s, s);
		}
	}

	std::vector<ExtraLasField::Shared> extraFieldsToSave;
	try
	{
		for (unsigned int i = 0; i < extraFields.size(); ++i)
		{
			if (!s_saveDlg || s_saveDlg->doSaveEVLR(i))
			{
				// All extra scalar fields are written as double.
				// A more specific solution would be welcome.
				extraFieldsToSave.push_back(extraFields[i]);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	//manage the point format
	minPointFormat = LasField::UpdateMinPointFormat(minPointFormat, hasColors, false, true);

	if (theCloud->hasMetaData(LAS_POINT_FORMAT_META_DATA)) //DGM: is it really necessary?
	{
		bool ok = false;
		unsigned previousPointFormat = theCloud->getMetaData(LAS_POINT_FORMAT_META_DATA).toUInt(&ok);
		if (ok && previousPointFormat < 256)
		{
			minPointFormat = std::max(static_cast<uint8_t>(previousPointFormat), minPointFormat);
		}
		else
		{
			ccLog::Warning("Invalid point_format metadata");
		}
	}

	CC_FILE_ERROR callbackError = CC_FERR_NO_ERROR;
	unsigned int ptsWritten = 0;
	auto convertOne = [&](PointRef& point)
	{
		if (ptsWritten == numberOfPoints)
			return false;

		if (pDlg && pDlg->isCancelRequested())
		{
			callbackError = CC_FERR_CANCELED_BY_USER;
			return false;
		}

		const CCVector3* P = theCloud->getPoint(ptsWritten);
		{
			CCVector3d Pglobal = theCloud->toGlobal3d<PointCoordinateType>(*P);
			point.setField(Id::X, Pglobal.x);
			point.setField(Id::Y, Pglobal.y);
			point.setField(Id::Z, Pglobal.z);
		}

		if (hasColors)
		{
			//DGM: LAS colors are stored on 16 bits!
			const ccColor::Rgb& rgb = theCloud->getPointColor(ptsWritten);
			point.setField(Id::Red,   static_cast<uint16_t>(rgb.r) << 8);
			point.setField(Id::Green, static_cast<uint16_t>(rgb.g) << 8);
			point.setField(Id::Blue,  static_cast<uint16_t>(rgb.b) << 8);
		}

		// standard las fields
		uint8_t classFlags = 0;
		uint8_t classification = 0;
		for (const LasField &lasField: fieldsToSave)
		{
			assert(lasField.sf);
			Id pdalId = typeToId(lasField.type, minPointFormat);
			switch (lasField.type)
			{
			case LAS_X:
			case LAS_Y:
			case LAS_Z:
			case LAS_RED:
			case LAS_GREEN:
			case LAS_BLUE:
				assert(false);
				break;
			case LAS_TIME:
				point.setField(pdalId, lasField.sf->getValue(ptsWritten) + lasField.sf->getGlobalShift());
				break;
			case LAS_CLASSIFICATION:
				classification = static_cast<uint8_t>(static_cast<int>(lasField.sf->getValue(ptsWritten)) & 255);
				break;
			case LAS_CLASSIF_VALUE:
				classification = static_cast<uint8_t>(static_cast<int>(lasField.sf->getValue(ptsWritten)) & (minPointFormat < 6 ? 31 : 255));
				break;
			case LAS_CLASSIF_SYNTHETIC:
				if (lasField.sf->getValue(ptsWritten) != 0)
				{
					if (minPointFormat < 6)
						classification |= 32; //bit #5 of the 'Classification' field
					else
						classFlags |= 1;      //bit #0 of the 'Classification Flags' field
				}
				break;
			case LAS_CLASSIF_KEYPOINT:
				if (lasField.sf->getValue(ptsWritten) != 0)
				{
					if (minPointFormat < 6)
						classification |= 64; //bit #6 of the 'Classification' field
					else
						classFlags |= 2;      //bit #1 of the 'Classification Flags' field
				}
				break;
			case LAS_CLASSIF_WITHHELD:
				if (lasField.sf->getValue(ptsWritten) != 0)
				{
					if (minPointFormat < 6)
						classification |= 128; //bit #7 of the 'Classification' field
					else
						classFlags |= 4;       //bit #2 of the 'Classification Flags' field
				}
				break;
			case LAS_CLASSIF_OVERLAP:
				if (lasField.sf->getValue(ptsWritten) != 0)
				{
					if (minPointFormat >= 6)
						classFlags |= 8;      //bit #3 of the 'Classification Flags' field
					else
						assert(false);
				}
				break;
			case LAS_INVALID:
				break;
			default:
				point.setField(pdalId, lasField.sf->getValue(ptsWritten));
				break;
			}
		}
		if (hasClassification)
			point.setField(Id::Classification, classification);
		if (hasClassifFlags)
			point.setField(Id::ClassFlags, classFlags);

		// extra las fields
		for (const ExtraLasField::Shared &extraField : extraFieldsToSave)
		{
			point.setField(extraField->pdalId, extraField->sf->getValue(ptsWritten) + extraField->sf->getGlobalShift());
		}

		nProgress.oneStep();

		++ptsWritten;
		return true;
	};

	try
	{
		LasWriter writer;
		Options writerOptions;

		if (theCloud->hasMetaData(s_LAS_SRS_Key))
		{
			//restore the SRS if possible
			QString wkt = theCloud->getMetaData(s_LAS_SRS_Key).value<QString>();
			writerOptions.add("a_srs", wkt.toStdString());
		}
		writerOptions.add("dataformat_id", minPointFormat);

		writerOptions.add("offset_x", lasOffset.x);
		writerOptions.add("offset_y", lasOffset.y);
		writerOptions.add("offset_z", lasOffset.z);

		writerOptions.add("scale_x", lasScale.x);
		writerOptions.add("scale_y", lasScale.y);
		writerOptions.add("scale_z", lasScale.z);

		writerOptions.add("filename", filename.toStdString());
		writerOptions.add("extra_dims", "all");

		if (theCloud->hasMetaData(LAS_GLOBAL_ENCODING_META_DATA))
		{
			bool ok = false;
			unsigned int global_encoding = theCloud->getMetaData(LAS_GLOBAL_ENCODING_META_DATA).toUInt(&ok);
			if (ok) {
				writerOptions.add("global_encoding", global_encoding);
			}
		}

		if (theCloud->hasMetaData(LAS_PROJECT_UUID_META_DATA))
		{
			QString uuid = theCloud->getMetaData(LAS_PROJECT_UUID_META_DATA).toString();
			writerOptions.add("project_id", uuid.toStdString());
		}

		if (theCloud->hasMetaData(LAS_VERSION_MINOR_META_DATA))
		{
			bool ok = false;
			int minor_version = theCloud->getMetaData(LAS_VERSION_MINOR_META_DATA).toInt(&ok);
			if (ok && minor_version != 0) // PDAL can read but not write LAS 1.0
				writerOptions.add("minor_version", minor_version);
			else if (!ok)
				ccLog::Warning(QString("Invalid minor_version metadata"));
		}

		StreamCallbackFilter f;
		f.setCallback(convertOne);
		writer.setInput(f);
		writer.setOptions(writerOptions);

		//field count 
		point_count_t tableSize = 3; //XYZ
		if (hasColors)
			tableSize += 3; //RGB
		tableSize += fieldsToSave.size();
		tableSize += extraFieldsToSave.size();

		FixedPointTable table(tableSize);

		table.layout()->registerDim(Id::X);
		table.layout()->registerDim(Id::Y);
		table.layout()->registerDim(Id::Z);

		if (hasColors)
		{
			table.layout()->registerDim(Id::Red);
			table.layout()->registerDim(Id::Green);
			table.layout()->registerDim(Id::Blue);
		}

		for (const LasField &lasField : fieldsToSave)
		{
			std::string dimName = lasField.getName().toStdString();
			Id pdalId = id(dimName);
			table.layout()->registerDim(pdalId);
		}

		for (const ExtraLasField::Shared extraField : extraFields)
		{
			std::string dimName = extraField->getName().toStdString();
			// All extra scalar fields are written as double.
			// A more specific solution would be welcome.
			Type t = Type::Double;
			extraField->pdalId = table.layout()->registerOrAssignDim(dimName, t);
		}

		writer.prepare(table);
		writer.execute(table);
	}
	catch (const pdal::pdal_error& p)
	{
		ccLog::Error(QString("PDAL exception: %1").arg(p.what()));
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch (const std::exception& e)
	{
		ccLog::Error(QString("PDAL generic exception: %1").arg(e.what()));
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return callbackError;
}

QSharedPointer<LASOpenDlg> s_lasOpenDlg(nullptr);

//! Class describing the current tiling process
class Tiler
{
public:
	Tiler()
	    : w(1)
	    , h(1)
	    , X(0)
	    , Y(1)
	    , Z(2)
	{}

	~Tiler() = default;

	inline size_t tileCount() const { return tilePointViews.size(); }

	bool init(unsigned int width,
	    unsigned int height,
	    unsigned int Zdim,
	    const QString &absoluteBaseFilename,
	    const CCVector3d& bbMin,
	    const CCVector3d& bbMax,
	    const PointTableRef table,
	    const LasHeader& header)
	{
		//init tiling dimensions
		assert(Zdim < 3);
		Z = Zdim;
		X = (Z == 2 ? 0 : Z + 1);
		Y = (X == 2 ? 0 : X + 1);

		bbMinCorner = bbMin;
		tileDiag = bbMax - bbMin;
		tileDiag.u[X] /= width;
		tileDiag.u[Y] /= height;
		unsigned int count = width * height;

		try
		{
			tilePointViews.resize(count);
			fileNames.resize(count);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return false;
		}

		w = width;
		h = height;

		//File extension
		QString ext = (header.compressed() ? "laz" : "las");

		for (unsigned int i = 0; i < width; ++i)
		{
			for (unsigned int j = 0; j < height; ++j)
			{
				unsigned int ii = index(i, j);
				QString filename = absoluteBaseFilename + QString("_%1_%2.%3").arg(QString::number(i), QString::number(j), ext);

				fileNames[ii] = filename;
				tilePointViews[ii] = std::make_shared<PointView>(table);
			}
		}

		return true;
	}

	void addPoint(const PointViewPtr &buffer, unsigned int pointIndex)
	{
		//determine the right tile
		CCVector3d Prel = CCVector3d(	buffer->getFieldAs<double>(Id::X, pointIndex),
		                                buffer->getFieldAs<double>(Id::Y, pointIndex),
		                                buffer->getFieldAs<double>(Id::Z, pointIndex));
		Prel -= bbMinCorner;
		int ii = static_cast<int>(floor(Prel.u[X] / tileDiag.u[X]));
		int ji = static_cast<int>(floor(Prel.u[Y] / tileDiag.u[Y]));
		unsigned int i = std::min(static_cast<unsigned int>(std::max(ii, 0)), w - 1);
		unsigned int j = std::min(static_cast<unsigned int>(std::max(ji, 0)), h - 1);
		PointViewPtr outputView = tilePointViews[index(i, j)];
		outputView->appendPoint(*buffer, pointIndex);
	}

	void writeAll()
	{
		for (unsigned int i = 0; i < tilePointViews.size(); ++i)
		{
			LasWriter writer;
			Options writerOptions;
			PointTable table;
			BufferReader bufferReader;

			writerOptions.add("filename", fileNames[i].toLocal8Bit().toStdString());
			if (tilePointViews[i]->empty())
				continue;
			try
			{
				bufferReader.addView(tilePointViews[i]);
				writer.setInput(bufferReader);
				writer.setOptions(writerOptions);
				writer.prepare(table);
				writer.execute(table);
			}
			catch (const pdal_error& e)
			{
				ccLog::Error(QString("PDAL exception '%1'").arg(e.what()));
			}
		}
	}

protected:

	inline unsigned int index(unsigned int i, unsigned int j) const { return i + j * w; }

	unsigned int w, h;
	unsigned int X, Y, Z;
	CCVector3d bbMinCorner, tileDiag;
	std::vector<PointViewPtr> tilePointViews;
	std::vector<QString> fileNames;
};


struct LasCloudChunk
{
	LasCloudChunk() : loadedCloud(nullptr), size(0) {}

	ccPointCloud* loadedCloud;
	std::vector< LasField::Shared > lasFields;
	unsigned int size;

	ccPointCloud* getLoadedCloud() const { return loadedCloud; }

	bool hasColors() const { return loadedCloud->hasColors(); }

	bool reserveSize(unsigned int nbPoints)
	{
		size = nbPoints;
		loadedCloud = new ccPointCloud();
		bool success = loadedCloud->reserveThePointsTable(nbPoints);
		if (!success)
			delete loadedCloud;

		return success;
	}

	void createFieldsToLoad(IdList extraFieldsToLoad, StringList extraNamesToLoad)
	{
		//DGM: from now on, we only enable scalar fields when we detect a valid value!
		if (s_lasOpenDlg->doLoad(LAS_CLASSIFICATION))
			lasFields.push_back(LasField::Shared(new LasField(LAS_CLASSIFICATION, 0, 0, 255))); //unsigned char: between 0 and 255
		if (s_lasOpenDlg->doLoad(LAS_CLASSIF_VALUE))
			lasFields.push_back(LasField::Shared(new LasField(LAS_CLASSIF_VALUE, 0, 0, 31))); //5 bits: between 0 and 31
		if (s_lasOpenDlg->doLoad(LAS_CLASSIF_SYNTHETIC))
			lasFields.push_back(LasField::Shared(new LasField(LAS_CLASSIF_SYNTHETIC, 0, 0, 1))); //1 bit: 0 or 1
		if (s_lasOpenDlg->doLoad(LAS_CLASSIF_KEYPOINT))
			lasFields.push_back(LasField::Shared(new LasField(LAS_CLASSIF_KEYPOINT, 0, 0, 1))); //1 bit: 0 or 1
		if (s_lasOpenDlg->doLoad(LAS_CLASSIF_WITHHELD))
			lasFields.push_back(LasField::Shared(new LasField(LAS_CLASSIF_WITHHELD, 0, 0, 1))); //1 bit: 0 or 1
		if (s_lasOpenDlg->doLoad(LAS_CLASSIF_OVERLAP))
			lasFields.push_back(LasField::Shared(new LasField(LAS_CLASSIF_OVERLAP, 0, 0, 1))); //1 bit: 0 or 1
		if (s_lasOpenDlg->doLoad(LAS_INTENSITY))
			lasFields.push_back(LasField::Shared(new LasField(LAS_INTENSITY, 0, 0, 65535))); //16 bits: between 0 and 65536
		if (s_lasOpenDlg->doLoad(LAS_TIME))
			lasFields.push_back(LasField::Shared(new LasField(LAS_TIME, 0, 0, -1.0))); //8 bytes (double) --> we use global shift!
		if (s_lasOpenDlg->doLoad(LAS_RETURN_NUMBER))
			lasFields.push_back(LasField::Shared(new LasField(LAS_RETURN_NUMBER, 1, 1, 7))); //3 bits: between 1 and 7
		if (s_lasOpenDlg->doLoad(LAS_NUMBER_OF_RETURNS))
			lasFields.push_back(LasField::Shared(new LasField(LAS_NUMBER_OF_RETURNS, 1, 1, 7))); //3 bits: between 1 and 7
		if (s_lasOpenDlg->doLoad(LAS_SCAN_DIRECTION))
			lasFields.push_back(LasField::Shared(new LasField(LAS_SCAN_DIRECTION, 0, 0, 1))); //1 bit: 0 or 1
		if (s_lasOpenDlg->doLoad(LAS_FLIGHT_LINE_EDGE))
			lasFields.push_back(LasField::Shared(new LasField(LAS_FLIGHT_LINE_EDGE, 0, 0, 1))); //1 bit: 0 or 1
		if (s_lasOpenDlg->doLoad(LAS_SCAN_ANGLE_RANK))
			lasFields.push_back(LasField::Shared(new LasField(LAS_SCAN_ANGLE_RANK, 0, -90, 90))); //signed char: between -90 and +90
		if (s_lasOpenDlg->doLoad(LAS_USER_DATA))
			lasFields.push_back(LasField::Shared(new LasField(LAS_USER_DATA, 0, 0, 255))); //unsigned char: between 0 and 255
		if (s_lasOpenDlg->doLoad(LAS_POINT_SOURCE_ID))
			lasFields.push_back(LasField::Shared(new LasField(LAS_POINT_SOURCE_ID, 0, 0, 65535))); //16 bits: between 0 and 65536

		//extra fields
		for (unsigned int i = 0; i < extraNamesToLoad.size(); ++i)
		{
			QString name = QString::fromStdString(extraNamesToLoad[i]);
			ExtraLasField *eField = new ExtraLasField(name, extraFieldsToLoad[i]);
			lasFields.emplace_back(eField);
		}
	}

	void addLasFieldsToCloud()
	{
		if (loadedCloud == nullptr)
			return;

		while (!lasFields.empty())
		{
			LasField::Shared& field = lasFields.back();
			if (field && field->sf)
			{
				field->sf->computeMinAndMax();

				if (   field->type == LAS_CLASSIFICATION
				    || field->type == LAS_CLASSIF_VALUE
				    || field->type == LAS_CLASSIF_SYNTHETIC
				    || field->type == LAS_CLASSIF_KEYPOINT
				    || field->type == LAS_CLASSIF_WITHHELD
				    || field->type == LAS_CLASSIF_OVERLAP
				    || field->type == LAS_RETURN_NUMBER
				    || field->type == LAS_NUMBER_OF_RETURNS)
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
				if (sfIndex >= 0 && !loadedCloud->hasDisplayedScalarField())
				{
					loadedCloud->setCurrentDisplayedScalarField(sfIndex);
					loadedCloud->showSF(!loadedCloud->hasColors());
				}
				field->sf->release();
				field->sf = nullptr;
			}
			else
			{
				ccLog::Warning(QString("[LAS] All '%1' values were the same (%2)! We ignored them...").arg(field->type == LAS_EXTRA ? field->getName() : QString(LAS_FIELD_NAMES[field->type])).arg(field->firstValue));
			}

			lasFields.pop_back();
		}
	}
};

/*
	The following functions until readExtraBytesVlr are copied from PDAL with slight modifications.
*/

using DT = Dimension::Type;
const Dimension::Type lastypes[] = {
    DT::None, DT::Unsigned8, DT::Signed8, DT::Unsigned16, DT::Signed16,
    DT::Unsigned32, DT::Signed32, DT::Unsigned64, DT::Signed64,
    DT::Float, DT::Double
};


void ExtraBytesIf::setType(uint8_t lastype)
{
	m_fieldCnt = 1;
	while (lastype > 10)
	{
		m_fieldCnt++;
		lastype -= 10;
	}

	m_type = lastypes[lastype];
	if (m_type == Dimension::Type::None)
		m_fieldCnt = 0;
}


void ExtraBytesIf::readFrom(const char *buf)
{
	LeExtractor extractor(buf, sizeof(ExtraBytesSpec));
	uint16_t dummy16;
	uint32_t dummy32;
	uint64_t dummy64;
	double dummyd;
	uint8_t options;
	uint8_t type;

	uint8_t SCALE_MASK = 1 << 3;
	uint8_t OFFSET_MASK = 1 << 4;

	extractor >> dummy16 >> type >> options;
	extractor.get(m_name, 32);
	extractor >> dummy32;
	for (size_t i = 0; i < 3; ++i)
		extractor >> dummy64;  // No data field.
	for (size_t i = 0; i < 3; ++i)
		extractor >> dummyd;  // Min.
	for (size_t i = 0; i < 3; ++i)
		extractor >> dummyd;  // Max.
	for (size_t i = 0; i < 3; ++i)
		extractor >> m_scale[i];
	for (size_t i = 0; i < 3; ++i)
		extractor >> m_offset[i];
	extractor.get(m_description, 32);

	setType(type);
	if (m_type == Dimension::Type::None)
		m_size = options;
	if (!(options & SCALE_MASK))
		for (size_t i = 0; i < 3; ++i)
			m_scale[i] = 1.0;
	if (!(options & OFFSET_MASK))
		for (size_t i = 0; i < 3; ++i)
			m_offset[i] = 0.0;
}


std::vector<ExtraDim> ExtraBytesIf::toExtraDims()
{
	std::vector<ExtraDim> eds;

	if (m_type == Dimension::Type::None)
	{
		ExtraDim ed(m_name, Dimension::Type::None);
		ed.m_size = m_size;
		eds.push_back(ed);
	}
	else if (m_fieldCnt == 1)
	{
		ExtraDim ed(m_name, m_type, m_scale[0], m_offset[0]);
		eds.push_back(ed);
	}
	else
	{
		for (size_t i = 0; i < m_fieldCnt; ++i)
		{
			ExtraDim ed(m_name + std::to_string(i), m_type,
			    m_scale[i], m_offset[i]);
			eds.push_back(ed);
		}
	}
	return eds;
}

static bool ReadExtraBytesVlr(LasHeader &header, std::vector<ExtraDim>& extraDims)
{
	const LasVLR *vlr = header.findVlr(SPEC_USER_ID, EXTRA_BYTES_RECORD_ID);
	if (!vlr)
	{
		return false;
	}
	
	size_t size = vlr->dataLen();
	if (size % sizeof(ExtraBytesSpec) != 0)
	{
		ccLog::Warning("[LAS] Bad size for extra bytes VLR. Ignoring.");
		return false;
	}
	size_t count = size / sizeof(ExtraBytesSpec);
	ccLog::PrintDebug("[LAS] VLR count: " + QString::number(count));

	try
	{
		const char* pos = vlr->data();
		for (size_t i = 0; i < count; ++i)
		{
			ExtraBytesIf eb;
			eb.readFrom(pos);
			pos += sizeof(ExtraBytesSpec);

			std::vector<ExtraDim> eds = eb.toExtraDims();
			for (const ExtraDim& ed : eds)
			{
				ccLog::PrintDebug(QString("[LAS] VLR #%1: %2").arg(i + 1).arg(QString::fromStdString(ed.m_name)));
				extraDims.push_back(ed);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[LAS] Not enough memory to retrieve the extra bytes fields.");
		return false;
	}

	return true;
}

CC_FILE_ERROR LASFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	try
	{
		Options las_opts;
		las_opts.add("filename", filename.toStdString());

		LasReader lasReader;
		lasReader.setOptions(las_opts);
		FixedPointTable fields(100);
		PointLayoutPtr layout(fields.layout());
		lasReader.prepare(fields);
		LasHeader lasHeader = lasReader.header();

		unsigned nbOfPoints = static_cast<unsigned>(lasHeader.pointCount());
		if (nbOfPoints == 0)
		{
			ccPointCloud* emptyCloud = new ccPointCloud("empty");
			container.addChild(emptyCloud);
			//strange file ;) 
			return CC_FERR_NO_ERROR; //Its still strange
		}

		//The VLR record describing the extra bytes has been added to LAS 1.4 to formalize
		//a process that has been used in prior versions of LAS.
		//So PDAL doesn't read this VLR if the version is <= 1.3.
		//The idea is to read the VLR manually, make a string of the names and data types,
		//and pass that back to the PDAL reader.
		std::vector<ExtraDim> extraDims;
		ReadExtraBytesVlr(lasHeader, extraDims);

		CCVector3d bbMin(lasHeader.minX(), lasHeader.minY(), lasHeader.minZ());
		CCVector3d bbMax(lasHeader.maxX(), lasHeader.maxY(), lasHeader.maxZ());
		CCVector3d lasScale(lasHeader.scaleX(), lasHeader.scaleY(), lasHeader.scaleZ());
		CCVector3d lasOffset(lasHeader.offsetX(), lasHeader.offsetY(), lasHeader.offsetZ());

		const uint8_t pointFormat = lasHeader.pointFormat();
		ccLog::Print("[LAS] Point format: " + QString::number(pointFormat));

		if (!s_lasOpenDlg)
		{
			s_lasOpenDlg = QSharedPointer<LASOpenDlg>(new LASOpenDlg());
		}

		QuickInfo file_info = lasReader.preview();
		s_lasOpenDlg->setDimensions(file_info.m_dimNames);
		s_lasOpenDlg->clearEVLRs();
		s_lasOpenDlg->setInfos(filename, nbOfPoints, bbMin, bbMax);
		if (pointFormat <= 5)
		{
			s_lasOpenDlg->classifOverlapCheckBox->setEnabled(false);
			s_lasOpenDlg->classifOverlapCheckBox->setVisible(false);
		}

		for (const ExtraDim& dim : extraDims)
		{
			s_lasOpenDlg->addEVLR(QString("%1").arg(QString::fromStdString(dim.m_name)));
		}

		if (parameters.sessionStart)
		{
			//we do this AFTER restoring the previous context because it may still be
			//good that the previous configuration is restored even though the user needs
			//to confirm it
			s_lasOpenDlg->resetApplyAll();
		}

		if (parameters.alwaysDisplayLoadDialog && !s_lasOpenDlg->autoSkipMode() && !s_lasOpenDlg->exec())
		{
			return CC_FERR_CANCELED_BY_USER;
		}

		bool ignoreDefaultFields = s_lasOpenDlg->ignoreDefaultFieldsCheckBox->isChecked();

		unsigned int short rgbColorMask[3] = { 0, 0, 0 };
		if (s_lasOpenDlg->doLoad(LAS_RED))
			rgbColorMask[0] = (~0);
		if (s_lasOpenDlg->doLoad(LAS_GREEN))
			rgbColorMask[1] = (~0);
		if (s_lasOpenDlg->doLoad(LAS_BLUE))
			rgbColorMask[2] = (~0);
		bool loadColor = (rgbColorMask[0] || rgbColorMask[1] || rgbColorMask[2]);

		//by default we read colors as triplets of 8 bits integers but we might dynamically change this
		//if we encounter values using 16 bits (16 bits is the standard!)
		unsigned char colorCompBitShift = 0;
		bool forced8bitRgbMode = s_lasOpenDlg->forced8bitRgbMode();
		ccColor::Rgb rgb(0, 0, 0);

		StringList extraNamesToLoad;
		std::string extraDimsArg;
		for (unsigned i = 0; i < extraDims.size(); ++i)
		{
			if (s_lasOpenDlg->doLoadEVLR(i))
			{
				extraDimsArg += extraDims[i].m_name + "=" + interpretationName(extraDims[i].m_dimType.m_type) + ",";
				extraNamesToLoad.push_back(extraDims[i].m_name);
			}
		}

		if (!extraNamesToLoad.empty())
		{
			// If extra fields are requested, reload the file with the new extra_dims parameters
			Options las_opts2;
			las_opts2.add("extra_dims", extraDimsArg);

			lasReader.addOptions(las_opts2);
			lasReader.prepare(fields);
		}

		std::vector<Id> extraDimensionsIds;
		for (std::string &dim : extraNamesToLoad)
		{
			extraDimensionsIds.push_back(layout->findDim(dim));
		}

		bool tiling = s_lasOpenDlg->tileGroupBox->isChecked();

		QScopedPointer<ccProgressDialog> pDlg(nullptr);
		if (parameters.parentWidget)
		{
			pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
			pDlg->setMethodTitle(QObject::tr("Open LAS file"));
			pDlg->setInfo(QObject::tr("Points: %L1").arg(nbOfPoints));
			pDlg->start();
		}

		if (tiling)
		{
			Tiler tiler;
			PointTable table;
			PointViewSet pointViewSet;

			// tiling (vertical) dimension
			unsigned int vertDim = 2;
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

			auto w = static_cast<unsigned int>(s_lasOpenDlg->wTileSpinBox->value());
			auto h = static_cast<unsigned int>(s_lasOpenDlg->hTileSpinBox->value());

			QString outputBaseName = s_lasOpenDlg->outputPathLineEdit->text() + "/" + QFileInfo(filename).baseName();
			if (!tiler.init(w, h, vertDim, outputBaseName, bbMin, bbMax, table, lasHeader))
			{
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}

			auto prepareAndExecute = [&lasReader, &table]() -> PointViewSet {
				lasReader.prepare(table);
				lasReader.prepare(table);
				return lasReader.execute(table);
			};

			if (parameters.parentWidget)
			{
				pDlg.reset(new ccProgressDialog(false, parameters.parentWidget));
				pDlg->setMethodTitle(QObject::tr("LAS file"));
				pDlg->setInfo(QObject::tr("Please wait... reading in progress"));
				pDlg->setRange(0, 0);
				pDlg->setModal(true);
				pDlg->start();
			}

			QFutureWatcher<PointViewSet> reader;
			QObject::connect(&reader, SIGNAL(finished()), pDlg.data(), SLOT(reset()));
			reader.setFuture(QtConcurrent::run(prepareAndExecute));

			if (pDlg)
			{
				pDlg->exec();
			}
			reader.waitForFinished();

			PointViewSet viewSet = reader.result();
			PointViewPtr pointView = *viewSet.begin();

			if (parameters.parentWidget && pDlg)
			{
				pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
				pDlg->setMethodTitle(QObject::tr("Tiling points"));
				pDlg->setInfo(QObject::tr("Points: %L1").arg(nbOfPoints));
				pDlg->start();
			}
			CCLib::NormalizedProgress nProgress(pDlg.data(), nbOfPoints);

			for (PointId idx = 0; idx < pointView->size(); ++idx)
			{
				if (pDlg->isCancelRequested())
					return CC_FERR_CANCELED_BY_USER;
				tiler.addPoint(pointView, idx);
				nProgress.oneStep();
			}

			// Now the tiler will actually write the points
			if (parameters.parentWidget)
			{
				pDlg.reset(new ccProgressDialog(false, parameters.parentWidget));
				pDlg->setMethodTitle(QObject::tr("LAS file"));
				pDlg->setInfo(QObject::tr("Please wait... writing in progress"));
				pDlg->setRange(0, 0);
				pDlg->setModal(true);
				pDlg->start();
			}

			QFutureWatcher<void> writer;
			QObject::connect(&writer, SIGNAL(finished()), pDlg.data(), SLOT(reset()));
			writer.setFuture(QtConcurrent::run([&tiler]() {tiler.writeAll(); }));

			pDlg->exec();
			writer.waitForFinished();

			return CC_FERR_NO_ERROR;
		}

		CCLib::NormalizedProgress nProgress(pDlg.data(), nbOfPoints);
		ccPointCloud* loadedCloud = nullptr;
		std::vector< LasField::Shared > fieldsToLoad;
		CCVector3d Pshift(0, 0, 0);
		bool preserveCoordinateShift = true;

		unsigned int fileChunkSize = 0;
		unsigned int nbPointsRead = 0;

		StreamCallbackFilter f;
		f.setInput(lasReader);

		unsigned int nbOfChunks = (nbOfPoints / CC_MAX_NUMBER_OF_POINTS_PER_CLOUD) + 1;
		std::vector<LasCloudChunk> chunks(nbOfChunks, LasCloudChunk());

		CC_FILE_ERROR callbackError = CC_FERR_NO_ERROR;
		auto ccProcessOne = [&](PointRef& point)
		{
			if (pDlg && pDlg->isCancelRequested())
			{
				callbackError = CC_FERR_CANCELED_BY_USER;
				return false;
			}

			LasCloudChunk &pointChunk = chunks[nbPointsRead / CC_MAX_NUMBER_OF_POINTS_PER_CLOUD];

			if (pointChunk.getLoadedCloud() == nullptr)
			{
				// create a new cloud
				unsigned int pointsToRead = nbOfPoints - nbPointsRead;
				fileChunkSize = std::min(pointsToRead, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
				if (!pointChunk.reserveSize(fileChunkSize))
				{
					ccLog::Warning("[LAS] Not enough memory!");
					callbackError = CC_FERR_NOT_ENOUGH_MEMORY;
					return false;
				}

				if (preserveCoordinateShift)
				{
					pointChunk.loadedCloud->setGlobalShift(Pshift);
				}

				//save the Spatial reference as meta-data
				SpatialReference srs = lasHeader.srs();
				if (!srs.empty())
				{
					QString wkt = QString::fromStdString(srs.getWKT());
					ccLog::Print("[LAS] Spatial reference: " + wkt);
					pointChunk.loadedCloud->setMetaData(s_LAS_SRS_Key, wkt);
				}
				else if (lasHeader.incompatibleSrs())
				{
					ccLog::Warning("[LAS] Incompatible spatial reference");
				}
				else
				{
					ccLog::Print("[LAS] Spatial reference: None");
				}

				pointChunk.createFieldsToLoad(extraDimensionsIds, extraNamesToLoad);
			}

			loadedCloud = pointChunk.loadedCloud;
			fieldsToLoad = pointChunk.lasFields;

			//first point check for 'big' coordinates
			if (nbPointsRead == 0)
			{
				CCVector3d P(	static_cast<PointCoordinateType>(point.getFieldAs<int>(Id::X)),
								static_cast<PointCoordinateType>(point.getFieldAs<int>(Id::Y)),
								static_cast<PointCoordinateType>(point.getFieldAs<int>(Id::Z)) );

				//backup input global parameters
				ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
				bool useLasOffset = false;
				//set the lasOffset as default if none was provided
				if (lasOffset.norm2() != 0 && (!parameters.coordinatesShiftEnabled || !*parameters.coordinatesShiftEnabled))
				{
					    if (csModeBackup != ccGlobalShiftManager::NO_DIALOG) //No dialog, practically means that we don't want any shift!
						{
							useLasOffset = true;
							Pshift = -lasOffset;
							if (csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
							{
								parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
							}
						}
				}

				if (HandleGlobalShift(P, Pshift, preserveCoordinateShift, parameters, useLasOffset))
				{
					if (preserveCoordinateShift)
					{
						loadedCloud->setGlobalShift(Pshift);
					}
					ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
				}

				//restore previous parameters
				parameters.shiftHandlingMode = csModeBackup;
			}

			CCVector3 P(static_cast<PointCoordinateType>(point.getFieldAs<double>(Id::X) + Pshift.x),
			            static_cast<PointCoordinateType>(point.getFieldAs<double>(Id::Y) + Pshift.y),
			            static_cast<PointCoordinateType>(point.getFieldAs<double>(Id::Z) + Pshift.z));
			loadedCloud->addPoint(P);

			if (loadColor)
			{
				unsigned short red   = point.getFieldAs<unsigned short>(Id::Red  ) & rgbColorMask[0];
				unsigned short green = point.getFieldAs<unsigned short>(Id::Green) & rgbColorMask[1];
				unsigned short blue  = point.getFieldAs<unsigned short>(Id::Blue ) & rgbColorMask[2];

				// if we don't have reserved a color field yet, we check that color is not black
				bool pushColor = true;
				if (!loadedCloud->hasColors())
				{
					if (red || green || blue)
					{
						if (loadedCloud->reserveTheRGBTable())
						{
							// we must set the color (black) of all previously skipped points
							for (unsigned int i = 0; i < loadedCloud->size() - 1; ++i)
							{
								loadedCloud->addColor(ccColor::black);
							}
						}
						else
						{
							ccLog::Warning("[LAS]: Not enough memory, color field will be ignored!");
							loadColor = false; //no need to retry with the other chunks anyway
							pushColor = false;
						}
					}
					else //otherwise we ignore it for the moment (we'll add it later if necessary)
					{
						pushColor = false;
					}
				}
				if (pushColor)
				{
					//we test if the color components are on 16 bits (standard) or only on 8 bits (it happens ;)
					if (!forced8bitRgbMode && colorCompBitShift == 0)
					{
						if (   (red   & 0xFF00)
						    || (green & 0xFF00)
						    || (blue  & 0xFF00) )
						{
							//the color components are on 16 bits!
							ccLog::Print("[LAS] Color components are coded on 16 bits");
							colorCompBitShift = 8;
							//we fix all the previously read colors
							for (unsigned int i = 0; i < loadedCloud->size() - 1; ++i)
							{
								loadedCloud->setPointColor(i, ccColor::black); //255 >> 8 = 0!
							}
						}
					}
					rgb.r = static_cast<ColorCompType>(red   >> colorCompBitShift);
					rgb.g = static_cast<ColorCompType>(green >> colorCompBitShift);
					rgb.b = static_cast<ColorCompType>(blue  >> colorCompBitShift);

					loadedCloud->addColor(rgb);
				}
			}

			// additional fields
			for (LasField::Shared& field : fieldsToLoad)
			{
				double value = 0.0;
				Id pdalId = typeToId(field->type, pointFormat);

				switch (field->type)
				{
				case LAS_EXTRA:
				{
					ExtraLasField* extraField = static_cast<ExtraLasField*>(field.data());
					value = point.getFieldAs<double>(extraField->pdalId);
					break;
				}
				case LAS_TIME:
					value = point.getFieldAs<double>(Id::GpsTime);
					if (field->sf)
					{
						//shift time values (so as to avoid losing accuracy)
						value -= field->sf->getGlobalShift();
					}
					break;
				case LAS_CLASSIF_VALUE:
					if (pointFormat <= 5)
						value = (point.getFieldAs<int>(pdalId) & 31);  //bit #0-4 of the 'Classification' field
					else
						value = point.getFieldAs<int>(pdalId);
					break;
				case LAS_CLASSIF_SYNTHETIC:
					if (pointFormat <= 5)
						value = (point.getFieldAs<int>(pdalId) & 32) ? 1.0 : 0.0;  //bit #5 of the 'Classification' field
					else
						value = (point.getFieldAs<int>(pdalId) & 1)  ? 1.0 : 0.0;   //bit #0 of the 'Classification Flags' field
					break;
				case LAS_CLASSIF_KEYPOINT:
					if (pointFormat <= 5)
						value = (point.getFieldAs<int>(pdalId) & 64) ? 1.0 : 0.0;  //bit #6 of the 'Classification' field
					else
						value = (point.getFieldAs<int>(pdalId) & 2 ) ? 1.0 : 0.0;   //bit #1 of the 'Classification Flags' field
					break;
				case LAS_CLASSIF_WITHHELD:
					if (pointFormat <= 5)
						value = (point.getFieldAs<int>(pdalId) & 128) ? 1.0 : 0.0; //bit #7 of the 'Classification' field
					else
						value = (point.getFieldAs<int>(pdalId) & 4)   ? 1.0 : 0.0;   //bit #2 of the 'Classification Flags' field
					break;
				case LAS_CLASSIF_OVERLAP:
					if (pointFormat <= 5)
					{
						assert(false);                                 //not present before point format 6
					}
					else
					{
						value = (point.getFieldAs<int>(pdalId) & 8) ? 1.0 : 0.0;   //bit #3 of the 'Classification Flags' field
					}
					break;
				default:
					value = point.getFieldAs<double>(pdalId);
					break;
				}
				if (field->sf)
				{
					auto s = static_cast<ScalarType>(value);
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
						if (field->sf->reserveSafe(fileChunkSize))
						{
							field->sf->link();
							if (field->type == LAS_TIME)
							{
								//we use the first value as 'global shift' (otherwise we will lose accuracy)
								field->sf->setGlobalShift(field->firstValue);
								value -= field->firstValue;
								ccLog::Warning("[LAS] Time SF has been shifted to prevent a loss of accuracy (%.2f)", field->firstValue);
								field->firstValue = 0;
							}

							auto defaultValue = static_cast<ScalarType>(field->defaultValue);
							for (unsigned i = 1; i < loadedCloud->size(); ++i)
							{
								field->sf->emplace_back(defaultValue);
							}
							auto s = static_cast<ScalarType>(value);
							field->sf->emplace_back(s);
						}
						else
						{
							ccLog::Warning(QString("[LAS] Not enough memory: '%1' field will be ignored!").arg(LAS_FIELD_NAMES[field->type]));
							field->sf->release();
							field->sf = nullptr;
						}
					}

				}

			}
			++nbPointsRead;
			nProgress.oneStep();
			return true;
		};

		f.setCallback(ccProcessOne);
		f.prepare(fields);
		f.execute(fields);

		if (callbackError != CC_FERR_NO_ERROR)
		{
			return callbackError;
		}

		for (auto &chunk : chunks)
		{
			chunk.addLasFieldsToCloud();
			loadedCloud = chunk.getLoadedCloud();

			if (loadedCloud)
			{
				if (loadedCloud->size())
				{
					bool thisChunkHasColors = chunk.hasColors();
					loadedCloud->showColors(thisChunkHasColors);
					if (loadColor && !thisChunkHasColors)
					{
						ccLog::Warning("[LAS] Color field was all black! We ignored it...");
					}

					// if we had reserved too much memory
					if (loadedCloud->size() < loadedCloud->capacity())
					{
						loadedCloud->resize(loadedCloud->size());
					}

					QString chunkName("unnamed - Cloud");
					unsigned int n = container.getChildrenNumber();
					if (n != 0)
					{
						if (n == 1)
						{
							container.getChild(0)->setName(chunkName + QString(" #1"));
						}
						chunkName += QString(" #%1").arg(n + 1);
					}
					loadedCloud->setName(chunkName);

					loadedCloud->setMetaData(LAS_SCALE_X_META_DATA, QVariant(lasScale.x));
					loadedCloud->setMetaData(LAS_SCALE_Y_META_DATA, QVariant(lasScale.y));
					loadedCloud->setMetaData(LAS_SCALE_Z_META_DATA, QVariant(lasScale.z));
					loadedCloud->setMetaData(LAS_OFFSET_X_META_DATA, QVariant(lasOffset.x));
					loadedCloud->setMetaData(LAS_OFFSET_Y_META_DATA, QVariant(lasOffset.y));
					loadedCloud->setMetaData(LAS_OFFSET_Z_META_DATA, QVariant(lasOffset.z));
					loadedCloud->setMetaData(LAS_GLOBAL_ENCODING_META_DATA, QVariant(lasHeader.globalEncoding()));

					const pdal::Uuid projectUUID = lasHeader.projectId();
					if (!projectUUID.isNull()) {
						loadedCloud->setMetaData(
							LAS_PROJECT_UUID_META_DATA,
							QVariant(QString::fromStdString(projectUUID.toString()))
						);
					}

					loadedCloud->setMetaData(LAS_VERSION_MAJOR_META_DATA, QVariant(lasHeader.versionMajor()));
					loadedCloud->setMetaData(LAS_VERSION_MINOR_META_DATA, QVariant(lasHeader.versionMinor()));
					loadedCloud->setMetaData(LAS_POINT_FORMAT_META_DATA, QVariant(pointFormat));

					container.addChild(loadedCloud);
					loadedCloud = nullptr;
				}
				else
				{
					//empty cloud?!
					delete loadedCloud;
					loadedCloud = nullptr;
				}
			}
		}
	}
	catch (const pdal::pdal_error& p)
	{
		ccLog::Error(QString("PDAL exception: %1").arg(p.what()));
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}
	catch (const std::exception& e)
	{
		ccLog::Error(QString("PDAL generic exception: %1").arg(e.what()));
		return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return CC_FERR_NO_ERROR;
}
