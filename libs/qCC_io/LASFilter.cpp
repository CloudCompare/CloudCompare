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
#include <iostream>				// std::cout
#include <bitset>

static const char s_LAS_SRS_Key[] = "LAS.spatialReference.nosave"; //DGM: added the '.nosave' suffix because this custom type can't be streamed properly

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
    return (	upperCaseExt == "LAS" ||
                upperCaseExt == "LAZ");
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
    //! Default constructor
    ExtraLasField(QString name, Id id, double defaultVal = 0, double min = 0.0, double max = -1.0)
        : LasField(LAS_EXTRA,defaultVal,min,max)
        , fieldName(name)
        , pdalId(id)
        , scale(1.0)
        , offset(0.0)
    {}

    //reimplemented from LasField
    virtual inline QString getName() const	{ return fieldName; }

    QString fieldName;
    Id pdalId;
    double scale;
    double offset;
};

//! Semi persistent save dialog
QSharedPointer<LASSaveDlg> s_saveDlg(0);
pdal::Dimension::Id typeToId(LAS_FIELDS sfType)
{
	switch (sfType) {
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
		return pdal::Dimension::Id::ClassFlags;
	case LAS_FIELDS::LAS_CLASSIF_KEYPOINT:
		return pdal::Dimension::Id::ClassFlags;
	case LAS_FIELDS::LAS_CLASSIF_WITHHELD:
		return pdal::Dimension::Id::ClassFlags;
		//Invald flag
	case LAS_FIELDS::LAS_INVALID:
	default:
		return pdal::Dimension::Id::Unknown;
	};
}


CC_FILE_ERROR LASFilter::saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters)
{
    if (!entity || filename.isEmpty())
        return CC_FERR_BAD_ARGUMENT;

    ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(entity);
    if (!theCloud)
    {
        ccLog::Warning("[LAS] This filter can only save one cloud at a time");
        return CC_FERR_BAD_ENTITY_TYPE;
    }

    unsigned numberOfPoints = theCloud->size();
    if (numberOfPoints == 0)
    {
        ccLog::Warning("[LAS] Cloud is empty!");
        return CC_FERR_NO_SAVE;
    }

    //colors
    bool hasColors = theCloud->hasColors();

    //additional fields (as scalar fields)
    std::vector<LasField> fieldsToSave;

    if (theCloud->isA(CC_TYPES::POINT_CLOUD))
    {
        ccPointCloud* pc = static_cast<ccPointCloud*>(theCloud);

        //match cloud SFs with official LASfields
        LasField::GetLASFields(pc, fieldsToSave);
    }

    //progress dialog
    QScopedPointer<ccProgressDialog> pDlg(0);
    if (parameters.parentWidget)
    {
        pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
        pDlg->setMethodTitle(QObject::tr("Save LAS file"));
        pDlg->setInfo(QObject::tr("Points: %1").arg(numberOfPoints));
        pDlg->start();
    }
    CCLib::NormalizedProgress nProgress(pDlg.data(), numberOfPoints);

	LasWriter writer;
    Options writerOptions;
    FixedPointTable table(100);

    CCVector3d bbMin, bbMax;
    if (!theCloud->getGlobalBB(bbMin, bbMax))
    {
        return CC_FERR_NO_SAVE;
    }

    CCVector3d diag = bbMax - bbMin;


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
    }
    else if (!hasScaleMetaData)
    {
        lasScale = optimalScale;
    }

	if (theCloud->hasMetaData(s_LAS_SRS_Key))
	{
		//restore the SRS if possible
		QString srs = theCloud->getMetaData(s_LAS_SRS_Key).value<QString>();
		writerOptions.add("a_srs", srs.toStdString());
	}

    for (LasField &lasField: fieldsToSave)
    {
        table.layout()->registerDim(id(lasField.getName().toStdString()));
    }

    if (hasColors)
    {
        table.layout()->registerDim(Id::Red);
        table.layout()->registerDim(Id::Green);
        table.layout()->registerDim(Id::Blue);
    }

    IdList dims = table.layout()->dims();
    for (auto &dimId: dims)
    {
        std::cerr << "dim: " << name(dimId) << std::endl;
    }
    table.layout()->registerDim(Id::X);
    table.layout()->registerDim(Id::Y);
    table.layout()->registerDim(Id::Z);

	unsigned ptsWritten = 0;

	auto convertOne = [&](PointRef& point)
	{
		if (ptsWritten == numberOfPoints || pDlg->isCancelRequested())
			return false;

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
			const ColorCompType* rgb = theCloud->getPointColor(ptsWritten);
			point.setField(Id::Red, static_cast<uint16_t>(rgb[0]) << 8);
			point.setField(Id::Green, static_cast<uint16_t>(rgb[1]) << 8);
			point.setField(Id::Blue, static_cast<uint16_t>(rgb[2]) << 8);
		}
		
		//additional fields
		for (std::vector<LasField>::const_iterator it = fieldsToSave.begin(); it != fieldsToSave.end(); ++it)
		{
			std::bitset<8> classFlags;
			Id pdalId;
		    assert(it->sf);
			pdalId = typeToId(it->type);
			switch(it->type)
			{
			case LAS_X:
			case LAS_Y:
			case LAS_Z:
				assert(false);
				break;
			case LAS_RED:
			case LAS_GREEN:
			case LAS_BLUE:
				assert(false);
				break;
			case LAS_TIME:
				point.setField(pdalId, it->sf->getValue(ptsWritten) + it->sf->getGlobalShift());
				break;
			case LAS_CLASSIF_SYNTHETIC:
				classFlags.set(0);
				break;
			case LAS_CLASSIF_KEYPOINT:
				classFlags.set(1);
				break;
			case LAS_CLASSIF_WITHHELD:
				classFlags.set(2);
				break;
			//TODO: Overlap flag (new in las 1.4)
			case LAS_INVALID:
				break;
			default:
				point.setField(pdalId, it->sf->getValue(ptsWritten));
				break;
			}
			point.setField(Id::ClassFlags, classFlags.to_ulong());
		 }

		nProgress.oneStep();

		++ptsWritten;
		return true;
	};

	//Set offset & scale, as points will be stored as boost::int32_t values (between 0 and 4294967296)
	//int_value = (double_value-offset)/scale
	writerOptions.add("offset_x", bbMin.x);
	writerOptions.add("offset_y", bbMin.y);
	writerOptions.add("offset_z", bbMin.z);

	writerOptions.add("scale_x", lasScale.x);
    writerOptions.add("scale_y", lasScale.y);
    writerOptions.add("scale_z", lasScale.z);

    writerOptions.add("filename", filename.toStdString());
    //make a dialog for this ?
    //writerOptions.add("minor_version", )
    //writerOptions.add("dataformat_id", );

	StreamCallbackFilter f;
	f.setCallback(convertOne);
	writer.setInput(f);
	writer.setOptions(writerOptions);

    try
    {
		writer.prepare(table);
		writer.execute(table);
    }
    catch (const pdal_error& e)
    {
        ccLog::Error(QString("PDAL exception '%1'").arg(e.what()));
        return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
    }
    catch (...)
    {
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    return CC_FERR_NO_ERROR;
}


QSharedPointer<LASOpenDlg> s_lasOpenDlg(0);

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

    bool init(	unsigned width,
                unsigned height,
                unsigned Zdim,
                QString absoluteBaseFilename,
                const CCVector3d& bbMin,
                const CCVector3d& bbMax,
                const PointTableRef table)
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
        //        QString ext = (header.Compressed() ? "laz" : "las");

        for (unsigned i = 0; i < width; ++i)
        {
            for (unsigned j = 0; j < height; ++j)
            {
                unsigned ii = index(i, j);
                //TODO: Don't forget to change the ext to be able to write .laz
                QString filename = absoluteBaseFilename + QString("_%1_%2.%3").arg(QString::number(i), QString::number(j), QString("las"));

                fileNames[ii] = filename;
                tilePointViews[ii] = PointViewPtr(new PointView(table));
            }
        }

        return true;
    }

    void addPoint(const PointViewPtr buffer, unsigned pointIndex)
    {
        //determine the right tile
        CCVector3d Prel = CCVector3d(buffer->getFieldAs<double>(Id::X, pointIndex),
                                     buffer->getFieldAs<double>(Id::Y, pointIndex),
                                     buffer->getFieldAs<double>(Id::Z, pointIndex));
        Prel -= bbMinCorner;
        int ii = static_cast<int>(floor(Prel.u[X] / tileDiag.u[X]));
        int ji = static_cast<int>(floor(Prel.u[Y] / tileDiag.u[Y]));
        unsigned i = std::min( static_cast<unsigned>(std::max(ii, 0)), w-1);
        unsigned j = std::min( static_cast<unsigned>(std::max(ji, 0)), h-1);
        PointViewPtr outputView = tilePointViews[index(i,j)];
        outputView->appendPoint(*buffer, pointIndex);
    }

    void writeAll()
    {

        for (unsigned i = 0; i < tilePointViews.size(); ++i)
        {
            LasWriter writer;
            Options writerOptions;
            PointTable table;
            BufferReader bufferReader;

            writerOptions.add("filename", fileNames[i].toStdString());
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

    inline unsigned index(unsigned i, unsigned j) const { return i + j * w; }

    unsigned w, h;
    unsigned X, Y, Z;
    CCVector3d bbMinCorner, tileDiag;
    std::vector<PointViewPtr> tilePointViews;
    std::vector<QString> fileNames;
};


struct LasCloudChunk 
{
	LasCloudChunk() : loadedCloud(nullptr), size(0) {}

	ccPointCloud* loadedCloud;
	std::vector< LasField::Shared > lasFields;
	unsigned size;

	ccPointCloud* getLoadedCloud() const { return loadedCloud; }

	bool hasColors() const { return loadedCloud->hasColors(); }

	bool reserveSize(unsigned nbPoints)
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
		for (unsigned i = 0; i < extraNamesToLoad.size(); ++i)
		{
			QString name = QString::fromStdString(extraNamesToLoad[i]);
			ExtraLasField *eField = new ExtraLasField(name, extraFieldsToLoad[i]);
			lasFields.push_back(LasField::Shared(eField));
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

				if (field->type == LAS_CLASSIFICATION
					|| field->type == LAS_CLASSIF_VALUE
					|| field->type == LAS_CLASSIF_SYNTHETIC
					|| field->type == LAS_CLASSIF_KEYPOINT
					|| field->type == LAS_CLASSIF_WITHHELD
					|| field->type == LAS_RETURN_NUMBER
					|| field->type == LAS_NUMBER_OF_RETURNS)
				{
					auto cMin = static_cast<int>(field->sf->getMin());
					auto cMax = static_cast<int>(field->sf->getMax());
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

CC_FILE_ERROR LASFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
    Options las_opts;
    las_opts.add("filename", filename.toStdString());

    FixedPointTable t(100);
    LasReader lasReader;
    LasHeader lasHeader;
    QuickInfo file_info;
    PointLayoutPtr layout(t.layout());

    try
    {
        lasReader.setOptions(las_opts);
        lasReader.prepare(t);
        lasHeader = lasReader.header();
        file_info = lasReader.preview();
    }
    catch (const pdal_error& e)
    {
        ccLog::Error(QString("PDAL exception '%1'").arg(e.what()));
        return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
    }
    catch (...)
    {
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    CCVector3d bbMin(lasHeader.minX(), lasHeader.minY(), lasHeader.minZ());
    CCVector3d bbMax(lasHeader.maxX(), lasHeader.maxY(), lasHeader.maxZ());

    CCVector3d lasScale =  CCVector3d(lasHeader.scaleX(), lasHeader.scaleY(), lasHeader.scaleZ());
    CCVector3d lasShift = -CCVector3d(lasHeader.offsetX(), lasHeader.offsetY(), lasHeader.offsetZ());

    unsigned nbOfPoints = lasHeader.pointCount();
    if (nbOfPoints == 0)
    {
        //strange file ;)
        return CC_FERR_NO_LOAD;
    }

    StringList allDims = file_info.m_dimNames;
    StringList dimensions;
    StringList extraDimensions;
    IdList extraDimensionsIds;

    for (std::string &dimName: allDims)
    {
        if (id(dimName) == Id::Unknown)
        {
            extraDimensions.push_back(dimName);
            extraDimensionsIds.push_back(layout->findProprietaryDim(dimName));
        }
        else
            dimensions.push_back(dimName);
    }

    if (!s_lasOpenDlg)
    {
        s_lasOpenDlg = QSharedPointer<LASOpenDlg>(new LASOpenDlg());
    }
    s_lasOpenDlg->setDimensions(dimensions);
    s_lasOpenDlg->clearEVLRs();
    s_lasOpenDlg->setInfos(filename, nbOfPoints, bbMin, bbMax);

    for (std::string &extraDimension: extraDimensions)
    {
        s_lasOpenDlg->addEVLR(QString("%1").arg(QString::fromStdString(extraDimension)));
    }

    if (parameters.alwaysDisplayLoadDialog && !s_lasOpenDlg->autoSkipMode() && !s_lasOpenDlg->exec())
    {
        return CC_FERR_CANCELED_BY_USER;
    }

    bool ignoreDefaultFields = s_lasOpenDlg->ignoreDefaultFieldsCheckBox->isChecked();

    unsigned short rgbColorMask[3] = {0, 0, 0};
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
	ColorCompType rgb[3] = { 0, 0, 0 };

	IdList extraFieldsToLoad;
	StringList extraNamesToLoad;
	for (unsigned i = 0; i < extraDimensionsIds.size(); ++i)
	{
		if (s_lasOpenDlg->doLoadEVLR(i)) {
			extraFieldsToLoad.push_back(extraDimensionsIds[i]);
			extraNamesToLoad.push_back(extraDimensions[i]);
		}
	}

    bool tiling = s_lasOpenDlg->tileGroupBox->isChecked();
   

    QScopedPointer<ccProgressDialog> pDlg(0);
    if (parameters.parentWidget)
    {
        pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
        pDlg->setMethodTitle(QObject::tr("Open LAS file"));
        pDlg->setInfo(QObject::tr("Points: %1").arg(nbOfPoints));
        pDlg->start();
    }

	if (tiling)
    {
		Tiler tiler;
		PointTable table;
		PointViewSet pointViewSet;

        // tiling (vertical) dimension
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

        auto w = static_cast<unsigned>(s_lasOpenDlg->wTileSpinBox->value());
        auto h = static_cast<unsigned>(s_lasOpenDlg->hTileSpinBox->value());

        QString outputBaseName = s_lasOpenDlg->outputPathLineEdit->text() + "/" + QFileInfo(filename).baseName();
        if (!tiler.init(w, h, vertDim, outputBaseName, bbMin, bbMax, table))
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }


		PointViewSet viewSet;
		PointViewPtr pointView;

		auto prepareAndExecture = [&lasReader, &table]() -> PointViewSet {
			lasReader.prepare(table);
			lasReader.prepare(table);
			return lasReader.execute(table);
		};

		QFuture<PointViewSet> reader = QtConcurrent::run(prepareAndExecture);
		if (parameters.parentWidget)
		{
			pDlg.reset(new ccProgressDialog(false, parameters.parentWidget));
			pDlg->setMethodTitle(QObject::tr("LAS file"));
			pDlg->setInfo(QObject::tr("Please wait... reading in progress"));
			pDlg->setRange(0, 0);
			pDlg->setModal(true);
			pDlg->start();
		}
		
		while (!reader.isFinished())
		{
#if defined(CC_WINDOWS)
			::Sleep(50);
#else
			usleep(500 * 100);
#endif
			if (pDlg)
			{
				pDlg->setValue(pDlg->value() + 1);
			}
			QApplication::processEvents();
		}

		viewSet = reader.result();
		pointView = *viewSet.begin();

		if (parameters.parentWidget)
		{
			pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
			pDlg->setMethodTitle(QObject::tr("Tiling points"));
			pDlg->setInfo(QObject::tr("Points: %1").arg(nbOfPoints));
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
		QFuture<void> writer = QtConcurrent::run([&tiler]() {tiler.writeAll(); });
		if (parameters.parentWidget)
		{
			pDlg.reset(new ccProgressDialog(false, parameters.parentWidget));
			pDlg->setMethodTitle(QObject::tr("LAS file"));
			pDlg->setInfo(QObject::tr("Please wait... writing in progress"));
			pDlg->setRange(0, 0);
			pDlg->setModal(true);
			pDlg->start();
		}
		while (!writer.isFinished())
		{
#if defined(CC_WINDOWS)
			::Sleep(50);
#else
			usleep(500 * 100);
#endif
			if (pDlg)
			{
				pDlg->setValue(pDlg->value() + 1);
			}
			QApplication::processEvents();
		}
		return CC_FERR_NO_ERROR;
    }

    CCLib::NormalizedProgress nProgress(pDlg.data(), nbOfPoints);
    ccPointCloud* loadedCloud = nullptr;
    std::vector< LasField::Shared > fieldsToLoad;
    CCVector3d Pshift(0, 0, 0);

    unsigned int fileChunkSize = 0;
    unsigned int nbPointsRead = 0;

    StreamCallbackFilter f;
    f.setInput(lasReader);
	
	unsigned nbOfChunks = std::max(1u, nbOfPoints / CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
	std::vector<LasCloudChunk> chunks(nbOfChunks, LasCloudChunk());

	CC_FILE_ERROR callbackError = CC_FERR_NO_ERROR;
    auto ccProcessOne = [&](PointRef& point)
    {
		if (pDlg->isCancelRequested())
			return false;

		LasCloudChunk &pointChunk = chunks[nbPointsRead / CC_MAX_NUMBER_OF_POINTS_PER_CLOUD];

		if (pointChunk.getLoadedCloud() == nullptr)
		{
			// create a new cloud
			unsigned int pointsToRead = static_cast<unsigned int>(nbOfPoints) - nbPointsRead;
			fileChunkSize = std::min(pointsToRead, CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
			if (!pointChunk.reserveSize(fileChunkSize))
			{
				ccLog::Warning("[LAS] Not enough memory!");
				callbackError = CC_FERR_NOT_ENOUGH_MEMORY;
				return false;
			}

			pointChunk.loadedCloud->setGlobalShift(Pshift);

			//save the Spatial reference as meta-data
			SpatialReference srs = lasHeader.srs();
			if (!srs.empty())
				pointChunk.loadedCloud->setMetaData(s_LAS_SRS_Key, QVariant::fromValue(srs));

			pointChunk.createFieldsToLoad(extraDimensionsIds, extraNamesToLoad);
		}

		loadedCloud = pointChunk.loadedCloud;
		fieldsToLoad = pointChunk.lasFields;

		//first point check for 'big' coordinates 
		if (nbPointsRead == 0)
		{

			CCVector3d P(static_cast<PointCoordinateType>(point.getFieldAs<int>(Id::X)),
						 static_cast<PointCoordinateType>(point.getFieldAs<int>(Id::Y)),
						 static_cast<PointCoordinateType>(point.getFieldAs<int>(Id::Z)));
			//backup input global parameters 
			ccGlobalShiftManager::Mode csBackupMode = parameters.shiftHandlingMode;
			bool useLasShift = false;
			//set the lasShift as default if none was provided 
			if (lasShift.norm2() != 0 && (!parameters.coordinatesShiftEnabled || !*parameters.coordinatesShiftEnabled))
			{
				useLasShift = true;
				Pshift = lasShift;

				if (csBackupMode != ccGlobalShiftManager::Mode::NO_DIALOG_AUTO_SHIFT &&
					csBackupMode != ccGlobalShiftManager::Mode::NO_DIALOG)
				{
					parameters.shiftHandlingMode = ccGlobalShiftManager::Mode::ALWAYS_DISPLAY_DIALOG;
				}
			}

			if (HandleGlobalShift(P, Pshift, parameters, useLasShift))
			{
				loadedCloud->setGlobalShift(Pshift);
				ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
			}

			//restore previous parameters
			parameters.shiftHandlingMode = csBackupMode;
		}

        CCVector3 P(static_cast<PointCoordinateType>(point.getFieldAs<double>(Id::X) + Pshift.x),
                    static_cast<PointCoordinateType>(point.getFieldAs<double>(Id::Y) + Pshift.y),
                    static_cast<PointCoordinateType>(point.getFieldAs<double>(Id::Z) + Pshift.z));
        loadedCloud->addPoint(P);


        if (loadColor)
        {
            unsigned short red = point.getFieldAs<unsigned short>(Id::Red) & rgbColorMask[0];
            unsigned short green = point.getFieldAs<unsigned short>(Id::Green) & rgbColorMask[1];
            unsigned short blue = point.getFieldAs<unsigned short>(Id::Blue) & rgbColorMask[2];

            // if we don't have reserved a color field yet, we check that color is not black
            bool pushColor = true;
            if (!loadedCloud->hasColors())
            {
                if (red || green || blue)
                {
                    if (loadedCloud->reserveTheRGBTable())
                    {
                        // we must set the color (black) of all previously skipped points
                        for (unsigned i = 0; i < loadedCloud->size() - 1; ++i)
                        {
                            loadedCloud->addRGBColor(ccColor::black.rgba);
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
                           || (green  & 0xFF00)
                           || (blue & 0xFF00))
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
                rgb[0] = static_cast<ColorCompType>(red   >> colorCompBitShift);
                rgb[1] = static_cast<ColorCompType>(green >> colorCompBitShift);
                rgb[2] = static_cast<ColorCompType>(blue  >> colorCompBitShift);

                loadedCloud->addRGBColor(rgb);
            }
        }

        // additional fields
        for (auto &field : fieldsToLoad) {

            double value = 0.0;
            Id pdalId = typeToId(field->type);

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
            case LAS_CLASSIF_SYNTHETIC:
                value = (point.getFieldAs<int>(pdalId) & 1); //bit #1
                break;
            case LAS_CLASSIF_KEYPOINT:
                value = (point.getFieldAs<int>(pdalId) & 2); //bit #2
                break;
            case LAS_CLASSIF_WITHHELD:
                value = (point.getFieldAs<int>(pdalId) & 4); //bit #3
                break;
                // Overlap flag is the 4 bit (new in las 1.4)
            default:
                value = point.getFieldAs<double>(pdalId);
                break;
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

                        for (unsigned i = 0; i < loadedCloud->size() - 1; ++i) {
                            field->sf->addElement(static_cast<ScalarType>(field->defaultValue));
                        }
                        auto s = static_cast<ScalarType>(value);
                        field->sf->addElement(s);
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
    f.prepare(t);
    f.execute(t);

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
				unsigned n = container.getChildrenNumber();
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


    return CC_FERR_NO_ERROR;
}

#endif
