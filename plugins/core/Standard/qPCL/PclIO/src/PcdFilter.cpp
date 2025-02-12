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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "PcdFilter.h"

//PclUtils
#include <sm2cc.h>
#include <cc2sm.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>

//Qt
#include <QFileInfo>
#include <QDialog>
#include <QPushButton>

//Dialog
#include <ui_PCDOutputFormatDlg.h>

//Boost
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

//System
#include <iostream>
#include <fstream>

//pcl
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

PcdFilter::PcdFilter()
    : FileIOFilter( {
                    "_Point Cloud Library Filter",
                    13.0f,	// priority
                    QStringList{ "pcd" },
                    "pcd",
                    QStringList{ "Point Cloud Library cloud (*.pcd)" },
                    QStringList{ "Point Cloud Library cloud (*.pcd)" },
                    Import | Export
                    } )
{
}

bool PcdFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//only one cloud per file
	if (type == CC_TYPES::POINT_CLOUD)
	{
		multiple = false;
		exclusive = true;
		return true;
	}

	return false;
}

static PcdFilter::PCDOutputFileFormat s_outputFileFormat = PcdFilter::AUTO;
void PcdFilter::SetOutputFileFormat(PCDOutputFileFormat format)
{
	s_outputFileFormat = format;
}

//! Dialog for the PCV plugin
class ccPCDFileOutputForamtDialog : public QDialog, public Ui::PCDOutputFormatDialog
{
public:
	explicit ccPCDFileOutputForamtDialog(QWidget* parent = nullptr)
		: QDialog(parent, Qt::Tool)
		, Ui::PCDOutputFormatDialog()
	{
		setupUi(this);
	}
};

CC_FILE_ERROR PcdFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	if (!entity || filename.isEmpty())
	{
		return CC_FERR_BAD_ARGUMENT;
	}

	PcdFilter::PCDOutputFileFormat outputFileFormat = s_outputFileFormat;
	if (outputFileFormat == AUTO)
	{
		if (nullptr == parameters.parentWidget)
		{
			// defaulting to compressed binary
			outputFileFormat = COMPRESSED_BINARY;
		}
		else
		{
			// GUI version: show a dialog to let the user choose the output format
			ccPCDFileOutputForamtDialog dialog;
			static PcdFilter::PCDOutputFileFormat s_previousOutputFileFormat = COMPRESSED_BINARY;
			switch (s_previousOutputFileFormat)
			{
			case COMPRESSED_BINARY:
				dialog.compressedBinaryRadioButton->setChecked(true);
				break;
			case BINARY:
				dialog.binaryRadioButton->setChecked(true);
				break;
			case ASCII:
				dialog.asciiRadioButton->setChecked(true);
				break;
			default:
				assert(false);
				break;
			}

			QAbstractButton* clickedButton = nullptr;

			QObject::connect(dialog.buttonBox, &QDialogButtonBox::clicked, [&](QAbstractButton* button)
				{
					clickedButton = button;
				});

			if (dialog.exec())
			{
				if (dialog.compressedBinaryRadioButton->isChecked())
				{
					outputFileFormat = COMPRESSED_BINARY;
				}
				else if (dialog.binaryRadioButton->isChecked())
				{
					outputFileFormat = BINARY;
				}
				else if (dialog.asciiRadioButton->isChecked())
				{
					outputFileFormat = ASCII;
				}
				else
				{
					assert(false);
				}

				s_previousOutputFileFormat = outputFileFormat;

				if (clickedButton == dialog.buttonBox->button(QDialogButtonBox::StandardButton::YesToAll))
				{
					// remember once and for all the output file format
					s_outputFileFormat = outputFileFormat;
				}
			}
			else
			{
				return CC_FERR_CANCELED_BY_USER;
			}
		}
	}

	//the cloud to save
	ccPointCloud* ccCloud = ccHObjectCaster::ToPointCloud(entity);
	if (!ccCloud)
	{
		ccLog::Warning("[PCL] This filter can only save one cloud at a time!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//search for a sensor as child (we take the first if there are several of them)
	ccSensor* sensor = nullptr;
	{
		for (unsigned i = 0; i < ccCloud->getChildrenNumber(); ++i)
		{
			ccHObject* child = ccCloud->getChild(i);

			//try to cast to a ccSensor
			sensor = ccHObjectCaster::ToSensor(child);
			if (sensor)
			{
				break;
			}
		}
	}

	const CCVector3d& globalShift = ccCloud->getGlobalShift();

	if (ccCloud->getGlobalScale() != 1.0)
	{
		ccLog::Warning("[PCD] Can't restore the Global scale when exporting to PCD format");
	}

	Eigen::Vector4f pos = Eigen::Vector4f::Zero();
	Eigen::Quaternionf ori = Eigen::Quaternionf::Identity();
	PCLCloud::Ptr pclCloud;

	if (sensor)
	{
		//get sensor data
		ccGLMatrix sensorMatrix = sensor->getRigidTransformation();

		//translation
		CCVector3 trans = sensorMatrix.getTranslationAsVec3D();

		//check that the sensor is not too far from the points
		CCVector3 bbMin, bbMax;
		ccCloud->getBoundingBox(bbMin, bbMax);
		CCVector3 bbCenter = (bbMin + bbMax) / 2.0;
		if (!ccGlobalShiftManager::NeedShift(trans - bbCenter))
		{
			pos(0) = trans.x;
			pos(1) = trans.y;
			pos(2) = trans.z;

			//rotation
			Eigen::Matrix3f eigrot;
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					eigrot(i, j) = sensorMatrix.getColumn(j)[i];

			//now translate to a quaternion
			ori = Eigen::Quaternionf(eigrot).normalized(); // need to normalize the output in case the input matrix is not purely 'orthogonal'

			//it happens that the saved pose matrix doesn't capture everything (such as reflections)
			//because of the quaternion form. Therefore we update the input 'poseMat' with what we
			//have actually saved
			{
				CCCoreLib::SquareMatrix transMat(sensorMatrix.data(), true);
				if (transMat.computeDet() < 0)
				{
					ccLog::Warning("[PCD] Sensor pose matrix seems to be a reflection. Can't save it as a quaternion (required by the PCD format). We will use the non-reflective form.");
				}

				eigrot = ori.toRotationMatrix();
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < 3; ++j)
					{
						sensorMatrix.getColumn(j)[i] = eigrot(i, j);
					}
				}
			}

			// we have to project the cloud to the sensor CS
			ccPointCloud* tempCloud = ccCloud->cloneThis(nullptr, true);
			if (!tempCloud)
			{
				return CC_FILE_ERROR::CC_FERR_NOT_ENOUGH_MEMORY;
			}
			tempCloud->applyRigidTransformation(sensorMatrix.inverse());

			pclCloud = cc2smReader(tempCloud).getAsSM();

			pos(0) = static_cast<float>(pos(0) - globalShift.x);
			pos(1) = static_cast<float>(pos(1) - globalShift.y);
			pos(2) = static_cast<float>(pos(2) - globalShift.z);

			delete tempCloud;
			tempCloud = nullptr;
		}
		else
		{
			ccLog::Warning("Sensor center is too far from the points. Can't restore it without losing numerical accuracy...");
		}
	}

	if (!pclCloud)
	{
		pclCloud = cc2smReader(ccCloud).getAsSM();

		pos(0) = -static_cast<float>(globalShift.x);
		pos(1) = -static_cast<float>(globalShift.y);
		pos(2) = -static_cast<float>(globalShift.z);
	}
	
	if (!pclCloud)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (ccCloud->size() == 0)
	{
		//Specific case: empty cloud (header only)
		pcl::PCDWriter p;
		QFile file(filename);
		if (!file.open(QFile::WriteOnly | QFile::Truncate))
			return CC_FERR_WRITING;
		QTextStream stream(&file);

		stream << QString(p.generateHeaderBinary(*pclCloud, pos, ori).c_str()) << "DATA binary\n";
		return CC_FERR_NO_ERROR;
	}

	try
	{
		pcl::PCDWriter p;

		switch (outputFileFormat)
		{
		case COMPRESSED_BINARY:
		{
#ifdef _MSC_VER
			std::ofstream ofs(filename.toStdWString(), std::wifstream::out | std::wifstream::binary);
#else
			std::ofstream ofs(filename.toStdString(), std::wifstream::out | std::wifstream::binary);
#endif
			if (p.writeBinaryCompressed(ofs, *pclCloud, pos, ori) < 0)
			{
				ofs.close();
				return CC_FERR_THIRD_PARTY_LIB_FAILURE;
			}
			ofs.close();
		}
		break;

		case BINARY:
		{
			if (p.writeBinary(filename.toStdString(), *pclCloud, pos, ori) < 0)
			{
				return CC_FERR_THIRD_PARTY_LIB_FAILURE;
			}
		}
		break;

		case ASCII:
		{
			if (p.writeASCII(filename.toStdString(), *pclCloud, pos, ori) < 0)
			{
				return CC_FERR_THIRD_PARTY_LIB_FAILURE;
			}
		}
		break;

		default:
			assert(false);
			break;
		}
	}
	catch (...)
	{
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return CC_FERR_NO_ERROR;
}

//! Helper: returns the field index (or -1 if not found)
static int FieldIndex(const PCLCloud& pclCloud, std::string fieldName)
{
	for (size_t index = 0; index < pclCloud.fields.size(); ++index)
		if (pclCloud.fields[index].name == fieldName)
			return static_cast<int>(index);

	return -1;
}

static double ReadValue(const uint8_t* data, const pcl::PCLPointField& field)
{
	switch (field.datatype)
	{
	case pcl::PCLPointField::INT8:
		return static_cast<double>(*(reinterpret_cast<const int8_t*>(data)));
	case pcl::PCLPointField::UINT8:
		return static_cast<double>(*(reinterpret_cast<const uint8_t*>(data)));
	case pcl::PCLPointField::INT16:
		return static_cast<double>(*(reinterpret_cast<const int16_t*>(data)));
	case pcl::PCLPointField::UINT16:
		return static_cast<double>(*(reinterpret_cast<const uint16_t*>(data)));
	case pcl::PCLPointField::INT32:
		return static_cast<double>(*(reinterpret_cast<const int32_t*>(data)));
	case pcl::PCLPointField::UINT32:
		return static_cast<double>(*(reinterpret_cast<const uint32_t*>(data)));
	case pcl::PCLPointField::FLOAT32:
		return *(reinterpret_cast<const float*>(data));
	case pcl::PCLPointField::FLOAT64:
		return *(reinterpret_cast<const double*>(data));
	default:
		assert(false);
		return std::numeric_limits<double>::quiet_NaN();
	}
}

CC_FILE_ERROR PcdFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	try
	{
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcdVersion = 0;
		int dataType = 0;
		unsigned int cloudDataIndex = 0;
		PCLCloud::Ptr inputCloud(new PCLCloud);

		//To avoid issues with local characters, we will open the file 'manually' and then provide the contents to PCL directly
#ifdef _MSC_VER
		std::ifstream ifs(filename.toStdWString(), std::wifstream::in | std::wifstream::binary);
#else
		std::ifstream ifs(filename.toStdString(), std::wifstream::in | std::wifstream::binary);
#endif

		//Load the input file
		pcl::PCDReader pcdReader;
		if (pcdReader.readHeader(ifs, *inputCloud, origin, orientation, pcdVersion, dataType, cloudDataIndex) < 0)
		{
			return CC_FERR_THIRD_PARTY_LIB_FAILURE;
		}
		ccLog::Print(QString("[PCL] PCD version: %1").arg(pcdVersion));

		unsigned pointCount = static_cast<unsigned>(inputCloud->width * inputCloud->height);
		ccLog::Print(QString("[PCL] %1: %2 points").arg(filename).arg(pointCount));
		if (pointCount == 0)
		{
			return CC_FERR_NO_LOAD;
		}

		if (dataType == 0) // ASCII format
		{
			ccLog::Print(QObject::tr("[PCL] Reading ASCII PCD file..."));
			ifs.seekg(cloudDataIndex, std::ios::beg);
			if (pcdReader.readBodyASCII(ifs, *inputCloud, pcdVersion) < 0) //DGM: warning, toStdString doesn't preserve "local" characters
			{
				return CC_FERR_THIRD_PARTY_LIB_FAILURE;
			}
		}
		else // Binary
		{
			ccLog::Print(QObject::tr("[PCL] Reading %1 binary PCD file...").arg(dataType == 2 ? QObject::tr("compressed") : QObject::tr("uncompressed")));

			std::vector<char> buffer;
			try
			{
				qint64 fileSize = QFileInfo(filename).size();
				buffer.resize(fileSize);

				ifs.seekg(0, std::ios::beg);
				ifs.read(buffer.data(), fileSize);
				ifs.close();

				if (pcdReader.readBodyBinary(reinterpret_cast<unsigned char*>(buffer.data()), *inputCloud, pcdVersion, dataType == 2, cloudDataIndex) < 0)
				{
					return CC_FERR_THIRD_PARTY_LIB_FAILURE;
				}
			}
			catch (const std::bad_alloc&)
			{
				// try to read the file 'normally' in this case
				if (pcdReader.read(qPrintable(filename), *inputCloud) < 0)
				{
					return CC_FERR_NOT_ENOUGH_MEMORY;
				}
			}
		}

		ccPointCloud::Grid::Shared grid;
		if (inputCloud->width > 1 && inputCloud->height > 1) // structured cloud?
		{
			try
			{
				if (inputCloud->is_dense)
				{
					grid.reset(new ccPointCloud::Grid);
					grid->h = static_cast<unsigned>(inputCloud->height);
					grid->w = static_cast<unsigned>(inputCloud->width);
					grid->indexes.resize(static_cast<size_t>(grid->h) * grid->w);
					grid->minValidIndex = 0;
					grid->maxValidIndex = pointCount;
					grid->validCount = pointCount;

					for (size_t i = 0; i < pointCount; ++i)
					{
						grid->indexes[i] = static_cast<int>(i);
					}
				}
				else
				{
					int xIndex = FieldIndex(*inputCloud, "x");
					int yIndex = FieldIndex(*inputCloud, "y");
					int zIndex = FieldIndex(*inputCloud, "z");

					if (xIndex >= 0 && yIndex >= 0 && zIndex >= 0)
					{
						grid.reset(new ccPointCloud::Grid);
						grid->h = static_cast<unsigned>(inputCloud->height);
						grid->w = static_cast<unsigned>(inputCloud->width);
						grid->indexes.resize(static_cast<size_t>(grid->h) * grid->w, -1);
						grid->validCount = 0;
						grid->minValidIndex = pointCount;
						grid->maxValidIndex = 0;

						const pcl::PCLPointField& fieldX = inputCloud->fields[xIndex];
						const pcl::PCLPointField& fieldY = inputCloud->fields[yIndex];
						const pcl::PCLPointField& fieldZ = inputCloud->fields[zIndex];

						// Determine the offsets to the X, Y and Z coordinates
						const uint8_t* dataX = inputCloud->data.data() + fieldX.offset;
						const uint8_t* dataY = inputCloud->data.data() + fieldY.offset;
						const uint8_t* dataZ = inputCloud->data.data() + fieldZ.offset;

						// reproduce the process of pcl::PassThrough (see below)
						for (unsigned i = 0; i < pointCount; ++i)
						{
							double x = ReadValue(dataX, fieldX);
							double y = ReadValue(dataY, fieldY);
							double z = ReadValue(dataZ, fieldZ);

							dataX += inputCloud->point_step;
							dataY += inputCloud->point_step;
							dataZ += inputCloud->point_step;

							// Check if the point is invalid
							if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
							{
								// valid index
								grid->indexes[i] = static_cast<int>(grid->validCount);
								++grid->validCount;

								grid->minValidIndex = std::min(i, grid->minValidIndex);
								grid->maxValidIndex = std::max(i, grid->maxValidIndex);
							}
						}
					}
					else
					{
						ccLog::Warning("Could not find the 'x', 'y' or 'z' field. Can't create the scan grid.");
					}
				}
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning("Not enough memory to load the scan grid");
			}
		}

		if (!inputCloud->is_dense) //data may contain NaNs
		{
			//we need to remove NaNs
			pcl::PassThrough<PCLCloud> passFilter;
			passFilter.setInputCloud(inputCloud);
			passFilter.filter(*inputCloud);
		}

		// get orientation as rot matrix and copy it into a ccGLMatrix
		ccGLMatrixd ccRot;
		{
			Eigen::Matrix3f eigRot = orientation.toRotationMatrix();
			double* X = ccRot.getColumn(0);
			double* Y = ccRot.getColumn(1);
			double* Z = ccRot.getColumn(2);

			X[0] = eigRot(0, 0); X[1] = eigRot(1, 0); X[2] = eigRot(2, 0);
			Y[0] = eigRot(0, 1); Y[1] = eigRot(1, 1); Y[2] = eigRot(2, 1);
			Z[0] = eigRot(0, 2); Z[1] = eigRot(1, 2); Z[2] = eigRot(2, 2);

			ccRot.getColumn(3)[3] = 1.0;
			ccRot.setTranslation(origin.data());
		}

		//convert to CC cloud
		// DGM: it appears that the sensor orientation/translation has to be applied manually to the points
		// (probably because they just correspond to the raw scan grid in the general case)
		ccPointCloud* ccCloud = pcl2cc::Convert(*inputCloud, &ccRot, &parameters);
		if (!ccCloud)
		{
			ccLog::Warning("[PCL] An error occurred while converting PCD cloud to CloudCompare cloud!");
			return CC_FERR_CONSOLE_ERROR;
		}
		
		ccCloud->setName(QStringLiteral("unnamed"));
		if (grid)
		{
			if (grid->validCount != static_cast<unsigned>(inputCloud->width * inputCloud->height))
			{
				ccLog::Warning("[PCL] Inconsistency detected between the reconstructed scan grid and the converted cloud");
			}
			else
			{
				ccCloud->addGrid(grid);
			}
		}

		//now we can construct a ccGBLSensor
		{
			if (grid)
			{
				grid->sensorPosition = ccRot;
			}

			ccGBLSensor* sensor = new ccGBLSensor;
			sensor->setRigidTransformation(ccGLMatrix(ccRot.data()));
			sensor->setYawStep(static_cast<PointCoordinateType>(0.05));
			sensor->setPitchStep(static_cast<PointCoordinateType>(0.05));
			sensor->setVisible(true);
			//uncertainty to some default
			sensor->setUncertainty(static_cast<PointCoordinateType>(0.01));
			//graphic scale
			sensor->setGraphicScale(ccCloud->getOwnBB().getDiagNorm() / 10);

			//Compute parameters
			ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(ccCloud);
			sensor->computeAutoParameters(pc);

			sensor->setEnabled(false);

			ccCloud->addChild(sensor);
		}

		container.addChild(ccCloud);
	}
	catch (const std::bad_alloc&)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	return CC_FERR_NO_ERROR;
}
