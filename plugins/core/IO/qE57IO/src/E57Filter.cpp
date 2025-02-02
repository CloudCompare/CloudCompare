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

#include "E57Filter.h"
#include "FileIO.h"

//Local
#include "E57Header.h"
#include "E57Version.h"

//libE57Format
#include <E57Format.h>

//qCC_db
#include <ccCameraSensor.h>
#include <ccColorScalesManager.h>
#include <ccGBLSensor.h>
#include <ccImage.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Qt
#include <QApplication>
#include <QBuffer>
#include <QUuid>

//system
#include <cassert>
#include <string>

using colorFieldType = double;

namespace
{
	constexpr char CC_E57_INTENSITY_FIELD_NAME[] = "Intensity";
	constexpr char CC_E57_RETURN_INDEX_FIELD_NAME[] = "Return index";
	constexpr char s_e57PoseKey[] = "E57_pose";
	constexpr char s_e57NodeInfoKey[] = "E57_node_info";
	constexpr char s_e57CameraInfoKey[] = "E57_camera_representation";

	constexpr uint8_t VALID_DATA = 0;
	constexpr uint8_t INVALID_DATA = 1;

	unsigned s_absoluteScanIndex = 0;
	bool s_cancelRequestedByUser = false;
	
	ScalarType s_maxIntensity = 0;
	ScalarType s_minIntensity = 0;
	
	//for coordinate shift handling
	FileIOFilter::LoadParameters s_loadParameters;
	
	//Array chunks for reading/writing information out of E57 files
	struct TempArrays
	{
		//points
		std::vector<double> xData;
		std::vector<double> yData;
		std::vector<double> zData;
		std::vector<int8_t> isInvalidData;
	
		//normals
		std::vector<double> xNormData;
		std::vector<double> yNormData;
		std::vector<double> zNormData;
	
		//scalar field
		std::vector<double>	intData;
		std::vector<int8_t> isInvalidIntData;
	
		//scan index field
		std::vector<int8_t> scanIndexData;

		//scan grid
		std::vector<int32_t> rowIndex;
		std::vector<int32_t> columnIndex;

		//color
		std::vector<colorFieldType> redData;
		std::vector<colorFieldType> greenData;
		std::vector<colorFieldType> blueData;
	};
	
	inline QString GetNewGuid()
	{
		return QUuid::createUuid().toString();
	}
}

QString GetStringFromNode(const e57::StructureNode& node, const char* fieldName, const QString& defaultValue)
{
	if (node.isDefined(fieldName))
	{
		return QString::fromStdString(e57::StringNode(node.get(fieldName)).value());
	}
	else
	{
		return defaultValue;
	}
}

void AddToE57NodeMap(E57NodeMap& nodeMap, const e57::StructureNode& node, const char* fieldName)
{
	E57NodeDesc nodeDesc;
	if (node.isDefined(fieldName))
	{
		e57::Node childNode = node.get(fieldName);
		switch (childNode.type())
		{
		case e57::TypeInteger:
			nodeDesc.type = "INT";
			nodeDesc.content = QString::number(e57::IntegerNode(node.get(fieldName)).value());
			break;
		case e57::TypeFloat:
			nodeDesc.type = "FLT";
			nodeDesc.content = QString::number(e57::FloatNode(node.get(fieldName)).value(), 'f', 12);
			break;
		case e57::TypeString:
			nodeDesc.type = "STR";
			nodeDesc.content = QString::fromStdString(e57::StringNode(node.get(fieldName)).value());
			break;
		default:
			//unhandled node type
			return;
		}

		nodeMap.insert(fieldName, nodeDesc);
	}
}

E57Filter::E57Filter()
    : FileIOFilter( {
                    "_E57 Filter",
					4.0f,	// priority
                    QStringList{ "e57" },
                    "e57",
                    QStringList{ "E57 cloud (*.e57)" },
                    QStringList{ "E57 cloud (*.e57)" },
                    Import | Export
                    } )
{
}

bool E57Filter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POINT_CLOUD)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

//Helper: save pose information
static void SavePoseInformation(e57::StructureNode& parentNode,
								const e57::ImageFile& imf,
								ccGLMatrixd& poseMat,
								const CCVector3d& globalShift)
{
	e57::StructureNode pose = e57::StructureNode(imf);
	parentNode.set("pose", pose);

	CCCoreLib::SquareMatrixd transMat(poseMat.data(), true);
	if (transMat.computeDet() < 0)
	{
		ccLog::Warning("[E57] Pose matrix seems to be a reflection. Can't save it as a quaternion (required by the E57 format). We will use the non-reflective form.");
	}

	double q[4];
	if (transMat.toQuaternion(q))
	{
		e57::StructureNode rotation = e57::StructureNode(imf);
		rotation.set("w", e57::FloatNode(imf, q[0]));
		rotation.set("x", e57::FloatNode(imf, q[1]));
		rotation.set("y", e57::FloatNode(imf, q[2]));
		rotation.set("z", e57::FloatNode(imf, q[3]));
		pose.set("rotation", rotation);
	}

	//translation
	CCVector3d tr = poseMat.getTranslationAsVec3D();
	{
		e57::StructureNode translation = e57::StructureNode(imf);
		translation.set("x", e57::FloatNode(imf, tr.x - globalShift.x));
		translation.set("y", e57::FloatNode(imf, tr.y - globalShift.y));
		translation.set("z", e57::FloatNode(imf, tr.z - globalShift.z));
		pose.set("translation", translation);
	}

	//it happens that the saved pose matrix doesn't capture everything (such as reflections)
	//because of the quaternion form. Therefore we update the input 'poseMat' with what we
	//have actually saved
	poseMat = ccGLMatrixd::FromQuaternion(q);
	poseMat.setTranslation(tr);
}

static void SaveNodeInfo(	const E57NodeMap& nodeMap,
							e57::StructureNode& outputNode,
							e57::ImageFile& imf )
{
	for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
	{
		if (!it.value().content.isEmpty())
		{
			if (it.value().type == "STR")
			{
				outputNode.set(it.key().toStdString(), e57::StringNode(imf, it.value().content.toStdString()));
			}
			else if (it.value().type == "FLT")
			{
				bool ok = false;
				double value = it.value().content.toDouble(&ok);
				if (ok)
				{
					outputNode.set(it.key().toStdString(), e57::FloatNode(imf, value));
				}
				else
				{
					ccLog::Warning("[E57] Invalid value in meta-data for field " + it.key());
				}
			}
			else if (it.value().type == "INT")
			{
				bool ok = false;
				int64_t value = it.value().content.toLongLong(&ok);
				if (ok)
				{
					outputNode.set(it.key().toStdString(), e57::IntegerNode(imf, value));
				}
				else
				{
					ccLog::Warning("[E57] Invalid value in meta-data for field " + it.key());
				}
			}
			else
			{
				ccLog::Warning(QString("[E57] Unkown E57 type %1 in meta-data for field %2").arg(it.value().type).arg(it.key()));
			}
		}
	}
}

static bool SaveScan(	ccPointCloud* cloud,
						e57::StructureNode& scanNode,
						e57::ImageFile& imf,
						e57::VectorNode& data3D,
						QString& guidStr,
						ccProgressDialog* progressDlg = nullptr )
{
	assert(cloud);

	unsigned pointCount = cloud->size();
	if (pointCount == 0)
	{
		ccLog::Error(QString("[E57Filter::SaveScan] Cloud '%1' is empty!").arg(cloud->getName()));
		return false;
	}

	//Restore scan (node) information if any
	if (cloud->hasMetaData(s_e57NodeInfoKey))
	{
		QStringList stringList = cloud->getMetaData(s_e57NodeInfoKey).toString().split("\n");

		E57NodeMap nodeMap;
		E57NodeMap::FromStringList(nodeMap, stringList);
		SaveNodeInfo(nodeMap, scanNode, imf);
	}

	ccGLMatrixd toSensorCS; // 'Sensor' output coordinate system (no Global Shift applied)
	toSensorCS.toIdentity();
	bool hasSensorPoseMat = false;

	double globalScale = 1.0;
	bool isScaled = false;
	{
		globalScale = cloud->getGlobalScale();
		assert(globalScale != 0);
		isScaled = (globalScale != 1.0);

		//restore original sensor pose (if any)
		QString poseStr = cloud->getMetaData(s_e57PoseKey).toString();
		if (!poseStr.isEmpty())
		{
			toSensorCS = ccGLMatrixd::FromString(poseStr, hasSensorPoseMat);
			if (hasSensorPoseMat)
			{
				//apply transformation history (i.e. transformations applied after the entity has been loaded)
				//--> we assume they have been implicitly applied to the sensor as well
				ccGLMatrixd historyTransform(cloud->getGLTransformationHistory().data());
				toSensorCS = historyTransform * toSensorCS;
			}
			else
			{
				ccLog::Warning("[E57Filter::saveFile] Sensor pose meta-data is invalid");
				toSensorCS.toIdentity();
			}
		}
		else
		{
			// look for a sensor
			ccHObject::Container sensors;
			cloud->filterChildren(sensors, false, CC_TYPES::GBL_SENSOR, false);
			if (sensors.size() != 0)
			{
				hasSensorPoseMat = true;
				toSensorCS = ccGLMatrixd(sensors.front()->getGLTransformationHistory().data());
			}
		}

		//we eventually add the global shift to the E57 file pose matrix
		CCVector3d globalShift = cloud->getGlobalShift();
		if (globalShift.norm2d() != 0)
		{
			//add the Global Shift to the 'pose' matrix
			SavePoseInformation(scanNode, imf, toSensorCS, globalShift);
		}
		else if (hasSensorPoseMat)
		{
			//simply save the sensor pose matrix
			SavePoseInformation(scanNode, imf, toSensorCS, {});
		}
	}

	ccGLMatrix fromSensorToLocalCS;
	if (hasSensorPoseMat)
	{
		//inverse transformation: from the sensor output CS to the local/input CS
		fromSensorToLocalCS = ccGLMatrix(toSensorCS.inverse().data());
	}

	CCVector3d bbMin;
	CCVector3d bbMax;
	if (hasSensorPoseMat)
	{
		//we have to compute the rotated cloud bounding-box!
		for (unsigned i = 0; i < pointCount; ++i)
		{
			//we apply the Global Scale but not the Global Shift (already incorporated in the 'pose' matrix above)
			CCVector3d Psensor = cloud->getPointPersistentPtr(i)->toDouble() / globalScale;

			//DGM: according to E57 specifications, the bounding-box is local
			//(i.e. in the sensor 'input' coordinate system)
			CCVector3d Plocal = fromSensorToLocalCS * Psensor;

			if (i != 0)
			{
				bbMin.x = std::min(bbMin.x, Plocal.x);
				bbMin.y = std::min(bbMin.y, Plocal.y);
				bbMin.z = std::min(bbMin.z, Plocal.z);
				bbMax.x = std::max(bbMax.x, Plocal.x);
				bbMax.y = std::max(bbMax.y, Plocal.y);
				bbMax.z = std::max(bbMax.z, Plocal.z);
			}
			else
			{
				bbMax.x = bbMin.x = Plocal.x;
				bbMax.y = bbMin.y = Plocal.y;
				bbMax.z = bbMin.z = Plocal.z;
			}
		}
	}
	else
	{
		if (!cloud->getOwnGlobalBB(bbMin, bbMax))
		{
			ccLog::Error(QString("[E57Filter::SaveScan] Internal error: cloud '%1' has an invalid bounding box?!").arg(cloud->getName()));
			return false;
		}
	}

	//GUID
	scanNode.set("guid", e57::StringNode(imf, guidStr.toStdString()));	//required

	//Name
	if (!cloud->getName().isEmpty())
		scanNode.set("name", e57::StringNode(imf, cloud->getName().toStdString()));
	else
		scanNode.set("name", e57::StringNode(imf, QString("Scan %1").arg(s_absoluteScanIndex).toStdString()));

	//Description
	scanNode.set("description", e57::StringNode(imf, FileIO::createdBy().toStdString()) );

	//Original GUIDs (TODO)
	//if (originalGuids.size() > 0 )
	//{
	//	scanNode.set("originalGuids", e57::VectorNode(imf));
	//	e57::VectorNode originalGuids(scanNode.get("originalGuids"));
	//	for(unsigned i = 0; i < data3DHeader.originalGuids.size(); i++)
	//		originalGuids.append(e57::StringNode(imf,originalGuids[i]));
	//}

	// Add various sensor and version strings to scan (TODO)
	//scan.set("sensorVendor",			e57::StringNode(imf,sensorVendor));
	//scan.set("sensorModel",				e57::StringNode(imf,sensorModel));
	//scan.set("sensorSerialNumber",		e57::StringNode(imf,sensorSerialNumber));
	//scan.set("sensorHardwareVersion",	e57::StringNode(imf,sensorHardwareVersion));
	//scan.set("sensorSoftwareVersion",	e57::StringNode(imf,sensorSoftwareVersion));
	//scan.set("sensorFirmwareVersion",	e57::StringNode(imf,sensorFirmwareVersion));

	// Add temp/humidity to scan (TODO)
	//scanNode.set("temperature",			e57::FloatNode(imf,temperature));
	//scanNode.set("relativeHumidity",		e57::FloatNode(imf,relativeHumidity));
	//scanNode.set("atmosphericPressure",	e57::FloatNode(imf,atmosphericPressure));

	// No index bounds for unstructured clouds!
	// But we can still have multiple return indexes
	ccScalarField* returnIndexSF = nullptr;
	int minReturnIndex = 0;
	int maxReturnIndex = 0;
	{
		int returnIndexSFIndex = cloud->getScalarFieldIndexByName(CC_E57_RETURN_INDEX_FIELD_NAME);
		if (returnIndexSFIndex >= 0)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(returnIndexSFIndex));
			assert(sf);

			assert(sf->getMin() >= 0);
			{
				//get min and max index
				double minIndex = static_cast<double>(sf->getMin());
				double maxIndex = static_cast<double>(sf->getMax());

				double intMin = 0.0;
				double intMax = 0.0;
				double fracMin = modf(minIndex, &intMin);
				double fracMax = modf(maxIndex, &intMax);

				if (fracMin == 0 && fracMax == 0 && static_cast<int>(intMax-intMin) < 256)
				{
					int minScanIndex = static_cast<int>(intMin);
					int maxScanIndex = static_cast<int>(intMax);
					
					minReturnIndex = minScanIndex;
					maxReturnIndex = maxScanIndex;
					if (maxReturnIndex>minReturnIndex)
					{
						returnIndexSF = sf;

						//DGM FIXME: should we really save this for an unstructured point cloud?
						e57::StructureNode ibox = e57::StructureNode(imf);
						ibox.set("rowMinimum",		e57::IntegerNode(imf, 0));
						ibox.set("rowMaximum",		e57::IntegerNode(imf, cloud->size() - 1));
						ibox.set("columnMinimum",	e57::IntegerNode(imf, 0));
						ibox.set("columnMaximum",	e57::IntegerNode(imf, 0));
						ibox.set("returnMinimum",	e57::IntegerNode(imf, minReturnIndex));
						ibox.set("returnMaximum",	e57::IntegerNode(imf, maxReturnIndex));
						scanNode.set("indexBounds", ibox);
					}
				}
			}
		}
	}

	//Intensity
	ccScalarField* intensitySF = nullptr;
	bool hasInvalidIntensities = false;
	{
		int intensitySFIndex = cloud->getScalarFieldIndexByName(CC_E57_INTENSITY_FIELD_NAME);
		if (intensitySFIndex < 0)
		{
			intensitySFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
			if (intensitySFIndex >= 0)
				ccLog::Print("[E57] No 'intensity' scalar field found, we'll use the currently displayed one instead (%s)", cloud->getScalarFieldName(intensitySFIndex).c_str());
		}
		if (intensitySFIndex >= 0)
		{
			intensitySF = static_cast<ccScalarField*>(cloud->getScalarField(intensitySFIndex));
			assert(intensitySF);

			e57::StructureNode intbox = e57::StructureNode(imf);
			intbox.set("intensityMinimum", e57::FloatNode(imf, intensitySF->getMin()));
			intbox.set("intensityMaximum", e57::FloatNode(imf, intensitySF->getMax()));
			scanNode.set("intensityLimits", intbox);

			//look for 'invalid' scalar values
			for (unsigned i = 0; i < intensitySF->currentSize(); ++i)
			{
				ScalarType d = intensitySF->getValue(i);
				if (!ccScalarField::ValidValue(d))
				{
					hasInvalidIntensities = true;
					break;
				}
			}
		}
	}

	//Color
	bool hasColors = cloud->hasColors();
	if (hasColors)
	{
		e57::StructureNode colorbox = e57::StructureNode(imf);
		colorbox.set("colorRedMinimum",		e57::IntegerNode(imf, 0));
		colorbox.set("colorRedMaximum",		e57::IntegerNode(imf, 255));
		colorbox.set("colorGreenMinimum",	e57::IntegerNode(imf, 0));
		colorbox.set("colorGreenMaximum",	e57::IntegerNode(imf, 255));
		colorbox.set("colorBlueMinimum",	e57::IntegerNode(imf, 0));
		colorbox.set("colorBlueMaximum",	e57::IntegerNode(imf, 255));
		scanNode.set("colorLimits", colorbox);
	}

	// Add Cartesian bounding box to scan.
	{
		e57::StructureNode bboxNode = e57::StructureNode(imf);
		bboxNode.set("xMinimum", e57::FloatNode(imf, bbMin.x));
		bboxNode.set("xMaximum", e57::FloatNode(imf, bbMax.x));
		bboxNode.set("yMinimum", e57::FloatNode(imf, bbMin.y));
		bboxNode.set("yMaximum", e57::FloatNode(imf, bbMax.y));
		bboxNode.set("zMinimum", e57::FloatNode(imf, bbMin.z));
		bboxNode.set("zMaximum", e57::FloatNode(imf, bbMax.z));
		scanNode.set("cartesianBounds", bboxNode);
	}

	// Add start/stop acquisition times to scan (TODO)
	//e57::StructureNode acquisitionStart = StructureNode(imf);
	//scanNode.set("acquisitionStart", acquisitionStart);
	//acquisitionStart.set("dateTimeValue",			e57::FloatNode(imf,	dateTimeValue));
	//acquisitionStart.set("isAtomicClockReferenced",	e57::IntegerNode(imf, isAtomicClockReferenced));
	//e57::StructureNode acquisitionEnd = e57::StructureNode(imf);
	//scanNode.set("acquisitionEnd", acquisitionEnd);
	//acquisitionEnd.set("dateTimeValue",				e57::FloatNode(imf, dateTimeValue));
	//acquisitionEnd.set("isAtomicClockReferenced",	e57::IntegerNode(imf, isAtomicClockReferenced));

	// Add grouping scheme area
	// No point grouping scheme (unstructured cloud)

	// Make a prototype of datatypes that will be stored in points record.
	/// This prototype will be used in creating the points CompressedVector.
	e57::StructureNode proto = e57::StructureNode(imf);

	//prepare temporary structures
	const unsigned chunkSize = std::min<unsigned>(pointCount,(1 << 20)); //we save the file in several steps to limit the memory consumption
	TempArrays arrays;
	std::vector<e57::SourceDestBuffer> dbufs;

	//Cartesian field
	{
		e57::FloatPrecision precision = sizeof(PointCoordinateType) == 8 || isScaled ? e57::E57_DOUBLE : e57::E57_SINGLE;

		CCVector3d bbCenter = (bbMin + bbMax) / 2;

		proto.set("cartesianX", e57::FloatNode(	imf,
												bbCenter.x,
												precision,
												bbMin.x,
												bbMax.x ) );
		arrays.xData.resize(chunkSize);
		dbufs.emplace_back( imf, "cartesianX",  arrays.xData.data(),  chunkSize, true, true );

		proto.set("cartesianY", e57::FloatNode(	imf,
												bbCenter.y,
												precision,
												bbMin.y,
												bbMax.y ) );
		arrays.yData.resize(chunkSize);
		dbufs.emplace_back( imf, "cartesianY",  arrays.yData.data(),  chunkSize, true, true );

		proto.set("cartesianZ", e57::FloatNode(	imf,
												bbCenter.z,
												precision,
												bbMin.z,
												bbMax.z ) );
		arrays.zData.resize(chunkSize);
		dbufs.emplace_back( imf, "cartesianZ",  arrays.zData.data(),  chunkSize, true, true );
	}

	//Normals
	bool hasNormals = cloud->hasNormals();
	if (hasNormals)
	{
		e57::FloatPrecision precision = sizeof(PointCoordinateType) == 8 ? e57::E57_DOUBLE : e57::E57_SINGLE;

		proto.set("nor:normalX", e57::FloatNode(imf, 0.0, precision, -1.0, 1.0));
		arrays.xNormData.resize(chunkSize);
		dbufs.emplace_back( imf, "nor:normalX",  arrays.xNormData.data(),  chunkSize, true, true );

		proto.set("nor:normalY", e57::FloatNode(imf, 0.0, precision, -1.0, 1.0));
		arrays.yNormData.resize(chunkSize);
		dbufs.emplace_back( imf, "nor:normalY",  arrays.yNormData.data(),  chunkSize, true, true );

		proto.set("nor:normalZ", e57::FloatNode(imf, 0.0, precision, -1.0, 1.0));
		arrays.zNormData.resize(chunkSize);
		dbufs.emplace_back( imf, "nor:normalZ",  arrays.zNormData.data(),  chunkSize, true, true );
	}

	//Return index
	if (returnIndexSF)
	{
		assert(maxReturnIndex > minReturnIndex);
		proto.set("returnIndex", e57::IntegerNode(imf, minReturnIndex, minReturnIndex, maxReturnIndex));
		arrays.scanIndexData.resize(chunkSize);
		dbufs.emplace_back( imf, "returnIndex",  arrays.scanIndexData.data(),  chunkSize, true, true );
	}

	//Intensity field
	if (intensitySF)
	{
		proto.set("intensity", e57::FloatNode(imf, intensitySF->getMin(), sizeof(ScalarType) == 8 ? e57::E57_DOUBLE : e57::E57_SINGLE, intensitySF->getMin(), intensitySF->getMax()));
		arrays.intData.resize(chunkSize);
		dbufs.emplace_back( imf, "intensity",  arrays.intData.data(),  chunkSize, true, true );

		if (hasInvalidIntensities)
		{
			proto.set("isIntensityInvalid", e57::IntegerNode(imf, 0, 0, 1));
			arrays.isInvalidIntData.resize(chunkSize);
			dbufs.emplace_back( imf, "isIntensityInvalid",  arrays.isInvalidIntData.data(),  chunkSize, true, true );
		}
	}

	//Color fields
	if (hasColors)
	{
		proto.set("colorRed",	e57::IntegerNode(imf, 0, 0, 255));
		arrays.redData.resize(chunkSize);
		dbufs.emplace_back( imf, "colorRed",  arrays.redData.data(),  chunkSize, true, true );
		proto.set("colorGreen",	e57::IntegerNode(imf, 0, 0, 255));
		arrays.greenData.resize(chunkSize);
		dbufs.emplace_back( imf, "colorGreen",  arrays.greenData.data(),  chunkSize, true, true );
		proto.set("colorBlue",	e57::IntegerNode(imf, 0, 0, 255));
		arrays.blueData.resize(chunkSize);
		dbufs.emplace_back( imf, "colorBlue",  arrays.blueData.data(),  chunkSize, true, true );
	}

	//ignored fields
	//"sphericalRange"
	//"sphericalAzimuth"
	//"sphericalElevation"
	//"rowIndex"
	//"columnIndex"
	//"timeStamp"
	//"cartesianInvalidState"
	//"sphericalInvalidState"
	//"isColorInvalid"
	//"isTimeStampInvalid"

	// Make empty codecs vector for use in creating points CompressedVector.
	/// If this vector is empty, it is assumed that all fields will use the BitPack codec.
	e57::VectorNode codecs = e57::VectorNode(imf, true);

	// Create CompressedVector for storing points.
	/// We use the prototype and empty codecs tree from above.
	e57::CompressedVectorNode points = e57::CompressedVectorNode(imf, proto, codecs);
	scanNode.set("points", points);
	data3D.append(scanNode);

	e57::CompressedVectorWriter writer = points.writer(dbufs);

	//progress bar
	CCCoreLib::NormalizedProgress nprogress(progressDlg, pointCount);
	if (progressDlg)
	{
		progressDlg->setMethodTitle(QObject::tr("Write E57 file"));
		progressDlg->setInfo(QObject::tr("Scan #%1 - %2 points").arg(s_absoluteScanIndex).arg(pointCount));
		progressDlg->start();
		QApplication::processEvents();
	}

	unsigned index = 0;
	unsigned remainingPointCount = pointCount;
	while (remainingPointCount != 0)
	{
		unsigned thisChunkSize = std::min(remainingPointCount, chunkSize);

		//load arrays
		for (unsigned i = 0; i < thisChunkSize; ++i, ++index)
		{
			//we apply the Global Scale but not the Global Shift (already incorporated in the 'pose' matrix above)
			CCVector3d Psensor = cloud->getPointPersistentPtr(index)->toDouble() / globalScale;

			//DGM: according to E57 specifications, the points are saved in the local CS
			//(i.e. in the sensor 'input' coordinate system)
			CCVector3d Plocal = (hasSensorPoseMat ? fromSensorToLocalCS * Psensor : Psensor);

			arrays.xData[i] = Plocal.x;
			arrays.yData[i] = Plocal.y;
			arrays.zData[i] = Plocal.z;

			if (intensitySF)
			{
				assert(!arrays.intData.empty());
				ScalarType sfVal = intensitySF->getValue(index);
				arrays.intData[i] = static_cast<double>(sfVal);
				if (!arrays.isInvalidIntData.empty())
				{
					arrays.isInvalidIntData[i] = ccScalarField::ValidValue(sfVal) ? VALID_DATA : INVALID_DATA;
				}
			}

			if (hasNormals)
			{
				const CCVector3& N = cloud->getPointNormal(index);
				arrays.xNormData[i] = static_cast<double>(N.x);
				arrays.yNormData[i] = static_cast<double>(N.y);
				arrays.zNormData[i] = static_cast<double>(N.z);
			}

			if (hasColors)
			{
				//Normalize color to 0 - 255
				const ccColor::Rgb& C = cloud->getPointColor(index);
				arrays.redData[i]	= static_cast<double>(C.r);
				arrays.greenData[i]	= static_cast<double>(C.g);
				arrays.blueData[i]	= static_cast<double>(C.b);
			}

			if (returnIndexSF)
			{
				assert(!arrays.scanIndexData.empty());
				arrays.scanIndexData[i] = static_cast<int8_t>(returnIndexSF->getValue(index));
			}
			
			if (!nprogress.oneStep())
			{
				QApplication::processEvents();
				s_cancelRequestedByUser = true;
				break;
			}
		}

		writer.write(thisChunkSize);
		
		assert(thisChunkSize <= remainingPointCount);
		remainingPointCount -= thisChunkSize;
	}

	writer.close();

	return true;
}

void SaveImage(	const ccImage* image,
				const QString& scanGUID,
				e57::ImageFile& imf,
				e57::VectorNode& images2D,
				unsigned imageIndex,
				const CCVector3d& globalShift)
{
	assert(image);

	e57::StructureNode imageNode = e57::StructureNode(imf);

	//GUID
	imageNode.set("guid", e57::StringNode(imf, GetNewGuid().toStdString())); //required

	//Name
	if (!image->getName().isEmpty())
	{
		imageNode.set("name", e57::StringNode(imf, image->getName().toStdString()));
	}
	else
	{
		imageNode.set("name", e57::StringNode(imf, QString("Image %1").arg(imageIndex).toStdString()));
	}

	imageNode.set("associatedData3DGuid", e57::StringNode(imf, scanGUID.toStdString()));

	//Restore scan (node) information if any
	if (image->hasMetaData(s_e57NodeInfoKey))
	{
		QString metaData = image->getMetaData(s_e57NodeInfoKey).toString();
		QStringList stringList = metaData.split("\n", Qt::SkipEmptyParts);

		E57NodeMap nodeMap;
		E57NodeMap::FromStringList(nodeMap, stringList);
		SaveNodeInfo(nodeMap, imageNode, imf);
	}

	//acquisitionDateTime
	{
		//e57::StructureNode acquisitionDateTime = e57::StructureNode(imf);
		//imageNode.set("acquisitionDateTime", acquisitionDateTime);
		//acquisitionDateTime.set("dateTimeValue", e57::FloatNode(imf, dateTimeValue));
		//acquisitionDateTime.set("isAtomicClockReferenced", e57::IntegerNode(imf, isAtomicClockReferenced));
	}

	// Create pose structure for image (if any)
	const ccCameraSensor* sensor = static_cast<const ccImage*>(image)->getAssociatedSensor();
	{
		ccGLMatrixd toSensorCS; // 'Sensor' output coordinate system (no Global Shift applied)
		toSensorCS.toIdentity();
		bool hasSensorPoseMat = false;
		{
			//restore original sensor pose (if any)
			if (sensor)
			{
				ccIndexedTransformation poseMat;
				if (sensor->getActiveAbsoluteTransformation(poseMat))
				{
					hasSensorPoseMat = true;
					toSensorCS = ccGLMatrixd(poseMat.data());
				}
			}
			else
			{
				// look for the saved sensor position in the meta-data
				QString poseStr = image->getMetaData(s_e57PoseKey).toString();
				if (!poseStr.isEmpty())
				{
					toSensorCS = ccGLMatrixd::FromString(poseStr, hasSensorPoseMat);
					if (hasSensorPoseMat)
					{
						//apply transformation history (i.e. transformations applied after the entity has been loaded)
						//--> we assume they have been implicitly applied to the sensor as well
						ccGLMatrixd historyTransform(image->getGLTransformationHistory().data());
						toSensorCS = historyTransform * toSensorCS;
					}
					else
					{
						ccLog::Warning("[E57Filter::saveFile] Sensor pose meta-data is invalid");
						toSensorCS.toIdentity();
					}
				}
			}
		}

		if (hasSensorPoseMat)
		{
			SavePoseInformation(imageNode, imf, toSensorCS, globalShift);
		}
	}

	//save image data as PNG
	QByteArray ba;
	{
		QBuffer buffer(&ba);
		buffer.open(QIODevice::WriteOnly);
		image->data().save(&buffer, "JPG"); // writes image into ba in JPG format
	}
	int imageSize = ba.size();

	e57::StructureNode cameraRepresentationNode = e57::StructureNode(imf);
	QString cameraRepresentationStr("visualReferenceRepresentation");

	//restore camera data from meta-data
	VisualReferenceRepresentation* cameraRepresentation = nullptr;
	if (image->hasMetaData(s_e57CameraInfoKey))
	{
		QStringList stringList = image->getMetaData(s_e57CameraInfoKey).toString().split("\n");
		if (stringList.size() != 0 && stringList.front().startsWith("type="))
		{
			QString sensorType = stringList.front().mid(5);
			stringList.pop_front();
			if (sensorType == VisualReferenceRepresentation::GetName())
			{
				cameraRepresentation = new VisualReferenceRepresentation;
			}
			else if (sensorType == PinholeRepresentation::GetName())
			{
				cameraRepresentation = new PinholeRepresentation;
			}
			else if (sensorType == SphericalRepresentation::GetName())
			{
				cameraRepresentation = new SphericalRepresentation;
			}
			else if (sensorType == CylindricalRepresentation::GetName())
			{
				cameraRepresentation = new CylindricalRepresentation;
			}
			else
			{
				ccLog::Warning("[E57] Invalid 'E57 camera info' meta-data (unkown sensor type)");
			}
		}
		else
		{
			ccLog::Warning("[E57] Invalid 'E57 camera info' meta-data (invalid format: no 'type' key)");
		}

		if (cameraRepresentation && !cameraRepresentation->fromStringList(stringList))
		{
			// failed to unserialize the meta-data
			ccLog::Warning("[E57] Invalid 'E57 camera info' meta-data (invalid format)");
		}
	}

	if (!cameraRepresentation && sensor)
	{
		// we still need to look if there's a camera sensor attached to the image
		// (in case the image comes from another source than E57)
		const auto& sensorParams = sensor->getIntrinsicParameters();

		const double& pixelWidth_mm = sensorParams.pixelSize_mm[0];
		const double& pixelHeight_mm = sensorParams.pixelSize_mm[1];
		double focal_mm = sensorParams.vertFocal_pix * pixelHeight_mm;


		PinholeRepresentation* pinholeRepresentation = new PinholeRepresentation;
		{
			pinholeRepresentation->focalLength = focal_mm / 1000.0;
			pinholeRepresentation->pixelWidth = pixelWidth_mm / 1000.0;
			pinholeRepresentation->pixelHeight = pixelHeight_mm / 1000.0;
			pinholeRepresentation->principalPointX = sensorParams.principal_point[0];
			pinholeRepresentation->principalPointY = sensorParams.principal_point[1];

			pinholeRepresentation->imageMaskSize = 0;
			pinholeRepresentation->imageWidth = image->data().width();
			pinholeRepresentation->imageHeight = image->data().height();
			pinholeRepresentation->imageSize = static_cast<int64_t>(pinholeRepresentation->imageWidth) * pinholeRepresentation->imageHeight;
		}
		cameraRepresentation = pinholeRepresentation;
	}

	if (cameraRepresentation)
	{
		cameraRepresentationStr = cameraRepresentation->getName();

		if (cameraRepresentation->getType() == E57_VISUAL
			|| cameraRepresentation->getType() == E57_SPHERICAL
			|| cameraRepresentation->getType() == E57_PINHOLE
			|| cameraRepresentation->getType() == E57_CYLINDRICAL
			)
		{
			//cameraRepresentationNode.set("imageWidth", e57::IntegerNode(imf, cameraRepresentation->imageWidth));		//overwritten later with the real image size
			//cameraRepresentationNode.set("imageHeight", e57::IntegerNode(imf, cameraRepresentation->imageHeight));	//overwritten later with the real image size
			//TODO: should we manage the image mask?

			if (cameraRepresentation->getType() == E57_SPHERICAL
				|| cameraRepresentation->getType() == E57_PINHOLE
				|| cameraRepresentation->getType() == E57_CYLINDRICAL
				)
			{
				SphericalRepresentation* spherical = static_cast<SphericalRepresentation*>(cameraRepresentation);

				cameraRepresentationNode.set("pixelWidth", e57::FloatNode(imf, spherical->pixelWidth));
				cameraRepresentationNode.set("pixelHeight", e57::FloatNode(imf, spherical->pixelHeight));

				if (cameraRepresentation->getType() == E57_PINHOLE)
				{
					PinholeRepresentation* pinhole = static_cast<PinholeRepresentation*>(cameraRepresentation);

					cameraRepresentationNode.set("focalLength", e57::FloatNode(imf, pinhole->focalLength));
					cameraRepresentationNode.set("principalPointX", e57::FloatNode(imf, pinhole->principalPointX));
					cameraRepresentationNode.set("principalPointY", e57::FloatNode(imf, pinhole->principalPointY));
				}
				else if (cameraRepresentation->getType() == E57_CYLINDRICAL)
				{
					PinholeRepresentation* pinhole = static_cast<PinholeRepresentation*>(cameraRepresentation);

					cameraRepresentationNode.set("focalLength", e57::FloatNode(imf, pinhole->focalLength));
					cameraRepresentationNode.set("principalPointX", e57::FloatNode(imf, pinhole->principalPointX));
					cameraRepresentationNode.set("principalPointY", e57::FloatNode(imf, pinhole->principalPointY));
				}
			}
		}
		else
		{
			ccLog::Warning("[E57] Invalid 'E57 camera info' meta-data (wrong camera type)");
		}

		delete cameraRepresentation;
		cameraRepresentation = nullptr;
	}

	e57::BlobNode blob(imf, imageSize);
	cameraRepresentationNode.set("jpegImage", blob);
	cameraRepresentationNode.set("imageHeight", e57::IntegerNode(imf, image->getH()));
	cameraRepresentationNode.set("imageWidth", e57::IntegerNode(imf, image->getW()));

	imageNode.set(cameraRepresentationStr.toStdString(), cameraRepresentationNode);
	images2D.append(imageNode);
	blob.write((uint8_t*)ba.data(), 0, static_cast<size_t>(imageSize));
}

CC_FILE_ERROR E57Filter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	//we assume the input entity is either a cloud or a group of clouds (=multiple scans)
	std::vector<ccPointCloud*> scans;

	if (entity->isA(CC_TYPES::POINT_CLOUD))
	{
		scans.push_back(static_cast<ccPointCloud*>(entity));
	}
	else
	{
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
		{
			if (entity->getChild(i)->isA(CC_TYPES::POINT_CLOUD))
			{
				scans.push_back(static_cast<ccPointCloud*>(entity->getChild(i)));
			}
		}
	}

	if (scans.empty())
		return CC_FERR_NO_SAVE;

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	try
	{
		e57::ImageFile imf(filename.toStdString(), "w");
		if (!imf.isOpen())
			return CC_FERR_WRITING;

		//get root
		e57::StructureNode root = imf.root();

		//header info

		/// We are using the E57 v1.0 data format standard fieldnames.
		/// The standard fieldnames are used without an extension prefix (in the default namespace).
		/// We explicitly register it for completeness (the reference implementaion would do it for us, if we didn't).
		imf.extensionsAdd("", e57::E57_V1_0_URI);

		// Set per-file properties.
		/// Path names: "/formatName", "/majorVersion", "/minorVersion", "/coordinateMetadata"
		root.set("formatName", e57::StringNode(imf, "ASTM E57 3D Imaging Data File"));
		root.set("guid", e57::StringNode(imf, GetNewGuid().toStdString()));

		// Get ASTM version number supported by library, so can write it into file
		root.set("versionMajor", e57::IntegerNode(imf, e57::Version::astmMajor()));
		root.set("versionMinor", e57::IntegerNode(imf, e57::Version::astmMinor()));
		root.set("e57LibraryVersion", e57::StringNode(imf, e57::Version::library()));

		// Save a dummy string for coordinate system.
		/// Really should be a valid WKT string identifying the coordinate reference system (CRS).
		root.set("coordinateMetadata", e57::StringNode(imf, ""));

		// Create creationDateTime structure
		/// Path name: "/creationDateTime
		e57::StructureNode creationDateTime = e57::StructureNode(imf);
		creationDateTime.set("dateTimeValue", e57::FloatNode(imf, 0.0));
		creationDateTime.set("isAtomicClockReferenced", e57::IntegerNode(imf, 0));
		root.set("creationDateTime", creationDateTime);

		//3D data
		e57::VectorNode data3D(imf, true);
		root.set("data3D", data3D);

		//Images
		e57::VectorNode images2D(imf, true);
		root.set("images2D", images2D);

		//we store (temporarily) the saved scans associated with
		//their unique GUID in a map (to retrieve them later if
		//necessary - for example to associate them with images)
		QMap<ccHObject*, QString> scansGUID;
		s_absoluteScanIndex = 0;

		//progress dialog
		QScopedPointer<ccProgressDialog> progressDlg(nullptr);
		if (parameters.parentWidget)
		{
			progressDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
			progressDlg->setAutoClose(false);
		}
		s_cancelRequestedByUser = false;

		//Extension for normals
		bool hasNormals = false;

		for (auto cloud : scans)
		{
			QString scanGUID = GetNewGuid();

			//we should only add the "normals" extension once
			if (!hasNormals && cloud->hasNormals())
			{
				hasNormals = true;
				imf.extensionsAdd("nor", "http://www.libe57.org/E57_NOR_surface_normals.txt");
			}

			//create corresponding node
			e57::StructureNode scanNode = e57::StructureNode(imf);
			if (SaveScan(cloud, scanNode, imf, data3D, scanGUID, progressDlg.data()))
			{
				++s_absoluteScanIndex;
				scansGUID.insert(cloud, scanGUID);
			}
			else
			{
				result = CC_FERR_WRITING;
				break;
			}

			if (s_cancelRequestedByUser)
			{
				result = CC_FERR_CANCELED_BY_USER;
				break;
			}
		}

		if (result == CC_FERR_NO_ERROR)
		{
			//Save images
			unsigned imageIndex = 0;
			size_t scanCount = scans.size();
			for (size_t i = 0; i < scanCount; ++i)
			{
				ccPointCloud* cloud = scans[i];
				ccHObject::Container images;
				unsigned imageCount = cloud->filterChildren(images, false, CC_TYPES::IMAGE);

				if (imageCount != 0)
				{
					//progress bar
					CCCoreLib::NormalizedProgress nprogress(progressDlg.data(), imageCount);
					if (progressDlg)
					{
						progressDlg->setMethodTitle(QObject::tr("Write E57 file"));
						progressDlg->setInfo(QObject::tr("Cloud #%1 - Images: %2").arg(i).arg(imageCount));
						progressDlg->start();
						QApplication::processEvents();
					}

					for (unsigned j = 0; j < imageCount; ++j)
					{
						assert(images[j]->isKindOf(CC_TYPES::IMAGE));
						assert(scansGUID.contains(cloud));
						QString scanGUID = scansGUID.value(cloud);
						SaveImage(static_cast<ccImage*>(images[j]), scanGUID, imf, images2D, imageIndex, cloud->getGlobalShift());
						++imageIndex;
						if (!nprogress.oneStep())
						{
							s_cancelRequestedByUser = true;
							i = scanCount; //double break!
							result = CC_FERR_CANCELED_BY_USER;
							break;
						}
					}
				}
			}
		}

		imf.close();
	}
	catch (const e57::E57Exception& e)
	{
		ccLog::Warning( QStringLiteral("[E57] Error: %1 (%2 line %3)")
						.arg( e57::Utilities::errorCodeToString( e.errorCode() ).c_str() )
						.arg( e.sourceFileName() )
						.arg( e.sourceLineNumber() )
						);
		
		if ( !e.context().empty() )
		{
			ccLog::Warning( QStringLiteral("    context: %1").arg( QString::fromStdString( e.context() ) ) );
		}
		
		result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch (...)
	{
		ccLog::Warning("[E57] Unknown error");
		result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}

	return result;
}

static bool NodeStructureToTree(ccHObject* currentTreeNode, const e57::Node &currentE57Node)
{
	assert(currentTreeNode);
	ccHObject* obj = new ccHObject(currentE57Node.elementName().c_str());
	currentTreeNode->addChild(obj);

	e57::ustring name = currentE57Node.elementName();
	QString infoStr = QString(name.c_str() == nullptr || name.c_str()[0]==0 ? "No name" : name.c_str());

	switch (currentE57Node.type())
	{
	case e57::E57_STRUCTURE:
	{
		infoStr += QString(" [STRUCTURE]");
		e57::StructureNode s = static_cast<e57::StructureNode>(currentE57Node);
		for (int64_t i = 0; i < s.childCount(); ++i)
			NodeStructureToTree(obj, s.get(i));
	}
	break;
	case e57::E57_VECTOR:
	{
		infoStr += QString(" [VECTOR]");
		e57::VectorNode v = static_cast<e57::VectorNode>(currentE57Node);
		for (int64_t i = 0; i < v.childCount(); ++i)
			NodeStructureToTree(obj, v.get(i));
	}
	break;
	case e57::E57_COMPRESSED_VECTOR:
	{
		e57::CompressedVectorNode cv = static_cast<e57::CompressedVectorNode>(currentE57Node);
		infoStr += QString(" [COMPRESSED VECTOR (%1 elements)]").arg(cv.childCount());
	}
	break;
	case e57::E57_INTEGER:
	{
		e57::IntegerNode i = static_cast<e57::IntegerNode>(currentE57Node);
		infoStr += QString(" [INTEGER: %1]").arg(i.value());
	}
	break;
	case e57::E57_SCALED_INTEGER:
	{
		e57::ScaledIntegerNode si = static_cast<e57::ScaledIntegerNode>(currentE57Node);
		infoStr += QString(" [SCALED INTEGER: %1]").arg(si.scaledValue());
	}
	break;
	case e57::E57_FLOAT:
	{
		e57::FloatNode f = static_cast<e57::FloatNode>(currentE57Node);
		infoStr += QString(" [FLOAT: %1]").arg(f.value());
	}
	break;
	case e57::E57_STRING:
	{
		e57::StringNode s = static_cast<e57::StringNode>(currentE57Node);
		infoStr += QString(" [STRING: %1]").arg(s.value().c_str());
	}
	break;
	case e57::E57_BLOB:
	{
		e57::BlobNode b = static_cast<e57::BlobNode>(currentE57Node);
		infoStr += QString(" [BLOB (%1 bytes)]").arg(b.byteCount());
	}
	break;
	default:
	{
		infoStr += QString("[INVALID]");
		obj->setName(infoStr);
		return false;
	}
	}

	obj->setName(infoStr);
	return true;
}

static void NodeToConsole(const e57::Node &node)
{
	QString infoStr = QString("[E57] '%1' - ").arg(node.elementName().c_str());
	switch(node.type())
	{
	case e57::E57_STRUCTURE:
		{
			e57::StructureNode s = static_cast<e57::StructureNode>(node);
			infoStr += QString("STRUCTURE, %1 child(ren)").arg(s.childCount());
		}
		break;
	case e57::E57_VECTOR:
		{
			e57::VectorNode v = static_cast<e57::VectorNode>(node);
			infoStr += QString("VECTOR, %1 child(ren)").arg(v.childCount());
		}
		break;
	case e57::E57_COMPRESSED_VECTOR:
		{
			e57::CompressedVectorNode cv = static_cast<e57::CompressedVectorNode>(node);
			infoStr += QString("COMPRESSED VECTOR, %1 elements").arg(cv.childCount());
		}
		break;
	case e57::E57_INTEGER:
		{
			e57::IntegerNode i = static_cast<e57::IntegerNode>(node);
			infoStr += QString("%1 (INTEGER)").arg(i.value());
		}
		break;
	case e57::E57_SCALED_INTEGER:
		{
			e57::ScaledIntegerNode si = static_cast<e57::ScaledIntegerNode>(node);
			infoStr += QString("%1 (SCALED INTEGER)").arg(si.scaledValue());
		}
		break;
	case e57::E57_FLOAT:
		{
			e57::FloatNode f = static_cast<e57::FloatNode>(node);
			infoStr += QString("%1 (FLOAT)").arg(f.value());
		}
		break;
	case e57::E57_STRING:
		{
			e57::StringNode s = static_cast<e57::StringNode>(node);
			infoStr += QString(s.value().c_str());
		}
		break;
	case e57::E57_BLOB:
		{
			e57::BlobNode b = static_cast<e57::BlobNode>(node);
			infoStr += QString("BLOB, size=%1").arg(b.byteCount());
		}
		break;
	default:
		{
			infoStr += QString("INVALID");
		}
		break;
	}

	ccLog::Print(infoStr);
}

static bool ChildNodeToConsole(const e57::Node &node, const char* childName)
{
	assert(childName);
	if (node.type() == e57::E57_STRUCTURE)
	{
		e57::StructureNode s = static_cast<e57::StructureNode>(node);
		if (!s.isDefined(childName))
		{
			ccLog::Warning("[E57] Couldn't find element named '%s'",childName);
			return false;
		}
		else
		{
			try
			{
				NodeToConsole(s.get(childName));
			}
			catch(e57::E57Exception& ex)
			{
				ex.report(__FILE__, __LINE__, __FUNCTION__);
				return false;
			}
		}
	}
	else if (node.type() == e57::E57_VECTOR)
	{
		e57::VectorNode v = static_cast<e57::VectorNode>(node);
		if (!v.isDefined(childName))
		{
			ccLog::Warning("[E57] Couldn't find element named '%s'",childName);
			return false;
		}
		else
		{
			try
			{
				NodeToConsole(v.get(childName));
			}
			catch(e57::E57Exception& ex)
			{
				ex.report(__FILE__, __LINE__, __FUNCTION__);
				return false;
			}
		}
	}
	else
	{
		ccLog::Warning("[E57] Element '%s' has no child (not a structure nor a vector!)",node.elementName().c_str());
		return false;
	}

	return true;
}

//Freely inspired from "E57 Simple API" by Stan Coleby
static void DecodePrototype(const e57::StructureNode& scan, const e57::StructureNode& proto, E57ScanHeader& header)
{
	// Get a prototype of datatypes that will be stored in points record.
	header.pointFields.cartesianXField = proto.isDefined("cartesianX");
	header.pointFields.cartesianYField = proto.isDefined("cartesianY");
	header.pointFields.cartesianZField = proto.isDefined("cartesianZ");
	header.pointFields.cartesianInvalidStateField = proto.isDefined("cartesianInvalidState");

	header.pointFields.pointRangeScaledInteger = 0; //FloatNode
	header.pointFields.pointRangeMinimum = 0;
	header.pointFields.pointRangeMaximum = 0; 

	if ( proto.isDefined("cartesianX") )
	{
		if ( proto.get("cartesianX").type() == e57::E57_SCALED_INTEGER )
		{
			double scale = e57::ScaledIntegerNode(proto.get("cartesianX")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("cartesianX")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("cartesianX")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("cartesianX")).maximum();
			header.pointFields.pointRangeMinimum = minimum * scale + offset;	
			header.pointFields.pointRangeMaximum = maximum * scale + offset;
			header.pointFields.pointRangeScaledInteger = scale;

		}
		else if ( proto.get("cartesianX").type() == e57::E57_FLOAT )
		{
			header.pointFields.pointRangeMinimum = e57::FloatNode(proto.get("cartesianX")).minimum();
			header.pointFields.pointRangeMaximum = e57::FloatNode(proto.get("cartesianX")).maximum();
		}
	} 
	else if ( proto.isDefined("sphericalRange") )
	{
		if ( proto.get("sphericalRange").type() == e57::E57_SCALED_INTEGER )
		{
			double scale = e57::ScaledIntegerNode(proto.get("sphericalRange")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("sphericalRange")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("sphericalRange")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("sphericalRange")).maximum();
			header.pointFields.pointRangeMinimum = minimum * scale + offset;	
			header.pointFields.pointRangeMaximum = maximum * scale + offset;
			header.pointFields.pointRangeScaledInteger = scale;

		}
		else if ( proto.get("sphericalRange").type() == e57::E57_FLOAT )
		{
			header.pointFields.pointRangeMinimum = e57::FloatNode(proto.get("sphericalRange")).minimum();
			header.pointFields.pointRangeMaximum = e57::FloatNode(proto.get("sphericalRange")).maximum();
		}
	}

	header.pointFields.sphericalRangeField = proto.isDefined("sphericalRange");
	header.pointFields.sphericalAzimuthField = proto.isDefined("sphericalAzimuth");
	header.pointFields.sphericalElevationField = proto.isDefined("sphericalElevation");
	header.pointFields.sphericalInvalidStateField = proto.isDefined("sphericalInvalidState");

	header.pointFields.angleScaledInteger = 0.; //FloatNode
	header.pointFields.angleMinimum = 0.;
	header.pointFields.angleMaximum = 0.;

	if ( proto.isDefined("sphericalAzimuth") )
	{
		if ( proto.get("sphericalAzimuth").type() == e57::E57_SCALED_INTEGER)
		{
			double scale = e57::ScaledIntegerNode(proto.get("sphericalAzimuth")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("sphericalAzimuth")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("sphericalAzimuth")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("sphericalAzimuth")).maximum();
			header.pointFields.angleMinimum = minimum * scale + offset;	
			header.pointFields.angleMaximum = maximum * scale + offset;
			header.pointFields.angleScaledInteger = scale;

		}
		else if ( proto.get("sphericalAzimuth").type() == e57::E57_FLOAT )
		{
			header.pointFields.angleMinimum = e57::FloatNode(proto.get("sphericalAzimuth")).minimum();
			header.pointFields.angleMaximum = e57::FloatNode(proto.get("sphericalAzimuth")).maximum();
		}
	}

	header.pointFields.rowIndexField = proto.isDefined("rowIndex");
	header.pointFields.columnIndexField = proto.isDefined("columnIndex");
	header.pointFields.rowIndexMaximum = 0;
	header.pointFields.columnIndexMaximum = 0;

	if ( proto.isDefined("rowIndex") )
	{
		header.pointFields.rowIndexMaximum = static_cast<uint32_t>(e57::IntegerNode(proto.get("rowIndex")).maximum());
	}

	if ( proto.isDefined("columnIndex") )
	{
		header.pointFields.columnIndexMaximum = static_cast<uint32_t>(e57::IntegerNode(proto.get("columnIndex")).maximum());
	}

	header.pointFields.returnIndexField = proto.isDefined("returnIndex");
	header.pointFields.returnCountField = proto.isDefined("returnCount");
	header.pointFields.returnMaximum = 0;

	if ( proto.isDefined("returnIndex") )
	{
		header.pointFields.returnMaximum = static_cast<uint8_t>(e57::IntegerNode(proto.get("returnIndex")).maximum());
	}

	header.pointFields.normXField = proto.isDefined("nor:normalX");
	header.pointFields.normYField = proto.isDefined("nor:normalY");
	header.pointFields.normZField = proto.isDefined("nor:normalZ");

	if ( proto.isDefined("nor:normalX") )
	{
		if ( proto.get("nor:normalX").type() == e57::E57_SCALED_INTEGER )
		{
			double scale = e57::ScaledIntegerNode(proto.get("nor:normalX")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("nor:normalX")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("nor:normalX")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("nor:normalX")).maximum();
			header.pointFields.normRangeMinimum = minimum * scale + offset;	
			header.pointFields.normRangeMaximum = maximum * scale + offset;
			header.pointFields.normRangeScaledInteger = scale;

		}
		else if ( proto.get("nor:normalX").type() == e57::E57_FLOAT )
		{
			header.pointFields.normRangeMinimum = e57::FloatNode(proto.get("nor:normalX")).minimum();
			header.pointFields.normRangeMaximum = e57::FloatNode(proto.get("nor:normalX")).maximum();
		}
	} 

	header.pointFields.timeStampField = proto.isDefined("timeStamp");
	header.pointFields.isTimeStampInvalidField = proto.isDefined("isTimeStampInvalid");
	header.pointFields.timeMaximum = 0.;

	if ( proto.isDefined("timeStamp") )
	{
		if ( proto.get("timeStamp").type() == e57::E57_INTEGER)
			header.pointFields.timeMaximum = static_cast<double>(e57::IntegerNode(proto.get("timeStamp")).maximum());
		else if ( proto.get("timeStamp").type() == e57::E57_FLOAT)
			header.pointFields.timeMaximum = static_cast<double>(e57::FloatNode(proto.get("timeStamp")).maximum());
	}

	header.pointFields.intensityField = proto.isDefined("intensity");
	header.pointFields.isIntensityInvalidField = proto.isDefined("isIntensityInvalid");
	header.pointFields.intensityScaledInteger = 0.;

	header.intensityLimits.intensityMinimum = 0.;
	header.intensityLimits.intensityMaximum = 0.;

	if ( scan.isDefined("intensityLimits") )
	{
		e57::StructureNode intbox(scan.get("intensityLimits"));
		if ( intbox.get("intensityMaximum").type() == e57::E57_SCALED_INTEGER )
		{
			header.intensityLimits.intensityMaximum = e57::ScaledIntegerNode(intbox.get("intensityMaximum")).scaledValue();
			header.intensityLimits.intensityMinimum = e57::ScaledIntegerNode(intbox.get("intensityMinimum")).scaledValue();
		}
		else if ( intbox.get("intensityMaximum").type() == e57::E57_FLOAT )
		{
			header.intensityLimits.intensityMaximum = e57::FloatNode(intbox.get("intensityMaximum")).value();
			header.intensityLimits.intensityMinimum = e57::FloatNode(intbox.get("intensityMinimum")).value();
		}
		else if ( intbox.get("intensityMaximum").type() == e57::E57_INTEGER)
		{
			header.intensityLimits.intensityMaximum = static_cast<double>(e57::IntegerNode(intbox.get("intensityMaximum")).value());
			header.intensityLimits.intensityMinimum = static_cast<double>(e57::IntegerNode(intbox.get("intensityMinimum")).value());
		}
	}
	
	if ( proto.isDefined("intensity") )
	{
		if (proto.get("intensity").type() == e57::E57_INTEGER)
		{
			if (header.intensityLimits.intensityMaximum == 0.)
			{
				header.intensityLimits.intensityMinimum = static_cast<double>(e57::IntegerNode(proto.get("intensity")).minimum());
				header.intensityLimits.intensityMaximum = static_cast<double>(e57::IntegerNode(proto.get("intensity")).maximum());
			}
			header.pointFields.intensityScaledInteger = -1.;

		}
		else if (proto.get("intensity").type() == e57::E57_SCALED_INTEGER)
		{
			double scale = e57::ScaledIntegerNode(proto.get("intensity")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("intensity")).offset();

			if (header.intensityLimits.intensityMaximum == 0.)
			{
				int64_t minimum = e57::ScaledIntegerNode(proto.get("intensity")).minimum();
				int64_t maximum = e57::ScaledIntegerNode(proto.get("intensity")).maximum();
				header.intensityLimits.intensityMinimum = minimum * scale + offset;	
				header.intensityLimits.intensityMaximum = maximum * scale + offset;
			}
			header.pointFields.intensityScaledInteger = scale;
		}
		else if (proto.get("intensity").type() == e57::E57_FLOAT)
		{
			if (header.intensityLimits.intensityMaximum == 0.)
			{
				header.intensityLimits.intensityMinimum = e57::FloatNode(proto.get("intensity")).minimum();
				header.intensityLimits.intensityMaximum = e57::FloatNode(proto.get("intensity")).maximum();
			}
		}
	}

	header.pointFields.colorRedField = proto.isDefined("colorRed");
	header.pointFields.colorGreenField = proto.isDefined("colorGreen");
	header.pointFields.colorBlueField = proto.isDefined("colorBlue");
	header.pointFields.isColorInvalidField = proto.isDefined("isColorInvalid");

	header.colorLimits.colorRedMinimum = 0.;
	header.colorLimits.colorRedMaximum = 0.;
	header.colorLimits.colorGreenMinimum = 0.;
	header.colorLimits.colorGreenMaximum = 0.;
	header.colorLimits.colorBlueMinimum = 0.;
	header.colorLimits.colorBlueMaximum = 0.;

	if ( scan.isDefined("colorLimits") )
	{
		e57::StructureNode colorbox(scan.get("colorLimits"));
		if ( colorbox.get("colorRedMaximum").type() == e57::E57_SCALED_INTEGER )
		{
			header.colorLimits.colorRedMaximum   = e57::ScaledIntegerNode(colorbox.get("colorRedMaximum")  ).scaledValue();
			header.colorLimits.colorRedMinimum   = e57::ScaledIntegerNode(colorbox.get("colorRedMinimum")  ).scaledValue();
			header.colorLimits.colorGreenMaximum = e57::ScaledIntegerNode(colorbox.get("colorGreenMaximum")).scaledValue();
			header.colorLimits.colorGreenMinimum = e57::ScaledIntegerNode(colorbox.get("colorGreenMinimum")).scaledValue();
			header.colorLimits.colorBlueMaximum  = e57::ScaledIntegerNode(colorbox.get("colorBlueMaximum") ).scaledValue();
			header.colorLimits.colorBlueMinimum  = e57::ScaledIntegerNode(colorbox.get("colorBlueMinimum") ).scaledValue();
		}
		else if ( colorbox.get("colorRedMaximum").type() == e57::E57_FLOAT )
		{
			header.colorLimits.colorRedMaximum =   e57::FloatNode(colorbox.get("colorRedMaximum")  ).value();
			header.colorLimits.colorRedMinimum =   e57::FloatNode(colorbox.get("colorRedMinimum")  ).value();
			header.colorLimits.colorGreenMaximum = e57::FloatNode(colorbox.get("colorGreenMaximum")).value();
			header.colorLimits.colorGreenMinimum = e57::FloatNode(colorbox.get("colorGreenMinimum")).value();
			header.colorLimits.colorBlueMaximum =  e57::FloatNode(colorbox.get("colorBlueMaximum") ).value();
			header.colorLimits.colorBlueMinimum =  e57::FloatNode(colorbox.get("colorBlueMinimum") ).value();
		}
		else if ( colorbox.get("colorRedMaximum").type() == e57::E57_INTEGER)
		{
			header.colorLimits.colorRedMaximum =   static_cast<double>(e57::IntegerNode(colorbox.get("colorRedMaximum")  ).value());
			header.colorLimits.colorRedMinimum =   static_cast<double>(e57::IntegerNode(colorbox.get("colorRedMinimum")  ).value());
			header.colorLimits.colorGreenMaximum = static_cast<double>(e57::IntegerNode(colorbox.get("colorGreenMaximum")).value());
			header.colorLimits.colorGreenMinimum = static_cast<double>(e57::IntegerNode(colorbox.get("colorGreenMinimum")).value());
			header.colorLimits.colorBlueMaximum =  static_cast<double>(e57::IntegerNode(colorbox.get("colorBlueMaximum") ).value());
			header.colorLimits.colorBlueMinimum =  static_cast<double>(e57::IntegerNode(colorbox.get("colorBlueMinimum") ).value());
		}
	}

	if ( (header.colorLimits.colorRedMaximum == 0.) && proto.isDefined("colorRed") )
	{
		if (proto.get("colorRed").type() == e57::E57_INTEGER)
		{
			header.colorLimits.colorRedMinimum = static_cast<double>(e57::IntegerNode(proto.get("colorRed")).minimum());
			header.colorLimits.colorRedMaximum = static_cast<double>(e57::IntegerNode(proto.get("colorRed")).maximum());
		}
		else if (proto.get("colorRed").type() == e57::E57_FLOAT)
		{
			header.colorLimits.colorRedMinimum = e57::FloatNode(proto.get("colorRed")).minimum();
			header.colorLimits.colorRedMaximum = e57::FloatNode(proto.get("colorRed")).maximum();
		}
		else if (proto.get("colorRed").type() == e57::E57_SCALED_INTEGER)
		{
			double scale = e57::ScaledIntegerNode(proto.get("colorRed")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("colorRed")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("colorRed")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("colorRed")).maximum();
			header.colorLimits.colorRedMinimum = minimum * scale + offset;	
			header.colorLimits.colorRedMaximum = maximum * scale + offset;
		}
	}

	if ( (header.colorLimits.colorGreenMaximum == 0.) && proto.isDefined("colorGreen") )
	{
		if (proto.get("colorGreen").type() == e57::E57_INTEGER)
		{
			header.colorLimits.colorGreenMinimum = static_cast<double>(e57::IntegerNode(proto.get("colorGreen")).minimum());
			header.colorLimits.colorGreenMaximum = static_cast<double>(e57::IntegerNode(proto.get("colorGreen")).maximum());
		}
		else if (proto.get("colorGreen").type() == e57::E57_FLOAT)
		{
			header.colorLimits.colorGreenMinimum = e57::FloatNode(proto.get("colorGreen")).minimum();
			header.colorLimits.colorGreenMaximum = e57::FloatNode(proto.get("colorGreen")).maximum();
		}
		else if (proto.get("colorGreen").type() == e57::E57_SCALED_INTEGER)
		{
			double scale = e57::ScaledIntegerNode(proto.get("colorGreen")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("colorGreen")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("colorGreen")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("colorGreen")).maximum();
			header.colorLimits.colorGreenMinimum = minimum * scale + offset;	
			header.colorLimits.colorGreenMaximum = maximum * scale + offset;
		}
	}
	if ( (header.colorLimits.colorBlueMaximum == 0.) && proto.isDefined("colorBlue") )
	{
		if ( proto.get("colorBlue").type() == e57::E57_INTEGER)
		{
			header.colorLimits.colorBlueMinimum = static_cast<double>(e57::IntegerNode(proto.get("colorBlue")).minimum());
			header.colorLimits.colorBlueMaximum = static_cast<double>(e57::IntegerNode(proto.get("colorBlue")).maximum());
		}
		else if ( proto.get("colorBlue").type() == e57::E57_FLOAT)
		{
			header.colorLimits.colorBlueMinimum = e57::FloatNode(proto.get("colorBlue")).minimum();
			header.colorLimits.colorBlueMaximum = e57::FloatNode(proto.get("colorBlue")).maximum();
		}
		else if (proto.get("colorBlue").type() == e57::E57_SCALED_INTEGER)
		{
			double scale = e57::ScaledIntegerNode(proto.get("colorBlue")).scale();
			double offset = e57::ScaledIntegerNode(proto.get("colorBlue")).offset();
			int64_t minimum = e57::ScaledIntegerNode(proto.get("colorBlue")).minimum();
			int64_t maximum = e57::ScaledIntegerNode(proto.get("colorBlue")).maximum();
			header.colorLimits.colorRedMinimum = minimum * scale + offset;	
			header.colorLimits.colorRedMaximum = maximum * scale + offset;
		}
	}
}

//Helper: decode pose information
static bool GetPoseInformation(const e57::StructureNode& node, ccGLMatrixd& poseMat)
{
	bool validPoseMat = false;
	if (node.isDefined("pose"))
	{
		poseMat.toIdentity();

		e57::StructureNode pose(node.get("pose"));
		if (pose.isDefined("rotation"))
		{
			e57::StructureNode rotNode(pose.get("rotation"));
			double quaternion[4];
			quaternion[0] = e57::FloatNode(rotNode.get("w")).value();
			quaternion[1] = e57::FloatNode(rotNode.get("x")).value();
			quaternion[2] = e57::FloatNode(rotNode.get("y")).value();
			quaternion[3] = e57::FloatNode(rotNode.get("z")).value();

			CCCoreLib::SquareMatrixd rotMat(3);
			rotMat.initFromQuaternion(quaternion);
			if (rotMat.computeDet() > 0.)
			{
				rotMat.toGlMatrix(poseMat.data());
				validPoseMat = true;
			}
			else
			{
				ccLog::Warning("[E57Filter] Ignoring invalid pose rotation");
			}
		}

		if (pose.isDefined("translation"))
		{
			e57::StructureNode transNode(pose.get("translation"));  
			poseMat.getTranslation()[0] = e57::FloatNode(transNode.get("x")).value();
			poseMat.getTranslation()[1] = e57::FloatNode(transNode.get("y")).value();
			poseMat.getTranslation()[2] = e57::FloatNode(transNode.get("z")).value();
			validPoseMat = true;
		}
	}

	return validPoseMat;
}

//! Loaded scan and its associated shift
struct LoadedScan
{
	ccPointCloud* entity = nullptr;
	bool globalShiftApplied = false;
	CCVector3d globalShift;
	bool preserveCoordinateShift = false;
};

static LoadedScan LoadScan(const e57::Node& node, QString& guidStr, ccProgressDialog* progressDlg = nullptr)
{
	if (node.type() != e57::E57_STRUCTURE)
	{
		ccLog::Warning("[E57Filter] Scan nodes should be STRUCTURES!");
		return {};
	}
	e57::StructureNode scanNode(node);

	QString scanName = GetStringFromNode(scanNode, "name", "unnamed");

	//log
	ccLog::Print(QString("[E57] Reading new scan node (%1) - %2").arg(scanNode.elementName().c_str()).arg(scanName));

	if (!scanNode.isDefined("points"))
	{
		ccLog::Warning(QString("[E57Filter] No point in scan '%1'!").arg(scanNode.elementName().c_str()));
		return {};
	}

	//unique GUID
	if (scanNode.isDefined("guid"))
	{
		e57::Node guidNode = scanNode.get("guid");
		assert(guidNode.type() == e57::E57_STRING);
		guidStr = QString(static_cast<e57::StringNode>(guidNode).value().c_str());
	}
	else
	{
		//No GUID!
		guidStr.clear();
	}

	//points
	e57::CompressedVectorNode points(scanNode.get("points"));
	const int64_t pointCount = points.childCount();
	
	//prototype for points
	e57::StructureNode prototype(points.prototype());
	E57ScanHeader header;
	DecodePrototype(scanNode, prototype, header);

	bool sphericalMode = false;
	//no cartesian fields?
	if (!header.pointFields.cartesianXField &&
		!header.pointFields.cartesianYField && 
		!header.pointFields.cartesianZField)
	{
		//let's look for spherical ones
		if (!header.pointFields.sphericalRangeField &&
			!header.pointFields.sphericalAzimuthField &&
			!header.pointFields.sphericalElevationField)
		{
			ccLog::Warning(QString("[E57Filter] No readable point in scan '%1'! (only cartesian and spherical coordinates are supported right now)").arg(scanNode.elementName().c_str()));
			return {};
		}
		sphericalMode = true;
	}

	ccPointCloud* cloud = new ccPointCloud();

	if (scanNode.isDefined("name"))
	{		
		cloud->setName(scanName);
	}
	
	if (scanNode.isDefined("description"))
	{
		ccLog::Print( QStringLiteral("[E57] Internal description: %1").arg(GetStringFromNode(scanNode, "description", QString())) );
	}

	int64_t gridRowCount = 0, gridColumnCount = 0;
	{
		if (scanNode.isDefined("indexBounds"))
		{
			e57::StructureNode indexBounds(scanNode.get("indexBounds"));
			if (indexBounds.isDefined("columnMaximum"))
			{
				gridColumnCount = e57::IntegerNode(indexBounds.get("columnMaximum")).value() - e57::IntegerNode(indexBounds.get("columnMinimum")).value() + 1;
			}

			if (indexBounds.isDefined("rowMaximum"))
			{
				gridRowCount = e57::IntegerNode(indexBounds.get("rowMaximum")).value() - e57::IntegerNode(indexBounds.get("rowMinimum")).value() + 1;
			}
		}

		bool bColumnIndex = false;
		int64_t elementCount = 0;
		int64_t groupPointCount = 0; // maximum point count per group
		if (scanNode.isDefined("pointGroupingSchemes"))
		{
			e57::StructureNode pointGroupingSchemes(scanNode.get("pointGroupingSchemes"));
			if (pointGroupingSchemes.isDefined("groupingByLine"))
			{
				e57::StructureNode groupingByLine(pointGroupingSchemes.get("groupingByLine"));

				e57::CompressedVectorNode groups(groupingByLine.get("groups"));
				int64_t groupCount = groups.childCount(); // total number of groups

				e57::StringNode idElementName(groupingByLine.get("idElementName"));
				if (idElementName.value().compare("columnIndex") == 0)
				{
					bColumnIndex = true;
				}

				e57::StructureNode lineGroupRecord(groups.prototype());
				if (lineGroupRecord.isDefined("idElementValue"))
					elementCount = e57::IntegerNode(lineGroupRecord.get("idElementValue")).maximum() - e57::IntegerNode(lineGroupRecord.get("idElementValue")).minimum() + 1;
				else if (bColumnIndex)
					elementCount = gridColumnCount;
				else
					elementCount = gridRowCount;

				if (lineGroupRecord.isDefined("pointCount"))
					groupPointCount = e57::IntegerNode(lineGroupRecord.get("pointCount")).maximum();
				else if (bColumnIndex)
					groupPointCount = gridRowCount;
				else
					groupPointCount = gridColumnCount;

#if 0
				if (groupCount != 0)
				{
					int64_t protoCount = lineGroupRecord.childCount();
					std::vector<e57::SourceDestBuffer> groupSDBuffers;

					std::vector<int64_t> idElementValue, startPointIndex, pointCount;
					idElementValue.resize(groupCount);
					startPointIndex.resize(groupCount);
					pointCount.resize(groupCount);

					for (int64_t protoIndex = 0; protoIndex < protoCount; protoIndex++)
					{
						e57::ustring name = lineGroupRecord.get(protoIndex).elementName();

						if ((name.compare("idElementValue") == 0) && lineGroupRecord.isDefined("idElementValue") && (!idElementValue.empty()))
							groupSDBuffers.push_back(e57::SourceDestBuffer(node.destImageFile(), "idElementValue", idElementValue.data(), static_cast<unsigned>(groupCount), true));
						else if ((name.compare("startPointIndex") == 0) && lineGroupRecord.isDefined("startPointIndex") && (!startPointIndex.empty()))
							groupSDBuffers.push_back(e57::SourceDestBuffer(node.destImageFile(), "startPointIndex", startPointIndex.data(), static_cast<unsigned>(groupCount), true));
						else if ((name.compare("pointCount") == 0) && lineGroupRecord.isDefined("pointCount") && (!pointCount.empty()))
							groupSDBuffers.push_back(e57::SourceDestBuffer(node.destImageFile(), "pointCount", pointCount.data(), static_cast<unsigned>(groupCount), true));
					}

					e57::CompressedVectorReader reader = groups.reader(groupSDBuffers);
					reader.read();
					reader.close();
					//optional end
				}
#endif
			}
		}

		// if indexBounds is not given
		if (gridRowCount == 0)
		{
			if (bColumnIndex)
				gridRowCount = groupPointCount;
			else
				gridRowCount = elementCount;
		}
		if (gridColumnCount == 0)
		{
			if (bColumnIndex)
				gridColumnCount = elementCount;
			else
				gridColumnCount = groupPointCount;
		}
	}

	//Collect generic node information
	{
		E57NodeMap nodeInfo;
		AddToE57NodeMap(nodeInfo, scanNode, "sensorVendor");
		AddToE57NodeMap(nodeInfo, scanNode, "sensorModel");
		AddToE57NodeMap(nodeInfo, scanNode, "sensorSerialNumber");
		AddToE57NodeMap(nodeInfo, scanNode, "sensorHardwareVersion");
		AddToE57NodeMap(nodeInfo, scanNode, "sensorSoftwareVersion");
		AddToE57NodeMap(nodeInfo, scanNode, "sensorFirmwareVersion");
		AddToE57NodeMap(nodeInfo, scanNode, "temperature");
		AddToE57NodeMap(nodeInfo, scanNode, "relativeHumidity");
		AddToE57NodeMap(nodeInfo, scanNode, "atmosphericPressure");
		AddToE57NodeMap(nodeInfo, scanNode, "acquisitionStart");
		AddToE57NodeMap(nodeInfo, scanNode, "acquisitionEnd");
		cloud->setMetaData(s_e57NodeInfoKey, nodeInfo.toStringList().join("\n"));
	}

	//Currently ignored fields
	//if (scanNode.isDefined("originalGuids"))
	//if (scanNode.isDefined("cartesianBounds"))
	//if (scanNode.isDefined("sphericalBounds"))

	//scan "pose" relatively to the others
	ccGLMatrixd poseMat;
	const bool validPoseMat = GetPoseInformation(scanNode, poseMat);
	bool preserveCoordinateShift = true;
	bool globalShiftApplied = false;
	bool poseMatWasShifted = false;
	CCVector3d poseMatShift;
	ccGBLSensor* sensor = nullptr;

	if (validPoseMat)
	{
		const CCVector3d T = poseMat.getTranslationAsVec3D();
		if (FileIOFilter::HandleGlobalShift(T, poseMatShift, preserveCoordinateShift, s_loadParameters))
		{
			poseMat.setTranslation((T + poseMatShift).u);
			if (preserveCoordinateShift)
			{
				cloud->setGlobalShift(poseMatShift);
			}
			poseMatWasShifted = true;
			globalShiftApplied = true;
			ccLog::Warning("[E57Filter::loadFile] Cloud %s has been recentered! Translation: (%.2f ; %.2f ; %.2f)", qPrintable(guidStr), poseMatShift.x, poseMatShift.y, poseMatShift.z);
		}

		//cloud->setGLTransformation(poseMat); //TODO-> apply it at the end instead! Otherwise we will loose original coordinates!

		sensor = new ccGBLSensor();
		sensor->setRigidTransformation(ccGLMatrix(poseMat.data()));
	}

	//prepare temporary structures
	const unsigned chunkSize = std::min<unsigned>(pointCount, (1 << 20)); //we load the file in several steps to limit the memory consumption
	TempArrays arrays;
	std::vector<e57::SourceDestBuffer> dbufs;

	if (!cloud->reserve(static_cast<unsigned>(pointCount)))
	{
		ccLog::Error("[E57] Not enough memory!");
		delete cloud;
		return {};
	}

	// scan grid
	ccPointCloud::Grid::Shared scanGrid;
	if (header.pointFields.rowIndexField && header.pointFields.columnIndexField && gridRowCount != 0 && gridColumnCount != 0)
	{
		scanGrid.reset(new ccPointCloud::Grid);
		if (scanGrid->init(static_cast<unsigned>(gridRowCount), static_cast<unsigned>(gridColumnCount)))
		{
			arrays.rowIndex.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "rowIndex", arrays.rowIndex.data(), chunkSize, true);
			arrays.columnIndex.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "columnIndex", arrays.columnIndex.data(), chunkSize, true);
		}
		else
		{
			ccLog::Warning("[E57] Not enough memory to load the scan grid");
			scanGrid.clear();
		}
		
	}

	if (sphericalMode)
	{
		//spherical coordinates
		if (header.pointFields.sphericalRangeField)
		{
			arrays.xData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "sphericalRange", arrays.xData.data(), chunkSize, true, (prototype.get("sphericalRange").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.sphericalAzimuthField)
		{
			arrays.yData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "sphericalAzimuth", arrays.yData.data(), chunkSize, true, (prototype.get("sphericalAzimuth").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.sphericalElevationField)
		{
			arrays.zData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "sphericalElevation", arrays.zData.data(), chunkSize, true, (prototype.get("sphericalElevation").type() == e57::E57_SCALED_INTEGER));
		}

		//data validity
		if (header.pointFields.sphericalInvalidStateField)
		{
			arrays.isInvalidData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "sphericalInvalidState", arrays.isInvalidData.data(), chunkSize, true, (prototype.get("sphericalInvalidState").type() == e57::E57_SCALED_INTEGER));
		}
	}
	else
	{
		//cartesian coordinates
		if (header.pointFields.cartesianXField)
		{
			arrays.xData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "cartesianX", arrays.xData.data(), chunkSize, true, (prototype.get("cartesianX").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.cartesianYField)
		{
			arrays.yData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "cartesianY", arrays.yData.data(), chunkSize, true, (prototype.get("cartesianY").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.cartesianZField)
		{
			arrays.zData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "cartesianZ", arrays.zData.data(), chunkSize, true, (prototype.get("cartesianZ").type() == e57::E57_SCALED_INTEGER));
		}

		//data validity
		if (header.pointFields.cartesianInvalidStateField)
		{
			arrays.isInvalidData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "cartesianInvalidState", arrays.isInvalidData.data(), chunkSize, true, (prototype.get("cartesianInvalidState").type() == e57::E57_SCALED_INTEGER));
		}
	}

	//normals
	bool hasNormals = (		header.pointFields.normXField
						||	header.pointFields.normYField
						||	header.pointFields.normZField);
	if (hasNormals)
	{
		if (!cloud->reserveTheNormsTable())
		{
			ccLog::Error("[E57] Not enough memory!");
			delete cloud;
			return {};
		}
		cloud->showNormals(true);
		if (header.pointFields.normXField)
		{
			arrays.xNormData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "nor:normalX", arrays.xNormData.data(), chunkSize, true, (prototype.get("nor:normalX").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.normYField)
		{
			arrays.yNormData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "nor:normalY", arrays.yNormData.data(), chunkSize, true, (prototype.get("nor:normalY").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.normZField)
		{
			arrays.zNormData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "nor:normalZ", arrays.zNormData.data(), chunkSize, true, (prototype.get("nor:normalZ").type() == e57::E57_SCALED_INTEGER));
		}
	}

	//intensity
	//double intRange = 0;
	//double intOffset = 0;
	//ScalarType invalidSFValue = 0;

	ccScalarField* intensitySF = nullptr;
	if (header.pointFields.intensityField)
	{
		intensitySF = new ccScalarField(CC_E57_INTENSITY_FIELD_NAME);
		if (!intensitySF->resizeSafe(static_cast<unsigned>(pointCount)))
		{
			ccLog::Error("[E57] Not enough memory!");
			intensitySF->release();
			delete cloud;
			return {};
		}
		cloud->addScalarField(intensitySF);

		arrays.intData.resize(chunkSize);
		dbufs.emplace_back(node.destImageFile(), "intensity", arrays.intData.data(), chunkSize, true, (prototype.get("intensity").type() == e57::E57_SCALED_INTEGER));
		//intRange = header.intensityLimits.intensityMaximum - header.intensityLimits.intensityMinimum;
		//intOffset = header.intensityLimits.intensityMinimum;

		if (header.pointFields.isIntensityInvalidField)
		{
			arrays.isInvalidIntData.resize(chunkSize);
			dbufs.emplace_back(node.destImageFile(), "isIntensityInvalid", arrays.isInvalidIntData.data(), chunkSize, true, (prototype.get("isIntensityInvalid").type() == e57::E57_SCALED_INTEGER));
		}
	}

	//color buffers
	double colorRedRange = 1;
	double colorRedOffset = 0;
	double colorGreenRange = 1;
	double colorGreenOffset = 0;
	double colorBlueRange = 1;
	double colorBlueOffset = 0;
	bool hasColors = (	header.pointFields.colorRedField
					||	header.pointFields.colorGreenField
					||	header.pointFields.colorBlueField);
	if (hasColors)
	{
		if (!cloud->reserveTheRGBTable())
		{
			ccLog::Error("[E57] Not enough memory!");
			delete cloud;
			return {};
		}
		if (header.pointFields.colorRedField)
		{
			arrays.redData.resize(chunkSize);
			colorRedOffset = header.colorLimits.colorRedMinimum;
			colorRedRange = header.colorLimits.colorRedMaximum - header.colorLimits.colorRedMinimum;
			if (colorRedRange <= 0.0)
				colorRedRange = 1.0;
			dbufs.emplace_back(node.destImageFile(), "colorRed", arrays.redData.data(), chunkSize, true, (prototype.get("colorRed").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.colorGreenField)
		{
			arrays.greenData.resize(chunkSize);
			colorGreenOffset = header.colorLimits.colorGreenMinimum;
			colorGreenRange = header.colorLimits.colorGreenMaximum - header.colorLimits.colorGreenMinimum;
			if (colorGreenRange <= 0.0)
				colorGreenRange = 1.0;
			dbufs.emplace_back(node.destImageFile(), "colorGreen", arrays.greenData.data(), chunkSize, true, (prototype.get("colorGreen").type() == e57::E57_SCALED_INTEGER));
		}
		if (header.pointFields.colorBlueField)
		{
			arrays.blueData.resize(chunkSize);
			colorBlueOffset = header.colorLimits.colorBlueMinimum;
			colorBlueRange = header.colorLimits.colorBlueMaximum - header.colorLimits.colorBlueMinimum;
			if (colorBlueRange <= 0.0)
				colorBlueRange = 1.0;
			dbufs.emplace_back(node.destImageFile(), "colorBlue", arrays.blueData.data(), chunkSize, true, (prototype.get("colorBlue").type() == e57::E57_SCALED_INTEGER));
		}
	}

	//return index (multiple shoots scanners)
	ccScalarField* returnIndexSF = nullptr;
	if (header.pointFields.returnIndexField && header.pointFields.returnMaximum > 0)
	{
		//we store the point return index as a scalar field
		returnIndexSF = new ccScalarField(CC_E57_RETURN_INDEX_FIELD_NAME);
		if (!returnIndexSF->resizeSafe(static_cast<unsigned>(pointCount)))
		{
			ccLog::Error("[E57] Not enough memory!");
			delete cloud;
			returnIndexSF->release();
			return {};
		}
		cloud->addScalarField(returnIndexSF);
		arrays.scanIndexData.resize(chunkSize);
		dbufs.emplace_back(node.destImageFile(), "returnIndex", arrays.scanIndexData.data(), chunkSize, true, (prototype.get("returnIndex").type() == e57::E57_SCALED_INTEGER));
	}

	//Read the point data
	e57::CompressedVectorReader dataReader = points.reader(dbufs);

	//local progress bar
	CCCoreLib::NormalizedProgress nprogress(progressDlg, static_cast<unsigned>(pointCount / chunkSize));
	if (progressDlg)
	{
		progressDlg->setMethodTitle(QObject::tr("Read E57 file"));
		progressDlg->setInfo(QObject::tr("Scan #%1 - %2 points").arg(s_absoluteScanIndex).arg(pointCount));
		progressDlg->start();
		QApplication::processEvents();
	}

	CCVector3d Pshift(0, 0, 0);
	unsigned size = 0;
	int64_t realCount = 0;
	int64_t invalidCount = 0;
	int64_t zeroCount = 0;
	int col = 0, row = 0;
	while ((size = dataReader.read()))
	{
		for (unsigned i = 0; i < size; ++i)
		{
			if (scanGrid)
			{
				col = arrays.columnIndex[i];
				row = arrays.rowIndex[i];
			}

			//we skip invalid points!
			if (!arrays.isInvalidData.empty() && arrays.isInvalidData[i] != 0)
			{
				++invalidCount;
				if (scanGrid)
				{
					scanGrid->setIndex(row, col, -1);
				}
				continue;
			}

			CCVector3d Pd(0, 0, 0);
			if (sphericalMode)
			{
				double r = (arrays.xData.empty() ? 0 : arrays.xData[i]);
				double theta = (arrays.yData.empty() ? 0 : arrays.yData[i]);	//Azimuth
				double phi = (arrays.zData.empty() ? 0 : arrays.zData[i]);		//Elevation

				double cos_phi = cos(phi);
				Pd.x = r * cos_phi * cos(theta);
				Pd.y = r * cos_phi * sin(theta);
				Pd.z = r * sin(phi);
			}
			//DGM TODO: not handled yet (-->what are the standard cylindrical field names?)
			/*else if (cylindricalMode)
			{
				//from cylindrical coordinates
				assert(arrays.xData);
				double theta = (arrays.yData ? arrays.yData[i] : 0);
				Pd.x = arrays.xData[i] * cos(theta);
				Pd.y = arrays.xData[i] * sin(theta);
				if (arrays.zData)
					Pd.z = arrays.zData[i];
			}
			//*/
			else //cartesian
			{
				if (!arrays.xData.empty())
					Pd.x = arrays.xData[i];
				if (!arrays.yData.empty())
					Pd.y = arrays.yData[i];
				if (!arrays.zData.empty())
					Pd.z = arrays.zData[i];
			}

			if (Pd.x == 0 && Pd.y == 0 && Pd.z == 0)
			{
				++zeroCount;
			}

			//first point: check for 'big' coordinates
			if (realCount == 0 && !poseMatWasShifted)
			{
				if (FileIOFilter::HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, s_loadParameters))
				{
					globalShiftApplied = true;
					if (preserveCoordinateShift)
					{
						cloud->setGlobalShift(Pshift);
					}
					ccLog::Warning("[E57Filter::loadFile] Cloud %s has been recentered! Translation: (%.2f ; %.2f ; %.2f)", qPrintable(guidStr), Pshift.x, Pshift.y, Pshift.z);
				}
			}

			if (scanGrid)
			{
				scanGrid->setIndex(row, col, static_cast<int>(cloud->size()));
			}

			const CCVector3 P = (Pd + Pshift).toPC();
			cloud->addPoint(P);

			if (hasNormals)
			{
				CCVector3 N(0, 0, 0);
				if (!arrays.xNormData.empty())
					N.x = static_cast<PointCoordinateType>(arrays.xNormData[i]);
				if (!arrays.yNormData.empty())
					N.y = static_cast<PointCoordinateType>(arrays.yNormData[i]);
				if (!arrays.zNormData.empty())
					N.z = static_cast<PointCoordinateType>(arrays.zNormData[i]);
				N.normalize();
				cloud->addNorm(N);
			}

			if (!arrays.intData.empty())
			{
				assert(intensitySF);
				if (!header.pointFields.isIntensityInvalidField || arrays.isInvalidIntData[i] != INVALID_DATA)
				{
					//ScalarType intensity = (ScalarType)((arrays.intData[i] - intOffset)/intRange); //Normalize intensity to 0 - 1.
					const ScalarType intensity = static_cast<ScalarType>(arrays.intData[i]);
					intensitySF->setValue(static_cast<unsigned>(realCount), intensity);

					//track max intensity (for proper visualization)
					if (s_absoluteScanIndex != 0 || realCount != 0)
					{
						if (s_maxIntensity < intensity)
							s_maxIntensity = intensity;
						else if (s_minIntensity > intensity)
							s_minIntensity = intensity;
					}
					else
					{
						s_maxIntensity = s_minIntensity = intensity;
					}
				}
				else
				{
					intensitySF->flagValueAsInvalid(static_cast<unsigned>(realCount));
				}
			}

			if (hasColors)
			{
				//Normalize color to 0 - 255
				ccColor::Rgb C(0, 0, 0);
				if (!arrays.redData.empty())
					C.r = static_cast<ColorCompType>(((arrays.redData[i] - colorRedOffset) * 255) / colorRedRange);
				if (!arrays.greenData.empty())
					C.g = static_cast<ColorCompType>(((arrays.greenData[i] - colorGreenOffset) * 255) / colorGreenRange);
				if (!arrays.blueData.empty())
					C.b = static_cast<ColorCompType>(((arrays.blueData[i] - colorBlueOffset) * 255) / colorBlueRange);

				cloud->addColor(C);
			}

			if (!arrays.scanIndexData.empty())
			{
				assert(returnIndexSF);
				const ScalarType s = static_cast<ScalarType>(arrays.scanIndexData[i]);
				returnIndexSF->setValue(static_cast<unsigned>(realCount), s);
			}

			realCount++;
		}

		if (progressDlg && !nprogress.oneStep())
		{
			QApplication::processEvents();
			s_cancelRequestedByUser = true;
			break;
		}
	}

	dataReader.close();

	if (zeroCount > 1)
	{
		ccLog::Warning(QString("[E57] Number of points clustured at the origin: %1 (consider removing duplicate points)").arg(zeroCount));
	}

	if (realCount == 0)
	{
		ccLog::Warning(QString("[E57] No valid point in scan '%1'!").arg(scanNode.elementName().c_str()));
		delete cloud;
		return {};
	}
	else if (realCount < pointCount)
	{
		if ( (realCount + invalidCount) != pointCount )
		{
			ccLog::Warning(QString("[E57] We read fewer points than expected for scan '%1' (%2/%3)").arg(scanNode.elementName().c_str()).arg(realCount).arg(pointCount));
		}
		
		cloud->resize(static_cast<unsigned>(realCount));
	}

	//Scan grid
	if (scanGrid)
	{
		scanGrid->validCount = realCount;
		scanGrid->minValidIndex = 0;
		scanGrid->maxValidIndex = realCount - 1;
		cloud->addGrid(scanGrid);

		ccLog::Print(QString("[E57] Scan grid loaded for scan '%1' (%2 x %3)").arg(scanNode.elementName().c_str()).arg(scanGrid->w).arg(scanGrid->h));
	}

	//Scalar fields
	if (intensitySF)
	{
		intensitySF->computeMinAndMax();
		if (intensitySF->getMin() >= 0 && intensitySF->getMax() <= 1.0)
			intensitySF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::ABS_NORM_GREY));
		else
			intensitySF->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
		cloud->setCurrentDisplayedScalarField(cloud->getScalarFieldIndexByName(intensitySF->getName()));
		cloud->showSF(true);
	}

	if (returnIndexSF)
	{
		returnIndexSF->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(cloud->getScalarFieldIndexByName(returnIndexSF->getName()));
		ccLog::Warning("[E57] Cloud has multiple echoes: use 'Edit > Scalar Fields > Filter by value' to extract one component");
		cloud->showSF(true);
	}

	cloud->showColors(hasColors);
	cloud->setVisible(true);

	//we don't deal with virtual transformation (yet)
	if (validPoseMat)
	{
		const ccGLMatrix poseMatf(poseMat.data());
		
		cloud->applyGLTransformation_recursive(&poseMatf);
		//this transformation is of no interest for the user
		cloud->resetGLTransformationHistory_recursive();

		//save the original pose matrix as meta-data
		cloud->setMetaData(s_e57PoseKey, poseMat.toString(12, ' '));
	}

	if (sensor) //add the sensor at the end, after calling applyGLTransformation_recursive!
	{
		sensor->setEnabled(false);
		sensor->setVisible(true);
		sensor->setGraphicScale(cloud->getOwnBB().getDiagNorm() / 20);
		cloud->addChild(sensor);
	}

	return { cloud, globalShiftApplied, poseMatWasShifted ? poseMatShift : Pshift, preserveCoordinateShift };
}

//! Loaded image
struct LoadedImage
{
	ccImage* entity = nullptr;
	ccGLMatrixd poseMat;
	bool validPoseMat = false;
	ccCameraSensor* sensor = nullptr;
};

static LoadedImage LoadImage(const e57::Node& node, QString& associatedData3DGuid)
{
	if (node.type() != e57::E57_STRUCTURE)
	{
		ccLog::Warning("[E57Filter] Image nodes should be STRUCTURES!");
		return {};
	}
	e57::StructureNode imageNode(node);

	QString imageName = GetStringFromNode(imageNode, "name", "unnamed");

	//Collect generic node information
	E57NodeMap nodeInfo;
	{
		AddToE57NodeMap(nodeInfo, imageNode, "sensorVendor");
		AddToE57NodeMap(nodeInfo, imageNode, "sensorModel");
		AddToE57NodeMap(nodeInfo, imageNode, "sensorSerialNumber");
		AddToE57NodeMap(nodeInfo, imageNode, "sensorHardwareVersion");
		AddToE57NodeMap(nodeInfo, imageNode, "sensorSoftwareVersion");
		AddToE57NodeMap(nodeInfo, imageNode, "sensorFirmwareVersion");
		AddToE57NodeMap(nodeInfo, imageNode, "temperature");
		AddToE57NodeMap(nodeInfo, imageNode, "relativeHumidity");
		AddToE57NodeMap(nodeInfo, imageNode, "atmosphericPressure");
		AddToE57NodeMap(nodeInfo, imageNode, "acquisitionStart");
		AddToE57NodeMap(nodeInfo, imageNode, "acquisitionEnd");
	}

	//log
	ccLog::Print(QString("[E57] Reading new image node (%1) - %2").arg(imageNode.elementName().c_str()).arg(imageName));

	if (imageNode.isDefined("description"))
	{
		ccLog::Print(QString("[E57] Description: %1").arg(e57::StringNode(imageNode.get("description")).value().c_str()));
	}

	if (imageNode.isDefined("associatedData3DGuid"))
	{
		associatedData3DGuid = e57::StringNode(imageNode.get("associatedData3DGuid")).value().c_str();
	}
	else
	{
		associatedData3DGuid.clear();
	}

	//output
	LoadedImage output;

	//get pose information
	output.validPoseMat = GetPoseInformation(imageNode, output.poseMat);

	//camera information
	QSharedPointer<VisualReferenceRepresentation> cameraRepresentation(nullptr);
	if (imageNode.isDefined(VisualReferenceRepresentation::GetName()))
	{
		cameraRepresentation.reset(new VisualReferenceRepresentation);
	}
	else if (imageNode.isDefined(PinholeRepresentation::GetName()))
	{
		cameraRepresentation.reset(new PinholeRepresentation);
	}
	else if (imageNode.isDefined(SphericalRepresentation::GetName()))
	{
		cameraRepresentation.reset(new SphericalRepresentation);
	}
	else if (imageNode.isDefined(CylindricalRepresentation::GetName()))
	{
		cameraRepresentation.reset(new CylindricalRepresentation);
	}
	else
	{
		ccLog::Warning(QString("[E57] Image %1 has no or an unknown associated camera representation!").arg(imageName));
		return {};
	}
	assert(cameraRepresentation);

	Image2DProjection cameraType = cameraRepresentation->getType();

	assert(cameraType != E57_NO_PROJECTION);
	e57::StructureNode cameraRepresentationNode(imageNode.get(e57::ustring(cameraRepresentation->getName())));

	//read standard image information
	cameraRepresentation->imageType = E57_NO_IMAGE;
	cameraRepresentation->imageMaskSize = 0;

	if (cameraRepresentationNode.isDefined("jpegImage"))
	{
		cameraRepresentation->imageType = E57_JPEG_IMAGE;
		cameraRepresentation->imageSize = e57::BlobNode(cameraRepresentationNode.get("jpegImage")).byteCount();
	}
	else if (cameraRepresentationNode.isDefined("pngImage"))
	{
		cameraRepresentation->imageType = E57_PNG_IMAGE;
		cameraRepresentation->imageSize = e57::BlobNode(cameraRepresentationNode.get("pngImage")).byteCount();
	}
	else
	{
		ccLog::Warning("[E57] Unhandled image format (only JPG and PNG are currently supported)");
		return {};
	}

	if (cameraRepresentation->imageSize == 0)
	{
		ccLog::Warning("[E57] Invalid image size!");
		return {};
	}

	uint8_t* imageBits = new uint8_t[cameraRepresentation->imageSize];
	if (!imageBits)
	{
		ccLog::Warning("[E57] Not enough memory to load image!");
		return {};
	}
	
	if (cameraRepresentationNode.isDefined("imageMask"))
	{
		cameraRepresentation->imageMaskSize = e57::BlobNode(cameraRepresentationNode.get("imageMask")).byteCount();
	}

	cameraRepresentation->imageHeight = static_cast<int32_t>(e57::IntegerNode(cameraRepresentationNode.get("imageHeight")).value());
	cameraRepresentation->imageWidth = static_cast<int32_t>(e57::IntegerNode(cameraRepresentationNode.get("imageWidth")).value());

	//Pixel size
	switch (cameraType)
	{
	case E57_PINHOLE:
	case E57_CYLINDRICAL:
	case E57_SPHERICAL:
		{
			SphericalRepresentation* spherical = static_cast<SphericalRepresentation*>(cameraRepresentation.data());
			spherical->pixelHeight = e57::FloatNode(cameraRepresentationNode.get("pixelHeight")).value();
			spherical->pixelWidth = e57::FloatNode(cameraRepresentationNode.get("pixelWidth")).value();
		}
		break;

	case E57_NO_PROJECTION:
	case E57_VISUAL:
		break;
	}

	if (cameraType == E57_PINHOLE)
	{
		PinholeRepresentation* pinhole = static_cast<PinholeRepresentation*>(cameraRepresentation.data());

		pinhole->focalLength = e57::FloatNode(cameraRepresentationNode.get("focalLength")).value();
		pinhole->principalPointX = e57::FloatNode(cameraRepresentationNode.get("principalPointX")).value();
		pinhole->principalPointY = e57::FloatNode(cameraRepresentationNode.get("principalPointY")).value();
	}
	else if (cameraType == E57_CYLINDRICAL)
	{
		CylindricalRepresentation* cylindrical = static_cast<CylindricalRepresentation*>(cameraRepresentation.data());

		cylindrical->principalPointY = e57::FloatNode(cameraRepresentationNode.get("principalPointY")).value();
		cylindrical->radius = e57::FloatNode(cameraRepresentationNode.get("radius")).value();
	}

	//reading image data
	char imageFormat[4] = "jpg";
	switch (cameraRepresentation->imageType)
	{
	case E57_JPEG_IMAGE:
		{
			assert(cameraRepresentationNode.isDefined("jpegImage"));
			e57::BlobNode jpegImage(cameraRepresentationNode.get("jpegImage"));
			jpegImage.read(imageBits, 0, static_cast<size_t>(cameraRepresentation->imageSize));
			break;
		}
	case E57_PNG_IMAGE:
		{
			strcpy(imageFormat, "png");
			assert(cameraRepresentationNode.isDefined("pngImage"));
			e57::BlobNode pngImage(cameraRepresentationNode.get("pngImage"));
			pngImage.read((uint8_t*)imageBits, 0, static_cast<size_t>(cameraRepresentation->imageSize));
			break;
		}
	default:
		assert(false);
		break;
	}

	//handle mask?
	if (cameraRepresentation->imageMaskSize > 0)
	{
		assert(cameraRepresentationNode.isDefined("imageMask"));
		e57::BlobNode imageMask(cameraRepresentationNode.get("imageMask"));
		//imageMask.read((uint8_t*)pBuffer, start, (size_t) count);
		//DGM: TODO
		cameraRepresentation->imageMaskSize = 0;
	}

	QImage qImage;
	assert(imageBits);
	bool loadResult = qImage.loadFromData(imageBits, static_cast<int>(cameraRepresentation->imageSize), imageFormat);

	// a bug in some 2.13.alpha versions was causing CC to save JPEG images declared as PNG images :(
	if (!loadResult && cameraRepresentation->imageType == E57_PNG_IMAGE)
	{
		loadResult = qImage.loadFromData(imageBits, static_cast<int>(cameraRepresentation->imageSize), "jpegImage");
		ccLog::Warning("[E57] JPG image was wrongly tagged as PNG. You should save this E57 again to fix this...");
	}

	delete[] imageBits;
	imageBits = nullptr;

	if (!loadResult)
	{
		ccLog::Warning("[E57] Failed to load image from blob data!");
		return {};
	}

	switch(cameraType)
	{
	case E57_CYLINDRICAL:
	case E57_SPHERICAL:
		ccLog::Warning("[E57] Unhandled camera type (image will be loaded as is)");
#if (QT_VERSION >= QT_VERSION_CHECK(5, 8, 0))
		Q_FALLTHROUGH();
#endif
	case E57_VISUAL:
		output.entity = new ccImage();
		break;
	case E57_PINHOLE:
		{
			PinholeRepresentation* pinhole = static_cast<PinholeRepresentation*>(cameraRepresentation.data());
			float focal_mm        = static_cast<float>(pinhole->focalLength * 1000.0);
			float pixelWidth_mm   = static_cast<float>(pinhole->pixelWidth * 1000.0);
			float pixelHeight_mm  = static_cast<float>(pinhole->pixelHeight * 1000.0);
			float ccdHeight_mm    = static_cast<float>(pinhole->imageHeight * pixelHeight_mm);
			
			ccCameraSensor::IntrinsicParameters params;
			params.vertFocal_pix      = ccCameraSensor::ConvertFocalMMToPix(focal_mm, pixelHeight_mm);
			params.arrayWidth         = pinhole->imageWidth;
			params.arrayHeight        = pinhole->imageHeight;
			params.principal_point[0] = static_cast<float>(pinhole->principalPointX);
			params.principal_point[1] = static_cast<float>(pinhole->principalPointY);
			params.pixelSize_mm[0]    = pixelWidth_mm;
			params.pixelSize_mm[1]    = pixelHeight_mm;
			params.vFOV_rad           = ccCameraSensor::ComputeFovRadFromFocalMm(focal_mm, ccdHeight_mm);
			
			output.sensor = new ccCameraSensor(params);
			if (output.validPoseMat)
			{
				ccGLMatrix poseMatf(output.poseMat.data()); //we'll use it as is for now (will be updated later with a potential Global Shift)
				output.sensor->setRigidTransformation(poseMatf);
			}

			output.sensor->setEnabled(false);
			output.sensor->setVisible(true);

			output.entity = new ccImage();
			output.entity->addChild(output.sensor);
			output.entity->setAssociatedSensor(output.sensor);
		}
		break;

	case E57_NO_PROJECTION:
		assert(false);
		break;
	}

	if (output.entity)
	{
		output.entity->setData(qImage);
		output.entity->setName(imageName);
		output.entity->setMetaData(s_e57NodeInfoKey, nodeInfo.toStringList().join("\n"));
		output.entity->setMetaData(s_e57CameraInfoKey, cameraRepresentation->toStringList().join("\n"));

		//save the original pose matrix as meta-data
		if (output.validPoseMat)
		{
			output.entity->setMetaData(s_e57PoseKey, output.poseMat.toString(12, ' ')); //we'll use it as is for now (will be updated later with a potential Global Shift)
		}

		//don't forget image aspect ratio
		if (cameraType == E57_CYLINDRICAL ||
			cameraType == E57_PINHOLE ||
			cameraType == E57_SPHERICAL)
		{
			SphericalRepresentation* spherical = static_cast<SphericalRepresentation*>(cameraRepresentation.data());
			output.entity->setAspectRatio(static_cast<float>(spherical->pixelWidth / spherical->pixelHeight) * output.entity->getAspectRatio());
		}
	}

	return output;
}

CC_FILE_ERROR E57Filter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	s_loadParameters = parameters;
	
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	try
	{
		e57::ImageFile imf(filename.toStdString(), "r", e57::CHECKSUM_POLICY_SPARSE);
		
		if (!imf.isOpen())
		{
			return CC_FERR_READING;
		}

		//for normals handling
		static const e57::ustring normalsExtension("http://www.libe57.org/E57_NOR_surface_normals.txt");
		e57::ustring _normalsExtension;
		if (!imf.extensionsLookupPrefix("nor", _normalsExtension)) //the extension may already be registered
		{
			imf.extensionsAdd("nor", normalsExtension);
		}

		e57::StructureNode root = imf.root();

		//header info
		e57::StructureNode rootStruct = e57::StructureNode(root);
		
		if (!ChildNodeToConsole(rootStruct,"formatName") ||
				!ChildNodeToConsole(rootStruct,"guid") ||
				!ChildNodeToConsole(rootStruct,"versionMajor") ||
				!ChildNodeToConsole(rootStruct,"versionMinor"))
		{
			imf.close();
			return CC_FERR_MALFORMED_FILE;
		}
		
		//unroll structure in tree (it's a quick to check structure + informative for user)
		ccHObject* fileStructureTree = new ccHObject("File structure");
		if (!NodeStructureToTree(fileStructureTree, rootStruct))
		{
			imf.close();
			return CC_FERR_MALFORMED_FILE;
		}
		container.addChild(fileStructureTree);

		//we store (temporarily) the loaded scans associated with
		//their unique GUID in a map (to retrieve them later if
		//necessary - for example to associate them with images)
		QMap<QString, LoadedScan> scans;

		//3D data?
		if (root.isDefined("/data3D"))
		{
			e57::Node n = root.get("/data3D"); //E57 standard: "data3D is a vector for storing an arbitrary number of 3D data sets "
			if (n.type() != e57::E57_VECTOR)
			{
				imf.close();
				return CC_FERR_MALFORMED_FILE;
			}
			e57::VectorNode data3D(n);

			unsigned scanCount = static_cast<unsigned>(data3D.childCount());

			//global progress bar
			QScopedPointer<ccProgressDialog> progressDlg(nullptr);
			if (parameters.parentWidget)
			{
				progressDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
				progressDlg->setAutoClose(false);
			}

			bool showGlobalProgress = (scanCount > 10);
			if (progressDlg && showGlobalProgress)
			{
				//Too many scans, will display a global progress bar
				progressDlg->setMethodTitle(QObject::tr("Read E57 file"));
				progressDlg->setInfo(QObject::tr("Scans: %1").arg(scanCount));
				progressDlg->start();
				QApplication::processEvents();
			}
			CCCoreLib::NormalizedProgress nprogress(progressDlg.data(), showGlobalProgress ? scanCount : 100);

			//static states
			s_absoluteScanIndex = 0;
			s_cancelRequestedByUser = false;
			s_minIntensity = s_maxIntensity = 0;
			for (unsigned i = 0; i < scanCount; ++i)
			{
				const e57::Node scanNode = data3D.get(i);
				QString scanGUID;
				
				LoadedScan scan = LoadScan(scanNode, scanGUID, showGlobalProgress ? nullptr : progressDlg.data());
				
				if (scan.entity)
				{
					if (scan.entity->getName().isEmpty())
					{
						QString name("Scan ");
						e57::ustring nodeName = scanNode.elementName();

						if (!nodeName.empty())
							name += QString::fromStdString(nodeName);
						else
							name += QString::number(i);

						scan.entity->setName(name);
					}
					container.addChild(scan.entity);

					//we also add the scan to the GUID/object map
					if (!scanGUID.isEmpty())
					{
						scans.insert(scanGUID, scan);
					}
				}
				
				if ((showGlobalProgress && progressDlg && !nprogress.oneStep()) || s_cancelRequestedByUser)
				{
					break;
				}
				++s_absoluteScanIndex;
			}

			if (progressDlg)
			{
				progressDlg->stop();
				QApplication::processEvents();
			}

			//set global max intensity (saturation) for proper display
			for (unsigned i = 0; i < container.getChildrenNumber(); ++i)
			{
				if (container.getChild(i)->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* pc = static_cast<ccPointCloud*>(container.getChild(i));
					ccScalarField* sf = pc->getCurrentDisplayedScalarField();
					if (sf)
					{
						sf->setSaturationStart(s_minIntensity);
						sf->setSaturationStop(s_maxIntensity);
					}
				}
			}
		}

		//we save parameters
		parameters = s_loadParameters;

		//Image data?
		if (!s_cancelRequestedByUser && root.isDefined("/images2D"))
		{
			e57::Node n = root.get("/images2D"); //E57 standard: "images2D is a vector for storing two dimensional images"
			if (n.type() != e57::E57_VECTOR)
			{
				imf.close();
				return CC_FERR_MALFORMED_FILE;
			}
			
			e57::VectorNode images2D(n);

			unsigned imageCount = static_cast<unsigned>(images2D.childCount());
			if (imageCount)
			{
				//progress bar
				QScopedPointer<ccProgressDialog> progressDlg(nullptr);
				if (parameters.parentWidget)
				{
					progressDlg.reset(new ccProgressDialog(true, parameters.parentWidget));
					progressDlg->setMethodTitle(QObject::tr("Read E57 file"));
					progressDlg->setInfo(QObject::tr("Images: %1").arg(imageCount));
					progressDlg->start();
					QApplication::processEvents();
				}
				CCCoreLib::NormalizedProgress nprogress(progressDlg.data(), imageCount);

				for (unsigned i = 0; i < imageCount; ++i)
				{
					e57::Node imageNode = images2D.get(i);
					QString associatedData3DGuid;
					LoadedImage image = LoadImage(imageNode, associatedData3DGuid);
					if (image.entity)
					{
						//no name?
						if (image.entity->getName().isEmpty())
						{
							QString name("Image");
							e57::ustring nodeName = imageNode.elementName();
							if (!nodeName.empty())
								name += QString::fromStdString(nodeName);
							else
								name += QString::number(i);
							image.entity->setName(name);
						}
						image.entity->setEnabled(false); //not displayed by default

						//existing link to a loaded scan?
						LoadedScan parentScan;
						if (!associatedData3DGuid.isEmpty() && scans.contains(associatedData3DGuid))
						{
							parentScan = scans.value(associatedData3DGuid);
						}

						if (parentScan.entity)
						{
							parentScan.entity->addChild(image.entity);
						}
						else // no parent scan
						{
							container.addChild(image.entity);
						}

						if (image.validPoseMat)
						{
							bool applySensorGlobalShift = false;
							bool preserveCoordinateShift = true;
							CCVector3d poseMatShift(0, 0, 0);

							if (parentScan.entity)
							{
								//we need to apply the scan Global Shift to the sensor (if any)
								if (parentScan.globalShiftApplied)
								{
									applySensorGlobalShift = true;
									poseMatShift = parentScan.globalShift;
									preserveCoordinateShift = parentScan.preserveCoordinateShift;
								}
							}
							else // no parent scan
							{
								//we may have to apply a Gloal Shift to the sensor (if any)
								const CCVector3d T = image.poseMat.getTranslationAsVec3D();
								applySensorGlobalShift = FileIOFilter::HandleGlobalShift(T, poseMatShift, preserveCoordinateShift, s_loadParameters);

								if (image.sensor && preserveCoordinateShift)
								{
									image.sensor->setMetaData("Global Shift X", poseMatShift.x);
									image.sensor->setMetaData("Global Shift Y", poseMatShift.y);
									image.sensor->setMetaData("Global Shift Z", poseMatShift.z);
								}
							}

							if (applySensorGlobalShift)
							{
								image.poseMat.setTranslation((image.poseMat.getTranslationAsVec3D() + poseMatShift).u);

								if (image.sensor)
								{
									ccGLMatrix poseMatf(image.poseMat.data());
									image.sensor->setRigidTransformation(poseMatf);

									ccLog::Warning("[E57Filter::loadFile] The sensor of image %s has been recentered! Translation: (%.2f ; %.2f ; %.2f)", qPrintable(image.entity->getName()), poseMatShift.x, poseMatShift.y, poseMatShift.z);
								}
							}

							//save the potentially shifted pose matrix as meta-data
							image.entity->setMetaData(s_e57PoseKey, image.poseMat.toString(12, ' '));
						}
					}

					if (progressDlg && !nprogress.oneStep())
					{
						s_cancelRequestedByUser = true;
						break;
					}
				}
			}
		}
		
		imf.close();		
	}
	catch (const e57::E57Exception& e)
	{
		ccLog::Warning(QString("[E57] Error: %1").arg(e57::Utilities::errorCodeToString(e.errorCode()).c_str()));
		
		if ( !e.context().empty() )
		{
			ccLog::Warning( QStringLiteral("    context: %1").arg( QString::fromStdString( e.context() ) ) );
		}

		result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}
	catch(...)
	{
		ccLog::Warning("[E57] Unknown error");
		result = CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
	}

	//special case: process has been cancelled by user
	if (result == CC_FERR_NO_ERROR && s_cancelRequestedByUser)
	{
		result = CC_FERR_CANCELED_BY_USER;
	}

	return result;
}
