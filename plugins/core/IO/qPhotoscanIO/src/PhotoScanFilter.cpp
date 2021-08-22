//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qPhotoScanIO                    #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "PhotoScanFilter.h"

//Qt
#include <QTextStream>
#include <QXmlStreamReader>
#include <QStringRef>
#include <QFileInfo>
#include <QDir>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObject.h>
#include <ccCameraSensor.h>
#include <ccImage.h>
#include <ccLog.h>
#include <ccProgressDialog.h>

//qCC_io
#include <PlyFilter.h>

//quazip
#include <quazip.h>
#include <quazipfile.h>

//System
#include <string.h>
#include <assert.h>

struct CameraDesc
{
	CameraDesc()
		: id(-1)
		, sensorId(-1)
	{}

	ccGLMatrix trans;
	QString imageFilename;
	int id, sensorId;
};

struct CloudDesc
{
	QString filename;
	QString type;
};

struct MeshDesc
{
	QString filename;
	QString texture;
};

enum Sections { DOCUMENT, CHUNKS, CHUNK, SENSORS, CAMERAS, FRAMES, FRAME, TRANSFORM };

QString ToName(Sections section)
{
	switch (section)
	{
	case DOCUMENT:
		return "DOCUMENT";
	case CHUNKS:
		return "CHUNKS";
	case CHUNK:
		return "CHUNK";
	case SENSORS:
		return "SENSORS";
	case CAMERAS:
		return "CAMERAS";
	case FRAMES:
		return "FRAMES";
	case FRAME:
		return "FRAME";
	case TRANSFORM:
		return "TRANSFORM";
	default:
		assert(false);
	}

	return QString();
}

template<typename T> bool DecodeRotation(const QString& rotationValues, ccGLMatrixTpl<T>& output)
{
	QStringList tokens = rotationValues.split(" ", QString::SkipEmptyParts);
	if (tokens.size() != 9)
	{
		return false;
	}

	T* m = output.data();
	for (int i = 0; i < 9; ++i)
	{
		int col = (i / 3);
		int row = (i % 3);
		bool ok = true;
		m[col * 4 + row] = static_cast<T>(tokens[i].toDouble(&ok));
		if (!ok)
		{
			//invalid input string
			return false;
		}
	}
	output.transpose();

	return true;
}

template<typename T> bool DecodeTransformation(const QString& transformationValues, ccGLMatrixTpl<T>& output)
{
	QStringList tokens = transformationValues.split(" ", QString::SkipEmptyParts);
	if (tokens.size() != 16)
	{
		return false;
	}

	T* m = output.data();
	for (int i = 0; i < 16; ++i)
	{
		bool ok = true;
		m[i] = static_cast<T>(tokens[i].toDouble(&ok));
		if (!ok)
		{
			//invalid input string
			return false;
		}
	}
	output.transpose();

	return true;
}

static void DisplayCurrentNodeInfo(QXmlStreamReader& stream)
{
	ccLog::Warning("--> " + stream.name().toString() + (stream.isStartElement() ? " [start]" : "") + (stream.isEndElement() ? " [end]" : ""));
	for (int i = 0; i < stream.attributes().size(); ++i)
	{
		ccLog::Warning(QString("\t") + stream.attributes().at(i).qualifiedName().toString());
	}
}

static ccCameraSensor* DecodeSensor(QXmlStreamReader& stream, int& sensorId)
{
	assert(stream.name() == "sensor");
	sensorId = -1;

	QXmlStreamAttributes sensorAttributes = stream.attributes();
	if (!sensorAttributes.hasAttribute("type") || sensorAttributes.value("type") != "frame")
	{
		//unhandled sensor type
		return nullptr;
	}
	if (!sensorAttributes.hasAttribute("id"))
	{
		//invalid sensor?!
		assert(false);
    return nullptr;
	}
	sensorId = sensorAttributes.value("id").toInt();

	ccCameraSensor* sensor = nullptr;
	ccCameraSensor::IntrinsicParameters params;
	bool hasPixelSize = false;

	while (stream.readNextStartElement())
	{
#ifdef _DEBUG
		DisplayCurrentNodeInfo(stream);
#endif

		if (stream.name() == "property")
		{
			if (stream.attributes().value("name") == "pixel_width")
			{
				params.pixelSize_mm[0] = stream.attributes().value("value").toDouble();
				//hasPixelSize = true;
			}
			else if (stream.attributes().value("name") == "pixel_height")
			{
				params.pixelSize_mm[1] = stream.attributes().value("value").toDouble();
				hasPixelSize = true;
			}
			stream.skipCurrentElement();
		}
		else if (stream.name() == "calibration" && stream.attributes().value("type") == "frame")
		{
			ccCameraSensor::ExtendedRadialDistortionParameters distParams;
			bool hasDistortion = false;
			bool hasResolution = false;
			bool hasVertFocal = false;
			bool hasCentralPoint = false;
			while (stream.readNextStartElement())
			{
#ifdef _DEBUG
				//DisplayCurrentNodeInfo(stream);
#endif

				if (stream.name() == "resolution")
				{
					int width = stream.attributes().value("width").toInt();
					int height = stream.attributes().value("height").toInt();
					if (width > 0 && height > 0)
					{
						params.arrayWidth = width;
						params.arrayHeight = height;
						hasResolution = true;
					}
					stream.skipCurrentElement();
				}
				else if (stream.name() == "fx")
				{
					double horizFocal_pix = stream.readElementText().toDouble();
					//++paramsCount;
				}
				else if (stream.name() == "fy")
				{
					params.vertFocal_pix = stream.readElementText().toDouble();
					hasVertFocal = true;
				}
				else if (stream.name() == "cx")
				{
					params.principal_point[0] = stream.readElementText().toDouble();
					hasCentralPoint = true;
				}
				else if (stream.name() == "cy")
				{
					params.principal_point[1] = stream.readElementText().toDouble();
					hasCentralPoint = true;
				}
				else if (stream.name() == "k1")
				{
					distParams.k1 = stream.readElementText().toDouble();
					hasDistortion = true;
				}
				else if (stream.name() == "k2")
				{
					distParams.k2 = stream.readElementText().toDouble();
					hasDistortion = true;
				}
				else if (stream.name() == "k3")
				{
					distParams.k3 = stream.readElementText().toDouble();
					hasDistortion = true;
				}
				else
				{
					stream.skipCurrentElement();
				}
			}

			if (hasResolution && hasVertFocal)
			{
				if (!hasCentralPoint)
				{
					//we define an arbitrary principal point
					params.principal_point[0] = params.arrayWidth / 2.0f;
					params.principal_point[1] = params.arrayHeight / 2.0f;
				}
				if (!hasPixelSize)
				{
					//we use an arbitrary 'pixel size'
					params.pixelSize_mm[0] = params.pixelSize_mm[1] = 1.0f / std::max(params.arrayWidth, params.arrayHeight);
				}
				params.vFOV_rad = ccCameraSensor::ComputeFovRadFromFocalPix(params.vertFocal_pix, params.arrayHeight);

				sensor = new ccCameraSensor(params);
				if (sensorAttributes.hasAttribute("label"))
				{
					sensor->setName(sensorAttributes.value("label").toString());
				}
				if (hasDistortion)
				{
					sensor->setDistortionParameters(ccCameraSensor::LensDistortionParameters::Shared(new ccCameraSensor::ExtendedRadialDistortionParameters(distParams)));
				}
			}

		} //"calibration"
		else
		{
			stream.skipCurrentElement();
		}
	}

	return sensor;
}

static bool DecodeCamera(QXmlStreamReader& stream, CameraDesc& camera)
{
	assert(stream.name() == "camera");

	QXmlStreamAttributes cameraAttributes = stream.attributes();
	if (	!cameraAttributes.hasAttribute("id")
		||	!cameraAttributes.hasAttribute("sensor_id")
		||	!cameraAttributes.hasAttribute("label"))
	{
		//invalid camera?!
		assert(false);
		return false;
	}
	
	camera.id = cameraAttributes.value("id").toInt();
	camera.sensorId = cameraAttributes.value("sensor_id").toInt();
	camera.imageFilename = cameraAttributes.value("label").toString();

	while (stream.readNextStartElement())
	{
#ifdef _DEBUG
		//DisplayCurrentNodeInfo(stream);
#endif

		if (stream.name() == "transform")
		{
			QString transformationValues = stream.readElementText();
			DecodeTransformation<float>(transformationValues, camera.trans);
		}
		else if (stream.name() == "reference")
		{
			QXmlStreamAttributes attributes = stream.attributes();
			if (attributes.value("enabled").toString() == "true")
			{
				CCVector3d T = {	attributes.value("x").toDouble(),
									attributes.value("y").toDouble(),
									attributes.value("z").toDouble() };
				//What is exactly the "reference" point?!
				//camera.trans.setTranslation(T.toPC());
			}
			stream.skipCurrentElement();
		}
		else //orientation? Not sure what it corresponds to!
		{
			stream.skipCurrentElement();
		}
	}

	return true;
}

static QString CreateTempFile(QuaZip& zip, QString zipFilename)
{
	if (!zip.setCurrentFile(zipFilename))
	{
		ccLog::Warning(QString("[Photoscan] Failed to locate '%1' in the Photoscan archive").arg(zipFilename));
		return QString();
	}

	//decompress the file
	QuaZipFile zipFile(&zip);
	if (!zipFile.open(QFile::ReadOnly))
	{
		ccLog::Warning(QString("[Photoscan] Failed to extract '%1' from Photoscan archive").arg(zipFilename));
		return QString();
	}

	QDir tempDir = QDir::temp();
	QString tempFilename = tempDir.absoluteFilePath(zipFilename);
	QFile tempFile(tempFilename);
	if (!tempFile.open(QFile::WriteOnly))
	{
		ccLog::Warning(QString("[Photoscan] Failed to create temp file '%1'").arg(tempFilename));
		return QString();
	}
	tempFile.write(zipFile.readAll());
	tempFile.close();

	return tempFilename;
}


PhotoScanFilter::PhotoScanFilter()
    : FileIOFilter( {
                    "_PhotoScan Filter",
					18.0f,	// priority
                    QStringList{ "psz" },
                    "psz",
                    QStringList{ "Photoscan project (*.psz)" },
                    QStringList(),
                    Import
                    } )
{
}

CC_FILE_ERROR PhotoScanFilter::loadFile(const QString& filename,
										ccHObject& container,
										LoadParameters& parameters)
{
	QuaZip zip(filename);

	if (!zip.open(QuaZip::mdUnzip))
	{
		//failed to open or read the zip file
		return CC_FERR_READING;
	}

	QStringList fileList = zip.getFileNameList();
	if (fileList.isEmpty())
	{
		//empty archive?
		return CC_FERR_NO_LOAD;
	}

	static const QString s_defaultXMLFilename("doc.xml");

	if (!fileList.contains(s_defaultXMLFilename))
	{
		//empty archive?
		ccLog::Warning(QString("[Photoscan] Couldn't find '%1' in Photoscan archive").arg(s_defaultXMLFilename));
		return CC_FERR_NO_LOAD;
	}

	//look for the XML file
	if (!zip.setCurrentFile(s_defaultXMLFilename))
	{
		ccLog::Warning(QString("[Photoscan] Failed to locate '%1' in the Photoscan archive").arg(s_defaultXMLFilename));
		return CC_FERR_MALFORMED_FILE;
	}

	//decompress the XML file
	QuaZipFile zipXML(&zip);
	if (!zipXML.open(QFile::ReadOnly))
	{
		ccLog::Warning(QString("[Photoscan] Failed to extract '%1' from Photoscan archive").arg(s_defaultXMLFilename));
		return CC_FERR_NO_LOAD;
	}

	QXmlStreamReader stream(&zipXML);

	//expected: "document"
	if (!stream.readNextStartElement() || stream.name() != "document")
	{
		return CC_FERR_MALFORMED_FILE;
	}

	std::vector<Sections> sections;
	sections.push_back(DOCUMENT);

	QMap<int, ccCameraSensor*> sensors;
	QMap<int, CameraDesc> cameras;
	QList<CloudDesc> clouds;
	QList<MeshDesc> meshes;
	ccGLMatrixd globalTransform;
	bool hasGlobalTransform = false;

	while (true)
	{
		if (!stream.readNextStartElement())
		{
			//end of section?
			if (!sections.empty())
			{
				ccLog::PrintDebug(" < " + stream.name().toString() + QString(" [%1]").arg(ToName(sections.back())));
				sections.pop_back();
				//stream.skipCurrentElement();
				continue;
			}
			else
			{
				//end of file
				break;
			}
		}
		ccLog::PrintDebug(" > " + stream.name().toString());

		switch (sections.back())
		{
		case DOCUMENT:
			if (stream.name() == "chunks")
			{
				sections.push_back(CHUNKS);
			}
			else
			{
				//not handled
				stream.skipCurrentElement();
			}
			break;

		case CHUNKS:
			if (stream.name() == "chunk")
			{
				sections.push_back(CHUNK);
			}
			else
			{
				//not handled
				stream.skipCurrentElement();
			}
			break;

		case CHUNK:
			if (stream.name() == "sensors")
			{
				sections.push_back(SENSORS);
			}
			else if (stream.name() == "cameras")
			{
				sections.push_back(CAMERAS);
			}
			else if (stream.name() == "frames")
			{
				sections.push_back(FRAMES);
			}
			else if (stream.name() == "transform")
			{
				//inner loop
				while (stream.readNextStartElement())
				{
					if (stream.name() == "rotation")
					{
						QString rotationValues = stream.readElementText();
						if (DecodeRotation<double>(rotationValues, globalTransform))
						{
							hasGlobalTransform = true;
						}
						else
						{
							assert(false);
						}
					}
					stream.skipCurrentElement();
				}
			}
			else //frames, reference, region, settings, meta, etc.
			{
				//not handled for now
				stream.skipCurrentElement();
			}
			break;

		case SENSORS:
			if (stream.name() == "sensor")
			{
				int sensorId = -1;
				ccCameraSensor* sensor = DecodeSensor(stream, sensorId);
				if (sensor)
				{
					assert(!sensors.contains(sensorId));
					sensors.insert(sensorId, sensor);
					//currentContainer->addChild(sensor);
				}
			}
			else
			{
				//not handled
				stream.skipCurrentElement();
			}
			break;

		case CAMERAS:
			if (stream.name() == "camera")
			{
				CameraDesc camera;
				ccGLMatrix trans;
				if (DecodeCamera(stream, camera))
				{
					assert(!cameras.contains(camera.id));
					cameras.insert(camera.id, camera);
					//currentContainer->addChild(camera.image);
				}
			}
			else
			{
				//not handled
				stream.skipCurrentElement();
			}
			break;

		case FRAMES:
			if (stream.name() == "frame")
			{
				sections.push_back(FRAME);
			}
			else
			{
				//not handled
				stream.skipCurrentElement();
			}
			break;

		case FRAME:
			if (stream.name() == "point_cloud" || stream.name() == "dense_cloud")
			{
				//inner loop
				bool denseCloud = (stream.name() == "dense_cloud");
				while (stream.readNextStartElement())
				{
					if (stream.name() == "points")
					{
						if (stream.attributes().hasAttribute("path"))
						{
							CloudDesc desc;
							desc.filename = stream.attributes().value("path").toString();
							desc.type = (denseCloud ? "dense cloud" : "keypoints");
							clouds.push_back(desc);
						}
						else
						{
							assert(false);
						}
					}
					stream.skipCurrentElement();
				}
			}
			else if (stream.name() == "model")
			{
				MeshDesc desc;

				//inner loop
				while (stream.readNextStartElement())
				{
					if (stream.name() == "mesh")
					{
						if (stream.attributes().hasAttribute("path"))
						{
							desc.filename = stream.attributes().value("path").toString();
						}
						else
						{
							assert(false);
						}
					}
					else if (stream.name() == "texture")
					{
						if (stream.attributes().hasAttribute("path"))
						{
							desc.texture = stream.attributes().value("path").toString();
						}
						else
						{
							assert(false);
						}
					}
					stream.skipCurrentElement();
				}
				if (!desc.filename.isEmpty())
				{
					meshes.push_back(desc);
				}
			}
			else
			{
				//not handled
				stream.skipCurrentElement();
			}
			break;

		case TRANSFORM:
			//not handled
			stream.skipCurrentElement();
			break;

		default:
			break;
		}
	}

	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (parameters.parentWidget)
	{
		progressDialog.reset(new ccProgressDialog(parameters.parentWidget));
		progressDialog->setRange(0, cameras.size() + clouds.size() + meshes.size());
		progressDialog->setWindowTitle("Loading data");
		progressDialog->start();
	}
	bool wasCanceled = false;
	int currentProgress = 0;

	//end of file: now we can sort the various extracted components
	QDir dir = QFileInfo(filename).dir();
	ccHObject* imageGroup = new ccHObject("Images");
	if (progressDialog && !cameras.empty())
	{
		progressDialog->setInfo(QString("Loading %1 image(s)").arg(cameras.size()));
	}
	for (CameraDesc& camera : cameras)
	{
		//progress
		if (progressDialog)
		{
			progressDialog->setValue(++currentProgress);
			if (progressDialog->wasCanceled())
			{
				wasCanceled = true;
				break;
			}
		}
		if (camera.imageFilename.isEmpty())
		{
			assert(false);
			continue;
		}

		//DGM: the images are not in the archive!
		//if (!zip.setCurrentFile(camera.imageFilename))
		//{
		//	ccLog::Warning(QString("[Photoscan] Failed to locate image '%1' in the Photoscan archive").arg(camera.imageFilename));
		//	continue;
		//}

		////decompress the image file
		//QuaZipFile zipImage(&zip);
		//if (!zipImage.open(QFile::ReadOnly))
		//{
		//	ccLog::Warning(QString("[Photoscan] Failed to extract '%1' from Photoscan archive").arg(camera.imageFilename));
		//	continue;
		//}

		QImage qImage;
		QString absoluteImageFilename = dir.absoluteFilePath(camera.imageFilename);
		//if (!qImage.load(&zipImage, qPrintable(QFileInfo(camera.imageFilename).suffix())))
		if (!qImage.load(absoluteImageFilename))
		{
			ccLog::Warning(QString("[Photoscan] Failed to load image '%1'").arg(camera.imageFilename));
			continue;
		}

		ccCameraSensor* const origSensor = sensors[camera.sensorId];
		if (origSensor)
		{
			origSensor->undistort(qImage);
		}

		ccImage* image = new ccImage(qImage);
		image->setName(camera.imageFilename);
		image->setAlpha(0.5f);
		image->setVisible(false);

		//associated sensor (if any)
		if (origSensor)
		{
			//make a copy of the original sensor
			ccCameraSensor* sensor = new ccCameraSensor(*origSensor);

			camera.trans.setColumn(1, -camera.trans.getColumnAsVec3D(1));
			camera.trans.setColumn(2, -camera.trans.getColumnAsVec3D(2));

			//FIXME: we would have to transform the clouds and meshes as well!
			//if (hasGlobalTransform)
			//{
			//	//apply global transformation (if any)
			//	camera.trans = ccGLMatrix(globalTransform.data()) * camera.trans;
			//}
			sensor->setRigidTransformation(camera.trans);
			sensor->setVisible(true);
			sensor->setGraphicScale(0.1f);
			image->setAssociatedSensor(sensor);
			image->addChild(sensor);
			imageGroup->addChild(image);
		}
	}
	if (imageGroup->getChildrenNumber())
	{
		container.addChild(imageGroup);
	}
	else
	{
		//no image?!
		delete imageGroup;
		imageGroup = nullptr;
	}

	//we can get rid of the original sensors
	for (ccCameraSensor*& sensor : sensors)
	{
		delete sensor;
		sensor = nullptr;
	}
	sensors.clear();

	//clouds
	if (!wasCanceled)
	{
		if (progressDialog && !clouds.empty())
		{
			progressDialog->setInfo(QString("Loading %1 cloud(s)").arg(cameras.size()));
		}
		for (CloudDesc& desc : clouds)
		{
			//progress
			if (progressDialog)
			{
				progressDialog->setValue(++currentProgress);
				if (progressDialog->wasCanceled())
				{
					wasCanceled = true;
					break;
				}
			}

			if (desc.filename.isEmpty())
			{
				assert(false);
				continue;
			}

			if (desc.filename.endsWith(".oc3", Qt::CaseInsensitive))
			{
				ccLog::Warning(QString("[Photoscan] OC3 format not supported. Can't import %1 from the Photoscan archive").arg(desc.type));
				continue;
			}

			QString tempFilename = CreateTempFile(zip, desc.filename);
			if (tempFilename.isNull())
			{
				continue;
			}

			ccHObject tempContainer;
			FileIOFilter::LoadParameters params;
			params.alwaysDisplayLoadDialog = false;
			params.autoComputeNormals = false;
			params.parentWidget = nullptr;
			CC_FILE_ERROR result = CC_FERR_NO_ERROR;
			ccHObject* newGroup = FileIOFilter::LoadFromFile(tempFilename, params, result);
			if (newGroup)
			{
				newGroup->setName(desc.type);
				if (desc.type == "keypoints")
				{
					newGroup->setEnabled(false);
				}
				container.addChild(newGroup);
			}
			else
			{
				ccLog::Warning(QString("[Photoscan] Failed to extract '%1' from Photoscan archive").arg(desc.filename));
			}
			QFile::remove(tempFilename);
		}
	}

	//meshes
	if (!wasCanceled)
	{
		if (progressDialog && !meshes.empty())
		{
			progressDialog->setInfo(QString("Loading %1 mesh(es)").arg(cameras.size()));
		}
		for (MeshDesc& desc : meshes)
		{
			//progress
			if (progressDialog)
			{
				progressDialog->setValue(++currentProgress);
				if (progressDialog->wasCanceled())
				{
					wasCanceled = true;
					break;
				}
			}

			if (desc.filename.isEmpty())
			{
				assert(false);
				continue;
			}

			QString tempFilename = CreateTempFile(zip, desc.filename);
			if (tempFilename.isNull())
			{
				continue;
			}

			FileIOFilter::LoadParameters params;
			params.alwaysDisplayLoadDialog = false;
			params.autoComputeNormals = false;
			params.parentWidget = nullptr;

			bool success = false;
			if (!desc.texture.isEmpty() && desc.filename.endsWith("ply", Qt::CaseInsensitive))
			{
				QString tempTextureFilename = CreateTempFile(zip, desc.texture);

				ccHObject tempContainer;
				if (PlyFilter().loadFile(tempFilename, desc.texture, tempContainer, params) == CC_FERR_NO_ERROR)
				{
					success = true;
					//transfer the loaded entities to the current container
					for (unsigned i = 0; i < tempContainer.getChildrenNumber(); ++i)
					{
						container.addChild(tempContainer.getChild(i));
					}
					tempContainer.detachAllChildren();
				}

				if (!tempTextureFilename.isNull())
				{
					QFile::remove(tempTextureFilename);
				}
			}
			else
			{
				CC_FILE_ERROR result = CC_FERR_NO_ERROR;
				ccHObject* newGroup = FileIOFilter::LoadFromFile(tempFilename, params, result);
				if (newGroup)
				{
					success = true;
					//transfer the loaded entities to the current container
					for (unsigned i = 0; i < newGroup->getChildrenNumber(); ++i)
					{
						container.addChild(newGroup->getChild(i));
					}
					newGroup->detachAllChildren();
					delete newGroup;
					newGroup = nullptr;
				}
			}

			if (!success)
			{
				ccLog::Warning(QString("[Photoscan] Failed to extract '%1' from Photoscan archive").arg(desc.filename));
			}

			QFile::remove(tempFilename);
		}
	}

	if (progressDialog)
	{
		progressDialog->stop();
	}

	return wasCanceled ? CC_FERR_CANCELED_BY_USER : CC_FERR_NO_ERROR;
}
