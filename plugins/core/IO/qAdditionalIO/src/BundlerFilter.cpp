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

#include "BundlerFilter.h"

//Local
#include "BinFilter.h"
#include "BundlerImportDlg.h"

//qCC_db
#include <ccCameraSensor.h>
#include <ccGLMatrix.h>
#include <ccHObjectCaster.h>
#include <ccImage.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Qt
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QString>
#include <QTextStream>

//CCLib (for DTM generation)
#include <ccMesh.h>
#include <MeshSamplingTools.h>
#include <PointProjectionTools.h>

//System
#include <string>

//! Bundler camera
struct BundlerCamera
{
	//! Default constructor
	BundlerCamera()
		: f_pix(0)
		, k1(0)
		, k2(0)
		, trans()
		, isValid(true)
	{}

	//! focal (in pixels)
	float f_pix;
	//! First radial distortion coef.
	float k1;
	//! Second radial distortion coef.
	float k2;
	//! Rotation + translation
	ccGLMatrixd trans;
	//! Validity
	bool isValid;
};

BundlerFilter::BundlerFilter()
	: FileIOFilter( {
					"_Snavely Bundler Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "out" },
					"out",
					QStringList{ "Snavely's Bundler output (*.out)" },
					QStringList(),
					Import
					} )
{
}

CC_FILE_ERROR BundlerFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	return loadFileExtended(filename, container, parameters);
}

//ortho-rectified image related information
struct ORImageInfo
{
	QString name; //image name
	unsigned w,h; //image dimensions
	double minC[2],maxC[2]; //local bounding box
};

CC_FILE_ERROR BundlerFilter::loadFileExtended(	const QString& filename,
												ccHObject& container,
												LoadParameters& parameters,
												const QString& _altKeypointsFilename/*=QString()*/,
												bool _undistortImages/*=false*/,
												bool _generateColoredDTM/*=false*/,
												unsigned _coloredDTMVerticesCount/*=1000000*/,
												float _scaleFactor/*=1.0f*/)
{
	//opening file (ASCII)
	QFile f(filename);
	if (!f.open(QIODevice::ReadOnly))
		return CC_FERR_READING;

	QTextStream stream(&f);

	//read header (should be "# Bundle file vX.Y")
	QString currentLine = stream.readLine();
	if (!currentLine.startsWith("# Bundle file",Qt::CaseInsensitive))
	{
		ccLog::Error("File should start by '# Bundle file vX.Y'!");
		return CC_FERR_MALFORMED_FILE;
	}
	unsigned majorVer = 0;
	unsigned minorVer = 0;
	sscanf(qPrintable(currentLine), "# Bundle file v%u.%u", &majorVer, &minorVer);
	if (majorVer != 0 || (minorVer != 3 && minorVer != 4))
	{
		ccLog::Error("Only version 0.3 and 0.4 of Bundler files are supported!");
		return CC_FERR_WRONG_FILE_TYPE;
	}

	//second header line (should be <num_cameras> <num_points>)
	currentLine = stream.readLine();
	QStringList list = currentLine.simplified().split(QChar(' '),QString::SkipEmptyParts);
	if (list.size() != 2)
	{
		ccLog::Error("[Bundler] Second line should be <num_cameras> <num_points>!");
		return CC_FERR_MALFORMED_FILE;
	}
	unsigned camCount = list[0].toInt();
	if (camCount == 0)
		ccLog::Warning("[Bundler] No camera defined in Bundler file!");

	unsigned ptsCount = list[1].toInt();
	if (ptsCount == 0)
		ccLog::Warning("[Bundler] No keypoints defined in Bundler file!");

	//parameters
	bool importKeypoints = false;
	bool useAltKeypoints = false;
	bool importImages = false;
	bool undistortImages = false;
	bool orthoRectifyImagesAsClouds = false;
	bool orthoRectifyImagesAsImages = false;
	bool orthoRectifyImages = false;
	bool generateColoredDTM = false;
	unsigned coloredDTMVerticesCount = 1000000;
	float scaleFactor = 1.0f;
	bool keepImagesInMemory = false;
	bool applyOptMatrix = false;
	ccGLMatrix orthoOptMatrix;
	orthoOptMatrix.toIdentity();
	BundlerImportDlg::OrthoRectMethod orthoRectMethod = BundlerImportDlg::OPTIMIZED;

	//default paths
	QString imageListFilename = QFileInfo(f).dir().absoluteFilePath("list.txt");
	QString altKeypointsFilename = QFileInfo(f).dir().absoluteFilePath("pmvs.ply");

	if (parameters.alwaysDisplayLoadDialog)
	{
		//open dialog
		BundlerImportDlg biDlg;
		biDlg.setKeypointsCount(ptsCount);
		biDlg.setCamerasCount(camCount);
		biDlg.setVer(majorVer,minorVer);
		biDlg.setImageListFilename(imageListFilename);
		biDlg.setAltKeypointsFilename(altKeypointsFilename);

		if (!biDlg.exec())
			return CC_FERR_CANCELED_BY_USER;

		importKeypoints = biDlg.importKeypoints();
		useAltKeypoints = biDlg.useAlternativeKeypoints();
		if (useAltKeypoints)
			altKeypointsFilename = biDlg.getAltKeypointsFilename();
		importImages = biDlg.importImages();
		undistortImages = biDlg.undistortImages();
		orthoRectifyImagesAsClouds = biDlg.orthoRectifyImagesAsClouds();
		orthoRectifyImagesAsImages = biDlg.orthoRectifyImagesAsImages();
		generateColoredDTM = biDlg.generateColoredDTM();
		coloredDTMVerticesCount = biDlg.getDTMVerticesCount();
		scaleFactor = static_cast<float>(biDlg.getScaleFactor());
		keepImagesInMemory = biDlg.keepImagesInMemory();
		imageListFilename = biDlg.getImageListFilename();
		applyOptMatrix = biDlg.getOptionalTransfoMatrix(orthoOptMatrix);
		orthoRectMethod = biDlg.getOrthorectificationMethod();
	}
	else
	{
		importImages = true;
		orthoRectifyImagesAsImages = true;
		useAltKeypoints = !_altKeypointsFilename.isEmpty();
		if (useAltKeypoints)
			altKeypointsFilename = _altKeypointsFilename;
		undistortImages = _undistortImages;
		generateColoredDTM = _generateColoredDTM;
		if (generateColoredDTM)
			coloredDTMVerticesCount = _coloredDTMVerticesCount;
		scaleFactor = _scaleFactor;
	}

	if (!importKeypoints && !importImages)
		return CC_FERR_NO_LOAD;

	orthoRectifyImages = orthoRectifyImagesAsClouds || orthoRectifyImagesAsImages;

	//data
	std::vector<BundlerCamera> cameras;
	ccPointCloud* keypointsCloud = nullptr;
	ccHObject* altEntity = nullptr;
	using KeypointAndCamIndex = std::pair<unsigned,ccCameraSensor::KeyPoint>;
	std::vector<KeypointAndCamIndex> keypointsDescriptors;

	//Read Bundler '.out' file
	{
		//progress dialog
		QScopedPointer<ccProgressDialog> pDlg(nullptr);
		if (parameters.parentWidget)
		{
			pDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
			pDlg->setMethodTitle(QObject::tr("Open Bundler file"));
			pDlg->setInfo(QObject::tr("Cameras: %1\nPoints: %2").arg(camCount).arg(ptsCount));
			pDlg->start();
		}
		CCLib::NormalizedProgress nprogress(pDlg.data(), camCount + (importKeypoints || orthoRectifyImages || generateColoredDTM ? ptsCount : 0));

		//read cameras info (whatever the case!)
		cameras.resize(camCount);
		unsigned camIndex = 0;
		for (std::vector<BundlerCamera>::iterator it = cameras.begin(); it != cameras.end(); ++it, ++camIndex)
		{
			//f, k1 and k2
			currentLine = stream.readLine();
			if (currentLine.isEmpty())
				return CC_FERR_READING;
			if (importImages)
			{
				QStringList tokens = currentLine.simplified().split(QChar(' '),QString::SkipEmptyParts);
				if (tokens.size() < 3)
					return CC_FERR_MALFORMED_FILE;
				bool ok[3] = {true,true,true};
				it->f_pix = tokens[0].toFloat(ok);
				it->k1 = tokens[1].toFloat(ok+1);
				it->k2 = tokens[2].toFloat(ok+2);
				if (!ok[0] ||!ok[1] || !ok[2])
					return CC_FERR_MALFORMED_FILE;
			}
			//Rotation matrix
			double* mat = (importImages ? it->trans.data() : nullptr);
			double sum = 0;
			for (unsigned l=0; l<3; ++l)
			{
				currentLine = stream.readLine();
				if (currentLine.isEmpty())
					return CC_FERR_READING;
				if (importImages)
				{
					QStringList tokens = currentLine.simplified().split(QChar(' '),QString::SkipEmptyParts);
					if (tokens.size() < 3)
						return CC_FERR_MALFORMED_FILE;
					bool ok[3] = {true,true,true};
					mat[l] = tokens[0].toDouble(ok);
					mat[4+l] = tokens[1].toDouble(ok+1);
					mat[8+l] = tokens[2].toDouble(ok+2);
					if (!ok[0] ||!ok[1] || !ok[2])
						return CC_FERR_MALFORMED_FILE;
					sum += fabs(mat[l]) + fabs(mat[4+l]) + fabs(mat[8+l]);
				}
			}
			if (importImages && sum < ZERO_TOLERANCE)
			{
				ccLog::Warning("[Bundler] Camera #%i is invalid!",camIndex+1);
				it->isValid = false;
			}

			//Translation
			currentLine = stream.readLine();
			if (currentLine.isEmpty())
				return CC_FERR_READING;
			if (importImages)
			{
				QStringList tokens = currentLine.simplified().split(QChar(' '),QString::SkipEmptyParts);
				if (tokens.size() < 3)
					return CC_FERR_MALFORMED_FILE;
				bool ok[3] = {true,true,true};
				mat[12] = tokens[0].toDouble(ok);
				mat[13] = tokens[1].toDouble(ok+1);
				mat[14] = tokens[2].toDouble(ok+2);
				if (!ok[0] ||!ok[1] || !ok[2])
					return CC_FERR_MALFORMED_FILE;
			}

			if (pDlg && !nprogress.oneStep()) //cancel requested?
			{
				return CC_FERR_CANCELED_BY_USER;
			}
		}

		//read points
		if (!useAltKeypoints && (importKeypoints || orthoRectifyImages || generateColoredDTM))
		{
			keypointsCloud = new ccPointCloud("Keypoints");
			if (!keypointsCloud->reserve(ptsCount))
			{
				delete keypointsCloud;
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}

			bool hasColors = false;
			if (importKeypoints)
			{
				hasColors = keypointsCloud->reserveTheRGBTable();
				if (!hasColors)
					ccLog::Warning("[Bundler] Not enough memory to load colors!");
				else
					keypointsCloud->showColors(true);
			}
			
			bool storeKeypoints = orthoRectifyImages/* && !useAltKeypoints*/;
			//we'll check if all cameras are used or not!
			std::vector<bool> camUsage;
			//if (!useAltKeypoints)
			{
				try
				{
					camUsage.resize(cameras.size(),false);
				}
				catch (const std::bad_alloc&)
				{
					//nothing serious here
				}
			}

			CCVector3d Pshift(0, 0, 0);
			for (unsigned i = 0; i < ptsCount; ++i)
			{
				//Point (X,Y,Z)
				currentLine = stream.readLine();
				if (currentLine.startsWith("--")) //skip lines starting with '--' (yes it happens in some weird version of Bundler?!)
					currentLine = stream.readLine();
				if (currentLine.isEmpty())
				{
					delete keypointsCloud;
					return CC_FERR_READING;
				}

				//read point coordinates (as strings)
				CCVector3d Pd(0,0,0);
				{
					QStringList tokens = currentLine.simplified().split(QChar(' '),QString::SkipEmptyParts);
					if (tokens.size() < 3)
					{
						delete keypointsCloud;
						return CC_FERR_MALFORMED_FILE;
					}
					//decode coordinates
					bool ok[3] = {true,true,true};
					Pd.x = tokens[0].toDouble(ok);
					Pd.y = tokens[1].toDouble(ok+1);
					Pd.z = tokens[2].toDouble(ok+2);
					if (!ok[0] ||!ok[1] || !ok[2])
					{
						delete keypointsCloud;
						return CC_FERR_MALFORMED_FILE;
					}
				}
				
				//first point: check for 'big' coordinates
				if (i == 0)
				{
					bool preserveCoordinateShift = true;
					if (HandleGlobalShift(Pd, Pshift, preserveCoordinateShift, parameters))
					{
						if (preserveCoordinateShift)
						{
							keypointsCloud->setGlobalShift(Pshift);
						}
						//we must apply the shift to the cameras as well!!!
						for (size_t j = 0; j < cameras.size(); ++j)
						{
							ccGLMatrixd& trans = cameras[j].trans;
							trans.invert();
							trans.setTranslation(trans.getTranslationAsVec3D() + Pshift);
							trans.invert();
						}
						ccLog::Warning("[Bundler] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
					}
				}
				keypointsCloud->addPoint(CCVector3::fromArray((Pd + Pshift).u));

				//RGB
				currentLine = stream.readLine();
				if (currentLine.isEmpty())
				{
					delete keypointsCloud;
					return CC_FERR_READING;
				}
				QStringList colorParts = currentLine.split(" ", QString::SkipEmptyParts);
				if (colorParts.size() == 3)
				{
					if (hasColors)
					{
						QStringList tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
						if (tokens.size() < 3)
						{
							delete keypointsCloud;
							return CC_FERR_MALFORMED_FILE;
						}
						int R = tokens[0].toInt();
						int G = tokens[1].toInt();
						int B = tokens[2].toInt();
						int A = (tokens.size() > 3 ? tokens[3].toInt() : ccColor::MAX);
						keypointsCloud->addColor(	static_cast<ColorCompType>(std::min<int>(R, ccColor::MAX)),
													static_cast<ColorCompType>(std::min<int>(G, ccColor::MAX)),
													static_cast<ColorCompType>(std::min<int>(B, ccColor::MAX)),
													static_cast<ColorCompType>(std::min<int>(A, ccColor::MAX)) );
					}

					currentLine = stream.readLine();
				}
				else if (colorParts.size() > 3)
				{
					//sometimes, it appears that keypoints have no associated color!
					//so we skip the line and assume it's in fact the keypoint description...
					ccLog::Warning("[Bundler] Keypoint #%i has no associated color!",i);
					if (hasColors)
						keypointsCloud->addColor(ccColor::black); //black by default
				}
				else
				{
					delete keypointsCloud;
					return CC_FERR_MALFORMED_FILE;
				}
				
				//view list (currentLine should already be read, see above)
				if (currentLine.isEmpty())
				{
					delete keypointsCloud;
					return CC_FERR_READING;
				}

				if (storeKeypoints || !camUsage.empty())
				{
					QStringList parts = currentLine.split(" ",QString::SkipEmptyParts);
					if (!parts.isEmpty())
					{
						bool ok = false;
						unsigned nviews = parts[0].toInt(&ok);
						if (!ok || nviews*4+1 > static_cast<unsigned>(parts.size()))
						{
							ccLog::Warning("[Bundler] View list for point #%i is invalid!",i);
						}
						else
						{
							unsigned pos = 1;
							for (unsigned n=0; n<nviews; ++n)
							{
								int cam = parts[pos++].toInt();			//camera index
								++pos; //int key = parts[pos++].toInt();		//index of the SIFT keypoint where the point was detected in that camera (not used)
								if (cam < 0 || static_cast<unsigned>(cam) >= camCount)
								{
									pos += 2;
									continue;
								}
								if (!camUsage.empty())
								{
									camUsage[cam] = true;
								}
								if (storeKeypoints)
								{
									float x = parts[pos++].toFloat();		//detected positions of that keypoint (x)
									float y = parts[pos++].toFloat();		//detected positions of that keypoint (y)
									//add key point
									KeypointAndCamIndex lastKeyPoint;
									lastKeyPoint.first = static_cast<unsigned>(cam);
									lastKeyPoint.second.index = i;
									lastKeyPoint.second.x =  x*scaleFactor;	//the origin is the center of the image, the x-axis increases to the right
									lastKeyPoint.second.y = -y*scaleFactor;	//and the y-axis increases towards the top of the image
									try
									{
										keypointsDescriptors.push_back(lastKeyPoint);
									}
									catch (const std::bad_alloc&)
									{
										ccLog::Warning("[Bundler] Not enough memory to store keypoints!");
										keypointsDescriptors.clear();
										orthoRectifyImages = false;
										storeKeypoints = false;
									}
								}
							}
						}
					}
				}

				if (pDlg && !nprogress.oneStep()) //cancel requested?
				{
					delete keypointsCloud;
					return CC_FERR_CANCELED_BY_USER;
				}
			}

			if (!camUsage.empty())
			{
				for (size_t i=0; i<camUsage.size(); ++i)
				{
					if (!camUsage[i])
						ccLog::Warning(QString("[Bundler] Camera #%1 has no associated keypoints!").arg(i+1));
				}
			}

			//apply optional matrix (if any)
			if (applyOptMatrix)
			{
				keypointsCloud->applyGLTransformation_recursive(&orthoOptMatrix);
				ccLog::Print("[Bundler] Keypoints cloud has been transformed with input matrix!");
				//this transformation is of no interest for the user
				keypointsCloud->resetGLTransformationHistory_recursive();
			}

			if (importKeypoints)
				container.addChild(keypointsCloud);
		}

		if (pDlg)
		{
			pDlg->stop();
			QApplication::processEvents();
		}
	}

	//use alternative cloud/mesh as keypoints
	if (useAltKeypoints)
	{
		FileIOFilter::LoadParameters altKeypointsParams;
		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		ccHObject* altKeypointsContainer = FileIOFilter::LoadFromFile(altKeypointsFilename, altKeypointsParams, result);
		if (	!altKeypointsContainer
			||	altKeypointsContainer->getChildrenNumber() != 1
			||	(!altKeypointsContainer->getChild(0)->isKindOf(CC_TYPES::POINT_CLOUD) && !altKeypointsContainer->getChild(0)->isKindOf(CC_TYPES::MESH)))
		{
			if (!altKeypointsContainer)
				ccLog::Error(QString("[Bundler] Failed to load alternative keypoints file:\n'%1'").arg(altKeypointsFilename));
			else
				ccLog::Error("[Bundler] Can't use this kind of entities as keypoints (need one and only one cloud or mesh)");

			return CC_FERR_WRONG_FILE_TYPE;
		}
		else
		{
			altEntity = altKeypointsContainer->getChild(0);
			if (importKeypoints)
				container.addChild(altEntity);
		}
	}

	if (!importImages)
		return CC_FERR_NO_ERROR;

	assert(camCount > 0);

	//load images
	QDir imageDir = QFileInfo(f).dir(); //by default we look in the Bundler file folder

	//let's try to open the images list file (if necessary)
	QStringList imageFilenames;
	{
		imageFilenames.clear();
		QFile imageListFile(imageListFilename);
		if (!imageListFile.exists() || !imageListFile.open(QIODevice::ReadOnly))
		{
			ccLog::Error(QString("[Bundler] Failed to open image list file! (%1)").arg(imageListFilename));
			if (!importKeypoints && keypointsCloud)
				delete keypointsCloud;
			if (!importKeypoints && altEntity)
				delete altEntity;
			return CC_FERR_UNKNOWN_FILE;
		}

		//we look for images in same directory
		imageDir = QFileInfo(imageListFile).dir();
		QTextStream imageListStream(&imageListFile);

		for (unsigned lineIndex=0; lineIndex<camCount; ++lineIndex)
		{
			QString nextLine = imageListStream.readLine();
			if (nextLine.isEmpty())
				break;

			QStringList parts = nextLine.simplified().split(QChar(' '),QString::SkipEmptyParts);
			if (!parts.empty())
			{
				imageFilenames << parts[0];
			}
			else
			{
				ccLog::Error(QString("[Bundler] Couldn't extract image name from line %1 in file '%2'!").arg(lineIndex).arg(imageListFilename));
				break;
			}
		}
	}

	if (imageFilenames.size() < static_cast<int>(camCount)) //not enough images!
	{
		if (imageFilenames.isEmpty())
			ccLog::Error(QString("[Bundler] No filename could be extracted from file '%1'!").arg(imageListFilename));
		else
			ccLog::Error(QString("[Bundler] Only %1 filenames (out of %2) could be extracted\nfrom file '%3'!").arg(imageFilenames.size()).arg(camCount).arg(imageListFilename));
		if (!importKeypoints && keypointsCloud)
			delete keypointsCloud;
		if (!importKeypoints && altEntity)
			delete altEntity;
		return CC_FERR_MALFORMED_FILE;
	}

	//let's try to open the image corresponding to each camera
	QScopedPointer<ccProgressDialog> ipDlg(nullptr);
	if (parameters.parentWidget)
	{
		ipDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
		ipDlg->setMethodTitle(QObject::tr("Open & process images"));
		ipDlg->setInfo(QObject::tr("Images: %1").arg(camCount));
		ipDlg->start();
		QApplication::processEvents();
	}
	CCLib::NormalizedProgress inprogress(ipDlg.data(), camCount);

	assert(imageFilenames.size() >= static_cast<int>(camCount));

	/*** pre-processing steps (colored MNT computation, etc.) ***/

	//for colored DTM generation
	std::vector<int> mntColors;
	QScopedPointer<CCLib::PointCloud> mntSamples;
	if (generateColoredDTM)
	{
		QScopedPointer<ccProgressDialog> toDlg(nullptr);
		if (parameters.parentWidget)
		{
			toDlg.reset(new ccProgressDialog(true, parameters.parentWidget)); //cancel available
			toDlg->setMethodTitle(QObject::tr("Preparing colored DTM"));
			toDlg->start();
			QApplication::processEvents();
		}

		//1st step: triangulate keypoints (or use existing one)
		ccGenericMesh* baseDTMMesh = (altEntity ? ccHObjectCaster::ToGenericMesh(altEntity) : nullptr);
		CCLib::GenericIndexedMesh* dummyMesh = baseDTMMesh;
		if (!baseDTMMesh)
		{
			//alternative keypoints?
			ccGenericPointCloud* altKeypoints = (altEntity ? ccHObjectCaster::ToGenericPointCloud(altEntity) : nullptr);
			char errorStr[1024];
			dummyMesh = CCLib::PointProjectionTools::computeTriangulation(	altKeypoints ? altKeypoints : keypointsCloud,
																			DELAUNAY_2D_BEST_LS_PLANE,
																			0,
																			0,
																			errorStr);
			if (!dummyMesh)
			{
				ccLog::Warning(QString("[Bundler] Failed to generate DTM! (%1)").arg(errorStr));
			}
		}

		if (dummyMesh)
		{
			//2nd step: samples points on resulting mesh
			mntSamples.reset(CCLib::MeshSamplingTools::samplePointsOnMesh((CCLib::GenericMesh*)dummyMesh, coloredDTMVerticesCount));
			if (!baseDTMMesh)
				delete dummyMesh;
			dummyMesh = nullptr;

			if (mntSamples)
			{
				//3rd step: project each point in all images and get average color
				unsigned count = mntSamples->size();
				try
				{
					mntColors.resize(4 * count, 0); //R + G + B + accum count
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					ccLog::Error("Not enough memory to store DTM colors! DTM generation cancelled");
					generateColoredDTM = false;
				}
			}
		}
		else
		{
			generateColoredDTM = false;
		}
	}

	std::vector<ORImageInfo> OR_infos;
	double OR_pixelSize = -1.0; //auto for first image
	double OR_globalCorners[4] = { 0, 0, 0, 0}; //corners for the global set

	//alternative keypoints? (for ortho-rectification only)
	ccGenericPointCloud* altKeypoints = nullptr;
	ccGenericPointCloud* _keypointsCloud = nullptr;
	if (orthoRectifyImages)
	{
		altKeypoints = (altEntity ? ccHObjectCaster::ToGenericPointCloud(altEntity) : nullptr);
		_keypointsCloud = (altKeypoints ? altKeypoints : keypointsCloud);
	}

	/*** process each image ***/

	bool cancelledByUser = false;
	for (unsigned i = 0; i < camCount; ++i)
	{
		const BundlerCamera& cam = cameras[i];
		if (!cam.isValid)
			continue;

		ccImage* image = new ccImage();
		QString errorStr;
		if (!image->load(imageDir.absoluteFilePath(imageFilenames[i]), errorStr))
		{
			ccLog::Error(QString("[Bundler] %1 (image '%2')").arg(errorStr,imageFilenames[i]));
			delete image;
			image = nullptr;
			break;
		}

		image->setName(imageFilenames[i]);
		image->setEnabled(false);
		image->setAlpha(0.75f); //semi transparent by default

		//associate image with calibration information
		ccCameraSensor* sensor = nullptr;
		{
			ccCameraSensor::IntrinsicParameters params;
			params.arrayWidth = static_cast<int>(image->getW());
			params.arrayHeight = static_cast<int>(image->getH());
			//we define an arbitrary principal point
			params.principal_point[0] = params.arrayWidth / 2.0f;
			params.principal_point[1] = params.arrayHeight / 2.0f;
			//we use an arbitrary 'pixel size'
			params.pixelSize_mm[0] = params.pixelSize_mm[1] = 1.0f / std::max(params.arrayWidth, params.arrayHeight);
			params.vertFocal_pix = cam.f_pix * scaleFactor;
			params.vFOV_rad = ccCameraSensor::ComputeFovRadFromFocalPix(cam.f_pix, params.arrayHeight);

			//camera position/orientation
			ccGLMatrix transf(cameras[i].trans.inverse().data());
			
			//dist to cloud
			PointCoordinateType dist = keypointsCloud ? (transf.getTranslationAsVec3D() - keypointsCloud->getOwnBB().getCenter()).norm() : PC_ONE;
			params.zFar_mm = dist;
			params.zNear_mm = 0.001f;

			sensor = new ccCameraSensor(params);
			sensor->setName(QString("Camera #%1").arg(i + 1));
			sensor->setEnabled(true);
			sensor->setVisible(true/*false*/);
			sensor->setGraphicScale(keypointsCloud ? keypointsCloud->getOwnBB().getDiagNorm() / 10 : PC_ONE);
			sensor->setRigidTransformation(transf);

			//distortion parameters
			if (cameras[i].k1 != 0 || cameras[i].k2 != 0)
			{
				ccCameraSensor::RadialDistortionParameters* distParams = new ccCameraSensor::RadialDistortionParameters;
				distParams->k1 = cameras[i].k1;
				distParams->k2 = cameras[i].k2;
				sensor->setDistortionParameters(ccCameraSensor::LensDistortionParameters::Shared(distParams));
			}

			//apply optional matrix (if any)
			if (applyOptMatrix)
			{
				sensor->applyGLTransformation_recursive(&orthoOptMatrix);
				//ccLog::Print("[Bundler] Camera cloud has been transformed with input matrix!");
				//this transformation is of no interest for the user
				sensor->resetGLTransformationHistory_recursive();
			}
		}
		//the image is a child of the sensor!
		image->setAssociatedSensor(sensor);
		sensor->addChild(image);

		//ortho-rectification
		if (orthoRectifyImages)
		{
			assert(sensor && _keypointsCloud);

			//select image keypoints
			std::vector<ccCameraSensor::KeyPoint> keypointsImage;
			ccBBox keypointsImageBB;
			if (_keypointsCloud == keypointsCloud) //keypoints from Bundler file
			{
				for (std::vector<KeypointAndCamIndex>::const_iterator key = keypointsDescriptors.begin(); key != keypointsDescriptors.end(); ++key)
				{
					if (key->first == i)
					{
						keypointsImage.push_back(key->second);
						keypointsImageBB.add(CCVector3(key->second.x,key->second.y,0));
					}
				}
			}
			else
			{
				//project alternative cloud in image!
				_keypointsCloud->placeIteratorAtBeginning();
				int half_w = (image->getW() >> 1);
				int half_h = (image->getH() >> 1);
				ccCameraSensor::KeyPoint kp;
				unsigned keyptsCount = _keypointsCloud->size();
				for (unsigned k = 0; k<keyptsCount; ++k)
				{
					CCVector3 P(*_keypointsCloud->getPointPersistentPtr(k));
					//apply bundler equation
					cam.trans.apply(P);
					//convert to keypoint
					kp.x = -cam.f_pix * static_cast<float>(P.x / P.z);
					kp.y = cam.f_pix * static_cast<float>(P.y / P.z);
					if (	static_cast<int>(kp.x) > -half_w && static_cast<int>(kp.x < half_w)
						&&	static_cast<int>(kp.y) > -half_h && static_cast<int>(kp.y < half_h))
					{
						kp.index = k;
						keypointsImage.push_back(kp);
						keypointsImageBB.add(CCVector3(kp.x,kp.y,0));
					}
				}
			}

			if (keypointsImage.size() < 4)
			{
				ccLog::Warning(QString("[Bundler] Not enough keypoints descriptors for image '%1'!").arg(image->getName()));
			}
			else if (!keypointsImageBB.isValid() || keypointsImageBB.getDiagNorm() < PC_ONE)
			{
				ccLog::Warning(QString("[Bundler] Keypoints descriptors for image '%1' are invalid (= all the same)").arg(image->getName()));
			}
			else
			{
				if (orthoRectifyImagesAsImages)
				{
					//for ortho-rectification log
					ORImageInfo info;
					double corners[8];
					ccImage* orthoImage = nullptr;
					
					//"standard" ortho-rectification method
					if (orthoRectMethod == BundlerImportDlg::OPTIMIZED)
					{
						orthoImage = sensor->orthoRectifyAsImage(	image,
																	_keypointsCloud,
																	keypointsImage,
																	OR_pixelSize,
																	info.minC,
																	info.maxC,
																	corners);
					}
					//"direct" ortho-rectification method
					else
					{
						assert(	orthoRectMethod == BundlerImportDlg::DIRECT
							||	orthoRectMethod == BundlerImportDlg::DIRECT_UNDISTORTED );

						//we take the keypoints 'middle altitude' by default
						CCVector3 bbMin;
						CCVector3 bbMax;
						_keypointsCloud->getBoundingBox(bbMin, bbMax);
						PointCoordinateType Z0 = (bbMin.z + bbMax.z) / 2;

						orthoImage = sensor->orthoRectifyAsImageDirect(	image,
																		Z0,
																		OR_pixelSize,
																		orthoRectMethod == BundlerImportDlg::DIRECT_UNDISTORTED,
																		info.minC,
																		info.maxC,
																		corners);
					}

					if (orthoImage)
					{
						assert(!orthoImage->data().isNull());
						info.name = QString("ortho_%1.png").arg(QFileInfo(imageFilenames[i]).baseName());
						info.w = orthoImage->getW();
						info.h = orthoImage->getH();
						orthoImage->data().save(imageDir.absoluteFilePath(info.name));
						ccLog::Print(QString("[Bundler] Ortho-rectified version of image '%1' (%2 x %3) saved to '%4'").arg(imageFilenames[i]).arg(info.w).arg(info.h).arg(imageDir.absoluteFilePath(info.name)));

#ifdef TEST_TEXTURED_BUNDLER_IMPORT

						//we tile the original image to avoid any OpenGL limitation on texture size
						#define TBI_DEFAULT_TILE_POW 10 // 2^10 = 1024
						const unsigned tileDim = (1<<TBI_DEFAULT_TILE_POW);
						unsigned horiTile = (info.w >> TBI_DEFAULT_TILE_POW);
						if (info.w-horiTile*tileDim != 0)
							++horiTile;
						unsigned vertTile = (info.h >> TBI_DEFAULT_TILE_POW);
						if (info.h-vertTile*tileDim != 0)
							++vertTile;
						unsigned tiles = horiTile*vertTile;
						unsigned verts = (horiTile+1)*(vertTile+1);

						//vertices
						ccPointCloud* rectVertices = new ccPointCloud("vertices");
						rectVertices->reserve(verts);

						//mesh
						ccMesh* rectMesh = new ccMesh(rectVertices);
						rectMesh->reserve(2*tiles);
						rectMesh->addChild(rectVertices);

						//materials (=textures)
						ccMaterialSet* matSet = new ccMaterialSet("Texture");

						//texture coordinates table
						TextureCoordsContainer* texCoords = new TextureCoordsContainer();
						//texCoords->reserve(verts);
						texCoords->reserve(4);
						{
							//float minu = 1.0f/(float)(2*tileDim);
							//float maxu = 1.0-1.0f/(float)(2*tileDim);
							float minu = 0.0f;
							float maxu = 1.0f;
							float TA[2] = {minu,minu};
							float TB[2] = {maxu,minu};
							float TC[2] = {maxu,maxu};
							float TD[2] = {minu,maxu};
							texCoords->addElement(TA);
							texCoords->addElement(TB);
							texCoords->addElement(TC);
							texCoords->addElement(TD);
						}

						//per triangle material indexes
						rectMesh->reservePerTriangleMtlIndexes();
						//per triangle texture coordinates indexes
						rectMesh->reservePerTriangleTexCoordIndexes();

						double dcx = info.maxC[0]-info.minC[0];
						double dcy = info.maxC[1]-info.minC[1];

						//process all tiles
						for (unsigned ti=0; ti<=horiTile; ++ti)
						{
							unsigned x = std::min(ti*tileDim,info.w);
							double xRel = static_cast<double>(x)/info.w;
							for (unsigned tj=0; tj<=vertTile; ++tj)
							{
								unsigned y = std::min(tj*tileDim,info.h);
								double yRel = static_cast<double>(y)/info.h;

								//add vertices
								CCVector3 P(info.minC[0]+dcx*xRel,
									info.maxC[1]-dcy*yRel,
									0);
								rectVertices->addPoint(P);

								//add texture coordinates
								//float T0[2]={xRel,1.0-yRel};
								//texCoords->addElement(T0);

								if (ti < horiTile && tj < vertTile)
								{
									unsigned w = std::min(info.w-x,tileDim);
									unsigned h = std::min(info.h-y,tileDim);

									//create corresponding texture
									unsigned tileIndex = matSet->size();
									matSet->push_back(ccMaterial(info.name));
									matSet->back().texture = orthoImage->data().copy(x,y,w,h);
									//matSet->back().texture.save(imageDir.absoluteFilePath(QString("tile_%1_").arg(tileIndex)+info.name));

									unsigned iA = ti*(vertTile+1)+tj;
									unsigned iB = (ti+1)*(vertTile+1)+tj;
									unsigned iC = iB+1;
									unsigned iD = iA+1;

									rectMesh->addTriangle(iA,iB,iD);
									//rectMesh->addTriangleTexCoordIndexes(iA,iB,iD);
									rectMesh->addTriangleTexCoordIndexes(0,1,3);
									rectMesh->addTriangleMtlIndex(tileIndex);
									rectMesh->addTriangle(iB,iC,iD);
									//rectMesh->addTriangleTexCoordIndexes(iB,iC,iD);
									rectMesh->addTriangleTexCoordIndexes(1,2,3);
									rectMesh->addTriangleMtlIndex(tileIndex);
								}
							}
						}

						rectMesh->showMaterials(true);
						rectMesh->setName(info.name);

						//associate texture coordinates table
						rectMesh->setTexCoordinatesTable(texCoords);
						//associate material set
						rectMesh->setMaterialSet(matSet);
						container.addChild(rectMesh);
#endif

						delete orthoImage;
						orthoImage = nullptr;

						OR_infos.push_back(info);

						//update global boundaries
						if (OR_globalCorners[0] > info.minC[0])
							OR_globalCorners[0] = info.minC[0];
						if (OR_globalCorners[1] > info.minC[1])
							OR_globalCorners[1] = info.minC[1];
						if (OR_globalCorners[2] < info.maxC[0])
							OR_globalCorners[2] = info.maxC[0];
						if (OR_globalCorners[3] < info.maxC[1])
							OR_globalCorners[3] = info.maxC[1];
					}
					else
					{
						ccLog::Warning(QString("[Bundler] Failed to ortho-rectify image '%1'!").arg(image->getName()));
					}
				}

				if (orthoRectifyImagesAsClouds)
				{
					ccPointCloud* orthoCloud = sensor->orthoRectifyAsCloud(image,_keypointsCloud,keypointsImage);
					if (orthoCloud)
					{
						orthoCloud->setGlobalScale(_keypointsCloud->getGlobalScale());
						orthoCloud->setGlobalShift(_keypointsCloud->getGlobalShift());
						container.addChild(orthoCloud);
					}
					else
					{
						ccLog::Warning(QString("[Bundler] Failed to ortho-rectify image '%1' as a cloud!").arg(image->getName()));
					}
				}
			}
		}

		//undistortion
		if (sensor && undistortImages)
			if (!sensor->undistort(image,true))
				ccLog::Warning(QString("[Bundler] Failed to undistort image '%1'!").arg(image->getName()));

		//DTM color 'blending'
		if (sensor && generateColoredDTM)
		{
			assert(mntSamples && !mntColors.empty());
			unsigned sampleCount = mntSamples->size();
			const QRgb blackValue = qRgb(0, 0, 0);

			ccGLMatrix sensorMatrix = sensor->getRigidTransformation().inverse();

			//back project each MNT samples in this image to get color
			for (unsigned k=0; k<sampleCount; ++k)
			{
				CCVector3 P = *mntSamples->getPointPersistentPtr(k);

				//apply bundler equation
				sensorMatrix.apply(P);
				if (fabs(P.z) > ZERO_TOLERANCE)
				{
					CCVector3 p(-P.x / P.z, -P.y / P.z, 0.0);
					//float norm_p2 = p.norm2();
					//float rp = 1.0+norm_p2*(cam.k1+cam.k2*norm_p2); //images are already undistorted
					float rp = 1.0f;
					CCVector3 pprime = cam.f_pix * rp * p;

					int px = static_cast<int>(image->getW() / 2.0f + pprime.x);
					if (px >= 0 && px < static_cast<int>(image->getW()))
					{
						int py = static_cast<int>(image->getH() / 2.0f - pprime.y);
						if (py >= 0 && py < static_cast<int>(image->getH()))
						{
							QRgb rgb = image->data().pixel(px, py);
							if (qAlpha(rgb) != 0 && rgb != blackValue) //black pixels are ignored
							{
								int* col = mntColors.data() + 4 * k;
								col[0] += qRed(rgb);
								col[1] += qGreen(rgb);
								col[2] += qBlue(rgb);
								col[3]++; //accum
							}
						}
					}
				}
			}
		}

		if (keepImagesInMemory)
		{
			container.addChild(sensor);
		}
		else
		{
			delete sensor;
			sensor = nullptr;
		}

		QApplication::processEvents();

		if (ipDlg && !inprogress.oneStep())
		{
			cancelledByUser = true;
			break;
		}
	}

	if (!importKeypoints && keypointsCloud)
		delete keypointsCloud;
	keypointsCloud = nullptr;

	if (!importKeypoints && altEntity)
		delete altEntity;
	altEntity = nullptr;

	/*** post-processing steps ***/

	if (orthoRectifyImages && OR_pixelSize > 0)
	{
		//'close' log
		QFile f(imageDir.absoluteFilePath("ortho_rectification_log.txt"));
		if (f.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream stream(&f);
			stream.setRealNumberNotation(QTextStream::FixedNotation);
			stream.setRealNumberPrecision(12);
			stream << "PixelSize" << ' ' << OR_pixelSize << endl;
			stream << "Global3DBBox" << ' ' << OR_globalCorners[0] << ' ' << OR_globalCorners[1] << ' ' << OR_globalCorners[2] << ' ' << OR_globalCorners[3] << endl;
			int globalWidth = static_cast<int>((OR_globalCorners[2]-OR_globalCorners[0])/OR_pixelSize);
			int globalHeight = static_cast<int>((OR_globalCorners[3]-OR_globalCorners[1])/OR_pixelSize);
			stream << "Global2DBBox" << ' ' << 0 << ' ' << 0 << ' ' << globalWidth-1 << ' ' << globalHeight-1 << endl;

			for (unsigned i=0; i<OR_infos.size(); ++i)
			{
				stream << "Image" << ' ' << OR_infos[i].name << ' ';
				stream << "Local3DBBox" << ' ' << OR_infos[i].minC[0] << ' ' << OR_infos[i].minC[1] << ' ' << OR_infos[i].maxC[0] << ' ' << OR_infos[i].maxC[1] << ' ';
				int xShiftGlobal = static_cast<int>((OR_infos[i].minC[0]-OR_globalCorners[0])/OR_pixelSize);
				int yShiftGlobal = static_cast<int>((OR_globalCorners[3]-OR_infos[i].maxC[1])/OR_pixelSize);
				stream << "Local2DBBox" << ' ' << xShiftGlobal << ' ' << yShiftGlobal <<  ' ' << xShiftGlobal+(static_cast<int>(OR_infos[i].w)-1) << ' ' << yShiftGlobal+(static_cast<int>(OR_infos[i].h)-1) << endl;
			}
		}
		else
		{
			ccLog::Warning("Failed to save orthorectification log file! (ortho_rectification_log.txt)");
		}
	}

	if (generateColoredDTM)
	{
		assert(mntSamples && !mntColors.empty());

		if (!cancelledByUser)
		{
			//3rd step: project each point in all images and get average color
			unsigned sampleCount = mntSamples->size();

			ccPointCloud* mntCloud = new ccPointCloud("colored DTM");
			if (mntCloud->reserve(sampleCount) && mntCloud->reserveTheRGBTable())
			{
				//for each point
				unsigned realCount = 0;
				const int* col = mntColors.data();
				for (unsigned i = 0; i < sampleCount; ++i, col += 4)
				{
					if (col[3] > 0) //accumulation (not alpha ;)
					{
						const CCVector3* X = mntSamples->getPointPersistentPtr(i);
						ccColor::Rgb avgCol(static_cast<ColorCompType>(col[0] / col[3]),
											static_cast<ColorCompType>(col[1] / col[3]),
											static_cast<ColorCompType>(col[2] / col[3]) );
						mntCloud->addPoint(*X);
						mntCloud->addColor(avgCol);
						++realCount;
					}
				}

				if (realCount != 0)
				{
					if (realCount < sampleCount)
						mntCloud->resize(realCount);

					mntCloud->showColors(true);
					container.addChild(mntCloud);
					ccLog::Warning("[Bundler] DTM vertices successfully generated: clean it if necessary then use 'Edit > Mesh > Compute Delaunay 2D (Best LS plane)' then 'Smooth' to get a proper mesh");

					if (!parameters.alwaysDisplayLoadDialog)
					{
						//auto save DTM vertices
						BinFilter bf;
						QString outputFile = imageDir.absoluteFilePath("colored_dtm_vertices.bin");
						BinFilter::SaveParameters parameters;
						{
							parameters.alwaysDisplaySaveDialog = false;
						}
						if (bf.saveToFile(mntCloud, outputFile, parameters) == CC_FERR_NO_ERROR)
							ccLog::Print(QString("[Bundler] Color DTM vertices automatically saved to '%2'").arg(outputFile));
						else
							ccLog::Warning(QString("[Bundler] Failed to save DTM vertices to '%2'").arg(outputFile));
					}
				}
				else
				{
					ccLog::Warning("[Bundler] Failed to generate DTM! (no point viewed in images?)");
				}
			}
			else
			{
				ccLog::Warning("[Bundler] Failed to generate DTM vertices cloud! (not enough memory?)");
				delete mntCloud;
				mntCloud = nullptr;
			}
		}
	}

	return cancelledByUser ? CC_FERR_CANCELED_BY_USER : CC_FERR_NO_ERROR;
}
