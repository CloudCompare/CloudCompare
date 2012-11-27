//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1947                                                              $
//$LastChangedDate:: 2011-11-27 09:57:04 +0100 (dim., 27 nov. 2011)        $
//**************************************************************************
//
#include "BundlerFilter.h"

//qCC
#include "BundlerImportDlg.h"
#include "BinFilter.h"

//qCC_db
#include <ccCalibratedImage.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccGLMatrix.h>
#include <ccHObjectCaster.h>
//#define TEST_TEXTURED_BUNDLER_IMPORT
#ifdef TEST_TEXTURED_BUNDLER_IMPORT
#include <ccMaterialSet.h>
#endif

//Qt
#include <QInputDialog>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QFileInfo>
#include <QDir>
#include <QFileDialog>
#include <QApplication>

//CCLib (for DTM generation)
#include <PointProjectionTools.h>
#include <MeshSamplingTools.h>
#include <SimpleCloud.h>
#include <ccMesh.h>

//! Bundler camera
struct BundlerCamera
{
	//! Default constructor
	BundlerCamera()
		: f(0.0f)
		, k1(0.0f)
		, k2(0.0f)
		, trans()
		, isValid(true)
	{}

	//! focal
	float f;
	//! First radial distortion coef.
	float k1;
	//! Second radial distortion coef.
	float k2;
	//! Rotation + translation
	ccGLMatrix trans;
	//! Validity
	bool isValid;
};

CC_FILE_ERROR BundlerFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	return loadFileExtended(filename,container,true);
}

//ortho-rectified image related information
struct ORImageInfo
{
	QString name; //image name
	unsigned w,h; //image dimensions
	double minC[2],maxC[2]; //local bounding box
};

CC_FILE_ERROR BundlerFilter::loadFileExtended(const char* filename,
									  ccHObject& container,
									  bool displayLoadDialog/*=true*/,
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
		ccConsole::Error("File should start by '# Bundle file vX.Y'!");
		return CC_FERR_MALFORMED_FILE;
	}
	unsigned majorVer=0,minorVer=0;
	sscanf(qPrintable(currentLine),"# Bundle file v%i.%i",&majorVer,&minorVer);
	if (majorVer!=0 || (minorVer!=3 && minorVer!=4))
	{
		ccConsole::Error("Only version 0.3 and 0.4 of Bundler files are supported!");
		return CC_FERR_WRONG_FILE_TYPE;
	}

	//second header line (should be <num_cameras> <num_points>)
	currentLine = stream.readLine();
	QStringList list = currentLine.split(QRegExp("\\s+"),QString::SkipEmptyParts);
	if (list.size() != 2)
	{
		ccConsole::Error("[BundlerFilter::loadFile] Second line should be <num_cameras> <num_points>!");
		return CC_FERR_MALFORMED_FILE;
	}
	unsigned camCount = list[0].toInt();
	if (camCount==0)
		ccConsole::Warning("[BundlerFilter::loadFile] No camera defined in Bundler file!");

	unsigned ptsCount = list[1].toInt();
	if (ptsCount==0)
		ccConsole::Warning("[BundlerFilter::loadFile] No keypoints defined in Bundler file!");

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

	//default paths
	QString imageListFilename = QFileInfo(f).dir().absoluteFilePath("list.txt");
	QString altKeypointsFilename = QFileInfo(f).dir().absoluteFilePath("pmvs.ply");

	if (displayLoadDialog)
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
		scaleFactor = (float)biDlg.getScaleFactor();
		keepImagesInMemory = biDlg.keepImagesInMemory();
		imageListFilename = biDlg.getImageListFilename();
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
	ccPointCloud* keypointsCloud = 0;
	ccHObject* altEntity = 0;
	typedef std::pair<unsigned,ccCalibratedImage::KeyPoint> KeypointAndCamIndex;
	std::vector<KeypointAndCamIndex> keypointsDescriptors;

	//Bundler file v0.4 contains filenames --> DGM: no, it was a 'fake' version of Bundler!
	QStringList imageFilenames;

	//Read Bundler's 'out' file
	{
		//progress dialog
		ccProgressDialog pdlg(true); //cancel available
		CCLib::NormalizedProgress nprogress(&pdlg,camCount + (importKeypoints || orthoRectifyImages || generateColoredDTM ? ptsCount : 0));
		pdlg.setMethodTitle("Open Bundler file");
		pdlg.setInfo(qPrintable(QString("Cameras: %1\nPoints: %2").arg(camCount).arg(ptsCount)));
		pdlg.start();

		//read cameras info
		if (importImages)
			cameras.resize(camCount);
		unsigned camIndex=0;
		for (std::vector<BundlerCamera>::iterator it=cameras.begin();it!=cameras.end();++it,++camIndex)
		{
			//--> 'fake' version of Bundler!
			//if (minorVer==4)
			//{
			//	//read filename first!
			//	currentLine = stream.readLine();
			//	if (currentLine.isEmpty())
			//		return CC_FERR_READING;
			//	if (importImages)
			//	{
			//		char filename[256];
			//		sscanf(qPrintable(currentLine),"Camera %s",filename);
			//		imageFilenames << QString(filename);
			//		//DGM: what to do with this?
			//	}
			//}

			//f, k1 and k2
			currentLine = stream.readLine();
			if (currentLine.isEmpty())
				return CC_FERR_READING;
			if (importImages)
				sscanf(qPrintable(currentLine),"%f %f %f",&it->f,&it->k1,&it->k2);
			//Rotation matrix
			float* mat = (importImages ? it->trans.data() : 0);
			float sum = 0.0f;
			for (unsigned l=0;l<3;++l)
			{
				currentLine = stream.readLine();
				if (currentLine.isEmpty())
					return CC_FERR_READING;
				if (importImages)
				{
					sscanf(qPrintable(currentLine),"%f %f %f",mat+l,mat+4+l,mat+8+l);
					sum += fabs(mat[l])+fabs(mat[4+l])+fabs(mat[8+l]);
				}
			}
			if (importImages && sum<ZERO_TOLERANCE)
			{
				ccConsole::Warning("[BundlerFilter::loadFile] Camera #%i is invalid!",camIndex+1);
				it->isValid=false;
			}
			//Translation
			currentLine = stream.readLine();
			if (currentLine.isEmpty())
				return CC_FERR_READING;
			if (importImages)
				sscanf(qPrintable(currentLine),"%f %f %f",mat+12,mat+13,mat+14);

			if (!nprogress.oneStep()) //cancel requested?
				return CC_FERR_CANCELED_BY_USER;
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

			bool hasColors=false;
			if (importKeypoints)
			{
				hasColors = keypointsCloud->reserveTheRGBTable();
				if (!hasColors)
					ccConsole::Warning("[BundlerFilter::loadFile] Not enough memory to load colors!");
				else
					keypointsCloud->showColors(true);
			}
			if (orthoRectifyImages)
				keypointsDescriptors.reserve(ptsCount*camCount/2); //at least!

			//--> 'fake' version of Bundler!
			//if (minorVer==4)
			//{
			//	//skip "----------Points" line!
			//	currentLine = stream.readLine();
			//	if (currentLine.isEmpty())
			//		return CC_FERR_READING;
			//}

			CCVector3 P;
			int R,G,B;
			for (unsigned i=0;i<ptsCount;++i)
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
				sscanf(qPrintable(currentLine),"%f %f %f",&P.x,&P.y,&P.z);
				keypointsCloud->addPoint(P);

				//RGB
				currentLine = stream.readLine();
				if (currentLine.isEmpty())
				{
					delete keypointsCloud;
					return CC_FERR_READING;
				}
				QStringList colorParts = currentLine.split(" ",QString::SkipEmptyParts);
				if (colorParts.size()==3)
				{
					if (hasColors)
					{
						sscanf(qPrintable(currentLine),"%i %i %i",&R,&G,&B);
						keypointsCloud->addRGBColor(R,G,B);
					}

					currentLine = stream.readLine();
				}
				else if (colorParts.size()>3)
				{
					//sometimes, it appears that keypoints has no associated color!
					//so we skip the line and assume it's in fact the keypoint description...
					ccConsole::Warning("[BundlerFilter::loadFile] Keypoint #%i has no associated color!",i);
					if (hasColors)
						keypointsCloud->addRGBColor(0,0,0); //black by default
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

				if (orthoRectifyImages && !useAltKeypoints)
				{
					QStringList parts = currentLine.split(" ",QString::SkipEmptyParts);
					if (!parts.isEmpty())
					{
						bool ok=false;
						unsigned nviews = parts[0].toInt(&ok);
						if (!ok || 1+nviews*4>(unsigned)parts.size())
						{
							ccConsole::Warning("[BundlerFilter::loadFile] View list for point #%i is invalid!",i);
						}
						else
						{
							unsigned pos=1;
							for (unsigned n=0;n<nviews;++n)
							{
								int cam = parts[pos++].toInt();			//camera index
								++pos;//int key = parts[pos++].toInt();	//index of the SIFT keypoint where the point was detected in that camera
								float x = parts[pos++].toFloat();		//detected positions of that keypoint (x)
								float y = parts[pos++].toFloat();		//detected positions of that keypoint (y)
								if (cam>=0 && (unsigned)cam<camCount)
								{
									//add key point
									KeypointAndCamIndex lastKeyPoint;
									lastKeyPoint.first = (unsigned)cam;
									lastKeyPoint.second.index = i;
									lastKeyPoint.second.x = x*scaleFactor;	//the origin is the center of the image, the x-axis increases to the right
									lastKeyPoint.second.y = -y*scaleFactor;	//and the y-axis increases towards the top of the image
									keypointsDescriptors.push_back(lastKeyPoint);
								}
							}
						}
					}
				}

				if (!nprogress.oneStep()) //cancel requested?
				{
					delete keypointsCloud;
					return CC_FERR_CANCELED_BY_USER;
				}
			}

			if (importKeypoints)
				container.addChild(keypointsCloud);
		}

		pdlg.stop();
		QApplication::processEvents();
	}

	//use alternative cloud/mesh as keypoints
	if (useAltKeypoints)
	{
		ccHObject* altKeypointsContainer = FileIOFilter::LoadFromFile(altKeypointsFilename);
		if (!altKeypointsContainer
			|| altKeypointsContainer->getChildrenNumber()!=1
			|| (!altKeypointsContainer->getChild(0)->isKindOf(CC_POINT_CLOUD) && !altKeypointsContainer->getChild(0)->isKindOf(CC_MESH)))
		{
			if (!altKeypointsContainer)
				ccConsole::Error(QString("[BundlerFilter::loadFile] Failed to load alternative keypoints file:\n'%1'").arg(altKeypointsFilename));
			else
				ccConsole::Error("[BundlerFilter::loadFile] Can't use this kind of entities as keypoints (need one and only one cloud or mesh)");

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

	assert(camCount>0);

	//load images
	QDir imageDir = QFileInfo(f).dir(); //by default we look in the Bundler file folder

	//let's try to open the images list file (if necessary)
	{
		imageFilenames.clear();
		QFile imageListFile(imageListFilename);
		if (!imageListFile.exists() || !imageListFile.open(QIODevice::ReadOnly))
		{
			ccConsole::Error(QString("[BundlerFilter::loadFile] Failed to open image list file! (%1)").arg(imageListFilename));
			if (!importKeypoints && keypointsCloud)
				delete keypointsCloud;
			if (!importKeypoints && altEntity)
				delete altEntity;
			return CC_FERR_UNKNOWN_FILE;
		}

		//we look for images in same directory
		imageDir = QFileInfo(imageListFile).dir();
		QTextStream imageListStream(&imageListFile);

		for (unsigned lineIndex=0;lineIndex<camCount;++lineIndex)
		{
			QString nextLine = imageListStream.readLine();
			if (nextLine.isEmpty())
				break;

			QStringList parts = nextLine.split(QRegExp("\\s+"),QString::SkipEmptyParts);
			if (parts.size()>0)
				imageFilenames << parts[0];
			else
			{
				ccConsole::Error(QString("[BundlerFilter::loadFile] Couldn't extract image name from line %1 in file '%2'!").arg(lineIndex).arg(imageListFilename));
				break;
			}
		}
	}

	if (imageFilenames.size() < (int)camCount) //not enough images!
	{
		if (imageFilenames.isEmpty())
			ccConsole::Error(QString("[BundlerFilter::loadFile] No filename could be extracted from file '%1'!").arg(imageListFilename));
		else
			ccConsole::Error(QString("[BundlerFilter::loadFile] Only %1 filenames (out of %2) could be extracted\nfrom file '%3'!").arg(imageFilenames.size()).arg(camCount).arg(imageListFilename));
		if (!importKeypoints && keypointsCloud)
			delete keypointsCloud;
		if (!importKeypoints && altEntity)
			delete altEntity;
		return CC_FERR_MALFORMED_FILE;
	}

	//let's try to open the image corresponding to each camera
	ccProgressDialog ipdlg(true); //cancel available
	CCLib::NormalizedProgress inprogress(&ipdlg,camCount);
	ipdlg.setMethodTitle("Open & process images");
	ipdlg.setInfo(qPrintable(QString("Images: %1").arg(camCount)));
	ipdlg.start();
	QApplication::processEvents();

	assert(imageFilenames.size()>=(int)camCount);

	/*** pre-processing steps (colored MNT computation, etc.) ***/

	//for colored DTM generation
	int* mntColors = 0;
	CCLib::SimpleCloud* mntSamples = 0;
	if (generateColoredDTM)
	{
		ccProgressDialog toDlg(true); //cancel available
		toDlg.setMethodTitle("Preparing colored DTM");
		toDlg.start();
		QApplication::processEvents();

		//1st step: triangulate keypoints (or use existing one)
		ccGenericMesh* baseDTMMesh = (altEntity ? ccHObjectCaster::ToGenericMesh(altEntity) : 0);
		CCLib::GenericIndexedMesh* dummyMesh = baseDTMMesh;
		if (!baseDTMMesh)
		{
			//alternative keypoints?
			ccGenericPointCloud* altKeypoints = (altEntity ? ccHObjectCaster::ToGenericPointCloud(altEntity) : 0);
			dummyMesh = CCLib::PointProjectionTools::computeTriangulation(altKeypoints ? altKeypoints : keypointsCloud,GENERIC_BEST_LS_PLANE);
			if (!dummyMesh)
			{
				ccConsole::Warning("[BundlerFilter::loadFile] Failed to generate DTM! (not enough memory?)");
			}
		}
		else
		{
			dummyMesh = baseDTMMesh;
		}

		if (dummyMesh)
		{
			//2nd step: samples points on resulting mesh
			mntSamples = CCLib::MeshSamplingTools::samplePointsOnMesh((CCLib::GenericMesh*)dummyMesh,coloredDTMVerticesCount);
			if (!baseDTMMesh)
				delete dummyMesh;
			dummyMesh=0;

			if (mntSamples)
			{
				//3rd step: project each point in all images and get average color
				unsigned count = mntSamples->size();
				mntColors = new int[4*count]; //R + G + B + accum count
				if (!mntColors)
				{
					//not enough memory
					ccConsole::Error("Not enough memory to store DTM colors! DTM generation cancelled");
					delete mntSamples;
					mntSamples=0;
					generateColoredDTM=false;
				}
				memset(mntColors,0,sizeof(int)*4*count);
			}
		}
	}

	std::vector<ORImageInfo> OR_infos;
	double OR_pixelSize=-1.0; //auto for first image
	double OR_globalCorners[4] = { 0, 0, 0, 0}; //corners for the global set

	//alternative keypoints? (for ortho-rectification only)
	ccGenericPointCloud* altKeypoints=0,*_keypointsCloud=0;
	if (orthoRectifyImages)
	{
		altKeypoints = (altEntity ? ccHObjectCaster::ToGenericPointCloud(altEntity) : 0);
		_keypointsCloud = (altKeypoints ? altKeypoints : keypointsCloud);
	}

	/*** process each image ***/

	bool cancelledByUser=false;
	for (unsigned i=0;i<camCount;++i)
	{
		const BundlerCamera& cam = cameras[i];
		if (!cam.isValid)
			continue;

		ccCalibratedImage* image = new ccCalibratedImage();
		QString errorStr;
		if (!image->load(imageDir.absoluteFilePath(imageFilenames[i]),errorStr))
		{
			ccConsole::Error(QString("[BundlerFilter::loadFile] %1 (image '%2')").arg(errorStr).arg(imageFilenames[i]));
			delete image;
			image=0;
			break;
		}

		image->setName(imageFilenames[i]);

		//associate image with calibration information
		image->setFocal(cameras[i].f * scaleFactor);
		image->setDistortionCoefficients(cameras[i].k1,cameras[i].k2);
		image->setCameraMatrix(cameras[i].trans);
		image->setEnabled(false);

		//ortho-rectification
		if (orthoRectifyImages)
		{
			assert(_keypointsCloud);

			//select image keypoints
			std::vector<ccCalibratedImage::KeyPoint> keypointsImage;
			if (_keypointsCloud == keypointsCloud) //keypoints from Bundler file
			{
				for (std::vector<KeypointAndCamIndex>::const_iterator key = keypointsDescriptors.begin(); key != keypointsDescriptors.end(); ++key)
					if (key->first == i)
						keypointsImage.push_back(key->second);
			}
			else
			{
				//project alternative cloud in image!
				_keypointsCloud->placeIteratorAtBegining();
				int half_w = (image->getW()>>1);
				int half_h = (image->getH()>>1);
				ccCalibratedImage::KeyPoint kp;
				unsigned keyptsCount = _keypointsCloud->size();
				for (unsigned k=0;k<keyptsCount;++k)
				{
					CCVector3 P(*_keypointsCloud->getPointPersistentPtr(k));
					//apply bundler equation
					cam.trans.apply(P);
					//convert to keypoint
					kp.x = -cam.f * P.x/P.z;
					kp.y = cam.f * P.y/P.z;
					if ((int)kp.x > -half_w && (int)kp.x < half_w
						&& (int)kp.y > -half_h && (int)kp.y < half_h)
					{
						kp.index = k;
						keypointsImage.push_back(kp);
					}
				}
			}

			if (keypointsImage.size()<4)
			{
				ccConsole::Warning(QString("[BundlerFilter::loadFile] Not enough keypoints descriptors for image '%1'!").arg(image->getName()));
			}
			else
			{
				if (orthoRectifyImagesAsImages)
				{
					//for ortho-rectification log
					ORImageInfo info;
					bool firstImage = (OR_pixelSize<0);
					double corners[8];
					ccImage* orthoImage = image->orthoRectifyAsImage(_keypointsCloud,
																		keypointsImage,
																		OR_pixelSize,
																		info.minC,
																		info.maxC,
																		corners);
					if (orthoImage)
					{
						assert(!orthoImage->data().isNull());
						info.name = QString("ortho_%1.png").arg(QFileInfo(imageFilenames[i]).baseName());
						info.w = orthoImage->getW();
						info.h = orthoImage->getH();
						orthoImage->data().save(imageDir.absoluteFilePath(info.name));
						ccConsole::Print(QString("[BundlerFilter] Ortho-rectified version of image '%1' saved to '%2'").arg(imageFilenames[i]).arg(imageDir.absoluteFilePath(info.name)));

#ifdef TEST_TEXTURED_BUNDLER_IMPORT

						//we tile the original image to avoid any OpenGL limitation on texture size
						#define TBI_DEFAULT_TILE_POW 10 // 2^10 = 1024
						const unsigned tileDim = (1<<TBI_DEFAULT_TILE_POW);
						unsigned horiTile = (info.w >> TBI_DEFAULT_TILE_POW);
						if (info.w-horiTile*tileDim!=0)
							++horiTile;
						unsigned vertTile = (info.h >> TBI_DEFAULT_TILE_POW);
						if (info.h-vertTile*tileDim!=0)
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
							float TA[2]={minu,minu};
							float TB[2]={maxu,minu};
							float TC[2]={maxu,maxu};
							float TD[2]={minu,maxu};
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
							double xRel = (double)x/(double)info.w;
							for (unsigned tj=0; tj<=vertTile; ++tj)
							{
								unsigned y = std::min(tj*tileDim,info.h);
								double yRel = (double)y/(double)info.h;

								//add vertices
								CCVector3 P(info.minC[0]+dcx*xRel,
									info.maxC[1]-dcy*yRel,
									0);
								rectVertices->addPoint(P);

								//add texture coordinates
								//float T0[2]={xRel,1.0-yRel};
								//texCoords->addElement(T0);

								if (ti<horiTile && tj<vertTile)
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
						orthoImage=0;

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
						ccConsole::Warning(QString("[BundlerFilter::loadFile] Failed to ortho-rectify image '%1'!").arg(image->getName()));
					}
				}

				if (orthoRectifyImagesAsClouds)
				{
					ccPointCloud* orthoCloud = image->orthoRectifyAsCloud(_keypointsCloud,keypointsImage);
					if (orthoCloud)
						container.addChild(orthoCloud);
					else
						ccConsole::Warning(QString("[BundlerFilter::loadFile] Failed to ortho-rectify image '%1' as a cloud!").arg(image->getName()));
				}
			}
		}

		//undistortion
		if (undistortImages)
			if (!image->undistort())
				ccConsole::Warning(QString("[BundlerFilter::loadFile] Failed to undistort image '%1'!").arg(image->getName()));

		//DTM color 'blending'
		if (generateColoredDTM)
		{
			assert(mntSamples && mntColors);
			unsigned k,sampleCount=mntSamples->size();
			QColor qColor;
			int blackValue = qColor.black();

			//back project each MNT samples in this image to get color
			for (k=0;k<sampleCount;++k)
			{
				CCVector3 P = *mntSamples->getPointPersistentPtr(k);

				//apply bundler equation
				image->getCameraMatrix().apply(P);
				if (fabs(P.z)>ZERO_TOLERANCE)
				{
					CCVector3 p(-P.x/P.z,-P.y/P.z,0.0);
					//float norm_p2 = p.norm2();
					//float rp = 1.0+norm_p2*(cam.k1+cam.k2*norm_p2); //images are already undistorted
					float rp = 1.0f;
					CCVector3 pprime = image->getFocal() * rp * p;

					int px = (int)(0.5f*(float)image->getW()+pprime.x);
					if (px >=0 && px < (int)image->getW())
					{
						int py = (int)(0.5f*(float)image->getH()-pprime.y);
						if (py >=0 && py < (int)image->getH())
						{
							QRgb rgb = image->data().pixel(px,py);
							if (qAlpha(rgb)!=0 && rgb != blackValue) //black pixels are ignored
							{
								int* col = mntColors+4*k;
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
			container.addChild(image);
		}
		else
		{
			delete image;
			image=0;
		}

		QApplication::processEvents();

		if (!inprogress.oneStep())
		{
			cancelledByUser=true;
			break;
		}
	}

	if (!importKeypoints && keypointsCloud)
		delete keypointsCloud;
	keypointsCloud=0;

	if (!importKeypoints && altEntity)
		delete altEntity;
	altEntity=0;

	/*** post-processing steps ***/

	if (orthoRectifyImages && OR_pixelSize>0)
	{
		//'close' log
		QFile f(imageDir.absoluteFilePath("ortho_rectification_log.txt"));
		if (f.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream stream(&f);
			stream.setRealNumberPrecision(12);
			stream << "PixelSize" << ' ' << OR_pixelSize << endl;
			stream << "Global3DBBox" << ' ' << OR_globalCorners[0] << ' ' << OR_globalCorners[1] << ' ' << OR_globalCorners[2] << ' ' << OR_globalCorners[3] << endl;
			int globalWidth = (int)((OR_globalCorners[2]-OR_globalCorners[0])/OR_pixelSize);
			int globalHeight = (int)((OR_globalCorners[3]-OR_globalCorners[1])/OR_pixelSize);
			stream << "Global2DBBox" << ' ' << 0 << ' ' << 0 << ' ' << globalWidth-1 << ' ' << globalHeight-1 << endl;

			for (unsigned i=0;i<OR_infos.size();++i)
			{
				stream << "Image" << ' ' << OR_infos[i].name << ' ';
				stream << "Local3DBBox" << ' ' << OR_infos[i].minC[0] << ' ' << OR_infos[i].minC[1] << ' ' << OR_infos[i].maxC[0] << ' ' << OR_infos[i].maxC[1] << ' ';
				int xShiftGlobal = (int)((OR_infos[i].minC[0]-OR_globalCorners[0])/OR_pixelSize);
				int yShiftGlobal = (int)((OR_globalCorners[3]-OR_infos[i].maxC[1])/OR_pixelSize);
				stream << "Local2DBBox" << ' ' << xShiftGlobal << ' ' << yShiftGlobal <<  ' ' << xShiftGlobal+((int)OR_infos[i].w-1) << ' ' << yShiftGlobal+((int)OR_infos[i].h-1) << endl;
			}
		}
		else
		{
			ccConsole::Warning("Failed to save orthorectification log file! (ortho_rectification_log.txt)");
		}
	}

	if (generateColoredDTM)
	{
		assert(mntSamples && mntColors);

		if (!cancelledByUser)
		{
			//3rd step: project each point in all images and get average color
			unsigned sampleCount = mntSamples->size();

			ccPointCloud* mntCloud = new ccPointCloud("colored DTM");
			if (mntCloud->reserve(sampleCount) && mntCloud->reserveTheRGBTable())
			{
				//for each point
				unsigned realCount=0;
				const int* col = mntColors;
				for (unsigned i=0;i<sampleCount;++i,col+=4)
				{
					if (col[3]>0) //accum
					{
						const CCVector3* X = mntSamples->getPointPersistentPtr(i);
						colorType avgCol[3]={(colorType)(col[0]/col[3]),
											 (colorType)(col[1]/col[3]),
											 (colorType)(col[2]/col[3])};
						mntCloud->addPoint(*X);
						mntCloud->addRGBColor(avgCol);
						++realCount;
					}
				}

				if (realCount!=0)
				{
					if (realCount<sampleCount)
						mntCloud->resize(realCount);

					mntCloud->showColors(true);
					container.addChild(mntCloud);
					ccConsole::Warning("[BundlerFilter::loadFile] DTM vertices sucessfully generated: clean it if necessary then use 'Edit > Mesh > Compute Delaunay 2D (Best LS plane)' then 'Smooth' to get a proper mesh");

					if (!displayLoadDialog)
					{
						//auto save DTM vertices
						BinFilter bf;
						QString outputFile = imageDir.absoluteFilePath("colored_dtm_vertices.bin");
						if (bf.saveToFile(mntCloud,qPrintable(outputFile)) == CC_FERR_NO_ERROR)
							ccConsole::Print(QString("[BundlerFilter] Color DTM vertices automatically saved to '%2'").arg(outputFile));
						else
							ccConsole::Warning(QString("[BundlerFilter] Failed to save DTM vertices to '%2'").arg(outputFile));
					}
				}
				else
				{
					ccConsole::Warning("[BundlerFilter::loadFile] Failed to generate DTM! (no point viewed in images?)");
				}
			}
			else
			{
				ccConsole::Warning("[BundlerFilter::loadFile] Failed to generate DTM vertices cloud! (not enough memory?)");
				delete mntCloud;
				mntCloud=0;
			}
		}

		delete mntSamples;
		mntSamples=0;
		delete[] mntColors;
		mntColors=0;
	}

	return cancelledByUser ? CC_FERR_CANCELED_BY_USER : CC_FERR_NO_ERROR;
}
