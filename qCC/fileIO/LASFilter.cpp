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
//$Rev:: 1631                                                              $
//$LastChangedDate:: 2010-08-25 07:21:40 +0200 (mer., 25 août 2010)       $
//**************************************************************************
//
#ifdef CC_LAS_SUPPORT

#include "LASFilter.h"

//qCC
#include <ccCommon.h>
#include "../ccCoordinatesShiftManager.h"

//Liblas
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <fstream>  // std::ifstream
#include <iostream> // std::cout

//CCLib
#include <CCMiscTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

CC_FILE_ERROR LASFilter::saveToFile(ccHObject* entity, const char* filename)
{
	if (!entity || !filename)
		return CC_FERR_BAD_ARGUMENT;

	ccHObject::Container clouds;
	if (entity->isKindOf(CC_POINT_CLOUD))
		clouds.push_back(entity);
	else
		entity->filterChildren(clouds, true, CC_POINT_CLOUD);

	if (clouds.empty())
	{
		ccConsole::Error("No point cloud in input selection!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}
	else if (clouds.size()>1)
	{
		ccConsole::Error("Can't save more than one cloud per LAS file!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//the cloud to save
	ccGenericPointCloud* theCloud = static_cast<ccGenericPointCloud*>(clouds[0]);
	unsigned numberOfPoints = theCloud->size();

	if (numberOfPoints==0)
	{
		ccConsole::Error("Cloud is empty!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//colors
	bool hasColor = theCloud->hasColors();

	//classification (as a scalar field)
	CCLib::ScalarField* classifSF = 0;
	if (theCloud->isA(CC_POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(theCloud);
		int sfIdx = pc->getScalarFieldIndexByName(CC_LAS_CLASSIFICATION_FIELD_NAME);
		if (sfIdx>=0)
		{
			classifSF = pc->getScalarField(sfIdx);
			if (/*classifSF->getMax()>(DistanceType)liblas::Classification::class_table_size ||*/ classifSF->getMin()<0)
			{
				ccConsole::Warning("[LASFilter] Found a 'classification' scalar field, but its values outbounds LAS specifications (0-255)...");
				classifSF = 0;
			}
			else
			{
				//we check that it's only integer values!
				unsigned i,count=classifSF->currentSize();
				classifSF->placeIteratorAtBegining();
				DistanceType integerPart;
				for (i=0;i<count;++i)
				{
					if (modf(classifSF->getCurrentValue(),&integerPart) != 0)
					{
						ccConsole::Warning("[LASFilter] Found a 'classification' scalar field, but its values are not pure integers...");
						classifSF = 0;
						break;
					}
				}
			}
		}
	}

	//open binary file for writing
	std::ofstream ofs;
	ofs.open(filename, std::ios::out | std::ios::binary);

	if (ofs.fail())
		return CC_FERR_WRITING;

	const double* shift = theCloud->getOriginalShift();

	liblas::Writer* writer = 0;
	try
	{
		liblas::Header header;
		//header.SetDataFormatId(liblas::ePointFormat3);
		ccBBox bBox = theCloud->getBB();
		if (bBox.isValid())
		{
			header.SetMin(-shift[0]+(double)bBox.minCorner().x,-shift[1]+(double)bBox.minCorner().y,-shift[2]+(double)bBox.minCorner().z);
			header.SetMax(-shift[0]+(double)bBox.maxCorner().x,-shift[1]+(double)bBox.maxCorner().y,-shift[2]+(double)bBox.maxCorner().z);
			CCVector3 diag = bBox.getDiagVec();

			//Set offset & scale, as points will be stored as boost::int32_t values (between 0 and 4294967296)
			//int_value = (double_value-offset)/scale
			header.SetOffset(-shift[0]+(double)bBox.minCorner().x,-shift[1]+(double)bBox.minCorner().y,-shift[2]+(double)bBox.minCorner().z);
			header.SetScale(1.0e-9*std::max<double>(diag.x,ZERO_TOLERANCE), //result must fit in 32bits?!
				1.0e-9*std::max<double>(diag.y,ZERO_TOLERANCE),
				1.0e-9*std::max<double>(diag.z,ZERO_TOLERANCE));
		}
		header.SetPointRecordsCount(numberOfPoints);
		//header.SetDataFormatId(Header::ePointFormat1);

		writer = new liblas::Writer(ofs, header);
	}
	catch (...)
	{
		return CC_FERR_WRITING;
	}

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,numberOfPoints);
	pdlg.setMethodTitle("Save LAS file");
	char buffer[256];
	sprintf(buffer,"Points: %i",numberOfPoints);
	pdlg.setInfo(buffer);
	pdlg.start();

	liblas::Point point(&writer->GetHeader());
	
	for (unsigned i=0; i<numberOfPoints; i++)
	{
		const CCVector3* P = theCloud->getPoint(i);
		{
			double x=-shift[0]+(double)P->x;
			double y=-shift[1]+(double)P->y;
			double z=-shift[2]+(double)P->z;
			point.SetCoordinates(x, y, z);
		}
		if (hasColor)
		{
			const colorType* rgb = theCloud->getPointColor(i);
			point.SetColor(liblas::Color(rgb[0],rgb[1],rgb[2]));
		}

		if (classifSF)
		{
			liblas::Classification classif;
			classif.SetClass((boost::uint32_t)classifSF->getValue(i));
			point.SetClassification(classif);
		}
		writer->WritePoint(point);

		if (!nprogress.oneStep())
			break;
	}

	delete writer;
	//ofs.close();

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LASFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	//opening file
	std::ifstream ifs;
	ifs.open(filename, std::ios::in | std::ios::binary);

	if (ifs.fail())
		return CC_FERR_READING;

	liblas::Reader* reader = 0;
	unsigned nbOfPoints = 0;
	std::vector<std::string> dimensions;

	try
	{
		reader = new liblas::Reader(ifs);

		liblas::Header const& header = reader->GetHeader();

#ifdef _DEBUG
		ccConsole::Print("[LAS FILE] %s - signature: %s",filename,header.GetFileSignature().c_str());
#endif

		//get fields present in file
		dimensions = header.GetSchema().GetDimensionNames();

		//and of course the number of points
		nbOfPoints = header.GetPointRecordsCount();
	}
	catch (...)
	{
		delete reader;
		ifs.close();
		return CC_FERR_READING;
	}

	if (nbOfPoints==0)
	{
		delete reader;
		ifs.close();
		return CC_FERR_NO_LOAD;
	}

	bool hasRGBColor[3] = {false,false,false};
	bool hasClassif = false;
	bool hasIntensity = false;
	for (unsigned k=0;k<dimensions.size();++k)
	{
		QString dim = QString(dimensions[k].c_str()).toUpper();
		bool handled=true;
		if (dim == "RED")
			hasRGBColor[0]=true;
		else if (dim == "BLUE")
			hasRGBColor[1]=true;
		else if (dim == "GREEN")
			hasRGBColor[2]=true;
		else if (dim == "CLASSIFICATION")
			hasClassif=true;
		else if (dim == "INTENSITY")
			hasIntensity=true;
		else if (dim != "X" && dim != "Y" && dim != "Z")
			handled=false;

		ccConsole::Print(QString("[LAS FILE] Found dimension '%1' (%2)").arg(dimensions[k].c_str()).arg(handled ? "handled" : "not handled"));
	}
	bool hasColor = (hasRGBColor[0] || hasRGBColor[1] || hasRGBColor[2]);
	colorType rgb[3]={0,0,0};

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,nbOfPoints);
	pdlg.setMethodTitle("Open LAS file");
	pdlg.setInfo(qPrintable(QString("Points: %1").arg(nbOfPoints)));
	pdlg.start();

	//number of points read from the begining of the current cloud part
	unsigned pointsRead=0;
	double Pshift[3]={0.0,0.0,0.0};

	char k=0; //current part index
	ccPointCloud* loadedCloud=0;
	ccScalarField* classifSF=0;
	ccScalarField* intensitySF=0;

	//if the file is too big, we will chunck it in multiple parts
	unsigned int fileChunkPos = 0;
	unsigned int fileChunkSize = 0;

	while (reader->ReadNextPoint())
	{
		//if we reach the max. cloud size limit, we cerate a new chunk
		if (pointsRead == fileChunkPos+fileChunkSize)
		{
			if (loadedCloud)
			{
				if (hasClassif && classifSF)
				{
					classifSF->computeMinAndMax();
					int cMin = (int)classifSF->getMin();
					int cMax = (int)classifSF->getMax();
					classifSF->setColorRampSteps(cMax-cMin);
					//classifSF->setMinSaturation(cMin);
					int sfIndex = loadedCloud->addScalarField(classifSF);
					if (cMax>2 || !hasColor)
					{
						loadedCloud->setCurrentDisplayedScalarField(sfIndex);
						loadedCloud->showSF(true);
					}
				}
				if (hasIntensity && intensitySF)
				{
					intensitySF->computeMinAndMax();
					intensitySF->setColorRamp(GREY);
					int sfIndex = loadedCloud->addScalarField(intensitySF);
					if (!hasColor && !hasClassif)
					{
						loadedCloud->setCurrentDisplayedScalarField(sfIndex);
						loadedCloud->showSF(true);
					}
				}
				container.addChild(loadedCloud);
			}

			fileChunkPos = pointsRead;
			fileChunkSize = ccMin(nbOfPoints-pointsRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
			loadedCloud = new ccPointCloud(QString("unnamed - Cloud #%1").arg(k++));
			if (!loadedCloud->reserveThePointsTable(fileChunkSize))
			{
				delete loadedCloud;
				delete reader;
				ifs.close();
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}
			loadedCloud->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);

			if (hasColor)
			{
				hasColor = loadedCloud->reserveTheRGBTable();
				if (hasColor)
					loadedCloud->showColors(true);
				else
					ccConsole::Warning("[LAS FILE] Failed to allocate color array for part #%1 (color will be skipped!)",k);
			}
			if (hasClassif)
			{
				classifSF = new ccScalarField(CC_LAS_CLASSIFICATION_FIELD_NAME,true);
				if (!classifSF->reserve(fileChunkSize))
				{
					classifSF->release();
					classifSF=0;
					hasClassif=false;
					ccConsole::Warning("[LAS FILE] Failed to allocate SF array for part #%1 (classification will be skipped!)",k);
				}
			}
			if (hasIntensity)
			{
				intensitySF = new ccScalarField("Intensity",true);
				if (!intensitySF->reserve(fileChunkSize))
				{
					intensitySF->release();
					intensitySF=0;
					hasIntensity=false;
					ccConsole::Warning("[LAS FILE] Failed to allocate SF array for part #%1 (intensity will be skipped!)",k);
				}
			}
		}

		const liblas::Point& p = reader->GetPoint();

		//first point: check for 'big' coordinates
		if (pointsRead==0)
		{
			double P[3]={p.GetX(),p.GetY(),p.GetZ()};
			bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
			if (shiftAlreadyEnabled)
				memcpy(Pshift,coordinatesShift,sizeof(double)*3);
			bool applyAll=false;
			if (ccCoordinatesShiftManager::Handle(P,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,applyAll))
			{
				loadedCloud->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
				ccConsole::Warning("[LASFilter::loadFile] Cloud has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift[0],Pshift[1],Pshift[2]);

				//we save coordinates shift information
				if (applyAll && coordinatesShiftEnabled && coordinatesShift)
				{
					*coordinatesShiftEnabled = true;
					coordinatesShift[0] = Pshift[0];
					coordinatesShift[1] = Pshift[1];
					coordinatesShift[2] = Pshift[2];
				}
			}
		}

		CCVector3 P(p.GetX()+Pshift[0],p.GetY()+Pshift[1],p.GetZ()+Pshift[2]);
		loadedCloud->addPoint(P);

		if (hasColor)
		{
			liblas::Color col = p.GetColor();
			for (unsigned c=0;c<3;++c)
				if (hasRGBColor[c])
					rgb[c]=col[c];
			loadedCloud->addRGBColor(rgb);
		}

		if (hasClassif)
		{
			assert(classifSF);
			DistanceType classValue = (DistanceType)p.GetClassification().GetClass();
			classifSF->addElement(classValue);
		}

		if (hasIntensity)
		{
			assert(intensitySF);
			uint16_t intValue = p.GetIntensity();
			intensitySF->addElement(intValue);
		}

		++pointsRead;

		if (!nprogress.oneStep()) //cancel requested
		{
			loadedCloud->resize(pointsRead+1-fileChunkPos);
			break;
		}
	}

	delete reader;
	reader=0;
	ifs.close();

	if (hasClassif && classifSF)
	{
		classifSF->computeMinAndMax();
		int cMin = (int)classifSF->getMin();
		int cMax = (int)classifSF->getMax();
		classifSF->setColorRampSteps(cMax-cMin);
		//classifSF->setMinSaturation(cMin);
		int sfIndex = loadedCloud->addScalarField(classifSF);
		if (cMax>2 || !hasColor)
		{
			loadedCloud->setCurrentDisplayedScalarField(sfIndex);
			loadedCloud->showSF(true);
		}
	}
	if (hasIntensity && intensitySF)
	{
		intensitySF->computeMinAndMax();
		intensitySF->setColorRamp(GREY);
		int sfIndex = loadedCloud->addScalarField(intensitySF);
		if (!hasColor && !hasClassif)
		{
			loadedCloud->setCurrentDisplayedScalarField(sfIndex);
			loadedCloud->showSF(true);
		}
	}
	container.addChild(loadedCloud);

	return CC_FERR_NO_ERROR;
}

#endif
