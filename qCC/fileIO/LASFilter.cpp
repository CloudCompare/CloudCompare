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

//CCLib
#include <CCMiscTools.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Liblas
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/factory.hpp>	// liblas::ReaderFactory
#include <fstream>				// std::ifstream
#include <iostream>				// std::cout

//Qt
#include<QFileInfo>

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

		//LAZ support based on extension!
		if (QFileInfo(filename).suffix().toUpper() == "LAZ") 
		{
			header.SetCompressed(true);
		}

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

	//liblas::Point point(boost::shared_ptr<liblas::Header>(new liblas::Header(writer->GetHeader())));
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
			point.SetColor(liblas::Color(rgb[0]<<8,rgb[1]<<8,rgb[2]<<8)); //DGM: LAS colors are stored on 16 bits!
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
		reader = new liblas::Reader(liblas::ReaderFactory().CreateWithStream(ifs));	//using factory for automatic and transparent
																					//handling of compressed/uncompressed files
		liblas::Header const& header = reader->GetHeader();

#ifdef _DEBUG
		//ccConsole::Print("[LAS FILE] %s - signature: %s",filename,header.GetFileSignature().c_str());
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
		//strange file ;)
		delete reader;
		ifs.close();
		return CC_FERR_NO_LOAD;
	}

	liblas::Color rgbColorMask; //(0,0,0) on construction
	bool hasClassif = false;
	bool hasIntensity = false;
	bool hasTime = false;
	bool hasReturnNumber = false;
	for (unsigned k=0;k<dimensions.size();++k)
	{
		QString dim = QString(dimensions[k].c_str()).toUpper();
		bool handled=true;
		if (dim == "RED")
			rgbColorMask.SetRed(~0);
		else if (dim == "BLUE")
			rgbColorMask.SetBlue(~0);
		else if (dim == "GREEN")
			rgbColorMask.SetGreen(~0);
		else if (dim == "CLASSIFICATION")
			hasClassif=true;
		else if (dim == "TIME")
			hasTime=true;
		else if (dim == "INTENSITY")
			hasIntensity=true;
		else if (dim == "RETURN NUMBER")
			hasReturnNumber=true;
		else if (dim != "X" && dim != "Y" && dim != "Z")
			handled=false;

		ccConsole::Print(QString("[LAS FILE] Found dimension '%1' (%2)").arg(dimensions[k].c_str()).arg(handled ? "handled" : "not handled"));
	}
	bool hasColor = (rgbColorMask[0] || rgbColorMask[1] || rgbColorMask[2]);

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,nbOfPoints);
	pdlg.setMethodTitle("Open LAS file");
	pdlg.setInfo(qPrintable(QString("Points: %1").arg(nbOfPoints)));
	pdlg.start();

	//number of points read from the begining of the current cloud part
	unsigned pointsRead=0;
	double Pshift[3]={0.0,0.0,0.0};

	//by default we read color as 8 bits integers and we will change this to 16 bits if it's not (16 bits is the standard!)
	unsigned char colorCompBitDec = 0;
	colorType rgb[3]={0,0,0};

	ccPointCloud* loadedCloud=0;
	
	ccScalarField* classifSF=0;
	uint8_t firstClassifValue=0;
	
	ccScalarField* timeSF=0;
	double firstTime=0.0;

	ccScalarField* intensitySF=0;
	uint16_t firstIntensity=0;
	
	ccScalarField* returnNumberSF=0;
	uint16_t firstReturnNumber=0;

	//if the file is too big, we will chunck it in multiple parts
	unsigned int fileChunkPos = 0;
	unsigned int fileChunkSize = 0;

	while (true)
	{
		//if we reach the end of the file, or the max. cloud size limit (in which case we cerate a new chunk)
		bool newPointAvailable = (nprogress.oneStep() && reader->ReadNextPoint());

		if (!newPointAvailable || pointsRead == fileChunkPos+fileChunkSize)
		{
			if (loadedCloud)
			{
				if (loadedCloud->size())
				{
					bool thisChunkHasColors = loadedCloud->hasColors();
					loadedCloud->showColors(thisChunkHasColors);
					if (hasColor && !thisChunkHasColors)
						ccLog::Warning("[LAS FILE] Color field was all black! We ignored it...");

					if (hasClassif)
					{
						if (classifSF)
						{
							classifSF->computeMinAndMax();
							int cMin = (int)classifSF->getMin();
							int cMax = (int)classifSF->getMax();
							classifSF->setColorRampSteps(cMax-cMin);
							//classifSF->setMinSaturation(cMin);
							int sfIndex = loadedCloud->addScalarField(classifSF);
							if (!loadedCloud->hasDisplayedScalarField())
							{
								loadedCloud->setCurrentDisplayedScalarField(sfIndex);
								loadedCloud->showSF(!thisChunkHasColors);
							}
						}
						else
							ccLog::Warning(QString("[LAS FILE] All classification values were the same (%1)! We ignored them...").arg(firstClassifValue));
					}

					if (hasIntensity)
					{
						if (intensitySF)
						{
							intensitySF->computeMinAndMax();
							intensitySF->setColorRamp(GREY);
							int sfIndex = loadedCloud->addScalarField(intensitySF);
							if (!loadedCloud->hasDisplayedScalarField())
							{
								loadedCloud->setCurrentDisplayedScalarField(sfIndex);
								loadedCloud->showSF(!thisChunkHasColors);
							}
						}
						else
							ccLog::Warning(QString("[LAS FILE] All intensities were the same (%1)! We ignored them...").arg(firstIntensity));
					}

					if (hasTime)
					{
						if (timeSF)
						{
							timeSF->computeMinAndMax();
							int sfIndex = loadedCloud->addScalarField(timeSF);
							if (!loadedCloud->hasDisplayedScalarField())
							{
								loadedCloud->setCurrentDisplayedScalarField(sfIndex);
								loadedCloud->showSF(!thisChunkHasColors);
							}
						}
						else
							ccLog::Warning(QString("[LAS FILE] All timestamps were the same (%1)! We ignored them...").arg(firstTime));
					}

					if (hasReturnNumber)
					{
						if (returnNumberSF)
						{
							returnNumberSF->computeMinAndMax();
							int rMin = (int)returnNumberSF->getMin();
							int rMax = (int)returnNumberSF->getMax();
							returnNumberSF->setColorRampSteps(rMax-rMin);
							int sfIndex = loadedCloud->addScalarField(returnNumberSF);
							if (!loadedCloud->hasDisplayedScalarField())
							{
								loadedCloud->setCurrentDisplayedScalarField(sfIndex);
								loadedCloud->showSF(!thisChunkHasColors);
							}
						}
						else
							ccLog::Warning(QString("[LAS FILE] All return numbers were the same (%1)! We ignored them...").arg(firstReturnNumber));
					}

					//if we have reserved too much memory
					if (loadedCloud->size() < loadedCloud->capacity())
						loadedCloud->resize(loadedCloud->size());

					QString chunkName("unnamed - Cloud");
					unsigned n = container.getChildrenNumber();
					if (n!=0) //if we have more than one cloud, we append an index
					{
						if (n==1)  //we must also update the first one!
							container.getChild(0)->setName(chunkName+QString(" #1"));
						chunkName += QString(" #%1").arg(n+1);
					}
					loadedCloud->setName(chunkName);

					container.addChild(loadedCloud);
					loadedCloud=0;
				}
				else
				{
					//empty cloud?!
					delete loadedCloud;
					loadedCloud=0;
				}

				if (classifSF)
					classifSF->release();
				classifSF=0;
				if (intensitySF)
					intensitySF->release();
				intensitySF=0;
				if (returnNumberSF)
					returnNumberSF->release();
				returnNumberSF=0;
				if (timeSF)
					timeSF->release();
				timeSF=0;
			}

			if (!newPointAvailable)
				break; //end of the file (or cancel requested)

			//otherwise, we must create a new cloud
			fileChunkPos = pointsRead;
			fileChunkSize = ccMin(nbOfPoints-pointsRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
			loadedCloud = new ccPointCloud();
			if (!loadedCloud->reserveThePointsTable(fileChunkSize))
			{
				ccLog::Warning("[LASFilter::loadFile] Not enough memory!");
				delete loadedCloud;
				delete reader;
				ifs.close();
				return CC_FERR_NOT_ENOUGH_MEMORY;
			}
			loadedCloud->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);

			//DGM: from now on, we only enable scalar fields when we detect a valid value!
			if (hasClassif)
			{
				assert(!classifSF);
				firstClassifValue = 0;
			}

			if (hasTime)
			{
				assert(!timeSF);
				firstTime = 0.0;
			}

			if (hasIntensity)
			{
				assert(!intensitySF);
				firstIntensity=0;
			}

			if (hasReturnNumber)
			{
				assert(!returnNumberSF);
				firstReturnNumber = 0;
			}
		}

		assert(newPointAvailable);
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

		//color field
		if (hasColor)
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
						//we must set the color (black) of all the precedently skipped points
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							loadedCloud->addRGBColor(ccColor::black);
					}
					else
					{
						ccConsole::Warning("[LAS FILE] Not enough memory: color field will be ignored!");
						hasColor = false; //no need to retry with the other chunks anyway
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
				if (colorCompBitDec==0)
				{
					if (	(col[0] & 0xFF00)
						||  (col[1] & 0xFF00)
						||  (col[2] & 0xFF00))
					{
						//the color components are on 16 bits!
						ccLog::Print("[LAS FILE] Color components are coded on 16 bits");
						colorCompBitDec = 8;
						//we fix all the precedently read colors
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							loadedCloud->setPointColor(i,ccColor::black); //255 >> 8 = 0!
					}
				}
				
				rgb[0]=(colorType)(col[0]>>colorCompBitDec);
				rgb[1]=(colorType)(col[1]>>colorCompBitDec);
				rgb[2]=(colorType)(col[2]>>colorCompBitDec);

				loadedCloud->addRGBColor(rgb);
			}
		}

		if (hasClassif)
		{
			uint8_t intValue = p.GetClassification().GetClass();
			if (classifSF)
			{
				classifSF->addElement(intValue);
			}
			else
			{
				//first point? we track its value
				if (loadedCloud->size()==1)
				{
					firstClassifValue = intValue;
				}
				else if (intValue != firstClassifValue)
				{
					classifSF = new ccScalarField(CC_LAS_CLASSIFICATION_FIELD_NAME,true);
					if (classifSF->reserve(fileChunkSize))
					{
						classifSF->link();
						//we must set the classification value (firstClassifValue) of all the precedently skipped points
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							classifSF->addElement(firstClassifValue);
						classifSF->addElement(intValue);
					}
					else
					{
						ccConsole::Warning("[LAS FILE] Not enough memory: classificaiton field will be ignored!");
						hasClassif = false; //no need to retry with the other chunks anyway
						classifSF->release();
						classifSF=0;
					}
				}
			}
		}

		if (hasTime)
		{
			double timeValue = p.GetTime();

			if (timeSF)
			{
				timeSF->addElement(timeValue);
			}
			else
			{
				//first point? we track its value
				if (loadedCloud->size()==1)
				{
					firstTime = timeValue;
				}
				else if (timeValue != firstIntensity)
				{
					timeSF = new ccScalarField("Time",true);
					if (timeSF->reserve(fileChunkSize))
					{
						timeSF->link();
						//we must set the timestamp value (firstTime) of all the precedently skipped points
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							timeSF->addElement(firstTime);
						timeSF->addElement(timeValue);
					}
					else
					{
						ccConsole::Warning("[LAS FILE] Not enough memory: 'time' field will be ignored!");
						hasTime = false; //no need to retry with the other chunks anyway
						timeSF->release();
						timeSF=0;
					}
				}
			}
		}

		if (hasIntensity)
		{
			uint16_t intValue = p.GetIntensity();
			if (intensitySF)
			{
				intensitySF->addElement(intValue);
			}
			else
			{
				//first point? we track its value
				if (loadedCloud->size()==1)
				{
					firstIntensity = intValue;
				}
				else if (intValue != firstIntensity)
				{
					intensitySF = new ccScalarField("Intensity",true);
					if (intensitySF->reserve(fileChunkSize))
					{
						intensitySF->link();
						//we must set the intensity (firstIntensity) of all the precedently skipped points
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							intensitySF->addElement(firstIntensity);
						intensitySF->addElement(intValue);
					}
					else
					{
						ccConsole::Warning("[LAS FILE] Not enough memory: intensity field will be ignored!");
						hasIntensity = false; //no need to retry with the other chunks anyway
						intensitySF->release();
						intensitySF=0;
					}
				}
			}
		}

		if (hasReturnNumber)
		{
			uint16_t intValue = p.GetReturnNumber();
			if (returnNumberSF)
			{
				returnNumberSF->addElement(intValue);
			}
			else
			{
				//first point? we track its value
				if (loadedCloud->size()==1)
				{
					firstReturnNumber = intValue;
				}
				else if (intValue != firstReturnNumber)
				{
					returnNumberSF = new ccScalarField("Return number",true);
					if (returnNumberSF->reserve(fileChunkSize))
					{
						returnNumberSF->link();
						//we must set the return index (firstReturnNumber) of all the precedently skipped points
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							returnNumberSF->addElement(firstReturnNumber);
						returnNumberSF->addElement(intValue);
					}
					else
					{
						ccConsole::Warning("[LAS FILE] Not enough memory: return number field will be ignored!");
						hasReturnNumber = false; //no need to retry with the other chunks anyway
						returnNumberSF->release();
						returnNumberSF=0;
					}
				}
			}

		}

		++pointsRead;
	}

	if (reader)
		delete reader;
	reader=0;
	ifs.close();

	return CC_FERR_NO_ERROR;
}

#endif
