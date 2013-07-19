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

#ifdef CC_LAS_SUPPORT

#include "LASFilter.h"

//qCC
#include "../ccCommon.h"
#include "../ccCoordinatesShiftManager.h"
#include "LASOpenDlg.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Liblas
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/factory.hpp>	// liblas::ReaderFactory

//Qt
#include <QFileInfo>

//System
#include <string.h>
#include <fstream>				// std::ifstream
#include <iostream>				// std::cout

//LAS field descriptor
struct LasField
{
	LAS_FIELDS type;
	ccScalarField* sf;
	double firstValue;
	double minValue;
	double maxValue;
	double defaultValue;

	LasField() : type(LAS_INVALID), sf(0), firstValue(0.0), minValue(0.0), maxValue(-1.0), defaultValue(0.0) {}
	LasField(LAS_FIELDS fieldType, double defaultVal, double min, double max) : type(fieldType), sf(0), firstValue(0.0), minValue(min), maxValue(max), defaultValue(defaultVal) {}

	//! Returns officiel field name
	inline const char* getName() { return (type < LAS_INVALID ? LAS_FIELD_NAMES[type] : 0); }
};

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
	ccGenericPointCloud* theCloud = ccHObjectCaster::ToGenericPointCloud(clouds[0]);
	unsigned numberOfPoints = theCloud->size();

	if (numberOfPoints==0)
	{
		ccConsole::Error("Cloud is empty!");
		return CC_FERR_BAD_ENTITY_TYPE;
	}

	//colors
	bool hasColor = theCloud->hasColors();

	//additional fields (as scalar fields)
	std::vector<LasField> fieldsToSave;

	if (theCloud->isA(CC_POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(theCloud);

		//Match cloud SFs with official LAS fields
		{
			//official LAS fields
			std::vector<LasField> lasFields;
			{
				lasFields.push_back(LasField(LAS_CLASSIFICATION,0,0,255)); //unsigned char: between 0 and 255
				lasFields.push_back(LasField(LAS_CLASSIF_VALUE,0,0,31)); //5 bits: between 0 and 31
				lasFields.push_back(LasField(LAS_CLASSIF_SYNTHETIC,0,0,1)); //1 bit: 0 or 1
				lasFields.push_back(LasField(LAS_CLASSIF_KEYPOINT,0,0,1)); //1 bit: 0 or 1
				lasFields.push_back(LasField(LAS_CLASSIF_WITHHELD,0,0,1)); //1 bit: 0 or 1
				lasFields.push_back(LasField(LAS_INTENSITY,0,0,65535)); //16 bits: between 0 and 65536
				lasFields.push_back(LasField(LAS_TIME,0,0,-1.0)); //8 bytes (double)
				lasFields.push_back(LasField(LAS_RETURN_NUMBER,1,1,7)); //3 bits: between 1 and 7
				lasFields.push_back(LasField(LAS_NUMBER_OF_RETURNS,1,1,7)); //3 bits: between 1 and 7
				lasFields.push_back(LasField(LAS_SCAN_DIRECTION,0,0,1)); //1 bit: 0 or 1
				lasFields.push_back(LasField(LAS_FLIGHT_LINE_EDGE,0,0,1)); //1 bit: 0 or 1
				lasFields.push_back(LasField(LAS_SCAN_ANGLE_RANK,0,-90,90)); //signed char: between -90 and +90
				lasFields.push_back(LasField(LAS_USER_DATA,0,0,255)); //unsigned char: between 0 and 255
				lasFields.push_back(LasField(LAS_POINT_SOURCE_ID,0,0,65535)); //16 bits: between 0 and 65536
			}

			//we are going to check now the existing cloud SFs
			for (unsigned i=0; i<pc->getNumberOfScalarFields(); ++i)
			{
				ccScalarField* sf = static_cast<ccScalarField*>(pc->getScalarField(i));
				//find an equivalent in official LAS fields
				QString sfName = QString(sf->getName()).toUpper();
				bool outBounds = false;
				for (size_t j=0; j<lasFields.size(); ++j)
				{
					//if the name matches
					if (sfName == QString(lasFields[j].getName()).toUpper())
					{
						//check bounds
						if (sf->getMin() < lasFields[j].minValue || (lasFields[j].maxValue != -1.0 && sf->getMax() > lasFields[j].maxValue)) //outbounds?
						{
							ccLog::Warning(QString("[LASFilter] Found a '%1' scalar field, but its values outbound LAS specifications (%2-%3)...").arg(sf->getName()).arg(lasFields[j].minValue).arg(lasFields[j].maxValue));
							outBounds = true;
						}
						else
						{
							//we add the SF to the list of saved fields
							fieldsToSave.push_back(lasFields[j]);
							fieldsToSave.back().sf = sf;
						}
						break;
					}
				}
				
				//no correspondance was found?
				if (!outBounds && (fieldsToSave.empty() || fieldsToSave.back().sf != sf))
				{
					ccLog::Warning(QString("[LASFilter] Found a '%1' scalar field, but it doesn't match with any of the official LAS fields... we will ignore it!").arg(sf->getName()));
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
	liblas::Classification classif = point.GetClassification();

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
				point.SetIntensity((boost::uint16_t)it->sf->getValue(i));
				break;
			case LAS_RETURN_NUMBER:
				point.SetReturnNumber((boost::uint16_t)it->sf->getValue(i));
				break;
			case LAS_NUMBER_OF_RETURNS:
				point.SetNumberOfReturns((boost::uint16_t)it->sf->getValue(i));
				break;
			case LAS_SCAN_DIRECTION:
				point.SetScanDirection((boost::uint16_t)it->sf->getValue(i));
				break;
			case LAS_FLIGHT_LINE_EDGE:
				point.SetFlightLineEdge((boost::uint16_t)it->sf->getValue(i));
				break;
			case LAS_CLASSIFICATION:
				{
					boost::uint32_t val = (boost::uint32_t)it->sf->getValue(i);
					classif.SetClass(val & 31);		//first 5 bits
					classif.SetSynthetic(val & 32); //6th bit
					classif.SetKeyPoint(val & 64);	//7th bit
					classif.SetWithheld(val & 128);	//8th bit
				}
				break;
			case LAS_SCAN_ANGLE_RANK:
				point.SetScanAngleRank((boost::uint8_t)it->sf->getValue(i));
				break;
			case LAS_USER_DATA:
				point.SetUserData((boost::uint8_t)it->sf->getValue(i));
				break;
			case LAS_POINT_SOURCE_ID:
				point.SetPointSourceID((boost::uint16_t)it->sf->getValue(i));
				break;
			case LAS_RED:
			case LAS_GREEN:
			case LAS_BLUE:
				assert(false);
				break;
			case LAS_TIME:
				point.SetTime((double)it->sf->getValue(i));
				break;
			case LAS_CLASSIF_VALUE:
				classif.SetClass((boost::uint32_t)it->sf->getValue(i));
				break;
			case LAS_CLASSIF_SYNTHETIC:
				classif.SetSynthetic((boost::uint32_t)it->sf->getValue(i));
				break;
			case LAS_CLASSIF_KEYPOINT:
				classif.SetKeyPoint((boost::uint32_t)it->sf->getValue(i));
				break;
			case LAS_CLASSIF_WITHHELD:
				classif.SetWithheld((boost::uint32_t)it->sf->getValue(i));
				break;
			case LAS_INVALID:
			default:
				assert(false);
				break;
			}
		}

		//set classification (it's mandatory anyway ;)
		point.SetClassification(classif);

		writer->WritePoint(point);

		if (!nprogress.oneStep())
			break;
	}

	delete writer;
	ofs.close();

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

		ccLog::PrintDebug(QString("[LAS FILE] %1 - signature: %2").arg(filename).arg(header.GetFileSignature().c_str()));

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

	//dialog to choose the fields to load
	LASOpenDlg dlg;
	dlg.setDimensions(dimensions);
	if (alwaysDisplayLoadDialog && !dlg.exec())
	{
		delete reader;
		ifs.close();
		return CC_FERR_CANCELED_BY_USER;
	}
	bool ignoreDefaultFields = dlg.ignoreDefaultFieldsCheckBox->isChecked();

	//RGB color
	liblas::Color rgbColorMask; //(0,0,0) on construction
	if (dlg.doLoad(LAS_RED))
		rgbColorMask.SetRed(~0);
	if (dlg.doLoad(LAS_GREEN))
		rgbColorMask.SetGreen(~0);
	if (dlg.doLoad(LAS_BLUE))
		rgbColorMask.SetBlue(~0);
	bool loadColor = (rgbColorMask[0] || rgbColorMask[1] || rgbColorMask[2]);

	//progress dialog
	ccProgressDialog pdlg(true); //cancel available
	CCLib::NormalizedProgress nprogress(&pdlg,nbOfPoints);
	pdlg.setMethodTitle("Open LAS file");
	pdlg.setInfo(qPrintable(QString("Points: %1").arg(nbOfPoints)));
	pdlg.start();

	//number of points read from the begining of the current cloud part
	unsigned pointsRead = 0;
	double Pshift[3] = {0.0,0.0,0.0};

	//by default we read color as 8 bits integers and we will change this to 16 bits if it's not (16 bits is the standard!)
	unsigned char colorCompBitDec = 0;
	colorType rgb[3] = {0,0,0};

	ccPointCloud* loadedCloud = 0;
	std::vector<LasField> fieldsToLoad;

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
					if (loadColor && !thisChunkHasColors)
						ccLog::Warning("[LAS FILE] Color field was all black! We ignored it...");

					while (!fieldsToLoad.empty())
					{
						LasField& field = fieldsToLoad.back();
						if (field.sf)
						{
							field.sf->computeMinAndMax();

							if (field.type == LAS_CLASSIFICATION
								|| field.type == LAS_RETURN_NUMBER
								|| field.type == LAS_NUMBER_OF_RETURNS)
							{
								int cMin = (int)field.sf->getMin();
								int cMax = (int)field.sf->getMax();
								field.sf->setColorRampSteps(std::min<int>(cMax-cMin+1,256));
								//classifSF->setMinSaturation(cMin);
							}
							else if (field.type == LAS_INTENSITY)
							{
								field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
							}

							int sfIndex = loadedCloud->addScalarField(field.sf);
							if (!loadedCloud->hasDisplayedScalarField())
							{
								loadedCloud->setCurrentDisplayedScalarField(sfIndex);
								loadedCloud->showSF(!thisChunkHasColors);
							}
							field.sf->release();
							field.sf=0;
						}
						else
						{
							ccLog::Warning(QString("[LAS FILE] All '%1' values were the same (%2)! We ignored them...").arg(LAS_FIELD_NAMES[field.type]).arg(field.firstValue));
						}

						fieldsToLoad.pop_back();
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
			}

			if (!newPointAvailable)
				break; //end of the file (or cancel requested)

			//otherwise, we must create a new cloud
			fileChunkPos = pointsRead;
			fileChunkSize = std::min(nbOfPoints-pointsRead,CC_MAX_NUMBER_OF_POINTS_PER_CLOUD);
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
			if (dlg.doLoad(LAS_CLASSIFICATION))
					fieldsToLoad.push_back(LasField(LAS_CLASSIFICATION,0,0,255)); //unsigned char: between 0 and 255
			if (dlg.doLoad(LAS_CLASSIF_VALUE))
				fieldsToLoad.push_back(LasField(LAS_CLASSIF_VALUE,0,0,31)); //5 bits: between 0 and 31
			if (dlg.doLoad(LAS_CLASSIF_SYNTHETIC))
				fieldsToLoad.push_back(LasField(LAS_CLASSIF_SYNTHETIC,0,0,1)); //1 bit: 0 or 1
			if (dlg.doLoad(LAS_CLASSIF_KEYPOINT))
				fieldsToLoad.push_back(LasField(LAS_CLASSIF_KEYPOINT,0,0,1)); //1 bit: 0 or 1
			if (dlg.doLoad(LAS_CLASSIF_WITHHELD))
				fieldsToLoad.push_back(LasField(LAS_CLASSIF_WITHHELD,0,0,1)); //1 bit: 0 or 1
			if (dlg.doLoad(LAS_INTENSITY))
				fieldsToLoad.push_back(LasField(LAS_INTENSITY,0,0,65535)); //16 bits: between 0 and 65536
			if (dlg.doLoad(LAS_TIME))
				fieldsToLoad.push_back(LasField(LAS_TIME,0,0,-1.0)); //8 bytes (double)
			if (dlg.doLoad(LAS_RETURN_NUMBER))
				fieldsToLoad.push_back(LasField(LAS_RETURN_NUMBER,1,1,7)); //3 bits: between 1 and 7
			if (dlg.doLoad(LAS_NUMBER_OF_RETURNS))
				fieldsToLoad.push_back(LasField(LAS_NUMBER_OF_RETURNS,1,1,7)); //3 bits: between 1 and 7
			if (dlg.doLoad(LAS_SCAN_DIRECTION))
				fieldsToLoad.push_back(LasField(LAS_SCAN_DIRECTION,0,0,1)); //1 bit: 0 or 1
			if (dlg.doLoad(LAS_FLIGHT_LINE_EDGE))
				fieldsToLoad.push_back(LasField(LAS_FLIGHT_LINE_EDGE,0,0,1)); //1 bit: 0 or 1
			if (dlg.doLoad(LAS_SCAN_ANGLE_RANK))
				fieldsToLoad.push_back(LasField(LAS_SCAN_ANGLE_RANK,0,-90,90)); //signed char: between -90 and +90
			if (dlg.doLoad(LAS_USER_DATA))
				fieldsToLoad.push_back(LasField(LAS_USER_DATA,0,0,255)); //unsigned char: between 0 and 255
			if (dlg.doLoad(LAS_POINT_SOURCE_ID))
				fieldsToLoad.push_back(LasField(LAS_POINT_SOURCE_ID,0,0,65535)); //16 bits: between 0 and 65536
		}

		assert(newPointAvailable);
		const liblas::Point& p = reader->GetPoint();

		//first point: check for 'big' coordinates
		if (pointsRead==0)
		{
			double P[3] = {p.GetX(),p.GetY(),p.GetZ()};
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
						//we must set the color (black) of all the precedently skipped points
						for (unsigned i=0;i<loadedCloud->size()-1;++i)
							loadedCloud->addRGBColor(ccColor::black);
					}
					else
					{
						ccConsole::Warning("[LAS FILE] Not enough memory: color field will be ignored!");
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
		
		//additional fields
		for (std::vector<LasField>::iterator it = fieldsToLoad.begin(); it != fieldsToLoad.end(); ++it)
		{
			double value = 0.0;
			switch (it->type)
			{
			case LAS_X:
			case LAS_Y:
			case LAS_Z:
				assert(false);
				break;
			case LAS_INTENSITY:
				value = (double)p.GetIntensity();
				break;
			case LAS_RETURN_NUMBER:
				value = (double)p.GetReturnNumber();
				break;
			case LAS_NUMBER_OF_RETURNS:
				value = (double)p.GetNumberOfReturns();
				break;
			case LAS_SCAN_DIRECTION:
				value = (double)p.GetScanDirection();
				break;
			case LAS_FLIGHT_LINE_EDGE:
				value = (double)p.GetFlightLineEdge();
				break;
			case LAS_CLASSIFICATION:
				value = (double)p.GetClassification().GetClass();
				break;
			case LAS_SCAN_ANGLE_RANK:
				value = (double)p.GetScanAngleRank();
				break;
			case LAS_USER_DATA:
				value = (double)p.GetUserData();
				break;
			case LAS_POINT_SOURCE_ID:
				value = (double)p.GetPointSourceID();
				break;
			case LAS_RED:
			case LAS_GREEN:
			case LAS_BLUE:
				assert(false);
				break;
			case LAS_TIME:
				value = p.GetTime();
				break;
			case LAS_CLASSIF_VALUE:
				value = (double)(p.GetClassification().GetClass() & 31); //5 bits
				break;
			case LAS_CLASSIF_SYNTHETIC:
				value = (double)(p.GetClassification().GetClass() & 32); //bit #6
				break;
			case LAS_CLASSIF_KEYPOINT:
				value = (double)(p.GetClassification().GetClass() & 64); //bit #7
				break;
			case LAS_CLASSIF_WITHHELD:
				value = (double)(p.GetClassification().GetClass() & 128); //bit #8
				break;
			case LAS_INVALID:
			default:
				assert(false);
				break;
			}

			if (it->sf)
			{
				it->sf->addElement(value);
			}
			else
			{
				//first point? we track its value
				if (loadedCloud->size() == 1)
				{
					it->firstValue = value;
				}
				
				if (!ignoreDefaultFields || value != it->firstValue || it->firstValue != it->defaultValue)
				{
					it->sf = new ccScalarField(qPrintable(it->getName()));
					if (it->sf->reserve(fileChunkSize))
					{
						it->sf->link();
						//we must set the value (firstClassifValue) of all the precedently skipped points
						for (unsigned i=0; i<loadedCloud->size()-1; ++i)
							it->sf->addElement(it->firstValue);
						it->sf->addElement(value);
					}
					else
					{
						ccConsole::Warning(QString("[LAS FILE] Not enough memory: '%1' field will be ignored!").arg(LAS_FIELD_NAMES[it->type]));
						it->sf->release();
						it->sf = 0;
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
