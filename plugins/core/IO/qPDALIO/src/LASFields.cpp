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
//#                       COPYRIGHT: CloudCompare project                  #
//#                                                                        #
//##########################################################################

#include "LASFields.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

LasField::LasField(	LAS_FIELDS fieldType/*=LAS_INVALID*/,
					double defaultVal/*=0*/,
					double min/*=0.0*/,
					double max/*=-1.0*/)
	: type(fieldType)
	, sf(0)
	, firstValue(0.0)
	, minValue(min)
	, maxValue(max)
	, defaultValue(defaultVal)
{}

bool LasField::GetLASFields(ccPointCloud* cloud, std::vector<LasField>& fieldsToSave)
{
	try
	{
		//official LAS fields
		std::vector<LasField> lasFields;
		lasFields.reserve(14);
		{
			lasFields.emplace_back(LAS_CLASSIFICATION, 0, 0, 255); //unsigned char: between 0 and 255
			lasFields.emplace_back(LAS_CLASSIF_VALUE, 0, 0, 31); //5 bits: between 0 and 31
			lasFields.emplace_back(LAS_CLASSIF_SYNTHETIC, 0, 0, 1); //1 bit: 0 or 1
			lasFields.emplace_back(LAS_CLASSIF_KEYPOINT, 0, 0, 1); //1 bit: 0 or 1
			lasFields.emplace_back(LAS_CLASSIF_WITHHELD, 0, 0, 1); //1 bit: 0 or 1
			lasFields.emplace_back(LAS_INTENSITY, 0, 0, 65535); //16 bits: between 0 and 65536
			lasFields.emplace_back(LAS_TIME, 0, 0, -1.0); //8 bytes (double)
			lasFields.emplace_back(LAS_RETURN_NUMBER, 1, 1, 7); //3 bits: between 1 and 7
			lasFields.emplace_back(LAS_NUMBER_OF_RETURNS, 1, 1, 7); //3 bits: between 1 and 7
			lasFields.emplace_back(LAS_SCAN_DIRECTION, 0, 0, 1); //1 bit: 0 or 1
			lasFields.emplace_back(LAS_FLIGHT_LINE_EDGE, 0, 0, 1); //1 bit: 0 or 1
			lasFields.emplace_back(LAS_SCAN_ANGLE_RANK, 0, -90, 90); //signed char: between -90 and +90
			lasFields.emplace_back(LAS_USER_DATA, 0, 0, 255); //unsigned char: between 0 and 255
			lasFields.emplace_back(LAS_POINT_SOURCE_ID, 0, 0, 65535); //16 bits: between 0 and 65536
		}

		//we are going to check now the existing cloud SFs
		for (unsigned i = 0; i < cloud->getNumberOfScalarFields(); ++i)
		{
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(i));
			//find an equivalent in official LAS fields
			QString sfName = QString(sf->getName()).toUpper();
			bool outBounds = false;
			for (size_t j = 0; j < lasFields.size(); ++j)
			{
				//if the name matches
				if (sfName == lasFields[j].getName().toUpper())
				{
					//check bounds
					double sfMin = sf->getGlobalShift() + sf->getMax();
					double sfMax = sf->getGlobalShift() + sf->getMax();
					if (sfMin < lasFields[j].minValue || (lasFields[j].maxValue != -1.0 && sfMax > lasFields[j].maxValue)) //outbounds?
					{
						ccLog::Warning(QString("[LAS] Found a '%1' scalar field, but its values outbound LAS specifications (%2-%3)...").arg(sf->getName()).arg(lasFields[j].minValue).arg(lasFields[j].maxValue));
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
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[LasField::GetLASFields] Not enough memory");
		return false;
	}

	return true;
}
