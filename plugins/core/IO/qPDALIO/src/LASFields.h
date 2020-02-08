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

#ifndef CC_LAS_FIELDS_HEADER
#define CC_LAS_FIELDS_HEADER

//Local
#include "qCC_io.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>

//Qt
#include <QSharedPointer>

//System
#include <vector>

class ccScalarField;
class ccPointCloud;

static const char LAS_SCALE_X_META_DATA[] = "LAS.scale.x";
static const char LAS_SCALE_Y_META_DATA[] = "LAS.scale.y";
static const char LAS_SCALE_Z_META_DATA[] = "LAS.scale.z";
static const char LAS_OFFSET_X_META_DATA[] = "LAS.offset.x";
static const char LAS_OFFSET_Y_META_DATA[] = "LAS.offset.y";
static const char LAS_OFFSET_Z_META_DATA[] = "LAS.offset.z";
static const char LAS_VERSION_MAJOR_META_DATA[] = "LAS.version.major";
static const char LAS_VERSION_MINOR_META_DATA[] = "LAS.version.minor";
static const char LAS_POINT_FORMAT_META_DATA[] = "LAS.point_format";
static const char LAS_GLOBAL_ENCODING_META_DATA[] = "LAS.global_encoding";
static const char LAS_PROJECT_UUID_META_DATA[] = "LAS.project_uuid";

enum LAS_FIELDS {
	LAS_X = 0,
	LAS_Y = 1,
	LAS_Z = 2,
	LAS_INTENSITY = 3,
	LAS_RETURN_NUMBER = 4,
	LAS_NUMBER_OF_RETURNS = 5,
	LAS_SCAN_DIRECTION = 6,
	LAS_FLIGHT_LINE_EDGE = 7,
	LAS_CLASSIFICATION = 8,
	LAS_SCAN_ANGLE_RANK = 9,
	LAS_USER_DATA = 10,
	LAS_POINT_SOURCE_ID = 11,
	LAS_RED = 12,
	LAS_GREEN = 13,
	LAS_BLUE = 14,
	LAS_TIME = 15,
	LAS_EXTRA = 16,
	//Sub fields
	LAS_CLASSIF_VALUE = 17,
	LAS_CLASSIF_SYNTHETIC = 18,
	LAS_CLASSIF_KEYPOINT = 19,
	LAS_CLASSIF_WITHHELD = 20,
	LAS_CLASSIF_OVERLAP = 21,
	//Invald flag
	LAS_INVALID = 255
};

const char LAS_FIELD_NAMES[][28] = {"X",
									"Y",
									"Z",
									"Intensity",
									"ReturnNumber",
									"NumberOfReturns",
									"ScanDirectionFlag",
									"EdgeOfFlightLine",
									"Classification",
									"ScanAngleRank",
									"UserData",
									"PointSourceId",
									"Red",
									"Green",
									"Blue",
									"GpsTime",
									"extra",
									"[Classif] Value",
									"[Classif] Synthetic flag",
									"[Classif] Key-point flag",
									"[Classif] Withheld flag",
									"[Classif] Overlap flag",
};

//! LAS field descriptor
struct LasField
{
	//! Shared type
	typedef QSharedPointer<LasField> Shared;

	//! Default constructor
	LasField(LAS_FIELDS fieldType = LAS_INVALID, double defaultVal = 0, double min = 0.0, double max = -1.0, uint8_t _minPointFormat = 0)
		: type(fieldType)
		, sf(nullptr)
		, firstValue(0.0)
		, minValue(min)
		, maxValue(max)
		, defaultValue(defaultVal)
		, minPointFormat(_minPointFormat)
	{}

	//! Returns official field name
	virtual inline QString getName() const { return type < LAS_INVALID ? QString(LAS_FIELD_NAMES[type]) : QString(); }

	//! Returns the (compliant) LAS fields in a point cloud
	static bool GetLASFields(ccPointCloud* cloud, std::vector<LasField>& fieldsToSave, uint8_t minPointFormat)
	{
		try
		{
			//official LAS fields
			std::vector<LasField> lasFields;
			lasFields.reserve(14);
			{
				lasFields.emplace_back(LAS_CLASSIFICATION, 0, 0, 255, 0); //unsigned char: between 0 and 255
				lasFields.emplace_back(LAS_CLASSIF_VALUE, 0, 0, 31, 0); //5 bits: between 0 and 31
				lasFields.emplace_back(LAS_CLASSIF_SYNTHETIC, 0, 0, 1, 0); //1 bit: 0 or 1
				lasFields.emplace_back(LAS_CLASSIF_KEYPOINT, 0, 0, 1, 0); //1 bit: 0 or 1
				lasFields.emplace_back(LAS_CLASSIF_WITHHELD, 0, 0, 1, 0); //1 bit: 0 or 1
				lasFields.emplace_back(LAS_CLASSIF_OVERLAP, 0, 0, 1, 6); //1 bit: 0 or 1
				lasFields.emplace_back(LAS_INTENSITY, 0, 0, 65535, 0); //16 bits: between 0 and 65536
				lasFields.emplace_back(LAS_TIME, 0, 0, -1.0, 1); //8 bytes (double)
				lasFields.emplace_back(LAS_RETURN_NUMBER, 1, 1, 7, 0); //3 bits: between 1 and 7
				lasFields.emplace_back(LAS_NUMBER_OF_RETURNS, 1, 1, 7, 0); //3 bits: between 1 and 7
				lasFields.emplace_back(LAS_SCAN_DIRECTION, 0, 0, 1, 0); //1 bit: 0 or 1
				lasFields.emplace_back(LAS_FLIGHT_LINE_EDGE, 0, 0, 1, 0); //1 bit: 0 or 1
				lasFields.emplace_back(LAS_SCAN_ANGLE_RANK, 0, -90, 90, 0); //signed char: between -90 and +90
				lasFields.emplace_back(LAS_USER_DATA, 0, 0, 255, 0); //unsigned char: between 0 and 255
				lasFields.emplace_back(LAS_POINT_SOURCE_ID, 0, 0, 65535, 0); //16 bits: between 0 and 65536
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

							minPointFormat = std::max(minPointFormat, fieldsToSave.back().minPointFormat);
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

	static unsigned GetFormatRecordLength(uint8_t pointFormat)
	{
		switch (pointFormat)
		{
		case 0:
			return 20;              //0 - base
		case 1:
			return 20 + 8;          //1 - base + GPS
		case 2:
			return 20 + 6;          //2 - base + RGB
		case 3:
			return 20 + 8 + 6;      //3 - base + GPS + RGB
		case 4:
			return 20 + 8 + 29;     //4 - base + GPS + FWF
		case 5:
			return 20 + 8 + 6 + 29; //5 - base + GPS + FWF + RGB
		case 6:
			return 30;              //6  - base (GPS included)
		case 7:
			return 30 + 6;          //7  - base + RGB
		case 8:
			return 30 + 6 + 2;      //8  - base + RGB + NIR (not used)
		case 9:
			return 30 + 29;         //9  - base + FWF
		case 10:
			return 30 + 6 + 2 + 29; //10 - base + RGB + NIR + FWF
		default:
			assert(false);
			return 0;
		}
	}

	static uint8_t VersionMinorForPointFormat(uint8_t pointFormat) {
		return pointFormat >= 6 ? 4 : 2;
	}

	static uint8_t UpdateMinPointFormat(uint8_t minPointFormat, bool withRGB, bool withFWF, bool allowLegacyFormats = true)
	{
		//can we keep the (short) legacy formats?
		if (allowLegacyFormats && minPointFormat < 6)
		{
			//LAS formats:
			//0 - base
			//1 - base + GPS TIME
			//2 - base + RGB
			//3 - base + GPS + RGB
			//4 - base + GPS + FWF
			//5 - base + GPS + FWF + RGB

			if (withFWF)
			{
				//0, 1 --> 4
				minPointFormat = std::max(minPointFormat, (uint8_t)4);
			}

			if (withRGB)
			{
				if (minPointFormat < 2)
				{
					//0 --> 2
					//1 --> 3
					minPointFormat += 2;
				}
				else if (minPointFormat == 4)
				{
					//4 --> 5
					minPointFormat = 5;
				}
				//else the format already has colors
			}
		}
		else //we'll use extended versions (up to 15 returns, up to 256 classes for classification, higher precision scan angle)
		{
			//new LAS formats:
			//6  - base (GPS included)
			//7  - base + RGB
			//8  - base + RGB + NIR (not used)
			//9  - base + FWF
			//10 - base + FWF + RGB + NIR
			assert(minPointFormat <= 6); //in effect, standard LAS fields will only require version 6 at most

			minPointFormat = std::max(minPointFormat, (uint8_t)6);
			//FWF data?
			if (withFWF)
			{
				//6 --> 9
				minPointFormat = std::max(minPointFormat, (uint8_t)9);
			}
			//Colors?
			if (withRGB)
			{
				if (minPointFormat == 6)
				{
					//6 --> 7
					minPointFormat = 7;
				}
				else if (minPointFormat == 9)
				{
					//9 --> 10
					minPointFormat = 10;
				}
			}
		}

		return minPointFormat;
	}

	static QString SanitizeString(QString str)
	{
		QString sanitizedStr;
		if (str.size() > 32)
		{
			sanitizedStr = str.left(32);
		}
		else
		{
			sanitizedStr = str;
		}
		sanitizedStr.replace('=', '_');

		return sanitizedStr;
	}

	LAS_FIELDS type;
	ccScalarField* sf;
	double firstValue;
	double minValue;
	double maxValue;
	double defaultValue;
	uint8_t minPointFormat;
};

#endif //CC_LAS_FIELDS_HEADER
