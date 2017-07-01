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

//Qt
#include <QSharedPointer>

//System
#include <vector>

class ccScalarField;
class ccPointCloud;

static const char LAS_SCALE_X_META_DATA[] = "LAS.scale.x";
static const char LAS_SCALE_Y_META_DATA[] = "LAS.scale.y";
static const char LAS_SCALE_Z_META_DATA[] = "LAS.scale.z";

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
									"Time",
									"extra",
									"[Classif] Value",
									"[Classif] Synthetic flag",
									"[Classif] Key-point flag",
									"[Classif] Withheld flag",
};

//! LAS field descriptor
struct QCC_IO_LIB_API LasField
{
	//! Shared type
	typedef QSharedPointer<LasField> Shared;

	//! Default constructor
	LasField(LAS_FIELDS fieldType = LAS_INVALID, double defaultVal = 0, double min = 0.0, double max = -1.0);

	//! Returns official field name
	virtual inline QString getName() const { return type < LAS_INVALID ? QString(LAS_FIELD_NAMES[type]) : QString(); }

	//! Returns the (compliant) LAS fields in a point cloud
	static bool GetLASFields(ccPointCloud* cloud, std::vector<LasField>& fieldsToSave);

	LAS_FIELDS type;
	ccScalarField* sf;
	double firstValue;
	double minValue;
	double maxValue;
	double defaultValue;
};

#endif //CC_LAS_FIELDS_HEADER
