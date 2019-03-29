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

#ifndef CC_PLY_FILTER_HEADER
#define CC_PLY_FILTER_HEADER

#include "FileIOFilter.h"
#include "rply.h"

//! PLY format types
static const char e_ply_type_names[][12]= {
	"PLY_INT8", "PLY_UINT8", "PLY_INT16", "PLY_UINT16",
	"PLY_INT32", "PLY_UIN32", "PLY_FLOAT32", "PLY_FLOAT64",
	"PLY_CHAR", "PLY_UCHAR", "PLY_SHORT", "PLY_USHORT",
	"PLY_INT", "PLY_UINT", "PLY_FLOAT", "PLY_DOUBLE",
	"PLY_LIST"
};

//! PLY format storage modes
static const char e_ply_storage_mode_names[][24]=
{"PLY_BIG_ENDIAN","PLY_LITTLE_ENDIAN","PLY_ASCII","PLY_DEFAULT"};

//! PLY file properties
struct plyProperty
{
	p_ply_property prop;
	const char* propName;
	e_ply_type type;
	e_ply_type length_type;
	e_ply_type value_type;
	int elemIndex;
};

//! PLY file nuclear element
struct plyElement
{
	p_ply_element elem;
	const char* elementName;
	long elementInstances;
	std::vector<plyProperty> properties;
	int propertiesCount;
	bool isFace;
};

//! Stanford PLY file I/O filter
class QCC_IO_LIB_API PlyFilter : public FileIOFilter
{
public:
	PlyFilter();
	
	//static accessors
	static void SetDefaultOutputFormat(e_ply_storage_mode format);

	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	
	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

	//! Custom loading method
	CC_FILE_ERROR loadFile(const QString& filename, const QString& textureFilename, ccHObject& container, LoadParameters& parameters);

private:
	//! Internal method
	CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, e_ply_storage_mode storageType);
};

#endif //CC_PLY_FILTER_HEADER
