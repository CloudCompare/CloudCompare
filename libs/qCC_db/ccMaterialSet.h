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

#ifndef CC_MATERIAL_SET_HEADER
#define CC_MATERIAL_SET_HEADER

//Local
#include "ccHObject.h"
#include "CCShareable.h"

class ccGenericGLDisplay;


//! Mesh (triangle) material
class QCC_DB_LIB_API ccMaterialSet : public std::vector<ccMaterial::CShared>, public CCShareable, public ccHObject
{
public:
	//! Default constructor
	ccMaterialSet(const QString& name = QString());

	//inherited from ccHObject
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::MATERIAL_SET; }
	bool isShareable() const override { return true; }

	//! Finds material by name
	/** \return material index or -1 if not found
	**/
	int findMaterialByName(const QString& mtlName);

	//! Finds material by unique identifier
	/** \return material index or -1 if not found
	**/
	int findMaterialByUniqueID(const QString& uniqueID);

	//! Adds a material
	/** Ensures unicity of material names.
		\param mat material
		\param allowDuplicateNames whether to allow duplicate names for materials or not (in which case the returned index is the one of the material with the same name)
		\return material index
	**/
	int addMaterial(ccMaterial::CShared mat, bool allowDuplicateNames = false);

	//! MTL (material) file parser
	/** Inspired from KIXOR.NET "objloader" (http://www.kixor.net/dev/objloader/)
	**/
	static bool ParseMTL(QString path, const QString& filename, ccMaterialSet& materials, QStringList& errors);

	//! Saves to an MTL file (+ associated texture images)
	bool saveAsMTL(const QString& path, const QString& baseFilename, QStringList& errors) const;

	//! Clones materials set
	ccMaterialSet* clone() const;

	//! Appends materials from another set
	bool append(const ccMaterialSet& source);

	//inherited from ccSerializableObject
	bool isSerializable() const override { return true; }

protected:
	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//! Default destructor (protected: use 'release' instead)
	~ccMaterialSet() override = default;
};

#endif //CC_MATERIAL_SET_HEADER