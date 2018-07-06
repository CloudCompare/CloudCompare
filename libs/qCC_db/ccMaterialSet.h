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
class ccMaterialSet : public std::vector<ccMaterial::CShared>, public CCShareable, public ccHObject
{
public:

	//! Default constructor
	QCC_DB_LIB_API ccMaterialSet(QString name = QString());

	//inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::MATERIAL_SET; }
	virtual bool isShareable() const override { return true; }

	//! Finds material by name
	/** \return material index or -1 if not found
	**/
	QCC_DB_LIB_API int findMaterialByName(QString mtlName);

	//! Finds material by unique identifier
	/** \return material index or -1 if not found
	**/
	QCC_DB_LIB_API int findMaterialByUniqueID(QString uniqueID);

	//! Adds a material
	/** Ensures unicity of material names.
		\param mat material
		\param allowDuplicateNames whether to allow duplicate names for materials or not (in which case the returned index is the one of the material with the same name)
		\return material index
	**/
	QCC_DB_LIB_API int addMaterial(ccMaterial::CShared mat, bool allowDuplicateNames = false);

	//! MTL (material) file parser
	/** Inspired from KIXOR.NET "objloader" (http://www.kixor.net/dev/objloader/)
	**/
	QCC_DB_LIB_API static bool ParseMTL(QString path, const QString& filename, ccMaterialSet& materials, QStringList& errors);

	//! Saves to an MTL file (+ associated texture images)
	QCC_DB_LIB_API bool saveAsMTL(QString path, const QString& baseFilename, QStringList& errors) const;

	//! Clones materials set
	QCC_DB_LIB_API ccMaterialSet* clone() const;

	//! Appends materials from another set
	QCC_DB_LIB_API bool append(const ccMaterialSet& source);

	//inherited from ccSerializableObject
	virtual bool isSerializable() const override { return true; }

protected:

	//inherited from ccHObject
	QCC_DB_LIB_API virtual bool toFile_MeOnly(QFile& out) const override;
	QCC_DB_LIB_API virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;

	//! Default destructor (protected: use 'release' instead)
	QCC_DB_LIB_API virtual ~ccMaterialSet();
};

#endif //CC_MATERIAL_SET_HEADER