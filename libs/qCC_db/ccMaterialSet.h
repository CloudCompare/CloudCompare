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

#ifndef CC_MATERIAL_SET_HEADER
#define CC_MATERIAL_SET_HEADER

//CCLib
#include <CCShareable.h>

//Local
#include "qCC_db.h"
#include "ccMaterial.h"
#include "ccHObject.h"

class ccGenericGLDisplay;

//! Mesh (triangle) material
class QCC_DB_LIB_API ccMaterialSet : public std::vector<ccMaterial>, public CCShareable, public ccHObject
{
public:

	//! Default constructor
	ccMaterialSet(QString name = QString());

	//inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::MATERIAL_SET; }
	virtual bool isShareable() const { return true; }

	//! Finds material by name
	/** \return material index or -1 if not found
	**/
	int findMaterial(QString mtlName);

	//! Adds a material
	/** Ensures unicity of material names.
	**/
	bool addMaterial(const ccMaterial& mat);

	//! MTL (material) file parser
	/** Inspired from KIXOR.NET "objloader" (http://www.kixor.net/dev/objloader/)
	**/
	static bool ParseMTL(QString path, const QString& filename, ccMaterialSet& materials, QStringList& errors);

	//! Saves to an MTL file (+ associated texture images)
	bool saveAsMTL(QString path, const QString& baseFilename, QStringList& errors) const;

	//! Associates to a given context
	void associateTo(ccGenericGLDisplay* display);

	//! Returns associated display
	const ccGenericGLDisplay* getAssociatedDisplay();

	//! Clones materials set
	ccMaterialSet* clone() const;

	//! Appends materials from another set
	bool append(const ccMaterialSet& source);

	//inherited from ccSerializableObject
	virtual bool isSerializable() const { return true; }

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);

	//! Default destructor (protected: use 'release' instead)
	virtual ~ccMaterialSet();

	//! Associated display
	ccGenericGLDisplay* m_display;

};

#endif //CC_MATERIAL_SET_HEADER