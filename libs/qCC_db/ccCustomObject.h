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

#ifndef CC_CUSTOM_OBJECT_HEADER
#define CC_CUSTOM_OBJECT_HEADER

//Local
#include "ccHObject.h"

//! Custom hierarchy object
/** Used internally for deserialization of plugin-defined hierarchy objects
	(see CC_TYPES::CUSTOM_H_OBJECT).
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db.h"
class QCC_DB_LIB_API ccCustomHObject : public ccHObject
#else
class ccCustomHObject : public ccHObject
#endif
{
public:

    //! Default constructor
    /** \param name object name (optional)
    **/
	ccCustomHObject(QString name = QString())
        : ccHObject(name)

	{}

    virtual bool isSerializable() const { return true; }
    
	// inherited from ccObject
    virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::CUSTOM_H_OBJECT; }

protected:
    virtual bool toFile_MeOnly(QFile &out) const
    {
        ccHObject::toFile_MeOnly(out);
        return true;
    }
    virtual bool fromFile_MeOnly(QFile &in, short dataVersion, int flags)
    {
        ccHObject::fromFile_MeOnly(in, dataVersion, flags);
        return true;
    }
};

//! Custom leaf object
/** Used internally for deserialization of plugin-defined leaf objects
	(see CC_TYPES::CUSTOM_LEAF_OBJECT).
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db.h"
class QCC_DB_LIB_API ccCustomLeafObject : public ccCustomHObject
#else
class ccCustomLeafObject : public ccCustomHObject
#endif
{
public:

    //! Default constructor
    /** \param name object name (optional)
    **/
	ccCustomLeafObject(QString name = QString()) : ccCustomHObject(name) {}

	// inherited from ccObject
    virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::CUSTOM_LEAF_OBJECT; }
};

#endif //CC_CUSTOM_OBJECT_HEADER
