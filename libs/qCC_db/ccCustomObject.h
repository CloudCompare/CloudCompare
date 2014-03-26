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
		, m_deserialized(false)
	{}
    
	// inherited from ccObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::CUSTOM_LEAF_OBJECT; }

	//! Returns whether this instance has been created by the user or if it's a plain ccCustomHObject instance created by deserialization
	/** As custom objects can only be serialized as plain ccHObject instances (i.e. mainly with
		meta-data and potentially more standard children) the user should check before using
		a custom object instance if it's a real one or if it's an empty shell (i.e. fake instance
		that has been deserialized). Note that if the custom type only relies on meta-data, then
		it's safe to use this instance.
	**/
	bool isDeserialized() const { return !m_deserialized; }

protected:

	//! Overloaded from ccHObject::fromFile
	virtual bool fromFile(QFile& in, short dataVersion, int flags)
	{
		bool result = ccHObject::fromFile(in,dataVersion,flags);
		m_deserialized = true;

		return result;
	}

	//! Whether the instance has been deserialized or not (see ccCustomHObject::isDeserialized)
	bool m_deserialized;
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
