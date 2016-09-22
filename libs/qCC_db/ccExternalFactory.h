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

#ifndef CC_EXTERNAL_FACTORY_HEADER
#define CC_EXTERNAL_FACTORY_HEADER

//Local
#include "ccHObject.h"

//Qt
#include <QMap>

/**  Provides new objects with an external factory
  *  This is intendend to be used into plugins.
  *  Each plugin may define a new factory by subclassing this class.
  *  Factories are then stored in a unique container and used to load custom types.
 **/
class QCC_DB_LIB_API ccExternalFactory
{
public:

	//! A convenience holder for all factories
	class QCC_DB_LIB_API Container
	{
	public:
		//! Default constructor
		Container() {}

		//! Returns factory using its (unique) name as key
		/** \param factoryName unique name
			\return corresponding factory (or null pointer if not found)
		**/
		ccExternalFactory* getFactoryByName(const QString& factoryName) const;

		//! Adds a new factory to the container
		/** Any previously existing factory with the same name will be overwritten.
		**/
		void addFactory(ccExternalFactory* factory);

		//! Shared pointer type
		typedef QSharedPointer<Container> Shared;

		//! Returns the unqiue static instance of the external factories container
		static Container::Shared GetUniqueInstance();

		//! Sets the unqiue static instance of the external factories container
		/** A default static instance is provided for convenience but another user defined instance
			can be declared here instead.
		**/
		static void SetUniqueInstance(Container::Shared container);

	protected:

		//! Set of factories
		QMap< QString, ccExternalFactory* > m_factories;
	};

	//! Default constructor
	/** \param factoryName unique name
	**/
	ccExternalFactory(QString factoryName);

	//! Returns the (unique) name of the factory
	inline QString getFactoryName() const {return m_factoryName; }

	//! Custom object building method
	/** Similar to ccHObject::New but virtual so as to be reimplemented by the plugin.
		\param metaName custom object name
		\return corresponding instance (or 0 if an error occurred)
	**/
	virtual ccHObject* buildObject(const QString& metaName) = 0;

protected:

	//! Name
	QString m_factoryName;

};

#endif // CC_EXTERNAL_FACTORY_HEADER
