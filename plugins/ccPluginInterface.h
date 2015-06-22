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

#ifndef CC_PLUGIN_INTERFACE_HEADER
#define CC_PLUGIN_INTERFACE_HEADER

//qCC_db
#include <ccObject.h> //for CC_QT5 def

//Qt
#include <QObject>
#include <QString>
#include <QIcon>

//Qt versop,
#include <qglobal.h>
#ifndef CC_QT5
	#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
		#define CC_QT5
	#endif
#endif

class ccExternalFactory;

//! Plugin type
enum  CC_PLUGIN_TYPE {	CC_STD_PLUGIN               = 0,
						CC_GL_FILTER_PLUGIN         = 1,
						CC_IO_FILTER_PLUGIN         = 2,
};

//! Standard CC plugin interface
/** Version 2.0
**/
class ccPluginInterface
{
public:

	//! Virtual destructor
	virtual ~ccPluginInterface() {}

	//! Returns plugin type (standard or OpenGL filter)
	virtual CC_PLUGIN_TYPE getType() const = 0;

	//! Returns (short) name (for menu entry, etc.)
	virtual QString getName() const = 0;

	//! Returns long name/description (for tooltip, etc.)
	virtual QString getDescription() const = 0;

	//! Returns icon
	/** Should be reimplemented if necessary
	**/
	virtual QIcon getIcon() const { return QIcon(); }

	//! Returns the plugin's custom object factory (if any)
	/** Plugins may provide a factory to build custom objects.
		This allows qCC_db to properly code and decode the custom
		objects stream in BIN files. Custom objects must inherit the
		ccCustomHObject or ccCustomLeafObject interfaces.
	**/
	virtual ccExternalFactory* getCustomObjectsFactory() const { return 0; }
};

Q_DECLARE_INTERFACE(ccPluginInterface,"edf.rd.CloudCompare.ccPluginInterface/2.0")

#endif //CC_PLUGIN_INTERFACE_HEADER
