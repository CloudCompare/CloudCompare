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
#include <QIcon>
#include <QList>
#include <QObject>
#include <QPair>
#include <QString>

//Qt version
#include <qglobal.h>
#ifndef CC_QT5
	#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
		#define CC_QT5
	#endif
#endif

class ccExternalFactory;
class ccCommandLineInterface;

//! Plugin type
enum  CC_PLUGIN_TYPE {	CC_STD_PLUGIN               = 1,
						CC_GL_FILTER_PLUGIN         = 2,
						CC_IO_FILTER_PLUGIN         = 4,
};

//! Standard CC plugin interface
/** Version 3.1
**/
class ccPluginInterface
{
public:	
	// Contact represents a person and is used for authors and maintainer lists
	struct Contact {
		QString name;
		QString email;
	};
	
	typedef QList<Contact> ContactList;
	
	// Reference represents a journal article or online post about the plugin where
	// the user can find more information.
	struct Reference {
		QString article;
		QString	url;
	};
	
	typedef QList<Reference> ReferenceList;
	
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
	
	//! Returns a list of references (articles and websites) for the plugin
	//! This is optional.
	//! See qDummyPlugin for a real example.
	//! Added in v3.1 of the plugin interface.
	virtual ReferenceList getReferences() const { return ReferenceList{}; }
	
	//! Returns a list of the authors' names and email addresses
	//! This is optional.
	//! See qDummyPlugin for a real example.
	//! Added in v3.1 of the plugin interface.
	virtual ContactList getAuthors() const { return ContactList{}; }
	
	//! Returns a list of the maintainers' names and email addresses
	//! This is optional.
	//! See qDummyPlugin for a real example.
	//! Added in v3.1 of the plugin interface.
	virtual ContactList getMaintainers() const { return ContactList{}; }
	
	//! Starts the plugin
	/** Should be reimplemented if necessary.
		Used when 'starting' a plugin from the command line
		(to start a background service, a thread, etc.)
	**/
	virtual bool start() { return true; }

	//! Stops the plugin
	/** Should be reimplemented if necessary.
		Used to stop a plugin previously started (see ccPluginInterface::start).
	**/
	virtual void stop() { }

	//! Returns the plugin's custom object factory (if any)
	/** Plugins may provide a factory to build custom objects.
		This allows qCC_db to properly code and decode the custom
		objects stream in BIN files. Custom objects must inherit the
		ccCustomHObject or ccCustomLeafObject interfaces.
	**/
	virtual ccExternalFactory* getCustomObjectsFactory() const { return 0; }

	//! Optional: registers commands (for the command line mode)
	/** Does nothing by default.
		\warning: don't use keywords that are already used by the main application or other plugins!
			(use a unique prefix for all commands if possible)
	**/
	virtual void registerCommands(ccCommandLineInterface* cmd) {}
};

Q_DECLARE_METATYPE(const ccPluginInterface *);

Q_DECLARE_INTERFACE(ccPluginInterface, "edf.rd.CloudCompare.ccPluginInterface/3.1")

#endif //CC_PLUGIN_INTERFACE_HEADER
