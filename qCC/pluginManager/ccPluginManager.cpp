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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QPluginLoader>
#include <QSet>
#include <QStandardPaths>

#include "ccExternalFactory.h"
#include "ccGLFilterPluginInterface.h"
#include "ccIOFilterPluginInterface.h"
#include "ccLog.h"
#include "ccPluginManager.h"
#include "ccStdPluginInterface.h"


ccPluginInterfaceList ccPluginManager::m_pluginList;


// Check for metadata and warn if it's not there
// This indicates that a plugin hasn't been converted to the new JSON metadata.
// It is going to be required in a future verison of CloudCompare.
static void	sWarnIfNoMetaData( QPluginLoader *loader, const QString &fileName )
{
	QJsonObject	metaObject = loader->metaData();
	
	if ( metaObject.isEmpty() || metaObject["MetaData"].toObject().isEmpty() )
	{
		ccLog::Warning( QStringLiteral( "\t%1 does not supply meta data in the Q_PLUGIN_METADATA (see one of the example plugins). This will be required in a future verison of CloudCompare." ).arg( fileName ) );				
	}
	else
	{
		QJsonObject	data = metaObject["MetaData"].toObject();
		
		// The plugin type is going to be required
		const QStringList validTypes{ "GL", "I/O", "Standard" };
		
		const QString	pluginType = data["type"].toString();
		
		if ( !validTypes.contains( pluginType ) )
		{
			ccLog::Warning( QStringLiteral( "\t%1 does not supply a valid plugin type in its info.json. It must be one of: %2" ).arg( fileName, validTypes.join( ", " ) ) );
		}
	}
}	


ccPluginManager::ccPluginManager( QObject *parent ) :
    QObject( parent )
{
}

ccPluginManager::~ccPluginManager()
{
}

void ccPluginManager::loadPlugins()
{
	m_pluginList.clear();

	// "static" plugins
	const QObjectList	pluginInstances = QPluginLoader::staticInstances();

	for ( QObject* plugin : pluginInstances )
	{
		ccPluginInterface* ccPlugin = dynamic_cast<ccPluginInterface*>(plugin);

		if (ccPlugin == nullptr)
		{
			continue;
		}

		ccLog::Print(QStringLiteral("[Plugin] Found: %1 (STATIC)").arg(ccPlugin->getName()));
		m_pluginList.push_back(ccPlugin);
	}

	// "dynamic" plugins
	loadFromPathsAndAddToList();

	// now iterate over plugins and automatically register what we can
	for ( ccPluginInterface *plugin : m_pluginList )
	{
		if ( plugin == nullptr )
		{
			Q_ASSERT(false);
			continue;
		}

		switch ( plugin->getType() )
		{
			case CC_STD_PLUGIN:
			{
				ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(plugin);

				//see if this plugin provides an additional factory for objects
				ccExternalFactory* factory = stdPlugin->getCustomObjectsFactory();
				if (factory)
				{
					//if it's valid, add it to the factories set
					Q_ASSERT(ccExternalFactory::Container::GetUniqueInstance());
					ccExternalFactory::Container::GetUniqueInstance()->addFactory(factory);
				}
			}
				break;

			case CC_IO_FILTER_PLUGIN: //I/O filter
			{
				ccIOFilterPluginInterface* ioPlugin = static_cast<ccIOFilterPluginInterface*>(plugin);
				
				for ( FileIOFilter::Shared filter : ioPlugin->getFilters() )
				{
					if (filter)
					{
						FileIOFilter::Register(filter);
						ccLog::Print(QString("[Plugin][%1] New file extension(s) registered: %2").arg(ioPlugin->getName(), filter->getDefaultExtension().toUpper()));
					}
				}
			}
				break;

			default:
				//nothing to do at this point
				break;
		}
	}
}

QStringList ccPluginManager::pluginPaths()
{
	QString appPath = QCoreApplication::applicationDirPath();

	QStringList pluginPaths;

#if defined(Q_OS_MAC)
	// plugins are in the bundle
	QDir  dir( appPath );

	if ( dir.dirName() == "MacOS" )
	{
		dir.cdUp();

		pluginPaths << (dir.absolutePath() + "/PlugIns/ccPlugins");
#if defined(CC_MAC_DEV_PATHS)
		// used for development only - this is the path where the plugins are built
		// this avoids having to install into the application bundle when developing
		dir.cdUp();
		dir.cdUp();
		dir.cdUp();
		pluginPaths << (dir.absolutePath() + "/ccPlugins");
#endif
	}
#elif defined(Q_OS_WIN)
	pluginPaths << (appPath + "/plugins");
#elif defined(Q_OS_LINUX)
	// Plugins are relative to the bin directory where the executable is found
	QDir  binDir(appPath);

	if (binDir.dirName() == "bin")
	{
		binDir.cdUp();

		pluginPaths << (binDir.absolutePath() + "/lib/cloudcompare/plugins");
	}
	else
	{
		// Choose a reasonable default to look in
		pluginPaths << "/usr/lib/cloudcompare/plugins";
	}
#else
#error Need to specify the plugin path for this OS.
#endif

	// Add any app data paths
	// Plugins in these directories take precendence over the included ones
	// This allows users to put plugins outside of the install directories.
	QStringList appDataPaths = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);

	for ( const QString &appDataPath : appDataPaths )
	{
		QString path = appDataPath + "/plugins";
		if (!pluginPaths.contains(path)) //avoid duplicate entries (can happen, at least on Windows)
		{
			pluginPaths << path;
		}
	}

	return pluginPaths;
}

ccPluginInterfaceList &ccPluginManager::pluginList()
{
#ifdef QT_DEBUG
	if ( m_pluginList.empty() )
	{
		qWarning() << "Plugin list is empty - did you call loadPlugins()?";
	}
#endif
	
	return m_pluginList;
}

void ccPluginManager::loadFromPathsAndAddToList()
{
	const QStringList nameFilters{
#if defined(Q_OS_MAC)
		"*.dylib"
#elif defined(Q_OS_WIN)
		"*.dll"
#elif defined(Q_OS_LINUX)
		"*.so"
#else
#error Need to specify the dynamic library extension for this OS.
#endif
	};
	
	// Map the plugin file name to its loader so we can unload it if necessary.
	//	This lets us override plugins by path.
	QMap<QString, QPluginLoader *> fileNameToLoaderMap;
	
	for ( const QString &path : pluginPaths() )
	{
		ccLog::Print( tr( "[Plugin] Searching: %1" ).arg( path ) );
		
		QDir pluginsDir( path );

		pluginsDir.setNameFilters( nameFilters );

		const QStringList	fileNames = pluginsDir.entryList();

		for ( const QString &fileName : fileNames )
		{
			const QString pluginPath = pluginsDir.absoluteFilePath( fileName );
			
			QPluginLoader *loader = new QPluginLoader( pluginPath );
			
			sWarnIfNoMetaData( loader, fileName );
			
			QObject *plugin = loader->instance();

			if ( plugin == nullptr )
			{
				ccLog::Warning( tr( "\t%1 does not seem to be a valid plugin\t(%2)" ).arg( fileName, loader->errorString() ) );
				
				delete loader;
				
				continue;
			}
			
			ccPluginInterface *ccPlugin = dynamic_cast<ccPluginInterface*>(plugin);

			if ( ccPlugin == nullptr )
			{				
				ccLog::Warning( tr( "\t%1 does not seem to be a valid plugin or it is not supported by this version" ).arg( fileName ) );

				loader->unload();
				
				delete loader;
				
				continue;
			}

			if ( ccPlugin->getName().isEmpty() )
			{
				ccLog::Error( tr( "\tPlugin %1 has a blank name" ).arg( fileName ) );

				loader->unload();
				
				delete loader;

				continue;
			}
			
			QPluginLoader *previousLoader = fileNameToLoaderMap.value( fileName );
			
			// If we have already loaded a plugin with this file name, unload it and replace the interface in the plugin list
			if ( previousLoader != nullptr )
			{
				ccPluginInterface *pluginInterface = dynamic_cast<ccPluginInterface *>( previousLoader->instance() );
				
				// maintain the order of the plugin list
				const int index = m_pluginList.indexOf( pluginInterface );
				m_pluginList.replace( index, ccPlugin );
				
				previousLoader->unload();
				
				delete previousLoader;
								
				ccLog::Warning( tr( "\t%1 overridden" ).arg( fileName ) );
			}
			else
			{
				m_pluginList.push_back( ccPlugin );
			}

			fileNameToLoaderMap[fileName] = loader;
			
			ccLog::Print( tr( "\tPlugin found: %1 (%2)" ).arg( ccPlugin->getName(), fileName ) );
		}
	}
	
	for ( QPluginLoader *loader : fileNameToLoaderMap.values() )
	{
		delete loader;
	}
}
