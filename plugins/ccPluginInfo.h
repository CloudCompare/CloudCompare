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
//#          COPYRIGHT: Andy Maloney                                       #
//#                                                                        #
//##########################################################################

#ifndef CC_PLUGIN_INFO
#define CC_PLUGIN_INFO

//plugins
#include <ccGLFilterPluginInterface.h>
#include <ccIOFilterPluginInterface.h>
#include <ccStdPluginInterface.h>

//qCC_db
#include <ccExternalFactory.h>
#include <ccLog.h>

//Qt
#include <QCoreApplication>
#include <QDir>
#include <QPluginLoader>
#include <QStandardPaths>
#include <QString>

//system
#include <vector>
#include <set>

class QObject;

//! This struct is used to store plugin information including
//! a pointer to its interface and a pointer to its QObject
struct tPluginInfo
{
	tPluginInfo(ccPluginInterface* o, QObject* q) :
		interface( o )
	  , qObject( q )
	{}
	
	//! Pointer to the plugin interface
	ccPluginInterface* interface;
	
	//! Pointer to the QObject
	QObject* qObject;
};

//! Simply a list of \see tPluginInfo
typedef std::vector<tPluginInfo> tPluginInfoList;

class ccPlugins
{
public:
	static tPluginInfoList LoadPlugins()
	{
		tPluginInfoList	plugins;
		
		//"static" plugins
		const QObjectList	pluginInstances = QPluginLoader::staticInstances();
		
		for ( QObject* plugin : pluginInstances )
		{
			ccPluginInterface* ccPlugin = dynamic_cast<ccPluginInterface*>(plugin);
			
			if (ccPlugin == nullptr)
			{
				continue;
			}

			plugins.push_back(tPluginInfo(ccPlugin, plugin)); 
		}
		
		//"dynamic" plugins
		ccPlugins::Find( plugins );
		
		//now iterate over plugins and automatically register what we can
		for ( tPluginInfo &plugin : plugins )
		{
			if (!plugin.interface)
			{
				Q_ASSERT(false);
				continue;
			}
			
			switch (plugin.interface->getType())
			{
				case CC_STD_PLUGIN:
				{
					ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(plugin.interface);
					
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
					ccIOFilterPluginInterface* ioPlugin = static_cast<ccIOFilterPluginInterface*>(plugin.interface);
					FileIOFilter::Shared filter = ioPlugin->getFilter();
					if (filter)
					{
						FileIOFilter::Register(filter);
						ccLog::Print(QString("[Plugin][%1] New file extension(s) registered: %2").arg(ioPlugin->getName(), filter->getDefaultExtension().toUpper()));
					}
				}
					break;
					
				default:
					//nothing to do at this point
					break;
			}
		}
		
		return plugins;
	}
	
	static QStringList GetPluginPaths()
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
			pluginPaths << (appDataPath + "/plugins");
		}
		
		return pluginPaths;		
	}
	
	static void Find( tPluginInfoList& plugins )
	{
		const QStringList pluginPaths = GetPluginPaths();
		
		ccLog::Print( QStringLiteral( "[Plugin] Lookup paths: %1" ).arg( pluginPaths.join( ", " ) ) );
		
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
		
		// maps plugin name (from inside the plugin) to its path & pointer
		//	This allows us to create a unique list (overridden by path)
		std::set<QString> alreadyLoadedFiles;
		
		for (const QString &path : pluginPaths)
		{
			QDir pluginsDir( path );
			
			pluginsDir.setNameFilters( nameFilters );
			
			const QStringList	fileNames = pluginsDir.entryList();
			
			for (const QString &filename : fileNames)
			{
				const QString pluginPath = pluginsDir.absoluteFilePath(filename);
				
				//if we have already encountered this file, we can skip it
				if (alreadyLoadedFiles.find(pluginPath) != alreadyLoadedFiles.end())
				{
					continue;
				}
				
				QPluginLoader loader(pluginPath);
				QObject* plugin = loader.instance();
				
				if ( plugin == nullptr )
				{
					ccLog::Warning( QStringLiteral( "[Plugin] File '%1' doesn't seem to be a valid plugin\t(%2)" ).arg( filename, loader.errorString() ) );
					continue;
				}
				
				ccPluginInterface* ccPlugin = dynamic_cast<ccPluginInterface*>(plugin);
				
				if ( ccPlugin == nullptr )
				{
					delete plugin;
					ccLog::Warning( QStringLiteral( "[Plugin] File '%1' doesn't seem to be a valid plugin or it is not supported by this version" ).arg( filename ) );
					continue;
				}
				
				if ( ccPlugin->getName().isEmpty() )
				{
					ccLog::Error( QStringLiteral( "[Plugin] Plugin '%1' has a blank name" ).arg( filename ) );

					loader.unload();
					
					continue;
				}
				else
				{
					ccLog::Print( QStringLiteral( "[Plugin] Found: %1 (%2)" ).arg( ccPlugin->getName(), filename ) );
				}
				
				plugins.push_back( tPluginInfo(ccPlugin, plugin) );
			}
		}
	}
};

#endif //CC_PLUGIN_INFO
