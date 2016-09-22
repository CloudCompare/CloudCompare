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
#include <ccStdPluginInterface.h>
#include <ccGLFilterPluginInterface.h>
#include <ccIOFilterPluginInterface.h>

//qCC_db
#include <ccLog.h>
#include <ccExternalFactory.h>

//Qt
#include <QString>
#include <QPluginLoader>
#include <QCoreApplication>
#include <QDir>
#ifdef Q_OS_MAC
#include <QStandardPaths>
#endif

//system
#include <vector>
#include <set>

class QObject;

//! This type is used to communicate information between the main window and the plugin dialog
//! It is a pair - first is path to the plugin, second is an object pointer to the plugin
struct tPluginInfo
{
	tPluginInfo(QString f, ccPluginInterface* o, QObject* q)
		: filename(f)
		, object(o)
		, qObject(q)
	{}

	//! Path to the plugin
	QString filename;
	//! Pointer to the plugin
	ccPluginInterface* object;
	//! Pointer to the QObject
	QObject* qObject;
};

//! Simply a list of \see tPluginInfo
typedef std::vector<tPluginInfo> tPluginInfoList;

class ccPlugins
{
public:

	static ccPluginInterface* ToValidPlugin(QObject* plugin, int filter = CC_STD_PLUGIN | CC_GL_FILTER_PLUGIN | CC_IO_FILTER_PLUGIN)
	{
		if (plugin)
		{
			//standard plugin?
			if (filter & CC_STD_PLUGIN)
			{
				ccStdPluginInterface* ccStdPlugin = qobject_cast<ccStdPluginInterface*>(plugin);
				if (ccStdPlugin)
					return static_cast<ccPluginInterface*>(ccStdPlugin);
			}

			//GL (shader) plugin
			if (filter & CC_GL_FILTER_PLUGIN)
			{
				ccGLFilterPluginInterface* ccGLPlugin = qobject_cast<ccGLFilterPluginInterface*>(plugin);
				if (ccGLPlugin)
					return static_cast<ccPluginInterface*>(ccGLPlugin);
			}

			//I/O filter plugin
			if (filter & CC_IO_FILTER_PLUGIN)
			{
				ccIOFilterPluginInterface* ccIOPlugin = qobject_cast<ccIOFilterPluginInterface*>(plugin);
				if (ccIOPlugin)
					return static_cast<ccPluginInterface*>(ccIOPlugin);
			}
		}

		return nullptr;
	}

	static void LoadPlugins(tPluginInfoList& plugins,
							const QStringList& pluginPaths,
							const QStringList& dirFilters,
							int filter = CC_STD_PLUGIN | CC_GL_FILTER_PLUGIN | CC_IO_FILTER_PLUGIN)
	{
		//"static" plugins
		for (QObject* plugin : QPluginLoader::staticInstances())
		{
			ccPluginInterface* ccPlugin = ToValidPlugin(plugin, filter);
			if (ccPlugin == nullptr)
			{
				continue;
			}
			//generate a fake filename
			plugins.push_back(tPluginInfo(QString(), ccPlugin, plugin)); //static plugins have no associated filename
		}

		//"dynamic" plugins
		ccPlugins::Find(plugins, pluginPaths, dirFilters);
	
		//now iterate over plugins and automatically register what we can
		for ( tPluginInfo &plugin : plugins )
		{
			if (!plugin.object)
			{
				assert(false);
				continue;
			}
			
			switch (plugin.object->getType())
			{
			case CC_STD_PLUGIN:
			{
				ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(plugin.object);

				//see if this plugin provides an additional factory for objects
				ccExternalFactory* factory = stdPlugin->getCustomObjectsFactory();
				if (factory)
				{
					//if it's valid, add it to the factories set
					assert(ccExternalFactory::Container::GetUniqueInstance());
					ccExternalFactory::Container::GetUniqueInstance()->addFactory(factory);
				}
			}
			break;

			case CC_IO_FILTER_PLUGIN: //I/O filter
			{
				ccIOFilterPluginInterface* ioPlugin = static_cast<ccIOFilterPluginInterface*>(plugin.object);
				FileIOFilter::Shared filter = ioPlugin->getFilter();
				if (filter)
				{
					FileIOFilter::Register(filter);
					ccLog::Print(QString("[Plugin][%1] New file extension(s) registered: %2").arg(ioPlugin->getName()).arg(filter->getDefaultExtension().toUpper()));
				}
			}
			break;

			default:
				//nothing to do at this point
				break;
			}
		}
	}

	static void Find(	tPluginInfoList& plugins,
						const QStringList& pluginPaths,
						const QStringList& dirFilters,
						int filter = CC_STD_PLUGIN | CC_GL_FILTER_PLUGIN | CC_IO_FILTER_PLUGIN)
	{
		ccLog::Print(QString("[Plugins] Plugin lookup dirs: %1").arg(pluginPaths.join(", ")));

		// maps plugin name (from inside the plugin) to its path & pointer
		//	This allows us to create a unique list (overridden by path)
		std::set<QString> alreadyLoadedFiles;

		for (const QString &path : pluginPaths)
		{
			QDir pluginsDir(path);
			pluginsDir.setNameFilters(dirFilters);

			for (const QString &filename : pluginsDir.entryList())
			{
				const QString pluginPath = pluginsDir.absoluteFilePath(filename);

				//if we have already encountered this file, we can skip it
				if (alreadyLoadedFiles.find(pluginPath) != alreadyLoadedFiles.end())
				{
					continue;
				}

				QPluginLoader loader(pluginPath);
				QObject* plugin = loader.instance();

				if (plugin == nullptr)
				{
					ccLog::Warning(QString("[Plugin] File '%1' doesn't seem to be a valid plugin\t(%2)").arg(filename).arg(loader.errorString()));
					continue;
				}

				ccPluginInterface* ccPlugin = ToValidPlugin(plugin, filter);
				if (ccPlugin == nullptr)
				{
					delete plugin;
					ccLog::Warning(QString("[Plugin] File '%1' doesn't seem to be a valid plugin or it is not supported by this version").arg(filename));
					continue;
				}

				ccLog::Print(QString("Found plugin: %1 (%2)").arg(ccPlugin->getName()).arg(filename));
				plugins.push_back(tPluginInfo(pluginPath, ccPlugin, plugin));
			}
		}
	}
};

#endif //CC_PLUGIN_INFO
