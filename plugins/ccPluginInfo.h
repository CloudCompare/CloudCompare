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

//Qt
#include <QPair>
#include <QVector>
#include <QString>
#include <QPluginLoader>
#include <QCoreApplication>
#include <QDir>
#ifdef Q_OS_MAC
#include <QStandardPaths>
#endif

class QObject;

//! This type is used to communicate information between the main window and the plugin dialog
//! It is a pair - first is path to the plugin, second is an object pointer to the plugin
typedef QPair<QString, QObject*>    tPluginInfo;

//! Simply a list of \see tPluginInfo
typedef QVector<tPluginInfo>        tPluginInfoList;

class ccPlugins
{
public:

	static ccPluginInterface* GetValidPlugin(QObject* plugin, int filter = CC_STD_PLUGIN | CC_GL_FILTER_PLUGIN | CC_IO_FILTER_PLUGIN)
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

		return 0;
	}

	static const tPluginInfoList Find(QStringList& pluginPaths, int filter = CC_STD_PLUGIN | CC_GL_FILTER_PLUGIN | CC_IO_FILTER_PLUGIN)
	{
		pluginPaths.clear();

		QStringList	dirFilters;
		QString		appPath = QCoreApplication::applicationDirPath();

#if defined(Q_OS_MAC)
		dirFilters << "*.dylib";

		// plugins are in the bundle
		appPath.remove("MacOS");

		pluginPaths += (appPath + "Plugins/ccPlugins");
#if defined(CC_MAC_DEV_PATHS)
		// used for development only - this is the path where the plugins are built
		// this avoids having to install into the application bundle when developing
		pluginPaths += (appPath + "../../../ccPlugins");
#endif
#elif defined(Q_OS_WIN)
		dirFilters << "*.dll";

		//plugins are in bin/plugins
		pluginPaths << (appPath + "/plugins");
#elif defined(Q_OS_LINUX)
		dirFilters << "*.so";

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
		#warning Need to specify the plugin path for this OS.
#endif

#ifdef Q_OS_MAC
		// Add any app data paths
		// Plugins in these directories take precendence over the included ones
		QStringList	appDataPaths = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);

		for (const QString &appDataPath : appDataPaths)
		{
			pluginPaths << (appDataPath + "/plugins");
		}
#endif

		ccLog::Print(QString("Plugin lookup dirs: %1").arg(pluginPaths.join(", ")));

		// maps plugin name (from inside the plugin) to its path & pointer
		//	This allows us to create a unique list (overridden by path)
		QMap<QString, tPluginInfo> pluginMap;

		for (const QString &path : pluginPaths)
		{
			QDir pluginsDir(path);
			pluginsDir.setNameFilters(dirFilters);

			for (const QString &filename : pluginsDir.entryList())
			{
				const QString pluginPath = pluginsDir.absoluteFilePath(filename);
				QPluginLoader loader(pluginPath);

				QObject	*plugin = loader.instance();

				if (plugin == nullptr)
				{
					ccLog::Warning(QString("[Plugin] %1").arg(loader.errorString()));
					continue;
				}

				ccPluginInterface* ccPlugin = GetValidPlugin(plugin, filter);

				if (ccPlugin == nullptr)
				{
					continue;
				}

				QString name = ccPlugin->getName();

				pluginMap.insert(name, tPluginInfo(pluginPath, plugin));
			}
		}

		return pluginMap.values().toVector();
	}

};

#endif //CC_PLUGIN_INFO
