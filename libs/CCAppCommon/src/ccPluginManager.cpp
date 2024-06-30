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

#include "ccApplicationBase.h"
#include "ccPluginManager.h"

// ccPluginAPI
#include <ccPersistentSettings.h>

//qCC_db
#include <ccExternalFactory.h>
#include <ccLog.h>

//plugins
#include "ccGLPluginInterface.h"
#include "ccIOPluginInterface.h"
#include "ccStdPluginInterface.h"

//Qt
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QPluginLoader>
#include <QSet>
#include <QSettings>
#include <QStandardPaths>

namespace
{
	// This is used to avoid having to make the ccPluginManager constructor public
	class PrivatePluginManager : public ccPluginManager {};
}

Q_GLOBAL_STATIC(PrivatePluginManager, s_pluginManager);

// Get the plugin's IID from the meta data
static QString GetPluginIID(QPluginLoader* loader)
{
	const QJsonObject metaObject = loader->metaData();

	if (metaObject.isEmpty())
	{
		const QString fileName = QFileInfo(loader->fileName()).fileName();

		ccLog::Warning(QStringLiteral("\t%1 does not supply meta data in the Q_PLUGIN_METADATA (see one of the example plugins).").arg(fileName));

		return {};
	}

	return metaObject["IID"].toString();
}

// Check for metadata and error if it's not there
// This indicates that a plugin hasn't been converted to the new JSON metadata.
static bool	IsMetaDataValid(QPluginLoader* loader)
{
	const QJsonObject metaObject = loader->metaData();

	const QString fileName = QFileInfo(loader->fileName()).fileName();

	if (metaObject.isEmpty() || metaObject["MetaData"].toObject().isEmpty())
	{
		ccLog::Error(QStringLiteral("%1 does not supply meta data in the Q_PLUGIN_METADATA (see one of the example plugins).").arg(fileName));

		return false;
	}
	else
	{
		const QJsonObject	data = metaObject["MetaData"].toObject();

		// The plugin type is going to be required
		const QStringList validTypes{ "GL", "I/O", "Standard" };

		const QString pluginType = data["type"].toString();

		if (!validTypes.contains(pluginType))
		{
			ccLog::Error(QStringLiteral("%1 does not supply a valid plugin type in its info.json.\n\nFound: %2\n\nIt must be one of: %3")
				.arg(fileName, pluginType, validTypes.join(", ")));

			return false;
		}
	}

	return true;
}


ccPluginManager::ccPluginManager(QObject* parent)
	: QObject(parent)
{
}

ccPluginManager& ccPluginManager::Get()
{
	return *s_pluginManager;
}

void ccPluginManager::setPaths(const QStringList& paths)
{
	m_pluginPaths = paths;
}

QStringList ccPluginManager::pluginPaths() const
{
	return m_pluginPaths;
}

void ccPluginManager::loadPlugins()
{
	m_pluginList.clear();

	if (m_pluginPaths.empty())
	{
		qWarning() << "There are no plugin paths set. Maybe missing a call to ccPluginManager::setPaths()?";
	}

	// "static" plugins
	const QObjectList pluginInstances = QPluginLoader::staticInstances();

	for (QObject* plugin : pluginInstances)
	{
		ccPluginInterface* ccPlugin = qobject_cast<ccPluginInterface*>(plugin);

		if (ccPlugin == nullptr)
		{
			continue;
		}

		ccLog::Print(tr("[Plugin] Found: %1 (STATIC)").arg(ccPlugin->getName()));

		m_pluginList.push_back(ccPlugin);
	}

	// "dynamic" plugins
	loadFromPathsAndAddToList();

	// now iterate over plugins and automatically register what we can
	const auto pluginList = m_pluginList;

	QStringList disabledList;
	getDisabledPluginIIDs(disabledList);

	for (ccPluginInterface* plugin : pluginList)
	{
		if (plugin == nullptr)
		{
			Q_ASSERT(false);
			continue;
		}

		// Disable if we are not running on the command line and it's in the disabled list
		if (!ccApp->isCommandLine() && disabledList.contains(plugin->IID()))
		{
			ccLog::Print(tr("[Plugin][%1] Disabled").arg(plugin->getName()));

			continue;
		}

		switch (plugin->getType())
		{
		case CC_STD_PLUGIN:
		{
			ccStdPluginInterface* stdPlugin = static_cast<ccStdPluginInterface*>(plugin);

			//see if this plugin provides an additional factory for objects
			ccExternalFactory* factory = stdPlugin->getCustomObjectsFactory();

			if (factory != nullptr)
			{
				//if it's valid, add it to the factories set
				Q_ASSERT(ccExternalFactory::Container::GetUniqueInstance());
				ccExternalFactory::Container::GetUniqueInstance()->addFactory(factory);
			}

			break;
		}

		case CC_IO_FILTER_PLUGIN: //I/O filter
		{
			ccIOPluginInterface* ioPlugin = static_cast<ccIOPluginInterface*>(plugin);

			QStringList	ioExtensions;

			for (auto &filter : ioPlugin->getFilters())
			{
				if (filter)
				{
					FileIOFilter::Register(filter);

					ioExtensions += filter->getDefaultExtension().toUpper();
				}
			}

			if (!ioExtensions.empty())
			{
				ioExtensions.sort();

				ccLog::Print(tr("[Plugin][%1] New file extensions registered: %2")
					.arg(ioPlugin->getName(), ioExtensions.join(' ')));
			}

			break;
		}

		default:
			//nothing to do at this point
			break;
		}
	}
}

ccPluginInterfaceList &ccPluginManager::pluginList()
{
#ifdef QT_DEBUG
	if (m_pluginList.empty())
	{
		qWarning() << "Plugin list is empty - did you call loadPlugins()?";
	}
#endif

	return m_pluginList;
}

void ccPluginManager::getDisabledPluginIIDs(QStringList& disabledPlugins) const
{
	QSettings settings;
	settings.beginGroup(ccPS::Plugins());
	{
		disabledPlugins = settings.value("Disabled").toStringList();
	}
	settings.endGroup();
}

void ccPluginManager::setPluginEnabled(const ccPluginInterface* plugin, bool enabled)
{
	QStringList disabledList;
	getDisabledPluginIIDs(disabledList);

	const QString &iid = plugin->IID();

	if (enabled)
	{
		disabledList.removeAll(iid);
	}
	else
	{
		if (!disabledList.contains(iid))
		{
			disabledList.append(iid);
		}
	}

	QSettings settings;
	settings.beginGroup(ccPS::Plugins());
	{
		settings.setValue("Disabled", disabledList);
	}
	settings.endGroup();
}

bool ccPluginManager::isEnabled(const ccPluginInterface *plugin) const
{
	QStringList disabledList;
	getDisabledPluginIIDs(disabledList);

	return !disabledList.contains(plugin->IID());
}

void ccPluginManager::loadFromPathsAndAddToList()
{
	const QStringList nameFilters{
#if defined(Q_OS_MAC)
		"*.dylib"
#elif defined(Q_OS_WIN)
		"*.dll"
#elif defined(Q_OS_UNIX)
		"*.so"
#else
#error Need to specify the dynamic library extension for this OS.
#endif
	};

	// Map the plugin's IID to its loader so we can unload it if necessary.
	// This lets us override plugins by path.
	QMap<QString, QSharedPointer<QPluginLoader> > pluginIIDToLoaderMap;

	const auto paths = pluginPaths();

	for (const QString& path : paths)
	{
		ccLog::Print(tr("[Plugin] Searching: %1").arg(path));

		QDir pluginsDir(path);

		pluginsDir.setNameFilters(nameFilters);

		const QStringList fileNames = pluginsDir.entryList();

		for (const QString& fileName : fileNames)
		{
			const QString pluginPath = pluginsDir.absoluteFilePath(fileName);

			QSharedPointer<QPluginLoader> loader(new QPluginLoader(pluginPath));

			const QString pluginIID = GetPluginIID(loader.data());

			bool metaDataValid = IsMetaDataValid(loader.data());

			if (!metaDataValid || pluginIID.isNull())
			{
				ccLog::Warning(tr("\t%1 has invalid meta data\t").arg(fileName));
				continue;
			}

			QObject* plugin = loader->instance();
			ccPluginInterface* ccPlugin = qobject_cast<ccPluginInterface*>(plugin);

			if ((plugin == nullptr) || (ccPlugin == nullptr))
			{
				if (plugin == nullptr)
				{
					ccLog::Warning(tr("\t%1 does not seem to be a valid plugin\t(%2)").arg(fileName, loader->errorString()));
				}
				else
				{
					ccLog::Warning(tr("\t%1 does not seem to be a valid plugin or it is not supported by this version").arg(fileName));
				}

				loader->unload();
				continue;
			}

			if (ccPlugin->getName().isEmpty())
			{
				ccLog::Error(tr("Plugin %1 has no name").arg(fileName));

				loader->unload();
				continue;
			}

			ccPlugin->setIID(pluginIID);

			// If we have already loaded a plugin with this IID, unload it and replace the interface in the plugin list
			if (pluginIIDToLoaderMap.contains(pluginIID))
			{
				QSharedPointer<QPluginLoader> previousLoader = pluginIIDToLoaderMap[pluginIID];
				assert(previousLoader);

				ccPluginInterface* pluginInterface = qobject_cast<ccPluginInterface *>(previousLoader->instance());

				// maintain the order of the plugin list
				const int index = m_pluginList.indexOf(pluginInterface);
				if (index >= 0)
				{
					m_pluginList.replace(index, ccPlugin);
				}
				else
				{
					assert(false);
				}

				previousLoader->unload();

				ccLog::Warning(tr("\t%1 overridden").arg(fileName));
			}
			else
			{
				m_pluginList.push_back(ccPlugin);
			}

			pluginIIDToLoaderMap[pluginIID] = loader;

			ccLog::Print(tr("\tPlugin found: %1 (%2)").arg(ccPlugin->getName(), fileName));
		}
	}

	pluginIIDToLoaderMap.clear();
}
