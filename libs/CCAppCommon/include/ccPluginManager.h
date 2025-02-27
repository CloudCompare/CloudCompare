#pragma once
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

#include "CCAppCommon.h"

#include <QObject>
#include <QVector>

class ccPluginInterface;

/**
 * \brief A list of plugin interfaces.
 * \details This type is an alias for a QVector of pointers to ccPluginInterface.
 * 
 * Provides a convenient way to manage and store multiple plugin interfaces 
 * within the CloudCompare application.
 */
using ccPluginInterfaceList = QVector<ccPluginInterface*>;

/**
 * \brief Manages plugin loading, configuration, and lifecycle in CloudCompare.
 * 
 * \details The ccPluginManager is responsible for:
 * - Loading plugins from specified paths
 * - Tracking enabled and disabled plugins
 * - Providing access to the list of loaded plugins
 * 
 * This class follows the Singleton pattern, ensuring only one instance 
 * of the plugin manager exists throughout the application.
 * 
 * \note Inherits from QObject to provide signal/slot mechanisms and 
 * object lifetime management.
 * 
 * \example
 * \code
 * // Get the singleton instance of the plugin manager
 * ccPluginManager& pluginMgr = ccPluginManager::Get();
 * 
 * // Set plugin search paths
 * QStringList paths = {"/path/to/plugins", "/another/plugin/dir"};
 * pluginMgr.setPaths(paths);
 * 
 * // Load plugins from the specified paths
 * pluginMgr.loadPlugins();
 * 
 * // Get the list of loaded plugins
 * ccPluginInterfaceList& plugins = pluginMgr.pluginList();
 * 
 * // Enable or disable a specific plugin
 * pluginMgr.setPluginEnabled(somePlugin, false);
 * \endcode
 */
class CCAPPCOMMON_LIB_API ccPluginManager : public QObject
{
	Q_OBJECT
	
public:
	/**
	 * \brief Destructor for the plugin manager
	 * 
	 * Cleans up resources associated with loaded plugins
	 */
	~ccPluginManager() override = default;
	
	/**
	 * \brief Get the singleton instance of the plugin manager
	 * 
	 * \return Reference to the single ccPluginManager instance
	 */
	static ccPluginManager& Get();
	
	/**
	 * \brief Set the paths to search for plugins
	 * 
	 * \param[in] paths List of directories to search for plugins
	 */
	void setPaths( const QStringList& paths );

	/**
	 * \brief Get the current plugin search paths
	 * 
	 * \return List of directories being searched for plugins
	 */
	QStringList pluginPaths() const;
	
	/**
	 * \brief Load plugins from the specified paths
	 * 
	 * Scans the plugin paths and loads all compatible plugins
	 */
	void loadPlugins();
	
	/**
	 * \brief Get the list of loaded plugins
	 * 
	 * \return Reference to the list of loaded plugin interfaces
	 */
	ccPluginInterfaceList& pluginList();
	
	/**
	 * \brief Enable or disable a specific plugin
	 * 
	 * \param[in] plugin Pointer to the plugin interface
	 * \param[in] enabled True to enable, false to disable
	 */
	void setPluginEnabled( const ccPluginInterface* plugin, bool enabled );

	/**
	 * \brief Check if a plugin is currently enabled
	 * 
	 * \param[in] plugin Pointer to the plugin interface
	 * \return True if the plugin is enabled, false otherwise
	 */
	bool isEnabled( const ccPluginInterface* plugin ) const;
	
protected:
	/**
	 * \brief Protected constructor to enforce Singleton pattern
	 * 
	 * \param[in] parent Optional parent QObject
	 */
	explicit ccPluginManager( QObject* parent = nullptr );

private: // methods
	/**
	 * \brief Internal method to load plugins from paths and add to the plugin list
	 */
	void loadFromPathsAndAddToList();
	
	/**
	 * \brief Retrieve the list of disabled plugin interface IDs
	 * 
	 * \param[out] disabledPlugins List to be populated with disabled plugin IDs
	 */
	void getDisabledPluginIIDs(QStringList& disabledPlugins) const;
	
private: // members
	/**
	 * \brief List of paths to search for plugins
	 */
	QStringList m_pluginPaths;

	/**
	 * \brief List of loaded plugin interfaces
	 */
	ccPluginInterfaceList m_pluginList;
};
