#ifndef CCPLUGINMANAGER_H
#define CCPLUGINMANAGER_H

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

#include <QObject>
#include <QVector>

class ccPluginInterface;

//! Simply a list of \see ccPluginInterface
using ccPluginInterfaceList = QVector<ccPluginInterface *>;


class ccPluginManager : public QObject
{
	Q_OBJECT
	
public:
	explicit ccPluginManager( QObject *parent = nullptr );
	~ccPluginManager() override = default;
	
	static void setPaths( const QStringList &paths );
	static QStringList pluginPaths();
	
	static void loadPlugins();
	
	static ccPluginInterfaceList &pluginList();
	
private:	
	static void loadFromPathsAndAddToList();	
	
	static QStringList m_PluginPaths;
	static ccPluginInterfaceList m_pluginList;
};

#endif