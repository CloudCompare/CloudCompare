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

//! Simply a list of \see ccPluginInterface
using ccPluginInterfaceList = QVector<ccPluginInterface*>;

//! Plugin manager
class CCAPPCOMMON_LIB_API ccPluginManager : public QObject
{
	Q_OBJECT
	
public:
	~ccPluginManager() override = default;
	
	static ccPluginManager& Get();
	
	void setPaths( const QStringList& paths );
	QStringList pluginPaths() const;
	
	void loadPlugins();
	
	ccPluginInterfaceList& pluginList();
	
	void setPluginEnabled( const ccPluginInterface* plugin, bool enabled );
	bool isEnabled( const ccPluginInterface* plugin ) const;
	
protected:
	explicit ccPluginManager( QObject* parent = nullptr );

private: // methods
	void loadFromPathsAndAddToList();
	
	void getDisabledPluginIIDs(QStringList& disabledPlugins) const;
	
private: // members
	QStringList m_pluginPaths;
	ccPluginInterfaceList m_pluginList;
};
