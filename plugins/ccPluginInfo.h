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

#include <QPair>
#include <QVector>

class QObject;
class QString;

//! This type is used to communicate information between the main window and the plugin dialog
//! It is a pair - first is path to the plugin, second is an object pointer to the plugin
typedef QPair<QString, QObject*>    tPluginInfo;

//! Simply a list of \see tPluginInfo
typedef QVector<tPluginInfo>        tPluginInfoList;

#endif //CC_PLUGIN_INFO
