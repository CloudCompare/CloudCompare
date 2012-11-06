//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_STRINGS_UTILS_H
#define Q_PCL_PLUGIN_STRINGS_UTILS_H


#include <QString>

/** \brief helper function returning a name for the children cloud
 * \param[in] filename input string
 * \todo move this function to another place
 */
QString getChildName(const QString &filename);

/** \brief for a parent name to be used in the nested strucutre in qCC (for cloud container)
 * \param[in] filename input string
 * \todo move this function to another place
 */
QString getParentName(const QString &filename);

#endif // STRINGS_UTILS_H
