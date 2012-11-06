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
#include <strings_utils.h>

#include <QFileInfo>

QString getChildName(const QString &filename)
{
    QFileInfo fi(filename);
    QString new_name = fi.baseName();
    return qPrintable(new_name);
}

QString getParentName(const QString &filename)
{
    QFileInfo fi(filename);
    QString new_name = QString("%1 (%2)").arg(fi.fileName()).arg(fi.absolutePath());
    return qPrintable(new_name);
}
