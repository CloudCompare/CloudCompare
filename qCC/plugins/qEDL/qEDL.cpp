//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qEDL                        #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//
#include <QtGui>

#include "qEDL.h"
#include "ccEDLFilter.h"

void qEDL::getDescription(ccPluginDescription& desc)
{
    strcpy(desc.name,"Eye-dome Lighting Filter Plugin");
    strcpy(desc.menuName,"E.D.L.");
    desc.hasAnIcon=true;
    desc.version=1;
}

QIcon qEDL::getIcon() const
{
    return QIcon(QString::fromUtf8(":/CC/plugin/qEDL/cc_edl.png"));
}

ccGlFilter* qEDL::getFilter()
{
    return new ccEDLFilter();
}

Q_EXPORT_PLUGIN2(qEDL,qEDL);
