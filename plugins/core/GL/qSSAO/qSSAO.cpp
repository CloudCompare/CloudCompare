//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qSSAO                       #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccSSAOFilter.h"

//Qt
#include <QtGui>

#include "qSSAO.h"

qSSAO::qSSAO(QObject *parent)
	: QObject(parent)
	, ccGLPluginInterface(":/CC/plugin/qSSAO/info.json")
{
}

ccGlFilter* qSSAO::getFilter()
{
	return new ccSSAOFilter();
}
