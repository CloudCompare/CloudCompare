//##########################################################################
//#                                                                        #
//#                 CLOUDCOMPARE PLUGIN: qSTEPCADImport                    #
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
//#                          COPYRIGHT: EDF R&D                            #
//#                                                                        #
//##########################################################################

#include "../include/qStepCADImport.h"
#include "../include/STEPFilter.h"

qStepCADImport::qStepCADImport(QObject *parent)
	: QObject(parent)
	, ccIOPluginInterface(":/CC/plugin/qStepCADImport/info.json")
{
}

void qStepCADImport::registerCommands( ccCommandLineInterface *inCmdLine )
{
	Q_UNUSED( inCmdLine );
}

ccIOPluginInterface::FilterList qStepCADImport::getFilters()
{
	return { FileIOFilter::Shared( new STEPFilter ) };
}
