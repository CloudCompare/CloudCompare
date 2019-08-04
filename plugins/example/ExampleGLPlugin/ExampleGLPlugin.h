#ifndef EXAMPLE_GL_PLUGIN_HEADER
#define EXAMPLE_GL_PLUGIN_HEADER

//##########################################################################
//#                                                                        #
//#             CLOUDCOMPARE PLUGIN: ExampleGLPlugin                       #
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

#include "ccGLPluginInterface.h"

/** Replace 'ExampleGLPlugin' by your own plugin class name throughout and then
	check 'ExampleGLPlugin.cpp' for more directions.
	
	Each plugin requires an info.json file to provide information about itself -
	the name, authors, maintainers, icon, etc..
	
	The one method you are required to implement is getFilter(). This method
	registers your GL filter with CloudCompare.
**/

//! Example GL Plugin
class ExampleGLPlugin : public QObject, public ccGLPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccGLPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.ExampleGL" FILE "info.json")
	
public:
	explicit ExampleGLPlugin( QObject *parent = nullptr );
	~ExampleGLPlugin() override = default;
	
	// inherited from ccGLFilterPluginInterface
	ccGlFilter *getFilter() override;
};

#endif
