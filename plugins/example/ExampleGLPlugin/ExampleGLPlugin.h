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

#include "ccGLFilterPluginInterface.h"

//! Example GL Plugin
class ExampleGLPlugin : public QObject, public ccGLFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccGLFilterPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.ExampleGL" FILE "info.json")

public:
	explicit ExampleGLPlugin( QObject *parent = nullptr );
	virtual ~ExampleGLPlugin() = default;

	// inherited from ccGLFilterPluginInterface
	virtual ccGlFilter *getFilter() override;
};

#endif
