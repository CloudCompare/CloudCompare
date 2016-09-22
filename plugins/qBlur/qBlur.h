//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBlur                       #
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

#ifndef Q_BLUR_PLUGIN_HEADER
#define Q_BLUR_PLUGIN_HEADER

#include "../ccGLFilterPluginInterface.h"

//! Blur shader
class qBlur : public QObject, public ccGLFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccGLFilterPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qBlur")

public:

	//inherited from ccPluginInterface
	virtual QString getName() const override { return "Blur (shader)"; }
	virtual QString getDescription() const override { return "Bilateral smoothing (OpenGL shader)"; }
	virtual QIcon getIcon() const override;

	//inherited from ccGLFilterPluginInterface
	ccGlFilter* getFilter() override;
};

#endif
