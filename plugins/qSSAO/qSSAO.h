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

#ifndef Q_SSAO_PLUGIN_HEADER
#define Q_SSAO_PLUGIN_HEADER

#include "../ccGLFilterPluginInterface.h"

//! SSAO shader (Screen Space Ambient Occlusion)
class qSSAO : public QObject, public ccGLFilterPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccGLFilterPluginInterface)
#ifdef CC_QT5
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qSSAO")
#endif

public:

	//inherited from ccPluginInterface
	virtual QString getName() const { return "S.S.A.O. (shader)"; }
	virtual QString getDescription() const { return "Screen Space Ambient Occlusion OpenGL shader"; }
	virtual QIcon getIcon() const;

	//inherited from ccGLFilterPluginInterface
	ccGlFilter* getFilter();
};

#endif
