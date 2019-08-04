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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GL_PLUGIN_INTERFACE_HEADER
#define CC_GL_PLUGIN_INTERFACE_HEADER

//CC_FBO_LIB
#include <ccGlFilter.h>

#include "ccDefaultPluginInterface.h"

//! GL Filter plugin interface
/** Version 1.3
**/
class ccGLPluginInterface : public ccDefaultPluginInterface
{
public:
	ccGLPluginInterface( const QString &resourcePath = QString() ) :
		ccDefaultPluginInterface( resourcePath )
	{
	}
	
	~ccGLPluginInterface() override = default;

	//inherited from ccPluginInterface
	CC_PLUGIN_TYPE getType() const override { return CC_GL_FILTER_PLUGIN; }

	//! Returns a GL filter object
	virtual ccGlFilter* getFilter() = 0;
};

Q_DECLARE_METATYPE(ccGLPluginInterface *);

Q_DECLARE_INTERFACE(ccGLPluginInterface,
                    "edf.rd.CloudCompare.ccGLFilterPluginInterface/1.3")

#endif //CC_GL_FILTER_PLUGIN_INTERFACE_HEADER
