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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2142                                                              $
//$LastChangedDate:: 2012-05-22 19:19:08 +0200 (mar., 22 mai 2012)         $
//**************************************************************************
//

#ifndef CC_PLUGIN_INTERFACE_HEADER
#define CC_PLUGIN_INTERFACE_HEADER

//Qt
#include <QString>
#include <QIcon>

//! Plugin type
enum  CC_PLUGIN_TYPE {  CC_STD_PLUGIN               = 0,
                        CC_GL_FILTER_PLUGIN         = 1,
};

//! Standard CC plugin interface
/** Version 2.0
**/
class ccPluginInterface
{
public:

	//! Virtual destructor
    virtual ~ccPluginInterface() {}

	//! Returns plugin type (standard or OpenGL filter)
    virtual CC_PLUGIN_TYPE getType() const = 0;

	//! Returns (short) name (for menu entry, etc.)
    virtual QString getName() const = 0;

	//! Returns long name/description (for tooltip, etc.)
    virtual QString getDescription() const = 0;

	//! Returns icon
	/** Should be reimplemented if necessary
	**/
	virtual QIcon getIcon() const { return QIcon(); }
};

Q_DECLARE_INTERFACE(ccPluginInterface,
                    "edf.rd.CloudCompare.ccPluginInterface/2.0")


#endif
