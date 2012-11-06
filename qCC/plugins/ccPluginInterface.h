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

#include <QIcon>

//! Plugin type
enum  CC_PLUGIN_TYPE {  CC_STD_PLUGIN               = 0,
                        CC_GL_FILTER_PLUGIN         = 1,
};

//! Plugin description structure
struct ccPluginDescription
{
    ccPluginDescription()
    {
        name[0]=0;
        menuName[0]=0;
        version=0;
        hasAnIcon=false;
    }

    char name[256];
    char menuName[64];
    int version;
    bool hasAnIcon;
};

//! Standard CC plugin interface
/** Version 1.1
**/
class ccPluginInterface
{
public:

	//! Virtual destructor
    virtual ~ccPluginInterface() {}

	//! Returns plugin type (standard or OpenGL filter)
    virtual CC_PLUGIN_TYPE getType()=0;

    //! Returns a short description of the plugin
    /** \param desc output description structure
    **/
    virtual void getDescription(ccPluginDescription& desc)=0;

    //! Returns icon (if available)
    virtual QIcon getIcon() const=0;
};

Q_DECLARE_INTERFACE(ccPluginInterface,
                    "edf.rd.CloudCompare.ccPluginInterface/1.1")


#endif
