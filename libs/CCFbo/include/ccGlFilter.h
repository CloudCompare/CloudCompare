//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
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
//$Rev:: 1933                                                              $
//$LastChangedDate:: 2011-11-20 23:42:07 +0100 (dim., 20 nov. 2011)        $
//**************************************************************************
//

#ifndef CC_GL_FILTER_HEADER
#define CC_GL_FILTER_HEADER

#include "ccGlew.h"

#include <string.h>

//! Default GL filter interface
/** A GL filter is a combination of shaders applied to
    textures (typically the rendered scene), typically
    through intensive use of Frame Buffer Objects.
**/
class ccGlFilter
{
public:

    //! Default constructor
    ccGlFilter(const char* filterName)
    {
        strcpy(name,filterName);
    };

	//! Default destructor
	virtual ~ccGlFilter() {};

    //! Initializes GL filter
    /** Must support reinit!
        \param width texture/screen width
        \param height texture/screen height
        \param shadersPath path where shader files are stored
        \return success
    **/
	virtual bool init(int width,
                        int height,
                        const char* shadersPath)=0;

    //! Applies filter to texture (depth + color)
	virtual void shade(GLuint texDepth, GLuint texColor, float zoom = 1.0)=0;

    //! Returns resulting texture
	virtual GLuint getTexture()=0;

    //! Returns filter name
	virtual const char* getName() const {return name;};

protected:

    //! Filter name
    char name[256];

};

#endif
