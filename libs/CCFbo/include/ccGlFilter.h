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

#ifndef CC_GL_FILTER_HEADER
#define CC_GL_FILTER_HEADER

//local
#include "ccGlew.h"

//system
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
    ccGlFilter(const char* description)
    {
        strcpy(m_description,description);
    }

	//! Default destructor
	virtual ~ccGlFilter() {}

	//! Cloning mechanism
	virtual ccGlFilter* clone() const = 0;

    //! Initializes GL filter
    /** Must support reinit!
        \param width texture/screen width
        \param height texture/screen height
        \param shadersPath path where shader files are stored
        \return success
    **/
	virtual bool init(	int width,
                        int height,
                        const char* shadersPath) = 0;

    //! Applies filter to texture (depth + color)
	virtual void shade(	GLuint texDepth,
						GLuint texColor,
						float zoom = 1.0f) = 0;

    //! Returns resulting texture
	virtual GLuint getTexture() = 0;

    //! Returns filter name
	inline virtual const char* getDescription() const { return m_description; }

protected:

    //! Filter description
    char m_description[256];

};

#endif
