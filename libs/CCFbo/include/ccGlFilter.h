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

//Qt
#include <QString>

//! Default GL filter interface
/** A GL filter is a combination of shaders applied to
	textures (typically the rendered scene), typically
	through intensive use of Frame Buffer Objects.
**/
class ccGlFilter
{
public:

	//! Default constructor
	ccGlFilter(QString description)
		: m_description(description)
	{}

	//! Default destructor
	virtual ~ccGlFilter() {}

	//! Cloning mechanism
	virtual ccGlFilter* clone() const = 0;

	//! Initializes GL filter
	/** Must support reinit!
		\param width texture/screen width
		\param height texture/screen height
		\param shadersPath path where shader files are stored
		\param error error string (if an error occurred)
		\return success
		**/
	virtual bool init(	int width,
						int height,
						QString shadersPath,
						QString& error) = 0;

	//! Minimal set of 3D viewport parameters that can be used by shaders
	struct ViewportParameters
	{
		//! Default constructor
		ViewportParameters()
			: zoom(1.0)
			, perspectiveMode(false)
			, zNear(0.0)
			, zFar(1.0)
		{}

		//! Zoom
		double zoom;
		//! Whether perspective mode is enabled or not
		bool perspectiveMode;
		//! Near clipping plane position (perspective mode only)
		double zNear;
		//! Far clipping plane position (perspective mode only)
		double zFar;
	};

	//! Applies filter to texture (depth + color)
	virtual void shade(	GLuint texDepth,
						GLuint texColor,
						ViewportParameters& parameters) = 0;

	//! Returns resulting texture
	virtual GLuint getTexture() = 0;

	//! Returns filter name
	inline virtual QString getDescription() const { return m_description; }

protected:

	//! Filter description
	QString m_description;
};

#endif
