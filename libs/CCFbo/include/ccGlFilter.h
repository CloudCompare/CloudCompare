//##########################################################################
//#                                                                        #
//#                               CCFBO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GL_FILTER_HEADER
#define CC_GL_FILTER_HEADER

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
		: m_isValid(false)
		, m_description(description)
	{}

	//! Default destructor
	virtual ~ccGlFilter() {}

	//! Returns filter name
	inline virtual QString getDescription() const { return m_description; }

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
	virtual bool init(	unsigned width,
						unsigned height,
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
	virtual void shade(	unsigned texDepth,
						unsigned texColor,
						ViewportParameters& parameters) = 0;

	//! Returns resulting texture
	virtual unsigned getTexture() = 0;

protected: //methods

	//! Sets whether the filter is valid
	inline void setValid(bool state) { m_isValid = state; }

	//! Returns whether the filter is valid
	inline bool isValid() const { return m_isValid; }

protected:

	//! Filter validity
	bool m_isValid;

	//! Filter description
	QString m_description;
};

#endif
