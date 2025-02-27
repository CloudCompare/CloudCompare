#pragma once
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

#include "CCFbo.h"

//Qt
#include <QString>

/**
 * \brief Abstract base class for OpenGL shader-based texture filtering
 * 
 * \details Provides a flexible interface for applying advanced rendering 
 * filters to textures using OpenGL shaders and Frame Buffer Objects (FBO).
 * 
 * Key characteristics:
 * - Defines a common interface for GL-based texture filtering
 * - Supports dynamic filter initialization and application
 * - Enables complex post-processing effects
 * 
 * Typical use cases:
 * - Scene rendering effects
 * - Image post-processing
 * - Custom shader-based transformations
 * 
 * \note This is an abstract base class that must be inherited and implemented
 * 
 * \example
 * \code
 * // Custom filter implementation
 * class MyCustomGlFilter : public ccGlFilter 
 * {
 * public:
 *     MyCustomGlFilter() : ccGlFilter("My Custom Filter") {}
 * 
 *     // Implement clone mechanism
 *     ccGlFilter* clone() const override 
 *     {
 *         return new MyCustomGlFilter(*this);
 *     }
 * 
 *     // Initialize filter
 *     bool init(unsigned width, unsigned height, 
 *               const QString& shadersPath, QString& error) override 
 *     {
 *         // Load and compile shaders
 *         // Set up Frame Buffer Object
 *         setValid(true);
 *         return true;
 *     }
 * 
 *     // Apply filter to textures
 *     void shade(unsigned texDepth, unsigned texColor, 
 *                ViewportParameters& parameters) override 
 *     {
 *         // Apply custom shader effects
 *     }
 * 
 *     // Return processed texture
 *     unsigned getTexture() override 
 *     {
 *         return m_processedTextureId;
 *     }
 * 
 * private:
 *     unsigned m_processedTextureId;
 * };
 * \endcode
 */
class CCFBO_LIB_API ccGlFilter
{
public:
	/**
	 * \brief Constructor with filter description
	 * 
	 * \param[in] description Human-readable description of the filter
	 */
	ccGlFilter(QString description)
		: m_isValid(false)
		, m_description(description)
	{}

	/**
	 * \brief Virtual destructor
	 * 
	 * Ensures proper cleanup of derived class resources
	 */
	virtual ~ccGlFilter() {}

	/**
	 * \brief Get the filter's description
	 * 
	 * \return Human-readable description of the filter
	 */
	inline virtual QString getDescription() const { return m_description; }

	/**
	 * \brief Create a deep copy of the filter
	 * 
	 * \return Pointer to a new, identical filter instance
	 * 
	 * \note Must be implemented by derived classes
	 */
	virtual ccGlFilter* clone() const = 0;

	/**
	 * \brief Initialize the GL filter
	 * 
	 * \param[in] width Texture or screen width
	 * \param[in] height Texture or screen height
	 * \param[in] shadersPath Path to shader files
	 * \param[out] error String to store any initialization errors
	 * 
	 * \return True if initialization was successful, false otherwise
	 * 
	 * \note Supports reinitialization
	 * \note Must be implemented by derived classes
	 */
	virtual bool init(	unsigned width,
						unsigned height,
						const QString& shadersPath,
						QString& error) = 0;

	/**
	 * \brief Viewport parameters for shader rendering
	 * 
	 * \details Provides essential rendering context information 
	 * that can be used by shaders during filter application
	 */
	struct CCFBO_LIB_API ViewportParameters
	{
		/**
		 * \brief Default constructor
		 * 
		 * Initializes viewport parameters with default values
		 */
		ViewportParameters()
			: perspectiveMode(false)
			, zNear(0.0)
			, zFar(1.0)
			, zoomFactor(1.0)
		{}

		/**
		 * \brief Flag indicating perspective rendering mode
		 */
		bool perspectiveMode;

		/**
		 * \brief Near clipping plane position
		 */
		double zNear;

		/**
		 * \brief Far clipping plane position
		 */
		double zFar;

		/**
		 * \brief Rendering zoom factor
		 */
		float zoomFactor;
	};

	/**
	 * \brief Apply filter to depth and color textures
	 * 
	 * \param[in] texDepth Depth texture identifier
	 * \param[in] texColor Color texture identifier
	 * \param[in,out] parameters Viewport rendering parameters
	 * 
	 * \note Must be implemented by derived classes
	 */
	virtual void shade(	unsigned texDepth,
						unsigned texColor,
						ViewportParameters& parameters) = 0;

	/**
	 * \brief Get the resulting filtered texture
	 * 
	 * \return Texture identifier of the processed image
	 * 
	 * \note Must be implemented by derived classes
	 */
	virtual unsigned getTexture() = 0;

protected: //methods
	/**
	 * \brief Set the filter's validity state
	 * 
	 * \param[in] state True if filter is valid, false otherwise
	 */
	inline void setValid(bool state) { m_isValid = state; }

	/**
	 * \brief Check if the filter is in a valid state
	 * 
	 * \return True if filter is valid, false otherwise
	 */
	inline bool isValid() const { return m_isValid; }

protected:
	/**
	 * \brief Flag indicating filter validity
	 */
	bool m_isValid;

	/**
	 * \brief Human-readable description of the filter
	 */
	QString m_description;
};
