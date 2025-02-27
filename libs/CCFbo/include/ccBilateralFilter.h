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

/***************************************************************/
//
//		FILTERBILATERAL
//
//		Cross Bilateral Filter - from Tomasi98/Durand02/Eisemann04
//
//		created by Christian Boucheny (INRIA/LPPA/EDF R&D) on 12/21/2007
//
//      freely updated by Daniel Girardeau-Montaut (EDF R&D) on 03/31/2009
//
/*****************************************************************/

#include "CCFbo.h"

//Local
#include "ccGlFilter.h"
#include "ccShader.h"
#include "ccFrameBufferObject.h"

//system
#include <vector>

/**
 * \brief Bilateral filer (shader)
 * 
 * \details The Bilateral Filter is an edge-preserving and noise-reducing smoothing filter 
 * for images. It replaces the intensity of each pixel with a weighted average of 
 * neighboring pixel intensities, where the weights depend on both spatial and 
 * depth (intensity) distances.
 * 
 * Key characteristics:
 * - Preserves edges while smoothing image
 * - Reduces noise without blurring sharp details
 * - Configurable spatial and depth parameters
 * 
 * \note Implemented as an OpenGL shader-based filter
 * 
 * References:
 * - https://en.wikipedia.org/wiki/Bilateral_filter
 * 
 * \example
 * \code
 * // Create a bilateral filter
 * ccBilateralFilter bilateralFilter;
 * 
 * // Configure filter parameters
 * // halfSpatialSize: kernel size (1-7)
 * // spatialSigma: spatial distribution variance
 * // depthSigma: depth distribution variance
 * bilateralFilter.setParams(2,   // half spatial size
 *                           2.0f, // spatial sigma
 *                           0.4f  // depth sigma
 *                           );
 * 
 * // Initialize the filter for a specific viewport
 * QString error;
 * if (!bilateralFilter.init(width, height, shadersPath, error))
 * {
 *     // Handle initialization error
 *     qDebug() << "Filter init error:" << error;
 * }
 * 
 * // Apply filter to depth and color textures
 * bilateralFilter.shade(depthTexture, colorTexture, viewportParams);
 * 
 * // Get the filtered texture
 * GLuint filteredTexture = bilateralFilter.getTexture();
 * \endcode
 */
class CCFBO_LIB_API ccBilateralFilter : public ccGlFilter
{
public:
	/**
	 * \brief Default constructor
	 * 
	 * Initializes the bilateral filter with default parameters:
	 * - Half spatial size: 2
	 * - Spatial sigma: 2.0
	 * - Depth sigma: 0.4
	 */
	ccBilateralFilter();

	/**
	 * \brief Destructor
	 * 
	 * Cleans up resources associated with the bilateral filter
	 */
	virtual ~ccBilateralFilter() = default;

	/**
	 * \brief Resets the filter to its initial state
	 * 
	 * Clears any previously computed filter parameters and resets to defaults
	 */
	void reset();

	/**
	 * \brief Creates a deep copy of the filter
	 * 
	 * \return Pointer to a new ccBilateralFilter with identical parameters
	 */
	virtual ccGlFilter* clone() const override;

	/**
	 * \brief Initialize the filter for a specific viewport
	 * 
	 * \param[in] width Viewport width
	 * \param[in] height Viewport height
	 * \param[in] shadersPath Path to shader files
	 * \param[out] error String to store any initialization errors
	 * 
	 * \return True if initialization was successful, false otherwise
	 */
	virtual bool init(unsigned width, unsigned height, const QString& shadersPath, QString& error) override;

	/**
	 * \brief Apply the bilateral filter to depth and color textures
	 * 
	 * \param[in] texDepth Depth texture
	 * \param[in] texColor Color texture
	 * \param[in,out] parameters Viewport parameters
	 */
	virtual void shade(GLuint texDepth, GLuint texColor, ViewportParameters& parameters) override;

	/**
	 * \brief Get the filtered texture
	 * 
	 * \return OpenGL texture ID of the filtered result
	 */
	inline virtual GLuint getTexture() override { return m_fbo.getColorTexture(); }

	/**
	 * \brief Set bilateral filter parameters
	 * 
	 * \param[in] halfSpatialSize Half spatial kernel size (1-7)
	 *            Total kernel size will be 2*halfSpatialSize+1
	 * \param[in] spatialSigma Variance of the spatial distribution
	 *            Controls the weight of spatial proximity
	 * \param[in] depthSigma Variance of the depth distribution
	 *            Controls the weight of depth/intensity differences
	 * 
	 * \note Kernel size range is limited to 1-7 for performance reasons
	 */
	void setParams(	unsigned halfSpatialSize,
					float spatialSigma,
					float depthSigma );

	/**
	 * \brief Toggle using the current OpenGL viewport
	 * 
	 * \param[in] state True to use existing viewport, false otherwise
	 */
	void useExistingViewport(bool state);

protected: //methods
	/**
	 * \brief Update the damping (weighting) table for the filter
	 * 
	 * Precomputes spatial distribution weights for faster filtering
	 */
	void updateDampingTable();

protected: //members
	/**
	 * \brief Width of the viewport/texture
	 */
	unsigned m_width;

	/**
	 * \brief Height of the viewport/texture
	 */
	unsigned m_height;

	/**
	 * \brief Frame buffer object for rendering
	 */
	ccFrameBufferObject m_fbo;

	/**
	 * \brief Shader for bilateral filtering
	 */
	ccShader m_shader;

	/**
	 * \brief Half spatial kernel size
	 * 
	 * Total kernel width will be 2*m_halfSpatialSize+1
	 */
	unsigned m_halfSpatialSize;

	/**
	 * \brief Variance of the spatial distribution
	 * 
	 * Controls the weight of spatial proximity between pixels
	 */
	float m_spatialSigma;

	/**
	 * \brief Variance of the depth distribution
	 * 
	 * Controls the weight of depth/intensity differences between pixels
	 */
	float m_depthSigma;

	/**
	 * \brief Precomputed spatial distribution weights
	 * 
	 * Stores kernel values for faster filtering
	 */
	std::vector<float> m_dampingPixelDist;

	/**
	 * \brief Flag to use current OpenGL viewport
	 * 
	 * Determines whether to use the existing viewport or create a new one
	 */
	bool m_useCurrentViewport;

	/**
	 * \brief OpenGL 2.1 function set
	 */
	QOpenGLFunctions_2_1 m_glFunc;

	/**
	 * \brief Validity flag for OpenGL functions
	 */
	bool m_glFuncIsValid;
};
