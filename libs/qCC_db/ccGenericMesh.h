//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GENERIC_MESH_HEADER
#define CC_GENERIC_MESH_HEADER

//CCLib
#include <GenericIndexedMesh.h>
#include <ReferenceCloud.h>
#include <GenericProgressCallback.h>

//Local
#include "ccGenericGLDisplay.h"
#include "ccAdvancedTypes.h"

class ccGenericPointCloud;
class ccPointCloud;
class ccMaterialSet;

//! Generic mesh interface
class QCC_DB_LIB_API ccGenericMesh : public CCLib::GenericIndexedMesh, public ccHObject
{

public:

	//! Default constructor
	/** \param name object name
	**/
	ccGenericMesh(QString name = QString());

	//! Destructor
	virtual ~ccGenericMesh() {}

	//inherited methods (ccDrawableObject)
	virtual void showNormals(bool state) override;

	//inherited methods (ccHObject)
	virtual bool isSerializable() const override { return true; }

	//! Returns the vertices cloud
	virtual ccGenericPointCloud* getAssociatedCloud() const = 0;

	//! Forces bounding-box update
	virtual void refreshBB() = 0;

	//! Returns max capacity
	virtual unsigned capacity() const = 0;

	//! Returns whether the mesh has materials/textures
	virtual bool hasMaterials() const = 0;

	//! Returns associated material set
	virtual const ccMaterialSet* getMaterialSet() const = 0;

	//! Returns a given triangle material indexes
	virtual int getTriangleMtlIndex(unsigned triangleIndex) const = 0;

	//! Returns whether textures are available for this mesh
	virtual bool hasTextures() const = 0;

	//! Returns per-triangle texture coordinates array
	virtual TextureCoordsContainer* getTexCoordinatesTable() const = 0;

	//! Returns per-triangle texture coordinates (pointer to)
	virtual void getTriangleTexCoordinates(unsigned triIndex, float* &tx1, float* &tx2, float* &tx3) const = 0;

	//! Returns whether this mesh as per-triangle triplets of tex coords indexes
	virtual bool hasPerTriangleTexCoordIndexes() const = 0;

	//! Returns the triplet of tex coords indexes for a given triangle
	/** \param triangleIndex triangle index
		\param i1 first vertex tex coords index
		\param i2 second vertex tex coords index
		\param i3 third vertex tex coords index
	**/
	virtual void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const = 0;

	//! Returns whether the mesh has per-triangle normals
	virtual bool hasTriNormals() const = 0;

	//! Returns a triplet of normal indexes for a given triangle (if any)
	/** \param[in] triangleIndex triangle index
		\param[out] i1 first vertex normal index (or -1 if none)
		\param[out] i2 second vertex normal index (or -1 if none)
		\param[out] i3 third vertex normal index (or -1 if none)
	**/
	virtual void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const = 0;

	//! Returns a given triangle normal
	/** Mesh must have triangle normals associated (see hasTriNormals)
	**/
	virtual bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const = 0;

	//! Returns per-triangle normals shared array
	virtual NormsIndexesTableType* getTriNormsTable() const = 0;
	
	//! Returns the (barycentric) interpolation weights for a given triangle
	virtual void computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const;

	//! Interpolates normal(s) inside a given triangle
	/** \param triIndex triangle index
		\param P point where to interpolate (should be inside the triangle!)
		\param[out] N interpolated normal
		\return success
	**/
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) = 0;

	//! Interpolates RGB colors inside a given triangle
	/** \param triIndex triangle index
		\param P point where to interpolate (should be inside the triangle!)
		\param[out] C interpolated color
		\return success
	**/
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& C) = 0;

	//! Returns RGB color fom a given triangle material/texture
	/** \param triIndex triangle index
		\param P point where to grab color (should be inside the triangle!)
		\param[out] C texel color
		\param interpolateColorIfNoTexture whether to return the color interpolated from the RGB field if no texture/material is associated to the given triangles
		\return success
	**/
	virtual bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& C, bool interpolateColorIfNoTexture) = 0;

	//! Returns RGB color of a vertex fom a given triangle material/texture
	/** \param triIndex triangle index
		\param vertIndex vertex index inside triangle (i.e. 0, 1 or 2!)
		\param[out] C texel color
		\param returnColorIfNoTexture whether to return the color from the vertex RGB field if no texture/material is associated to the given triangle
		\return success
	**/
	virtual bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& C, bool returnColorIfNoTexture) = 0;

	//! Returns whether the mesh is displayed as wired or with plain facets
	virtual bool isShownAsWire() const { return m_showWired; }

	//! Sets whether mesh should be displayed as a wire or with plain facets
	virtual void showWired(bool state) { m_showWired = state; }

	//! Returns whether per-triangle normals are shown or not 
	virtual bool triNormsShown() const { return m_triNormsShown; }

	//! Sets whether to show or not per-triangle normals
	virtual void showTriNorms(bool state) { m_triNormsShown = state; }

	//! Sets whether textures/material should be displayed or not
	virtual bool materialsShown() const { return m_materialsShown; }

	//! Sets whether textures should be displayed or not
	virtual void showMaterials(bool state) { m_materialsShown = state; }

	//! Returns whether polygon stippling is enabled or not
	virtual bool stipplingEnabled() const { return m_stippling; }

	//! Enables polygon stippling
	void enableStippling(bool state) { m_stippling = state; }

	//! Samples points on a mesh
	ccPointCloud* samplePoints(	bool densityBased,
								double samplingParameter,
								bool withNormals,
								bool withRGB,
								bool withTexture,
								CCLib::GenericProgressCallback* pDlg = 0);

	//! Imports the parameters from another mesh
	/** Only the specific parameters are imported.
	**/
	void importParametersFrom(const ccGenericMesh* mesh);

	//! Brute force triangle picking
	virtual bool trianglePicking(	const CCVector2d& clickPos,
									const ccGLCameraParameters& camera,
									int& nearestTriIndex,
									double& nearestSquareDist,
									CCVector3d& nearestPoint);

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;

	//Static arrays for OpenGL drawing
	static PointCoordinateType* GetVertexBuffer();
	static PointCoordinateType* GetNormalsBuffer();
	static ColorCompType* GetColorsBuffer();

	//! Returns a pre-initialized array of vertex indexes for wired display
	/** Array size is MAX_NUMBER_OF_ELEMENTS_PER_CHUNK*6 by default
	**/
	static unsigned* GetWireVertexIndexes();

	//! Enables (OpenGL) stipple mask
	static void EnableGLStippleMask(const QOpenGLContext* context, bool state);

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Handles the color ramp display
	void handleColorRamp(CC_DRAW_CONTEXT& context);

	//! Per-triangle normals display flag
	bool m_triNormsShown;

	//! Texture/material display flag
	bool m_materialsShown;

	//! Wireframe display mode
	bool m_showWired;

	//! Polygon stippling state
	bool m_stippling;
};

#endif //CC_GENERIC_MESH_HEADER
