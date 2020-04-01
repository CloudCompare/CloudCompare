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

#ifndef CC_MESH_GROUP_HEADER
#define CC_MESH_GROUP_HEADER

//Local
#include "ccGenericMesh.h"

//! [DEPRECATED] A group of meshes sharing vertices (and associated properties) in a unique cloud
/** This is an empty shell for backward compatibility.
	Shouldn't be used anymore. 
**/
class QCC_DB_LIB_API ccMeshGroup : public ccGenericMesh
{
public:

	//! Default constructor
	ccMeshGroup() : ccGenericMesh("[Deprecated] Mesh Group") {}

	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::MESH_GROUP; }

	//inherited methods (ccGenericMesh)
	ccGenericPointCloud* getAssociatedCloud() const override { return 0; }
	void refreshBB() override {}
	bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) override { return false; }
	bool interpolateNormalsBC(unsigned triIndex, const CCVector3d& w, CCVector3& N) override { return false; }
	bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& color) override { return false; }
	bool interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgb& color) override { return false; }
	bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgba& color) override { return false; }
	bool interpolateColorsBC(unsigned triIndex, const CCVector3d& w, ccColor::Rgba& color) override { return false; }
	bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgba& color, bool interpolateColorIfNoTexture) override { return false; }
	bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgba& color, bool returnColorIfNoTexture) override { return false; }
	bool hasMaterials() const override { return false; }
	const ccMaterialSet* getMaterialSet() const override { return 0; }
	int getTriangleMtlIndex(unsigned triangleIndex) const override { return -1; }
	bool hasTextures() const override { return false; }
	TextureCoordsContainer* getTexCoordinatesTable() const override { return 0; }
	void getTriangleTexCoordinates(unsigned triIndex, TexCoords2D* &tx1, TexCoords2D* &tx2, TexCoords2D* &tx3) const override { tx1 = tx2 = tx3 = nullptr; }
	bool hasPerTriangleTexCoordIndexes() const override { return false; }
	void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override { i1 = i2 = i3 = -1; }
	bool hasTriNormals() const override { return false; }
	void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const override { i1 = i2 = i3 = -1; }
	bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const override { return false; }
	NormsIndexesTableType* getTriNormsTable() const override { return 0; }
	virtual unsigned capacity() const override { return 0; }
	virtual bool trianglePicking(	const CCVector2d& clickPos,
									const ccGLCameraParameters& camera,
									int& nearestTriIndex,
									double& nearestSquareDist,
									CCVector3d& nearestPoint,
									CCVector3d* barycentricCoords = nullptr) const override { return false; }

	//inherited methods (ccHObject)
	bool isSerializable() const override { return true; }
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//inherited methods (GenericIndexedMesh)
	unsigned size() const override { return 0; }
	void forEach(genericTriangleAction action) override {}
	void placeIteratorAtBeginning() override {}
	CCLib::GenericTriangle* _getNextTriangle() override { return 0; }
	CCLib::GenericTriangle* _getTriangle(unsigned index) override { return 0; }
	CCLib::VerticesIndexes* getNextTriangleVertIndexes() override { return 0; }
	CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override { return 0; }
	void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override {}
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override {}

protected:

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;
};

#endif //CC_MESH_GROUP_HEADER
