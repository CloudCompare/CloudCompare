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

#ifndef CC_MESH_GROUP_HEADER
#define CC_MESH_GROUP_HEADER

//Local
#include "qCC_db.h"
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
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::MESH_GROUP; }

	//inherited methods (ccGenericMesh)
	virtual ccGenericPointCloud* getAssociatedCloud() const { return 0; }
	virtual ccGenericMesh* clone(ccGenericPointCloud* vertices = 0, ccMaterialSet* clonedMaterials = 0, NormsIndexesTableType* clonedNormsTable = 0, TextureCoordsContainer* cloneTexCoords = 0) { return 0; }
	virtual void refreshBB() {};
	virtual bool interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N) { return false; }
	virtual bool interpolateColors(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb) { return false; }
	virtual bool getColorFromMaterial(unsigned triIndex, const CCVector3& P, ccColor::Rgb& rgb, bool interpolateColorIfNoTexture) { return false; }
	virtual bool getVertexColorFromMaterial(unsigned triIndex, unsigned char vertIndex, ccColor::Rgb& rgb, bool returnColorIfNoTexture) { return false; }
	virtual bool hasMaterials() const { return false; }
	const ccMaterialSet* getMaterialSet() const { return 0; }
	virtual int getTriangleMtlIndex(unsigned triangleIndex) const { return -1; }
	virtual bool hasTextures() const { return false; }
	virtual TextureCoordsContainer* getTexCoordinatesTable() const { return 0; }
	virtual void getTriangleTexCoordinates(unsigned triIndex, float* &tx1, float* &tx2, float* &tx3) const { tx1 = tx2 = tx3 = 0; }
	virtual bool hasPerTriangleTexCoordIndexes() const { return false; }
	virtual void getTriangleTexCoordinatesIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const { i1 = i2 = i3 = -1; }
	virtual bool hasTriNormals() const { return false; }
	virtual void getTriangleNormalIndexes(unsigned triangleIndex, int& i1, int& i2, int& i3) const { i1 = i2 = i3 = -1; }
	virtual bool getTriangleNormals(unsigned triangleIndex, CCVector3& Na, CCVector3& Nb, CCVector3& Nc) const { return false; }
	virtual NormsIndexesTableType* getTriNormsTable() const { return 0; }
	virtual unsigned maxSize() const { return 0; }

	//inherited methods (ccHObject)
	virtual bool isSerializable() const { return true; }
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);

	//inherited methods (GenericIndexedMesh)
	virtual unsigned size() const { return 0; }
	virtual void forEach(genericTriangleAction& action) {}
	virtual void placeIteratorAtBegining() {}
	virtual CCLib::GenericTriangle* _getNextTriangle() { return 0; }
	virtual CCLib::GenericTriangle* _getTriangle(unsigned index) { return 0; }
	virtual CCLib::VerticesIndexes* getNextTriangleVertIndexes() { return 0; }
	virtual CCLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) { return 0; }
	virtual void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) {}
	virtual void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) {};

protected:

	//inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
};

#endif //CC_MESH_GROUP_HEADER
