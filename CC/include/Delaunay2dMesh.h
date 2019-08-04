//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef DELAUNAY2D_MESH_HEADER
#define DELAUNAY2D_MESH_HEADER

//Local
#include "GenericIndexedCloudPersist.h"
#include "GenericIndexedMesh.h"
#include "SimpleTriangle.h"

#include <cstddef>
#include <vector>

namespace CCLib
{

class GenericIndexedCloud;
class Polyline;

//! A class to compute and handle a Delaunay 2D mesh on a subset of points
class CC_CORE_LIB_API Delaunay2dMesh : public GenericIndexedMesh
{
public:

	//! Delaunay2dMesh constructor
	Delaunay2dMesh();

	//! Delaunay2dMesh destructor
	~Delaunay2dMesh() override;

	//! Returns whether 2D Delaunay triangulation is supported or not
	/** 2D Delaunay triangulation requires the CGAL library.
	**/
	static bool Available();

	//! Associate this mesh to a point cloud
	/** This particular mesh structure deals with point indexes instead of points.
		Therefore, it is possible to change the associated point cloud (if the
		new cloud has the same size). For example, it can be useful to compute
		the mesh on 2D points corresponding to 3D points that have been projected
		on a plane and then to link this structure with the 3D original	points.
		\param aCloud a point cloud
		\param passOwnership if true the Delaunay2dMesh destructor will delete the cloud as well
	**/
	virtual void linkMeshWith(GenericIndexedCloud* aCloud, bool passOwnership = false);

	//! Build the Delaunay mesh on top a set of 2D points
	/** \param points2D a set of 2D points
		\param pointCountToUse number of points to use from the input set (0 = all)
		\param outputErrorStr error string as output by the CGAL lib. (if any) [optional]
		\return success
	**/
	virtual bool buildMesh(	const std::vector<CCVector2>& points2D,
							std::size_t pointCountToUse = 0,
							char* outputErrorStr = nullptr);

	//! Build the Delaunay mesh from a set of 2D polylines
	/** \param points2D a set of 2D points
		\param segments2D constraining segments (as 2 indexes per segment)
		\param outputErrorStr error string as output by the CGAL lib. (if any) [optional]
		\return success
	**/
	virtual bool buildMesh(	const std::vector<CCVector2>& points2D,
							const std::vector<int>& segments2D,
							char* outputErrorStr = nullptr);

	//! Removes the triangles falling outside of a given (2D) polygon
	/** \param vertices2D vertices of the mesh as 2D points (typically the one used to triangulate the mesh!)
		\param polygon2D vertices of the 2D boundary polygon (ordered)
		\param removeOutside whether to remove triangles outside (default) or inside
		\return success
	**/
	virtual bool removeOuterTriangles(	const std::vector<CCVector2>& vertices2D,
										const std::vector<CCVector2>& polygon2D,
										bool removeOutside = true);



	//inherited methods (see GenericMesh)
	unsigned size() const override { return m_numberOfTriangles; }
	void forEach(genericTriangleAction action) override;
	void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;
	void placeIteratorAtBeginning() override;
	GenericTriangle* _getNextTriangle() override;
	GenericTriangle* _getTriangle(unsigned triangleIndex) override;
	VerticesIndexes* getNextTriangleVertIndexes() override;
	VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override;
	void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override;

	//! Returns triangles indexes array (pointer to)
	/** Handle with care!
	**/
	inline int* getTriangleVertIndexesArray() { return m_triIndexes; }

	//! Filters out the triangles based on their edge length
	/** Warning: may remove ALL triangles!
		Check the resulting size afterwards.
	**/
	bool removeTrianglesWithEdgesLongerThan(PointCoordinateType maxEdgeLength);

	//! Returns associated cloud
	inline GenericIndexedCloud* getAssociatedCloud() { return m_associatedCloud; }

	//! Tesselates a 2D polyline (shortcut to buildMesh and removeOuterTriangles)
	static Delaunay2dMesh* TesselateContour(const std::vector<CCVector2>& contourPoints);

	//! Tesselates a 2D polyline (not necessarily axis-aligned)
	static Delaunay2dMesh* TesselateContour(GenericIndexedCloudPersist* contourPoints, int flatDimension = -1);


protected:

	//! Associated point cloud
	GenericIndexedCloud* m_associatedCloud;

	//! Triangle vertex indexes
	int* m_triIndexes;

	//! Iterator on the list of triangle vertex indexes
	int* m_globalIterator;

	//! End position of global iterator
	int* m_globalIteratorEnd;

	//! The number of triangles
	unsigned m_numberOfTriangles;

	//! Specifies if the associated cloud should be deleted when the mesh is deleted
	bool m_cloudIsOwnedByMesh;

	//! Dump triangle structure to transmit temporary data
	SimpleTriangle m_dumpTriangle;

	//! Dump triangle index structure to transmit temporary data
	VerticesIndexes m_dumpTriangleIndexes;

};

}

#endif //DELAUNAY2D_MESH_HEADER
