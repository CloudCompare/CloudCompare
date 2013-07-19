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

#ifndef CC_FACET_HEADER
#define CC_FACET_HEADER

//Local
#include "ccHObject.h"
#include "ccMesh.h"
#include "ccPolyline.h"
#include "ccPointCloud.h"

//! Facet
/** Composite object: point cloud + 2D1/2 contour polyline + 2D1/2 surface mesh
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccFacet : public ccHObject
#else
class ccFacet : public ccHObject
#endif
{
public:

	//! Default constructor
	ccFacet(QString name = QString("Facet"));

	//! Destructor
	virtual ~ccFacet();

	//! Create a facet from a set of points
	/** \param cloud cloud from which to create the facet
		\param transferOwnership whether the input cloud will be 'kept' by the facet (true) or cloned (false)
		\return a facet (or 0 if an error occured)
	**/
	static ccFacet* Create(ccPointCloud* cloud, bool transferOwnership = false);

	////! Operator: casts to generic cloud
	//operator ccGenericPointCloud*() const { return m_originPoints; }
	////! Operator: casts to cloud
	//operator ccPointCloud*() const { return m_originPoints; }
	////! Operator: casts to generic mesh
	//operator ccGenericMesh*() const { return m_polygonMesh; }
	////! Operator: casts to mesh
	//operator ccMesh*() const { return m_polygonMesh; }
	////! Operator: casts to polyline
	//operator ccPolyline*() const { return m_contourPolyline; }

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const { return CC_FACET; }
	virtual bool isSerializable() const { return true; }

	//inherited methods (ccHObject)
    virtual ccBBox getMyOwnBB();
    virtual void setDisplay_recursive(ccGenericGLDisplay* win);

	//! Sets the facet unique color
	/** \param rgb RGB color
	**/
	void setColor(const colorType rgb[]);

	//! Returns associated RMS
	double getRMS() const { return m_rms; }
	//! Returns associated surface
	double getSurface() const { return m_surface; }
	//! Returns plane equation
	const PointCoordinateType* getPlaneEquation() const { return m_planeEquation; }
	//! Returns normal
	CCVector3 getNormal() const { return CCVector3(m_planeEquation); }
	//! Returns center
	const CCVector3& getCenter() const { return m_center; }

	//! Returns polygon mesh (if any)
	ccMesh* getPolygon() { return m_polygonMesh; }
	//! Returns contour polyline (if any)
	ccPolyline* getContour() { return m_contourPolyline; }
	//! Returns associated points (if any)
	ccPointCloud* getAssociatedPoints() { return m_originPoints; }

protected:

    //inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
    
	//! Updates internal representation (polygon, polyline, etc.)
	bool updateInternalRepresentation();

	//! Clears the internal representation (polygon, polyline, etc.)
	void clearInternalRepresentation();

	//! Facet 
	ccMesh* m_polygonMesh;
	//! Facet contour
	ccPolyline* m_contourPolyline;
	//! Shared vertices (between polygon and contour)
	ccPointCloud* m_contourVertices;
	//! Origin points
	ccPointCloud* m_originPoints;

	//! Plane equation
	PointCoordinateType m_planeEquation[4];
	
	//! Facet centroid
	CCVector3 m_center;

	//! RMS (relatively to m_associatedPoints)
	double m_rms;

	//! Surface (m_polygon)
	double m_surface;

    //inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);
};

#endif //CC_FACET_PRIMITIVE_HEADER
