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

#ifndef CC_GL_POLYLINE_HEADER
#define CC_GL_POLYLINE_HEADER

//CCLib
#include <Polyline.h>

#include "ccHObject.h"

//! Colored polyline
/** Extends the Polyline class of CCLib.
Check CCLib documentation for more information about it.
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccPolyline : public CCLib::Polyline, public ccHObject
#else
class ccPolyline : public CCLib::Polyline, public ccHObject
#endif
{
public:

	//! Default constructor
	/** \param associatedCloud the associated point cloud (e.g. the summits)
	**/
	ccPolyline(GenericIndexedCloudPersist* associatedCloud);

	//! Copy constructor
	/** \param poly polyline to clone
	**/
	ccPolyline(const ccPolyline& poly);

	//! Destructor
	virtual ~ccPolyline() {};

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const {return CC_POLY_LINE;}

	//inherited methods (ccHObject)
	virtual bool isSerializable() const { return true; }
	virtual bool hasColors() const;
    virtual void applyGLTransformation(const ccGLMatrix& trans);
	virtual unsigned getUniqueIDForDisplay() const;

	//! Defines if the polyline is considered as 2D or 3D
	/** \param state if true, the polyline is 2D
	**/
	void set2DMode(bool state);

	//! Returns whether the polyline is considered as 2D or 3D
	inline bool is2DMode() const { return m_mode2D; }

	//! Defines if the polyline is drawn in background or foreground
	/** \param state if true, the polyline is drawn in foreground
	**/
	void setForeground(bool state);

	//! Sets the polyline color
	/** \param col RGB color
	**/
	void setColor(const colorType col[]);

	//! Sets the width of the line
	/**  \param width the desired width
	**/
	void setWidth(PointCoordinateType width);

	//! Returns the polyline color
	/** \return a pointer to the polyline RGB color
	**/
	const colorType* getColor() const;

	//inherited methods (ccHObject)
	virtual ccBBox getMyOwnBB();

	//! Extracts the (flat) contour of a point cloud
	/** Projects the cloud on its best fitting LS plane first.
		\param points point cloud
		\param maxEdgelLength max edge length (ignored if 0, in which case the contour is the convex hull)
		\return contour polyline (or 0 if an error occurred)
	**/
	static ccPolyline* ExtractFlatContour(	CCLib::GenericIndexedCloudPersist* points,
											PointCoordinateType maxEdgelLength = 0);

	//! Computes the polyline length
	PointCoordinateType computeLength() const;

protected:

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);

	//inherited methods (ccHObject)
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! Unique RGB color
	colorType m_rgbColor[3];

	//! Width of the line
	PointCoordinateType m_width;

	//! Whether poyline should be considered as 2D (true) or 3D (false)
	bool m_mode2D;

	//! Whether poyline should draws itself in background (false) or foreground (true)
	bool m_foreground;
};

#endif //CC_GL_POLYLINE_HEADER
