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

#ifndef CC_GL_POLYLINE_HEADER
#define CC_GL_POLYLINE_HEADER

//CCCoreLib
#include <Polyline.h>

//Local
#include "ccShiftedObject.h"

class ccPointCloud;
class ccGLCameraParameters;
//! Colored polyline
/** Extends the CCCoreLib::Polyline class
**/
class QCC_DB_LIB_API ccPolyline : public CCCoreLib::Polyline, public ccShiftedObject
{
public:

	//! Default constructor
	/** \param associatedCloud the associated point cloud (i.e. the vertices)
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccPolyline(GenericIndexedCloudPersist* associatedCloud, unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Copy constructor
	/** \param poly polyline to clone
	**/
	ccPolyline(const ccPolyline& poly);

	//! Destructor
	virtual ~ccPolyline() override = default;

	//! Returns class ID
	CC_CLASS_ENUM getClassID() const override {return CC_TYPES::POLY_LINE;}

	//inherited methods (ccHObject)
	bool isSerializable() const override { return true; }
	bool hasColors() const override;
	void applyGLTransformation(const ccGLMatrix& trans) override;
	unsigned getUniqueIDForDisplay() const override;

	//inherited methods (ccShiftedObject)
	void setGlobalShift(const CCVector3d& shift) override;
	void setGlobalScale(double scale) override;
	const CCVector3d& getGlobalShift() const override;
	double getGlobalScale() const override;

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
	inline void setColor(const ccColor::Rgb& col) { m_rgbColor = col; }

	//! Sets the width of the line
	/**  \param width the desired width
	**/
	void setWidth(PointCoordinateType width);

	//! Returns the width of the line
	/** \return the width of the line in pixels
	**/
	inline PointCoordinateType getWidth() const { return m_width; }

	//! Returns the polyline color
	/** \return a pointer to the polyline RGB color
	**/
	inline const ccColor::Rgb& getColor() const { return m_rgbColor; }

	//inherited methods (ccHObject)
	virtual ccBBox getOwnBB(bool withGLFeatures = false) override;
	inline virtual void drawBB(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col) override
	{
		//DGM: only for 3D polylines!
		if (!is2DMode())
			ccShiftedObject::drawBB(context, col);
	}


	//! Splits the polyline into several parts based on a maximum edge length
	/** \warning output polylines set (parts) may be empty if all the vertices are too far from each other!
		\param maxEdgeLength maximum edge length
		\param[out] parts output polyline parts
		\return success
	**/
	bool split(	PointCoordinateType maxEdgeLength,
				std::vector<ccPolyline*>& parts );

	//! Computes the polyline length
	PointCoordinateType computeLength() const;

	//! Sets whether to display or hide the polyline vertices
	void showVertices(bool state) { m_showVertices = state; }
	//! Whether the polyline vertices should be displayed or not
	bool verticesShown() const { return m_showVertices; }

	//! Sets the width of vertex markers
	void setVertexMarkerWidth(int width) { m_vertMarkWidth = width; }

	//! Returns the width of vertex markers
	int getVertexMarkerWidth() const { return m_vertMarkWidth; }

	//! Return Poly Group Index
	unsigned getGroupIndex() const { return m_groupIndex; };

	//! Set Poly Group Index
	void setGroupIndex(unsigned groupIndex) { m_groupIndex = groupIndex; }
	
	//! Return if point inside/Outside
	bool getPointInside() const { return m_pointInside; }

	//! Set if inside/outside segmentation
	void setPointInside(bool pointInside) { m_pointInside = pointInside?true:false; }



	//! Initializes the polyline with a given set of vertices and the parameters of another polyline
	/** \warning Even the 'closed' state is copied as is!
		\param vertices set of vertices (can be null, in which case the polyline vertices will be cloned)
		\param poly polyline
		\return success
	**/
	bool initWith(ccPointCloud*& vertices, const ccPolyline& poly);

	//! Copy the parameters from another polyline
	void importParametersFrom(const ccPolyline& poly);

	//! Shows an arrow in place of a given vertex
	void showArrow(bool state, unsigned vertIndex, PointCoordinateType length);

	//! Returns the number of segments
	unsigned segmentCount() const;

	//! Samples points on the polyline
	ccPointCloud* samplePoints(	bool densityBased,
								double samplingParameter,
								bool withRGB);

	//! Smoothes the polyline (Chaikin algorithm)
	/** \param ratio between 0 and 0.5 (excluded)
		\param iterationCount of iteration
		\return smoothed polyline
	**/
	ccPolyline* smoothChaikin(	PointCoordinateType ratio,
								unsigned iterationCount) const;
	

public: //meta-data keys
	
	//! Meta data key: vertical direction (for 2D polylines, contour plots, etc.)
	/** Expected value: 0(=X), 1(=Y) or 2(=Z) as int
	**/
	static QString MetaKeyUpDir()			{ return "up.dir"; }
	//! Meta data key: contour plot constant altitude (for contour plots, etc.)
	/** Expected value: altitude as double
	**/
	static QString MetaKeyConstAltitude()	{ return "contour.altitude"; }
	//! Meta data key: profile abscissa along generatrix
	static QString MetaKeyAbscissa()		{ return "profile.abscissa"; }
	//! Meta data key (prefix): intersection point between profile and its generatrix
	/** Expected value: 3D vector
		\warning: must be followed by '.x', '.y' or '.z'
	**/
	static QString MetaKeyPrefixCenter()	{ return "profile.center"; }
	//! Meta data key (prefix): generatrix orientation at the point of intersection with the profile
	/** Expected value: 3D vector
		\warning: must be followed by '.x', '.y' or '.z'
	**/
	static QString MetaKeyPrefixDirection()	{ return "profile.direction"; }

protected:

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//inherited methods (ccHObject)
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Unique RGB color
	ccColor::Rgb m_rgbColor;

	//! Width of the line
	PointCoordinateType m_width;

	//! Whether poyline should be considered as 2D (true) or 3D (false)
	bool m_mode2D;

	//! Whether poyline should draws itself in background (false) or foreground (true)
	bool m_foreground;
	
	//! Whether vertices should be displayed or not
	bool m_showVertices;

	//! Vertex marker width
	int m_vertMarkWidth;

	//! Whether to show an arrow or not
	bool m_showArrow;
	//! Arrow length
	PointCoordinateType m_arrowLength;
	//! Arrow index
	unsigned m_arrowIndex;

	//! Group index
	unsigned m_groupIndex;

	//! Point Inside
	bool m_pointInside;

};

#endif //CC_GL_POLYLINE_HEADER
