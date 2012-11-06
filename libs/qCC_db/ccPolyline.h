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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2265                                                              $
//$LastChangedDate:: 2012-10-13 22:22:51 +0200 (sam., 13 oct. 2012)        $
//**************************************************************************
//

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

    //! Default destructor
	virtual ~ccPolyline() {};

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return CC_POLY_LINE;};

	//inherited methods (ccHObject)
    virtual bool hasColors() const;

	//! Defines if the polyline is considered as 2D or 3D
	/** \param state if true, the polyline is 2D
	**/
    void set2DMode(bool state);

	//! Defines if the polyline is drawn in background or foreground
	/** \param state if true, the polyline is drawn in foreground
	**/
	void setForeground(bool state);

	//! Sets the polyline color
	/** \param col RGB color
	**/
	void setColor(const colorType col[]);

	//! Returns the polyline color
	/** \return a pointer to the polyline RGB color
	**/
	const colorType* getColor() const;

	//inherited methods (ccHObject)
    virtual ccBBox getMyOwnBB();

protected:

	//inherited methods (ccHObject)
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! Unique RGB color
	colorType m_rgbColor[3];

	//! Whether poyline should be considered as 2D (true) or 3D (false)
	bool m_mode2D;

	//! Whether poyline should draws itself in background (false) or foreground (true)
	bool m_foreground;
};

#endif
