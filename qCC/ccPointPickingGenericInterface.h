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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_POINT_PICKING_GENERIC_INTERFACE_HEADER
#define CC_POINT_PICKING_GENERIC_INTERFACE_HEADER

//Qt
#include <QDialog>

//CCLib
#include <CCGeom.h>

#include "ccCommon.h"

#include <vector>

class ccGLWindow;
class ccPointCloud;
class ccHObject;

/** Generic interface for any dialog/graphical interactor that relies on point picking.
**/
class ccPointPickingGenericInterface : public QDialog
{
    Q_OBJECT

public:

    //! Default constructor
	ccPointPickingGenericInterface(QWidget* parent);

    //! Default destructor
	virtual ~ccPointPickingGenericInterface();

    //! Links interactor with a 3D GL window
    virtual void linkWith(ccGLWindow* win);

#ifndef CC_OPENGL_POINT_PICKING
    //! Adds a cloud to internal 'pickable' entities DB
    /** Cloud's display should be the same as the associated
        GL window (see ccPointPickingGenericInterface::linkWith).
        \return true if entity has been pushed successfully
    **/
	virtual bool addCloud(const ccPointCloud* cloud);

	//! Returns the number of clouds in internal 'pickable' entities DB
    virtual unsigned getDBSize() const;
#endif

    //! Starts process
    /** \return success
    **/
	virtual bool start();

    //! Stops process
    /** Signal processFinished is automatically called with input state.
        \param state process result
    **/
	virtual void stop(bool state);

protected slots:

    //! Slot to handle directly a picked point (OpenGL based picking)
    virtual void handlePickedPoint(int cloudID, unsigned pointIdx, int x, int y);

    #ifndef CC_OPENGL_POINT_PICKING
    //! Slot to handle a clicked pixel (CPU based picking)
    virtual void handleClickedPixel(int xPix, int yPix);
    #endif

signals:

    //! Signal emitted when process is finished
    /** \param success specifies how the process finished (canceled, etc.)
    **/
    void processFinished(bool success);

protected:

    //! Generic method to process picked points
    /** \param P picked point coordinates
        \param cloud picked point cloud
        \param pointIndex point index in cloud
    **/
    virtual void processPickedPoint(ccPointCloud* cloud, unsigned pointIndex, int x, int y)=0;

#ifndef CC_OPENGL_POINT_PICKING
    //! Associated clouds DB (CPU based picking)
    std::vector<const ccPointCloud*> m_clouds;
#endif

    //! Associated window
    ccGLWindow* m_win;

    //! Running state
    bool m_processing;
};

#endif
