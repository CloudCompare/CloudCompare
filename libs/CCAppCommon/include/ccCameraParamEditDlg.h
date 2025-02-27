#pragma once
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

#include "CCAppCommon.h"

//Local
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"

//qCC_db
#include <ccGLMatrix.h>
//qCC_gl
#include <ccGLUtils.h>

//system
#include <map>

class QMdiSubWindow;
class ccGLWindowInterface;
class ccHObject;
class ccPickingHub;

namespace Ui
{
    class CameraParamDlg;
}

/**
 * \class ccCameraParamEditDlg
 * \brief Interactive dialog for editing camera parameters
 *
 * Provides a user interface to manipulate camera pose, orientation,
 * view parameters, and clipping planes in a 3D scene.
 *
 * \note Inherits from ccOverlayDialog and ccPickingListener
 * \warning Requires a valid ccPickingHub for object picking functionality
 */
class CCAPPCOMMON_LIB_API ccCameraParamEditDlg : public ccOverlayDialog, public ccPickingListener
{
    Q_OBJECT

public:
    /**
     * \brief Constructs a camera parameter editing dialog
     *
     * \param parent Parent widget
     * \param pickingHub Pointer to the picking hub for object selection
     *
     * \note Initializes the dialog with default camera settings
     */
    explicit ccCameraParamEditDlg(QWidget* parent, ccPickingHub* pickingHub);

    /**
     * \brief Destructor
     *
     * Cleans up resources associated with the dialog
     */
    ~ccCameraParamEditDlg() override;

    /**
     * \brief Removes the dialog frame
     *
     * Makes the dialog frameless, useful for overlay or custom UI designs
     */
    void makeFrameless();

    /**
     * \brief Retrieves the current camera transformation matrix
     *
     * \return ccGLMatrixd Transformation matrix representing current camera pose
     */
    ccGLMatrixd getMatrix();

    /**
     * \brief Starts the dialog
     *
     * \return bool True if dialog initialization was successful
     *
     * \note Inherited from ccOverlayDialog
     */
    bool start() override;

    /**
     * \brief Links the dialog with a specific 3D window
     *
     * \param win Pointer to the 3D window interface
     * \return bool True if linking was successful
     *
     * \note Inherited from ccOverlayDialog
     */
    bool linkWith(ccGLWindowInterface* win) override;

    /**
     * \brief Handles item picking events
     *
     * \param pi Details of the picked item
     *
     * \note Inherited from ccPickingListener
     */
    void onItemPicked(const PickedItem& pi) override;

public:
    /**
     * \brief Links the dialog with an MDI sub-window
     *
     * \param qWin Pointer to the MDI sub-window
     */
    void linkWith(QMdiSubWindow* qWin);

    /**
     * \brief Initializes dialog values with a given transformation matrix
     *
     * \param mat Camera transformation matrix
     */
    void initWithMatrix(const ccGLMatrixd& mat);

    /**
     * \brief Updates dialog values with a new pivot point
     *
     * \param P New pivot point coordinates
     */
    void updatePivotPoint(const CCVector3d& P);

    /**
     * \brief Updates dialog values with a new camera center
     *
     * \param P New camera center coordinates
     */
    void updateCameraCenter(const CCVector3d& P);

    /**
     * \brief Updates the current view mode
     */
    void updateViewMode();

    /**
     * \brief Updates the window's field of view
     *
     * \param fov_deg Field of view in degrees
     */
    void updateWinFov(float fov_deg);

    /**
     * \brief Updates the near clipping plane depth
     *
     * \param depth Depth of the near clipping plane
     */
    void updateNearClippingDepth(double depth);

    /**
     * \brief Updates the far clipping plane depth
     *
     * \param depth Depth of the far clipping plane
     */
    void updateFarClippingDepth(double depth);

    // Predefined view setters
    void setFrontView();   ///< Sets the front view orientation
    void setBottomView();  ///< Sets the bottom view orientation
    void setTopView();     ///< Sets the top view orientation
    void setBackView();    ///< Sets the back view orientation
    void setLeftView();    ///< Sets the left view orientation
    void setRightView();   ///< Sets the right view orientation
    void setIso1View();    ///< Sets the first isometric view
    void setIso2View();    ///< Sets the second isometric view

    // Angle and parameter change handlers
    void iThetaValueChanged(int);     ///< Handles integer theta value changes
    void iPsiValueChanged(int);       ///< Handles integer psi value changes
    void iPhiValueChanged(int);       ///< Handles integer phi value changes

    void dThetaValueChanged(double);  ///< Handles double theta value changes
    void dPsiValueChanged(double);    ///< Handles double psi value changes
    void dPhiValueChanged(double);    ///< Handles double phi value changes

    // Clipping plane and camera parameter change handlers
    void nearClippingDepthChanged(double);
    void nearClippingCheckBoxToggled(bool);
    void farClippingDepthChanged(double);
    void farClippingCheckBoxToggled(bool);
    void pivotChanged();
    void cameraCenterChanged();
    void fovChanged(double);

    /**
     * \brief Enables point picking to set pivot
     *
     * \param enable If true, activates pivot point picking mode
     */
    void pickPointAsPivot(bool);

    /**
     * \brief Processes a picked item for pivot point selection
     *
     * \param obj Picked object
     * \param cloudIndex Point cloud index
     * \param x X coordinate of picked point
     * \param y Y coordinate of picked point
     * \param point3D 3D point coordinates
     * \param point3D_world World coordinates of picked point
     */
    void processPickedItem(ccHObject* obj, unsigned cloudIndex, int x, int y, const CCVector3& point3D, const CCVector3d& point3D_world);

protected:
    /**
     * \brief Reflects parameter changes in the dialog
     *
     * Updates the dialog UI to reflect current camera parameters
     */
    void reflectParamChange();

    /**
     * \brief Sets a predefined camera view orientation
     *
     * \param orientation Predefined view orientation
     */
    void setView(CC_VIEW_ORIENTATION orientation);

    /**
     * \brief Stores the current camera matrix
     *
     * Pushes the current camera matrix to a map for potential restoration
     */
    void pushCurrentMatrix();

    /**
     * \brief Reverts to the previously pushed camera matrix
     *
     * Restores the camera matrix from the pushed state
     */
    void revertToPushedMatrix();

protected:
    /**
     * \brief Initializes dialog values with a specific window
     *
     * \param win Pointer to the 3D window interface
     */
    void initWith(ccGLWindowInterface* win);

    /**
     * \brief Type definition for the pushed matrices map
     *
     * Maps 3D windows to their corresponding camera matrices
     */
    using PushedMatricesMapType = std::map<ccGLWindowInterface*, ccGLMatrixd>;

    /**
     * \brief Type definition for a pushed matrices map element
     *
     * Represents a key-value pair of a 3D window and its camera matrix
     */
    using PushedMatricesMapElement = std::pair<ccGLWindowInterface*, ccGLMatrixd>;

    PushedMatricesMapType pushedMatrices; ///< Stored camera matrices per window

    ccPickingHub* m_pickingHub;           ///< Pointer to the picking hub

private:
    Ui::CameraParamDlg* m_ui;             ///< Pointer to the UI generated from Qt Designer
};
