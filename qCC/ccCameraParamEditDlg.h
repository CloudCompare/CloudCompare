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

#ifndef CC_CAMERA_PARAM_EDIT_DLG_HEADER
#define CC_CAMERA_PARAM_EDIT_DLG_HEADER

//Local
#include "ccOverlayDialog.h"
#include "ccGLWindow.h"

#include <ui_cameraParamDlg.h>

//qCC_db
#include <ccGLMatrix.h>

//system
#include <map>

class QMdiSubWindow;
class ccGLWindow;

//! Dialog to interactively edit the camera pose parameters
class ccCameraParamEditDlg : public ccOverlayDialog, public Ui::CameraParamDlg
{
    Q_OBJECT

public:

    //! Default constructor
	ccCameraParamEditDlg(QWidget* parent);

    //! Destructor
	virtual ~ccCameraParamEditDlg();

	//! Makes this dialog frameless
	void makeFrameless();

    //! Returns matrix corresponding to dialog values
    ccGLMatrix getMatrix();

	//inherited from ccOverlayDialog
	virtual bool start();
    virtual bool linkWith(ccGLWindow* win);

public slots:

	//! Links this dialog with a given sub-window
    void linkWith(QMdiSubWindow* qWin);

    //! Inits dialog values with matrix
    void initWithMatrix(const ccGLMatrix& mat);

	//! Updates dialog values with pivot point
	void updatePivotPoint(const CCVector3& P);

    void setFrontView();
    void setBottomView();
    void setTopView();
    void setBackView();
    void setLeftView();
    void setRightView();

    void iThetaValueChanged(int);
    void iPsiValueChanged(int);
    void iPhiValueChanged(int);

    void dThetaValueChanged(double);
    void dPsiValueChanged(double);
    void dPhiValueChanged(double);

    void translationChanged(double);
    void fovChanged(double);

	void pickPointAsPivot();
	void processPickedPoint(int, unsigned, int, int);

protected slots:

    //! Reflects any dialog parameter change
    void reflectParamChange();

    //! Places the camera in a given prefedined orientation
    void setView(CC_VIEW_ORIENTATION orientation);

    //! Pushes current matrix
    void pushCurrentMatrix();

    //! Reverts to pushed matrix
    void revertToPushedMatrix();

protected:

    //! Type of the pushed matrices map structure
    typedef std::map<ccGLWindow*,ccGLMatrix> PushedMatricesMapType;
    //! Type of an element of the pushed matrices map structure
    typedef std::pair<ccGLWindow*,ccGLMatrix> PushedMatricesMapElement;

    //! Pushed camera matrices (per window)
    PushedMatricesMapType pushedMatrices;
};

#endif
