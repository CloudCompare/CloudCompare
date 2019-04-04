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

#ifndef CC_CAMERA_PARAM_EDIT_DLG_HEADER
#define CC_CAMERA_PARAM_EDIT_DLG_HEADER

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
class ccGLWindow;
class ccHObject;
class ccPickingHub;

namespace Ui
{
	class CameraParamDlg;
}

//! Dialog to interactively edit the camera pose parameters
class ccCameraParamEditDlg : public ccOverlayDialog, public ccPickingListener
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccCameraParamEditDlg(QWidget* parent, ccPickingHub* pickingHub);

	//! Destructor
	~ccCameraParamEditDlg() override;

	//! Makes this dialog frameless
	void makeFrameless();

	//! Returns matrix corresponding to dialog values
	ccGLMatrixd getMatrix();

	//inherited from ccOverlayDialog
	bool start() override;
	bool linkWith(ccGLWindow* win) override;

	//inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

public slots:

	//! Links this dialog with a given sub-window
	void linkWith(QMdiSubWindow* qWin);

	//! Inits dialog values with matrix
	void initWithMatrix(const ccGLMatrixd& mat);

	//! Updates dialog values with pivot point
	void updatePivotPoint(const CCVector3d& P);
	//! Updates dialog values with camera center
	void updateCameraCenter(const CCVector3d& P);
	//! Updates current view mode
	void updateViewMode();
	//! Updates view f.o.v.
	void updateWinFov(float fov_deg);
	//! Update the zNear coef.
	void updateZNearCoef(float zNearCoef);

	void setFrontView();
	void setBottomView();
	void setTopView();
	void setBackView();
	void setLeftView();
	void setRightView();
	void setIso1View();
	void setIso2View();

	void iThetaValueChanged(int);
	void iPsiValueChanged(int);
	void iPhiValueChanged(int);

	void dThetaValueChanged(double);
	void dPsiValueChanged(double);
	void dPhiValueChanged(double);

	void zNearSliderMoved(int);
	void pivotChanged();
	void cameraCenterChanged();
	void fovChanged(double);

	void pickPointAsPivot(bool);
	void processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&, const CCVector3d&);

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

	//! Inits dialog values with specified window
	void initWith(ccGLWindow* win);

	//! Type of the pushed matrices map structure
	using PushedMatricesMapType = std::map<ccGLWindow*,ccGLMatrixd>;
	//! Type of an element of the pushed matrices map structure
	using PushedMatricesMapElement = std::pair<ccGLWindow*,ccGLMatrixd>;

	//! Pushed camera matrices (per window)
	PushedMatricesMapType pushedMatrices;

	//! Picking hub
	ccPickingHub* m_pickingHub;
	
private:
	Ui::CameraParamDlg* m_ui;
};

#endif //CC_CAMERA_PARAM_EDIT_DLG_HEADER
