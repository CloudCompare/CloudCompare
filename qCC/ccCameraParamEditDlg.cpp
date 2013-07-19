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

#include "ccCameraParamEditDlg.h"

//Local
#include "ccPointCloud.h"

//qCC_db
#include <ccGLUtils.h>
#include <ccHObjectCaster.h>

//CCLib
#include <CCConst.h>

//Qt
#include <QDoubleValidator>
#include <QMdiSubWindow>

ccCameraParamEditDlg::ccCameraParamEditDlg(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::CameraParamDlg()
{
    setupUi(this);

    connect(phiSlider,				SIGNAL(valueChanged(int)),		this,   SLOT(iPhiValueChanged(int)));
    connect(thetaSlider,			SIGNAL(valueChanged(int)),		this,   SLOT(iThetaValueChanged(int)));
    connect(psiSlider,				SIGNAL(valueChanged(int)),		this,   SLOT(iPsiValueChanged(int)));
    connect(phiSpinBox,				SIGNAL(valueChanged(double)),	this,   SLOT(dPhiValueChanged(double)));
    connect(thetaSpinBox,			SIGNAL(valueChanged(double)),	this,   SLOT(dThetaValueChanged(double)));
    connect(psiSpinBox,				SIGNAL(valueChanged(double)),	this,   SLOT(dPsiValueChanged(double)));

	//rotation center
    connect(rcxDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,   SLOT(pivotChanged()));
    connect(rcyDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,   SLOT(pivotChanged()));
    connect(rczDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,   SLOT(pivotChanged()));

	//camera center
    connect(exDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,   SLOT(cameraCenterChanged()));
    connect(eyDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,   SLOT(cameraCenterChanged()));
    connect(ezDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,   SLOT(cameraCenterChanged()));

    connect(fovDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(fovChanged(double)));

    connect(viewUpToolButton,		SIGNAL(clicked()),				this,	SLOT(setTopView()));
    connect(viewDownToolButton,		SIGNAL(clicked()),				this,	SLOT(setBottomView()));
    connect(viewFrontToolButton,	SIGNAL(clicked()),				this,	SLOT(setFrontView()));
    connect(viewBackToolButton,		SIGNAL(clicked()),				this,	SLOT(setBackView()));
    connect(viewLeftToolButton,		SIGNAL(clicked()),				this,	SLOT(setLeftView()));
    connect(viewRightToolButton,	SIGNAL(clicked()),				this,	SLOT(setRightView()));
    connect(viewIso1ToolButton,		SIGNAL(clicked()),				this,	SLOT(setIso1View()));
    connect(viewIso2ToolButton,		SIGNAL(clicked()),				this,	SLOT(setIso2View()));

    connect(pushMatrixToolButton,	SIGNAL(clicked()),				this,	SLOT(pushCurrentMatrix()));
    connect(revertMatrixToolButton,	SIGNAL(clicked()),				this,	SLOT(revertToPushedMatrix()));

	connect(pivotPickingToolButton,	SIGNAL(clicked()),				this,	SLOT(pickPointAsPivot()));
}

ccCameraParamEditDlg::~ccCameraParamEditDlg()
{
}

void ccCameraParamEditDlg::makeFrameless()
{
    setWindowFlags(Qt::FramelessWindowHint |Qt::Tool);
}

void ccCameraParamEditDlg::iThetaValueChanged(int val)
{
    thetaSpinBox->blockSignals(true);
    thetaSpinBox->setValue((double)val/10);
    thetaSpinBox->blockSignals(false);

    reflectParamChange();
}

void ccCameraParamEditDlg::iPsiValueChanged(int val)
{
    psiSpinBox->blockSignals(true);
    psiSpinBox->setValue((double)val/10);
    psiSpinBox->blockSignals(false);

    reflectParamChange();
}

void ccCameraParamEditDlg::iPhiValueChanged(int val)
{
    phiSpinBox->blockSignals(true);
    phiSpinBox->setValue((double)val/10);
    phiSpinBox->blockSignals(false);

    reflectParamChange();
}

void ccCameraParamEditDlg::dThetaValueChanged(double val)
{
    thetaSlider->blockSignals(true);
    thetaSlider->setValue((int)(val*10.0));
    thetaSlider->blockSignals(false);
    reflectParamChange();
}

void ccCameraParamEditDlg::dPsiValueChanged(double val)
{
    psiSlider->blockSignals(true);
    psiSlider->setValue((int)(val*10.0));
    psiSlider->blockSignals(false);
    reflectParamChange();
}

void ccCameraParamEditDlg::dPhiValueChanged(double val)
{
    phiSlider->blockSignals(true);
    phiSlider->setValue((int)(val*10.0));
    phiSlider->blockSignals(false);
    reflectParamChange();
}

void ccCameraParamEditDlg::cameraCenterChanged()
{
    if (!m_associatedWin)
        return;

	m_associatedWin->blockSignals(true);
    m_associatedWin->setCameraPos( CCVector3(exDoubleSpinBox->value(),
												eyDoubleSpinBox->value(),
												ezDoubleSpinBox->value()));
	m_associatedWin->blockSignals(false);

	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::pivotChanged()
{
    if (!m_associatedWin)
        return;

	m_associatedWin->blockSignals(true);
    m_associatedWin->setPivotPoint(CCVector3(rcxDoubleSpinBox->value(),
												rcyDoubleSpinBox->value(),
													rczDoubleSpinBox->value()));
	m_associatedWin->blockSignals(false);

	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::fovChanged(double value)
{
    if (!m_associatedWin)
        return;

    m_associatedWin->setFov(value);
    m_associatedWin->redraw();
}

void ccCameraParamEditDlg::pushCurrentMatrix()
{
    if (!m_associatedWin)
        return;

    ccGLMatrix mat = m_associatedWin->getBaseViewMat();

    std::pair<PushedMatricesMapType::iterator,bool> ret;
    ret = pushedMatrices.insert(PushedMatricesMapElement(m_associatedWin,mat));
    if (ret.second == false) //already exists
        ret.first->second = mat;

    buttonsFrame->setEnabled(true);
}

void ccCameraParamEditDlg::revertToPushedMatrix()
{
    PushedMatricesMapType::iterator it = pushedMatrices.find(m_associatedWin);
    if (it == pushedMatrices.end())
        return;

    initWithMatrix(it->second);
    m_associatedWin->blockSignals(true);
    m_associatedWin->setBaseViewMat(it->second);
    m_associatedWin->blockSignals(false);
    m_associatedWin->redraw();
}

void ccCameraParamEditDlg::pickPointAsPivot()
{
	if (m_associatedWin)
	{
		m_associatedWin->setPickingMode(ccGLWindow::POINT_PICKING);
		connect(m_associatedWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));
	}
}

void ccCameraParamEditDlg::processPickedPoint(int cloudUniqueID, unsigned pointIndex, int, int)
{
	if (!m_associatedWin)
		return;

	ccHObject* obj = 0;
	ccHObject* db = m_associatedWin->getSceneDB();
	if (db)
		obj = db->find(cloudUniqueID);
	if (obj && obj->isKindOf(CC_POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
		const CCVector3* P = cloud->getPoint(pointIndex);

		if (P)
		{
			m_associatedWin->setPivotPoint(*P);
			m_associatedWin->redraw();
		}
	}

	m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	disconnect(m_associatedWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));
}

void ccCameraParamEditDlg::setView(CC_VIEW_ORIENTATION orientation)
{
    if (!m_associatedWin)
        return;

    PushedMatricesMapType::iterator it = pushedMatrices.find(m_associatedWin);

    m_associatedWin->makeCurrent();
    ccGLMatrix mat = ccGLUtils::GenerateViewMat(orientation) * (it->second);
    initWithMatrix(mat);
    m_associatedWin->blockSignals(true);
    m_associatedWin->setBaseViewMat(mat);
    m_associatedWin->blockSignals(false);
    m_associatedWin->redraw();
}

void ccCameraParamEditDlg::setTopView()
{
    setView(CC_TOP_VIEW);
}

void ccCameraParamEditDlg::setBottomView()
{
    setView(CC_BOTTOM_VIEW);
}

void ccCameraParamEditDlg::setFrontView()
{
    setView(CC_FRONT_VIEW);
}

void ccCameraParamEditDlg::setBackView()
{
    setView(CC_BACK_VIEW);
}

void ccCameraParamEditDlg::setLeftView()
{
    setView(CC_LEFT_VIEW);
}

void ccCameraParamEditDlg::setRightView()
{
    setView(CC_RIGHT_VIEW);
}

void ccCameraParamEditDlg::setIso1View()
{
    setView(CC_ISO_VIEW_1);
}

void ccCameraParamEditDlg::setIso2View()
{
    setView(CC_ISO_VIEW_2);
}

bool ccCameraParamEditDlg::start()
{
	ccOverlayDialog::start();

	m_processing = false; //no such concept for this dialog! (+ we want to allow dynamic change of associated window)

	return true;
}

void ccCameraParamEditDlg::linkWith(QMdiSubWindow* qWin)
{
    //corresponding ccGLWindow
    ccGLWindow* associatedWin = (qWin ? static_cast<ccGLWindow*>(qWin->widget()) : 0);

	linkWith(associatedWin);
}

bool ccCameraParamEditDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
		return false;

	if (oldWin)
	{
		this->disconnect(oldWin);
	}

    if (m_associatedWin)
    {
        initWithMatrix(m_associatedWin->getBaseViewMat());
        connect(m_associatedWin,	SIGNAL(baseViewMatChanged(const ccGLMatrix&)),	this,	SLOT(initWithMatrix(const ccGLMatrix&)));
        connect(m_associatedWin,	SIGNAL(cameraPosChanged(const CCVector3&)),		this,	SLOT(updateCameraCenter(const CCVector3&)));
        connect(m_associatedWin,	SIGNAL(pivotPointChanged(const CCVector3&)),	this,	SLOT(updatePivotPoint(const CCVector3&)));
        connect(m_associatedWin,	SIGNAL(pivotPointChanged(const CCVector3&)),	this,	SLOT(updatePivotPoint(const CCVector3&)));
        connect(m_associatedWin,	SIGNAL(perspectiveStateChanged()),				this,	SLOT(updateViewMode()));
        connect(m_associatedWin,	SIGNAL(destroyed(QObject*)),					this,	SLOT(hide()));

        PushedMatricesMapType::iterator it = pushedMatrices.find(m_associatedWin);
        buttonsFrame->setEnabled(it != pushedMatrices.end());
    }
    else
    {
        hide();
        buttonsFrame->setEnabled(false);
    }

	return true;
}

void ccCameraParamEditDlg::reflectParamChange()
{
    if (!m_associatedWin)
        return;

    ccGLMatrix mat = getMatrix();
    m_associatedWin->blockSignals(true);
    m_associatedWin->setBaseViewMat(mat);
    m_associatedWin->blockSignals(false);
    m_associatedWin->redraw();
}

void ccCameraParamEditDlg::updateViewMode()
{
	if (m_associatedWin)
	{
		bool objectBased = true;
		bool perspective = m_associatedWin->getPerspectiveState(objectBased);

		if (!perspective)
			currentModeLabel->setText("parallel projection");
		else
			currentModeLabel->setText(QString(objectBased ? "Object" : "Viewer")+QString("-based perspective"));

		rotationCenterFrame->setEnabled(objectBased);
		pivotPickingToolButton->setEnabled(objectBased);
		eyePositionFrame->setEnabled(perspective);
	}

}

void ccCameraParamEditDlg::initWithMatrix(const ccGLMatrix& mat)
{
    PointCoordinateType phi=0.0,theta=0.0,psi=0.0;
    CCVector3 trans;
    mat.getParameters(phi,theta,psi,trans);

    //to prevent retro-action!
    ccGLWindow* win = m_associatedWin;
    m_associatedWin = 0;

    phiSpinBox->setValue(CC_RAD_TO_DEG*phi);
    psiSpinBox->setValue(CC_RAD_TO_DEG*psi);
    thetaSpinBox->setValue(CC_RAD_TO_DEG*theta);

	m_associatedWin = win;

	if (m_associatedWin)
	{
		updatePivotPoint(win->getViewportParameters().pivotPoint);
		updateCameraCenter(win->getViewportParameters().cameraCenter);
	}
}

void ccCameraParamEditDlg::updateCameraCenter(const CCVector3& P)
{
	if (!m_associatedWin)
		return;

    //to prevent retro-action!
    ccGLWindow* win = m_associatedWin;
    m_associatedWin = 0;

	exDoubleSpinBox->setValue(P.x);
	eyDoubleSpinBox->setValue(P.y);
	ezDoubleSpinBox->setValue(P.z);
	
	fovDoubleSpinBox->setValue(win->getViewportParameters().fov);

	m_associatedWin = win;
}

void ccCameraParamEditDlg::updatePivotPoint(const CCVector3& P)
{
	if (!m_associatedWin)
		return;

    //to prevent retro-action!
    ccGLWindow* win = m_associatedWin;
    m_associatedWin = 0;

	rcxDoubleSpinBox->setValue(P.x);
	rcyDoubleSpinBox->setValue(P.y);
	rczDoubleSpinBox->setValue(P.z);
	
	fovDoubleSpinBox->setValue(win->getViewportParameters().fov);

	m_associatedWin = win;
}

ccGLMatrix ccCameraParamEditDlg::getMatrix()
{
    PointCoordinateType phi=0.0,theta=0.0,psi=0.0;
    CCVector3 trans;

    phi     = CC_DEG_TO_RAD * (PointCoordinateType)phiSpinBox->value();
    psi     = CC_DEG_TO_RAD * (PointCoordinateType)psiSpinBox->value();
    theta   = CC_DEG_TO_RAD * (PointCoordinateType)thetaSpinBox->value();

    ccGLMatrix mat;
    mat.initFromParameters(phi,theta,psi,trans);

    return mat;
}
