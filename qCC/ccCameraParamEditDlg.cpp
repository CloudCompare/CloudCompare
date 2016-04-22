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
#include <ccGenericMesh.h>

//qCC_gl
#include <ccGLWidget.h>

//CCLib
#include <CCConst.h>
#include <GenericTriangle.h>

//Qt
#include <QDoubleValidator>
#include <QMdiSubWindow>

double SliderPosToZNearCoef(int i, int iMax)
{
	assert(i >= 0 && i <= iMax);
	return pow(10,-static_cast<double>((iMax-i)*3)/iMax); //between 10^-3 and 1
}

int ZNearCoefToSliderPos(double coef, int iMax)
{
	assert(coef >= 0 && coef <= 1.0);
	int i = static_cast<int>(-(static_cast<double>(iMax)/3) * log10(coef));
	assert(i >= 0 && i <= iMax);
	return iMax-i;
}

ccCameraParamEditDlg::ccCameraParamEditDlg(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::CameraParamDlg()
{
	setupUi(this);

	connect(phiSlider,				SIGNAL(valueChanged(int)),		this,	SLOT(iPhiValueChanged(int)));
	connect(thetaSlider,			SIGNAL(valueChanged(int)),		this,	SLOT(iThetaValueChanged(int)));
	connect(psiSlider,				SIGNAL(valueChanged(int)),		this,	SLOT(iPsiValueChanged(int)));
	connect(phiSpinBox,				SIGNAL(valueChanged(double)),	this,	SLOT(dPhiValueChanged(double)));
	connect(thetaSpinBox,			SIGNAL(valueChanged(double)),	this,	SLOT(dThetaValueChanged(double)));
	connect(psiSpinBox,				SIGNAL(valueChanged(double)),	this,	SLOT(dPsiValueChanged(double)));

	//rotation center
	connect(rcxDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(pivotChanged()));
	connect(rcyDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(pivotChanged()));
	connect(rczDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(pivotChanged()));

	//camera center
	connect(exDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(cameraCenterChanged()));
	connect(eyDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(cameraCenterChanged()));
	connect(ezDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(cameraCenterChanged()));

	connect(fovDoubleSpinBox,		SIGNAL(valueChanged(double)),	this,	SLOT(fovChanged(double)));
	connect(zNearHorizontalSlider,	SIGNAL(sliderMoved(int)),		this,	SLOT(zNearSliderMoved(int)));

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
	thetaSpinBox->setValue(val / 10.0);
	thetaSpinBox->blockSignals(false);

	reflectParamChange();
}

void ccCameraParamEditDlg::iPsiValueChanged(int val)
{
	psiSpinBox->blockSignals(true);
	psiSpinBox->setValue(val / 10.0);
	psiSpinBox->blockSignals(false);

	reflectParamChange();
}

void ccCameraParamEditDlg::iPhiValueChanged(int val)
{
	phiSpinBox->blockSignals(true);
	phiSpinBox->setValue(val / 10.0);
	phiSpinBox->blockSignals(false);

	reflectParamChange();
}

void ccCameraParamEditDlg::dThetaValueChanged(double val)
{
	thetaSlider->blockSignals(true);
	thetaSlider->setValue(val * 10.0);
	thetaSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::dPsiValueChanged(double val)
{
	psiSlider->blockSignals(true);
	psiSlider->setValue(val * 10.0);
	psiSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::dPhiValueChanged(double val)
{
	phiSlider->blockSignals(true);
	phiSlider->setValue(val * 10.0);
	phiSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::cameraCenterChanged()
{
	if (!m_associatedWin)
		return;

	m_associatedWin->blockSignals(true);
	m_associatedWin->setCameraPos( CCVector3d(	exDoubleSpinBox->value(),
												eyDoubleSpinBox->value(),
												ezDoubleSpinBox->value() ));
	m_associatedWin->blockSignals(false);

	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::pivotChanged()
{
	if (!m_associatedWin)
		return;

	m_associatedWin->blockSignals(true);
	m_associatedWin->setPivotPoint(
		CCVector3d(	rcxDoubleSpinBox->value(),
					rcyDoubleSpinBox->value(),
					rczDoubleSpinBox->value() ));
	m_associatedWin->blockSignals(false);

	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::fovChanged(double value)
{
	if (!m_associatedWin)
		return;

	m_associatedWin->setFov(static_cast<float>(value));
	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::zNearSliderMoved(int i)
{
	if (!m_associatedWin)
		return;

	double zNearCoef = SliderPosToZNearCoef(i, zNearHorizontalSlider->maximum());
	m_associatedWin->setZNearCoef(zNearCoef);
	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::pushCurrentMatrix()
{
	if (!m_associatedWin)
		return;

	ccGLMatrixd mat = m_associatedWin->getBaseViewMat();

	std::pair<PushedMatricesMapType::iterator, bool> ret;
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
		m_associatedWin->setPickingMode(ccGLWindow::POINT_OR_TRIANGLE_PICKING);
		connect(m_associatedWin, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&)), this, SLOT(processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&)));
	}
}

void ccCameraParamEditDlg::processPickedItem(ccHObject* entity, unsigned itemIndex, int x, int y, const CCVector3& P)
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}
	
	if (!entity)
	{
		return;
	}

	m_associatedWin->setPivotPoint(CCVector3d::fromArray(P.u));
	m_associatedWin->redraw();

	m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	disconnect(m_associatedWin, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&)), this, SLOT(processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&)));
}

void ccCameraParamEditDlg::setView(CC_VIEW_ORIENTATION orientation)
{
	if (!m_associatedWin)
		return;

	PushedMatricesMapType::iterator it = pushedMatrices.find(m_associatedWin);

	ccGLMatrixd mat = ccGLUtils::GenerateViewMat(orientation) * (it->second);
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
	ccGLWindow* associatedWin = (qWin ? GLWindowFromWidget(qWin->widget()) : 0);

	linkWith(associatedWin);
}

bool ccCameraParamEditDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (oldWin)
	{
		oldWin->disconnect(this);
	}

	if (m_associatedWin)
	{
		initWith(m_associatedWin);
		connect(m_associatedWin,	SIGNAL(baseViewMatChanged(const ccGLMatrixd&)),		this,	SLOT(initWithMatrix(const ccGLMatrixd&)));

		connect(m_associatedWin,	SIGNAL(cameraPosChanged(const CCVector3d&)),		this,	SLOT(updateCameraCenter(const CCVector3d&)));
		connect(m_associatedWin,	SIGNAL(pivotPointChanged(const CCVector3d&)),		this,	SLOT(updatePivotPoint(const CCVector3d&)));
		connect(m_associatedWin,	SIGNAL(perspectiveStateChanged()),					this,	SLOT(updateViewMode()));
		connect(m_associatedWin,	SIGNAL(destroyed(QObject*)),						this,	SLOT(hide()));
		connect(m_associatedWin,	SIGNAL(fovChanged(float)),							this,	SLOT(updateWinFov(float)));

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

	ccGLMatrixd mat = getMatrix();
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
			currentModeLabel->setText(QString(objectBased ? "object" : "viewer") + QString("-based perspective"));

		rotationCenterFrame->setEnabled(objectBased);
		pivotPickingToolButton->setEnabled(objectBased);
		eyePositionFrame->setEnabled(perspective);
	}
}

void ccCameraParamEditDlg::initWithMatrix(const ccGLMatrixd& mat)
{
	double phi=0, theta=0, psi=0;
	CCVector3d trans;
	mat.getParameters(phi,theta,psi,trans);

	//to avoid retro-action
	ccGLWindow* win = m_associatedWin;
	m_associatedWin = 0;

	phiSpinBox->blockSignals(true);
	phiSpinBox->setValue(CC_RAD_TO_DEG*phi);
	dPhiValueChanged(phiSpinBox->value());
	phiSpinBox->blockSignals(false);
	
	psiSpinBox->blockSignals(true);
	psiSpinBox->setValue(CC_RAD_TO_DEG*psi);
	dPsiValueChanged(psiSpinBox->value());
	psiSpinBox->blockSignals(false);

	thetaSpinBox->blockSignals(true);
	thetaSpinBox->setValue(CC_RAD_TO_DEG*theta);
	dThetaValueChanged(thetaSpinBox->value());
	thetaSpinBox->blockSignals(false);

	m_associatedWin = win;
}

void ccCameraParamEditDlg::initWith(ccGLWindow* win)
{
	setEnabled(win != 0);
	if (!win)
		return;

	//update matrix (angles)
	initWithMatrix(win->getBaseViewMat());

	const ccViewportParameters& params = m_associatedWin->getViewportParameters();

	//update view mode
	updateViewMode();

	//update pivot point
	updatePivotPoint(params.pivotPoint);
	//update camera center
	updateCameraCenter(params.cameraCenter);

	//update FOV
	updateWinFov(win->getFov());

	//update zNearCoef
	zNearHorizontalSlider->blockSignals(true);
	zNearHorizontalSlider->setValue(ZNearCoefToSliderPos(params.zNearCoef, zNearHorizontalSlider->maximum()));
	zNearHorizontalSlider->blockSignals(false);
}

void ccCameraParamEditDlg::updateCameraCenter(const CCVector3d& P)
{
	exDoubleSpinBox->blockSignals(true);
	eyDoubleSpinBox->blockSignals(true);
	ezDoubleSpinBox->blockSignals(true);
	exDoubleSpinBox->setValue(P.x);
	eyDoubleSpinBox->setValue(P.y);
	ezDoubleSpinBox->setValue(P.z);
	exDoubleSpinBox->blockSignals(false);
	eyDoubleSpinBox->blockSignals(false);
	ezDoubleSpinBox->blockSignals(false);
}

void ccCameraParamEditDlg::updatePivotPoint(const CCVector3d& P)
{
	if (!m_associatedWin)
		return;

	rcxDoubleSpinBox->blockSignals(true);
	rcyDoubleSpinBox->blockSignals(true);
	rczDoubleSpinBox->blockSignals(true);
	rcxDoubleSpinBox->setValue(P.x);
	rcyDoubleSpinBox->setValue(P.y);
	rczDoubleSpinBox->setValue(P.z);
	rcxDoubleSpinBox->blockSignals(false);
	rcyDoubleSpinBox->blockSignals(false);
	rczDoubleSpinBox->blockSignals(false);
}

void ccCameraParamEditDlg::updateWinFov(float fov_deg)
{
	if (!m_associatedWin)
		return;

	fovDoubleSpinBox->blockSignals(true);
	fovDoubleSpinBox->setValue(fov_deg);
	fovDoubleSpinBox->blockSignals(false);
}

ccGLMatrixd ccCameraParamEditDlg::getMatrix()
{
	double phi = 0, theta = 0, psi = 0;

	phi		= CC_DEG_TO_RAD * phiSpinBox->value();
	psi		= CC_DEG_TO_RAD * psiSpinBox->value();
	theta	= CC_DEG_TO_RAD * thetaSpinBox->value();

	ccGLMatrixd mat;
	CCVector3d T(0,0,0);
	mat.initFromParameters(phi,theta,psi,T);

	return mat;
}
