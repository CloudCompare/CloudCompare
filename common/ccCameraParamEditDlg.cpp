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

#include "ccCameraParamEditDlg.h"

//Local
#include "ccPickingHub.h"

//qCC_db
#include <ccGLUtils.h>
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>

//qCC_gl
#include <ccGLWidget.h>
#include <ccGLWindow.h>

//CCLib
#include <CCConst.h>
#include <GenericTriangle.h>

//Qt
#include <QDoubleValidator>
#include <QMdiSubWindow>
#include <QtMath>

ccCameraParamEditDlg::ccCameraParamEditDlg(QWidget* parent, ccPickingHub* pickingHub)
	: ccOverlayDialog(parent, pickingHub ? Qt::FramelessWindowHint | Qt::Tool : Qt::Tool) //pickingHub = CloudCompare / otherwise = ccViewer
	, Ui::CameraParamDlg()
	, m_pickingHub(pickingHub)
{
	setupUi(this);

	connect(phiSlider,		&QAbstractSlider::valueChanged,	this,	&ccCameraParamEditDlg::iPhiValueChanged);
	connect(thetaSlider,	&QAbstractSlider::valueChanged,	this,	&ccCameraParamEditDlg::iThetaValueChanged);
	connect(psiSlider,		&QAbstractSlider::valueChanged,	this,	&ccCameraParamEditDlg::iPsiValueChanged);
	
	connect(phiSpinBox,		static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::dPhiValueChanged);
	connect(thetaSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::dThetaValueChanged);
	connect(psiSpinBox,		static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::dPsiValueChanged);

	//rotation center
	connect(rcxDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::pivotChanged);
	connect(rcyDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::pivotChanged);
	connect(rczDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::pivotChanged);

	//camera center
	connect(exDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::cameraCenterChanged);
	connect(eyDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::cameraCenterChanged);
	connect(ezDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::cameraCenterChanged);

	connect(fovDoubleSpinBox,		static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::fovChanged);
	connect(zNearHorizontalSlider,	&QAbstractSlider::sliderMoved,	this,	&ccCameraParamEditDlg::zNearSliderMoved);

	connect(viewUpToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setTopView);
	connect(viewDownToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setBottomView);
	connect(viewFrontToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setFrontView);
	connect(viewBackToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setBackView);
	connect(viewLeftToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setLeftView);
	connect(viewRightToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setRightView);
	connect(viewIso1ToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setIso1View);
	connect(viewIso2ToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setIso2View);

	connect(pushMatrixToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::pushCurrentMatrix);
	connect(revertMatrixToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::revertToPushedMatrix);

	connect(pivotPickingToolButton,	&QAbstractButton::toggled,	this,	&ccCameraParamEditDlg::pickPointAsPivot);
}

void ccCameraParamEditDlg::makeFrameless()
{
	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
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
	thetaSlider->setValue(qFloor(val * 10.0));
	thetaSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::dPsiValueChanged(double val)
{
	psiSlider->blockSignals(true);
	psiSlider->setValue(qFloor(val * 10.0));
	psiSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::dPhiValueChanged(double val)
{
	phiSlider->blockSignals(true);
	phiSlider->setValue(qFloor(val * 10.0));
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

	double zNearCoef = ccViewportParameters::IncrementToZNearCoef(i, zNearHorizontalSlider->maximum() + 1);
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

void ccCameraParamEditDlg::pickPointAsPivot(bool state)
{
	if (m_pickingHub)
	{
		if (state)
		{
			if (!m_pickingHub->addListener(this, true))
			{
				ccLog::Error("Can't start the picking process (another tool is using it)");
				state = false;
			}
		}
		else
		{
			m_pickingHub->removeListener(this);
		}
	}
	else if (m_associatedWin)
	{
		if (state)
		{
			m_associatedWin->setPickingMode(ccGLWindow::POINT_OR_TRIANGLE_PICKING);
			connect(m_associatedWin, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&)), this, SLOT(processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&)));
		}
		else
		{
			m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
			disconnect(m_associatedWin, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&)), this, SLOT(processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&)));
		}
	}

	pivotPickingToolButton->blockSignals(true);
	pivotPickingToolButton->setChecked(state);
	pivotPickingToolButton->blockSignals(false);
}

void ccCameraParamEditDlg::onItemPicked(const PickedItem& pi)
{
	//with picking hub (CloudCompare)
	if (!m_associatedWin || !m_pickingHub)
	{
		assert(false);
		return;
	}

	if (m_associatedWin != m_pickingHub->activeWindow())
	{
		assert(false);
		ccLog::Warning("Point has been picked in the wrong window");
		return;
	}

	m_associatedWin->setPivotPoint(CCVector3d::fromArray(pi.P3D.u));
	m_associatedWin->redraw();

	pickPointAsPivot(false);
}

void ccCameraParamEditDlg::processPickedItem(ccHObject* entity, unsigned, int, int, const CCVector3& P)
{
	//without picking hub (ccViewer)
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

	pickPointAsPivot(false);
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
	ccGLWindow* associatedWin = (qWin ? GLWindowFromWidget(qWin->widget()) : nullptr);

	linkWith(associatedWin);
}

bool ccCameraParamEditDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (oldWin != m_associatedWin && pivotPickingToolButton->isChecked())
	{
		//automatically disable picking mode when changing th
		pickPointAsPivot(false);
	}

	if (oldWin)
	{
		oldWin->disconnect(this);
	}

	if (m_associatedWin)
	{
		initWith(m_associatedWin);
		connect(m_associatedWin,	&ccGLWindow::baseViewMatChanged,		this,	&ccCameraParamEditDlg::initWithMatrix);

		connect(m_associatedWin,	&ccGLWindow::cameraPosChanged,			this,	&ccCameraParamEditDlg::updateCameraCenter);
		connect(m_associatedWin,	&ccGLWindow::pivotPointChanged,			this,	&ccCameraParamEditDlg::updatePivotPoint);
		connect(m_associatedWin,	&ccGLWindow::perspectiveStateChanged,	this,	&ccCameraParamEditDlg::updateViewMode);
		connect(m_associatedWin,	&QObject::destroyed,					this,	&QWidget::hide);
		connect(m_associatedWin,	&ccGLWindow::fovChanged,				this,	&ccCameraParamEditDlg::updateWinFov);
		connect(m_associatedWin,	&ccGLWindow::zNearCoefChanged,			this,	&ccCameraParamEditDlg::updateZNearCoef);

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
	m_associatedWin = nullptr;

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
	setEnabled(win != nullptr);
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
	updateZNearCoef(params.zNearCoef);
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

void ccCameraParamEditDlg::updateZNearCoef(float zNearCoef)
{
	if (!m_associatedWin)
		return;

	zNearHorizontalSlider->blockSignals(true);
	zNearHorizontalSlider->setValue(ccViewportParameters::ZNearCoefToIncrement(zNearCoef, zNearHorizontalSlider->maximum() + 1));
	zNearHorizontalSlider->blockSignals(false);
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
