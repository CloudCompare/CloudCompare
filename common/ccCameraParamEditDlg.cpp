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
#include "ui_cameraParamDlg.h"

//Local
#include "ccPickingHub.h"

//qCC_db
#include <ccGLUtils.h>
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>

//qCC_gl
#include <ccGLWidget.h>

//CCLib
#include <CCConst.h>
#include <GenericTriangle.h>

//Qt
#include <QMdiSubWindow>
#include <QtMath>

ccCameraParamEditDlg::ccCameraParamEditDlg(QWidget* parent, ccPickingHub* pickingHub)
	: ccOverlayDialog(parent, pickingHub ? Qt::FramelessWindowHint | Qt::Tool : Qt::Tool) //pickingHub = CloudCompare / otherwise = ccViewer
	, m_pickingHub(pickingHub)
	, m_ui( new Ui::CameraParamDlg )
{
	m_ui->setupUi(this);

	connect(m_ui->phiSlider,	&QAbstractSlider::valueChanged,	this,	&ccCameraParamEditDlg::iPhiValueChanged);
	connect(m_ui->thetaSlider,	&QAbstractSlider::valueChanged,	this,	&ccCameraParamEditDlg::iThetaValueChanged);
	connect(m_ui->psiSlider,	&QAbstractSlider::valueChanged,	this,	&ccCameraParamEditDlg::iPsiValueChanged);
	
	connect(m_ui->phiSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::dPhiValueChanged);
	connect(m_ui->thetaSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::dThetaValueChanged);
	connect(m_ui->psiSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::dPsiValueChanged);

	//rotation center
	connect(m_ui->rcxDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::pivotChanged);
	connect(m_ui->rcyDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::pivotChanged);
	connect(m_ui->rczDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::pivotChanged);

	//camera center
	connect(m_ui->exDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::cameraCenterChanged);
	connect(m_ui->eyDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::cameraCenterChanged);
	connect(m_ui->ezDoubleSpinBox,	static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::cameraCenterChanged);

	connect(m_ui->fovDoubleSpinBox,			static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this,	&ccCameraParamEditDlg::fovChanged);
	connect(m_ui->zNearHorizontalSlider,	&QAbstractSlider::sliderMoved,	this,	&ccCameraParamEditDlg::zNearSliderMoved);

	connect(m_ui->viewUpToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setTopView);
	connect(m_ui->viewDownToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setBottomView);
	connect(m_ui->viewFrontToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setFrontView);
	connect(m_ui->viewBackToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setBackView);
	connect(m_ui->viewLeftToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setLeftView);
	connect(m_ui->viewRightToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setRightView);
	connect(m_ui->viewIso1ToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setIso1View);
	connect(m_ui->viewIso2ToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::setIso2View);

	connect(m_ui->pushMatrixToolButton,		&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::pushCurrentMatrix);
	connect(m_ui->revertMatrixToolButton,	&QAbstractButton::clicked,	this,	&ccCameraParamEditDlg::revertToPushedMatrix);

	connect(m_ui->pivotPickingToolButton,	&QAbstractButton::toggled,	this,	&ccCameraParamEditDlg::pickPointAsPivot);
}

ccCameraParamEditDlg::~ccCameraParamEditDlg()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccCameraParamEditDlg::makeFrameless()
{
	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
}

void ccCameraParamEditDlg::iThetaValueChanged(int val)
{
	m_ui->thetaSpinBox->blockSignals(true);
	m_ui->thetaSpinBox->setValue(val / 10.0);
	m_ui->thetaSpinBox->blockSignals(false);

	reflectParamChange();
}

void ccCameraParamEditDlg::iPsiValueChanged(int val)
{
	m_ui->psiSpinBox->blockSignals(true);
	m_ui->psiSpinBox->setValue(val / 10.0);
	m_ui->psiSpinBox->blockSignals(false);

	reflectParamChange();
}

void ccCameraParamEditDlg::iPhiValueChanged(int val)
{
	m_ui->phiSpinBox->blockSignals(true);
	m_ui->phiSpinBox->setValue(val / 10.0);
	m_ui->phiSpinBox->blockSignals(false);

	reflectParamChange();
}

void ccCameraParamEditDlg::dThetaValueChanged(double val)
{
	m_ui->thetaSlider->blockSignals(true);
	m_ui->thetaSlider->setValue(qFloor(val * 10.0));
	m_ui->thetaSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::dPsiValueChanged(double val)
{
	m_ui->psiSlider->blockSignals(true);
	m_ui->psiSlider->setValue(qFloor(val * 10.0));
	m_ui->psiSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::dPhiValueChanged(double val)
{
	m_ui->phiSlider->blockSignals(true);
	m_ui->phiSlider->setValue(qFloor(val * 10.0));
	m_ui->phiSlider->blockSignals(false);
	reflectParamChange();
}

void ccCameraParamEditDlg::cameraCenterChanged()
{
	if (!m_associatedWin)
		return;

	m_associatedWin->blockSignals(true);
	m_associatedWin->setCameraPos( CCVector3d(	m_ui->exDoubleSpinBox->value(),
												m_ui->eyDoubleSpinBox->value(),
												m_ui->ezDoubleSpinBox->value() ));
	m_associatedWin->blockSignals(false);

	m_associatedWin->redraw();
}

void ccCameraParamEditDlg::pivotChanged()
{
	if (!m_associatedWin)
		return;

	m_associatedWin->blockSignals(true);
	m_associatedWin->setPivotPoint(
		CCVector3d(	m_ui->rcxDoubleSpinBox->value(),
					m_ui->rcyDoubleSpinBox->value(),
					m_ui->rczDoubleSpinBox->value() ));
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

	double zNearCoef = ccViewportParameters::IncrementToZNearCoef(i, m_ui->zNearHorizontalSlider->maximum() + 1);
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

	m_ui->buttonsFrame->setEnabled(true);
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
			connect(m_associatedWin, &ccGLWindow::itemPicked, this, &ccCameraParamEditDlg::processPickedItem);
		}
		else
		{
			m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
			disconnect(m_associatedWin, &ccGLWindow::itemPicked, this, &ccCameraParamEditDlg::processPickedItem);
		}
	}

	m_ui->pivotPickingToolButton->blockSignals(true);
	m_ui->pivotPickingToolButton->setChecked(state);
	m_ui->pivotPickingToolButton->blockSignals(false);
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

void ccCameraParamEditDlg::processPickedItem(ccHObject* entity, unsigned, int, int, const CCVector3& P, const CCVector3d& uvw)
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

	if (oldWin != m_associatedWin && m_ui->pivotPickingToolButton->isChecked())
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
		m_ui->buttonsFrame->setEnabled(it != pushedMatrices.end());
	}
	else
	{
		hide();
		m_ui->buttonsFrame->setEnabled(false);
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
			m_ui->currentModeLabel->setText("parallel projection");
		else
			m_ui->currentModeLabel->setText(QString(objectBased ? "object" : "viewer") + QString("-based perspective"));

		m_ui->rotationCenterFrame->setEnabled(objectBased);
		m_ui->pivotPickingToolButton->setEnabled(objectBased);
		m_ui->eyePositionFrame->setEnabled(perspective);
	}
}

void ccCameraParamEditDlg::initWithMatrix(const ccGLMatrixd& mat)
{
	double phi = 0;
	double theta = 0;
	double psi = 0;
	CCVector3d trans;
	mat.getParameters(phi,theta,psi,trans);

	//to avoid retro-action
	ccGLWindow* win = m_associatedWin;
	m_associatedWin = nullptr;

	m_ui->phiSpinBox->blockSignals(true);
	m_ui->phiSpinBox->setValue(CC_RAD_TO_DEG*phi);
	dPhiValueChanged(m_ui->phiSpinBox->value());
	m_ui->phiSpinBox->blockSignals(false);
	
	m_ui->psiSpinBox->blockSignals(true);
	m_ui->psiSpinBox->setValue(CC_RAD_TO_DEG*psi);
	dPsiValueChanged(m_ui->psiSpinBox->value());
	m_ui->psiSpinBox->blockSignals(false);

	m_ui->thetaSpinBox->blockSignals(true);
	m_ui->thetaSpinBox->setValue(CC_RAD_TO_DEG*theta);
	dThetaValueChanged(m_ui->thetaSpinBox->value());
	m_ui->thetaSpinBox->blockSignals(false);

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
	m_ui->exDoubleSpinBox->blockSignals(true);
	m_ui->eyDoubleSpinBox->blockSignals(true);
	m_ui->ezDoubleSpinBox->blockSignals(true);
	m_ui->exDoubleSpinBox->setValue(P.x);
	m_ui->eyDoubleSpinBox->setValue(P.y);
	m_ui->ezDoubleSpinBox->setValue(P.z);
	m_ui->exDoubleSpinBox->blockSignals(false);
	m_ui->eyDoubleSpinBox->blockSignals(false);
	m_ui->ezDoubleSpinBox->blockSignals(false);
}

void ccCameraParamEditDlg::updatePivotPoint(const CCVector3d& P)
{
	if (!m_associatedWin)
		return;

	m_ui->rcxDoubleSpinBox->blockSignals(true);
	m_ui->rcyDoubleSpinBox->blockSignals(true);
	m_ui->rczDoubleSpinBox->blockSignals(true);
	m_ui->rcxDoubleSpinBox->setValue(P.x);
	m_ui->rcyDoubleSpinBox->setValue(P.y);
	m_ui->rczDoubleSpinBox->setValue(P.z);
	m_ui->rcxDoubleSpinBox->blockSignals(false);
	m_ui->rcyDoubleSpinBox->blockSignals(false);
	m_ui->rczDoubleSpinBox->blockSignals(false);
}

void ccCameraParamEditDlg::updateWinFov(float fov_deg)
{
	if (!m_associatedWin)
		return;

	m_ui->fovDoubleSpinBox->blockSignals(true);
	m_ui->fovDoubleSpinBox->setValue(fov_deg);
	m_ui->fovDoubleSpinBox->blockSignals(false);
}

void ccCameraParamEditDlg::updateZNearCoef(float zNearCoef)
{
	if (!m_associatedWin)
		return;

	m_ui->zNearHorizontalSlider->blockSignals(true);
	m_ui->zNearHorizontalSlider->setValue(ccViewportParameters::ZNearCoefToIncrement(zNearCoef, m_ui->zNearHorizontalSlider->maximum() + 1));
	m_ui->zNearHorizontalSlider->blockSignals(false);
}

ccGLMatrixd ccCameraParamEditDlg::getMatrix()
{
	double phi = 0;
	double theta = 0;
	double psi = 0;

	phi		= CC_DEG_TO_RAD * m_ui->phiSpinBox->value();
	psi		= CC_DEG_TO_RAD * m_ui->psiSpinBox->value();
	theta	= CC_DEG_TO_RAD * m_ui->thetaSpinBox->value();

	ccGLMatrixd mat;
	CCVector3d T(0,0,0);
	mat.initFromParameters(phi,theta,psi,T);

	return mat;
}
