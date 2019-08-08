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
//#                       COPYRIGHT: SAGE INGENIERIE                       #
//#                                                                        #
//##########################################################################

#include "bdrPlaneEditorDlg.h"

//local
#include "mainwindow.h"

//common
#include <ccPickingHub.h>

//qCC_db
#include <ccPlane.h>
#include <ccFacet.h>
#include <ccNormalVectors.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QDoubleValidator>

//semi-persistent parameters
static double s_dip = 0;
static double s_dipDir = 0;
static double s_width = 10.0;
static double s_height = 10.0;
static bool s_upward = true;
static CCVector3d s_center(0, 0, 0);

bdrPlaneEditorDlg::bdrPlaneEditorDlg(ccPickingHub* pickingHub, QWidget* parent)
	: QDialog(parent)
	, Ui::BDRPlaneEditorDlg()
	, m_pickingWin(0)
	, m_associatedPlane(0)
	, m_pickingHub(pickingHub)
{
	assert(pickingHub);

	setModal(false);
	setupUi(this);

	//restore semi-persistent parameters
	dipDoubleSpinBox->setValue(s_dip);
	dipDirDoubleSpinBox->setValue(s_dipDir);
	upwardCheckBox->setChecked(s_upward);
	onDipDirChanged(0); //0 = fake argument
	wDoubleSpinBox->setValue(s_width);
	hDoubleSpinBox->setValue(s_height);
	cxAxisDoubleSpinBox->setValue(s_center.x);
	cyAxisDoubleSpinBox->setValue(s_center.y);
	czAxisDoubleSpinBox->setValue(s_center.z);

	connect(pickCenterToolButton,	SIGNAL(toggled(bool)),			this, SLOT(pickPointAsCenter(bool)));
	connect(dipDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onDipDirChanged(double)));
	connect(dipDirDoubleSpinBox,	SIGNAL(valueChanged(double)),	this, SLOT(onDipDirChanged(double)));
	connect(upwardCheckBox,			SIGNAL(toggled(bool)),			this, SLOT(onDipDirModified(bool)));
	connect(nxDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onNormalChanged(double)));
	connect(nyDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onNormalChanged(double)));
	connect(nzDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onNormalChanged(double)));

	connect(previewCheckBox, &QAbstractButton::clicked, this, &bdrPlaneEditorDlg::preview);
	connect(restoreToolButton, &QAbstractButton::clicked, this, &bdrPlaneEditorDlg::restore);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveParamsAndAccept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(cancle()));
	
	//auto disable picking mode on quit
	connect(this, &QDialog::finished, [&]()
	{
		if (pickCenterToolButton->isChecked()) pickCenterToolButton->setChecked(false); }
	);
}

bdrPlaneEditorDlg::~bdrPlaneEditorDlg()
{
	assert(!pickCenterToolButton->isChecked());
}

void bdrPlaneEditorDlg::updateParams()
{
	if (m_associatedPlane) {
		updatePlane(m_associatedPlane);
		if (MainWindow::TheInstance())
			MainWindow::TheInstance()->updatePropertiesView();

		m_associatedPlane->getPlane()->redrawDisplay();
	}
}

void bdrPlaneEditorDlg::saveParamsAndAccept()
{
	//save semi-persistent parameters
	if (!m_associatedPlane)
	{
		s_dip = dipDoubleSpinBox->value();
		s_dipDir = dipDirDoubleSpinBox->value();
		s_upward = upwardCheckBox->isChecked();
		s_width = wDoubleSpinBox->value();
		s_height = hDoubleSpinBox->value();
		s_center.x = cxAxisDoubleSpinBox->value();
		s_center.y = cyAxisDoubleSpinBox->value();
		s_center.z = czAxisDoubleSpinBox->value();
	}

	//edition mode
	if (m_associatedPlane)
	{
		updateParams();
	}
	else //creation
	{
		ccPlane* plane = new ccPlane();
		updatePlane(plane);
		if (m_pickingWin)
		{
			plane->setDisplay(m_pickingWin);
		}
		if (MainWindow::TheInstance())
		{
			MainWindow::TheInstance()->addToDB(plane, MainWindow::TheInstance()->getCurrentDB());
		}
		else
		{
			delete plane;
			plane = nullptr;
		}
	}

	disconnectPlane();
	accept();

//	deleteLater();
}

void bdrPlaneEditorDlg::onDipDirModified(bool)
{
	onDipDirChanged(0); //0 = fake argument
}

void bdrPlaneEditorDlg::onDipDirChanged(double)
{
	double dip = dipDoubleSpinBox->value();
	double dipDir = dipDirDoubleSpinBox->value();
	bool upward = upwardCheckBox->isChecked();
	CCVector3 Nd = ccNormalVectors::ConvertDipAndDipDirToNormal(static_cast<PointCoordinateType>(dip), static_cast<PointCoordinateType>(dipDir), upward);

	nxDoubleSpinBox->blockSignals(true);
	nyDoubleSpinBox->blockSignals(true);
	nzDoubleSpinBox->blockSignals(true);
	
	nxDoubleSpinBox->setValue(Nd.x);
	nyDoubleSpinBox->setValue(Nd.y);
	nzDoubleSpinBox->setValue(Nd.z);
	
	nxDoubleSpinBox->blockSignals(false);
	nyDoubleSpinBox->blockSignals(false);
	nzDoubleSpinBox->blockSignals(false);

	if (previewCheckBox->isChecked()) {
		updateParams();
	}	
}

void bdrPlaneEditorDlg::onNormalChanged(double)
{
	CCVector3 Nd;
	Nd.x = nxDoubleSpinBox->value();
	Nd.y = nyDoubleSpinBox->value();
	Nd.z = nzDoubleSpinBox->value();
	Nd.normalize();

	PointCoordinateType dip = 0, dipDir = 0;
	ccNormalVectors::ConvertNormalToDipAndDipDir(Nd, dip, dipDir);

	dipDoubleSpinBox->blockSignals(true);
	dipDirDoubleSpinBox->blockSignals(true);
	upwardCheckBox->blockSignals(true);
	
	dipDoubleSpinBox->setValue(dip);
	dipDirDoubleSpinBox->setValue(dipDir);
	upwardCheckBox->setChecked(Nd.z >= 0);
	
	dipDoubleSpinBox->blockSignals(false);
	dipDirDoubleSpinBox->blockSignals(false);
	upwardCheckBox->blockSignals(false);

	if (previewCheckBox->isChecked()) {
		updateParams();
	}
}

void bdrPlaneEditorDlg::preview()
{
	if (previewCheckBox->isChecked()) {
		updateParams();
	}
	else {
		restore();
	}
}

void bdrPlaneEditorDlg::restore()
{
	if (!m_associatedPlane) {
		return;
	}
	CCVector3 Nd = m_planePara.normal;
	CCVector3 Cd = m_planePara.center;
	PointCoordinateType width = m_planePara.size.x;
	PointCoordinateType height = m_planePara.size.y;

	double dip = 0;
	double dipDir = 0;

	CCVector3 N = m_associatedPlane->getNormal();
	CCVector3 C = m_associatedPlane->getCenter();

	//shall we transform (translate and / or rotate) the plane?
	ccGLMatrix trans;
	bool needToApplyTrans = false;
	bool needToApplyRot = false;

	needToApplyRot = (fabs(N.dot(Nd) - PC_ONE) > std::numeric_limits<PointCoordinateType>::epsilon());
	needToApplyTrans = needToApplyRot || ((C - Cd).norm2d() != 0);

	if (needToApplyTrans)
	{
		trans.setTranslation(-C);
		needToApplyTrans = true;
	}
	if (needToApplyRot)
	{
		ccGLMatrix rotation;
		//special case: plane parallel to XY
		if (fabs(N.z) > PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon())
		{
			ccGLMatrix rotX; rotX.initFromParameters(-dip * CC_DEG_TO_RAD, CCVector3(1, 0, 0), CCVector3(0, 0, 0)); //plunge
			ccGLMatrix rotZ; rotZ.initFromParameters(dipDir * CC_DEG_TO_RAD, CCVector3(0, 0, -1), CCVector3(0, 0, 0));
			rotation = rotZ * rotX;
		}
		else //general case
		{
			rotation = ccGLMatrix::FromToRotation(N, Nd);
		}
		trans = rotation * trans;
	}
	if (needToApplyTrans)
	{
		trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
	}
	if (needToApplyRot || needToApplyTrans)
	{
		m_associatedPlane->getPlane()->applyGLTransformation_recursive(&trans);

		ccLog::Print("[Plane edit] Applied transformation matrix:");
		ccLog::Print(trans.toString(12, ' ')); //full precision
	}

	if (m_associatedPlane->getPlane()->isA(CC_TYPES::PLANE)) {
		ccPlane* plane = static_cast<ccPlane*>(m_associatedPlane);
		if (plane->getXWidth() != width
			|| plane->getYWidth() != height) {
			plane->setXWidth(width, false);
			plane->setYWidth(height, true);
		}
	}

	nxDoubleSpinBox->setValue(m_planePara.normal.x);
	nyDoubleSpinBox->setValue(m_planePara.normal.y);
	nzDoubleSpinBox->setValue(m_planePara.normal.z);
	
	wDoubleSpinBox->setValue(m_planePara.size.x);
	hDoubleSpinBox->setValue(m_planePara.size.y);

	cxAxisDoubleSpinBox->setValue(m_planePara.center.x);
	cyAxisDoubleSpinBox->setValue(m_planePara.center.y);
	czAxisDoubleSpinBox->setValue(m_planePara.center.z);

	if (MainWindow::TheInstance())
		MainWindow::TheInstance()->updatePropertiesView();
	m_associatedPlane->getPlane()->redrawDisplay();
}

void bdrPlaneEditorDlg::cancle()
{
	restore();
	disconnectPlane();
	
	//deleteLater();
}

void bdrPlaneEditorDlg::pickPointAsCenter(bool state)
{
	if (!m_pickingHub)
	{
		return;
	}
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

	pickCenterToolButton->blockSignals(true);
	pickCenterToolButton->setChecked(state);
	pickCenterToolButton->blockSignals(false);
}

void bdrPlaneEditorDlg::onItemPicked(const PickedItem& pi)
{
	if (!pi.entity)
	{
		return;
	}

	m_pickingWin = m_pickingHub->activeWindow();

	cxAxisDoubleSpinBox->setValue(pi.P3D.x);
	cyAxisDoubleSpinBox->setValue(pi.P3D.y);
	czAxisDoubleSpinBox->setValue(pi.P3D.z);

	pickCenterToolButton->setChecked(false);

	if (previewCheckBox->isChecked()) {
		updateParams();
	}
}

void bdrPlaneEditorDlg::initWithPlane(ccPlanarEntityInterface* plane)
{
	if (!plane) { assert(false); return; }

	if (m_associatedPlane != plane) {
		if (m_associatedPlane) {
			disconnectPlane();
		}
		m_associatedPlane = plane;
		m_associatedPlane->normalEditState(true);
		connect(m_associatedPlane, SIGNAL(planarEntityChanged()), this, SLOT(updateUI()));
	}
	
	CCVector3 N = plane->getNormal();

	//init the dialog
	nxDoubleSpinBox->blockSignals(true);
	nyDoubleSpinBox->blockSignals(true);
	nzDoubleSpinBox->blockSignals(true);

	nxDoubleSpinBox->setValue(N.x);
	nyDoubleSpinBox->setValue(N.y);
	nzDoubleSpinBox->setValue(N.z);

	nxDoubleSpinBox->blockSignals(false);
	nyDoubleSpinBox->blockSignals(false);
	nzDoubleSpinBox->blockSignals(false);

	onNormalChanged(0);
	//PointCoordinateType dip = 0, dipDir = 0;
	//ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

	//dipDoubleSpinBox->setValue(dip);
	//dipDirDoubleSpinBox->setValue(dipDir);
	//upwardCheckBox->setChecked(N.z >= 0);

	ccPlane* plane_ = static_cast<ccPlane*>(plane);
	if (plane_) {
		wDoubleSpinBox->setValue(plane_->getXWidth());
		hDoubleSpinBox->setValue(plane_->getYWidth());
		m_planePara.size = CCVector2(plane_->getXWidth(), plane_->getYWidth());
	}
	
	
	CCVector3 C = plane->getCenter();
	cxAxisDoubleSpinBox->setValue(C.x);
	cyAxisDoubleSpinBox->setValue(C.y);
	czAxisDoubleSpinBox->setValue(C.z);

	m_planePara.normal = N;
	m_planePara.center = C;
	//m_planePara.size = CCVector2(plane->getXWidth(), plane->getYWidth());
}

void bdrPlaneEditorDlg::disconnectPlane()
{
	if (m_associatedPlane) {
		m_associatedPlane->normalEditState(false);	// restore old plane edit state
		disconnect(m_associatedPlane, SIGNAL(planarEntityChanged), this, SLOT(updateUI));
		m_associatedPlane = nullptr;
	}
}

void bdrPlaneEditorDlg::updateUI()
{
	if (m_associatedPlane) {
		initWithPlane(m_associatedPlane);

		if (MainWindow::TheInstance())
			MainWindow::TheInstance()->updatePropertiesView();
	}
}

void bdrPlaneEditorDlg::closeEvent(QCloseEvent *)
{
	disconnectPlane();
}

void bdrPlaneEditorDlg::updatePlane(ccPlanarEntityInterface* plane)
{
	if (!plane)
	{
		assert(false);
		return;
	}
	
	double dip = dipDoubleSpinBox->value();
	double dipDir = dipDirDoubleSpinBox->value();
	bool upward = upwardCheckBox->isChecked();
	PointCoordinateType width  = static_cast<PointCoordinateType>(wDoubleSpinBox->value());
	PointCoordinateType height = static_cast<PointCoordinateType>(hDoubleSpinBox->value());
	CCVector3 Nd = ccNormalVectors::ConvertDipAndDipDirToNormal(dip, dipDir, upward);
	CCVector3 Cd = {	static_cast<PointCoordinateType>(cxAxisDoubleSpinBox->value()),
						static_cast<PointCoordinateType>(cyAxisDoubleSpinBox->value()),
						static_cast<PointCoordinateType>(czAxisDoubleSpinBox->value()) };
	
	CCVector3 N = plane->getNormal();
	CCVector3 C = plane->getCenter();

	//shall we transform (translate and / or rotate) the plane?
	ccGLMatrix trans;
	bool needToApplyTrans = false;
	bool needToApplyRot = false;

	needToApplyRot = (fabs(N.dot(Nd) - PC_ONE) > std::numeric_limits<PointCoordinateType>::epsilon());
	needToApplyTrans = needToApplyRot || ((C - Cd).norm2d() != 0);

	if (needToApplyTrans)
	{
		trans.setTranslation(-C);
		needToApplyTrans = true;
	}
	if (needToApplyRot)
	{
		ccGLMatrix rotation;
		//special case: plane parallel to XY
		if (fabs(N.z) > PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon())
		{
			ccGLMatrix rotX; rotX.initFromParameters(-dip * CC_DEG_TO_RAD, CCVector3(1, 0, 0), CCVector3(0, 0, 0)); //plunge
			ccGLMatrix rotZ; rotZ.initFromParameters(dipDir * CC_DEG_TO_RAD, CCVector3(0, 0, -1), CCVector3(0, 0, 0));
			rotation = rotZ * rotX;
		}
		else //general case
		{
			rotation = ccGLMatrix::FromToRotation(N, Nd);
		}
		trans = rotation * trans;
	}
	if (needToApplyTrans)
	{
		trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
	}
	
	if (needToApplyRot || needToApplyTrans)
	{
		m_associatedPlane->getPlane()->applyGLTransformation_recursive(&trans);

		ccLog::Print("[Plane edit] Applied transformation matrix:");
		ccLog::Print(trans.toString(12, ' ')); //full precision
	}

	if (plane->getPlane()->isA(CC_TYPES::PLANE)) {
		ccPlane* plane_ = static_cast<ccPlane*> (plane->getPlane());
		if (plane_) {
			if (plane_->getXWidth() != width
				|| plane_->getYWidth() != height)
			{
				plane_->setXWidth(width, false);
				plane_->setYWidth(height, true);
			}
		}
	}
}
