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

#include "ccPlaneEditDlg.h"

//local
#include "mainwindow.h"
#include "ccPickingHub.h"

//qCC_db
#include <ccPlane.h>
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

ccPlaneEditDlg::ccPlaneEditDlg(ccPickingHub* pickingHub, QWidget* parent)
	: QDialog(parent)
	, Ui::PlaneEditDlg()
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
	wDoubleSpinBox->setValue(s_width);
	hDoubleSpinBox->setValue(s_height);
	cxAxisDoubleSpinBox->setValue(s_center.x);
	cyAxisDoubleSpinBox->setValue(s_center.y);
	czAxisDoubleSpinBox->setValue(s_center.z);

	connect(pickCenterToolButton, SIGNAL(toggled(bool)), this, SLOT(pickPointAsCenter(bool)));
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveParamsAndAccept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(deleteLater()));
	//auto disable picking mode on quit
	connect(this, &QDialog::finished, [&]()
	{
		if (pickCenterToolButton->isChecked()) pickCenterToolButton->setChecked(false); }
	);
}

ccPlaneEditDlg::~ccPlaneEditDlg()
{
	assert(!pickCenterToolButton->isChecked());
}

void ccPlaneEditDlg::saveParamsAndAccept()
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
		updatePlane(m_associatedPlane);
		if (MainWindow::TheInstance())
			MainWindow::TheInstance()->updatePropertiesView();
		m_associatedPlane->redrawDisplay();
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
			MainWindow::TheInstance()->addToDB(plane);
	}

	accept();

	deleteLater();
}

void ccPlaneEditDlg::pickPointAsCenter(bool state)
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

void ccPlaneEditDlg::onItemPicked(const PickedItem& pi)
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
}

void ccPlaneEditDlg::initWithPlane(ccPlane* plane)
{
	m_associatedPlane = plane;
	if (!plane)
	{
		assert(false);
		return;
	}
	
	CCVector3 N = plane->getNormal();
	PointCoordinateType dip = 0, dipDir = 0;
	ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

	dipDoubleSpinBox->setValue(dip);
	dipDirDoubleSpinBox->setValue(dipDir);
	upwardCheckBox->setChecked(N.z >= 0);
	wDoubleSpinBox->setValue(plane->getXWidth());
	hDoubleSpinBox->setValue(plane->getYWidth());
	
	CCVector3 C = plane->getCenter();
	cxAxisDoubleSpinBox->setValue(C.x);
	cyAxisDoubleSpinBox->setValue(C.y);
	czAxisDoubleSpinBox->setValue(C.z);
}

void ccPlaneEditDlg::updatePlane(ccPlane* plane)
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
	if ((C - Cd).norm2d() != 0)
	{
		trans.setTranslation(-C);
		needToApplyTrans = true;
	}
	if (fabs(N.dot(Nd) - PC_ONE) > std::numeric_limits<PointCoordinateType>::epsilon())
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
		needToApplyRot = true;
	}
	if (needToApplyTrans)
	{
		trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
	}
	if (needToApplyRot || needToApplyTrans)
	{
		plane->applyGLTransformation_recursive(&trans);
	}

	if (	plane->getXWidth() != width
		||	plane->getYWidth() != height)
	{
		plane->setXWidth(width, false);
		plane->setYWidth(height, true);
	}
}
