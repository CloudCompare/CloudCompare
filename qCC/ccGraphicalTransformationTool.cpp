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

#include "ccGraphicalTransformationTool.h"
#include "mainwindow.h"

#include <ccGLUtils.h>
#include <ccGLWindowInterface.h>

//qCC_db
#include <ccLog.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <ccPlane.h>
#include <ccCoordinateSystem.h>
#include <ccDBRoot.h>


ccGraphicalTransformationTool::ccGraphicalTransformationTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::GraphicalTransformationDlg()
	, m_toTransform("transformed")
{
	setupUi(this);
	
	rotComboBox->clear();
	rotComboBox->insertItem(0, "XYZ", rotComboBoxItems::XYZ);
	rotComboBox->insertItem(1, "X", rotComboBoxItems::X);
	rotComboBox->insertItem(2, "Y", rotComboBoxItems::Y);
	rotComboBox->insertItem(3, "Z", rotComboBoxItems::Z);
	rotComboBox->insertItem(4, "None", rotComboBoxItems::NONE);
	rotComboBox->setCurrentIndex(rotComboBoxItems::XYZ);
	
	connect(pauseButton,    &QAbstractButton::toggled, this, &ccGraphicalTransformationTool::pause);
	connect(okButton,       &QAbstractButton::clicked, this, &ccGraphicalTransformationTool::apply);
	connect(razButton,	    &QAbstractButton::clicked, this, &ccGraphicalTransformationTool::reset);
	connect(cancelButton,   &QAbstractButton::clicked, this, &ccGraphicalTransformationTool::cancel);

	connect(TxCheckBox,     &QCheckBox::clicked, this, &ccGraphicalTransformationTool::incrementalTranslationToggle);
	connect(TyCheckBox,     &QCheckBox::clicked, this, &ccGraphicalTransformationTool::incrementalTranslationToggle);
	connect(TzCheckBox,     &QCheckBox::clicked, this, &ccGraphicalTransformationTool::incrementalTranslationToggle);

	connect(advPushButton,  &QPushButton::toggled,     this, &ccGraphicalTransformationTool::advModeToggle);
	connect(refAxisRadio,   &QRadioButton::toggled,    this, &ccGraphicalTransformationTool::advRefAxisRadioToggled);
	connect(objCenterRadio, &QRadioButton::toggled,    this, &ccGraphicalTransformationTool::advObjectAxisRadioToggled);
	connect(advTranslateComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccGraphicalTransformationTool::advTranslateRefUpdate);
	connect(advRotateComboBox,    qOverload<int>(&QComboBox::currentIndexChanged), this, &ccGraphicalTransformationTool::advRotateRefUpdate);
	connect(rotComboBox,          qOverload<int>(&QComboBox::activated),           this, &ccGraphicalTransformationTool::advRotateComboBoxUpdate);
	connect(incrementalForwardButton, &QAbstractButton::clicked, this, &ccGraphicalTransformationTool::incrementalTransform);
	connect(incrementalBackwardButton, &QAbstractButton::clicked, this, &ccGraphicalTransformationTool::incrementalTransform);

	//add shortcuts
	addOverriddenShortcut(Qt::Key_Space); //space bar for the "pause" button
	addOverriddenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverriddenShortcut(Qt::Key_Return); //return key for the "ok" button
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccGraphicalTransformationTool::onShortcutTriggered);

	objCenterRadio->setChecked(true);
	advModeToggle(false);
}

ccGraphicalTransformationTool::~ccGraphicalTransformationTool()
{
	clear();
}

void ccGraphicalTransformationTool::onShortcutTriggered(int key)
{
 	switch(key)
	{
	case Qt::Key_Space:
		pauseButton->toggle();
		return;

	case Qt::Key_Return:
		okButton->click();
		return;

	case Qt::Key_Escape:
		cancelButton->click();
		return;

	default:
		//nothing to do
		break;
	}
}

void ccGraphicalTransformationTool::pause(bool state)
{
	if (!m_associatedWin)
	{
		return;
	}

	if (state)
	{
		m_associatedWin->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA);
		m_associatedWin->displayNewMessage("Transformation [PAUSED]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->displayNewMessage("Unpause to transform again", ccGLWindowInterface::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
	}
	else
	{
		m_associatedWin->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_ENTITIES);
		updateDisplayMessage();
	}

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);

	m_associatedWin->redraw(false, false); //we have to redraw the 3D layer for labels!
}

void ccGraphicalTransformationTool::advModeToggle(bool state)
{
	advRotateComboBox->setVisible(state);
	advTranslateComboBox->setVisible(state);
	translateLabel->setVisible(state);
	rotateLabel->setVisible(state);
	rotAxisLabel->setVisible(state);
	objCenterRadio->setVisible(state);
	refAxisRadio->setVisible(state);
	incrementalRotLabel->setVisible(state);
	incrementalRotSpin->setVisible(state);
	incrementalTransLabel->setVisible(state);
	incrementalTransSpin->setVisible(state);
	incrementalForwardButton->setVisible(state);
	incrementalBackwardButton->setVisible(state);
	IncrementalTransformLabel->setVisible(state);

	m_advMode = state;
	int wPrev = this->width();
	if (state)
	{
		this->setGeometry(this->x() + (-wPrev + 250), this->y(), 250, 235);
		if (advTranslateComboBox->currentIndex() != 0)
		{
			TxCheckBox->setEnabled(false);
			TyCheckBox->setEnabled(false);
		}
		advRotateRefUpdate(advRotateComboBox->currentIndex());
		advTranslateRefUpdate(advTranslateComboBox->currentIndex());
	}
	else
	{
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		this->setGeometry(this->x() , this->y(), 0, 0);
		this->adjustSize(); //adjust size will minimize the display height with the dropdowns not visible
		this->setGeometry(this->x() + (wPrev - 250), this->y(), 250, this->height());
		advRotateComboBox->setCurrentIndex(0); //index 0 is always the origin
		advTranslateComboBox->setCurrentIndex(0); //index 0 is always the origin
	}
	//update mini-GUI
	advPushButton->blockSignals(true);
	advPushButton->setChecked(state);
	advPushButton->blockSignals(false);
	updateDisplayMessage();
}

void ccGraphicalTransformationTool::populateAdvModeItems()
{
	advRotateComboBox->clear();
	advTranslateComboBox->clear();
	advRotateComboBox->insertItem(0, "Origin");
	advTranslateComboBox->insertItem(0, "Origin");
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		ccHObject* root = mainWindow->dbRootObject();
		ccHObject::Container polylines;
		ccHObject::Container coordinateSystems;
		if (root)
		{
			root->filterChildren(polylines, true, CC_TYPES::POLY_LINE);
			root->filterChildren(coordinateSystems, true, CC_TYPES::COORDINATESYSTEM);
			root->filterChildren(m_advancedModeObjectList, true, CC_TYPES::PLANE);
		}
		if (!polylines.empty())
		{
			for (size_t i = 0; i < polylines.size(); i++)
			{
				ccPolyline* poly = static_cast<ccPolyline*>(polylines[i]);
				if (poly->size() == 2) //only single segment polylines allowed
				{
					m_advancedModeObjectList.push_back(polylines[i]);
				}
			}
		}
		if (!coordinateSystems.empty())
		{
			for (size_t i = 0; i < coordinateSystems.size(); i++)
			{
				m_advancedModeObjectList.push_back(coordinateSystems[i]);
			}
		}
		if (!m_advancedModeObjectList.empty())
		{
			for (size_t i = 0; i < m_advancedModeObjectList.size(); ++i)
			{
				QString item = QString("%1 (ID=%2)").arg(m_advancedModeObjectList[i]->getName()).arg(m_advancedModeObjectList[i]->getUniqueID());
				advTranslateComboBox->insertItem(static_cast<int>(i) + 1, item, QVariant(m_advancedModeObjectList[i]->getUniqueID()));
				advRotateComboBox->insertItem(static_cast<int>(i) + 1, item, QVariant(m_advancedModeObjectList[i]->getUniqueID()));
			}
		}
	}
}

ccGLMatrixd ccGraphicalTransformationTool::arbitraryVectorTranslation(const CCVector3& vec)
{
	double theta = 0;

	if (CCCoreLib::LessThanEpsilon(std::abs(vec.z)))
	{
		if (CCCoreLib::LessThanEpsilon(std::abs(vec.y)))
		{
			theta = 0;
		}
		else if (vec.y < 0)
		{
			theta = -M_PI_2; //atan of -infinity is -pi/2
		}
		else
		{
			theta = M_PI_2; //atan of +infinity is pi/2
		}
	}
	else
	{
		theta = std::atan(vec.y / vec.z);
		if (vec.y < 0 && vec.z < 0)
		{
			theta = theta - M_PI;
		}
		else if (vec.z < 0 && vec.y > 0)
		{
			theta = M_PI + theta;
		}
	}

	double phiDenominator = std::sqrt((vec.y * vec.y) + (vec.z * vec.z));
	double phi = 0;

	if (CCCoreLib::LessThanEpsilon(phiDenominator))
	{
		if (CCCoreLib::LessThanEpsilon(std::abs(vec.x)))
		{
			phi = 0;
		}
		else if (vec.x < 0)
		{
			phi = -M_PI_2; //atan of -infinity is -pi/2
		}
		else
		{
			phi = M_PI_2; //atan of +infinity is pi/2
		}
	}
	else
	{
		phi = std::atan(vec.x / phiDenominator);
	}

	ccGLMatrixd xRotation = ccGLMatrixd();
	xRotation.setColumn(1, CCVector3d(0, std::cos(theta), -std::sin(theta)));
	xRotation.setColumn(2, CCVector3d(0, std::sin(theta), std::cos(theta)));
	ccGLMatrixd yRotation = ccGLMatrixd();
	yRotation.setColumn(0, CCVector3d(std::cos(phi), 0, -std::sin(phi)));
	yRotation.setColumn(2, CCVector3d(std::sin(phi), 0, std::cos(phi)));

	ccGLMatrixd arbitraryVectorTranslationAdjust = xRotation * yRotation;

	//special case 
	if (CCCoreLib::LessThanEpsilon(std::abs(vec.x)) && CCCoreLib::LessThanEpsilon(std::abs(vec.y)) && vec.z < 0)
	{
		arbitraryVectorTranslationAdjust.scaleRotation(-1);
	}
	return arbitraryVectorTranslationAdjust;
}

ccGLMatrixd ccGraphicalTransformationTool::arbitraryVectorRotation(double angle, const CCVector3d& arbitraryVector)
{
	// advRotationTransform = (cos(theta)*I) + ((1-cos(theta)*u) (X) u) + (sin(theta)*u_skewsym)
	// (X) represents tensor product
	// Reference: Ch 4.7.3 in Geometric Tools for Computer Graphics - P. Schneider & D. Eberly
	double cosTheta = std::cos(angle);
	double sinTheta = std::sin(angle);
	ccGLMatrixd firstTerm = ccGLMatrixd();
	firstTerm.scaleRotation(cosTheta);
	ccGLMatrixd secondTerm = ccGLMatrixd();
	CCVector3d v = (1 - cosTheta) * arbitraryVector;
	CCVector3d w = arbitraryVector;
	secondTerm.setColumn(0, CCVector3d(v[0] * w[0], v[1] * w[0], v[2] * w[0]));
	secondTerm.setColumn(1, CCVector3d(v[0] * w[1], v[1] * w[1], v[2] * w[1]));
	secondTerm.setColumn(2, CCVector3d(v[0] * w[2], v[1] * w[2], v[2] * w[2]));
	ccGLMatrixd thirdTerm = ccGLMatrixd();
	thirdTerm.setColumn(0, CCVector3d(0, arbitraryVector[2], -arbitraryVector[1]));
	thirdTerm.setColumn(1, CCVector3d(-arbitraryVector[2], 0, arbitraryVector[0]));
	thirdTerm.setColumn(2, CCVector3d(arbitraryVector[1], -arbitraryVector[0], 0));
	thirdTerm.scaleRotation(sinTheta);
	ccGLMatrixd advRotationTransform = firstTerm;
	advRotationTransform += secondTerm;
	advRotationTransform.scaleRow(3, .5);
	advRotationTransform += thirdTerm;
	advRotationTransform.scaleRow(3, .5);
	return advRotationTransform;
}

bool ccGraphicalTransformationTool::setAdvTranslationTransform(ccHObject* translateRef) 
{
	if (!m_associatedWin)
	{
		assert(false);
		return false;
	}
	if (translateRef == nullptr)
	{
		return false;
	}
	if (translateRef->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* line = static_cast<ccPolyline*>(translateRef);
		CCVector3 arbitraryVec = *line->getPoint(1) - *line->getPoint(0);
		m_advTranslationTransform = arbitraryVectorTranslation(arbitraryVec);
		TxCheckBox->setChecked(false);
		TyCheckBox->setChecked(false);
		TxCheckBox->setEnabled(false);
		TyCheckBox->setEnabled(false);
		return true;
	}
	else if (translateRef->isA(CC_TYPES::PLANE))
	{
		ccPlane* plane = static_cast<ccPlane*>(translateRef);
		m_advTranslationTransform = ccGLMatrixd(plane->getTransformation().data());
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		return true;
	}
	else if (translateRef->isA(CC_TYPES::COORDINATESYSTEM))
	{
		ccCoordinateSystem* cs = static_cast<ccCoordinateSystem*>(translateRef);
		m_advTranslationTransform = ccGLMatrixd(cs->getTransformation().data());
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		return true;
	}
	else
	{
		advTranslateComboBox->setCurrentIndex(0);
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		return false;
	}
}

bool ccGraphicalTransformationTool::setAdvRotationAxis(ccHObject* rotateRef, rotComboBoxItems selectedAxis)
{
	if (!m_associatedWin || !rotateRef)
	{
		assert(false);
		return false;
	}
	CCVector3d newCenter;
	CCVector3d arbitraryVec;
	if (rotateRef->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* line = static_cast<ccPolyline*>(rotateRef);
		CCVector3d end = *line->getPoint(1);
		CCVector3d start = *line->getPoint(0);
		arbitraryVec = end - start;
		rotComboBox->clear();
		rotComboBox->insertItem(0, "Z", rotComboBoxItems::Z);
		rotComboBox->insertItem(1, "None", rotComboBoxItems::NONE);
		rotComboBox->setCurrentIndex(rotComboBoxItems::Z);
		incrementalRotationToggle(rotComboBoxItems::Z);
		m_advRotationRefObjCenter = (start + end) / 2;
		arbitraryVec.normalize();
	}
	else if (rotateRef->isA(CC_TYPES::PLANE))
	{
		ccPlane* plane = static_cast<ccPlane*>(rotateRef);
		arbitraryVec = plane->getNormal();
		rotComboBox->clear();
		rotComboBox->insertItem(0, "Z", rotComboBoxItems::Z);
		rotComboBox->insertItem(1, "None", rotComboBoxItems::NONE);
		rotComboBox->setCurrentIndex(rotComboBoxItems::Z);
		incrementalRotationToggle(rotComboBoxItems::Z);
		m_advRotationRefObjCenter = plane->getCenter();
	}
	else if (rotateRef->isA(CC_TYPES::COORDINATESYSTEM))
	{
		ccCoordinateSystem* cs = static_cast<ccCoordinateSystem*>(rotateRef);
		switch (selectedAxis)
		{
			case rotComboBoxItems::X:
			{
				arbitraryVec = cs->getYZPlaneNormal();
				break;
			}
			case rotComboBoxItems::Y:
			{
				arbitraryVec = cs->getZXPlaneNormal();
				break;
			}
			case rotComboBoxItems::Z:
			default:
			{
				selectedAxis = rotComboBoxItems::Z;
				arbitraryVec = cs->getXYPlaneNormal();
				break;
			}
		}
		rotComboBox->clear();
		rotComboBox->insertItem(0, "X", rotComboBoxItems::X);
		rotComboBox->insertItem(1, "Y", rotComboBoxItems::Y);
		rotComboBox->insertItem(2, "Z", rotComboBoxItems::Z);
		rotComboBox->insertItem(3, "None", rotComboBoxItems::NONE);	
		rotComboBox->setCurrentIndex(rotComboBoxItems::Z);
		incrementalRotationToggle(rotComboBoxItems::Z);
		m_advRotationRefObjCenter = cs->getOrigin();
	}
	else //Not a supported primitive for rotateRef
	{
		CCVector3d newCenter = m_toTransform.getBB_recursive().getCenter();
		setRotationCenter(newCenter);
		advRotateComboBox->setCurrentIndex(0);
		return false;
	}

	if (rotComboBox->findData(selectedAxis) == -1)
	{
		rotComboBox->setCurrentIndex(rotComboBox->findData(rotComboBoxItems::Z)); // Default to Z axis if passed an invalid axis selection
	}
	else
	{
		rotComboBox->setCurrentIndex(rotComboBox->findData(selectedAxis));
	}

	if (refAxisRadio->isChecked())
	{
		if (m_advRotateRefIsChild)
		{
			newCenter = m_advRotationRefObjCenter;
			m_position.applyRotation(arbitraryVec);
		}
		else
		{
			newCenter = (m_rotation.inverse() * m_advRotationRefObjCenter - m_rotation.inverse() * m_position.getTranslationAsVec3D());
		}
	}
	else if (objCenterRadio->isChecked())
	{
		if (m_advRotateRefIsChild)
		{
			m_position.applyRotation(arbitraryVec);
		}
		newCenter = m_toTransform.getBB_recursive().getCenter();
	}
	m_advRotationAxis = m_rotation.inverse() * arbitraryVec;
	setRotationCenter(newCenter);
	return true;
}

bool ccGraphicalTransformationTool::entityInTransformList(ccHObject* entity)
{
	for (unsigned j = 0; j < m_toTransform.getChildrenNumber(); j++) 
	{
		if (entity == m_toTransform.getChild(j) || m_toTransform.getChild(j)->isAncestorOf(entity))
		{
			return true;
		}
	}
	return false;
}

void ccGraphicalTransformationTool::advTranslateRefUpdate(int index)
{
	if (index == 0 || m_advancedModeObjectList.empty()) // index 0 is always the origin
	{
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		m_advTranslationTransform.toIdentity();
		MainWindow* mainWindow = MainWindow::TheInstance();
		if (mainWindow && m_advTranslateRef != nullptr)
		{
			mainWindow->db()->unselectEntity(m_advTranslateRef);
			m_advTranslateRef = nullptr;
		}
		return;
	}
	int id = advTranslateComboBox->itemData(index).toInt();
	for (size_t i = 0; i < m_advancedModeObjectList.size(); i++)
	{
		if (id == m_advancedModeObjectList[i]->getUniqueID())
		{
			MainWindow* mainWindow = MainWindow::TheInstance();
			if (mainWindow)
			{
				if (m_advTranslateRef != nullptr && m_advTranslateRef != m_advRotateRef) //leave selected if rotate ref
				{
					mainWindow->db()->unselectEntity(m_advTranslateRef);
				}
				m_advTranslateRef = m_advancedModeObjectList[i];
				m_advTranslateRefIsChild = entityInTransformList(m_advTranslateRef);
				if (m_advTranslateRef != m_advRotateRef) // already selected
				{
					mainWindow->db()->selectEntity(m_advTranslateRef, true);
				}
			}
			if (!setAdvTranslationTransform(m_advTranslateRef))
			{
				ccLog::Error("Error calculating adv translation transform, cannot translate along selected item");
				advTranslateComboBox->setCurrentIndex(0);
			}
			return;
		}
	}
	ccLog::Error("Error finding the selected object in DB Tree, cannot translate along selected object");
	advTranslateComboBox->setCurrentIndex(0);
}

void ccGraphicalTransformationTool::advRotateComboBoxUpdate(int index)
{
	rotComboBoxItems selectedAxis = static_cast<rotComboBoxItems>(rotComboBox->itemData(index).toInt());
	incrementalRotationToggle(selectedAxis);
	if (!m_advMode || !m_advRotateRef)
	{
		return;
	}

	if (selectedAxis == rotComboBoxItems::NONE)
	{
		return;
	}
	if (!setAdvRotationAxis(m_advRotateRef, selectedAxis))
	{
		ccLog::Error("Error setting adv rotation axis, cannot rotate around selected item");
		advRotateComboBox->setCurrentIndex(0);
	}

}

void ccGraphicalTransformationTool::advRotateRefUpdate(int index)
{
	if (index == 0 || m_advancedModeObjectList.empty()) // index 0 is always the origin
	{
		if (m_advRotateRef != nullptr)
		{
			rotComboBox->clear();
			rotComboBox->insertItem(0, "XYZ", rotComboBoxItems::XYZ);
			rotComboBox->insertItem(1, "X", rotComboBoxItems::X);
			rotComboBox->insertItem(2, "Y", rotComboBoxItems::Y);
			rotComboBox->insertItem(3, "Z", rotComboBoxItems::Z);
			rotComboBox->insertItem(4, "None", rotComboBoxItems::NONE);
			rotComboBox->setCurrentIndex(rotComboBoxItems::Z);
			incrementalRotationToggle(rotComboBoxItems::Z);
		}
		CCVector3d center = m_toTransform.getBB_recursive().getCenter();
		setRotationCenter(center);
		m_advRotationRefObjCenter = CCVector3d(0, 0, 0);
		m_advRotationAxis = CCVector3d(0, 0, 1);
		objCenterRadio->setEnabled(true);
		objCenterRadio->setChecked(true);
		refAxisRadio->setEnabled(false);
		MainWindow* mainWindow = MainWindow::TheInstance();
		if (mainWindow && m_advRotateRef != nullptr)
		{
			mainWindow->db()->unselectEntity(m_advRotateRef);
			m_advRotateRef = nullptr;
		}
		return;
	}
	int id = advRotateComboBox->itemData(index).toInt();
	for (size_t i = 0; i < m_advancedModeObjectList.size(); i++)
	{
		if (id == m_advancedModeObjectList[i]->getUniqueID())
		{
			MainWindow* mainWindow = MainWindow::TheInstance();
			if (mainWindow)
			{
				if (m_advRotateRef != nullptr && m_advTranslateRef != m_advRotateRef)//leave selected if translate ref
				{
					mainWindow->db()->unselectEntity(m_advRotateRef);
				}
				m_advRotateRef = m_advancedModeObjectList[i];
				if (m_advTranslateRef != m_advRotateRef) // already selected
				{
					mainWindow->db()->selectEntity(m_advRotateRef, true);
				}
				refAxisRadio->setEnabled(true);
				m_advRotateRefIsChild = entityInTransformList(m_advRotateRef);
				if (m_advRotateRefIsChild)
				{
					refAxisRadio->setChecked(true);
					objCenterRadio->setEnabled(false);
				}
				else
				{
					objCenterRadio->setEnabled(true);
				}
			}
			if (!setAdvRotationAxis(m_advRotateRef, static_cast<rotComboBoxItems>(rotComboBox->itemData(index).toInt())))
			{
				ccLog::Error("Error setting adv rotation axis, cannot rotate around selected item");
				advRotateComboBox->setCurrentIndex(0);
			}
			return;
		}
	}
	ccLog::Error("Error finding the selected object in DB Tree, cannot translate along selected object");
	advRotateComboBox->setCurrentIndex(0);
}

void ccGraphicalTransformationTool::advRefAxisRadioToggled(bool state)
{
	if (state)
	{
		objCenterRadio->setChecked(false);
		advRotateRefUpdate(advRotateComboBox->currentIndex()); //force an update
	}
}

void ccGraphicalTransformationTool::incrementalTranslationToggle()
{
	const bool isIncrementalActive = TxCheckBox->isChecked() || TyCheckBox->isChecked() || TzCheckBox->isChecked();
	incrementalTransLabel->setEnabled(isIncrementalActive);
	incrementalTransSpin->setEnabled(isIncrementalActive);
}

void ccGraphicalTransformationTool::incrementalRotationToggle(const rotComboBoxItems & selectedRotationItem)
{
	if(selectedRotationItem == rotComboBoxItems::NONE || selectedRotationItem == rotComboBoxItems::XYZ) {
		incrementalRotLabel->setDisabled(true);
		incrementalRotSpin->setDisabled(true);
	} else {
		incrementalRotLabel->setEnabled(true);
		incrementalRotSpin->setEnabled(true);
	}
}

void ccGraphicalTransformationTool::advObjectAxisRadioToggled(bool state)
{
	if (state)
	{
		refAxisRadio->setChecked(false);
		advRotateRefUpdate(advRotateComboBox->currentIndex()); //force an update
	}
}

void ccGraphicalTransformationTool::clear()
{
	m_toTransform.detachAllChildren();

	m_rotation.toIdentity();
	m_translation = CCVector3d(0,0,0);
	m_rotationCenter = CCVector3d(0,0,0);
}

void ccGraphicalTransformationTool::clearAdvModeEntities()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow && m_advTranslateRef != nullptr && m_associatedWin)
	{
		mainWindow->db()->unselectEntity(m_advTranslateRef);
		m_advTranslateRef = nullptr;
	}
	if (mainWindow && m_advRotateRef != nullptr && m_associatedWin)
	{
		mainWindow->db()->unselectEntity(m_advRotateRef);
		m_advRotateRef = nullptr;
	}
}

bool ccGraphicalTransformationTool::addEntity(ccHObject* entity)
{
	assert(entity);
	if (!entity)
		return false;

	//we don't handle entities associated to another context
	if (entity->getDisplay() != m_associatedWin)
	{
		ccLog::Warning(QString("[Graphical Transformation Tool] Can't use entity '%1' cause it's not displayed in the active 3D view!").arg(entity->getName()));
		return false;
	}

	//we can't transform locked entities
	if (entity->isLocked())
	{
		ccLog::Warning(QString("[Graphical Transformation Tool] Can't transform entity '%1' cause it's locked!").arg(entity->getName()));
		return false;
	}

	//we can't transform child meshes
	if (entity->isA(CC_TYPES::MESH) && entity->getParent() && entity->getParent()->isKindOf(CC_TYPES::MESH))
	{
		ccLog::Warning(QString("[Graphical Transformation Tool] Entity '%1' can't be modified as it is part of a mesh group. You should 'clone' it first.").arg(entity->getName()));
		return false;
	}

	//eventually, we must check that there is no "parent + sibling" in the selection!
	//otherwise, the sibling will be rotated twice!
	unsigned n = m_toTransform.getChildrenNumber();
	for (unsigned i=0; i<n; )
	{
		ccHObject* previous = m_toTransform.getChild(i);
		if (previous->isAncestorOf(entity))
		{
			//we have found a parent, we won't add this entity
			return false;
		}
		//if the inverse is true, then we get rid of the current element!
		else if (entity->isAncestorOf(previous))
		{
			m_toTransform.detachChild(previous);
			--n;
		}
		else
		{
			//proceed
			++i;
		}
	}

	m_toTransform.addChild(entity,ccHObject::DP_NONE);

	return true;
}

unsigned ccGraphicalTransformationTool::getNumberOfValidEntities() const
{
	return m_toTransform.getChildrenNumber();
}

bool ccGraphicalTransformationTool::linkWith(ccGLWindowInterface* win)
{
	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}
	
	assert(!win || m_toTransform.getChildrenNumber() == 0);
	m_toTransform.setDisplay(win);
	
	return true;
}

void ccGraphicalTransformationTool::updateDisplayMessage()
{
	if (!m_associatedWin || pauseButton->isChecked())
	{
		return;
	}
	if (advPushButton->isChecked())
	{
		m_associatedWin->displayNewMessage("[Advanced mode]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->displayNewMessage("[Select Plane or Line to translate along/rotate around]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->displayNewMessage("[If plane selected, rotation will be around normal vector]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
	}
	else
	{
		m_associatedWin->displayNewMessage("[Rotation/Translation mode]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
	}
	m_associatedWin->redraw(true, false);
}

bool ccGraphicalTransformationTool::start()
{
	assert(!m_processing);
	assert(m_associatedWin);
	if (!m_associatedWin)
		return false;

	unsigned childNum = m_toTransform.getChildrenNumber();
	if (childNum == 0)
		return false;

	m_rotation.toIdentity();
	m_translation = CCVector3d(0,0,0);
	m_rotationCenter = m_toTransform.getBB_recursive().getCenter(); //m_rotation center == selected entities center

	//activate "moving mode" in associated GL window
	m_associatedWin->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_ENTITIES);
	m_associatedWin->setPickingMode(ccGLWindowInterface::NO_PICKING, Qt::OpenHandCursor);
	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	connect(m_associatedWin->signalEmitter(), &ccGLWindowSignalEmitter::rotation, this, &ccGraphicalTransformationTool::glRotate);
	connect(m_associatedWin->signalEmitter(), &ccGLWindowSignalEmitter::translation, this, &ccGraphicalTransformationTool::glTranslate);
	m_associatedWin->displayNewMessage(QString(), ccGLWindowInterface::UPPER_CENTER_MESSAGE); //clear the area
	pauseButton->setChecked(false);
	m_advancedModeObjectList.clear();
	populateAdvModeItems();
	advRotateComboBox->setCurrentIndex(0);
	advTranslateComboBox->setCurrentIndex(0);
	updateDisplayMessage();
	m_associatedWin->redraw(true, false);

	return ccOverlayDialog::start();
}

void ccGraphicalTransformationTool::stop(bool state)
{
	if (m_associatedWin)
	{
		//deactivate "moving mode" in associated GL window
		m_associatedWin->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA);
		m_associatedWin->setPickingMode(ccGLWindowInterface::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
		m_associatedWin->signalEmitter()->disconnect(this);
		m_associatedWin->displayNewMessage("[Rotation/Translation mode OFF]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, false, 2, ccGLWindowInterface::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->redraw(false, false); //we have to redraw the 3D layer for labels!
	}

	ccOverlayDialog::stop(state);
}

void ccGraphicalTransformationTool::glTranslate(const CCVector3d& realT)
{
	CCVector3d t;
	if (m_advMode)
	{
		CCVector3d mouseMove = realT;
		m_advTranslationTransform.inverse().applyRotation(mouseMove);
		t = CCVector3d(	mouseMove.x * (TxCheckBox->isChecked() ? 1 : 0),
						mouseMove.y * (TyCheckBox->isChecked() ? 1 : 0),
						mouseMove.z * (TzCheckBox->isChecked() ? 1 : 0));
		m_advTranslationTransform.applyRotation(t);
	}
	else
	{
		t = CCVector3d(	realT.x * (TxCheckBox->isChecked() ? 1 : 0),
						realT.y * (TyCheckBox->isChecked() ? 1 : 0),
						realT.z * (TzCheckBox->isChecked() ? 1 : 0));
	}
	
	if (t.norm2() != 0)
	{
		m_translation += t;
		
		if (m_advMode && m_advRotateRef != nullptr && refAxisRadio->isChecked() && !m_advRotateRefIsChild)
		{
			//If we translate we have to update the rotation center when using the ref axis as center of rotation.
			//Dont update the rotation center if the rotation ref is contained in m_toTranslate.
			CCVector3d centerUpdate = m_rotation.inverse() * m_advRotationRefObjCenter - m_rotation.inverse() * m_position.getTranslationAsVec3D();
			m_translation += (m_rotationCenter - centerUpdate) - m_rotation * (m_rotationCenter - centerUpdate);
			m_rotationCenter = centerUpdate;
		}
		updateAllGLTransformations();
	}
}

void ccGraphicalTransformationTool::glRotate(const ccGLMatrixd& rotMat)
{
	if (m_advMode && m_advRotateRef != nullptr)
	{
		rotComboBoxItems rotAxis = static_cast<rotComboBoxItems>(rotComboBox->itemData(rotComboBox->currentIndex()).toInt());
		double angle = 0;
		switch (rotAxis)
		{
		case rotComboBoxItems::X:
			angle = std::asin(rotMat.xRotation()(1, 2));
			break;
		case rotComboBoxItems::Y:
			angle = std::asin(rotMat.yRotation()(2, 0));
			break;
		case rotComboBoxItems::Z:
			angle = std::asin(rotMat.zRotation()(1, 0));
			break;
		default:
			return;
		}
		ccLog::Print(QString("Angles(%1)").arg(angle));

		m_rotation = m_rotation * arbitraryVectorRotation(angle, m_advRotationAxis);
	}
	else
	{
		switch (rotComboBox->currentIndex())
		{
		case rotComboBoxItems::XYZ:
			m_rotation = rotMat * m_rotation;
			break;
		case rotComboBoxItems::X:
			m_rotation = rotMat.xRotation() * m_rotation;
			break;
		case rotComboBoxItems::Y:
			m_rotation = rotMat.yRotation() * m_rotation;
			break;
		case rotComboBoxItems::Z:
			m_rotation = rotMat.zRotation() * m_rotation;
			break;
		case rotComboBoxItems::NONE:
			break;
		}
	}

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::incrementalTransform() {
	
	//We check the sender to handle forward or backward transformation
	QObject* senderButton = sender();
	const bool isForwardTransform = senderButton == incrementalForwardButton;
	
	// The rotation part:
	// No incremental rotation if rotComboBoxItems == XYZ or rotComboBoxItems == NONE because it has no sense
	// it could be handled in the subsequent switch cases but it is somehiow more explicit to check
	// if the SpinBox is enabled
	if(incrementalRotSpin->isEnabled()) {
		const double alpha = isForwardTransform ? CCCoreLib::DegreesToRadians(incrementalRotSpin->value()) : -CCCoreLib::DegreesToRadians(incrementalRotSpin->value());
		if (m_advMode && m_advRotateRef != nullptr)
		{
			m_rotation = m_rotation * arbitraryVectorRotation(alpha, m_advRotationAxis);
		} 
		else 
		{
			ccGLMatrixd rotMat;
			// compute rotation matrix from spinbox
			switch (static_cast<rotComboBoxItems>(rotComboBox->itemData(rotComboBox->currentIndex()).toInt()))
			{
			case rotComboBoxItems::X:
				rotMat.initFromParameters(alpha, CCVector3d(1.0,0.0,0.0), CCVector3d(0.0,0.0,0.0));
				break;
			case rotComboBoxItems::Y:
				rotMat.initFromParameters(alpha, CCVector3d(0.0,1.0,0.0), CCVector3d(0.0,0.0,0.0));
				break;
			case rotComboBoxItems::Z:
				rotMat.initFromParameters(alpha, CCVector3d(0.0,0.0,1.0), CCVector3d(0.0,0.0,0.0));
				break;
			default:
				return;
			}
			m_rotation = rotMat * m_rotation;
		}
		updateAllGLTransformations();
	}

	// The translation part:
	// we pass a vector with equal magnitude in the three components; gltranslate will handle the correct transform for us.
	const double transMagnitude = isForwardTransform ? incrementalTransSpin->value() : -incrementalTransSpin->value() ;
	glTranslate(CCVector3d(transMagnitude, transMagnitude, transMagnitude));
}

void ccGraphicalTransformationTool::reset()
{
	m_rotation.toIdentity();
	m_translation = CCVector3d(0, 0, 0);

	updateAllGLTransformations();
	advRotateRefUpdate(advRotateComboBox->currentIndex()); //force an update
	advTranslateRefUpdate(advTranslateComboBox->currentIndex()); //force an update
}

void ccGraphicalTransformationTool::setRotationCenter(CCVector3d& center)
{
	m_translation += (m_rotationCenter - center) - m_rotation * (m_rotationCenter - center);
	m_rotationCenter = center;

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::updateAllGLTransformations()
{
	//we recompute global GL transformation matrix
	m_position = m_rotation;
	m_position += m_rotationCenter + m_translation - m_rotation * m_rotationCenter;
 
	ccGLMatrix newTransf(m_position.data());
	for (unsigned i = 0; i < m_toTransform.getChildrenNumber(); ++i)
	{
		ccHObject* child = m_toTransform.getChild(i);
		child->setGLTransformation(newTransf);
		child->prepareDisplayForRefresh_recursive();
	}

	if (m_advTranslateRefIsChild && m_advTranslateRef != nullptr) 
	{
		//update m_advTranslationTransform if the ref object is translated/rotated by virtue of being in m_toTransform
		if (m_advTranslateRef->isA(CC_TYPES::PLANE))
		{
			m_advTranslationTransform = m_position * ccGLMatrixd(m_advTranslateRef->getGLTransformationHistory().data());
		}
		else if (m_advTranslateRef->isA(CC_TYPES::POLY_LINE))
		{
			ccPolyline* line = static_cast<ccPolyline*>(m_advTranslateRef);
			CCVector3 arbitraryVec = line->getGLTransformation() * (*line->getPoint(1) - *line->getPoint(0));
			m_advTranslationTransform = m_position * arbitraryVectorTranslation(arbitraryVec);
		}
	}
	
	MainWindow::RefreshAllGLWindow(false);
}

void ccGraphicalTransformationTool::apply()
{
	//we recompute global GL transformation matrix and display it in console
	ccGLMatrixd finalTrans = m_rotation;
	finalTrans += m_rotationCenter + m_translation - m_rotation * m_rotationCenter;

	ccGLMatrixd finalTransCorrected = finalTrans;
#define NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER
#ifdef NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER
	{
		//convert matrix back and forth so as to be sure to get a 'true' rotation matrix
		//DGM: we use Euler angles, as the axis/angle method (formerly used) is not robust
		//enough! Shifts could be perceived by the user.
		double phi_rad = 0.0;
		double theta_rad = 0.0;
		double psi_rad = 0.0;
		CCVector3d t3D;
		finalTrans.getParameters(phi_rad,theta_rad,psi_rad,t3D);
		finalTransCorrected.initFromParameters(phi_rad,theta_rad,psi_rad,t3D);

#ifdef QT_DEBUG
		ccLog::Print("[GraphicalTransformationTool] Final transformation (before correction):");
		ccLog::Print(finalTrans.toString(12,' ')); //full precision
		ccLog::Print(QString("Angles(%1,%2,%3) T(%5,%6,%7)").arg(phi_rad).arg(theta_rad).arg(psi_rad).arg(t3D.x).arg(t3D.y).arg(t3D.z));
#endif
	}
#endif //NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER

#ifdef QT_DEBUG
	//test: compute rotation "norm" (as it may not be exactly 1 due to numerical (in)accuracy!)
	{
		ccGLMatrixd finalRotation = finalTransCorrected;
		finalRotation.setTranslation(CCVector3(0,0,0));
		ccGLMatrixd finalRotationT = finalRotation.transposed();
		ccGLMatrixd idTrans = finalRotation * finalRotationT;
		double norm = idTrans.data()[0] * idTrans.data()[5] * idTrans.data()[10];
		ccLog::PrintDebug("[GraphicalTransformationTool] T*T-1:");
		ccLog::PrintDebug(idTrans.toString(12,' ')); //full precision
		ccLog::PrintDebug(QString("Rotation norm = %1").arg(norm,0,'f',12));
	}
#endif

	//update GL transformation for all entities
	ccGLMatrix correctedFinalTrans(finalTransCorrected.data());

	for (unsigned i = 0; i < m_toTransform.getChildrenNumber(); ++i)
	{
		ccHObject* toTransform = m_toTransform.getChild(i);
		toTransform->setGLTransformation(correctedFinalTrans);

		//DGM: warning, applyGLTransformation may delete the associated octree!
		MainWindow::ccHObjectContext objContext = MainWindow::TheInstance()->removeObjectTemporarilyFromDBTree(toTransform);
		toTransform->applyGLTransformation_recursive();
		toTransform->prepareDisplayForRefresh_recursive();
		MainWindow::TheInstance()->putObjectBackIntoDBTree(toTransform,objContext);

		//special case: if the object is a mesh vertices set, we may have to update the mesh normals!
		if (toTransform->isA(CC_TYPES::POINT_CLOUD) && toTransform->getParent() && toTransform->getParent()->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(toTransform->getParent());
			if (mesh->hasTriNormals() && !m_toTransform.isAncestorOf(mesh))
			{
				mesh->transformTriNormals(correctedFinalTrans);
			}
		}
	}

	clearAdvModeEntities();

	stop(true);

	clear();

	//output resulting transformation matrix
	ccLog::Print("[GraphicalTransformationTool] Applied transformation:");
	ccLog::Print(correctedFinalTrans.toString(12,' ')); //full precision
#ifdef QT_DEBUG
	{
		float phi_rad = 0.0f;
		float theta_rad = 0.0f;
		float psi_rad = 0.0f;
		CCVector3f t3D;
		correctedFinalTrans.getParameters(phi_rad,theta_rad,psi_rad,t3D);
		ccLog::Print(QString("Angles(%1,%2,%3) T(%5,%6,%7)").arg(phi_rad).arg(theta_rad).arg(psi_rad).arg(t3D.x).arg(t3D.y).arg(t3D.z));
	}
#endif
}

void ccGraphicalTransformationTool::cancel()
{
	for (unsigned i=0; i<m_toTransform.getChildrenNumber(); ++i)
	{
		ccHObject* child = m_toTransform.getChild(i);
		child->resetGLTransformation();
		child->prepareDisplayForRefresh_recursive();
	}

	clearAdvModeEntities();

	stop(false);

	clear();

	//MainWindow::RefreshAllGLWindow();
}
