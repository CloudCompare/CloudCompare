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
#include <ccGLWindow.h>

#include "ccItemSelectionDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <ccPlane.h>
#include <ccDBRoot.h>


ccGraphicalTransformationTool::ccGraphicalTransformationTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::GraphicalTransformationDlg()
	, m_toTransform("transformed")
{
	setupUi(this);

	connect(pauseButton,    &QAbstractButton::toggled,	this, &ccGraphicalTransformationTool::pause);
	connect(okButton,       &QAbstractButton::clicked,	this, &ccGraphicalTransformationTool::apply);
	connect(razButton,	  &QAbstractButton::clicked,	this, &ccGraphicalTransformationTool::reset);
	connect(cancelButton,   &QAbstractButton::clicked,	this, &ccGraphicalTransformationTool::cancel);
	connect(advPushButton, &QPushButton::toggled, this, &ccGraphicalTransformationTool::advModeVisible);
	connect(translateComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccGraphicalTransformationTool::advTranslateRefChanged);
	connect(rotateComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &ccGraphicalTransformationTool::advRotateRefChanged);
	connect(refAxisRadio, &QRadioButton::toggled, this, &ccGraphicalTransformationTool::advAxisRefChanged);

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space); //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverridenShortcut(Qt::Key_Return); //return key for the "ok" button
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccGraphicalTransformationTool::onShortcutTriggered);

	objCenterRadio->setChecked(true);
	rotateComboBox->setCurrentIndex(0);
	translateComboBox->setCurrentIndex(0);
	advModeVisible(false);
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
		return;

	if (state)
	{
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->displayNewMessage("Transformation [PAUSED]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->displayNewMessage("Unpause to transform again",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
	}
	else
	{
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_ENTITIES());
		updateDisplayMessage();
	}

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);

	m_associatedWin->redraw(true, false);
}

void ccGraphicalTransformationTool::advModeVisible(bool state)
{
	rotateComboBox->setVisible(state);
	translateComboBox->setVisible(state);
	translateLabel->setVisible(state);
	rotateLabel->setVisible(state);
	rotAxisLabel->setVisible(state);
	objCenterRadio->setVisible(state);
	refAxisRadio->setVisible(state);
	groupBox->setVisible(state);
	int wPrev = this->width();
	if (state)
	{
		this->setGeometry(this->x() + (-wPrev + 200), this->y(), 200, 225);
		if (rotateComboBox->currentIndex() != 0)
		{
			rotComboBox->setEnabled(false);
		}
		if (translateComboBox->currentIndex() != 0)
		{
			TxCheckBox->setEnabled(false);
			TyCheckBox->setEnabled(false);
		}
	}
	else
	{
		rotComboBox->setEnabled(true);
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		this->setGeometry(this->x() , this->y(), 0, 0);
		this->adjustSize(); //adjust size will minimize the display height with the dropdowns blocked
		this->setGeometry(this->x() + (wPrev - 200), this->y(), 200, this->height());
	}
	//update mini-GUI
	advPushButton->blockSignals(true);
	advPushButton->setChecked(state);
	advPushButton->blockSignals(false);
	updateDisplayMessage();
}

void ccGraphicalTransformationTool::populateAdvModeItems()
{
	rotateComboBox->clear();
	translateComboBox->clear();
	rotateComboBox->insertItem(0, "Origin");
	translateComboBox->insertItem(0, "Origin");
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		ccHObject* root = mainWindow->dbRootObject();
		ccHObject::Container polylines;
		if (root)
		{
			root->filterChildren(polylines, true, CC_TYPES::POLY_LINE);
			root->filterChildren(m_planesAndLineSegments, true, CC_TYPES::PLANE);
		}
		if (!polylines.empty())
		{
			for (int i = 0; i < polylines.size(); i++)
			{
				ccPolyline* poly = static_cast<ccPolyline*>(polylines[i]);
				if (poly->size() == 2) //only single segment polylines allowed
				{
					m_planesAndLineSegments.push_back(polylines[i]);
				}
			}
		}
		if (!m_planesAndLineSegments.empty())
		{
			for (int i = 0; i < m_planesAndLineSegments.size(); ++i)
			{
				//add one line per entity
				QString item = QString("%1 (ID=%2)").arg(m_planesAndLineSegments[i]->getName()).arg(m_planesAndLineSegments[i]->getUniqueID());
				translateComboBox->insertItem(i+1, item, QVariant(m_planesAndLineSegments[i]->getUniqueID()));
				rotateComboBox->insertItem(i+1, item, QVariant(m_planesAndLineSegments[i]->getUniqueID()));
			}
		}
	}
}

bool ccGraphicalTransformationTool::setAdvancedTranslationTransform(ccHObject* translateRef) 
{
	if (!m_associatedWin)
	{
		assert(false);
		return false;
	}
	
	if (translateRef->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* line = static_cast<ccPolyline*>(translateRef);
		CCVector3 arbitraryVec = *line->getPoint(1) - *line->getPoint(0);
		m_advTranslationTransform = getArbitraryVectorTranslationTransform(arbitraryVec);
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
	else
	{
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		return false;
	}
}

bool ccGraphicalTransformationTool::setAdvancedRotationTransform(ccHObject* translateRef)
{
	if (!m_associatedWin)
	{
		assert(false);
		return false;
	}
	if (translateRef->isA(CC_TYPES::POLY_LINE))
	{
		ccPolyline* line = static_cast<ccPolyline*>(translateRef);
		CCVector3 start = *line->getPoint(0);
		CCVector3 arbitraryVec = *line->getPoint(1) - *line->getPoint(0);
		arbitraryVec.normalize();
		m_advRotationAxis = m_rotation.inverse() * CCVector3d::fromArray(arbitraryVec.u);
		if (refAxisRadio->isChecked())
		{
			ccGLMatrixd lineTransform = ccGLMatrixd(line->getGLTransformationHistory().data());
			m_advRotationRefObjCenter = lineTransform.getTranslationAsVec3D();
			ccLog::Print(QString("%1,%2,%3").arg(m_advRotationRefObjCenter[0]).arg(m_advRotationRefObjCenter[1]).arg(m_advRotationRefObjCenter[2]));
			CCVector3d newCenter = (m_advRotationRefObjCenter - m_rotation.inverse() * m_position.getTranslationAsVec3D());
			setRotationCenter(newCenter);
		}
		else
		{
			CCVector3d newCenter = CCVector3d::fromArray(m_toTransform.getBB_recursive().getCenter().u);
			setRotationCenter(newCenter);
		}
		rotComboBox->setCurrentIndex(3); //Z
		rotComboBox->setEnabled(false);
		return true;
	}
	else if (translateRef->isA(CC_TYPES::PLANE))
	{
		ccPlane* plane = static_cast<ccPlane*>(translateRef);
		CCVector3 planeNorm = plane->getNormal();
		m_advRotationAxis = m_rotation.inverse() * CCVector3d::fromArray(planeNorm.u);
		if (refAxisRadio->isChecked())
		{
			ccGLMatrixd planeTransform = ccGLMatrixd(plane->getTransformation().data());
			m_advRotationRefObjCenter = planeTransform.getTranslationAsVec3D();
			CCVector3d newCenter = (m_rotation.inverse() * m_advRotationRefObjCenter - m_rotation.inverse() * m_position.getTranslationAsVec3D());
			ccLog::Print(QString("%1,%2,%3").arg(newCenter[0]).arg(newCenter[1]).arg(newCenter[2]));

			setRotationCenter(newCenter);
		}
		else
		{
			CCVector3d newCenter = CCVector3d::fromArray(m_toTransform.getBB_recursive().getCenter().u);
			setRotationCenter(newCenter);
		}
		rotComboBox->setCurrentIndex(3); //Z
		rotComboBox->setEnabled(false);
		return true;
	}
	else
	{
		rotateComboBox->setCurrentIndex(0);
		rotComboBox->setEnabled(true);
		return false;
	}
}

void ccGraphicalTransformationTool::advTranslateRefChanged(int index)
{
	if (m_planesAndLineSegments.empty())
	{
		return;
	}
	int id = translateComboBox->itemData(index).toInt();
	int selectedObjectIndex = -1;
	for (int i = 0; i < m_planesAndLineSegments.size(); i++)
	{
		if (id == m_planesAndLineSegments[i]->getUniqueID())
		{
			selectedObjectIndex = i;
		}
	}
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		for (int i = 0; i < m_planesAndLineSegments.size(); i++)
		{
			mainWindow->db()->unselectEntity(m_planesAndLineSegments[i]);
			if (selectedObjectIndex == i)
			{
				mainWindow->db()->selectEntity(m_planesAndLineSegments[selectedObjectIndex], true);
			}
		}
	}
	if (selectedObjectIndex != -1)
	{
		if (!setAdvancedTranslationTransform(m_planesAndLineSegments[selectedObjectIndex]))
		{
			ccLog::Error("Error calculating adv translation transform, cannot translate along selected item");
			m_advTranslationTransform.toIdentity();
		}
	}
	else
	{
		TxCheckBox->setEnabled(true);
		TyCheckBox->setEnabled(true);
		m_advTranslationTransform.toIdentity();
	}

}

void ccGraphicalTransformationTool::advRotateRefChanged(int index)
{
	if (m_planesAndLineSegments.empty())
	{
		return;
	}
	int id = rotateComboBox->itemData(index).toInt();
	int selectedObjectIndex = -1;
	for (int i = 0; i < m_planesAndLineSegments.size(); i++)
	{
		if (id == m_planesAndLineSegments[i]->getUniqueID())
		{
			selectedObjectIndex = i;
		}
	}
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		for (int i = 0; i < m_planesAndLineSegments.size(); i++)
		{
			mainWindow->db()->unselectEntity(m_planesAndLineSegments[i]);
			if (selectedObjectIndex == i)
			{
				mainWindow->db()->selectEntity(m_planesAndLineSegments[selectedObjectIndex], true);
			}
		}
	}
	if (selectedObjectIndex != -1)
	{
		if (!setAdvancedRotationTransform(m_planesAndLineSegments[selectedObjectIndex]))
		{
			ccLog::Error("Error calculating adv translation transform, cannot rotate around selected item");
		}
	}
	else
	{
		ccLog::Print("origin rotation");
		CCVector3d center = CCVector3d::fromArray(m_toTransform.getBB_recursive().getCenter().u);
		setRotationCenter(center);
		rotComboBox->setEnabled(true);
	}
}

void ccGraphicalTransformationTool::advAxisRefChanged(bool state)
{
	advRotateRefChanged(rotateComboBox->currentIndex()); //force an update
}

void ccGraphicalTransformationTool::clear()
{
	m_toTransform.detatchAllChildren();

	m_rotation.toIdentity();
	m_translation = CCVector3d(0,0,0);
	m_rotationCenter = CCVector3d(0,0,0);
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

bool ccGraphicalTransformationTool::linkWith(ccGLWindow* win)
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
	if (!m_associatedWin)
	{
		return;
	}
	if (advPushButton->isChecked())
	{
		m_associatedWin->displayNewMessage("[Advanced mode]", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->displayNewMessage("[Select Plane or Line to update reference frame]", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->displayNewMessage("[If plane selected, rotation will be around normal vector]", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
	}
	else
	{
		m_associatedWin->displayNewMessage("[Rotation/Translation mode]", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
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
	m_rotationCenter = CCVector3d::fromArray(m_toTransform.getBB_recursive().getCenter().u); //m_rotation center == selected entities center

	//activate "moving mode" in associated GL window
	m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_ENTITIES());
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	connect(m_associatedWin, &ccGLWindow::rotation, this, &ccGraphicalTransformationTool::glRotate);
	connect(m_associatedWin, &ccGLWindow::translation, this, &ccGraphicalTransformationTool::glTranslate);
	m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	pauseButton->setChecked(false);
	TxCheckBox->setEnabled(true);
	TyCheckBox->setEnabled(true);
	rotComboBox->setEnabled(true);
	m_planesAndLineSegments.clear();
	populateAdvModeItems();
	updateDisplayMessage();
	m_associatedWin->redraw(true, false);
	return ccOverlayDialog::start();
}

void ccGraphicalTransformationTool::stop(bool state)
{
	if (m_associatedWin)
	{
		//deactivate "moving mode" in associated GL window
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
		m_associatedWin->disconnect(this);
		m_associatedWin->displayNewMessage("[Rotation/Translation mode OFF]",ccGLWindow::UPPER_CENTER_MESSAGE,false,2,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->redraw(true, false);
	}

	ccOverlayDialog::stop(state);
}

ccGLMatrixd ccGraphicalTransformationTool::getArbitraryVectorTranslationTransform(const CCVector3& vec)
{
	PointCoordinateType theta;
	PointCoordinateType phi;

	if (std::abs(vec.z) < ZERO_TOLERANCE)
	{
		if (std::abs(vec.y) < ZERO_TOLERANCE)
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
			theta = M_PI + theta;
		}
		else if (vec.z < 0 && vec.y > 0)
		{
			theta = M_PI_2 - theta;
		}
	}

	PointCoordinateType phiDenominator = std::sqrt((vec.y * vec.y) + (vec.z * vec.z));
	if (phiDenominator < ZERO_TOLERANCE)
	{
		if (std::abs(vec.x) < ZERO_TOLERANCE)
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
	if (std::abs(vec.x) < ZERO_TOLERANCE && std::abs(vec.y) < ZERO_TOLERANCE && vec.z < 0)
	{
		arbitraryVectorTranslationAdjust.scaleRotation(-1);
	}
	return arbitraryVectorTranslationAdjust;
}

void ccGraphicalTransformationTool::glTranslate(const CCVector3d& realT)
{
	CCVector3d t(	realT.x * (TxCheckBox->isChecked() ? 1 : 0),
					realT.y * (TyCheckBox->isChecked() ? 1 : 0),
					realT.z * (TzCheckBox->isChecked() ? 1 : 0));

	if (advPushButton->isChecked()) //advance translate mode
	{
		m_advTranslationTransform.applyRotation(t);
	}
		
	if (t.norm2() != 0)
	{
		m_translation += t;
		if (advPushButton->isChecked() && rotateComboBox->currentIndex() != 0)
		{
			if (refAxisRadio->isChecked())
			{ 
				CCVector3d centerUpdate = m_rotation.inverse() * m_advRotationRefObjCenter - m_rotation.inverse() * m_position.getTranslationAsVec3D();
				m_translation += (m_rotationCenter - centerUpdate) - m_rotation * (m_rotationCenter - centerUpdate);
				m_rotationCenter = centerUpdate;
			}
		}
		updateAllGLTransformations();
	}
}

void ccGraphicalTransformationTool::glRotate(const ccGLMatrixd& rotMat)
{
	if (advPushButton->isChecked() && rotateComboBox->currentIndex() != 0)
	{
		// advRotationTransform = cos(theta)I + 1-cos(theta)u (X) u + sin(theta)u_skewsym
		double cosTheta = rotMat.zRotation()(0, 0);
		double sinTheta = rotMat.zRotation()(1, 0);
		ccGLMatrixd firstTerm = ccGLMatrixd();
		firstTerm.scaleRotation(cosTheta);
		ccGLMatrixd secondTerm = ccGLMatrixd();
		CCVector3d v = (1 - cosTheta) * m_advRotationAxis;
		CCVector3d w = m_advRotationAxis;
		secondTerm.setColumn(0, CCVector3d(v[0] * w[0], v[1] * w[0], v[2] * w[0]));
		secondTerm.setColumn(1, CCVector3d(v[0] * w[1], v[1] * w[1], v[2] * w[1]));
		secondTerm.setColumn(2, CCVector3d(v[0] * w[2], v[1] * w[2], v[2] * w[2]));
		ccGLMatrixd thirdTerm = ccGLMatrixd();
		thirdTerm.setColumn(0, CCVector3d(0, m_advRotationAxis[2], -m_advRotationAxis[1]));
		thirdTerm.setColumn(1, CCVector3d(-m_advRotationAxis[2], 0, m_advRotationAxis[0]));
		thirdTerm.setColumn(2, CCVector3d(m_advRotationAxis[1], -m_advRotationAxis[0], 0));
		thirdTerm.scaleRotation(sinTheta);
		ccGLMatrixd advRotationTransform = firstTerm;
		advRotationTransform += secondTerm;
		advRotationTransform.scaleRow(3, .5);
		advRotationTransform += thirdTerm;
		advRotationTransform.scaleRow(3, .5);
		m_rotation = m_rotation * advRotationTransform;
	}
	else
	{
		switch (rotComboBox->currentIndex())
		{
		case 0: //XYZ
			m_rotation = rotMat * m_rotation;
			break;
		case 1: //X
			m_rotation = rotMat.xRotation() * m_rotation;
			break;
		case 2: //Y
			m_rotation = rotMat.yRotation() * m_rotation;
			break;
		case 3: //Z
			m_rotation = rotMat.zRotation() * m_rotation;
			break;
		case 4: //None
			break;
		}
	}

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::reset()
{
	m_rotation.toIdentity();
	m_translation = CCVector3d(0, 0, 0);
	updateAllGLTransformations();
	advRotateRefChanged(rotateComboBox->currentIndex()); //force an update
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

	MainWindow::RefreshAllGLWindow(false);
}

void ccGraphicalTransformationTool::apply()
{
	//we recompute global GL transformation matrix and display it in console
	/*ccGLMatrixd finalTrans = m_rotation;
	finalTrans += m_rotationCenter + m_translation - m_rotation * m_rotationCenter;*/

	ccGLMatrixd finalTransCorrected = m_position;
#define NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER
#ifdef NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER
	{
		//convert matrix back and forth so as to be sure to get a 'true' rotation matrix
		//DGM: we use Euler angles, as the axis/angle method (formerly used) is not robust
		//enough! Shifts could be perceived by the user.
		double phi_rad,theta_rad,psi_rad;
		CCVector3d t3D;
		m_position.getParameters(phi_rad,theta_rad,psi_rad,t3D);
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

	for (unsigned i=0; i<m_toTransform.getChildrenNumber(); ++i)
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

	stop(true);

	clear();

	//output resulting transformation matrix
	ccLog::Print("[GraphicalTransformationTool] Applied transformation:");
	ccLog::Print(correctedFinalTrans.toString(12,' ')); //full precision
#ifdef QT_DEBUG
	{
		float phi_rad,theta_rad,psi_rad;
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

	stop(false);

	clear();

	//MainWindow::RefreshAllGLWindow();
}
