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

//qCC_db
#include <ccLog.h>
#include <ccMesh.h>


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

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space); //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverridenShortcut(Qt::Key_Return); //return key for the "ok" button
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccGraphicalTransformationTool::onShortcutTriggered);
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
		m_associatedWin->displayNewMessage("[Rotation/Translation mode]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
	}

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);

	m_associatedWin->redraw(true, false);
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
	m_associatedWin->displayNewMessage("[Rotation/Translation mode]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
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

void ccGraphicalTransformationTool::glTranslate(const CCVector3d& realT)
{
	CCVector3d t(	realT.x * (TxCheckBox->isChecked() ? 1 : 0),
					realT.y * (TyCheckBox->isChecked() ? 1 : 0),
					realT.z * (TzCheckBox->isChecked() ? 1 : 0));

	if (t.norm2() != 0)
	{
		m_translation += t;
		updateAllGLTransformations();
	}
}

void ccGraphicalTransformationTool::glRotate(const ccGLMatrixd& rotMat)
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
	}

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::reset()
{
	m_rotation.toIdentity();
	m_translation = CCVector3d(0, 0, 0);

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::setRotationCenter(CCVector3d& center)
{
	m_translation += (m_rotationCenter-center) - m_rotation*(m_rotationCenter-center);
	m_rotationCenter = center;

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::updateAllGLTransformations()
{
	//we recompute global GL transformation matrix
	ccGLMatrixd newTrans = m_rotation;
	newTrans += m_rotationCenter + m_translation - m_rotation * m_rotationCenter;

	ccGLMatrix newTransf(newTrans.data());
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
	ccGLMatrixd finalTrans = m_rotation;
	finalTrans += m_rotationCenter + m_translation - m_rotation*m_rotationCenter;

	ccGLMatrixd finalTransCorrected = finalTrans;
#define NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER
#ifdef NORMALIZE_TRANSFORMATION_MATRIX_WITH_EULER
	{
		//convert matrix back and forth so as to be sure to get a 'true' rotation matrix
		//DGM: we use Euler angles, as the axis/angle method (formerly used) is not robust
		//enough! Shifts could be perceived by the user.
		double phi_rad,theta_rad,psi_rad;
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
