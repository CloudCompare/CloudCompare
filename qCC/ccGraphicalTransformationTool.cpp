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

#include "ccGraphicalTransformationTool.h"

#include "ccGLWindow.h"
#include "mainwindow.h"

//qCC_db
#include <ccLog.h>
#include <ccHObject.h>
#include <ccGLUtils.h>

ccGraphicalTransformationTool::ccGraphicalTransformationTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::GraphicalTransformationDlg()
	, m_toTransform(0)
{
	setupUi(this);

	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

    connect(pauseButton,	SIGNAL(toggled(bool)),  this, SLOT(pause(bool)));
	connect(okButton,		SIGNAL(clicked()),		this, SLOT(apply()));
	connect(razButton,		SIGNAL(clicked()),		this, SLOT(reset()));
	connect(cancelButton,	SIGNAL(clicked()),		this, SLOT(cancel()));

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space); //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverridenShortcut(Qt::Key_Return); //return key for the "ok" button
	connect(this, SIGNAL(shortcutTriggered(int)), this, SLOT(onShortcutTriggered(int)));

	m_toTransform = new ccHObject("ToTransform");
}

ccGraphicalTransformationTool::~ccGraphicalTransformationTool()
{
	clear();

	if (m_toTransform)
		delete m_toTransform;
	m_toTransform=0;
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
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
        m_associatedWin->displayNewMessage("Transformation [PAUSED]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
        m_associatedWin->displayNewMessage("Unpause to transform again",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
    }
    else
    {
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_ENTITY);
		m_associatedWin->displayNewMessage("[Rotation/Translation mode]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
    }

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);

    m_associatedWin->redraw();
}

void ccGraphicalTransformationTool::clear()
{
	m_toTransform->removeAllChildren();

	m_rotation.toIdentity();
	m_translation = CCVector3(0,0,0);
	m_rotationCenter = CCVector3(0,0,0);
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

	//we can't tranform locked entities
	if (entity->isLocked())
	{
		ccLog::Warning(QString("[Graphical Transformation Tool] Can't transform entity '%1' cause it's locked!").arg(entity->getName()));
		return false;
	}

	//we can't tranform child meshes
	if (entity->isA(CC_MESH) && entity->getParent() && entity->getParent()->isKindOf(CC_MESH))
	{
		ccLog::Warning(QString("[Graphical Transformation Tool] Entity '%1' can't be modified as it is part of a mesh group. You should 'clone' it first.").arg(entity->getName()));
		return false;
	}

	//eventually, we must check that there is no "parent + sibling" in the selection!
	//otherwise, the sibling will be rotated twice!
	assert(m_toTransform);
	unsigned n = m_toTransform->getChildrenNumber();
	for (unsigned i=0; i<n;)
	{
		ccHObject* previous = m_toTransform->getChild(i);
		if (previous->isAncestorOf(entity))
		{
			//we have found a parent, we won't add this entity
			return false;
		}
		//if the inverse is true, then we get rid of the current element!
		else if (entity->isAncestorOf(previous))
		{
			m_toTransform->removeChild(previous,true);
			--n;
		}
		else
		{
			//proceed
			++i;
		}
	}

	m_toTransform->addChild(entity,false);

	return true;
}

unsigned ccGraphicalTransformationTool::getNumberOfValidEntities()
{
	assert(m_toTransform);
	return m_toTransform->getChildrenNumber();
}

bool ccGraphicalTransformationTool::linkWith(ccGLWindow* win)
{
	if (!ccOverlayDialog::linkWith(win))
		return false;
	
	if (m_toTransform)
	{
		assert(!win || m_toTransform->getChildrenNumber() == 0);
		m_toTransform->setDisplay(win);
	}
	else
	{
		assert(false);
	}
	
	return true;
}

bool ccGraphicalTransformationTool::start()
{
	assert(!m_processing);
	assert(m_toTransform);

	assert(m_associatedWin);
	if (!m_associatedWin)
		return false;

	unsigned childNum = m_toTransform->getChildrenNumber();
	if (childNum == 0)
		return false;

	m_rotation.toIdentity();
	m_translation = CCVector3(0,0,0);
	m_rotationCenter = m_toTransform->getCenter(); //m_rotation center == selected entities center

	//activate "moving mode" in associated GL window
	m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_ENTITY);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	connect(m_associatedWin, SIGNAL(rotation(const ccGLMatrix&)),	this, SLOT(glRotate(const ccGLMatrix&)));
	connect(m_associatedWin, SIGNAL(translation(const CCVector3&)),	this, SLOT(glTranslate(const CCVector3&)));
	m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("[Rotation/Translation mode]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
	m_associatedWin->updateGL();

	return ccOverlayDialog::start();
}

void ccGraphicalTransformationTool::stop(bool state)
{
	if (m_associatedWin)
	{
		//deactivate "moving mode" in associated GL window
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
		disconnect(m_associatedWin, SIGNAL(rotation(const ccGLMatrix&)),	this, SLOT(glRotate(const ccGLMatrix&)));
		disconnect(m_associatedWin, SIGNAL(translation(const CCVector3&)),	this, SLOT(glTranslate(const CCVector3&)));
		m_associatedWin->displayNewMessage("[Rotation/Translation mode OFF]",ccGLWindow::UPPER_CENTER_MESSAGE,false,2,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->updateGL();
	}

	ccOverlayDialog::stop(state);
}

void ccGraphicalTransformationTool::glTranslate(const CCVector3& realT)
{
	CCVector3 t(realT.x * (TxCheckBox->isChecked() ? PC_ONE : 0),
				realT.y * (TyCheckBox->isChecked() ? PC_ONE : 0),
				realT.z * (TzCheckBox->isChecked() ? PC_ONE : 0));

	if (t.norm2() != 0)
	{
		m_translation += t;
		updateAllGLTransformations();
	}
}

void ccGraphicalTransformationTool::glRotate(const ccGLMatrix& rotMat)
{
	switch(rotComboBox->currentIndex())
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
	m_translation = CCVector3(0,0,0);

	updateAllGLTransformations();
}

void ccGraphicalTransformationTool::updateAllGLTransformations()
{
	assert(m_toTransform);

	//we recompute global GL transformation matrix
	ccGLMatrix newTrans = m_rotation;
	newTrans += (m_rotationCenter+m_translation-m_rotation*m_rotationCenter);

	for (unsigned i=0; i<m_toTransform->getChildrenNumber();++i)
	{
		ccHObject* child = m_toTransform->getChild(i);
		child->setGLTransformation(newTrans);
		child->prepareDisplayForRefresh_recursive();
	}

	MainWindow::RefreshAllGLWindow();
}

void ccGraphicalTransformationTool::apply()
{
	assert(m_toTransform);

	{
		//we recompute global GL transformation matrix and display it in console
		ccGLMatrix finalTrans = m_rotation;
		finalTrans += (m_rotationCenter+m_translation-m_rotation*m_rotationCenter);
		//output resulting transformation matrix
		ccLog::Print("[GraphicalTransformationTool] Applied transformation:");
		ccLog::Print(finalTrans.toString(12,' ')); //full precision
	}

	for (unsigned i=0; i<m_toTransform->getChildrenNumber();++i)
	{
		ccHObject* toTransform = m_toTransform->getChild(i);
		ccHObject* parent = 0;
		//DGM: warning, applyGLTransformation may delete associated octree!
		MainWindow::TheInstance()->removeObjectTemporarilyFromDBTree(toTransform,parent);
		toTransform->applyGLTransformation_recursive();
		toTransform->prepareDisplayForRefresh_recursive();
		MainWindow::TheInstance()->putObjectBackIntoDBTree(toTransform,parent);
	}

	stop(true);

	clear();

	//MainWindow::RefreshAllGLWindow();

}

void ccGraphicalTransformationTool::cancel()
{
	assert(m_toTransform);

	for (unsigned i=0; i<m_toTransform->getChildrenNumber();++i)
	{
		ccHObject* child = m_toTransform->getChild(i);
		child->razGLTransformation();
		child->prepareDisplayForRefresh_recursive();
	}

	stop(false);

	clear();

	//MainWindow::RefreshAllGLWindow();
}
