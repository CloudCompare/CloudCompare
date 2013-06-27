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

#include "ccMouse3DContextMenu.h"

//local
#include <ccGLWindow.h>

#ifdef CC_3DXWARE_SUPPORT
//3DxWare
#include "devices/3dConnexion/Mouse3DParameters.h"
#endif

//system
#include <assert.h>

ccMouse3DContextMenu::ccMouse3DContextMenu(Mouse3DParameters* params, ccGLWindow* win, QWidget* parent/*=0*/)
	: QMenu(parent)
	, m_rotationMode(0)
	, m_panZoomMode(0)
	, m_autoRotationCenter(0)
	, m_selectedItemAsRotationCenter(0)
	, m_alwaysShowRotationCenter(0)
	, m_showRotationCenterOnMotion(0)
	, m_alwaysHideRotationCenter(0)
	, m_objectMode(0)
	, m_cameraMode(0)
	, m_lockHorizon(0)
	, m_dominantMode(0)
	, m_params(params)
	, m_glWindow(win)
{
	assert(m_params);
	{
		for (int i=0; i<SPEED_ACTION_COUNT; ++i)
			m_speedActions[i]=0;
	}

	if (!m_params)
		return;

#ifdef CC_3DXWARE_SUPPORT

	assert(Mouse3DParameters::HighestSpeed+1 == SPEED_ACTION_COUNT);

	//'Rotate' mode 
	{
		m_rotationMode = new QAction("Rotate",this);
		m_rotationMode->setCheckable(true);
		m_rotationMode->setChecked(m_params->rotationEnabled());
		connect(m_rotationMode, SIGNAL(toggled(bool)), this, SLOT(rotationModeToggled(bool)));
	}
	
	//'Pan/Zoom' mode 
	{
		m_panZoomMode = new QAction("Pan Zoom",this);
		m_panZoomMode->setCheckable(true);
		m_panZoomMode->setChecked(m_params->panZoomEnabled());
		connect(m_panZoomMode, SIGNAL(toggled(bool)), this, SLOT(panZoomModeToggled(bool)));
	}

	//speed actions
	{
		QString square = QString(QChar(0x25A0));
		QString squareSeq;
		for (int i=0; i<SPEED_ACTION_COUNT; ++i)
		{
			squareSeq += square;
			m_speedActions[i] = new QAction(squareSeq,this);
			m_speedActions[i]->setCheckable(true);
			m_speedActions[i]->setChecked(i == static_cast<int>(m_params->speedMode()));
			connect(m_speedActions[i], SIGNAL(triggered()), this, SLOT(speedModeChanged()));
		}
	}

	//rotation center actions
	{
		//Auto-rotation center (feature is not available in CC - yet?)
		{
			//m_autoRotationCenter				= new QAction("Auto",this);
			//m_selectedItemAsRotationCenter	= new QAction("Use Selected Item",this);
			//m_autoRotationCenter->setCheckable(true);
			//m_selectedItemAsRotationCenter->setCheckable(true);
			//m_selectedItemAsRotationCenter->setChecked(true);
		}

		//Rotation center visibility
		{
			m_alwaysShowRotationCenter		= new QAction("Always Show",this);
			m_showRotationCenterOnMotion	= new QAction("Show on Motion",this);
			m_alwaysHideRotationCenter		= new QAction("Hide",this);

			//we deduce the rotation visibility mode from the current display!
			ccGLWindow::PivotVisibility visibility = (m_glWindow ? m_glWindow->getPivotVisibility() : ccGLWindow::PIVOT_HIDE);

			m_alwaysShowRotationCenter->setCheckable(true);
			m_alwaysShowRotationCenter->setChecked(visibility == ccGLWindow::PIVOT_ALWAYS_SHOW);
			m_showRotationCenterOnMotion->setCheckable(true);
			m_showRotationCenterOnMotion->setChecked(visibility == ccGLWindow::PIVOT_SHOW_ON_MOVE);
			m_alwaysHideRotationCenter->setCheckable(true);
			m_alwaysHideRotationCenter->setChecked(visibility == ccGLWindow::PIVOT_HIDE);

			connect(m_alwaysShowRotationCenter,		SIGNAL(triggered()), this, SLOT(rotationCenterVisibilityChanged()));
			connect(m_showRotationCenterOnMotion,	SIGNAL(triggered()), this, SLOT(rotationCenterVisibilityChanged()));
			connect(m_alwaysHideRotationCenter,		SIGNAL(triggered()), this, SLOT(rotationCenterVisibilityChanged()));
		}
	}

	//we deduce the viewing mode (OBJECT, CAMERA, etc.) from the current display settings!
	{
		bool objectMode = true;
		if (m_glWindow)
			m_glWindow->getPerspectiveState(objectMode);

		//object mode
		{
			m_objectMode = new QAction("Object Mode",this);
			m_objectMode->setCheckable(true);
			//m_objectMode->setChecked(m_params->navigationMode() == Mouse3DParameters::ObjectMode);
			m_objectMode->setChecked(objectMode);
			connect(m_objectMode, SIGNAL(triggered()), this, SLOT(objectModeTriggered()));
		}
		//camera mode
		{
			m_cameraMode = new QAction("Camera Mode",this);
			m_cameraMode->setCheckable(true);
			//m_cameraMode->setChecked(m_params->navigationMode() == Mouse3DParameters::CameraMode);
			m_cameraMode->setChecked(!objectMode);
			connect(m_cameraMode, SIGNAL(triggered()), this, SLOT(cameraModeTriggered()));
		}
	}
	
	//lock horizon
	{
		m_lockHorizon = new QAction("Lock Horizon",this);
		m_lockHorizon->setCheckable(true);
		m_lockHorizon->setChecked(m_params->horizonLocked());
		connect(m_lockHorizon, SIGNAL(toggled(bool)), this, SLOT(lockHorizonToggled(bool)));
	}

	//dominant mode
	{
		m_dominantMode = new QAction("Dominant mode",this);
		m_dominantMode->setCheckable(true);
		m_dominantMode->setChecked(m_params->dominantModeEnabled());
		connect(m_dominantMode, SIGNAL(toggled(bool)), this, SLOT(dominantModeToggled(bool)));
	}

	/*** build menu up ***/

	addAction(m_rotationMode);
	addAction(m_panZoomMode);

	addSeparator(); //------------------------------
	
	QMenu* speedMenu = new QMenu("Speed");
	{
		for (int i=0; i<SPEED_ACTION_COUNT; ++i)
			speedMenu->addAction(m_speedActions[i]);
	}	
	addMenu(speedMenu);

	addSeparator(); //------------------------------
	
	addAction(m_objectMode);
	addAction(m_cameraMode);

	addSeparator(); //------------------------------

	QMenu* rotationCenterMenu = new QMenu("Rotation Center");
	{
		if (m_autoRotationCenter) //Auto-rotation center feature is not available in CC
		{
			rotationCenterMenu->addAction(m_autoRotationCenter);
			rotationCenterMenu->addAction(m_selectedItemAsRotationCenter);
	
			addSeparator(); //------------------------------
		}
		rotationCenterMenu->addAction(m_alwaysShowRotationCenter);
		rotationCenterMenu->addAction(m_showRotationCenterOnMotion);
		rotationCenterMenu->addAction(m_alwaysHideRotationCenter);
	}
	addMenu(rotationCenterMenu);

	addSeparator(); //------------------------------

	addAction(m_lockHorizon);
	addAction(m_dominantMode);

#endif
}

void ccMouse3DContextMenu::rotationModeToggled(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
		m_params->enableRotation(state);
#endif
}

void ccMouse3DContextMenu::panZoomModeToggled(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
		m_params->enablePanZoom(state);
#endif
}

void ccMouse3DContextMenu::objectModeTriggered()
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_glWindow)
	{
		bool objectCentered = true;
		bool perspective = m_glWindow->getPerspectiveState(objectCentered);
		if (!objectCentered) //force object-based mode!
			m_glWindow->setPerspectiveState(perspective,true);
	}

	//if (m_params)
	//	m_params->setNavigationMode(Mouse3DParameters::ObjectMode);
#endif
}

void ccMouse3DContextMenu::cameraModeTriggered()
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_glWindow)
	{
		bool objectCentered = true;
		/*bool perspective = */m_glWindow->getPerspectiveState(objectCentered);
		if (objectCentered) //force viewer-based perspective!
			m_glWindow->setPerspectiveState(true,false);
	}

	//if (m_params)
	//	m_params->setNavigationMode(Mouse3DParameters::CameraMode);
#endif
}

void ccMouse3DContextMenu::lockHorizonToggled(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
		m_params->lockHorizon(state);
#endif
}

void ccMouse3DContextMenu::dominantModeToggled(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
		m_params->enableDominantMode(state);
#endif
}

void ccMouse3DContextMenu::speedModeChanged()
{
#ifdef CC_3DXWARE_SUPPORT
	if (!m_params)
		return;

	//determine which action has triggered this slot
	const QObject* sender = QObject::sender();
	int actionIndex = 0;
	for (actionIndex=0; actionIndex<SPEED_ACTION_COUNT; ++actionIndex)
		if (m_speedActions[actionIndex] == sender)
			break;

	//change speed mode accordingly
	switch(actionIndex)
	{
	case Mouse3DParameters::LowestSpeed:
		m_params->setSpeedMode(Mouse3DParameters::LowestSpeed);
		break;
	case Mouse3DParameters::LowSpeed:
		m_params->setSpeedMode(Mouse3DParameters::LowSpeed);
		break;
	case Mouse3DParameters::MidSpeed:
		m_params->setSpeedMode(Mouse3DParameters::MidSpeed);
		break;
	case Mouse3DParameters::HighSpeed:
		m_params->setSpeedMode(Mouse3DParameters::HighSpeed);
		break;
	case Mouse3DParameters::HighestSpeed:
		m_params->setSpeedMode(Mouse3DParameters::HighestSpeed);
		break;
	}
#endif
}

void ccMouse3DContextMenu::rotationCenterVisibilityChanged()
{
	//determine which action has triggered this slot
	const QObject* sender = QObject::sender();
	if (!sender)
		return;

	//we also need an actve 3D view!
	if (!m_glWindow)
		return;

	//let's determine the new visibility based on the sending action 
	ccGLWindow::PivotVisibility visibility = m_glWindow->getPivotVisibility();
	if (sender == m_alwaysShowRotationCenter)
	{
		visibility = ccGLWindow::PIVOT_ALWAYS_SHOW;
	}
	else if (sender == m_showRotationCenterOnMotion)
	{
		visibility = ccGLWindow::PIVOT_SHOW_ON_MOVE;
	}
	else if (sender == m_alwaysHideRotationCenter)
	{
		visibility = ccGLWindow::PIVOT_HIDE;
	}
	else
	{
		assert(false);
		return;
	}

	//update visibility... only if necessary!
	if (m_glWindow->getPivotVisibility() != visibility)
	{
		m_glWindow->setPivotVisibility(visibility);
		m_glWindow->redraw();
	}
}