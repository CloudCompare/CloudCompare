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

#ifdef CC_3DXWARE_SUPPORT
//3DxWare
#include "devices/3dConnexion/Mouse3DParameters.h"
#endif

//system
#include <assert.h>

ccMouse3DContextMenu::ccMouse3DContextMenu(Mouse3DParameters* params, QWidget* parent/*=0*/)
	: QMenu(parent)
	, m_rotateCheckbox(0)
	, m_panZoomCheckbox(0)
	, m_params(params)
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

	/*** menu actions ***/

	m_rotateCheckbox = new QAction("Rotate",this);
	m_rotateCheckbox->setCheckable(true);
	m_rotateCheckbox->setChecked(m_params->rotationEnabled());
	connect(m_rotateCheckbox, SIGNAL(toggled(bool)), this, SLOT(rotateCheckBoxToggled(bool)));

	m_panZoomCheckbox = new QAction("Pan Zoom",this);
	m_panZoomCheckbox->setCheckable(true);
	m_panZoomCheckbox->setChecked(m_params->panZoomEnabled());
	connect(m_panZoomCheckbox, SIGNAL(toggled(bool)), this, SLOT(panZoomCheckBoxToggled(bool)));

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
			connect(m_speedActions[i], SIGNAL(toggled(bool)), this, SLOT(speedModeChanged(bool)));
		}
	}

	//build up menu
	addAction(m_rotateCheckbox);
	addAction(m_panZoomCheckbox);
	addSeparator();
	QMenu* speedMenu = new QMenu("Speed");
	for (int i=0; i<SPEED_ACTION_COUNT; ++i)
		speedMenu->addAction(m_speedActions[i]);
	addMenu(speedMenu);
	addSeparator();

#endif
}

void ccMouse3DContextMenu::rotateCheckBoxToggled(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
		m_params->enableRotation(state);
#endif
}

void ccMouse3DContextMenu::panZoomCheckBoxToggled(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
		m_params->enablePanZoom(state);
#endif
}

void ccMouse3DContextMenu::speedModeChanged(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_params)
	{
		//look for the given action than emitted the signal
		const QObject* sender = QObject::sender();
		int actionIndex = 0;
		for (actionIndex=0; actionIndex<SPEED_ACTION_COUNT; ++actionIndex)
			if (m_speedActions[actionIndex] == sender)
				break;

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
	}
#endif
}
