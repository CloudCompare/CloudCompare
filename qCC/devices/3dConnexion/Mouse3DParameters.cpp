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

#include "Mouse3DParameters.h"

//Qt
#include <QSettings>

//system
#include <assert.h>

Mouse3DParameters::Mouse3DParameters()
	: m_speedMode(LowestSpeed)
	, m_panZoomEnabled(true)
	, m_rotationEnabled(true)
	//, m_navigationMode(ObjectMode)
	//, m_pivotMode(AutoPivot)
	//, m_pivotVisibility(ShowPivot)
	, m_horizonLocked(false)
	, m_dominantMode(false)
{
}

QString Mouse3DParameters::GetName(NavigationMode mode)
{
	QString modeName;
	switch (mode)
	{
	case Mouse3DParameters::ObjectMode:
		return "Object";
	case Mouse3DParameters::CameraMode:
		return "Camera";
	case Mouse3DParameters::FlyMode:
		return "Fly";
	case Mouse3DParameters::WalkMode:
		return "Walk";
	case Mouse3DParameters::HelicopterMode:
		return "Helicopter";
	default:
		assert(false);
		break;
	}

	return "unknown";
}

/*** Persistent settings ***/

const char c_ps_groupName[]			= "Mouse3DParams";

const char c_ps_speedMode[]			= "SpeedMode";
const char c_ps_navigationMode[]	= "NavigationMode";
const char c_ps_pivotMode[]			= "PivotMode";
const char c_ps_pivotVisiblity[]	= "PivotVisiblity";

const char c_ps_panZoomEnabled[]	= "PanZoomEnabled";
const char c_ps_rotationEnabled[]	= "RotationEnabled";
const char c_ps_horizonLocked[]		= "HorizonLocked";

void Mouse3DParameters::fromPersistentSettings()
{
    QSettings settings;
    settings.beginGroup(c_ps_groupName);

	int speedMode		= settings.value(c_ps_speedMode,		(int)LowestSpeed).toInt();
	int navigationMode	= settings.value(c_ps_navigationMode,	(int)ObjectMode).toInt();
	int pivotMode		= settings.value(c_ps_pivotMode,		(int)AutoPivot).toInt();
	int pivotVisiblity	= settings.value(c_ps_pivotVisiblity,	(int)ShowPivot).toInt();

	m_speedMode			= static_cast<SpeedMode>(speedMode);
	//m_navigationMode	= static_cast<NavigationMode>(navigationMode);
	//m_pivotMode			= static_cast<PivotMode>(pivotMode);
	//m_pivotVisibility	= static_cast<PivotVisibility>(pivotVisiblity);

	m_panZoomEnabled	= settings.value(c_ps_panZoomEnabled,	true).toBool();
	m_rotationEnabled	= settings.value(c_ps_rotationEnabled,	true).toBool();
	m_horizonLocked		= settings.value(c_ps_horizonLocked,	false).toBool();
}

void Mouse3DParameters::toPersistentSettings()
{
    QSettings settings;
    settings.beginGroup(c_ps_groupName);

	settings.setValue(c_ps_speedMode,		(int)m_speedMode);
	//settings.setValue(c_ps_navigationMode,	(int)m_navigationMode);
	//settings.setValue(c_ps_pivotMode,		(int)m_pivotMode);
	//settings.setValue(c_ps_pivotVisiblity,	(int)m_pivotVisibility);

	settings.setValue(c_ps_panZoomEnabled,	m_panZoomEnabled);
	settings.setValue(c_ps_rotationEnabled,	m_rotationEnabled);
	settings.setValue(c_ps_horizonLocked,	m_horizonLocked);
}

void Mouse3DParameters::accelerate()
{
	switch(m_speedMode)
	{
	case LowestSpeed:
		setSpeedMode(Mouse3DParameters::LowSpeed);
		break;
	case LowSpeed:
		setSpeedMode(Mouse3DParameters::MidSpeed);
		break;
	case MidSpeed:
		setSpeedMode(Mouse3DParameters::HighSpeed);
		break;
	case HighSpeed:
		setSpeedMode(Mouse3DParameters::HighestSpeed);
		break;
	case HighestSpeed:
		//can't accelerate any more!
		break;
	}
}

void Mouse3DParameters::slowDown()
{
	switch(m_speedMode)
	{
	case LowestSpeed:
		//can't slow down any more
		break;
	case LowSpeed:
		setSpeedMode(Mouse3DParameters::LowestSpeed);
		break;
	case MidSpeed:
		setSpeedMode(Mouse3DParameters::LowSpeed);
		break;
	case HighSpeed:
		setSpeedMode(Mouse3DParameters::MidSpeed);
		break;
	case HighestSpeed:
		setSpeedMode(Mouse3DParameters::HighSpeed);
		break;
	}
}
