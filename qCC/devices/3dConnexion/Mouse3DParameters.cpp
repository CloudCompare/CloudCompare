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

Mouse3DParameters::Mouse3DParameters()
	: m_speedMode(LowestSpeed)
	, m_panZoomEnabled(true)
	, m_rotationEnabled(true)
	, m_navigationMode(ObjectMode)
	, m_pivotMode(AutoPivot)
	, m_pivotVisibility(ShowPivot)
	, m_horizonLocked(false)
{
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
	m_navigationMode	= static_cast<NavigationMode>(navigationMode);
	m_pivotMode			= static_cast<PivotMode>(pivotMode);
	m_pivotVisibility	= static_cast<PivotVisibility>(pivotVisiblity);

	m_panZoomEnabled	= settings.value(c_ps_panZoomEnabled,	true).toBool();
	m_rotationEnabled	= settings.value(c_ps_rotationEnabled,	true).toBool();
	m_horizonLocked		= settings.value(c_ps_horizonLocked,	false).toBool();
}

void Mouse3DParameters::toPersistentSettings()
{
    QSettings settings;
    settings.beginGroup(c_ps_groupName);

	settings.setValue(c_ps_speedMode,		(int)m_speedMode);
	settings.setValue(c_ps_navigationMode,	(int)m_navigationMode);
	settings.setValue(c_ps_pivotMode,		(int)m_pivotMode);
	settings.setValue(c_ps_pivotVisiblity,	(int)m_pivotVisibility);

	settings.setValue(c_ps_panZoomEnabled,	m_panZoomEnabled);
	settings.setValue(c_ps_rotationEnabled,	m_rotationEnabled);
	settings.setValue(c_ps_horizonLocked,	m_horizonLocked);
}
