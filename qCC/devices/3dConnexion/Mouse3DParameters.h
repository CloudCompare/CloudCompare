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

#ifndef MOUSE_3D_PARAMETERS_HEADER
#define MOUSE_3D_PARAMETERS_HEADER

/** This file is inspired from the Qt wrapper for 3dConnexion devices graciously shared by Dabid Dibben:
	http://www.codegardening.com/2011/02/using-3dconnexion-mouse-with-qt.html
**/

//! Parameters for the 3D mouse based on the SDK from 3Dconnexion
class Mouse3DParameters
{
public:

	//! Speed mode
	enum SpeedMode {	LowestSpeed			= 0,	/**< Lowest speed (X0.25) **/
						LowSpeed			= 1,	/**< Low speed (X0.5) **/
						MidSpeed			= 2,	/**< Standard speed (X1.0) **/
						HighSpeed			= 3,	/**< High speed (X2.0) **/
						HighestSpeed		= 4,	/**< Highest speed (X4.0) **/
	};

	//! Pivot mode
	enum PivotMode {	ManualPivot			= 0,
						AutoPivot			= 1,
						AutoPivotOverride	= 2,
	};

	//! Navigation mode
	enum NavigationMode {	ObjectMode		= 0,
							CameraMode		= 1,
							FlyMode			= 2,
							WalkMode		= 3,
							HelicopterMode	= 4,
	};

	enum PivotVisibility {	HidePivot		= 0,
							ShowPivot		= 1,
							ShowMovingPivot	= 2,
	};

	//! Default constructor
	Mouse3DParameters();

	virtual inline bool panZoomEnabled() const { return m_panZoomEnabled; }
	virtual inline void enablePanZoom(bool state) { m_panZoomEnabled = state; }

	virtual inline bool rotationEnabled() const { return m_rotationEnabled; }
	virtual inline void enableRotation(bool state) { m_rotationEnabled = state; }

	virtual inline SpeedMode speedMode() const { return m_speedMode; }
	virtual inline void setSpeedMode(SpeedMode mode) { m_speedMode = mode; }

	virtual inline NavigationMode navigationMode() const { return m_navigationMode; }
	virtual inline void setNavigationMode(NavigationMode mode) { m_navigationMode = mode; }

	virtual inline PivotMode pivotMode() const { return m_pivotMode; }
	virtual inline void setPivotMode(PivotMode mode) {  if (m_pivotMode != ManualPivot || mode != AutoPivotOverride) m_pivotMode = mode; }
	
	virtual inline PivotVisibility pivotVisibility() const { return m_pivotVisibility; }
	virtual inline void setPivotVisibility(PivotVisibility visibility) { m_pivotVisibility = visibility; }

	virtual inline bool horizonLocked() const { return m_horizonLocked; }
	virtual inline void lockHorizon(bool state) { m_horizonLocked = state; }

	//! Saves parameters to persistent settings
	void fromPersistentSettings();
	//! Laods parameters from persistent settings
	void toPersistentSettings();

protected:

	SpeedMode		m_speedMode;
	bool			m_panZoomEnabled;
	bool			m_rotationEnabled;

	NavigationMode	m_navigationMode;
	PivotMode		m_pivotMode;
	PivotVisibility	m_pivotVisibility;
	bool			m_horizonLocked;

};

#endif //MOUSE_3D_PARAMETERS_HEADER
