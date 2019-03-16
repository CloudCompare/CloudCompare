//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#ifndef CC_COMPASS_DIALOG_HEADER
#define CC_COMPASS_DIALOG_HEADER

//Qt
#include <QDialog>
#include <QList>
#include <QAction>

//CC
#include <ccGLWindow.h>
#include <ccOverlayDialog.h>

//Local
#include <ui_compassDlg.h>
#include "ccTrace.h"

class ccCompassDlg : public ccOverlayDialog, public Ui::compassDlg
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccCompassDlg(QWidget* parent = 0);

	/*
	Returns a flag describing the currently selected ccTrace::COST_MODE (used to build the cost function for optimisation)
	*/
	int getCostMode();

	/*
	Returns true if the m_plane_fit action is checked -> used to check if the user expects us to fit a plane to finished traces.
	*/
	bool planeFitMode();

	//menus
	QMenu *m_cost_algorithm_menu;
	QMenu *m_settings_menu;
	QMenu *m_pairpicking_menu;
	QMenu *m_research_menu;

	//settings menu actions
	QAction *m_plane_fit;
	QAction *m_showStippled;
	QAction *m_showNormals;
	QAction *m_showNames;
	QAction *m_recalculate;

	//pair picking menu actions
	QAction *m_pinchTool;
	QAction *m_measure_thickness; //activates thickness tool
	QAction *m_measure_thickness_twoPoint; //activates thickness tool
	//--
	QAction *m_youngerThan; //activates topology tool - crosscutting relations
	QAction *m_follows; //activates topology tool - stratigraphic younging relations
	QAction *m_equivalent; //activates topology tool - coeval/equivalent relations
	//--
	QAction *m_fitPlaneToGeoObject; //fits a plane to the upper/lower surfaces of all points in a GeoObject
	QAction *m_recalculateFitPlanes;
	QAction *m_estimateNormals; //estimate structure normals
	QAction *m_estimateP21; //estimate the intensity of structures (doesn't need to know orientation) 
	QAction *m_estimateStrain; //estimate strain from Mode-I dykes and veins
	QAction *m_mergeSelected; //merges the selected geoObjects
	QAction *m_toPointCloud; //converts geoObject data to point cloud
	QAction *m_distributeSelection; //tool for distributing imported data (meshes etc.) into GeoObjects based on shared names.

	//--
	QAction *m_noteTool; //activates note tool
	QAction *m_loadFoliations; //load field data from a file
	QAction *m_loadLineations; //load field data from a file
	QAction *m_toSVG; //export to svg
	

protected slots:
	//! To capture overridden shortcuts (pause button, etc.)
	void onShortcutTriggered(int);

	//callbacks to update cost
	void setDarkCost() { clearCost(); m_dark->setChecked(true); }
	void setLightCost() { clearCost(); m_light->setChecked(true); }
	void setRGBCost() { clearCost(); m_rgb->setChecked(true); }
	void setGradCost() { clearCost(); m_grad->setChecked(true); }
	void setCurveCost() { clearCost(); m_curve->setChecked(true); }
	void setDistCost() { clearCost(); m_dist->setChecked(true); }
	void setScalarCost() { clearCost(); m_scalar->setChecked(true); }
	void setInvScalarCost() { clearCost(); m_scalar_inv->setChecked(true); }
private:
	//algorithm menu
	QAction *m_dark;
	QAction *m_light;
	QAction *m_rgb;
	QAction *m_grad;
	QAction *m_curve;
	QAction *m_dist;
	QAction *m_scalar;
	QAction *m_scalar_inv;
	
	//disactivates all cost function checkboxes
	void clearCost()
	{
		m_dark->setChecked(false);
		m_light->setChecked(false);
		m_rgb->setChecked(false);
		m_grad->setChecked(false);
		m_curve->setChecked(false);
		m_dist->setChecked(false);
		m_scalar->setChecked(false);
		m_scalar_inv->setChecked(false);
	}
};

#endif
