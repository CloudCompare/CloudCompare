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

#include "ccCompassDlg.h"

//Local
#include "ccGLWindow.h"

//qCC_db
#include <ccLog.h>

//Qt
#include <QEvent>
#include <QKeyEvent>
#include <QApplication>
#include <qmenu.h>
#include <qaction.h>

//system
#include <assert.h>

ccCompassDlg::ccCompassDlg(QWidget* parent/*=0*/)
	: ccOverlayDialog(parent)
	, Ui::compassDlg()
{
	setupUi(this);

	//setup "algorithm" dropdown
	m_cost_algorithm_menu = new QMenu(this);
	m_cost_algorithm_menu->setTitle("Algorithm");
	m_dark = new QAction("Darkness", this); m_dark->setCheckable(true); m_dark->setChecked(true);
	m_light = new QAction("Lightness", this); m_light->setCheckable(true);
	m_rgb = new QAction("RGB similarity", this); m_rgb->setCheckable(true);
	m_grad = new QAction("RGB gradient", this); m_grad->setCheckable(true); //m_grad->setEnabled(true);
	m_curve = new QAction("Curvature", this); m_curve->setCheckable(true); //m_curve->setEnabled(false);
	m_dist = new QAction("Distance", this); m_dist->setCheckable(true);
	m_scalar = new QAction("Scalar field", this); m_scalar->setCheckable(true);
	m_scalar_inv = new QAction("Inverse scalar field", this); m_scalar_inv->setCheckable(true);
	m_recalculate = new QAction("Recalculate selection", this); //used to recalculate selected traces with new cost function
	m_plane_fit = new QAction("Fit planes", this); m_plane_fit->setCheckable(true); m_plane_fit->setChecked(true);

	//setup tool-tips
	m_dark->setToolTip("Traces follow 'dark' points. Good for shadowed fracture traces.");
	m_light->setToolTip("Traces follow 'light' points. Good for thin quartz veins etc.");
	m_rgb->setToolTip("Traces follow points that have a similar color to the start and end points. Useful if structures being traced have a distinct color.");
	m_grad->setToolTip("Traces follow points in neighbourhoods with high colour gradients. Good for contacts.");
	m_curve->setToolTip("Traces follow ridges and valleys. Good for fractures with high relief, especially in combination with \"Darkness\".");
	m_dist->setToolTip("Traces take the shortest euclidean path (how booring...)");
	m_scalar->setToolTip("Use the active scalar field to define path cost (i.e. the path follows low scalar values).");
	m_scalar_inv->setToolTip("Use the inverse of the active scalar field to define path cost (i.e. the path follows high scalar values).");
	m_recalculate->setToolTip("Recalculate the selected traces using the latest cost function");

	//add to menu
	m_cost_algorithm_menu->addAction(m_dark);
	m_cost_algorithm_menu->addAction(m_light);
	m_cost_algorithm_menu->addAction(m_rgb);
	m_cost_algorithm_menu->addAction(m_grad);
	m_cost_algorithm_menu->addAction(m_curve);
	m_cost_algorithm_menu->addAction(m_dist);
	m_cost_algorithm_menu->addAction(m_scalar);
	m_cost_algorithm_menu->addAction(m_scalar_inv);
	m_cost_algorithm_menu->addSeparator();
	m_cost_algorithm_menu->addAction(m_recalculate);

	//add callbacks
	ccCompassDlg::connect(m_dark, SIGNAL(triggered()), this, SLOT(setDarkCost()));
	ccCompassDlg::connect(m_light, SIGNAL(triggered()), this, SLOT(setLightCost()));
	ccCompassDlg::connect(m_rgb, SIGNAL(triggered()), this, SLOT(setRGBCost()));
	ccCompassDlg::connect(m_grad, SIGNAL(triggered()), this, SLOT(setGradCost()));
	ccCompassDlg::connect(m_curve, SIGNAL(triggered()), this, SLOT(setCurveCost()));
	ccCompassDlg::connect(m_dist, SIGNAL(triggered()), this, SLOT(setDistCost()));
	ccCompassDlg::connect(m_scalar, SIGNAL(triggered()), this, SLOT(setScalarCost()));
	ccCompassDlg::connect(m_scalar_inv, SIGNAL(triggered()), this, SLOT(setInvScalarCost()));

	//setup settings menu
	m_settings_menu = new QMenu(this);
	m_plane_fit = new QAction("Fit planes", this); m_plane_fit->setCheckable(true); m_plane_fit->setChecked(false);
	m_showStippled = new QAction("Show stippled", this); m_showStippled->setCheckable(true); m_showStippled->setChecked(true);
	m_showNormals = new QAction("Show normals", this); m_showNormals->setCheckable(true); m_showNormals->setChecked(true);
	m_showNames = new QAction("Show names", this); m_showNames->setCheckable(true); m_showNames->setChecked(false);

	m_plane_fit->setToolTip("If checked, a plane will automatically be fitted to traces matching the criteria defined in the ccCompass description.");
	m_showStippled->setToolTip("If checked, planes will be drawn partially transparent (stippled).");
	m_showNormals->setToolTip("If checked, plane normals will be drawn.");
	m_showNames->setToolTip("If checked, plane orientations will be displayed in the 3D view.");

	m_settings_menu->addAction(m_plane_fit);
	m_settings_menu->addAction(m_showStippled);
	m_settings_menu->addAction(m_showNormals);
	m_settings_menu->addAction(m_showNames);
	m_settings_menu->addSeparator();
	m_settings_menu->addMenu(m_cost_algorithm_menu);
	
	algorithmButton->setPopupMode(QToolButton::InstantPopup);
	algorithmButton->setMenu(m_settings_menu); //add settings menu
	algorithmButton->setEnabled(true);
	
	//setup pair picking menu
	m_pairpicking_menu = new QMenu(this);
	m_research_menu = new QMenu(this);
	m_research_menu->setTitle("Research");

	m_loadFoliations = new QAction("Import foliations...", this);
	m_loadLineations = new QAction("Import lineations...", this);
	m_toSVG = new QAction("Export SVG...", this);
	m_noteTool = new QAction("Add note", this);
	m_pinchTool = new QAction("Add pinch nodes", this);
	m_measure_thickness = new QAction("Measure one-point thickness", this);
	m_measure_thickness_twoPoint = new QAction("Measure two-point thickness", this);
	m_youngerThan = new QAction("Assign \"Younger-Than\" relationship", this); m_youngerThan->setEnabled(false); //todo
	m_follows = new QAction("Assign \"Follows\" relationship", this); m_follows->setEnabled(false);
	m_equivalent = new QAction("Assign \"Equivalent\" relationship", this); m_equivalent->setEnabled(false);
	m_fitPlaneToGeoObject = new QAction("Fit plane to GeoObject", this);
	m_mergeSelected = new QAction("Merge selected GeoObjects", this);
	m_estimateNormals = new QAction("Estimate structure normals", this);

	m_pinchTool->setToolTip("Add Pinch Node objects to record features such as dyke tips or sedimentary units that pinch-out.");
	m_loadFoliations->setToolTip("Converts a point cloud containing points (measurement location) and dip/dip-direction scalar fields to planes.");
	m_loadLineations->setToolTip("Convert a point cloud containing measurement points and trend->plunge scalar fields into foliation objects.");
	m_toSVG->setToolTip("Export the currently visible trace to a SVG vector graphic using an orthographic projection of the current view.");
	m_noteTool->setToolTip("Add short notes to a point in a point cloud for future reference.");
	m_measure_thickness->setToolTip("Select a plane and then a point to measure plane-perpendicular thickness.");
	m_measure_thickness_twoPoint->setToolTip("Measure the plane-perpendicular distance between two points.");
	m_youngerThan->setToolTip("Pick two GeoObjects to assign a \"younger-than\" (i.e. crosscutting, superposition) relationshi.p");
	m_follows->setToolTip("Select two GeoObjects to assign a \"follows\" (i.e. conformable) relationship.");
	m_equivalent->setToolTip("Select two GeoObjects to assign an \"equivalent\" (i.e. coeval) relationship.");
	m_fitPlaneToGeoObject->setToolTip("Calculates best fit planes for the entire upper/lower surfaces of the GeoObject.");
	m_mergeSelected->setToolTip("Merge all selected GeoObjects into a single GeoObject.");
	m_estimateNormals->setToolTip("Estimate trace structure normals with maximum a-postiori plane fitting algorithm.");
	

	m_pairpicking_menu->addAction(m_pinchTool);
	m_pairpicking_menu->addAction(m_measure_thickness);
	m_pairpicking_menu->addAction(m_measure_thickness_twoPoint);
	m_pairpicking_menu->addAction(m_noteTool);
	m_pairpicking_menu->addSeparator();
	//m_pairpicking_menu->addAction(m_youngerThan); //TODO - reenable these once topology code has been fixed
	//m_pairpicking_menu->addAction(m_follows);
	//m_pairpicking_menu->addAction(m_equivalent);
	//m_pairpicking_menu->addSeparator();
	m_pairpicking_menu->addAction(m_fitPlaneToGeoObject);
	m_pairpicking_menu->addAction(m_mergeSelected);
	m_pairpicking_menu->addAction(m_estimateNormals);
	m_pairpicking_menu->addSeparator();
	m_pairpicking_menu->addAction(m_loadFoliations);
	m_pairpicking_menu->addAction(m_loadLineations);
	m_pairpicking_menu->addAction(m_toSVG);
	m_pairpicking_menu->addSeparator();
	m_pairpicking_menu->addMenu(m_research_menu);
	
	
	//Add tools to research menu
	m_recalculateFitPlanes = new QAction("Recalculate fit-planes", this);
	m_toPointCloud = new QAction("Convert to point cloud", this);
	m_distributeSelection = new QAction("Distribute to GeoObjects", this);
	m_estimateP21 = new QAction("Estimate P21 intensity", this);
	m_estimateStrain = new QAction("Estimate strain", this);

	m_recalculateFitPlanes->setToolTip("Recalculates all fit-planes deriving from traces and GeoObjects (but not those calculated with the Plane Tool).");
	m_toPointCloud->setToolTip("Converts the selected GeoObject(s) or individual traces to a point cloud (typically for proximity analysis).");
	m_distributeSelection->setToolTip("Distributes the selected objects into GeoObjects that have matching names.");
	m_estimateP21->setToolTip("Uses structure traces and the mesh octree to measure fracture intensity.");
	m_estimateStrain->setToolTip("Estimate bulk strain tensor from Mode-I dykes and veins with structure normal estimates.");

	m_research_menu->addAction(m_recalculateFitPlanes);
	m_research_menu->addAction(m_estimateP21);
	m_research_menu->addAction(m_estimateStrain);
	m_research_menu->addAction(m_distributeSelection);
	m_research_menu->addAction(m_toPointCloud);
	
	extraModeButton->setPopupMode(QToolButton::InstantPopup);
	extraModeButton->setMenu(m_pairpicking_menu);

	//set background color
	QPalette p;
	p.setColor(backgroundRole(), QColor(240, 240, 240, 200));
	setPalette(p);
	setAutoFillBackground(true);

	//add shortcuts
	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button
	addOverridenShortcut(Qt::Key_Space); //space key also hits the "apply" button (easier for some)
	connect(this, SIGNAL(shortcutTriggered(int)), this, SLOT(onShortcutTriggered(int)));
}

int ccCompassDlg::getCostMode()
{
	int out = 0;
	if (m_dark->isChecked())
		out = out | ccTrace::MODE::DARK;
	if (m_light->isChecked())
		out = out | ccTrace::MODE::LIGHT;
	if (m_rgb->isChecked())
		out = out | ccTrace::MODE::RGB;
	if (m_curve->isChecked())
		out = out | ccTrace::MODE::CURVE;
	if (m_grad->isChecked())
		out = out | ccTrace::MODE::GRADIENT;
	if (m_dist->isChecked())
		out = out | ccTrace::MODE::DISTANCE;
	if (m_scalar->isChecked())
		out = out | ccTrace::MODE::SCALAR;
	if (m_scalar_inv->isChecked())
		out = out | ccTrace::MODE::INV_SCALAR;

	if (out == 0) 
		return ccTrace::MODE::DISTANCE; //default to distance if everything has been unchecked

	return out;
}

bool ccCompassDlg::planeFitMode()
{
	return m_plane_fit->isChecked();
}

void ccCompassDlg::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Return:
		acceptButton->click();
		return;
	case Qt::Key_Space:
		acceptButton->click();
		return;
	case Qt::Key_Escape:
		closeButton->click();
		return;
	default:
		//nothing to do
		break;
	}
}
