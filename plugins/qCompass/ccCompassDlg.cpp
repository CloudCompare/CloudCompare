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

	//setup combo box
	categoryBox->clear();
	categoryBox->addItems(QStringList() << "S0 (Bedding)" << "S1" << "S2" << "L1" << "L2" << "Custom..."); //default items

	//setup "algorithm" dropdown
	m_cost_algorithm_menu = new QMenu();
	m_dark = new QAction("Darkness", this); m_dark->setCheckable(true); m_dark->setChecked(true);
	m_light = new QAction("Lightness", this); m_light->setCheckable(true);
	m_rgb = new QAction("RGB Similarity", this); m_rgb->setCheckable(true);
	m_grad = new QAction("RGB Gradient", this); m_grad->setCheckable(true); //m_grad->setEnabled(true);
	m_curve = new QAction("Curvature", this); m_curve->setCheckable(true); //m_curve->setEnabled(false);
	m_dist = new QAction("Distance", this); m_dist->setCheckable(true);
	m_scalar = new QAction("Scalar Field", this); m_scalar->setCheckable(true);
	m_scalar_inv = new QAction("Inverse Scalar Field", this); m_scalar_inv->setCheckable(true);
	m_plane_fit = new QAction("Fit Planes", this); m_plane_fit->setCheckable(true); m_plane_fit->setChecked(true);

	//setup tool-tips
	m_dark->setToolTip("Traces follow 'dark' points. Good for shadowed fracture traces.");
	m_light->setToolTip("Traces follow 'light' points. Good for thin quartz veins etc.");
	m_rgb->setToolTip("Traces follow points that have a similar color to the start and end points. Useful if structures being traced have a distinct color.");
	m_grad->setToolTip("Traces follow points in neighbourhoods with high colour gradients. Good for contacts.");
	m_curve->setToolTip("Traces follow ridges and valleys. Good for fractures with high relief, especially in combination with \"Darkness\".");
	m_dist->setToolTip("Traces take the shortest euclidean path (how booring...)");
	m_scalar->setToolTip("Use the active scalar field to define path cost (i.e. the path follows low scalar values).");
	m_scalar_inv->setToolTip("Use the inverse of the active scalar field to define path cost (i.e. the path follows high scalar values).");
	m_plane_fit->setToolTip("If checked, a plane will automatically be fitted to traces matching the criteria defined in the ccCompass description.");
	
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
	m_cost_algorithm_menu->addAction(m_plane_fit);

	algorithmButton->setPopupMode(QToolButton::InstantPopup);
	//N.B. the menu is added to the algorithm button in the functions that are called with "mode" (plane, trace, lineation) changes - see ccCompass

	//set background color
	QPalette p;
	p.setColor(backgroundRole(), QColor(240, 240, 240, 200));
	setPalette(p);
	setAutoFillBackground(true);
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