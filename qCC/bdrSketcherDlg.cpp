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

#include "bdrSketcherDlg.h"

//Local
#include "ccContourExtractor.h"
#include "ccItemSelectionDlg.h"
#include "ccOrthoSectionGenerationDlg.h"
#include "ccSectionExtractionSubDlg.h"
#include "mainwindow.h"

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include "ccPlane.h"

//qCC_gl
#include <ccGLWindow.h>

//CCLib
#include <ReferenceCloud.h>

//Qt
#include <QCoreApplication>
#include <QInputDialog>
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QMenu>

//GUI
#include <ui_bdrSketcherDlg.h>
#include <bdr2.5DimEditor.h>

//System
#include <cassert>
#include <cmath>

#ifdef USE_STOCKER
#include "stocker_parser.h"
#endif // USE_STOCKER


//default parameters
static const ccColor::Rgb& s_defaultPolylineColor = ccColor::lightCoral;
static const ccColor::Rgb& s_defaultFpNormalColor = ccColor::doderBlue;
static const ccColor::Rgb& s_defaultFpHoleColor = ccColor::skyBlue;
static const ccColor::Rgb& s_defaultContourColor = ccColor::green;
static const ccColor::Rgb& s_defaultEditedPolylineColor = ccColor::green;
static const ccColor::Rgb& s_defaultSelectedPolylineColor = ccColor::red;

constexpr int	s_defaultPolylineWidth = 1;
constexpr int	s_defaultSelectedPolylineWidth = 3;

//default export groups
static unsigned s_polyExportGroupID = 0;
static unsigned s_profileExportGroupID = 0;
static unsigned s_cloudExportGroupID = 0;

//default arrow size
static const PointCoordinateType s_defaultArrowSize = 20;

bdrSketcher::bdrSketcher(QWidget* parent)
	: ccOverlayDialog(parent)
	, m_UI( new Ui::bdrSketcherDlg )
	, m_selectedSO(nullptr)
	, m_state(0)
	, m_editedPoly(nullptr)
	, m_editedPolyVertices(nullptr)
	, m_workingPlane(nullptr)
	, m_currentSOMode(SO_POINT)
{
	m_UI->setupUi(this);

	connect(m_UI->undoToolButton, &QAbstractButton::clicked, this, &bdrSketcher::undo);
	connect(m_UI->validToolButton, &QAbstractButton::clicked, this, &bdrSketcher::apply);
	connect(m_UI->cancelToolButton, &QAbstractButton::clicked, this, &bdrSketcher::cancel);

	//////////////////////////////////////////////////////////////////////////

	//! sketch objects
	///< point
	connect(m_UI->soPointToolButton, &QAbstractButton::clicked, this, [=]() { createSketchObject(SO_POINT); });
	///< polyline
	connect(m_UI->soPolylineToolButton, &QAbstractButton::clicked, this, [=]() { createSketchObject(SO_POLYLINE); });
	///< polygon - center point, radius, edge

	///< circle
	QMenu* menuCircle = new QMenu(m_UI->soCircleToolButton);
	menuCircle->addAction(m_UI->actionSoCircleByCenter);
	menuCircle->addAction(m_UI->actionSoCircleBy3Points);
	m_UI->soCircleToolButton->setMenu(menuCircle);
	///< circle1 - circle: center point and radius
	connect(m_UI->actionSoCircleByCenter, &QAction::triggered, this, [=]() { createSketchObject(SO_CIRCLE_CENTER); });
	///< circle2 - circle by 3 points: draw a circle from 3 points located in the circumference
	connect(m_UI->actionSoCircleBy3Points, &QAction::triggered, this, [=]() { createSketchObject(SO_CIRCLE_3POINT); });

	///< arc
	QMenu* menuArc = new QMenu(m_UI->soArcToolButton);
	menuArc->addAction(m_UI->actionSoArcByCenter);
	menuArc->addAction(m_UI->actionSoArcBy3Points);
	m_UI->soArcToolButton->setMenu(menuArc);
	///< arc1 - arc: center point, radius, start angle, stop angle
	connect(m_UI->actionSoArcByCenter, &QAction::triggered, this, [=]() { createSketchObject(SO_ARC_CENTER); });
	///< arc2 - arc 3point: three points to create an arc
	connect(m_UI->actionSoArcBy3Points, &QAction::triggered, this, [=]() { createSketchObject(SO_ARC_3POINT); });
	
	///< curve
	QMenu* menuCurve = new QMenu(m_UI->soCurveToolButton);
	menuCurve->addAction(m_UI->actionSoCurveCubicBezier);
	menuCurve->addAction(m_UI->actionSoCurveBezier);
	menuCurve->addAction(m_UI->actionSoCurveBSpline);
	m_UI->soCurveToolButton->setMenu(menuCurve);
	///< curve1 - Bezier Curve
	connect(m_UI->actionSoCurveCubicBezier, &QAction::triggered, this, [=]() { createSketchObject(SO_CURVE_BEZIER3); });
	///< curve2 - Cubic Bezier curve
	connect(m_UI->actionSoCurveBezier, &QAction::triggered, this, [=]() { createSketchObject(SO_CURVE_BEZIER); });
	///< curve3 - BSpline
	connect(m_UI->actionSoCurveBSpline, &QAction::triggered, this, [=]() { createSketchObject(SO_CURVE_BSPLINE); });

	///< regular polygon
	connect(m_UI->soNPolyToolButton, &QAbstractButton::clicked, this, [=]() { createSketchObject(SO_NPOLYGON); });
	///< rectangle
	connect(m_UI->soRectangleToolButton, &QAbstractButton::clicked, this, [=]() { createSketchObject(SO_RECTANGLE); });

	//////////////////////////////////////////////////////////////////////////

	connect(m_UI->editingToolButton, &QAbstractButton::toggled, this, &bdrSketcher::enableSelectedSOEditingMode);
	

	connect(m_UI->importFromDBToolButton, &QAbstractButton::clicked, this, &bdrSketcher::doImportPolylinesFromDB);
	//connect(m_UI->vertAxisComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &bdrSketcher::setVertDimension);

	connect(m_UI->exportSectionsToolButton, &QAbstractButton::clicked, this, &bdrSketcher::exportSections);

	//////////////////////////////////////////////////////////////////////////

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space);  //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //cancel current polyline edition
	addOverridenShortcut(Qt::Key_Delete); //delete key to delete the selected polyline
	addOverridenShortcut(Qt::Key_Enter);  //close
	addOverridenShortcut(Qt::Key_C);		// click

	connect(this, &ccOverlayDialog::shortcutTriggered, this, &bdrSketcher::onShortcutTriggered);
	connect(m_UI->saveFootprintInsidetoolButton, &QAbstractButton::clicked, this, &bdrSketcher::exportFootprintInside);
	connect(m_UI->saveFootprintOutsidetoolButton, &QAbstractButton::clicked, this, &bdrSketcher::exportFootprintOutside);

	setTraceViewMode(true);
}

bdrSketcher::~bdrSketcher()
{
	if (m_editedPoly)
	{
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		delete m_editedPoly;
		m_editedPoly = nullptr;
	}
	
	delete m_UI;
}

void bdrSketcher::setVertDimension(int dim)
{
	assert(dim >= 0 && dim < 3);
	if (!m_associatedWin)
		return;

	switch (dim)
	{
	case 0:
		m_associatedWin->setView(CC_RIGHT_VIEW);
		break;
	case 1:
		m_associatedWin->setView(CC_FRONT_VIEW);
		break;
	case 2:
	default:
		m_associatedWin->setView(CC_TOP_VIEW);
		break;
 	}
	m_associatedWin->updateConstellationCenterAndZoom();
}

void bdrSketcher::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Space:
		getCurrentSOButton()->clicked();
		return;

	case Qt::Key_Escape:
		cancelCurrentPolyline();
		return;

	case Qt::Key_Delete:
		deleteSelectedPolyline();
		return;
	case Qt::Key_Enter:
		closeFootprint();
		return;
	case Qt::Key_C:
		// Add point
		return;
	default:
		//nothing to do
		break;
	}
}

void bdrSketcher::echoLeftButtonClicked(int x, int y)
{
	switch (m_currentSOMode)
	{
	case bdrSketcher::SO_POINT:
		break;
	case bdrSketcher::SO_POLYLINE:
		if (m_state & PS_STARTED) {
			addPointToPolyline(x, y);
		}
		else if (m_state == PS_EDITING) {

		}
		break;
	case bdrSketcher::SO_CIRCLE_CENTER:
		break;
	case bdrSketcher::SO_CIRCLE_3POINT:
		break;
	case bdrSketcher::SO_ARC_CENTER:
		break;
	case bdrSketcher::SO_ARC_3POINT:
		break;
	case bdrSketcher::SO_CURVE_BEZIER:
		break;
	case bdrSketcher::SO_CURVE_BEZIER3:
		break;
	case bdrSketcher::SO_CURVE_BSPLINE:
		break;
	case bdrSketcher::SO_NPOLYGON:
		break;
	case bdrSketcher::SO_RECTANGLE:
		break;
	default:
		break;
	}
}

void bdrSketcher::echoRightButtonClicked(int x, int y)
{
	switch (m_currentSOMode)
	{
	case bdrSketcher::SO_POINT:
		break;
	case bdrSketcher::SO_POLYLINE:
		if (m_state & PS_RUNNING) {
			closePolyLine(x, y);
		}
		break;
	case bdrSketcher::SO_CIRCLE_CENTER:
		break;
	case bdrSketcher::SO_CIRCLE_3POINT:
		break;
	case bdrSketcher::SO_ARC_CENTER:
		break;
	case bdrSketcher::SO_ARC_3POINT:
		break;
	case bdrSketcher::SO_CURVE_BEZIER:
		break;
	case bdrSketcher::SO_CURVE_BEZIER3:
		break;
	case bdrSketcher::SO_CURVE_BSPLINE:
		break;
	case bdrSketcher::SO_NPOLYGON:
		break;
	case bdrSketcher::SO_RECTANGLE:
		break;
	default:
		break;
	}
}

void bdrSketcher::echoMouseMoved(int x, int y, Qt::MouseButtons buttons)
{
	switch (m_currentSOMode)
	{
	case bdrSketcher::SO_POINT:
		break;
	case bdrSketcher::SO_POLYLINE:
		if ((m_state & PS_RUNNING) || (m_state & PS_EDITING)) {
			updatePolyLine(x, y, buttons);
		}
		break;
	case bdrSketcher::SO_CIRCLE_CENTER:
		break;
	case bdrSketcher::SO_CIRCLE_3POINT:
		break;
	case bdrSketcher::SO_ARC_CENTER:
		break;
	case bdrSketcher::SO_ARC_3POINT:
		break;
	case bdrSketcher::SO_CURVE_BEZIER:
		break;
	case bdrSketcher::SO_CURVE_BEZIER3:
		break;
	case bdrSketcher::SO_CURVE_BSPLINE:
		break;
	case bdrSketcher::SO_NPOLYGON:
		break;
	case bdrSketcher::SO_RECTANGLE:
		break;
	default:
		break;
	}
}

bool bdrSketcher::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	selectPolyline(nullptr);

	if (oldWin)
	{
		//restore sections original display
		for (auto & section : m_sections)
		{
			if (section.entity)
			{
				if (!section.isInDB)
					oldWin->removeFromOwnDB(section.entity);
				section.entity->setDisplay_recursive(section.originalDisplay);
			}
		}

		//Restore clouds original display
		for (auto & cloud : m_clouds)
		{
			if (cloud.entity)
			{
				if (!cloud.isInDB)
					oldWin->removeFromOwnDB(cloud.entity);
				cloud.entity->setDisplay(cloud.originalDisplay);
			}
		}

		if (m_editedPoly)
		{
			m_editedPoly->setDisplay_recursive(nullptr);
		}

		//auto-close formerly associated window
		if (MainWindow::TheInstance())
		{
			QMdiSubWindow* subWindow = MainWindow::TheInstance()->getMDISubWindow(oldWin);
			if (subWindow)
			{
				subWindow->close();
			}
		}
	}

	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked, this, &bdrSketcher::echoLeftButtonClicked);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked, this, &bdrSketcher::echoRightButtonClicked);
		connect(m_associatedWin, &ccGLWindow::mouseMoved, this, &bdrSketcher::echoMouseMoved);
		connect(m_associatedWin, &ccGLWindow::entitySelectionChanged, this, &bdrSketcher::entitySelected);

		//import sections in current display
		for (auto & section : m_sections)
		{
			if (section.entity)
			{
				section.originalDisplay = section.entity->getDisplay();
				section.entity->setDisplay_recursive(m_associatedWin);
				if (!section.isInDB)
					m_associatedWin->addToOwnDB(section.entity);
			}
		}

		//import clouds in current display
		for (auto & cloud : m_clouds)
		{
			if (cloud.entity)
			{
				cloud.originalDisplay = cloud.entity->getDisplay();
				cloud.entity->setDisplay(m_associatedWin);
				if (!cloud.isInDB)
				{
					m_associatedWin->addToOwnDB(cloud.entity);
				}
			}
		}

		if (m_editedPoly)
		{
			m_editedPoly->setDisplay_recursive(m_associatedWin);
		}

		//update view direction	// TODO: set plane
//		setVertDimension(m_UI->vertAxisComboBox->currentIndex());

		//section extraction only works in orthoraphic mode!
		m_associatedWin->setPerspectiveState(false, true);
	}

	return true;
}

void bdrSketcher::selectPolyline(Section* poly, bool autoRefreshDisplay/*=true*/)
{
	bool redraw = false;

	//deselect previously selected polyline
	if (m_selectedSO && m_selectedSO->entity)
	{
		m_selectedSO->entity->showColors(true);
		m_selectedSO->entity->setColor(s_defaultPolylineColor);
		m_selectedSO->entity->setWidth(s_defaultPolylineWidth);
		redraw = true;
	}

	m_selectedSO = poly;

	//select new polyline (if any)
	if (m_selectedSO)
	{
		m_selectedSO->entity->showColors(true);
		m_selectedSO->entity->setColor(s_defaultSelectedPolylineColor);
		m_selectedSO->entity->setWidth(s_defaultSelectedPolylineWidth);
		m_selectedSO->entity->setSelected(false); //as the window selects it by default (with bounding-box, etc.) and we don't want that
		redraw = true;
	}

	if (redraw && autoRefreshDisplay && m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void bdrSketcher::releasePolyline(Section* section)
{
	if (section && section->entity)
	{
		if (!section->isInDB)
		{
			//remove from display
			if (m_associatedWin)
				m_associatedWin->removeFromOwnDB(section->entity);
			//delete entity
			delete section->entity;
			section->entity = nullptr;
		}
		else
		{
			//restore original display and style
			section->entity->showColors(section->backupColorShown);
			section->entity->setColor(section->backupColor);
			section->entity->setWidth(section->backupWidth);
			section->entity->setDisplay_recursive(section->originalDisplay);
		}
	}
}

void bdrSketcher::deleteSelectedPolyline()
{
	if (!m_selectedSO)
		return;

	Section* selectedPoly = m_selectedSO;

	//deslect polyline before anything
	selectPolyline(nullptr, false);

	releasePolyline(selectedPoly);

	//remove the section from the list
	m_sections.removeOne(*selectedPoly);
	m_undoCount.resize(0);
	m_UI->undoToolButton->setEnabled(false);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void bdrSketcher::entitySelected(ccHObject* entity)
{
	if (!entity)
	{
		return;
	}

	//look if this selected entity corresponds to an active polyline
	for (auto & section : m_sections)
	{
		if (section.entity == entity)
		{
			selectPolyline(&section);
			break;
		}
	}
}

bool bdrSketcher::start()
{
	assert(!m_editedPolyVertices && !m_editedPoly);

	if (!m_associatedWin)
	{
		ccLog::Warning("[Graphical Segmentation Tool] No associated window!");
		return false;
	}

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	//m_associatedWin->updateConstellationCenterAndZoom();	// XYLIU update when set plane // TODO
	updateCloudsBox();

	//enableSketcherEditingMode(true);	// XYLIU NOT START BY DEFAULT

	return ccOverlayDialog::start();
}

void bdrSketcher::removeAllEntities()
{
	reset(false);

	//and we remove the remaining clouds (if any)
	for (auto & cloud : m_clouds)
	{
		if (cloud.entity)
		{
			assert(cloud.isInDB);
			//restore original display
			cloud.entity->setDisplay(cloud.originalDisplay);
		}
	}
	
	m_clouds.clear();
	m_cloudsBox.clear();
}

void bdrSketcher::setTraceViewMode(bool trace_image)
{
	m_trace_image = trace_image;
	if (trace_image) {
		m_UI->importFromDBToolButton->setVisible(true);
		m_UI->exportSectionsToolButton->setVisible(false);
		m_UI->saveFootprintInsidetoolButton->setVisible(false);
		m_UI->saveFootprintOutsidetoolButton->setVisible(false);
		setFixedWidth(360);
	}
	else {
		m_UI->exportSectionsToolButton->setVisible(false);
		m_UI->saveFootprintInsidetoolButton->setVisible(true);
		m_UI->saveFootprintOutsidetoolButton->setVisible(true);
		setFixedWidth(210);
	}
}

void bdrSketcher::SetDestAndGround(ccHObject * dest, double ground)
{
	m_dest_obj = dest;
	m_ground = ground;
}

void bdrSketcher::importEntities(ccHObject::Container entities)
{

}

void bdrSketcher::undo()
{
	if (m_undoCount.empty())
		return;

	size_t count = 0;
	do
	{
		count = m_undoCount.back();
		m_undoCount.pop_back();
	} while (static_cast<int>(count) >= m_sections.size() && !m_undoCount.empty());

	//ask for a confirmation
	if (QMessageBox::question(MainWindow::TheInstance(), "Undo", QString("Remove %1 polylines?").arg(m_sections.size() - count), QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
	{
		//restore undo stack!
		m_undoCount.push_back(count);
		return;
	}

	selectPolyline(nullptr);

	//we remove all polylines after a given point
	{
		while (m_sections.size() > static_cast<int>(count))
		{
			Section& section = m_sections.back();
			releasePolyline(&section);
			m_sections.pop_back();
		}
	}

	//update GUI
	m_UI->exportSectionsToolButton->setEnabled(count != 0);
	m_UI->undoToolButton->setEnabled(!m_undoCount.empty());

	if (m_associatedWin)
		m_associatedWin->redraw();
}

bool bdrSketcher::reset(bool askForConfirmation/*=true*/)
{
	if (m_sections.empty() && m_clouds.empty())
	{
		//nothing to do
		return true;
	}

	if (askForConfirmation)
	{
		//if we found at least one temporary polyline, we display a confirmation message
		for (auto & section : m_sections)
		{
			if (section.entity && !section.isInDB)
			{
				if (QMessageBox::question(MainWindow::TheInstance(), "Reset", "You'll lose all manually defined polylines: are you sure?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
					return false;
				else
					break;
			}
		}
	}

	selectPolyline(nullptr);

	//we remove all polylines
	for (auto & section : m_sections)
	{
		releasePolyline(&section);
	}
	
	m_sections.clear();
	m_undoCount.resize(0);
	m_UI->undoToolButton->setEnabled(false);
	m_UI->exportSectionsToolButton->setEnabled(false);

	//and we remove only temporary clouds
	for (int i = 0; i < m_clouds.size();)
	{
		Cloud& cloud = m_clouds[i];
		if (cloud.entity && !cloud.isInDB)
		{
			if (m_associatedWin)
				m_associatedWin->removeFromOwnDB(cloud.entity);
			delete cloud.entity;
			cloud.entity = nullptr;

			m_clouds.removeAt(i);
		}
		else
		{
			++i;
		}
	}
	
	updateCloudsBox();

	if (m_associatedWin)
		m_associatedWin->redraw();

	return true;
}

void bdrSketcher::stop(bool accepted)
{
	if (m_editedPoly)
	{
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		delete m_editedPoly;
		m_editedPoly = nullptr;
	}
	m_editedPolyVertices = nullptr;
	if (m_dest_obj) m_dest_obj = nullptr;

	enableSketcherEditingMode(false);
	reset(true);

	if (m_associatedWin)
	{
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
	}

	ccOverlayDialog::stop(accepted);
}

void bdrSketcher::updateCloudsBox()
{
	m_cloudsBox.clear();

	for (auto & cloud : m_clouds)
	{
		if (cloud.entity)
			m_cloudsBox += cloud.entity->getOwnBB();
	}
}

bool bdrSketcher::addPolyline(ccPolyline* inputPoly, bool alreadyInDB/*=true*/)
{
	if (!inputPoly)
	{
		assert(false);
		return false;
	}

	for (auto & section : m_sections)
	{
		if (section.entity == inputPoly)
		{
			//cloud already in DB
			return false;
		}
	}

	//convert the polyline to 3D mode if necessary
	if (inputPoly->is2DMode())
	{
		//viewing parameters (for conversion from 2D to 3D)
		ccGLCameraParameters camera;
		m_associatedWin->getGLCameraParameters(camera);
		const double half_w = camera.viewport[2] / 2.0;
		const double half_h = camera.viewport[3] / 2.0;

		//working dimension
		int vertDim = 2;
		assert(vertDim >= 0 && vertDim < 3);

		//get default altitude from the cloud(s) bouding-box
		PointCoordinateType defaultZ = IMAGE_MARKER_DISPLAY_Z;//! XYLIU for image in 2d (3d fake)
		if (m_cloudsBox.isValid())
		{
			defaultZ = m_cloudsBox.maxCorner()[vertDim];
		}

		//duplicate polyline //! TODO: WHY DUPLICATE?? - TO GET A CLEAN VERTIVES
		ccPolyline* duplicatePoly = new ccPolyline(nullptr);
		ccPointCloud* duplicateVertices = nullptr;
		if (duplicatePoly->initWith(duplicateVertices, *inputPoly))
		{
			assert(duplicateVertices);
			for (unsigned i = 0; i < duplicateVertices->size(); ++i)
			{
				CCVector3& P = const_cast<CCVector3&>(*duplicateVertices->getPoint(i));
				CCVector3d Pd(half_w + P.x, half_h + P.y, 0/*P.z*/);
				CCVector3d Q3D;
				camera.unproject(Pd, Q3D);
				P = CCVector3::fromArray(Q3D.u);
				P.u[vertDim] = defaultZ;

				//! project to working plane
			}

			duplicateVertices->invalidateBoundingBox();
			duplicateVertices->setEnabled(false);
			duplicatePoly->set2DMode(false);
			duplicatePoly->setDisplay_recursive(inputPoly->getDisplay());
			duplicatePoly->setName(inputPoly->getName());
			duplicatePoly->setGlobalScale(inputPoly->getGlobalScale());
			duplicatePoly->setGlobalShift(inputPoly->getGlobalShift());

			if (!alreadyInDB)
				delete inputPoly;
			else
				alreadyInDB = false;
			inputPoly = duplicatePoly;
		}
		else
		{
			delete duplicatePoly;
			duplicatePoly = nullptr;

			ccLog::Error("Not enough memory to import polyline!");
			return false;
		}
	}

	//add polyline to the 'sections' set
	//(all its parameters will be backuped!)
	m_sections.push_back(Section(inputPoly, alreadyInDB));
	m_UI->exportSectionsToolButton->setEnabled(true);

	//apply default look
	inputPoly->setEnabled(true);
	inputPoly->setVisible(true);
	inputPoly->showColors(true);
	inputPoly->setColor(s_defaultPolylineColor);
	inputPoly->setWidth(s_defaultPolylineWidth);

	//add to display
	if (m_associatedWin)
	{
		inputPoly->setDisplay_recursive(m_associatedWin);
		if (!alreadyInDB)
			m_associatedWin->addToOwnDB(inputPoly);
	}

	return true;
}

static bool s_mixedShiftAndScaleInfo = false;

bool bdrSketcher::addCloud(ccGenericPointCloud* inputCloud, bool alreadyInDB/*=true*/)
{
	assert(inputCloud);

	if (m_clouds.empty())
		s_mixedShiftAndScaleInfo = false;

	for (CloudPool::iterator it = m_clouds.begin(); it != m_clouds.end(); ++it)
	{
		Cloud& cloud = *it;
		if (cloud.entity == inputCloud)
		{
			//cloud already in DB
			return false;
		}

		//test (on the first cloud) that the global shift & scale info is the same
		if (!s_mixedShiftAndScaleInfo && it == m_clouds.begin())
		{
			if (cloud.entity->getGlobalScale() != inputCloud->getGlobalScale()
				|| (cloud.entity->getGlobalShift() - inputCloud->getGlobalShift()).norm() < ZERO_TOLERANCE)
			{
				ccLog::Warning("[bdrSketcher] Clouds have different shift & scale information! Only the first one will be used");
				s_mixedShiftAndScaleInfo = true;
			}
		}
	}

	m_clouds.push_back(Cloud(inputCloud, alreadyInDB));
	if (m_associatedWin)
	{
		inputCloud->setDisplay(m_associatedWin);
		if (!alreadyInDB)
			m_associatedWin->addToOwnDB(inputCloud);
	}

	return true;
}

void bdrSketcher::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	Q_UNUSED( buttons );
	
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (m_state & PS_EDITING) {
		//! edit the polyline



		return;
	}

	//process not started yet?
	if ((m_state & PS_RUNNING) == 0)
		return;

	if (!m_editedPoly)
		return;

	unsigned vertCount = m_editedPolyVertices->size();
	if (vertCount < 2)
		return;

	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	//we replace last point by the current one
	CCVector3* lastP = const_cast<CCVector3*>(m_editedPolyVertices->getPointPersistentPtr(vertCount - 1));
	*lastP = P;

	m_associatedWin->redraw(true, false);
}

void bdrSketcher::addPointToPolyline(int x, int y)
{
	if ((m_state & PS_STARTED) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	if (!m_editedPoly)
	{
		assert(!m_editedPolyVertices);
		m_editedPolyVertices = new ccPointCloud("vertices");
		m_editedPoly = new ccPolyline(m_editedPolyVertices);
		m_editedPoly->setForeground(true);
		m_editedPoly->setColor(s_defaultEditedPolylineColor);
		m_editedPoly->showColors(true);
		m_editedPoly->set2DMode(true);
		//copy (first) cloud shift & scale info!
		if (!m_clouds.empty() && m_clouds.front().entity)
		{
			ccGenericPointCloud* cloud = m_clouds.front().entity;
			m_editedPoly->setGlobalScale(cloud->getGlobalScale());
			m_editedPoly->setGlobalShift(cloud->getGlobalShift());
		}
		m_editedPoly->addChild(m_editedPolyVertices);
		m_associatedWin->addToOwnDB(m_editedPoly);
	}

	unsigned vertCount = m_editedPolyVertices->size();

	//clicked point (2D)
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);	// displayed in foreground
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	//start new polyline?
	if (((m_state & PS_RUNNING) == 0) || vertCount == 0)
	{
		//reset state
		m_state = (PS_STARTED | PS_RUNNING);
		//reset polyline
		m_editedPolyVertices->clear();
		if (!m_editedPolyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_editedPolyVertices->addPoint(P);
		m_editedPolyVertices->addPoint(P);
		m_editedPoly->clear();
		if (!m_editedPoly->addPointIndex(0, 2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
	}
	else //next points
	{
		if (!m_editedPolyVertices->reserve(vertCount + 1))
		{
			ccLog::Error("Out of memory!");
			return;
		}

		//we replace last point by the current one
		assert(vertCount >= 2);
		CCVector3* lastP = const_cast<CCVector3*>(m_editedPolyVertices->getPointPersistentPtr(vertCount - 1));
		CCVector3* lastQ = const_cast<CCVector3*>(m_editedPolyVertices->getPointPersistentPtr(vertCount - 2));
		PointCoordinateType tipLength = (*lastQ - *lastP).norm();
		*lastP = P;
		//and add a new (equivalent) one
		m_editedPolyVertices->addPoint(P);
		if (!m_editedPoly->addPointIndex(vertCount))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		PointCoordinateType defaultArrowSize = std::min(s_defaultArrowSize, tipLength / 2);
		m_editedPoly->showArrow(true, vertCount - 1, defaultArrowSize);
	}

	m_associatedWin->redraw(true, false);
}

void bdrSketcher::addCurrentPointToPolyline()
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	//process not started yet?
	if ((m_state & PS_RUNNING) == 0)
		return;

	if (!m_editedPoly)
		return;

	unsigned vertCount = m_editedPolyVertices->size();
	if (vertCount < 2)
		return;

	//we replace last point by the current one
	CCVector3* lastP = const_cast<CCVector3*>(m_editedPolyVertices->getPointPersistentPtr(vertCount - 1));
	addPointToPolyline(lastP->x, lastP->y);
}

void bdrSketcher::closeFootprint()
{

}

void bdrSketcher::closePolyLine(int, int)
{
	//only in RUNNING mode
	if ((m_state & PS_RUNNING) == 0 || !m_editedPoly)
		return;

	assert(m_editedPoly);
	unsigned vertCount = m_editedPoly->size();
	if (vertCount < 3)
	{
		m_editedPoly->clear();
		m_editedPolyVertices->clear();
	}
	else
	{
		//remove last point!
		m_editedPoly->resize(vertCount - 1); //can't fail --> smaller
		CCVector3* end_point = const_cast<CCVector3*>(m_editedPolyVertices->getPoint(vertCount - 2));
		CCVector3* start_point = const_cast<CCVector3*>(m_editedPolyVertices->getPoint(0));
		if (((*start_point) - (*end_point)).norm2() < 10) {
			*end_point = *start_point;
//			m_editedPoly->setPointIndex(vertCount - 2, 0);
			m_editedPoly->resize(vertCount - 2);
			m_editedPoly->setClosed(true);
		}

		//remove polyline from the 'temporary' world
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		//set default display style
		m_editedPoly->showColors(true);
		m_editedPoly->setColor(s_defaultPolylineColor);
		m_editedPoly->setWidth(s_defaultPolylineWidth);
		if (!m_clouds.isEmpty())
			m_editedPoly->setDisplay_recursive(m_associatedWin); //set the same 'default' display as the cloud
		m_editedPoly->setName(QString("Polyline #%1").arg(m_sections.size() + 1));
		//save polyline
		if (!addPolyline(m_editedPoly, false))
		{
			//if something went wrong, we have to remove the polyline manually
			delete m_editedPoly;
		}
		m_editedPoly = nullptr;
		m_editedPolyVertices = nullptr;

		enableSketcherEditingMode(false); return;	//XYLIU stop now 
	}
	
	//stop
	m_state &= (~PS_RUNNING);

	if (m_associatedWin)
	{
		m_associatedWin->redraw(true, false);
	}
}

void bdrSketcher::cancelCurrentPolyline()
{
	if ((m_state & PS_STARTED) == 0
		|| !m_editedPoly)
	{
		return;
	}

	assert(m_editedPolyVertices);
	m_editedPoly->clear();
	m_editedPolyVertices->clear();

	//stop
	m_state &= (~PS_RUNNING);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void bdrSketcher::enableSketcherEditingMode(bool state)
{
	if (!m_associatedWin)
		return;

	if (!state/*=activate pause mode*/)
	{
		//select the last polyline by default (if any)
		if (!m_sections.empty() && !m_sections.back().isInDB)
			selectPolyline(&m_sections.back());

		m_state = PS_PAUSED;

		if (m_editedPoly && m_editedPolyVertices)
		{
			m_editedPoly->clear();
			m_editedPolyVertices->clear();
		}
		m_associatedWin->setInteractionMode(ccGLWindow::PAN_ONLY());
		m_associatedWin->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false, 0, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->setPickingMode(ccGLWindow::ENTITY_PICKING); //to be able to select polylines!
	}
	else
	{
		//deselect all currently selected polylines
		selectPolyline(nullptr);

		//set new 'undo' step
		addUndoStep();

		m_state = PS_STARTED;

		m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		m_associatedWin->displayNewMessage("Section edition mode", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->displayNewMessage("Left click: add section points / Right click: stop", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	}

	//update mini-GUI
	QToolButton* tb = getCurrentSOButton();
	tb->blockSignals(true);
	tb->setChecked(state);
	m_UI->frame->setEnabled(!state);
	tb->blockSignals(false);

	m_associatedWin->redraw();
}

void bdrSketcher::createSketchObject(SketchObjectMode mode)
{
	if (!m_associatedWin)
		return;

	if (m_state & PS_STARTED)
	{
		enableSketcherEditingMode(false);
	}
	else {
		m_currentSOMode = mode;
		enableSketcherEditingMode(true);

		// FIXME: the checked state is wrong
		switch (m_currentSOMode)
		{
		case bdrSketcher::SO_POINT:
		case bdrSketcher::SO_POLYLINE:
			break;
		case bdrSketcher::SO_CIRCLE_CENTER:
			m_UI->soCircleToolButton->setDefaultAction(m_UI->actionSoCircleByCenter);
			break;
		case bdrSketcher::SO_CIRCLE_3POINT:
			m_UI->soCircleToolButton->setDefaultAction(m_UI->actionSoCircleBy3Points);
			break;
		case bdrSketcher::SO_ARC_CENTER:
			m_UI->soArcToolButton->setDefaultAction(m_UI->actionSoArcByCenter);
			break;
		case bdrSketcher::SO_ARC_3POINT:
			m_UI->soArcToolButton->setDefaultAction(m_UI->actionSoArcBy3Points);
			break;
		case bdrSketcher::SO_CURVE_BEZIER:
			m_UI->soCurveToolButton->setDefaultAction(m_UI->actionSoCurveBezier);
			break;
		case bdrSketcher::SO_CURVE_BEZIER3:
			m_UI->soCurveToolButton->setDefaultAction(m_UI->actionSoCurveCubicBezier);
			break;
		case bdrSketcher::SO_CURVE_BSPLINE:
			m_UI->soCurveToolButton->setDefaultAction(m_UI->actionSoCurveBSpline);
			break;
		case bdrSketcher::SO_NPOLYGON:
		case bdrSketcher::SO_RECTANGLE:
			break;
		default:
			break;
		}
	}
}

QToolButton * bdrSketcher::getCurrentSOButton()
{
	switch (m_currentSOMode)
	{
	case SO_POINT:
		return m_UI->soPointToolButton;
	case SO_POLYLINE:
		return m_UI->soPolylineToolButton;
	case SO_CIRCLE_CENTER:
	case SO_CIRCLE_3POINT:
		return m_UI->soCircleToolButton;
	case SO_ARC_CENTER:
	case SO_ARC_3POINT:
		return m_UI->soArcToolButton;
	case SO_CURVE_BEZIER:
	case SO_CURVE_BEZIER3:
	case SO_CURVE_BSPLINE:
		return m_UI->soCurveToolButton;
	case SO_NPOLYGON:
		return m_UI->soNPolyToolButton;
	case SO_RECTANGLE:
		return m_UI->soRectangleToolButton;
	default:
		assert(false);
		break;
	}
	return nullptr;
}

void bdrSketcher::enableSelectedSOEditingMode(bool state)
{
	if (state && m_selectedSO) {
		m_state = PS_EDITING;
		m_associatedWin->setPickingMode(ccGLWindow::POINT_PICKING);
	}
	else {
		m_state = PS_PAUSED;
		m_associatedWin->setPickingMode(ccGLWindow::ENTITY_PICKING);
	}
}

void bdrSketcher::addUndoStep()
{
	if (m_undoCount.empty() || (static_cast<int>(m_undoCount.back()) < m_sections.size()))
	{
		m_undoCount.push_back(m_sections.size());
		m_UI->undoToolButton->setEnabled(true);
	}
}

void bdrSketcher::doImportPolylinesFromDB()
{
	//! from file
	if (m_trace_image) {
		
		return;
	}

	MainWindow* mainWindow = MainWindow::TheInstance();
	if (!mainWindow)
		return;

	ccHObject* root = mainWindow->dbRootObject(mainWindow->getCurrentDB());
	ccHObject::Container polylines;
	if (root)
	{
		root->filterChildren(polylines, true, CC_TYPES::POLY_LINE);
	}

	if (!polylines.empty())
	{
		std::vector<int> indexes;
		if (!ccItemSelectionDlg::SelectEntities(polylines, indexes, this))
		{
			return;
		}

		//set new 'undo' step
		addUndoStep();

		enableSketcherEditingMode(false);
		
		for (int index : indexes)
		{
			assert(index >= 0 && index < static_cast<int>(polylines.size()));
			assert(polylines[index]->isA(CC_TYPES::POLY_LINE));
			
			ccPolyline* poly = static_cast<ccPolyline*>(polylines[index]);
			addPolyline(poly, true);
		}
		
		//auto-select the last one
		if (!m_sections.empty())
			selectPolyline(&(m_sections.back()));
		if (m_associatedWin)
			m_associatedWin->redraw();
	}
	else
	{
		ccLog::Error("No polyline in DB!");
	}
}

void bdrSketcher::cancel()
{
	reset(false);

	stop(false);
}

void bdrSketcher::apply()
{
	if (!reset(true))
		return;

	stop(true);
}

ccHObject* bdrSketcher::getExportGroup(unsigned& defaultGroupID, const QString& defaultName)
{
	MainWindow* mainWin = MainWindow::TheInstance();
	ccHObject* root = mainWin ? mainWin->dbRootObject(mainWin->getCurrentDB()) : nullptr;
	if (!root)
	{
		ccLog::Warning("Internal error (no MainWindow or DB?!)");
		assert(false);
		return nullptr;
	}

	ccHObject* destEntity = (defaultGroupID != 0 ? root->find(defaultGroupID) : nullptr);
	if (!destEntity)
	{
		destEntity = new ccHObject(defaultName);
		//assign default display
		for (auto & cloud : m_clouds)
		{
			if (cloud.entity)
			{
				destEntity->setDisplay_recursive(cloud.originalDisplay);
				break;
			}
		}
		mainWin->addToDB_Main(destEntity);
		defaultGroupID = destEntity->getUniqueID();
	}
	return destEntity;
}

void bdrSketcher::exportSections()
{
	if (m_sections.empty())
		return;

	//we only export 'temporary' objects
	unsigned exportCount = 0;

	for (auto & section : m_sections)
	{
		if (section.entity && !section.isInDB)
			++exportCount;
	}

	if (!exportCount)
	{
		//nothing to do
		ccLog::Warning("[bdrSketcher] All active sections are already in DB");
		return;
	}

	ccHObject* destEntity = getExportGroup(s_polyExportGroupID, "Exported sections");
	assert(destEntity);

	MainWindow* mainWin = MainWindow::TheInstance();

	//export entites
	for (auto & section : m_sections)
	{
		if (section.entity && !section.isInDB)
		{
			destEntity->addChild(section.entity);
			section.isInDB = true;
			section.entity->setDisplay_recursive(destEntity->getDisplay());
			mainWin->addToDB_Build(section.entity, false, false);
		}
	}

	ccLog::Print(QString("[bdrSketcher] %1 sections exported").arg(exportCount));
}

void bdrSketcher::exportFootprints()
{
	if (m_sections.empty())
		return;

	//we only export 'temporary' objects
	unsigned exportCount = 0;
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			if (section.entity && !section.isInDB)
				++exportCount;
		}
	}

	if (!exportCount)
	{
		//nothing to do
		ccLog::Warning("[bdrSketcher] All active sections are already in DB");
		return;
	}

	MainWindow* mainWin = MainWindow::TheInstance();

	if (!m_dest_obj) {
		return;
	}

	//export entities
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			if (section.entity && !section.isInDB) {
				StFootPrint* duplicatePoly = new StFootPrint(0);
				ccPointCloud* duplicateVertices = 0;

				int biggest_number = GetMaxNumberExcludeChildPrefix(m_dest_obj, BDDB_FOOTPRINT_PREFIX);
				QString cur_name = BDDB_FOOTPRINT_PREFIX + QString::number(biggest_number + 1);
				if (duplicatePoly->initWith(duplicateVertices, *section.entity))
				{
					duplicatePoly->setAssociatedCloud(duplicateVertices);
					assert(duplicateVertices);

					if (section.type != POLYLINE_OPEN) {
						stocker::Contour2d stocker_points;
						for (unsigned i = 0; i < duplicateVertices->size(); ++i) {
							stocker_points.push_back(stocker::parse_xy(*duplicateVertices->getPoint(i)));
						}
						if (section.type == FOOTPRINT_NORMAL) {
							if (!stocker::IsCounterClockWise(stocker_points)) {
								if (duplicatePoly->reverseVertexOrder()) {
									duplicatePoly->setHoleState(false);
								}
								else return;
							}
						}
						else if(section.type == FOOTPRINT_HOLE) {
							if (stocker::IsCounterClockWise(stocker_points)) {
								if (duplicatePoly->reverseVertexOrder()) {
									duplicatePoly->setHoleState(true);
								}
								else return;
							}
						}
					}
					
					//duplicateVertices->invalidateBoundingBox();
					duplicateVertices->setEnabled(false);
					duplicatePoly->set2DMode(false);
					duplicatePoly->setDisplay_recursive(m_dest_obj->getDisplay());
					duplicatePoly->setName(cur_name);
					duplicatePoly->setGlobalScale(section.entity->getGlobalScale());
					duplicatePoly->setGlobalShift(section.entity->getGlobalShift());
					duplicatePoly->setBottom(m_ground);
					duplicatePoly->setTop(m_ground);
					duplicatePoly->setHeight(m_ground);

					section.entity = duplicatePoly;

				}
				else {
					delete duplicatePoly;
					duplicatePoly = 0;

					ccLog::Error("Not enough memory to export polyline!");
					return;
				}

				section.isInDB = true;
				m_dest_obj->addChild(duplicatePoly);
				if (m_trace_image) {
					mainWin->addToDB_Image(duplicatePoly, false, false);
				}
				else {
					mainWin->addToDB_Build(duplicatePoly, false, false);
				}
				
			}
		}
	}

	ccLog::Print(QString("[FootPrint Extraction] %1 footprints exported").arg(exportCount));
}

void bdrSketcher::exportFootprintInside()
{
	if (m_trace_image) {
		return;
	}

	if (m_sections.empty())
		return;

	//we only export 'temporary' objects
	unsigned exportCount = 0;
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			if (section.entity && !section.isInDB)
				++exportCount;
		}
	}

	if (!exportCount)
	{
		//nothing to do
		ccLog::Warning("[bdrSketcher] All active sections are already in DB");
		return;
	}

	MainWindow* mainWin = MainWindow::TheInstance();
	
	if (!m_dest_obj) {
		return;
	}

	//export entities
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			if (section.entity && !section.isInDB) {
				StFootPrint* duplicatePoly = new StFootPrint(0);
				ccPointCloud* duplicateVertices = 0;

				int biggest_number = GetMaxNumberExcludeChildPrefix(m_dest_obj, BDDB_FOOTPRINT_PREFIX);
				QString cur_name = BDDB_FOOTPRINT_PREFIX + QString::number(biggest_number + 1);
				if (duplicatePoly->initWith(duplicateVertices, *section.entity))
				{
					duplicatePoly->setAssociatedCloud(duplicateVertices);
					assert(duplicateVertices);
					stocker::Contour2d stocker_points;
					for (unsigned i = 0; i < duplicateVertices->size(); ++i) {
						stocker_points.push_back(stocker::parse_xy(*duplicateVertices->getPoint(i)));
					}
					if (!stocker::IsCounterClockWise(stocker_points)) {
						if (duplicatePoly->reverseVertexOrder()) {
							duplicatePoly->setHoleState(false);
						}
						else return;
					}

					//duplicateVertices->invalidateBoundingBox();
					duplicateVertices->setEnabled(false);
					duplicatePoly->set2DMode(false);
					duplicatePoly->setDisplay_recursive(m_dest_obj->getDisplay());
					duplicatePoly->setName(cur_name);
					duplicatePoly->setGlobalScale(section.entity->getGlobalScale());
					duplicatePoly->setGlobalShift(section.entity->getGlobalShift());
					duplicatePoly->setBottom(m_ground);
					duplicatePoly->setTop(m_ground);
					duplicatePoly->setHeight(m_ground);
					section.entity = duplicatePoly;
				}
				else {
					delete duplicatePoly;
					duplicatePoly = 0;

					ccLog::Error("Not enough memory to export polyline!");
					return;
				}
				
				section.isInDB = true;
				m_dest_obj->addChild(duplicatePoly);
				mainWin->addToDB_Build(duplicatePoly, false, false);
			}
		}
	}

	ccLog::Print(QString("[FootPrint Extraction] %1 footprints exported").arg(exportCount));
}

void bdrSketcher::exportFootprintOutside()
{
	if (m_trace_image) {
		return;
	}
	if (m_sections.empty())
		return;

	//we only export 'temporary' objects
	unsigned exportCount = 0;
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			if (section.entity && !section.isInDB)
				++exportCount;
		}
	}

	if (!exportCount)
	{
		//nothing to do
		ccLog::Warning("[bdrSketcher] All active sections are already in DB");
		return;
	}

	MainWindow* mainWin = MainWindow::TheInstance();

	if (!m_dest_obj) {
		return;
	}

	//export entites
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			if (section.entity && !section.isInDB) {
				StFootPrint* duplicatePoly = new StFootPrint(0);
				ccPointCloud* duplicateVertices = 0;

				int biggest_number = GetMaxNumberExcludeChildPrefix(m_dest_obj, BDDB_FOOTPRINT_PREFIX);
				QString cur_name = BDDB_FOOTPRINT_PREFIX + QString::number(biggest_number + 1);
				if (duplicatePoly->initWith(duplicateVertices, *section.entity))
				{
					duplicatePoly->setAssociatedCloud(duplicateVertices);
					assert(duplicateVertices);
					stocker::Contour2d stocker_points;
					for (unsigned i = 0; i < duplicateVertices->size(); ++i) {
						stocker_points.push_back(stocker::parse_xy(*duplicateVertices->getPoint(i)));
					}
					if (stocker::IsCounterClockWise(stocker_points)) {
						if (duplicatePoly->reverseVertexOrder()) {
							duplicatePoly->setHoleState(true);
						}
						else return;
					}

					//duplicateVertices->invalidateBoundingBox();
					duplicateVertices->setEnabled(false);
					duplicatePoly->set2DMode(false);
					duplicatePoly->setDisplay_recursive(m_dest_obj->getDisplay());
					duplicatePoly->setName(cur_name);
					duplicatePoly->setGlobalScale(section.entity->getGlobalScale());
					duplicatePoly->setGlobalShift(section.entity->getGlobalShift());
					duplicatePoly->setBottom(m_ground);
					duplicatePoly->setHeight(m_ground);
					duplicatePoly->setTop(m_ground);
					section.entity = duplicatePoly;
				}
				else {
					delete duplicatePoly;
					duplicatePoly = 0;

					ccLog::Error("Not enough memory to export polyline!");
					return;
				}

				section.isInDB = true;
				m_dest_obj->addChild(duplicatePoly);
				mainWin->addToDB_Build(duplicatePoly, false, false);
			}
		}
	}

	ccLog::Print(QString("[FootPrint Extraction] %1 footprints exported").arg(exportCount));
}
