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

#include "ccSectionExtractionTool.h"

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

//qCC_gl
#include <ccGLWindow.h>

//CCLib
#include <ReferenceCloud.h>

//Qt
#include <QCoreApplication>
#include <QInputDialog>
#include <QMdiSubWindow>
#include <QMessageBox>

//GUI
#include <ui_sectionExtractionDlg.h>

//System
#include <cassert>
#include <cmath>

//default parameters
static const ccColor::Rgb& s_defaultPolylineColor = ccColor::magenta;
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

ccSectionExtractionTool::ccSectionExtractionTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, m_UI( new Ui::SectionExtractionDlg )
	, m_selectedPoly(nullptr)
	, m_state(0)
	, m_editedPoly(nullptr)
	, m_editedPolyVertices(nullptr)
{
	m_UI->setupUi(this);

	connect(m_UI->undoToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::undo);
	connect(m_UI->validToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::apply);
	connect(m_UI->cancelToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::cancel);
	connect(m_UI->polylineToolButton, &QAbstractButton::toggled, this, &ccSectionExtractionTool::enableSectionEditingMode);
	connect(m_UI->importFromDBToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::doImportPolylinesFromDB);
	connect(m_UI->vertAxisComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ccSectionExtractionTool::setVertDimension);

	connect(m_UI->generateOrthoSectionsToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::generateOrthoSections);
	connect(m_UI->extractPointsToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::extractPoints);
	connect(m_UI->unfoldToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::unfoldPoints);
	connect(m_UI->exportSectionsToolButton, &QAbstractButton::clicked, this, &ccSectionExtractionTool::exportSections);

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space);  //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //cancel current polyline edition
	addOverridenShortcut(Qt::Key_Delete); //delete key to delete the selected polyline

	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccSectionExtractionTool::onShortcutTriggered);
}

ccSectionExtractionTool::~ccSectionExtractionTool()
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

void ccSectionExtractionTool::setVertDimension(int dim)
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

void ccSectionExtractionTool::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Space:
		m_UI->polylineToolButton->toggle();
		return;

	case Qt::Key_Escape:
		cancelCurrentPolyline();
		return;

	case Qt::Key_Delete:
		deleteSelectedPolyline();
		return;

	default:
		//nothing to do
		break;
	}
}

bool ccSectionExtractionTool::linkWith(ccGLWindow* win)
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
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked, this, &ccSectionExtractionTool::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked, this, &ccSectionExtractionTool::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved, this, &ccSectionExtractionTool::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::entitySelectionChanged, this, &ccSectionExtractionTool::entitySelected);

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

		//update view direction
		setVertDimension(m_UI->vertAxisComboBox->currentIndex());

		//section extraction only works in orthoraphic mode!
		m_associatedWin->setPerspectiveState(false, true);
	}

	return true;
}

void ccSectionExtractionTool::selectPolyline(Section* poly, bool autoRefreshDisplay/*=true*/)
{
	bool redraw = false;

	//deselect previously selected polyline
	if (m_selectedPoly && m_selectedPoly->entity)
	{
		m_selectedPoly->entity->showColors(true);
		m_selectedPoly->entity->setColor(s_defaultPolylineColor);
		m_selectedPoly->entity->setWidth(s_defaultPolylineWidth);
		redraw = true;
	}

	m_selectedPoly = poly;

	//select new polyline (if any)
	if (m_selectedPoly)
	{
		m_selectedPoly->entity->showColors(true);
		m_selectedPoly->entity->setColor(s_defaultSelectedPolylineColor);
		m_selectedPoly->entity->setWidth(s_defaultSelectedPolylineWidth);
		m_selectedPoly->entity->setSelected(false); //as the window selects it by default (with bounding-box, etc.) and we don't want that
		redraw = true;
	}

	if (redraw && autoRefreshDisplay && m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	m_UI->generateOrthoSectionsToolButton->setEnabled(m_selectedPoly != nullptr);
	m_UI->unfoldToolButton->setEnabled(m_selectedPoly != nullptr);
}

void ccSectionExtractionTool::releasePolyline(Section* section)
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

void ccSectionExtractionTool::deleteSelectedPolyline()
{
	if (!m_selectedPoly)
		return;

	Section* selectedPoly = m_selectedPoly;

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

void ccSectionExtractionTool::entitySelected(ccHObject* entity)
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

bool ccSectionExtractionTool::start()
{
	assert(!m_editedPolyVertices && !m_editedPoly);

	if (!m_associatedWin)
	{
		ccLog::Warning("[Graphical Segmentation Tool] No associated window!");
		return false;
	}

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->updateConstellationCenterAndZoom();
	updateCloudsBox();

	enableSectionEditingMode(true);

	return ccOverlayDialog::start();
}

void ccSectionExtractionTool::removeAllEntities()
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

void ccSectionExtractionTool::undo()
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
	m_UI->extractPointsToolButton->setEnabled(count != 0);
	m_UI->undoToolButton->setEnabled(!m_undoCount.empty());

	if (m_associatedWin)
		m_associatedWin->redraw();
}

bool ccSectionExtractionTool::reset(bool askForConfirmation/*=true*/)
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
	m_UI->extractPointsToolButton->setEnabled(false);

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

void ccSectionExtractionTool::stop(bool accepted)
{
	if (m_editedPoly)
	{
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		delete m_editedPoly;
		m_editedPoly = nullptr;
	}
	m_editedPolyVertices = nullptr;

	enableSectionEditingMode(false);
	reset(true);

	if (m_associatedWin)
	{
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
	}

	ccOverlayDialog::stop(accepted);
}

void ccSectionExtractionTool::updateCloudsBox()
{
	m_cloudsBox.clear();

	for (auto & cloud : m_clouds)
	{
		if (cloud.entity)
			m_cloudsBox += cloud.entity->getOwnBB();
	}
}

bool ccSectionExtractionTool::addPolyline(ccPolyline* inputPoly, bool alreadyInDB/*=true*/)
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
		int vertDim = m_UI->vertAxisComboBox->currentIndex();
		assert(vertDim >= 0 && vertDim < 3);

		//get default altitude from the cloud(s) bouding-box
		PointCoordinateType defaultZ = 0;
		if (m_cloudsBox.isValid())
		{
			defaultZ = m_cloudsBox.maxCorner()[vertDim];
		}

		//duplicate polyline
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
	m_UI->extractPointsToolButton->setEnabled(true);

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

bool ccSectionExtractionTool::addCloud(ccGenericPointCloud* inputCloud, bool alreadyInDB/*=true*/)
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
				ccLog::Warning("[ccSectionExtractionTool] Clouds have different shift & scale information! Only the first one will be used");
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

void ccSectionExtractionTool::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	Q_UNUSED( buttons );
	
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	//process not started yet?
	if ((m_state & RUNNING) == 0)
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

void ccSectionExtractionTool::addPointToPolyline(int x, int y)
{
	if ((m_state & STARTED) == 0)
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
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	//start new polyline?
	if (((m_state & RUNNING) == 0) || vertCount == 0)
	{
		//reset state
		m_state = (STARTED | RUNNING);
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

void ccSectionExtractionTool::closePolyLine(int, int)
{
	//only in RUNNING mode
	if ((m_state & RUNNING) == 0 || !m_editedPoly)
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

		//remove polyline from the 'temporary' world
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		//set default display style
		m_editedPoly->showColors(true);
		m_editedPoly->setColor(s_defaultPolylineColor);
		m_editedPoly->setWidth(s_defaultPolylineWidth);
		if (!m_clouds.isEmpty())
			m_editedPoly->setDisplay_recursive(m_clouds.front().originalDisplay); //set the same 'default' display as the cloud
		m_editedPoly->setName(QString("Polyline #%1").arg(m_sections.size() + 1));
		//save polyline
		if (!addPolyline(m_editedPoly, false))
		{
			//if something went wrong, we have to remove the polyline manually
			delete m_editedPoly;
		}
		m_editedPoly = nullptr;
		m_editedPolyVertices = nullptr;
	}

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
	{
		m_associatedWin->redraw(true, false);
	}
}

void ccSectionExtractionTool::cancelCurrentPolyline()
{
	if ((m_state & STARTED) == 0
		|| !m_editedPoly)
	{
		return;
	}

	assert(m_editedPolyVertices);
	m_editedPoly->clear();
	m_editedPolyVertices->clear();

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccSectionExtractionTool::enableSectionEditingMode(bool state)
{
	if (!m_associatedWin)
		return;

	if (!state/*=activate pause mode*/)
	{
		//select the last polyline by default (if any)
		if (!m_sections.empty() && !m_sections.back().isInDB)
			selectPolyline(&m_sections.back());

		m_state = PAUSED;

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

		m_state = STARTED;

		m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		m_associatedWin->displayNewMessage("Section edition mode", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->displayNewMessage("Left click: add section points / Right click: stop", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	}

	//update mini-GUI
	m_UI->polylineToolButton->blockSignals(true);
	m_UI->polylineToolButton->setChecked(state);
	m_UI->frame->setEnabled(!state);
	m_UI->polylineToolButton->blockSignals(false);

	m_associatedWin->redraw();
}

void ccSectionExtractionTool::addUndoStep()
{
	if (m_undoCount.empty() || (static_cast<int>(m_undoCount.back()) < m_sections.size()))
	{
		m_undoCount.push_back(m_sections.size());
		m_UI->undoToolButton->setEnabled(true);
	}
}

void ccSectionExtractionTool::doImportPolylinesFromDB()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (!mainWindow)
		return;

	ccHObject* root = mainWindow->dbRootObject();
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

		enableSectionEditingMode(false);
		
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

void ccSectionExtractionTool::cancel()
{
	reset(false);

	stop(false);
}

void ccSectionExtractionTool::apply()
{
	if (!reset(true))
		return;

	stop(true);
}

static double s_orthoSectionWidth = -1.0;
static double s_orthoSectionStep = -1.0;
static bool s_autoSaveAndRemoveGeneratrix = true;
void ccSectionExtractionTool::generateOrthoSections()
{
	if (!m_selectedPoly)
	{
		ccLog::Warning("[ccSectionExtractionTool] No polyline selected");
		return;
	}

	//compute poyline length
	ccPolyline* poly = m_selectedPoly->entity;
	unsigned vertCount = (poly ? poly->size() : 0);
	if (vertCount < 2)
	{
		ccLog::Warning("[ccSectionExtractionTool] Invalid polyline");
		return;
	}

	PointCoordinateType length = poly->computeLength();

	//show arrow
	{
		assert(vertCount >= 2);
		const CCVector3* lastQ = poly->getPoint(vertCount - 2);
		const CCVector3* lastP = poly->getPoint(vertCount - 1);
		PointCoordinateType tipLength = (*lastQ - *lastP).norm();
		PointCoordinateType defaultArrowSize = m_associatedWin->computeActualPixelSize() * s_defaultArrowSize;
		defaultArrowSize = std::min(defaultArrowSize, tipLength / 2);
		poly->showArrow(true, poly->size() - 1, defaultArrowSize);
		m_associatedWin->redraw();
	}

	//display dialog
	ccOrthoSectionGenerationDlg osgDlg(MainWindow::TheInstance());
	osgDlg.setPathLength(length);
	if (s_orthoSectionWidth > 0.0)
		osgDlg.setSectionsWidth(s_orthoSectionWidth);
	if (s_orthoSectionStep > 0.0)
		osgDlg.setGenerationStep(s_orthoSectionStep);
	osgDlg.setAutoSaveAndRemove(s_autoSaveAndRemoveGeneratrix);

	if (osgDlg.exec())
	{
		//now generate the orthogonal sections
		s_orthoSectionStep = osgDlg.getGenerationStep();
		s_orthoSectionWidth = osgDlg.getSectionsWidth();
		s_autoSaveAndRemoveGeneratrix = osgDlg.autoSaveAndRemove();

		if (s_autoSaveAndRemoveGeneratrix)
		{
			//save
			if (!m_selectedPoly->isInDB)
			{
				ccHObject* destEntity = getExportGroup(s_polyExportGroupID, "Exported sections");
				assert(destEntity);
				destEntity->addChild(m_selectedPoly->entity);
				m_selectedPoly->isInDB = true;
				m_selectedPoly->entity->setDisplay_recursive(destEntity->getDisplay());
				MainWindow::TheInstance()->addToDB(m_selectedPoly->entity, false, false);
			}
			//and remove
			deleteSelectedPolyline();
		}

		//set new 'undo' step
		addUndoStep();

		//normal to the plane
		CCVector3 N(0, 0, 0);
		int vertDim = m_UI->vertAxisComboBox->currentIndex();
		assert(vertDim >= 0 && vertDim < 3);
		{
			N.u[vertDim] = 1.0;
		}

		//curvilinear position
		double s = 0;
		//current length
		double l = 0;
		unsigned maxCount = vertCount;
		if (!poly->isClosed())
			maxCount--;
		unsigned polyIndex = 0;
		for (unsigned i = 0; i < maxCount; ++i)
		{
			const CCVector3* A = poly->getPoint(i);
			const CCVector3* B = poly->getPoint((i + 1) % vertCount);
			CCVector3 AB = (*B - *A);
			AB.u[vertDim] = 0;
			CCVector3 nAB = AB.cross(N);
			nAB.normalize();

			double lAB = (*B - *A).norm();
			while (s < l + lAB)
			{
				double s_local = s - l;
				assert(s_local < lAB);

				//create orhogonal polyline
				ccPointCloud* vertices = new ccPointCloud("vertices");
				ccPolyline* orthoPoly = new ccPolyline(vertices);
				orthoPoly->addChild(vertices);
				if (vertices->reserve(2) && orthoPoly->reserve(2))
				{
					//intersection point
					CCVector3 I = *A + AB * (s_local / lAB);
					CCVector3 I1 = I + nAB * static_cast<PointCoordinateType>(s_orthoSectionWidth / 2);
					CCVector3 I2 = I - nAB * static_cast<PointCoordinateType>(s_orthoSectionWidth / 2);

					vertices->addPoint(I1);
					orthoPoly->addPointIndex(0);
					vertices->addPoint(I2);
					orthoPoly->addPointIndex(1);

					orthoPoly->setClosed(false);
					orthoPoly->set2DMode(false);
					orthoPoly->setGlobalScale(poly->getGlobalScale());
					orthoPoly->setGlobalShift(poly->getGlobalShift());

					//set default display style
					vertices->setEnabled(false);
					orthoPoly->showColors(true);
					orthoPoly->setColor(s_defaultPolylineColor);
					orthoPoly->setWidth(s_defaultPolylineWidth);
					if (!m_clouds.isEmpty())
						orthoPoly->setDisplay_recursive(m_clouds.front().originalDisplay); //set the same 'default' display as the cloud
					orthoPoly->setName(QString("%1.%2").arg(poly->getName()).arg(++polyIndex));

					//add meta data (for Mascaret export)
					{
						orthoPoly->setMetaData(ccPolyline::MetaKeyUpDir(), QVariant(vertDim));
						orthoPoly->setMetaData(ccPolyline::MetaKeyAbscissa(), QVariant(s));
						orthoPoly->setMetaData(ccPolyline::MetaKeyPrefixCenter() + ".x", QVariant(static_cast<double>(I.x)));
						orthoPoly->setMetaData(ccPolyline::MetaKeyPrefixCenter() + ".y", QVariant(static_cast<double>(I.y)));
						orthoPoly->setMetaData(ccPolyline::MetaKeyPrefixCenter() + ".z", QVariant(static_cast<double>(I.z)));
						orthoPoly->setMetaData(ccPolyline::MetaKeyPrefixDirection() + ".x", QVariant(static_cast<double>(nAB.x)));
						orthoPoly->setMetaData(ccPolyline::MetaKeyPrefixDirection() + ".y", QVariant(static_cast<double>(nAB.y)));
						orthoPoly->setMetaData(ccPolyline::MetaKeyPrefixDirection() + ".z", QVariant(static_cast<double>(nAB.z)));
					}

					if (!addPolyline(orthoPoly, false))
					{
						delete orthoPoly;
						orthoPoly = nullptr;
					}
				}
				else
				{
					delete orthoPoly;
					orthoPoly = nullptr;
					ccLog::Error("Not enough memory!");
					//early stop
					i = maxCount;
					break;
				}

				s += s_orthoSectionStep;
			}

			l += lAB;
		}
	}

	poly->showArrow(false, 0, 0);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

ccHObject* ccSectionExtractionTool::getExportGroup(unsigned& defaultGroupID, const QString& defaultName)
{
	MainWindow* mainWin = MainWindow::TheInstance();
	ccHObject* root = mainWin ? mainWin->dbRootObject() : nullptr;
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
		mainWin->addToDB(destEntity);
		defaultGroupID = destEntity->getUniqueID();
	}
	return destEntity;
}

void ccSectionExtractionTool::exportSections()
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
		ccLog::Warning("[ccSectionExtractionTool] All active sections are already in DB");
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
			mainWin->addToDB(section.entity, false, false);
		}
	}

	ccLog::Print(QString("[ccSectionExtractionTool] %1 sections exported").arg(exportCount));
}

bool ccSectionExtractionTool::extractSectionContour(const ccPolyline* originalSection,
	const ccPointCloud* originalSectionCloud,
	ccPointCloud* unrolledSectionCloud,
	unsigned sectionIndex,
	ccContourExtractor::ContourType contourType,
	PointCoordinateType maxEdgeLength,
	bool multiPass,
	bool splitContour,
	bool& contourGenerated,
	bool visualDebugMode/*=false*/)
{
	contourGenerated = false;

	if (!originalSectionCloud || !unrolledSectionCloud)
	{
		ccLog::Warning("[ccSectionExtractionTool][extract contour] Internal error: invalid input parameter(s)");
		return false;
	}

	if (originalSectionCloud->size() < 2)
	{
		//nothing to do
		ccLog::Warning(QString("[ccSectionExtractionTool][extract contour] Section #%1 contains less than 2 points and will be ignored").arg(sectionIndex));
		return true;
	}

	//by default, the points in 'unrolledSectionCloud' are 2D (X = curvilinear coordinate, Y = height, Z = 0)
	CCVector3 N(0, 0, 1);
	CCVector3 Y(0, 1, 0);

	std::vector<unsigned> vertIndexes;
	ccPolyline* contour = ccContourExtractor::ExtractFlatContour(unrolledSectionCloud,
		multiPass,
		maxEdgeLength,
		N.u,
		Y.u,
		contourType,
		&vertIndexes,
		visualDebugMode);
	if (contour)
	{
		//update vertices (to replace 'unrolled' points by 'original' ones
		{
			CCLib::GenericIndexedCloud* vertices = contour->getAssociatedCloud();
			if (vertIndexes.size() == static_cast<size_t>(vertices->size()))
			{
				for (unsigned i = 0; i < vertices->size(); ++i)
				{
					const CCVector3* P = vertices->getPoint(i);
					assert(vertIndexes[i] < originalSectionCloud->size());
					*const_cast<CCVector3*>(P) = *originalSectionCloud->getPoint(vertIndexes[i]);
				}

				ccPointCloud* verticesAsPC = dynamic_cast<ccPointCloud*>(vertices);
				if (verticesAsPC)
					verticesAsPC->refreshBB();
			}
			else
			{
				ccLog::Warning("[ccSectionExtractionTool][extract contour] Internal error (couldn't fetch original points indexes?!)");
				delete contour;
				return false;
			}
		}

		std::vector<ccPolyline*> parts;
		if (splitContour)
		{
#ifdef QT_DEBUG
			//compute some stats on the contour
			{
				double minLength = 0;
				double maxLength = 0;
				double sumLength = 0;
				unsigned count = contour->size();
				if (!contour->isClosed())
					--count;
				for (unsigned i=0; i<count; ++i)
				{
					const CCVector3* A = contour->getPoint(i);
					const CCVector3* B = contour->getPoint((i+1) % contour->size());
					CCVector3 e = *B - *A;
					double l = e.norm();
					if (i != 0)
					{
						minLength = std::min(minLength,l);
						maxLength = std::max(maxLength,l);
						sumLength += l;
					}
					else
					{
						minLength = maxLength = sumLength = l;
					}
				}
				ccLog::PrintDebug(QString("Contour: min = %1 / avg = %2 / max = %3").arg(minLength).arg(sumLength/count).arg(maxLength));
			}
#endif

			/*bool success = */contour->split(maxEdgeLength, parts);
			delete contour;
			contour = nullptr;
		}
		else
		{
			parts.push_back(contour);
		}

		//create output group if necessary
		ccHObject* destEntity = getExportGroup(s_profileExportGroupID, "Extracted profiles");
		assert(destEntity);

		for (size_t p = 0; p < parts.size(); ++p)
		{
			ccPolyline* contourPart = parts[p];
			QString name = QString("Section contour #%1").arg(sectionIndex);
			if (parts.size() > 1)
				name += QString("(part %1/%2)").arg(p + 1).arg(parts.size());
			contourPart->setName(name);
			contourPart->setGlobalScale(originalSectionCloud->getGlobalScale());
			contourPart->setGlobalShift(originalSectionCloud->getGlobalShift());
			contourPart->setColor(s_defaultContourColor);
			contourPart->showColors(true);
			//copy meta-data (import for Mascaret export!)
			{
				const QVariantMap& metaData = originalSection->metaData();
				for (QVariantMap::const_iterator it = metaData.begin(); it != metaData.end(); ++it)
				{
					contourPart->setMetaData(it.key(), it.value());
				}
			}

			//add to main DB
			destEntity->addChild(contourPart);
			contourPart->setDisplay_recursive(destEntity->getDisplay());
			MainWindow::TheInstance()->addToDB(contourPart, false, false);
		}

		contourGenerated = true;
	}

	return true;
}

bool ccSectionExtractionTool::extractSectionCloud(const std::vector<CCLib::ReferenceCloud*>& refClouds,
	unsigned sectionIndex,
	bool& cloudGenerated)
{
	cloudGenerated = false;

	ccPointCloud* sectionCloud = nullptr;
	for (int i = 0; i < static_cast<int>(refClouds.size()); ++i)
	{
		if (!refClouds[i])
			continue;
		assert(m_clouds[i].entity); //a valid ref. cloud must have a valid counterpart!

		//extract part/section from each cloud
		ccPointCloud* part = nullptr;

		//if the cloud is a ccPointCloud, we can keep a lot more information
		//when extracting the section cloud
		ccPointCloud* pc = dynamic_cast<ccPointCloud*>(m_clouds[i].entity);
		if (pc)
		{
			part = pc->partialClone(refClouds[i]);
		}
		else
		{
			part = ccPointCloud::From(refClouds[i], m_clouds[i].entity);
		}

		if (part)
		{
			if (i == 0)
			{
				//we simply use this 'part' cloud as the section cloud
				sectionCloud = part;
			}
			else
			{
				//fuse it with the global cloud
				unsigned cloudSizeBefore = sectionCloud->size();
				unsigned partSize = part->size();
				sectionCloud->append(part, cloudSizeBefore, true);

				//don't need it anymore
				delete part;
				part = nullptr;
				//check that it actually worked!
				if (sectionCloud->size() != cloudSizeBefore + partSize)
				{
					//not enough memory
					ccLog::Warning("[ccSectionExtractionTool][extract cloud] Not enough memory");
					delete sectionCloud;
					return false;
				}
			}
		}
		else
		{
			//not enough memory
			ccLog::Warning("[ccSectionExtractionTool][extract cloud] Not enough memory");
			delete sectionCloud;
			return false;
		}
	}

	if (sectionCloud)
	{
		//create output group if necessary
		ccHObject* destEntity = getExportGroup(s_cloudExportGroupID, "Extracted section clouds");
		assert(destEntity);

		sectionCloud->setName(QString("Section cloud #%1").arg(sectionIndex));
		sectionCloud->setDisplay(destEntity->getDisplay());

		//add to main DB
		destEntity->addChild(sectionCloud);
		MainWindow::TheInstance()->addToDB(sectionCloud, false, false);

		cloudGenerated = true;
	}

	return true;
}

struct Segment
{
	Segment()
		: A(0, 0)
		, B(0, 0)
		, u(0, 0)
		, d(0)
		, curvPos(0)
	{}

	CCVector2 A, B, u;
	PointCoordinateType d, curvPos;
};

void ccSectionExtractionTool::unfoldPoints()
{
	if (!m_selectedPoly || !m_selectedPoly->entity)
	{
		assert(false);
		return;
	}

	ccPolyline* poly = m_selectedPoly->entity;
	unsigned polyVertCount = poly->size();
	if (polyVertCount < 2)
	{
		ccLog::Error("Invalid polyline?!");
		assert(false);
		return;
	}

	//compute loaded clouds bounding-box
	ccBBox box;
	unsigned totalPointCount = 0;
	{
		for (auto & cloud : m_clouds)
		{
			if (cloud.entity)
			{
				box += cloud.entity->getOwnBB();
				totalPointCount += cloud.entity->size();
			}
		}
	}

	static double s_defaultThickness = -1.0;
	if (s_defaultThickness <= 0)
	{
		s_defaultThickness = box.getMaxBoxDim() / 10.0;
	}

	bool ok;
	double thickness = QInputDialog::getDouble(MainWindow::TheInstance(), "Thickness", "Distance to polyline:", s_defaultThickness, 1.0e-6, 1.0e6, 6, &ok);
	if (!ok)
		return;
	s_defaultThickness = thickness;

	//projection direction
	int vertDim = m_UI->vertAxisComboBox->currentIndex();
	int xDim = (vertDim < 2 ? vertDim + 1 : 0);
	int yDim = (xDim < 2 ? xDim + 1 : 0);

	//we consider half of the total thickness as points can be on both sides!
	double maxSquareDistToPolyline = (thickness / 2) * (thickness / 2);

	//prepare the computation of 2D distances
	std::vector<Segment> segments;
	unsigned polySegmentCount = poly->isClosed() ? polyVertCount : polyVertCount - 1;
	{
		try
		{
			segments.reserve(polySegmentCount);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			ccLog::Error("Not enough memory");
			return;
		}

		PointCoordinateType curvPos = 0;
		for (unsigned j = 0; j < polySegmentCount; ++j)
		{
			//current polyline segment
			const CCVector3* A = poly->getPoint(j);
			const CCVector3* B = poly->getPoint((j + 1) % polyVertCount);

			Segment s;
			{
				s.A = CCVector2(A->u[xDim], A->u[yDim]);
				s.B = CCVector2(B->u[xDim], B->u[yDim]);
				s.u = s.B - s.A;
				s.d = s.u.norm();
				if (s.d > ZERO_TOLERANCE)
				{
					s.curvPos = curvPos;
					s.u /= s.d;
					segments.push_back(s);
				}
			}

			//update curvilinear pos
			curvPos += (*B - *A).norm();
		}
	}

	ccProgressDialog pdlg(true);
	CCLib::NormalizedProgress nprogress(&pdlg, totalPointCount);
	pdlg.setMethodTitle(tr("Unfold cloud(s)"));
	pdlg.setInfo(tr("Number of segments: %1\nNumber of points: %2").arg(polySegmentCount).arg(totalPointCount));
	pdlg.start();
	QCoreApplication::processEvents();

	unsigned exportedClouds = 0;

	//for each cloud
	for (auto & pc : m_clouds)
	{
		ccGenericPointCloud* cloud = pc.entity;
		if (!cloud)
		{
			assert(false);
			continue;
		}

		CCLib::ReferenceCloud unfoldedIndexes(cloud);
		if (!unfoldedIndexes.reserve(cloud->size()))
		{
			ccLog::Error("Not enough memory");
			return;
		}
		std::vector<CCVector3> unfoldedPoints;
		try
		{
			unfoldedPoints.reserve(cloud->size());
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory");
			return;
		}

		//now test each point and see if it's close to the current polyline (in 2D)
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			CCVector2 P2D(P->u[xDim], P->u[yDim]);

			//test each segment
			int closestSegment = -1;
			PointCoordinateType minSquareDist = -PC_ONE;
			for (unsigned j = 0; j < polySegmentCount; ++j)
			{
				const Segment& s = segments[j];
				CCVector2 AP2D = P2D - s.A;

				//longitudinal 'distance'
				PointCoordinateType dotprod = s.u.dot(AP2D);

				PointCoordinateType squareDist = 0;
				if (dotprod < 0.0f)
				{
					//dist to nearest vertex
					squareDist = AP2D.norm2();
				}
				else if (dotprod > s.d)
				{
					//dist to nearest vertex
					squareDist = (P2D - s.B).norm2();
				}
				else
				{
					//orthogonal distance
					squareDist = (AP2D - s.u*dotprod).norm2();
				}

				if (squareDist <= maxSquareDistToPolyline)
				{
					if (closestSegment < 0 || squareDist < minSquareDist)
					{
						minSquareDist = squareDist;
						closestSegment = static_cast<int>(j);
					}
				}
			}

			if (closestSegment >= 0)
			{
				const Segment& s = segments[closestSegment];

				//we use the curvilinear position of the point in the X dimension (and Y is 0)
				CCVector3 Q;
				{
					CCVector2 AP2D = P2D - s.A;
					PointCoordinateType dotprod = s.u.dot(AP2D);
					PointCoordinateType d = (AP2D - s.u*dotprod).norm();

					//compute the sign of 'minDist'
					PointCoordinateType crossprod = AP2D.y * s.u.x - AP2D.x * s.u.y;

					Q.u[xDim] = s.curvPos + dotprod;
					Q.u[yDim] = crossprod < 0 ? -d : d; //signed orthogonal distance to the polyline
					Q.u[vertDim] = P->u[vertDim];
				}

				unfoldedIndexes.addPointIndex(i);
				unfoldedPoints.push_back(Q);
			}

			if (!nprogress.oneStep())
			{
				ccLog::Warning("[Unfold] Process cancelled by the user");
				return;
			}

		} //for each point

		if (unfoldedIndexes.size() != 0)
		{
			//assign the default global shift & scale info
			ccPointCloud* unfoldedCloud = nullptr;
			{
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
					unfoldedCloud = static_cast<ccPointCloud*>(cloud)->partialClone(&unfoldedIndexes);
				else
					unfoldedCloud = ccPointCloud::From(&unfoldedIndexes, cloud);
			}
			if (!unfoldedCloud)
			{
				ccLog::Error("Not enough memory");
				return;
			}

			assert(unfoldedCloud->size() == unfoldedPoints.size());
			CCVector3 C = box.minCorner();
			C.u[vertDim] = 0;
			C.u[xDim] = box.minCorner().u[xDim]; //we start at the bounding-box limit
			for (unsigned i = 0; i < unfoldedCloud->size(); ++i)
			{
				//update the points positions
				*const_cast<CCVector3*>(unfoldedCloud->getPoint(i)) = unfoldedPoints[i] + C;
			}
			unfoldedCloud->invalidateBoundingBox();

			unfoldedCloud->setName(cloud->getName() + ".unfolded");
			unfoldedCloud->setGlobalShift(cloud->getGlobalShift());
			unfoldedCloud->setGlobalScale(cloud->getGlobalScale());

			unfoldedCloud->shrinkToFit();
			unfoldedCloud->setDisplay(pc.originalDisplay);
			MainWindow::TheInstance()->addToDB(unfoldedCloud);

			++exportedClouds;
		}
		else
		{
			ccLog::Warning(QString("[Unfold] No point of the cloud '%1' were unfolded (check parameters)").arg(cloud->getName()));
		}

	} //for each cloud

	ccLog::Print(QString("[Unfold] %1 cloud(s) exported").arg(exportedClouds));
}

struct Segment2D
{
	Segment2D() : s(0) {}

	CCVector2 A, B, uAB;
	PointCoordinateType lAB;
	PointCoordinateType s; //curvilinear coordinate
};

void ccSectionExtractionTool::extractPoints()
{
	static double s_defaultSectionThickness = -1.0;
	static double s_contourMaxEdgeLength = 0;
	static bool s_extractSectionsAsClouds = false;
	static bool s_extractSectionsAsContours = true;
	static bool s_multiPass = false;
	static bool s_splitContour = false;
	static ccContourExtractor::ContourType s_extractSectionsType = ccContourExtractor::LOWER;

	//number of eligible sections
	unsigned sectionCount = 0;
	{
		for (auto & section : m_sections)
		{
			if (section.entity && section.entity->size() > 1)
				++sectionCount;
		}
	}
	if (sectionCount == 0)
	{
		ccLog::Error("No (valid) section!");
		return;
	}

	//compute loaded clouds bounding-box
	ccBBox box;
	unsigned pointCount = 0;

	for (auto & cloud : m_clouds)
	{
		if (cloud.entity)
		{
			box += cloud.entity->getOwnBB();
			pointCount += cloud.entity->size();
		}
	}

	if (s_defaultSectionThickness <= 0)
	{
		s_defaultSectionThickness = box.getMaxBoxDim() / 500.0;
	}
	if (s_contourMaxEdgeLength <= 0)
	{
		s_contourMaxEdgeLength = box.getMaxBoxDim() / 500.0;
	}

	//show dialog
	ccSectionExtractionSubDlg sesDlg(MainWindow::TheInstance());
	sesDlg.setActiveSectionCount(sectionCount);
	sesDlg.setSectionThickness(s_defaultSectionThickness);
	sesDlg.setMaxEdgeLength(s_contourMaxEdgeLength);
	sesDlg.doExtractClouds(s_extractSectionsAsClouds);
	sesDlg.doExtractContours(s_extractSectionsAsContours, s_extractSectionsType);
	sesDlg.doUseMultiPass(s_multiPass);
	sesDlg.doSplitContours(s_splitContour);

	if (!sesDlg.exec())
		return;

	s_defaultSectionThickness = sesDlg.getSectionThickness();
	s_contourMaxEdgeLength = sesDlg.getMaxEdgeLength();
	s_extractSectionsAsClouds = sesDlg.extractClouds();
	s_extractSectionsAsContours = sesDlg.extractContours();
	s_extractSectionsType = sesDlg.getContourType();
	s_multiPass = sesDlg.useMultiPass();
	s_splitContour = sesDlg.splitContours();
	bool visualDebugMode = sesDlg.visualDebugMode();

	//progress dialog
	ccProgressDialog pdlg(true);
	CCLib::NormalizedProgress nprogress(&pdlg, static_cast<unsigned>(sectionCount));
	if (!visualDebugMode)
	{
		pdlg.setMethodTitle(tr("Extract sections"));
		pdlg.setInfo(tr("Number of sections: %1\nNumber of points: %2").arg(sectionCount).arg(pointCount));
		pdlg.start();
		QCoreApplication::processEvents();
	}

	int vertDim = m_UI->vertAxisComboBox->currentIndex();
	int xDim = (vertDim < 2 ? vertDim + 1 : 0);
	int yDim = (xDim < 2 ? xDim + 1 : 0);

	//we consider half of the total thickness as points can be on both sides!
	double sectionThicknessSq = std::pow(s_defaultSectionThickness / 2.0, 2.0);
	bool error = false;

	unsigned generatedContours = 0;
	unsigned generatedClouds = 0;

	try
	{
		//for each slice
		for (int s = 0; s < m_sections.size(); ++s)
		{
			ccPolyline* poly = m_sections[s].entity;
			if (poly)
			{
				unsigned polyVertCount = poly->size();
				if (polyVertCount < 2)
				{
					assert(false);
					continue;
				}
				unsigned polySegmentCount = poly->isClosed() ? polyVertCount : polyVertCount - 1;

				//project the section in '2D'
				std::vector<Segment2D> polySegments2D;
				{
					polySegments2D.reserve(polySegmentCount);
					PointCoordinateType s = 0;
					for (unsigned j = 0; j < polySegmentCount; ++j)
					{
						Segment2D seg2D;
						const CCVector3* A = poly->getPoint(j);
						const CCVector3* B = poly->getPoint((j + 1) % polyVertCount);
						seg2D.A = CCVector2(A->u[xDim], A->u[yDim]);
						seg2D.B = CCVector2(B->u[xDim], B->u[yDim]);
						seg2D.uAB = seg2D.B - seg2D.A; //(unit) direction
						seg2D.lAB = seg2D.uAB.norm(); //length
						seg2D.s = s;
						s += seg2D.lAB;

						if (seg2D.lAB < ZERO_TOLERANCE)
						{
							//ignore too small segments
							continue;
						}
						
						seg2D.uAB /= seg2D.lAB;
						polySegments2D.push_back(seg2D);
					}

					if (polySegments2D.empty())
					{
						assert(false);
						continue;
					}
					polySegments2D.shrink_to_fit();
				}

				int cloudCount = m_clouds.size();
				std::vector<CCLib::ReferenceCloud*> refClouds;
				if (s_extractSectionsAsClouds)
				{
					refClouds.resize(cloudCount, nullptr);
				}

				//for contour extraction as a polyline
				ccPointCloud* originalSlicePoints = nullptr;
				ccPointCloud* unrolledSlicePoints = nullptr;
				if (s_extractSectionsAsContours)
				{
					originalSlicePoints = new ccPointCloud("section.orig");
					unrolledSlicePoints = new ccPointCloud("section.unroll");

					//assign them the default (first!) global shift & scale info
					assert(!m_clouds.empty());
					ccGenericPointCloud* cloud = m_clouds.front().entity;
					originalSlicePoints->setGlobalScale(cloud->getGlobalScale());
					originalSlicePoints->setGlobalShift(cloud->getGlobalShift());
				}

				//for each cloud
				for (int c = 0; c < cloudCount; ++c)
				{
					ccGenericPointCloud* cloud = m_clouds[c].entity;
					if (cloud)
					{
						//for contour extraction as a cloud
						CCLib::ReferenceCloud* refCloud = nullptr;
						if (s_extractSectionsAsClouds)
						{
							refCloud = new CCLib::ReferenceCloud(cloud);
						}

						//compute the distance of each point to the current polyline segment
						for (unsigned i = 0; i < cloud->size(); ++i)
						{
							const CCVector3* P = cloud->getPoint(i);
							CCVector2 P2D(P->u[xDim], P->u[yDim]);

							//for each vertex
							PointCoordinateType minSquareDist = -PC_ONE;
							PointCoordinateType curvilinearPos = 0.0;
							size_t minIndex = 0;
							for (size_t j = 0; j < polySegments2D.size(); ++j)
							{
								const Segment2D& seg2D = polySegments2D[j];
								CCVector2 AP2D = P2D - seg2D.A;

								//square distance to the polyline
								PointCoordinateType squareDist = 0;

								//longitudinal 'distance'
								double dotprod = seg2D.uAB.dot(AP2D);
								if (dotprod < 0)
								{
									if (j == 0 && !poly->isClosed())
										continue;
									squareDist = AP2D.norm2();
								}
								else if (dotprod > seg2D.lAB)
								{
									if (j + 1 == polySegments2D.size() && !poly->isClosed())
										continue;
									squareDist = (P2D - seg2D.B).norm2();
								}
								else
								{
									//orthogonal distance
									squareDist = (AP2D - seg2D.uAB * dotprod).norm2();
								}

								if (minSquareDist < 0 || squareDist < minSquareDist)
								{
									minSquareDist = squareDist;
									curvilinearPos = dotprod;
									minIndex = j;
								}
							}

							//elligible point?
							if (minSquareDist >= 0 && minSquareDist < sectionThicknessSq)
							{
								//if we extract the section as cloud(s), we add the point to the (current) ref. cloud
								if (s_extractSectionsAsClouds)
								{
									assert(refCloud);
									unsigned refCloudSize = refCloud->size();
									if (refCloudSize == refCloud->capacity())
									{
										refCloudSize += (refCloudSize / 2 + 1);
										if (!refCloud->reserve(refCloudSize))
										{
											//not enough memory
											ccLog::Warning("[ccSectionExtractionTool] Not enough memory");
											error = true;
											break;
										}
									}
									refCloud->addPointIndex(i);
								}

								//if we extract the section as contour(s), we add it to the 2D points set
								if (s_extractSectionsAsContours)
								{
									assert(originalSlicePoints && unrolledSlicePoints);
									assert(originalSlicePoints->size() == unrolledSlicePoints->size());

									unsigned cloudSize = originalSlicePoints->size();
									if (cloudSize == originalSlicePoints->capacity())
									{
										cloudSize += (cloudSize / 2 + 1);
										if (!originalSlicePoints->reserve(cloudSize)
											|| !unrolledSlicePoints->reserve(cloudSize))
										{
											//not enough memory
											ccLog::Warning("[ccSectionExtractionTool] Not enough memory");
											error = true;
											break;
										}
									}

									const Segment2D& seg2D = polySegments2D[minIndex];

									//we project the 'real' 3D point in the section plane
									CCVector3 Pproj3D;
									{
										Pproj3D.u[xDim] = seg2D.A.x + seg2D.uAB.x * curvilinearPos;
										Pproj3D.u[yDim] = seg2D.A.y + seg2D.uAB.y * curvilinearPos;
										Pproj3D.u[vertDim] = P->u[vertDim];
									}
									originalSlicePoints->addPoint(Pproj3D);
									unrolledSlicePoints->addPoint(CCVector3(seg2D.s + curvilinearPos, P->u[vertDim], 0));
								}
							}

							if (error)
							{
								break;
							}

						} //for each point

						if (refCloud)
						{
							assert(s_extractSectionsAsClouds);
							if (error || refCloud->size() == 0)
							{
								delete refCloud;
								refCloud = nullptr;
							}
							else
							{
								refClouds[c] = refCloud;
							}
						}

					}

					if (error)
					{
						break;
					}

				} //for each cloud

				if (!error)
				{
					//Extract sections as (polyline) contours
					if (/*!error && */s_extractSectionsAsContours)
					{
						assert(originalSlicePoints && unrolledSlicePoints);
						bool contourGenerated = false;
						error = !extractSectionContour(	poly,
														originalSlicePoints,
														unrolledSlicePoints,
														s + 1,
														s_extractSectionsType,
														s_contourMaxEdgeLength,
														s_multiPass,
														s_splitContour,
														contourGenerated,
														visualDebugMode);

						if (contourGenerated)
							++generatedContours;
					}

					//Extract sections as clouds
					if (!error && s_extractSectionsAsClouds)
					{
						assert(static_cast<int>(refClouds.size()) == cloudCount);
						bool cloudGenerated = false;
						error = !extractSectionCloud(refClouds, s + 1, cloudGenerated);
						if (cloudGenerated)
							++generatedClouds;
					}
				}

				//release memory
				for (auto & refCloud : refClouds)
				{
					delete refCloud;
					refCloud = nullptr;
				}

				delete originalSlicePoints;
				originalSlicePoints = nullptr;

				delete unrolledSlicePoints;
				unrolledSlicePoints = nullptr;
			} //if (poly)

			if (!nprogress.oneStep())
			{
				ccLog::Warning("[ccSectionExtractionTool] Canceled by user");
				error = true;
			}

			if (error)
				break;
		} //for (int s=0; s<m_sections.size(); ++s)
	}
	catch (const std::bad_alloc&)
	{
		error = true;
	}

	if (error)
	{
		ccLog::Error("An error occurred (see console)");
	}
	else
	{
		ccLog::Print(QString("[ccSectionExtractionTool] Job done (%1 contour(s) and %2 cloud(s) were generated)").arg(generatedContours).arg(generatedClouds));
	}
}
