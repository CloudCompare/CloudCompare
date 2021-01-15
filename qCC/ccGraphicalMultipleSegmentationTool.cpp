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

#include "ccGraphicalMultipleSegmentationTool.h"

//Local
#include "mainwindow.h"
#include "ccItemSelectionDlg.h"
#include "ccReservedIDs.h"

//CCCoreLib
#include <ManualSegmentationTools.h>
#include <SquareMatrix.h>

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>
#include <cc2DViewportObject.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>

//System
#include <assert.h>

ccGraphicalMultipleSegmentationTool::ccGraphicalMultipleSegmentationTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::GraphicalMultipleSegmentationDlg()
	, m_somethingHasChanged(false)
	, m_state(0)
	, m_segmentationPoly(nullptr)
	, m_polyVertices(nullptr)
	, m_rectangularSelection(false)
	, m_deleteHiddenParts(false)
	, m_currentGroupIndex(0)
	

{
	// Set QDialog background as transparent (DGM: doesn't work over an OpenGL context)
	//setAttribute(Qt::WA_NoSystemBackground);

	setupUi(this);

	connect(inButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::segmentIn);
	connect(outButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::segmentOut);
	connect(validButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::apply);
	connect(cancelButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::cancel);
	connect(pauseButton, &QToolButton::toggled, this, &ccGraphicalMultipleSegmentationTool::pauseSegmentationMode);
	connect(addSliceButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::addToBeSliced);
	connect(cancelPreviousCropButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::cancelPreviousCrop);
	connect(cancelCurrentSelectionButton, &QToolButton::clicked, this, &ccGraphicalMultipleSegmentationTool::cancelCurrentSelection);

	//selection modes
	connect(actionSetPolylineSelection, &QAction::triggered, this, &ccGraphicalMultipleSegmentationTool::doSetPolylineSelection);
	connect(actionSetRectangularSelection, &QAction::triggered, this, &ccGraphicalMultipleSegmentationTool::doSetRectangularSelection);
	//import/export options
	connect(actionUseExistingPolyline, &QAction::triggered, this, &ccGraphicalMultipleSegmentationTool::doActionUseExistingPolyline);
	connect(actionExportSegmentationPolyline, &QAction::triggered, this, &ccGraphicalMultipleSegmentationTool::doExportSegmentationPolyline);

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space);  //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button	addOverridenShortcut(Qt::Key_Tab);    //tab key to switch between rectangular and polygonal selection modes
	addOverridenShortcut(Qt::Key_I);      //'I' key for the "segment in" button
	addOverridenShortcut(Qt::Key_O);      //'O' key for the "segment out" button
	addOverridenShortcut(Qt::Key_U);      //'U' key for the "Add To Be Sliced" button
	addOverridenShortcut(Qt::Key_S);      //'S' key for the "Cancel Previous Crop" button
	addOverridenShortcut(Qt::Key_D);      //'D' key for the "Cancel Current Selection" button
	connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccGraphicalMultipleSegmentationTool::onShortcutTriggered);

	QMenu* selectionModeMenu = new QMenu(this);
	selectionModeMenu->addAction(actionSetPolylineSelection);
	selectionModeMenu->addAction(actionSetRectangularSelection);
	selectionModelButton->setDefaultAction(actionSetPolylineSelection);
	selectionModelButton->setMenu(selectionModeMenu);

	QMenu* importExportMenu = new QMenu(this);
	importExportMenu->addAction(actionUseExistingPolyline);
	importExportMenu->addAction(actionExportSegmentationPolyline);

	m_polyVertices = new ccPointCloud("vertices", static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES));
	m_segmentationPoly = new ccPolyline(m_polyVertices, static_cast<unsigned>(ReservedIDs::INTERACTIVE_SEGMENTATION_TOOL_POLYLINE));
	m_segmentationPoly->setForeground(true);
	m_segmentationPoly->setColor(ccColor::green);
	m_segmentationPoly->showColors(true);
	m_segmentationPoly->set2DMode(true);
	allowPolylineExport(false);
}

void ccGraphicalMultipleSegmentationTool::allowPolylineExport(bool state)
{
	if (state)
	{
		actionExportSegmentationPolyline->setEnabled(true);
	}
	else
	{
		actionExportSegmentationPolyline->setEnabled(false);
	}
}

ccGraphicalMultipleSegmentationTool::~ccGraphicalMultipleSegmentationTool()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = nullptr;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = nullptr;

	if (m_polyGroup.size()>0)
			m_polyGroup.clear();

	if (m_viewportGroup.size() > 0)
		m_viewportGroup.clear();
}

void ccGraphicalMultipleSegmentationTool::onShortcutTriggered(int key)
{
	switch (key)
	{
	case Qt::Key_Space:
		pauseButton->toggle();
		return;

	case Qt::Key_I:
		inButton->click();
		return;

	case Qt::Key_O:
		outButton->click();
		return;
	case Qt::Key_U:
		addSliceButton->click();
		return;
	case Qt::Key_Return:
		validButton->click();
		return;
	case Qt::Key_Escape:
		cancelButton->click();
		return;
	case Qt::Key_D:
		cancelCurrentSelectionButton->click();
		return;

	case Qt::Key_Tab:
		if (m_rectangularSelection)
			doSetPolylineSelection();
		else
			doSetRectangularSelection();
		return;

	default:
		//nothing to do
		break;
	}
}

bool ccGraphicalMultipleSegmentationTool::linkWith(ccGLWindow* win)
{
	assert(m_segmentationPoly);

	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (oldWin)
	{
		oldWin->disconnect(this);
		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(nullptr);
		}
	}

	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked, this, &ccGraphicalMultipleSegmentationTool::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked, this, &ccGraphicalMultipleSegmentationTool::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved, this, &ccGraphicalMultipleSegmentationTool::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::buttonReleased, this, &ccGraphicalMultipleSegmentationTool::closeRectangle);

		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(m_associatedWin);
		}
	}

	return true;
}

bool ccGraphicalMultipleSegmentationTool::start()
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
	{
		ccLog::Warning("[Graphical Segmentation Tool] No associated window!");
		return false;
	}

	m_segmentationPoly->clear();
	m_polyVertices->clear();
	m_polyGroup.clear();
	m_viewportGroup.clear();
	allowPolylineExport(false);

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_segmentationPoly);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	pauseSegmentationMode(false);
	validButton->setEnabled(false);
	m_somethingHasChanged = false;

	reset();


	return ccOverlayDialog::start();
}

void ccGraphicalMultipleSegmentationTool::removeAllEntities(bool unallocateVisibilityArrays)
{
	if (unallocateVisibilityArrays)
	{
		for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
		{
			ccHObjectCaster::ToGenericPointCloud(*p)->unallocateVisibilityArray();
		}
	}

	m_toSegment.clear();

}

void ccGraphicalMultipleSegmentationTool::stop(bool accepted)
{
	assert(m_segmentationPoly);




	m_associatedWin->displayNewMessage("Segmentation [OFF]",
		ccGLWindow::UPPER_CENTER_MESSAGE,
		false,
		2,
		ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);

	m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
	m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	m_associatedWin->setUnclosable(false);
	m_associatedWin->removeFromOwnDB(m_segmentationPoly);


	ccOverlayDialog::stop(accepted);

}

void ccGraphicalMultipleSegmentationTool::reset()
{
	if (m_somethingHasChanged)
	{

		for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
		{
			ccHObjectCaster::ToGenericPointCloud(*p)->resetVisibilityArray();
		}

		if (m_associatedWin)
			m_associatedWin->redraw(false);
		m_somethingHasChanged = false;
	}
	//Initiate Group index
	m_currentGroupIndex = 0;
	m_polyGroup.clear();
	m_viewportGroup.clear();
	addSliceButton->setEnabled(false);
	cancelPreviousCropButton->setEnabled(false);
	cancelCurrentSelectionButton->setEnabled(false);
	validButton->setEnabled(false);
}

bool ccGraphicalMultipleSegmentationTool::addEntity(ccHObject* entity)
{

	//FIXME
	/*if (entity->isLocked())
		ccLog::Warning(QString("Can't use entity [%1] cause it's locked!").arg(entity->getName()));
	else */
	if (!entity->isDisplayedIn(m_associatedWin))
	{
		ccLog::Warning(QString("[Graphical Segmentation Tool] Entity [%1] is not visible in the active 3D view!").arg(entity->getName()));
	}

	bool result = false;

	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{

		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		//detect if this cloud is in fact a vertex set for at least one mesh
		{
			//either the cloud is the child of its parent mesh
			if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH) && ccHObjectCaster::ToGenericMesh(cloud->getParent())->getAssociatedCloud() == cloud)
			{
				ccLog::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its parent mesh instead!").arg(entity->getName()));
				return false;
			}
			//or the parent of its child mesh!
			ccHObject::Container meshes;
			if (cloud->filterChildren(meshes, false, CC_TYPES::MESH) != 0)
			{
				for (unsigned i = 0; i < meshes.size(); ++i)
					if (ccHObjectCaster::ToGenericMesh(meshes[i])->getAssociatedCloud() == cloud)
					{
						ccLog::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its child mesh instead!").arg(entity->getName()));
						return false;
					}
			}
		}

		cloud->resetVisibilityArray();
		m_toSegment.insert(cloud);

		//automatically add cloud's children
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
			result |= addEntity(entity->getChild(i));
	}
	else if (entity->isKindOf(CC_TYPES::MESH))
	{
		if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment primitives yet! Sorry...");
			return false;
		}
		if (entity->isKindOf(CC_TYPES::SUB_MESH))
		{
			ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment sub-meshes! Select the parent mesh...");
			return false;
		}
		else
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);

			//first, we must check that there's no mesh and at least one of its sub-mesh mixed in the current selection!
			for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
			{
				if ((*p)->isKindOf(CC_TYPES::MESH))
				{
					ccGenericMesh* otherMesh = ccHObjectCaster::ToGenericMesh(*p);
					if (otherMesh->getAssociatedCloud() == mesh->getAssociatedCloud())
					{
						if ((otherMesh->isA(CC_TYPES::SUB_MESH) && mesh->isA(CC_TYPES::MESH))
							|| (otherMesh->isA(CC_TYPES::MESH) && mesh->isA(CC_TYPES::SUB_MESH)))
						{
							ccLog::Warning("[Graphical Segmentation Tool] Can't mix sub-meshes with their parent mesh!");
							return false;
						}
					}
				}
			}

			mesh->getAssociatedCloud()->resetVisibilityArray();
			m_toSegment.insert(mesh);
			result = true;
		}
	}
	else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//automatically add entity's children
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
			result |= addEntity(entity->getChild(i));
	}

	return result;
}

unsigned ccGraphicalMultipleSegmentationTool::getNumberOfValidEntities() const
{
	return static_cast<unsigned>(m_toSegment.size());
}

void ccGraphicalMultipleSegmentationTool::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	//process not started yet?
	if ((m_state & RUNNING) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);

	unsigned vertCount = m_polyVertices->size();

	//new point (expressed relatively to the screen center)
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	if (m_state & RECTANGLE)
	{
		//we need 4 points for the rectangle!
		if (vertCount != 4)
			m_polyVertices->resize(4);

		const CCVector3* A = m_polyVertices->getPointPersistentPtr(0);
		CCVector3* B = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(1));
		CCVector3* C = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(2));
		CCVector3* D = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(3));
		*B = CCVector3(A->x, P.y, 0);
		*C = P;
		*D = CCVector3(P.x, A->y, 0);

		if (vertCount != 4)
		{
			m_segmentationPoly->clear();
			if (!m_segmentationPoly->addPointIndex(0, 4))
			{
				ccLog::Error("Out of memory!");
				allowPolylineExport(false);
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
	}
	else if (m_state & POLYLINE)
	{
		if (vertCount < 2)
			return;
		//we replace last point by the current one
		CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
		*lastP = P;
	}

	m_associatedWin->redraw(true, false);
}

void ccGraphicalMultipleSegmentationTool::addPointToPolyline(int x, int y)
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

	assert(m_polyVertices);
	assert(m_segmentationPoly);
	unsigned vertCount = m_polyVertices->size();

	//particular case: we close the rectangular selection by a 2nd click
	if (m_rectangularSelection && vertCount == 4 && (m_state & RUNNING))
		return;

	//new point
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
		static_cast<PointCoordinateType>(pos2D.y()),
		0);

	//CTRL key pressed at the same time?
	bool ctrlKeyPressed = m_rectangularSelection || ((QApplication::keyboardModifiers() & Qt::ControlModifier) == Qt::ControlModifier);

	//start new polyline?
	if (((m_state & RUNNING) == 0) || vertCount == 0 || ctrlKeyPressed)
	{
		//reset state
		m_state = (ctrlKeyPressed ? RECTANGLE : POLYLINE);
		m_state |= (STARTED | RUNNING);
		//reset polyline
		m_polyVertices->clear();
		if (!m_polyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			allowPolylineExport(false);
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->clear();
		if (!m_segmentationPoly->addPointIndex(0, 2))
		{
			ccLog::Error("Out of memory!");
			allowPolylineExport(false);
			return;
		}
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE)
		{
			if (!m_polyVertices->reserve(vertCount + 1))
			{
				ccLog::Error("Out of memory!");
				allowPolylineExport(false);
				return;
			}

			//we replace last point by the current one
			CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount - 1));
			*lastP = P;
			//and add a new (equivalent) one
			m_polyVertices->addPoint(P);
			if (!m_segmentationPoly->addPointIndex(vertCount))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
		else //we must change mode
		{
			assert(false); //we shouldn't fall here?!
			m_state &= (~RUNNING);
			addPointToPolyline(x, y);
			return;
		}
	}

	m_associatedWin->redraw(true, false);
}

void ccGraphicalMultipleSegmentationTool::closeRectangle()
{
	//only for rectangle selection in RUNNING mode
	if ((m_state & RECTANGLE) == 0 || (m_state & RUNNING) == 0)
		return;

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		//first point only? we keep the real time update mechanism
		if (m_rectangularSelection)
			return;
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		allowPolylineExport(false);
	}
	else
	{
		allowPolylineExport(true);
	}

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
		m_associatedWin->redraw(true, false);
}

void ccGraphicalMultipleSegmentationTool::closePolyLine(int, int)
{
	//only for polyline in RUNNING mode
	if ((m_state & POLYLINE) == 0 || (m_state & RUNNING) == 0)
		return;

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}
	else
	{
		//remove last point!
		m_segmentationPoly->resize(vertCount - 1); //can't fail --> smaller
		m_segmentationPoly->setClosed(true);
	}

	//stop
	m_state &= (~RUNNING);

	//set the default import/export icon to 'export' mode
	allowPolylineExport(m_segmentationPoly->size() > 1);

	if (m_associatedWin)
	{
		m_associatedWin->redraw(true, false);
	}
}

void ccGraphicalMultipleSegmentationTool::segmentIn()
{
	m_segmentationPoly->setPointInside(true);
	segment(true);
	addSliceButton->setEnabled(true);
	

}

void ccGraphicalMultipleSegmentationTool::segmentOut()
{
	m_segmentationPoly->setPointInside(false);
	segment(false);
	addSliceButton->setEnabled(true);
	
}

void ccGraphicalMultipleSegmentationTool::segment(bool keepPointsInside)
{

	if (!m_associatedWin)
		return;

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return;
	}

	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//check if the polyline is totally inside the frustum or not
	bool polyInsideFrustum = true;
	{
		int vertexCount = static_cast<int>(m_segmentationPoly->size());
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < vertexCount; ++i)
		{
			const CCVector3* P = m_segmentationPoly->getPoint(i);

			CCVector3d Q2D;
			bool pointInFrustum = false;
			camera.project(*P, Q2D, &pointInFrustum);

			if (!pointInFrustum)
			{
				polyInsideFrustum = false;
			}
		}
	}
	ccLog::PrintDebug("Polyline is fully inside frustrum: " + QString(polyInsideFrustum ? "Yes" : "No"));

	//for each selected entity
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);

		ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
		assert(!visibilityArray.empty());

		int cloudSize = static_cast<int>(cloud->size());

		//we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < cloudSize; ++i)
		{
			if (visibilityArray[i] == CCCoreLib::POINT_VISIBLE)
			{
				const CCVector3* P3D = cloud->getPoint(i);

				CCVector3d Q2D;
				bool pointInFrustum = false;
				camera.project(*P3D, Q2D, &pointInFrustum);

				bool pointInside = false;
				if (pointInFrustum || !polyInsideFrustum) //we can only skip the test if the polyline is fully inside the frustum
				{
					CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
						static_cast<PointCoordinateType>(Q2D.y - half_h));

					pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);
				}

				visibilityArray[i] = (keepPointsInside != pointInside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIGHLIGHTED);
			}
		}
	}
	// Save viewport

	cc2DViewportObject* viewportObject = new cc2DViewportObject();
	viewportObject->setParameters(m_associatedWin->getViewportParameters());
	viewportObject->setDisplay(m_associatedWin);
	m_viewportGroup.push_back(viewportObject);
	// Save Polyline 

	m_segmentationPoly->setGroupIndex(m_currentGroupIndex);

	
	ccPolyline* poly = new ccPolyline(*m_segmentationPoly);
	m_polyGroup.push_back(poly);

	m_somethingHasChanged = true;
	validButton->setEnabled(true);
	cancelCurrentSelectionButton->setEnabled(true);
	pauseSegmentationMode(true);
	pauseSegmentationMode(false);
	m_segmentationPoly->clear();
	m_polyVertices->clear();
}

void ccGraphicalMultipleSegmentationTool::segmentByEntity(bool keepPointsInside, ccGenericPointCloud* cloud)
{


	if (!m_associatedWin)
		return;

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return;
	}


	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//check if the polyline is totally inside the frustum or not
	bool polyInsideFrustum = true;
	{
		int vertexCount = static_cast<int>(m_segmentationPoly->size());
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < vertexCount; ++i)
		{
			const CCVector3* P = m_segmentationPoly->getPoint(i);

			CCVector3d Q2D;
			bool pointInFrustum = false;
			camera.project(*P, Q2D, &pointInFrustum);

			if (!pointInFrustum)
			{
				polyInsideFrustum = false;
			}
		}
	}
	ccLog::PrintDebug("Polyline is fully inside frustrum: " + QString(polyInsideFrustum ? "Yes" : "No"));

	//entity passed by the algo 

	ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
	assert(!visibilityArray.empty());

	int cloudSize = static_cast<int>(cloud->size());

	//we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
#pragma omp parallel for
#endif
	for (int i = 0; i < cloudSize; ++i)
	{
		if (visibilityArray[i] == CCCoreLib::POINT_VISIBLE)
		{
			const CCVector3* P3D = cloud->getPoint(i);

			CCVector3d Q2D;
			bool pointInFrustum = false;
			camera.project(*P3D, Q2D, &pointInFrustum);

			bool pointInside = false;
			if (pointInFrustum || !polyInsideFrustum) //we can only skip the test if the polyline is fully inside the frustum
			{
				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
					static_cast<PointCoordinateType>(Q2D.y - half_h));

				pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);
			}
			visibilityArray[i] = (keepPointsInside!= pointInside ? CCCoreLib::POINT_VISIBLE : CCCoreLib::POINT_HIDDEN);
		}
	}



}
void ccGraphicalMultipleSegmentationTool::redoSegmentation(bool keepPointsInside)
{
	bool keepIn = keepPointsInside ? true : false;

	if (!m_associatedWin)
		return;

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return;
	}

	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//check if the polyline is totally inside the frustum or not
	bool polyInsideFrustum = true;
	{
		int vertexCount = static_cast<int>(m_segmentationPoly->size());
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < vertexCount; ++i)
		{
			const CCVector3* P = m_segmentationPoly->getPoint(i);

			CCVector3d Q2D;
			bool pointInFrustum = false;
			camera.project(*P, Q2D, &pointInFrustum);

			if (!pointInFrustum)
			{
				polyInsideFrustum = false;
			}
		}
	}
	ccLog::PrintDebug("Polyline is fully inside frustrum: " + QString(polyInsideFrustum ? "Yes" : "No"));

	//for each selected entity
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);

		ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
		assert(!visibilityArray.empty());

		int cloudSize = static_cast<int>(cloud->size());

		//we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < cloudSize; ++i)
		{
			if (visibilityArray[i] == CCCoreLib::POINT_VISIBLE)
			{
				const CCVector3* P3D = cloud->getPoint(i);

				CCVector3d Q2D;
				bool pointInFrustum = false;
				camera.project(*P3D, Q2D, &pointInFrustum);

				bool pointInside = false;
				if (pointInFrustum || !polyInsideFrustum) //we can only skip the test if the polyline is fully inside the frustum
				{
					CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
						static_cast<PointCoordinateType>(Q2D.y - half_h));

					pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);
				}

				visibilityArray[i] = (keepPointsInside == pointInside ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE);
			}
		}
	}

	m_somethingHasChanged = true;
	validButton->setEnabled(true);
	cancelCurrentSelectionButton->setEnabled(false);
	pauseSegmentationMode(true);
}

void ccGraphicalMultipleSegmentationTool::pauseSegmentationMode(bool state)
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
		return;

	if (state/*=activate pause mode*/)
	{

		m_state = PAUSED;
		if (m_polyVertices->size() != 0)
		{
			m_segmentationPoly->clear();
			m_polyVertices->clear();
			allowPolylineExport(false);
		}
		m_associatedWin->setInteractionMode(ccGLWindow::MODE_TRANSFORM_CAMERA);
		m_associatedWin->displayNewMessage("Segmentation [PAUSED HERE]", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->displayNewMessage("Unpause to segment again", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);

	}
	else
	{
		m_state = STARTED;
		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		if (m_rectangularSelection)
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Left click: set opposite corners", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
		else
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (polygonal selection)", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Left click: add contour points / Right click: close", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
	}

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);
	m_associatedWin->redraw(!state);
}

void ccGraphicalMultipleSegmentationTool::doSetPolylineSelection()
{
	if (!m_rectangularSelection)
		return;

	selectionModelButton->setDefaultAction(actionSetPolylineSelection);

	m_rectangularSelection = false;
	if (m_state != PAUSED)
	{
		pauseSegmentationMode(true);
		pauseSegmentationMode(false);
	}

	m_associatedWin->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	m_associatedWin->displayNewMessage("Right click: set opposite corners", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
}

void ccGraphicalMultipleSegmentationTool::doSetRectangularSelection()
{
	if (m_rectangularSelection)
		return;

	selectionModelButton->setDefaultAction(actionSetRectangularSelection);

	m_rectangularSelection = true;
	if (m_state != PAUSED)
	{
		pauseSegmentationMode(true);
		pauseSegmentationMode(false);
	}

	m_associatedWin->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	m_associatedWin->displayNewMessage("Right click: set opposite corners", ccGLWindow::UPPER_CENTER_MESSAGE, true, 3600, ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
}

void ccGraphicalMultipleSegmentationTool::doActionUseExistingPolyline()
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		ccHObject* root = mainWindow->dbRootObject();
		ccHObject::Container polylines;
		if (root)
		{
			root->filterChildren(polylines, true, CC_TYPES::POLY_LINE);
		}

		if (!polylines.empty())
		{
			int index = ccItemSelectionDlg::SelectEntity(polylines, 0, this);
			if (index < 0)
				return;
			assert(index >= 0 && index < static_cast<int>(polylines.size()));
			assert(polylines[index]->isA(CC_TYPES::POLY_LINE));
			ccPolyline* poly = static_cast<ccPolyline*>(polylines[index]);

			//look for an associated viewport
			ccHObject::Container viewports;
			if (poly->filterChildren(viewports, false, CC_TYPES::VIEWPORT_2D_OBJECT, true) == 1)
			{
				//shall we apply this viewport?
				if (QMessageBox::question(m_associatedWin->asWidget(),
					"Associated viewport",
					"The selected polyline has an associated viewport: do you want to apply it?",
					QMessageBox::Yes,
					QMessageBox::No) == QMessageBox::Yes)
				{
					m_associatedWin->setViewportParameters(static_cast<cc2DViewportObject*>(viewports.front())->getParameters());
					m_associatedWin->redraw(false);
				}
			}

			CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
			bool mode3D = !poly->is2DMode();

			//viewing parameters (for conversion from 3D to 2D)
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;

			//force polygonal selection mode
			doSetPolylineSelection();
			m_segmentationPoly->clear();
			m_polyVertices->clear();
			allowPolylineExport(false);

			//duplicate polyline 'a minima' (only points and indexes + closed state)
			if (m_polyVertices->reserve(vertices->size() + (poly->isClosed() ? 0 : 1))
				&& m_segmentationPoly->reserve(poly->size() + (poly->isClosed() ? 0 : 1)))
			{
				for (unsigned i = 0; i < vertices->size(); ++i)
				{
					CCVector3 P = *vertices->getPoint(i);
					if (mode3D)
					{
						CCVector3d Q2D;
						camera.project(P, Q2D);

						P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
						P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
						P.z = 0;
					}
					m_polyVertices->addPoint(P);
				}
				for (unsigned j = 0; j < poly->size(); ++j)
				{
					m_segmentationPoly->addPointIndex(poly->getPointGlobalIndex(j));
				}

				m_segmentationPoly->setClosed(poly->isClosed());
				if (m_segmentationPoly->isClosed())
				{
					//stop (but we can't all pauseSegmentationMode as it would remove the current polyline)
					m_state &= (~RUNNING);
					allowPolylineExport(m_segmentationPoly->size() > 1);
				}
				else if (vertices->size())
				{
					//we make as if the segmentation was in progress
					pauseSegmentationMode(false);
					unsigned lastIndex = vertices->size() - 1;
					m_polyVertices->addPoint(*m_polyVertices->getPoint(lastIndex));
					m_segmentationPoly->addPointIndex(lastIndex + 1);
					m_segmentationPoly->setClosed(true);
					m_state |= (POLYLINE | RUNNING);
				}

				m_rectangularSelection = false;
				m_associatedWin->redraw(true, false);
			}
			else
			{
				ccLog::Error("Not enough memory!");
			}
		}
		else
		{
			ccLog::Error("No polyline in DB!");
		}
	}
}
void ccGraphicalMultipleSegmentationTool::useExistingPolyline(ccPolyline* poly, cc2DViewportObject* viewport)
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{

		m_associatedWin->setViewportParameters(viewport->getParameters());
		m_associatedWin->redraw(false);

		CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
		bool mode3D = !poly->is2DMode();

		//viewing parameters (for conversion from 3D to 2D)
		ccGLCameraParameters camera;
		m_associatedWin->getGLCameraParameters(camera);
		const double half_w = camera.viewport[2] / 2.0;
		const double half_h = camera.viewport[3] / 2.0;

		//force polygonal selection mode
		doSetPolylineSelection();
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		allowPolylineExport(false);
		//duplicate polyline 'a minima' (only points and indexes + closed state)
		if (m_polyVertices->reserve(vertices->size() + (poly->isClosed() ? 0 : 1))
			&& m_segmentationPoly->reserve(poly->size() + (poly->isClosed() ? 0 : 1)))
		{
			for (unsigned i = 0; i < vertices->size(); ++i)
			{
				CCVector3 P = *vertices->getPoint(i);
				if (mode3D)
				{
					CCVector3d Q2D;
					camera.project(P, Q2D);

					P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
					P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
					P.z = 0;
				}
				m_polyVertices->addPoint(P);
			}
			for (unsigned j = 0; j < poly->size(); ++j)
			{
				m_segmentationPoly->addPointIndex(poly->getPointGlobalIndex(j));
			}

			m_segmentationPoly->setClosed(poly->isClosed());
			if (m_segmentationPoly->isClosed())
			{
				//stop (but we can't all pauseSegmentationMode as it would remove the current polyline)
				m_state &= (~RUNNING);
				allowPolylineExport(m_segmentationPoly->size() > 1);
			}
			else if (vertices->size())
			{
				//we make as if the segmentation was in progress
				pauseSegmentationMode(false);
				unsigned lastIndex = vertices->size() - 1;
				m_polyVertices->addPoint(*m_polyVertices->getPoint(lastIndex));
				m_segmentationPoly->addPointIndex(lastIndex + 1);
				m_segmentationPoly->setClosed(true);
				m_state |= (POLYLINE | RUNNING);
			}

			m_rectangularSelection = false;
			m_associatedWin->redraw(true, false);
		}
		else
		{
			ccLog::Error("Not enough memory!");
		}
	}
}


static unsigned s_polylineExportCount = 0;
void ccGraphicalMultipleSegmentationTool::doExportSegmentationPolyline()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow && m_segmentationPoly)
	{
		bool mode2D = false;
#ifdef ALLOW_2D_OR_3D_EXPORT
		QMessageBox messageBox(0);
		messageBox.setWindowTitle("Choose export type");
		messageBox.setText("Export polyline in:\n - 2D (with coordinates relative to the screen)\n - 3D (with coordinates relative to the segmented entities)");
		QPushButton* button2D = new QPushButton("2D");
		QPushButton* button3D = new QPushButton("3D");
		messageBox.addButton(button2D, QMessageBox::AcceptRole);
		messageBox.addButton(button3D, QMessageBox::AcceptRole);
		messageBox.addButton(QMessageBox::Cancel);
		messageBox.setDefaultButton(button3D);
		messageBox.exec();
		if (messageBox.clickedButton() == messageBox.button(QMessageBox::Cancel))
		{
			//process cancelled by user
			return;
		}
		mode2D = (messageBox.clickedButton() == button2D);
#endif

		ccPolyline* poly = new ccPolyline(*m_segmentationPoly);

		//if the polyline is 2D and we export the polyline in 3D, we must project its vertices
		if (!mode2D)
		{
			//get current display parameters
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;

			//project the 2D polyline in 3D
			CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
			ccPointCloud* verticesPC = dynamic_cast<ccPointCloud*>(vertices);
			if (verticesPC)
			{
				for (unsigned i = 0; i < vertices->size(); ++i)
				{
					CCVector3* Pscreen = const_cast<CCVector3*>(verticesPC->getPoint(i));
					CCVector3d Pd(half_w + Pscreen->x, half_h + Pscreen->y, 0/*Pscreen->z*/);
					CCVector3d Q3D;
					camera.unproject(Pd, Q3D);
					*Pscreen = CCVector3::fromArray(Q3D.u);
				}
				verticesPC->invalidateBoundingBox();
			}
			else
			{
				assert(false);
				ccLog::Warning("[Segmentation] Failed to convert 2D polyline to 3D! (internal inconsistency)");
				mode2D = false;
			}

			//export Global Shift & Scale info (if any)
			bool hasGlobalShift = false;
			CCVector3d globalShift(0, 0, 0);
			double globalScale = 1.0;
			{
				for (QSet<ccHObject*>::const_iterator it = m_toSegment.constBegin(); it != m_toSegment.constEnd(); ++it)
				{
					ccShiftedObject* shifted = ccHObjectCaster::ToShifted(*it);
					bool isShifted = (shifted && shifted->isShifted());
					if (isShifted)
					{
						globalShift = shifted->getGlobalShift();
						globalScale = shifted->getGlobalScale();
						hasGlobalShift = true;
						break;
					}
				}
			}

			if (hasGlobalShift && m_toSegment.size() != 1)
			{
				hasGlobalShift = (QMessageBox::question(MainWindow::TheInstance(), "Apply Global Shift", "At least one of the segmented entity has been shifted. Apply the same shift to the polyline?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
			}

			if (hasGlobalShift)
			{
				poly->setGlobalShift(globalShift);
				poly->setGlobalScale(globalScale);
			}
		}

		QString polyName = QString("Segmentation polyline #%1").arg(++s_polylineExportCount);
		poly->setName(polyName);
		poly->setEnabled(false); //we don't want it to appear while the segmentation mode is enabled! (anyway it's 2D only...)
		poly->set2DMode(mode2D);
		poly->setColor(ccColor::yellow); //we use a different color so as to differentiate them from the active polyline!

		//save associated viewport
		cc2DViewportObject* viewportObject = new cc2DViewportObject(polyName + QString(" viewport"));
		viewportObject->setParameters(m_associatedWin->getViewportParameters());
		viewportObject->setDisplay(m_associatedWin);
		poly->addChild(viewportObject);

		mainWindow->addToDB(poly, false, false, false);
		ccLog::Print(QString("[Segmentation] Polyline exported (%1 vertices)").arg(poly->size()));
	}
}

void ccGraphicalMultipleSegmentationTool::addToBeSliced()
{
	if (m_polyGroup.size() <= 0)
	{
		return;
	}
	bool newSegments = false;
	for (int i = 0; i < m_polyGroup.size(); i++)
	{
		ccPolyline* poly = m_polyGroup[i];
		if (poly->getGroupIndex() >= m_currentGroupIndex)
		{
			newSegments = true;
		}
	}
	if (!newSegments)
	{
		return;
	}
	//Changing the group index
	m_currentGroupIndex++;
	//Now that the polylines are saved with their corresponding view, we can now hide the higlighted points
	//for each selected entity
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);
		cloud->invertHighlightedArray();
	}
	m_somethingHasChanged = true;
	m_associatedWin->redraw();
	cancelPreviousCropButton->setEnabled(true);
	cancelCurrentSelectionButton->setEnabled(false);
	addSliceButton->setEnabled(false);
	m_segmentationPoly->clear();
	m_polyVertices->clear();
}
void ccGraphicalMultipleSegmentationTool::cancelCurrentSelection() {
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);
		cloud->cancelHighlightedArray();
	}
	unsigned currentGroupIndex;
	int j = 0;
	ccPolyline* poly;
	for (int i = 0; i < m_polyGroup.size(); i++)
	{
		poly = m_polyGroup[i];
		currentGroupIndex = poly->getGroupIndex();

		if (currentGroupIndex == m_currentGroupIndex)
		{
			j++;
		}
	}
	
	
	// Remove polylines and from saved vector
	m_polyGroup.erase(m_polyGroup.end() - j, m_polyGroup.end());
	m_viewportGroup.erase(m_viewportGroup.end() - j, m_viewportGroup.end());
	m_somethingHasChanged = true;
	m_associatedWin->redraw();
	cancelCurrentSelectionButton->setEnabled(false);
	addSliceButton->setEnabled(false);
	m_segmentationPoly->clear();
	m_polyVertices->clear();
	
}

void ccGraphicalMultipleSegmentationTool::cancelPreviousCrop()
{
	if (m_polyGroup.empty())
	{
		validButton->setEnabled(false);
		return;
	}
	if (m_currentGroupIndex > 0)
	{
		m_currentGroupIndex--;
	}
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);
		cloud->resetVisibilityArray();
	}
	unsigned currentGroupIndex;
	int j = 0;
	ccPolyline* poly;
	cc2DViewportObject* viewport;
	for (int i = 0; i < m_polyGroup.size(); i++)
	{
		poly = m_polyGroup[i];
		currentGroupIndex = poly->getGroupIndex();

		if (currentGroupIndex < m_currentGroupIndex)
		{
			viewport = m_viewportGroup[i];
			useExistingPolyline(poly, viewport);
			if (poly->getPointInside()) {
				redoSegmentation(true);
			}
			else
			{
				redoSegmentation(false);
				ccLog::Print("out");
			}

			j++;
		}
	}
	// Remove polylines and from saved vector
	m_polyGroup.erase(m_polyGroup.begin() + j, m_polyGroup.end());
	m_viewportGroup.erase(m_viewportGroup.begin() + j, m_viewportGroup.end());


	m_somethingHasChanged = true;
	m_associatedWin->redraw();
	cancelCurrentSelectionButton->setEnabled(false);
	if (m_viewportGroup.size() > 0)
	{
		cancelPreviousCropButton->setEnabled(true);
	}
	else
	{
		cancelPreviousCropButton->setEnabled(false);
	}
	addSliceButton->setEnabled(false);
	m_segmentationPoly->clear();
	m_polyVertices->clear();
	
}

void ccGraphicalMultipleSegmentationTool::segmentFromIndex(unsigned index, ccGenericPointCloud* cloud)
{

	ccPolyline* poly;
	cc2DViewportObject* viewport;
	cloud->resetVisibilityArray();
	for (int i = 0; i < m_polyGroup.size(); i++)
	{
		poly = m_polyGroup[i];
		
		if (poly->getGroupIndex() == index)
		{

			viewport = m_viewportGroup[i];
			useExistingPolyline(poly, viewport);
			if (poly->getPointInside()) {
				segmentByEntity(true,cloud);
			}
			else
			{
				segmentByEntity(false, cloud);
			}
		}

	}
	cloud->invertVisibilityArray();
}

void ccGraphicalMultipleSegmentationTool::apply()
{
	if (QMessageBox::question(m_associatedWin->asWidget(),
		"Segmenting",
		"Are you sure you want to segment in?",
		QMessageBox::Yes,
		QMessageBox::No) == QMessageBox::Yes)
	{
		if (m_polyGroup.size() > 0) {
			m_deleteHiddenParts = false;
			emit ccOverlayDialog::processFinished(true);
			stop(false);
		}
		else
		{
			cancel(false);
		}
	}
	else
	{
		return;
	}
	
}

void ccGraphicalMultipleSegmentationTool::applyAndDelete()
{
	m_deleteHiddenParts = true;
	stop(true);
}

void ccGraphicalMultipleSegmentationTool::cancel(bool ask=true)
{
	if (ask)
	{
		if (QMessageBox::question(m_associatedWin->asWidget(),
			"Cancel",
			"Are you sure you want to cancel all progress will be lost?",
			QMessageBox::Yes,
			QMessageBox::No) == QMessageBox::Yes)
		{

			reset();
			m_deleteHiddenParts = false;
			stop(false);
		}
		else
		{
			return;
		}
	}
	else
	{
		reset();
		m_deleteHiddenParts = false;
		stop(false);
	}
}