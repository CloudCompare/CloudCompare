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

#include "ccGraphicalSegmentationTool.h"

//Local
#include "ccGLWindow.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <Matrix.h>

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>

//Qt
#include <QMenu>

//System
#include <assert.h>

ccGraphicalSegmentationTool::ccGraphicalSegmentationTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::GraphicalSegmentationDlg()
	, m_somethingHasChanged(false)
	, m_state(0)
	, m_segmentationPoly(0)
	, m_polyVertices(0)
	, m_rectangularSelection(false)
	, m_deleteHiddenParts(false)
{
	// Set QDialog background as transparent (DGM: doesn't work over an OpenGL context)
	//setAttribute(Qt::WA_NoSystemBackground);

	setupUi(this);
	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(inButton,				SIGNAL(clicked()),		this,	SLOT(segmentIn()));
	connect(outButton,				SIGNAL(clicked()),		this,	SLOT(segmentOut()));
	connect(razButton,				SIGNAL(clicked()),		this,	SLOT(reset()));
	connect(validButton,			SIGNAL(clicked()),		this,	SLOT(apply()));
	connect(validAndDeleteButton,	SIGNAL(clicked()),		this,	SLOT(applyAndDelete()));
	connect(cancelButton,			SIGNAL(clicked()),		this,	SLOT(cancel()));
	connect(pauseButton,			SIGNAL(toggled(bool)),	this,	SLOT(pauseSegmentationMode(bool)));

	//selection modes
	connect(actionSetPolylineSelection,		SIGNAL(triggered()),	this,	SLOT(doSetPolylineSelection()));
	connect(actionSetRectangularSelection,	SIGNAL(triggered()),	this,	SLOT(doSetRectangularSelection()));

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space); //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
	addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button
	addOverridenShortcut(Qt::Key_Delete); //delete key for the "apply and delete" button
	addOverridenShortcut(Qt::Key_Tab); //tab key to switch between rectangular and polygonal selection modes
	addOverridenShortcut(Qt::Key_I); //'I' key for the "segment in" button
	addOverridenShortcut(Qt::Key_O); //'O' key for the "segment out" button
	connect(this, SIGNAL(shortcutTriggered(int)), this, SLOT(onShortcutTriggered(int)));

	QMenu* selectionModeMenu = new QMenu(this);
	selectionModeMenu->addAction(actionSetPolylineSelection);
	selectionModeMenu->addAction(actionSetRectangularSelection);
	selectionModelButton->setDefaultAction(actionSetPolylineSelection);
	selectionModelButton->setMenu(selectionModeMenu);

	m_polyVertices = new ccPointCloud();
	m_segmentationPoly = new ccPolyline(m_polyVertices);
	m_segmentationPoly->setForeground(true);
	m_segmentationPoly->setColor(ccColor::green);
	m_segmentationPoly->showColors(true);
	m_segmentationPoly->set2DMode(true);
}

ccGraphicalSegmentationTool::~ccGraphicalSegmentationTool()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = 0;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = 0;
}

void ccGraphicalSegmentationTool::onShortcutTriggered(int key)
{
 	switch(key)
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

	case Qt::Key_Return:
		validButton->click();
		return;
	case Qt::Key_Delete:
		validAndDeleteButton->click();
		return;
	case Qt::Key_Escape:
		cancelButton->click();
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

bool ccGraphicalSegmentationTool::linkWith(ccGLWindow* win)
{
	assert(m_segmentationPoly);

	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
		return false;

	if (oldWin)
	{
		disconnect(m_associatedWin, SIGNAL(leftButtonClicked(int,int)), this, SLOT(addPointToPolyline(int,int)));
		disconnect(m_associatedWin, SIGNAL(rightButtonClicked(int,int)), this, SLOT(closePolyLine(int,int)));
		disconnect(m_associatedWin, SIGNAL(mouseMoved(int,int,Qt::MouseButtons)), this, SLOT(updatePolyLine(int,int,Qt::MouseButtons)));
		disconnect(m_associatedWin, SIGNAL(buttonReleased()), this, SLOT(closeRectangle()));

		if (m_segmentationPoly)
			m_segmentationPoly->setDisplay(0);
	}
	
	if (m_associatedWin)
	{
		connect(m_associatedWin, SIGNAL(leftButtonClicked(int,int)), this, SLOT(addPointToPolyline(int,int)));
		connect(m_associatedWin, SIGNAL(rightButtonClicked(int,int)), this, SLOT(closePolyLine(int,int)));
		connect(m_associatedWin, SIGNAL(mouseMoved(int,int,Qt::MouseButtons)), this, SLOT(updatePolyLine(int,int,Qt::MouseButtons)));
		connect(m_associatedWin, SIGNAL(buttonReleased()), this, SLOT(closeRectangle()));

		if (m_segmentationPoly)
			m_segmentationPoly->setDisplay(m_associatedWin);
	}

	return true;
}

bool ccGraphicalSegmentationTool::start()
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
	{
		ccLog::Warning("[Graphical Segmentation Tool] No associated window!");
		return false;
	}

	m_segmentationPoly->clear();
	m_polyVertices->clear();

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_segmentationPoly);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	pauseSegmentationMode(false);

	m_somethingHasChanged = false;

	reset();

	return ccOverlayDialog::start();
}

void ccGraphicalSegmentationTool::removeAllEntities(bool unallocateVisibilityArrays)
{
	if (unallocateVisibilityArrays)
	{
		for (std::set<ccHObject*>::iterator p = m_toSegment.begin(); p != m_toSegment.end(); ++p)
		{
			ccHObject* entity = *p;

			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
				ccHObjectCaster::ToGenericPointCloud(entity)->unallocateVisibilityArray();
			else if (entity->isKindOf(CC_TYPES::MESH))
				ccHObjectCaster::ToGenericMesh(entity)->getAssociatedCloud()->unallocateVisibilityArray();
		}
	}

	m_toSegment.clear();
}

void ccGraphicalSegmentationTool::stop(bool accepted)
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin) //job already done
		return;

	m_associatedWin->displayNewMessage("Segmentation [OFF]",
										ccGLWindow::UPPER_CENTER_MESSAGE,
										false,
										2,
										ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);

	m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
	m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	m_associatedWin->setUnclosable(false);
	m_associatedWin->removeFromOwnDB(m_segmentationPoly);

	ccOverlayDialog::stop(accepted);
}

void ccGraphicalSegmentationTool::reset()
{
	if (m_somethingHasChanged)
	{
		for (std::set<ccHObject*>::iterator p = m_toSegment.begin(); p != m_toSegment.end(); ++p)
		{
			if ((*p)->isKindOf(CC_TYPES::POINT_CLOUD))
				ccHObjectCaster::ToGenericPointCloud(*p)->resetVisibilityArray();
			else if ((*p)->isKindOf(CC_TYPES::MESH))
				ccHObjectCaster::ToGenericMesh(*p)->getAssociatedCloud()->resetVisibilityArray();
		}

		if (m_associatedWin)
			m_associatedWin->redraw();
		m_somethingHasChanged = false;
	}

	razButton->setEnabled(false);
	validButton->setEnabled(false);
	validAndDeleteButton->setEnabled(false);
}

bool ccGraphicalSegmentationTool::addEntity(ccHObject* anObject)
{
	//FIXME
	/*if (anObject->isLocked())
		ccLog::Warning(QString("Can't use entity [%1] cause it's locked!").arg(anObject->getName()));
	else */
	if (anObject->getDisplay() != m_associatedWin)
	{
		ccLog::Warning(QString("[Graphical Segmentation Tool] Can't use entity [%1] cause it's not displayed in the active 3D view!").arg(anObject->getName()));
		return false;
	}
	if (!anObject->isVisible() || !anObject->isBranchEnabled())
	{
		ccLog::Warning(QString("[Graphical Segmentation Tool] Entity [%1] is not visible in the active 3D view!").arg(anObject->getName()));
	}

	bool result = false;
	if (anObject->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(anObject);
		//detect if this cloud is in fact a vertex set for at least one mesh
		{
			//either the cloud is the child of its parent mesh
			if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH) && ccHObjectCaster::ToGenericMesh(cloud->getParent())->getAssociatedCloud() == cloud)
			{
				ccLog::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its parent mesh instead!").arg(anObject->getName()));
				return false;
			}
			//or the parent of its child mesh!
			ccHObject::Container meshes;
			if (cloud->filterChildren(meshes,false,CC_TYPES::MESH) != 0)
			{
				for (unsigned i=0; i<meshes.size(); ++i)
					if (ccHObjectCaster::ToGenericMesh(meshes[i])->getAssociatedCloud() == cloud)
					{
						ccLog::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its child mesh instead!").arg(anObject->getName()));
						return false;
					}
			}
		}

		cloud->resetVisibilityArray();
		m_toSegment.insert(cloud);

		//automatically add cloud's children
		for (unsigned i=0; i<anObject->getChildrenNumber(); ++i)
			result |= addEntity(anObject->getChild(i));
	}
	else if (anObject->isKindOf(CC_TYPES::MESH))
	{
		if (anObject->isKindOf(CC_TYPES::PRIMITIVE))
		{
			ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment primitives yet! Sorry...");
			return false;
		}
		if (anObject->isKindOf(CC_TYPES::SUB_MESH))
		{
			ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment sub-meshes! Select the parent mesh...");
			return false;
		}
		else
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(anObject);

			//first, we must check that there's no mesh and at least one of its sub-mesh mixed in the current selection!
			for (std::set<ccHObject*>::iterator p = m_toSegment.begin(); p != m_toSegment.end(); ++p)
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
	else
	{
		//automatically add entity's children
		for (unsigned i=0;i<anObject->getChildrenNumber();++i)
			result |= addEntity(anObject->getChild(i));
	}

	return result;
}

unsigned ccGraphicalSegmentationTool::getNumberOfValidEntities() const
{
	return static_cast<unsigned>(m_toSegment.size());
}

void ccGraphicalSegmentationTool::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	//process not started yet?
	if ((m_state & RUNNING) == 0)
		return;

	assert(m_polyVertices);
	assert(m_segmentationPoly);

	unsigned sz = m_polyVertices->size();

	//new point
	CCVector3 P((PointCoordinateType)x,(PointCoordinateType)y,(PointCoordinateType)0);

	if (m_state & RECTANGLE)
	{
		//we need 4 points for the rectangle!
		if (sz!=4)
			m_polyVertices->resize(4);

		const CCVector3* A = m_polyVertices->getPointPersistentPtr(0);
		CCVector3* B = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(1));
		CCVector3* C = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(2));
		CCVector3* D = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(3));
		*B = CCVector3(A->x,P.y,0);
		*C = P;
		*D = CCVector3(P.x,A->y,0);

		if (sz!=4)
		{
			m_segmentationPoly->clear();
			if (!m_segmentationPoly->addPointIndex(0,4))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
	}
	else if (m_state & POLYLINE)
	{
		if (sz<2)
			return;
		//we replace last point by the current one
		CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(sz-1));
		*lastP = P;
	}

	if (m_associatedWin)
		m_associatedWin->updateGL();
}

void ccGraphicalSegmentationTool::addPointToPolyline(int x, int y)
{
	if ((m_state & STARTED) == 0)
		return;

	assert(m_polyVertices);
	assert(m_segmentationPoly);
	unsigned sz = m_polyVertices->size();

	//particular case: we close the rectangular selection by a 2nd click
	if (m_rectangularSelection && sz==4 && (m_state & RUNNING))
		return;

	//new point
	CCVector3 P((PointCoordinateType)x,(PointCoordinateType)y,(PointCoordinateType)0);

	//CTRL key pressed at the same time?
	bool ctrlKeyPressed = m_rectangularSelection || ((QApplication::keyboardModifiers() & Qt::ControlModifier) == Qt::ControlModifier);

	//start new polyline?
	if (((m_state & RUNNING) == 0) || sz==0 || ctrlKeyPressed)
	{
		//reset state
		m_state = (ctrlKeyPressed ? RECTANGLE : POLYLINE);
		m_state |= (STARTED | RUNNING);
		//reset polyline
		m_polyVertices->clear();
		if (!m_polyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->clear();
		if (!m_segmentationPoly->addPointIndex(0,2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE)
		{
			if (!m_polyVertices->reserve(sz+1))
			{
				ccLog::Error("Out of memory!");
				return;
			}

			//we replace last point by the current one
			CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(sz-1));
			*lastP = P;
			//and add a new (equivalent) one
			m_polyVertices->addPoint(P);
			if (!m_segmentationPoly->addPointIndex(sz))
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
			addPointToPolyline(x,y);
			return;
		}
	}

	if (m_associatedWin)
		m_associatedWin->updateGL();
}

void ccGraphicalSegmentationTool::closeRectangle()
{
	//only for rectangle selection in RUNNING mode
	if ((m_state & RECTANGLE)==0 || (m_state & RUNNING)==0)
		return;

	assert(m_segmentationPoly);
	unsigned sz = m_segmentationPoly->size();
	if (sz<4)
	{
		//first point only? we keep the real time update mechanism
		if (m_rectangularSelection)
			return;
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
		m_associatedWin->updateGL();
}

void ccGraphicalSegmentationTool::closePolyLine(int, int)
{
	//only for polyline in RUNNING mode
	if ((m_state & POLYLINE)==0 || (m_state & RUNNING)==0)
		return;

	assert(m_segmentationPoly);
	unsigned sz = m_segmentationPoly->size();
	if (sz<4)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}
	else
	{
		//remove last point!
		m_segmentationPoly->resize(sz-1); //can't fail --> smaller
		m_segmentationPoly->setClosed(true);
	}

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
		m_associatedWin->updateGL();
}

void ccGraphicalSegmentationTool::segmentIn()
{
	segment(true);
}

void ccGraphicalSegmentationTool::segmentOut()
{
	segment(false);
}

void ccGraphicalSegmentationTool::segment(bool keepPointsInside)
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
	const double* MM = m_associatedWin->getModelViewMatd(); //viewMat
	const double* MP = m_associatedWin->getProjectionMatd(); //projMat
	const GLdouble half_w = static_cast<GLdouble>(m_associatedWin->width())/2;
	const GLdouble half_h = static_cast<GLdouble>(m_associatedWin->height())/2;

	int VP[4];
	m_associatedWin->getViewportArray(VP);

	//for each selected entity
	for (std::set<ccHObject*>::iterator p = m_toSegment.begin(); p != m_toSegment.end(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);

		ccGenericPointCloud::VisibilityTableType* visibilityArray = cloud->getTheVisibilityArray();
		assert(visibilityArray);

		unsigned cloudSize = cloud->size();

		//we project each point and we check if it falls inside the segmentation polyline
		for (unsigned i=0; i<cloudSize; ++i)
		{
			if (visibilityArray->getValue(i) == POINT_VISIBLE)
			{
				CCVector3 P;
				cloud->getPoint(i,P);

				GLdouble xp,yp,zp;
				gluProject(P.x,P.y,P.z,MM,MP,VP,&xp,&yp,&zp);

				CCVector2 P2D(	static_cast<PointCoordinateType>(xp-half_w),
								static_cast<PointCoordinateType>(yp-half_h) );
				bool pointInside = CCLib::ManualSegmentationTools::isPointInsidePoly(P2D,m_segmentationPoly);

				visibilityArray->setValue(i, keepPointsInside != pointInside ? POINT_HIDDEN : POINT_VISIBLE );
			}
		}
	}

	m_somethingHasChanged = true;
	validButton->setEnabled(true);
	validAndDeleteButton->setEnabled(true);
	razButton->setEnabled(true);
	pauseSegmentationMode(true);
}

void ccGraphicalSegmentationTool::pauseSegmentationMode(bool state)
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
		return;

	if (state)
	{
		m_state = PAUSED;
		if (m_polyVertices->size()>0)
		{
			m_segmentationPoly->clear();
			m_polyVertices->clear();
		}
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
		m_associatedWin->displayNewMessage("Segmentation [PAUSED]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->displayNewMessage("Unpause to segment again",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	}
	else
	{
		m_state = STARTED;
		m_associatedWin->setInteractionMode(ccGLWindow::SEGMENT_ENTITY);
		if (m_rectangularSelection)
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Left click: set opposite corners",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
		else
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (polygonal selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Left click: add contour points / Right click: close",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
	}

	//update mini-GUI
	pauseButton->blockSignals(true);
	pauseButton->setChecked(state);
	pauseButton->blockSignals(false);

	m_associatedWin->redraw();
}

void ccGraphicalSegmentationTool::doSetPolylineSelection()
{
	if (!m_rectangularSelection)
		return;

	QIcon icon(QString::fromUtf8(":/CC/images/smallPolygonSelect.png"));
	selectionModelButton->setIcon(icon);

	m_rectangularSelection=false;
	if (m_state != PAUSED)
	{
		pauseSegmentationMode(true);
		pauseSegmentationMode(false);
	}

	m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	m_associatedWin->displayNewMessage("Right click: set opposite corners",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
}

void ccGraphicalSegmentationTool::doSetRectangularSelection()
{
	if (m_rectangularSelection)
		return;

	QIcon icon(QString::fromUtf8(":/CC/images/smallRectangleSelect.png"));
	selectionModelButton->setIcon(icon);

	m_rectangularSelection=true;
	if (m_state != PAUSED)
	{
		pauseSegmentationMode(true);
		pauseSegmentationMode(false);
	}

	m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	m_associatedWin->displayNewMessage("Right click: set opposite corners",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
}

void ccGraphicalSegmentationTool::apply()
{
	m_deleteHiddenParts = false;
	stop(true);
}

void ccGraphicalSegmentationTool::applyAndDelete()
{
	m_deleteHiddenParts = true;
	stop(true);
}

void ccGraphicalSegmentationTool::cancel()
{
	reset();
	m_deleteHiddenParts = false;
	stop(false);
}
