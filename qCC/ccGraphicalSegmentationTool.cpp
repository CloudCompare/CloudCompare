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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//

#include "ccGraphicalSegmentationTool.h"

//Local
#include "ccGLWindow.h"
#include "ccConsole.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <Matrix.h>

//qCC_db
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
	, m_deleteHiddenPoints(false)
{
    // Set QDialog background as transparent (DGM: doesn't work over an OpenGL context)
    //setAttribute(Qt::WA_NoSystemBackground);
	
	setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

    connect(inButton,				SIGNAL(clicked()),      this,   SLOT(segmentIn()));
    connect(outButton,				SIGNAL(clicked()),      this,   SLOT(segmentOut()));
    connect(razButton,				SIGNAL(clicked()),      this,   SLOT(reset()));
    connect(validButton,			SIGNAL(clicked()),      this,   SLOT(apply()));
    connect(validAndDeleteButton,	SIGNAL(clicked()),      this,   SLOT(applyAndDelete()));
    connect(cancelButton,			SIGNAL(clicked()),      this,   SLOT(cancel()));
    connect(pauseButton,			SIGNAL(toggled(bool)),  this,   SLOT(pauseSegmentationMode(bool)));

	//selection modes
	connect(actionSetPolylineSelection,		SIGNAL(triggered()),	this,	SLOT(doSetPolylineSelection()));
	connect(actionSetRectangularSelection,	SIGNAL(triggered()),	this,	SLOT(doSetRectangularSelection()));

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
    m_segmentationPoly=0;

    if (m_polyVertices)
        delete m_polyVertices;
    m_polyVertices=0;
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
        ccConsole::Warning("[Graphical Segmentation Tool] No associated window!");
        return false;
    }

    m_segmentationPoly->clear();
    m_polyVertices->clear();

    //the user must not close this window!
    m_associatedWin->setUnclosable(true);
    //m_associatedWin->setPerspectiveState(false,false);
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
		while (!m_toSegment.empty())
		{
			ccHObject* entity = m_toSegment.back();
			m_toSegment.pop_back();

			if (entity->isKindOf(CC_POINT_CLOUD))
				static_cast<ccGenericPointCloud*>(entity)->unallocateVisibilityArray();
			else if (entity->isKindOf(CC_MESH))
				static_cast<ccGenericMesh*>(entity)->getAssociatedCloud()->unallocateVisibilityArray();
		}
	}
	else
	{
		m_toSegment.clear();
	}
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
        ccHObject::Container::iterator p;
        for (p=m_toSegment.begin();p!=m_toSegment.end();++p)
        {
            if ((*p)->isKindOf(CC_POINT_CLOUD))
                static_cast<ccGenericPointCloud*>(*p)->razVisibilityArray();
            else if ((*p)->isKindOf(CC_MESH))
                static_cast<ccGenericMesh*>(*p)->getAssociatedCloud()->razVisibilityArray();
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
        ccConsole::Warning(QString("Can't use entity [%1] cause it's locked!").arg(anObject->getName()));
    else */
    if (anObject->getDisplay() != m_associatedWin)
    {
        ccConsole::Warning(QString("[Graphical Segmentation Tool] Can't use entity [%1] cause it's not displayed in the active 3D view!").arg(anObject->getName()));
		return false;
    }

	bool result = false;
	if (anObject->isKindOf(CC_POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(anObject);
		//detect vertices
		if (cloud->getParent() && cloud->getParent()->isKindOf(CC_MESH) && static_cast<ccGenericMesh*>(cloud->getParent())->getAssociatedCloud() == cloud)
		{
			ccConsole::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its parent mesh instead!").arg(anObject->getName()));
			return false;
		}
		cloud->razVisibilityArray();
		m_toSegment.push_back(cloud);

		for (unsigned i=0;i<anObject->getChildrenNumber();++i)
			result |= addEntity(anObject->getChild(i));
	}
	else if (anObject->isKindOf(CC_MESH))
	{
		if (anObject->isKindOf(CC_PRIMITIVE))
		{
			ccLog::Warning("[ccGraphicalSegmentationTool] Can't segment primitives yet! Sorry...");
		}
		else
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(anObject);
			mesh->getAssociatedCloud()->razVisibilityArray();
			m_toSegment.push_back(mesh);
			result = true;
		}
	}
	else
	{
		for (unsigned i=0;i<anObject->getChildrenNumber();++i)
			result |= addEntity(anObject->getChild(i));
	}

	return result;
}

unsigned ccGraphicalSegmentationTool::getNumberOfValidEntities()
{
    return m_toSegment.size();
}

ccHObject* ccGraphicalSegmentationTool::getEntity(unsigned pos)
{
	return (pos<m_toSegment.size() ? m_toSegment[pos] : 0);
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
			m_segmentationPoly->reserve(4);
			for (unsigned i=0;i<4;++i)
				m_segmentationPoly->addPointIndex(i);
			m_segmentationPoly->setClosingState(true);
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
		//reset polyline
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		//reset state
		m_state = (ctrlKeyPressed ? RECTANGLE : POLYLINE);
		m_state |= (STARTED | RUNNING);
		//we add the same point twice (the last point will be used for display only)
		m_polyVertices->reserve(2);
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->addPointIndex(0);
		m_segmentationPoly->addPointIndex(1);
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE)
		{
			m_polyVertices->reserve(sz+4);

			//we replace last point by the current one
			CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(sz));
			*lastP = P;
			//and add a new (equivalent) one
			m_polyVertices->addPoint(P);
			m_segmentationPoly->addPointIndex(sz);
			m_segmentationPoly->setClosingState(true);
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
		m_segmentationPoly->resize(sz-1);
		m_segmentationPoly->setClosingState(true);
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

void ccGraphicalSegmentationTool::segment(bool inside)
{
    if (!m_associatedWin)
        return;

    if (!m_segmentationPoly)
    {
        ccConsole::Error("No polyline defined!");
        return;
    }

    if (!m_segmentationPoly->getClosingState())
    {
        ccConsole::Error("Define and/or close the segmentation border first! (right click to close)");
        return;
    }

    //viewing parameters
    const double* MM = m_associatedWin->getModelViewMatd(); //viewMat
	const double* MP = m_associatedWin->getProjectionMatd(); //projMat
	const float half_w = (float)m_associatedWin->width() * 0.5f;
	const float half_h = (float)m_associatedWin->height() * 0.5f;

	int VP[4];
	m_associatedWin->getViewportArray(VP);

	CCVector3 P;
	CCVector2 P2D;
	bool pointInside;

    //for each selected entity
    for (ccHObject::Container::iterator p=m_toSegment.begin();p!=m_toSegment.end();++p)
    {
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
        assert(cloud);

        ccGenericPointCloud::VisibilityTableType* vis = cloud->getTheVisibilityArray();
		assert(vis);

        unsigned i,cloudSize = cloud->size();

        //we project each point and we check if it falls inside the segmentation polyline
        for (i=0;i<cloudSize;++i)
        {
			cloud->getPoint(i,P);

			GLdouble xp,yp,zp;
			gluProject(P.x,P.y,P.z,MM,MP,VP,&xp,&yp,&zp);
			P2D.x = xp - half_w;
			P2D.y = yp - half_h;

			pointInside = CCLib::ManualSegmentationTools::isPointInsidePoly(P2D,m_segmentationPoly);

            if ((inside && !pointInside)||(!inside && pointInside))
				vis->setValue(i,0); //hiddenValue=0
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
        m_associatedWin->displayNewMessage("Unpause to segment",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
    }
    else
    {
		m_state = STARTED;
		m_associatedWin->setInteractionMode(ccGLWindow::SEGMENT_ENTITY);
		if (m_rectangularSelection)
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (rectangular selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Right click: set opposite corners",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
		else
		{
			m_associatedWin->displayNewMessage("Segmentation [ON] (polygonal selection)",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
			m_associatedWin->displayNewMessage("Right click: add contour points / Left click: close",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		}
    }

	//update mini-GUI
	pauseButton->setChecked(state);

    m_associatedWin->redraw();
}

void ccGraphicalSegmentationTool::doSetPolylineSelection()
{
	if (!m_rectangularSelection)
		return;

	QIcon icon(QString::fromUtf8(":/CC/Comp/images/comp/polygonSelect.png"));
	//QIcon icon;
	//icon.addFile(QString::fromUtf8(":/CC/Comp/images/comp/polygonSelect.png"), QSize(), QIcon::Normal, QIcon::Off);
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

	QIcon icon(QString::fromUtf8(":/CC/Comp/images/comp/rectangleSelect.png"));
	//QIcon icon;
	//icon.addFile(QString::fromUtf8(":/CC/Comp/images/comp/rectangleSelect.png"), QSize(), QIcon::Normal, QIcon::Off);
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
	m_deleteHiddenPoints = false;
	stop(true);
}

void ccGraphicalSegmentationTool::applyAndDelete()
{
	m_deleteHiddenPoints = true;
	stop(true);
}

void ccGraphicalSegmentationTool::cancel()
{
	reset();
	m_deleteHiddenPoints = false;
	stop(false);
}
