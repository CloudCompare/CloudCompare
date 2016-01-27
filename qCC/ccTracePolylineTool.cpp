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

#include "ccTracePolylineTool.h"

//Local
#include "mainwindow.h"
#include "ccEntityPickerDlg.h"

//CCLib
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

#include <iostream>

ccTracePolylineTool::ccTracePolylineTool(QWidget* parent)
    : ccOverlayDialog(parent)
    , Ui::TracePolyLineDlg()
    , m_polyTip(0)
    , m_polyTipVertices(0)
    , m_poly3D(0)
    , m_poly3DVertices(0)
	, m_done(false)
{
    setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

    connect(saveToolButton, SIGNAL(clicked()), this, SLOT(exportLine()));
    connect(resetToolButton, SIGNAL(clicked()), this, SLOT(resetLine()));
    connect(validButton, SIGNAL(clicked()), this, SLOT(apply()));
    connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancel()));
	connect(snapSizeSpinBox, SIGNAL(valueChanged(int)), this, SLOT(onSnapSizeChanged(int)));
	connect(widthSpinBox, SIGNAL(valueChanged(int)), this, SLOT(onWidthSizeChanged(int)));

	//add shortcuts
    addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
    addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button
    connect(this, SIGNAL(shortcutTriggered(int)), this, SLOT(onShortcutTriggered(int)));

    m_polyTipVertices = new ccPointCloud("Tip vertices");
	m_polyTipVertices->reserve(2);
	m_polyTipVertices->addPoint(CCVector3(0,0,0));
	m_polyTipVertices->addPoint(CCVector3(1,1,1));
	m_polyTipVertices->setEnabled(false);

	m_polyTip = new ccPolyline(m_polyTipVertices);
    m_polyTip->setForeground(true);
	m_polyTip->setTempColor(ccColor::green);
    m_polyTip->set2DMode(true);
	m_polyTip->reserve(2);
	m_polyTip->addPointIndex(0,2);
	m_polyTip->setWidth(widthSpinBox->value());
	m_polyTip->addChild(m_polyTipVertices);

	validButton->setEnabled(false);
}

ccTracePolylineTool::~ccTracePolylineTool()
{
    if (m_polyTip)
        delete m_polyTip;
	//DGM: already a child of m_polyTip
    //if (m_polyTipVertices)
	//	delete m_polyTipVertices;

	if (m_poly3D)
		delete m_poly3D;
	//DGM: already a child of m_poly3D
	//if (m_poly3DVertices)
	//	delete m_poly3DVertices;
}

void ccTracePolylineTool::onShortcutTriggered(int key)
{
    switch (key)
	{
    case Qt::Key_Return:
        apply();
        return;

	case Qt::Key_Escape:
        cancel();
        return;

	default:
        //nothing to do
        break;
    }
}

ccPolyline* ccTracePolylineTool::PolylineOverSampling(ccPolyline* polyline, ccPointCloud* vertices, unsigned steps)
{
	if (!polyline || !vertices)
	{
		assert(false);
		return 0;
	}
    
	if (steps <= 1)
	{
		//nothing to do
        return 0;
	}

    unsigned n_verts = vertices->size();
	unsigned n_segments = polyline->size() - (polyline->isClosed() ? 0 : 1);
    unsigned end_size = n_segments * steps + (polyline->isClosed() ? 0 : 1);

	ccPointCloud* newVertices = new ccPointCloud();
	ccPolyline* newPoly = new ccPolyline(newVertices);
	newPoly->addChild(newVertices);

	if (	!newVertices->reserve(end_size)
		||	!newPoly->reserve(end_size) )
	{
		ccLog::Warning("[ccTracePolylineTool::PolylineOverSampling] Not enough memory");
		delete newPoly;
		return 0;
	}
	newVertices->importParametersFrom(vertices);
	newVertices->setName(vertices->getName());
	newPoly->importParametersFrom(*polyline);
	newPoly->setDisplay_recursive(polyline->getDisplay());


	for (unsigned i = 0; i < n_segments; ++i)
	{
        const CCVector3* p1 = vertices->getPoint(i);
        newVertices->addPoint(*p1);

        const CCVector3* p2 = vertices->getPoint((i + 1) % n_verts); // the next point in polyline
        CCVector3 v = *p2 - *p1;
        v /= steps;

        for (unsigned j = 1; j < steps; j++)
		{
            CCVector3 newPoint = *p1 + j*v;
			
			//FIXME: find the nearest point in the cloud?

            newVertices->addPoint(newPoint);
        }
    }

	//add last point
	if (!polyline->isClosed())
	{
		newVertices->addPoint(*vertices->getPoint(n_verts - 1));
	}
	
	newPoly->addPointIndex(0, newVertices->size());

	return newPoly;
}

bool ccTracePolylineTool::linkWith(ccGLWindow* win)
{
    assert(m_polyTip);
	assert(!m_poly3D);

    ccGLWindow* oldWin = m_associatedWin;

    if (!ccOverlayDialog::linkWith(win))
	{
        return false;
	}

    if (oldWin)
	{
		oldWin->removeFromOwnDB(m_polyTip);
        oldWin->disconnect(this);

		if (m_polyTip)
            m_polyTip->setDisplay(0);
    }

    if (m_associatedWin)
	{
		connect(m_associatedWin, SIGNAL(itemPicked(int, unsigned, int, int)), this, SLOT(handlePickedItem(int, unsigned, int, int)));
        //connect(m_associatedWin, SIGNAL(leftButtonClicked(int, int)), this, SLOT(addPointToPolyline(int, int)));
        connect(m_associatedWin, SIGNAL(rightButtonClicked(int, int)), this, SLOT(closePolyLine(int, int)));
        connect(m_associatedWin, SIGNAL(mouseMoved(int, int, Qt::MouseButtons)), this, SLOT(updatePolyLineTip(int, int, Qt::MouseButtons)));
    }

    return true;
}

bool ccTracePolylineTool::start()
{
    assert(m_polyTip);
	assert(!m_poly3D);

    if (!m_associatedWin)
	{
        ccLog::Warning("[Trace Polyline Tool] No associated window!");
        return false;
    }

    m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_polyTip);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA() | ccGLWindow::INTERACT_SIG_RB_CLICKED | ccGLWindow::INTERACT_SIG_MOUSE_MOVED);
    m_associatedWin->setCursor(Qt::CrossCursor);

	snapSizeSpinBox->blockSignals(true);
	snapSizeSpinBox->setValue(m_associatedWin->getPickingRadius());
	snapSizeSpinBox->blockSignals(false);
	
	resetLine(); //to reset the GUI

    return ccOverlayDialog::start();
}

void ccTracePolylineTool::stop(bool accepted)
{
    assert(m_polyTip);

	if (m_associatedWin)
	{
		m_associatedWin->displayNewMessage("Polyline tracing [OFF]",
			ccGLWindow::UPPER_CENTER_MESSAGE,
			false,
			2,
			ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);

		m_associatedWin->setUnclosable(false);
		m_associatedWin->setPickingRadius(ccGLWindow::DefaultPickRadius);
		m_associatedWin->removeFromOwnDB(m_polyTip);
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setCursor(Qt::ArrowCursor);
	}

    ccOverlayDialog::stop(accepted);
}

void ccTracePolylineTool::updatePolyLineTip(int x, int y, Qt::MouseButtons buttons)
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}
	
	if (buttons != Qt::NoButton)
	{
		//nothing to do (just hide the tip)
		if (m_polyTip->isEnabled())
		{
			m_polyTip->setEnabled(false);
			m_associatedWin->redraw(true, false);
		}
		return;
	}
    
	if (!m_poly3DVertices || m_poly3DVertices->size() == 0)
	{
		//there should be at least one point already picked!
		return;
	}

    if (m_done)
	{
		// when it is done do nothing
        return;
	}

    assert(m_polyTip && m_polyTipVertices && m_polyTipVertices->size() == 2);

    //we replace the last point by the new one
	{
		CCVector3 P2D(	static_cast<PointCoordinateType>(x - m_associatedWin->width() / 2),
						static_cast<PointCoordinateType>(m_associatedWin->height() / 2 - y),
						0);

		CCVector3* lastP = const_cast<CCVector3*>(m_polyTipVertices->getPointPersistentPtr(1));
		*lastP = P2D;
	}

	//just in case (e.g. if the view has been rotated or zoomed)
	//we also update the first vertex position!
	{
		const CCVector3* P3D = m_poly3DVertices->getPoint(m_poly3DVertices->size()-1);

		int VP[4];
		m_associatedWin->getViewportArray(VP);
		CCVector3d A2D;
		ccGL::Project<PointCoordinateType, double>(*P3D, m_associatedWin->getModelViewMatd(), m_associatedWin->getProjectionMatd(), VP, A2D);

		CCVector3* firstP = const_cast<CCVector3*>(m_polyTipVertices->getPointPersistentPtr(0));
		*firstP = CCVector3(static_cast<PointCoordinateType>(A2D.x - m_associatedWin->width() / 2),
							static_cast<PointCoordinateType>(A2D.y - m_associatedWin->height() / 2),
							0);

	}

	m_polyTip->setEnabled(true);

	m_associatedWin->redraw(true, false);
}

void ccTracePolylineTool::handlePickedItem(int entityID, unsigned itemIdx, int x, int y)
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	ccHObject* object = MainWindow::TheInstance()->db()->find(entityID);
	if (!object || !object->isA(CC_TYPES::POINT_CLOUD))
	{
		//we ignore this object
		return;
	}

	//if the 3D polyline doesn't exist yet, we create it
	if (!m_poly3D || !m_poly3DVertices)
	{
		m_poly3DVertices = new ccPointCloud("Vertices");
		m_poly3DVertices->setEnabled(false);
		m_poly3DVertices->setDisplay(m_associatedWin);

		m_poly3D = new ccPolyline(m_poly3DVertices);
		m_poly3D->setTempColor(ccColor::green);
		m_poly3D->set2DMode(false);
		m_poly3D->addChild(m_poly3DVertices);
		m_poly3D->setWidth(widthSpinBox->value());

		m_associatedWin->addToOwnDB(m_poly3D);
	}

	//try to add one more point
	if (	!m_poly3DVertices->reserve(m_poly3DVertices->size()+1)
		||	!m_poly3D->reserve(m_poly3DVertices->size()+1) )
	{
		ccLog::Error("Not enough memory");
		return;
	}

	const CCVector3* P = ccHObjectCaster::ToPointCloud(object)->getPoint(itemIdx);

	m_poly3DVertices->addPoint(*P);
	m_poly3D->addPointIndex(m_poly3DVertices->size()-1);

    //we replace the first point of the tip by this new point
	{
		CCVector3 P2D(	static_cast<PointCoordinateType>(x - m_associatedWin->width() / 2),
						static_cast<PointCoordinateType>(m_associatedWin->height() / 2 - y),
						0);

		CCVector3* firstTipPoint = const_cast<CCVector3*>(m_polyTipVertices->getPointPersistentPtr(0));
		*firstTipPoint = P2D;
		m_polyTip->setEnabled(false); //don't need to display it for now
	}

	m_associatedWin->redraw(false, false);
}

void ccTracePolylineTool::closePolyLine(int, int)
{
	if (!m_poly3D)
	{
		return;
	}

	unsigned vertCount = m_poly3D->size();
    if (vertCount < 2)
	{
		//discard this polyline
		resetLine();
    }
    else
	{
        //hide the tip
		if (m_polyTip)
		{
			m_polyTip->setEnabled(false);
		}
		//update the GUI
		validButton->setEnabled(true);
		saveToolButton->setEnabled(true);
		resetToolButton->setEnabled(true);
		m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING); //no more picking
		m_done = true;

		if (m_associatedWin)
		{
			m_associatedWin->redraw(true, false);
		}
	}
}

void ccTracePolylineTool::resetLine()
{
	if (m_poly3D)
	{
		//discard this polyline
		if (m_associatedWin)
		{
			m_associatedWin->removeFromOwnDB(m_poly3D);
		}
		//hide the tip
		if (m_polyTip)
		{
			m_polyTip->setEnabled(false);
		}

		delete m_poly3D;
		//delete m_poly3DVertices;
		m_poly3D = 0;
		m_poly3DVertices = 0;
	}

	if (m_associatedWin)
	{
		//enable picking
		m_associatedWin->setPickingMode(ccGLWindow::POINT_PICKING);
        m_associatedWin->redraw(false, false);
	}
	validButton->setEnabled(false);
	saveToolButton->setEnabled(false);
	resetToolButton->setEnabled(false);
	m_done = false;
}

void ccTracePolylineTool::exportLine()
{
	if (!m_poly3D)
	{
		return;
    }

	if (m_associatedWin)
	{
		m_associatedWin->removeFromOwnDB(m_poly3D);
	}

	unsigned overSampling = static_cast<unsigned>(oversampleSpinBox->value());
	if (overSampling > 1)
	{
		ccPolyline* poly = PolylineOverSampling(m_poly3D, m_poly3DVertices, overSampling);
		if (poly)
		{
			delete m_poly3D;
			m_poly3DVertices = 0;
			m_poly3D = poly;
		}
	}

	m_poly3D->enableTempColor(false);
	MainWindow::TheInstance()->db()->addElement(m_poly3D, true);

	m_poly3D = 0;
	m_poly3DVertices = 0;

	resetLine(); //to update the GUI
}

void ccTracePolylineTool::apply()
{
	exportLine();

	stop(true);
}

void ccTracePolylineTool::cancel()
{
    resetLine();

	stop(false);
}

void ccTracePolylineTool::onSnapSizeChanged(int size)
{
	if (m_associatedWin)
	{
		m_associatedWin->setPickingRadius(size);
	}
}

void ccTracePolylineTool::onWidthSizeChanged(int width)
{
	if (m_poly3D)
	{
		m_poly3D->setWidth(width);
	}
	if (m_polyTip)
	{
		m_polyTip->setWidth(width);
	}
	
	if (m_associatedWin)
	{
		m_associatedWin->redraw(m_poly3D == 0, false);
	}
}