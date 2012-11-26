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
//$Rev:: 2187                                                              $
//$LastChangedDate:: 2012-07-03 15:22:55 +0200 (mar., 03 juil. 2012)       $
//**************************************************************************
//

#include "ccPointPropertiesDlg.h"

//Local
#include "ccCommon.h"
#include "ccGLWindow.h"
#include "ccConsole.h"
#include "ccGuiParameters.h"

//qCC_db
#include <ccPointCloud.h>
#include <cc2DViewportLabel.h>
#include <cc2DLabel.h>

//CCLib
#include <ScalarField.h>

//Qt
#include <QInputDialog>

//System
#include <assert.h>

ccPointPropertiesDlg::ccPointPropertiesDlg(QWidget* parent)
	: ccPointPickingGenericInterface(parent)
	, Ui::PointPropertiesDlg()
    , m_pickingMode(POINT_INFO)
{
    setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint |Qt::Tool);

    connect(cancelButton,				SIGNAL(clicked()), this, SLOT(cancel()));
    connect(pointPropertiesButton,		SIGNAL(clicked()), this, SLOT(activatePointPropertiesDisplay()));
    connect(pointPointDistanceButton,	SIGNAL(clicked()), this, SLOT(activateDistanceDisplay()));
    connect(pointsAngleButton,			SIGNAL(clicked()), this, SLOT(activateAngleDisplay()));
    connect(rectZoneToolButton,			SIGNAL(clicked()), this, SLOT(activate2DZonePicking()));
	connect(saveLabelButton,			SIGNAL(clicked()), this, SLOT(exportCurrentLabel()));
    connect(razButton,					SIGNAL(clicked()), this, SLOT(initializeState()));

	//for points picking
	m_label = new cc2DLabel();
	m_label->setSelected(true);

	//for 2D zone picking
	m_rect2DLabel = new cc2DViewportLabel();
	m_rect2DLabel->setVisible(false);	//=invalid
	m_rect2DLabel->setSelected(true);	//=closed
}

ccPointPropertiesDlg::~ccPointPropertiesDlg()
{
	if (m_label)
		delete m_label;

	if (m_rect2DLabel)
        delete m_rect2DLabel;
    m_rect2DLabel=0;
}

void ccPointPropertiesDlg::linkWith(ccGLWindow* win)
{
	assert(m_label && m_rect2DLabel);

	if (m_win)
	{
		m_win->removeFromOwnDB(m_label);
		m_win->removeFromOwnDB(m_rect2DLabel);
		m_win->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
		m_win->disconnect(this);
	}

	ccPointPickingGenericInterface::linkWith(win);

    m_rect2DLabel->setVisible(false);	//=invalid
	m_rect2DLabel->setSelected(true);	//=closed
	m_label->clear();

	if (m_win)
	{
		m_win->addToOwnDB(m_label);
		m_win->addToOwnDB(m_rect2DLabel);
		connect(m_win, SIGNAL(mouseMoved(int,int,Qt::MouseButtons)), this, SLOT(update2DZone(int,int,Qt::MouseButtons)));
		connect(m_win, SIGNAL(leftButtonClicked(int,int)), this, SLOT(processClickedPoint(int,int)));
		connect(m_win, SIGNAL(buttonReleased()), this, SLOT(close2DZone()));
	}
}

bool ccPointPropertiesDlg::start()
{
#ifndef CC_OPENGL_POINT_PICKING
    for (unsigned i=0; i<clouds.size(); i++)
    {
        clouds[i]->showColors(false);
        clouds[i]->setSelected(false);
    }
#endif

    activatePointPropertiesDisplay();

    return ccPointPickingGenericInterface::start();
}

void ccPointPropertiesDlg::stop(bool state)
{
	assert(m_label && m_rect2DLabel);
	m_label->clear();
	m_rect2DLabel->setVisible(false);	//=invalid
	m_rect2DLabel->setSelected(true);	//=closed
	if (m_win)
		m_win->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
    ccPointPickingGenericInterface::stop(state);
}

void ccPointPropertiesDlg::cancel()
{
    stop(false);
}

void ccPointPropertiesDlg::activatePointPropertiesDisplay()
{
	if (m_win)
		m_win->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
    m_pickingMode = POINT_INFO;
    pointPropertiesButton->setDown(true);
    pointPointDistanceButton->setDown(false);
    pointsAngleButton->setDown(false);
	rectZoneToolButton->setDown(false);
	m_label->setVisible(true);
	m_rect2DLabel->setVisible(false);
}

void ccPointPropertiesDlg::activateDistanceDisplay()
{
    m_pickingMode = POINT_POINT_DISTANCE;
    pointPropertiesButton->setDown(false);
    pointPointDistanceButton->setDown(true);
    pointsAngleButton->setDown(false);
	rectZoneToolButton->setDown(false);
	m_label->setVisible(true);
	m_rect2DLabel->setVisible(false);
	if (m_win)
	{
		m_win->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
		m_win->updateGL();
	}
}

void ccPointPropertiesDlg::activateAngleDisplay()
{
    m_pickingMode = POINTS_ANGLE;
    pointPropertiesButton->setDown(false);
    pointPointDistanceButton->setDown(false);
    pointsAngleButton->setDown(true);
	rectZoneToolButton->setDown(false);
	m_label->setVisible(true);
	m_rect2DLabel->setVisible(false);
	if (m_win)
	{
		m_win->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
		m_win->updateGL();
	}
}

void ccPointPropertiesDlg::activate2DZonePicking()
{
    m_pickingMode = RECT_ZONE;
    pointPropertiesButton->setDown(false);
    pointPointDistanceButton->setDown(false);
    pointsAngleButton->setDown(false);
	rectZoneToolButton->setDown(true);
	m_label->setVisible(false);
	//m_rect2DLabel->setVisible(false);
	if (m_win)
	{
		m_win->setInteractionMode(ccGLWindow::SEGMENT_ENTITY);
		m_win->updateGL();
	}
}

void ccPointPropertiesDlg::initializeState()
{
	assert(m_label && m_rect2DLabel);
	m_label->clear();
	m_rect2DLabel->setVisible(false);	//=invalid
	m_rect2DLabel->setSelected(true);	//=closed
    if (m_win)
        m_win->redraw();
}

void ccPointPropertiesDlg::exportCurrentLabel()
{
	ccHObject* labelObject = 0;
	if (m_pickingMode == RECT_ZONE)
		labelObject = (m_rect2DLabel->isSelected() && m_rect2DLabel->isVisible() ? m_rect2DLabel : 0);
	else
		labelObject = (m_label && m_label->size()>0 ? m_label : 0);
	if (!labelObject)
		return;

	//detach current label from window
	if (m_win)
		m_win->removeFromOwnDB(labelObject);
	labelObject->setSelected(false);

	ccHObject* newLabelObject = 0;
	if (m_pickingMode == RECT_ZONE)
	{
		//if (m_win)
		//	m_rect2DLabel->setParameters(m_win->getViewportParameters());
		newLabelObject = m_rect2DLabel = new cc2DViewportLabel();
		m_rect2DLabel->setVisible(false);	//=invalid
		m_rect2DLabel->setSelected(true);	//=closed
	}
	else
	{
		//attach old label to first point cloud by default
		m_label->getPoint(0).cloud->addChild(labelObject);
		newLabelObject = m_label = new cc2DLabel();
		m_label->setSelected(true);
	}

	emit newLabel(labelObject);

	if (m_win)
	{
		m_win->addToOwnDB(newLabelObject);
		m_win->redraw();
	}	
}

void ccPointPropertiesDlg::processPickedPoint(ccPointCloud* cloud, unsigned pointIndex, int x, int y)
{
    assert(cloud);
	assert(m_label);

	switch(m_pickingMode)
    {
	case POINT_INFO:
		m_label->clear();
        break;
    case POINT_POINT_DISTANCE:
        if (m_label->size() >= 2)
			m_label->clear();
        break;
    case POINTS_ANGLE:
        if (m_label->size() >= 3)
			m_label->clear();
        break;
	case RECT_ZONE:
		return; //we don't use this slot for 2D mode
    }

	m_label->addPoint(cloud,pointIndex);
	m_label->setVisible(true);
	if (m_label->size()==1)
		m_label->setPosition((float)(x+20)/(float)m_win->width(),(float)(y+20)/(float)m_win->height());

	//output info to Console
	QStringList body = m_label->getLabelContent(ccGui::Parameters().displayedNumPrecision);
	ccConsole::Print(QString("[Picked] ")+m_label->getName());
	for (int i=0;i<body.size();++i)
		ccConsole::Print(QString("[Picked]\t- ")+body[i]);

    m_win->redraw();
}

void ccPointPropertiesDlg::processClickedPoint(int x, int y)
{
	if (m_pickingMode != RECT_ZONE)
		return;

	assert(m_rect2DLabel);
	assert(m_win);

	if (m_rect2DLabel->isSelected()) //already closed? we start a new label
	{
		float roi[4]={(float)x,(float)y,0,0};
		m_rect2DLabel->setParameters(m_win->getViewportParameters());
		m_rect2DLabel->setVisible(false);	//=invalid
		m_rect2DLabel->setSelected(false);	//=not closed
		m_rect2DLabel->setRoi(roi);
		m_rect2DLabel->setName(""); //reset name before display!
	}
	else //we close the existing one
	{
		float roi[4]={m_rect2DLabel->roi()[0],
						m_rect2DLabel->roi()[1],
						(float)x,
						(float)y};
		m_rect2DLabel->setRoi(roi);
		m_rect2DLabel->setVisible(true);  //=valid
		m_rect2DLabel->setSelected(true); //=closed
	}

    m_win->updateGL();
}

void ccPointPropertiesDlg::update2DZone(int x, int y, Qt::MouseButtons buttons)
{
	if (m_pickingMode != RECT_ZONE)
		return;

	if (m_rect2DLabel->isSelected())
		return;

	float roi[4]={m_rect2DLabel->roi()[0],
					m_rect2DLabel->roi()[1],
					(float)x,
					(float)y};
	m_rect2DLabel->setRoi(roi);
	m_rect2DLabel->setVisible(true);

	m_win->updateGL();
}

static QString s_last2DLabelComment("");
void ccPointPropertiesDlg::close2DZone()
{
	if (m_pickingMode != RECT_ZONE)
		return;

	if (m_rect2DLabel->isSelected() || !m_rect2DLabel->isVisible())
		return;

	m_rect2DLabel->setSelected(true);

	bool ok;
	QString title = QInputDialog::getText(this, "Set area label title", "Title:", QLineEdit::Normal, s_last2DLabelComment, &ok);
	if (!ok)
	{
		m_rect2DLabel->setVisible(false);
	}
	else
	{
		m_rect2DLabel->setName(qPrintable(title));
		s_last2DLabelComment = title;
	}

	m_win->updateGL();
}
