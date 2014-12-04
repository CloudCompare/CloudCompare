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

#include "ccSectionExtractionTool.h"

//Local
#include "mainwindow.h"
#include "ccEntityPickerDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QMessageBox>
#include <QMdiSubWindow>

//System
#include <assert.h>

ccSectionExtractionTool::ccSectionExtractionTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::SectionExtractionDlg()
	, m_state(0)
	, m_editedPoly(0)
	, m_editedPolyVertices(0)
{
	setupUi(this);
	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(resetToolButton,					SIGNAL(clicked()),					this,	SLOT(reset()));
	connect(validToolButton,					SIGNAL(clicked()),					this,	SLOT(apply()));
	connect(polylineToolButton,					SIGNAL(toggled(bool)),				this,	SLOT(enableSectionEditingMode(bool)));
	connect(importFromDBToolButton,				SIGNAL(clicked()),					this,	SLOT(doImportPolylinesFromDB()));
	connect(vertAxisComboBox,					SIGNAL(currentIndexChanged(int)),	this,	SLOT(setVertDimension(int)));

	//add shortcuts
	addOverridenShortcut(Qt::Key_Space);  //space bar for the "pause" button
	addOverridenShortcut(Qt::Key_Escape); //cancel current polyline edition

	//addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button
	//addOverridenShortcut(Qt::Key_Delete); //delete key for the "apply and delete" button
	//addOverridenShortcut(Qt::Key_Tab);    //tab key to switch between rectangular and polygonal selection modes
	//addOverridenShortcut(Qt::Key_I);      //'I' key for the "segment in" button
	//addOverridenShortcut(Qt::Key_O);      //'O' key for the "segment out" button
	connect(this, SIGNAL(shortcutTriggered(int)), this, SLOT(onShortcutTriggered(int)));
}

ccSectionExtractionTool::~ccSectionExtractionTool()
{
	if (m_editedPoly)
	{
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		delete m_editedPoly;
		m_editedPoly = 0;
	}
}

void ccSectionExtractionTool::setVertDimension(int dim)
{
	assert(dim >=0 && dim < 3);
	if (!m_associatedWin)
		return;

	switch(dim)
	{
	case 0:
		m_associatedWin->setView(CC_LEFT_VIEW);
		break;
	case 1:
		m_associatedWin->setView(CC_BOTTOM_VIEW);
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
 	switch(key)
	{
	case Qt::Key_Space:
		polylineToolButton->toggle();
		return;

	case Qt::Key_Escape:
		cancelCurrentPolyline();
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
		return false;

	if (oldWin)
	{
		disconnect(m_associatedWin, SIGNAL(leftButtonClicked(int,int)), this, SLOT(addPointToPolyline(int,int)));
		disconnect(m_associatedWin, SIGNAL(rightButtonClicked(int,int)), this, SLOT(closePolyLine(int,int)));
		disconnect(m_associatedWin, SIGNAL(mouseMoved(int,int,Qt::MouseButtons)), this, SLOT(updatePolyLine(int,int,Qt::MouseButtons)));
		disconnect(m_associatedWin, SIGNAL(buttonReleased()), this, SLOT(closeRectangle()));

		//restore sections original display
		{
			for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
			{
				Section& section = *it;
				if (section.entity)
				{
					if (!section.isInDB)
						oldWin->removeFromOwnDB(section.entity);
					section.entity->setDisplay(section.originalDisplay);
				}
			}
		}
		//Restore clouds original display
		{
			for (CloudPool::iterator it = m_clouds.begin(); it != m_clouds.end(); ++it)
			{
				Cloud& cloud = *it;
				if (cloud.entity)
				{
					if (!cloud.isInDB)
						oldWin->removeFromOwnDB(cloud.entity);
					cloud.entity->setDisplay(cloud.originalDisplay);
				}
			}
		}

		if (m_editedPoly)
			m_editedPoly->setDisplay(0);

		//update view direction
		setVertDimension(vertAxisComboBox->currentIndex());

		//auto-close formerly associated window
		if (MainWindow::TheInstance())
		{
			QMdiSubWindow* subWindow = MainWindow::TheInstance()->getMDISubWindow(oldWin);
			if (subWindow)
				subWindow->close();
		}
	}
	
	if (m_associatedWin)
	{
		connect(m_associatedWin, SIGNAL(leftButtonClicked(int,int)), this, SLOT(addPointToPolyline(int,int)));
		connect(m_associatedWin, SIGNAL(rightButtonClicked(int,int)), this, SLOT(closePolyLine(int,int)));
		connect(m_associatedWin, SIGNAL(mouseMoved(int,int,Qt::MouseButtons)), this, SLOT(updatePolyLine(int,int,Qt::MouseButtons)));
		connect(m_associatedWin, SIGNAL(buttonReleased()), this, SLOT(closeRectangle()));

		//import sections in current display
		{
			for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
			{
				Section& section = *it;
				if (section.entity)
				{
					section.originalDisplay = section.entity->getDisplay();
					section.entity->setDisplay(m_associatedWin);
					if (!section.isInDB)
						m_associatedWin->addToOwnDB(section.entity);
				}
			}
		}
		//import clouds in current display
		{
			for (CloudPool::iterator it = m_clouds.begin(); it != m_clouds.end(); ++it)
			{
				Cloud& cloud = *it;
				if (cloud.entity)
				{
					cloud.originalDisplay = cloud.entity->getDisplay();
					cloud.entity->setDisplay(m_associatedWin);
					if (!cloud.isInDB)
						m_associatedWin->addToOwnDB(cloud.entity);
				}
			}
		}

		if (m_editedPoly)
			m_editedPoly->setDisplay(m_associatedWin);
	}

	return true;
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
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING/*ENTITY_PICKING*/); //DGM: we should be able to pick polyine?
	m_associatedWin->updateConstellationCenterAndZoom();
	
	enableSectionEditingMode(true);

	return ccOverlayDialog::start();
}

void ccSectionExtractionTool::removeAllEntities()
{
	reset(false);

	//and we remove the remaining clouds (if any)
	{
		for (int i=0; i < m_clouds.size(); ++i)
		{
			Cloud& cloud = m_clouds[i];
			if (cloud.entity)
			{
				assert(cloud.isInDB);
				//restore original display
				cloud.entity->setDisplay(cloud.originalDisplay);
			}
		}
		m_clouds.clear();
	}
}

void ccSectionExtractionTool::reset(bool askForConfirmation/*=true*/)
{
	if (m_sections.empty() && m_clouds.empty())
	{
		//nothing to do
		return;
	}

	if (askForConfirmation)
	{
		if (QMessageBox::question(MainWindow::TheInstance(), "Reset", "You'll lose all manually defined polylines: are you sure?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return;
	}

	//we remove all polylines
	{
		for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
		{
			Section& section = *it;
			//restore original display
			if (section.entity)
			{
				if (!section.isInDB)
				{
					if (m_associatedWin)
						m_associatedWin->removeFromOwnDB(section.entity);
					delete section.entity;
					section.entity = 0;
				}
				else
				{
					section.entity->setDisplay(section.originalDisplay);
				}
			}
		}
		m_sections.clear();
	}

	//and we remove only temporary clouds
	{
		for (int i=0; i < m_clouds.size(); )
		{
			Cloud& cloud = m_clouds[i];
			//restore original display
			if (cloud.entity && !cloud.isInDB)
			{
				if (m_associatedWin)
					m_associatedWin->removeFromOwnDB(cloud.entity);
				delete cloud.entity;
				cloud.entity = 0;

				m_sections.removeAt(i);
			}
			else
			{
				++i;
			}
		}
	}

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccSectionExtractionTool::stop(bool accepted)
{
	if (m_editedPoly)
	{
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		delete m_editedPoly;
		m_editedPoly = 0;
	}

	enableSectionEditingMode(false);
	reset();

	if (m_associatedWin)
	{
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA);
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setUnclosable(false);
	}

	ccOverlayDialog::stop(accepted);
}

bool ccSectionExtractionTool::addPolyline(ccPolyline* inputPoly, bool alreadyInDB/*=true*/)
{
	if (!inputPoly)
	{
		assert(false);
		return false;
	}

	for (SectionPool::iterator it = m_sections.begin(); it != m_sections.end(); ++it)
	{
		Section& section = *it;
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
		const double* MM = m_associatedWin->getModelViewMatd(); //viewMat
		const double* MP = m_associatedWin->getProjectionMatd(); //projMat
		const GLdouble half_w = static_cast<GLdouble>(m_associatedWin->width())/2;
		const GLdouble half_h = static_cast<GLdouble>(m_associatedWin->height())/2;
		int VP[4];
		m_associatedWin->getViewportArray(VP);

		ccPointCloud* duplicateVertices = new ccPointCloud();
		ccPolyline* duplicatePoly = new ccPolyline(duplicateVertices);
		duplicatePoly->addChild(duplicateVertices);

		//duplicate polyline 'a minima' (only points and indexes + closed state)
		CCLib::GenericIndexedCloudPersist* vertices = inputPoly->getAssociatedCloud();
		if (	duplicateVertices->reserve(vertices->size())
			&&	duplicatePoly->reserve(inputPoly->size()))
		{
			for (unsigned i=0; i<vertices->size(); ++i)
			{
				CCVector3 P = *vertices->getPoint(i);
				GLdouble xp,yp,zp;
				gluUnProject(half_w+P.x,half_h+P.y,0/*P.z*/,MM,MP,VP,&xp,&yp,&zp);
				P.x = static_cast<PointCoordinateType>(xp);
				P.y = static_cast<PointCoordinateType>(yp);
				P.z = static_cast<PointCoordinateType>(zp);

				duplicateVertices->addPoint(P);
			}
			for (unsigned j=0; j<inputPoly->size(); ++j)
				duplicatePoly->addPointIndex(inputPoly->getPointGlobalIndex(j));

			duplicatePoly->setClosed(inputPoly->isClosed());
			duplicatePoly->set2DMode(false);
			duplicatePoly->setColor(inputPoly->getColor());
			duplicatePoly->showColors(inputPoly->colorsShown());
			duplicatePoly->setClosed(inputPoly->isClosed());

			if (!alreadyInDB)
				delete inputPoly;
			else
				alreadyInDB = false;
			inputPoly = duplicatePoly;
			vertices = 0;
		}
		else
		{
			delete duplicatePoly;
			duplicatePoly = 0;
			
			ccLog::Error("Not enough memory to import polyline!");
			return false;
		}
	}
	
	m_sections.push_back(Section(inputPoly,alreadyInDB));
	if (m_associatedWin)
	{
		inputPoly->setDisplay(m_associatedWin);
		if (!alreadyInDB)
			m_associatedWin->addToOwnDB(inputPoly);
	}

	return true;
}

bool ccSectionExtractionTool::addCloud(ccGenericPointCloud* inputCloud, bool alreadyInDB/*=true*/)
{
	assert(inputCloud);

	for (CloudPool::iterator it = m_clouds.begin(); it != m_clouds.end(); ++it)
	{
		Cloud& cloud = *it;
		if (cloud.entity == inputCloud)
		{
			//cloud already in DB
			return false;
		}
	}

	m_clouds.push_back(Cloud(inputCloud,alreadyInDB));
	if (m_associatedWin)
	{
		inputCloud->setDisplay(m_associatedWin);
		if (!alreadyInDB)
			m_associatedWin->addToOwnDB(inputCloud);
	}

	return true;
}

//CCVector3 ccSectionExtractionTool::project2Dto3D(int x, int y) const
//{
//	//get current display parameters
//	const double* MM = m_associatedWin->getModelViewMatd(); //viewMat
//	const double* MP = m_associatedWin->getProjectionMatd(); //projMat
//	const GLdouble half_w = static_cast<GLdouble>(m_associatedWin->width())/2;
//	const GLdouble half_h = static_cast<GLdouble>(m_associatedWin->height())/2;
//	int VP[4];
//	m_associatedWin->getViewportArray(VP);
//
//	GLdouble xp,yp,zp;
//	gluUnProject(half_w+x,half_h+y,0/*z*/,MM,MP,VP,&xp,&yp,&zp);
//
//	return CCVector3(	static_cast<PointCoordinateType>(xp),
//						static_cast<PointCoordinateType>(yp),
//						static_cast<PointCoordinateType>(zp) );
//}

void ccSectionExtractionTool::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	if (!m_associatedWin)
		return;

	//process not started yet?
	if ((m_state & RUNNING) == 0)
		return;

	if (!m_editedPoly)
		return;

	unsigned vertCount = m_editedPolyVertices->size();
	if (vertCount < 2)
		return;
	
	CCVector3 P = CCVector3(static_cast<PointCoordinateType>(x),
							static_cast<PointCoordinateType>(y),
							0);

	//we replace last point by the current one
	CCVector3* lastP = const_cast<CCVector3*>(m_editedPolyVertices->getPointPersistentPtr(vertCount-1));
	*lastP = P;

	if (m_associatedWin)
		m_associatedWin->updateGL();
}

void ccSectionExtractionTool::addPointToPolyline(int x, int y)
{
	if ((m_state & STARTED) == 0)
		return;

	if (!m_editedPoly)
	{
		assert(!m_editedPolyVertices);
		m_editedPolyVertices = new ccPointCloud("vertices");
		m_editedPoly = new ccPolyline(m_editedPolyVertices);
		m_editedPoly->setForeground(true);
		m_editedPoly->setColor(ccColor::green);
		m_editedPoly->showColors(true);
		m_editedPoly->set2DMode(true);
		m_editedPoly->addChild(m_editedPolyVertices);
		if (m_associatedWin)
			m_associatedWin->addToOwnDB(m_editedPoly);
	}

	unsigned vertCount = m_editedPolyVertices->size();

	//clicked point (2D)
	CCVector3 P = CCVector3(static_cast<PointCoordinateType>(x),
							static_cast<PointCoordinateType>(y),
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
		if (!m_editedPoly->addPointIndex(0,2))
		{
			ccLog::Error("Out of memory!");
			return;
		}
	}
	else //next points
	{
		if (!m_editedPolyVertices->reserve(vertCount+1))
		{
			ccLog::Error("Out of memory!");
			return;
		}

		//we replace last point by the current one
		CCVector3* lastP = const_cast<CCVector3*>(m_editedPolyVertices->getPointPersistentPtr(vertCount-1));
		*lastP = P;
		//and add a new (equivalent) one
		m_editedPolyVertices->addPoint(P);
		if (!m_editedPoly->addPointIndex(vertCount))
		{
			ccLog::Error("Out of memory!");
			return;
		}
	}

	if (m_associatedWin)
		m_associatedWin->updateGL();
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
		m_editedPoly->resize(vertCount-1); //can't fail --> smaller

		//remove polyline from the 'temporary' world
		if (m_associatedWin)
			m_associatedWin->removeFromOwnDB(m_editedPoly);
		//save polyline
		m_editedPoly->setColor(ccColor::yellow);
		if (!addPolyline(m_editedPoly,false))
		{
			//if something went wrong, we have to remove the polyline manually
			delete m_editedPoly;
		}
		m_editedPoly = 0;
		m_editedPolyVertices = 0;
	}

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
		m_associatedWin->updateGL();
}

void ccSectionExtractionTool::cancelCurrentPolyline()
{
	if (	(m_state & STARTED) == 0
		||	!m_editedPoly)
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
		m_state = PAUSED;
		
		if (m_editedPoly && m_editedPolyVertices)
		{
			m_editedPoly->clear();
			m_editedPolyVertices->clear();
		}
		m_associatedWin->setInteractionMode(ccGLWindow::PAN_ONLY);
		m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE,false,0,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	}
	else
	{
		m_state = STARTED;
		
		m_associatedWin->setInteractionMode(ccGLWindow::SEGMENT_ENTITY);
		m_associatedWin->displayNewMessage("Section edition mode",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
		m_associatedWin->displayNewMessage("Left click: add section points / Right click: stop",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600,ccGLWindow::MANUAL_SEGMENTATION_MESSAGE);
	}

	//update mini-GUI
	polylineToolButton->blockSignals(true);
	polylineToolButton->setChecked(state);
	polylineToolButton->blockSignals(false);

	hydroAxisToolButton->setEnabled(!state);
	extractPointsToolButton->setEnabled(!state);

	m_associatedWin->redraw();
}

void ccSectionExtractionTool::doImportPolylinesFromDB()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		ccHObject* root = mainWindow->dbRootObject();
		ccHObject::Container polylines;
		if (root)
		{
			root->filterChildren(polylines,true,CC_TYPES::POLY_LINE);
		}

		if (!polylines.empty())
		{
			ccEntityPickerDlg epDlg(polylines,0,this);
			if (!epDlg.exec())
				return;

			enableSectionEditingMode(false);

			int index = epDlg.getSelectedIndex();
			assert(index >= 0 && index < static_cast<int>(polylines.size()));
			assert(polylines[index]->isA(CC_TYPES::POLY_LINE));
			ccPolyline* poly = static_cast<ccPolyline*>(polylines[index]);
			addPolyline(poly,true);
			if (m_associatedWin)
				m_associatedWin->redraw();
		}
		else
		{
			ccLog::Error("No polyline in DB!");
		}
	}
}

void ccSectionExtractionTool::apply()
{
	reset(false);
	stop(true);
}
