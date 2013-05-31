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

#include "ccClippingBoxTool.h"

#include "ccGLWindow.h"
#include "ccConsole.h"
#include "mainwindow.h"

//qCC_db
#include <ccHObject.h>
#include <ccClipBox.h>
#include <ccGenericPointCloud.h>

ccClippingBoxTool::ccClippingBoxTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::ClippingBoxDlg()
	, m_clipBox(0)
{
	setupUi(this);

	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(exportButton,	SIGNAL(clicked()), this, SLOT(exportCloud()));
	connect(resetButton,	SIGNAL(clicked()), this, SLOT(reset()));
	connect(closeButton,	SIGNAL(clicked()), this, SLOT(closeDialog()));

	connect(showInteractorsCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleInteractors(bool)));

	connect(thickXDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(thicknessChanged(double)));
	connect(thickYDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(thicknessChanged(double)));
	connect(thickZDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(thicknessChanged(double)));

	connect(minusXShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftXMinus()));
	connect(plusXShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftXPlus()));
	connect(minusYShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftYMinus()));
	connect(plusYShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftYPlus()));
	connect(minusZShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftZMinus()));
	connect(plusZShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftZPlus()));

	viewButtonsFrame->setEnabled(true);
    connect(viewUpToolButton,		SIGNAL(clicked()),	this,	SLOT(setTopView()));
    connect(viewDownToolButton,		SIGNAL(clicked()),	this,	SLOT(setBottomView()));
    connect(viewFrontToolButton,	SIGNAL(clicked()),	this,	SLOT(setFrontView()));
    connect(viewBackToolButton,		SIGNAL(clicked()),	this,	SLOT(setBackView()));
    connect(viewLeftToolButton,		SIGNAL(clicked()),	this,	SLOT(setLeftView()));
    connect(viewRightToolButton,	SIGNAL(clicked()),	this,	SLOT(setRightView()));
}

ccClippingBoxTool::~ccClippingBoxTool()
{
	if (m_clipBox)
		delete m_clipBox;
	m_clipBox = 0;
}

void ccClippingBoxTool::toggleInteractors(bool state)
{
	if (m_clipBox)
		m_clipBox->setSelected(state);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

bool ccClippingBoxTool::setAssociatedEntity(ccHObject* entity)
{
	if (!m_associatedWin || !m_clipBox)
	{
		ccConsole::Error(QString("[Clipping box] No associated 3D view or no valid clipping box!"));
		return false;
	}

	//we don't handle entities associated to another context
	if (entity->getDisplay() != m_associatedWin)
	{
		ccConsole::Warning(QString("[Clipping box] Can't use entity '%1' cause it's not displayed in the active 3D view!").arg(entity->getName()));
		return false;
	}

	//we can't handle other entities than clouds for the moment
	if (!entity->isA(CC_POINT_CLOUD))
	{
		ccConsole::Warning(QString("[Clipping box] Only points clouds are handled! Entity '%1' will be ignored.").arg(entity->getName()));
		return false;
	}

	//force visibility
	entity->setVisible(true);
	entity->setEnabled(true);

	m_clipBox->setAssociatedEntity(entity);
	if (m_associatedWin)
		m_associatedWin->redraw();

	return true;
}

bool ccClippingBoxTool::linkWith(ccGLWindow* win)
{
	if (m_associatedWin && m_clipBox)
	{
		//remove clipping box from precedent window
		m_associatedWin->removeFromOwnDB(m_clipBox);
		m_clipBox->disconnect(this);
		delete m_clipBox;
		m_clipBox = 0;
		m_associatedWin->redraw();
	}

	if (!ccOverlayDialog::linkWith(win))
		return false;

	if (win)
	{
		if (!m_clipBox)
		{
			m_clipBox = new ccClipBox();
			m_clipBox->setVisible(true);
			m_clipBox->setEnabled(true);
			m_clipBox->setSelected(true);
			connect(m_clipBox, SIGNAL(boxModified(const ccBBox*)), this, SLOT(onBoxModified(const ccBBox*)));
		}
		m_associatedWin->addToOwnDB(m_clipBox);
	}
	
	return true;
}

bool ccClippingBoxTool::start()
{
	assert(!m_processing);
	assert(m_associatedWin);
	if (!m_associatedWin || !m_clipBox)
		return false;

	m_clipBox->reset();

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	//m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	//m_associatedWin->displayNewMessage("[Rotation/Translation mode]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
	m_associatedWin->updateGL();

	return ccOverlayDialog::start();
}

void ccClippingBoxTool::stop(bool state)
{
	if (m_associatedWin)
	{
		m_associatedWin->setUnclosable(false);
		//m_associatedWin->displayNewMessage("[Rotation/Translation mode OFF]",ccGLWindow::UPPER_CENTER_MESSAGE,false,2,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->updateGL();
	}

	ccOverlayDialog::stop(state);
}

void ccClippingBoxTool::exportCloud()
{
	if (!m_clipBox)
		return;

	ccHObject* obj = m_clipBox->getAssociatedEntity();

	if (obj && obj->isKindOf(CC_POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(obj)->createNewCloudFromVisibilitySelection(false);
		MainWindow::TheInstance()->addToDB(cloud);
	}
}

void ccClippingBoxTool::onBoxModified(const ccBBox* box)
{
	if (box && box->isValid())
	{
		CCVector3 dd = box->maxCorner() - box->minCorner();
		thickXDoubleSpinBox->setValue(dd.x);
		thickYDoubleSpinBox->setValue(dd.y);
		thickZDoubleSpinBox->setValue(dd.z);
		thicknessGroupBox->setEnabled(true);
	}
	else
	{
		thicknessGroupBox->setEnabled(false);
	}
}

void ccClippingBoxTool::thicknessChanged(double)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	CCVector3 th(thickXDoubleSpinBox->value(),
				 thickYDoubleSpinBox->value(),
				 thickZDoubleSpinBox->value());

	ccBBox box = m_clipBox->getBox();
	CCVector3 boxCenter = (box.maxCorner() + box.minCorner())/2.0;

	box.minCorner() = boxCenter - th/2.0;
	box.maxCorner() = boxCenter + th/2.0;

	m_clipBox->setBox(box);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::shiftXMinus()
{
	shiftBox(0,true);
}

void ccClippingBoxTool::shiftXPlus()
{
	shiftBox(0,false);
}

void ccClippingBoxTool::shiftYMinus()
{
	shiftBox(1,true);
}

void ccClippingBoxTool::shiftYPlus()
{
	shiftBox(1,false);
}

void ccClippingBoxTool::shiftZMinus()
{
	shiftBox(2,true);
}

void ccClippingBoxTool::shiftZPlus()
{
	shiftBox(2,false);
}

void ccClippingBoxTool::shiftBox(unsigned char dim, bool minus)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	assert(dim<3);

	PointCoordinateType width = (m_clipBox->getBox().maxCorner() - m_clipBox->getBox().minCorner()).u[dim];
	CCVector3 shiftVec(0.0,0.0,0.0);
	shiftVec.u[dim] = (minus ? -width : width);
	m_clipBox->shift(shiftVec);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::reset()
{
	if (m_clipBox)
		m_clipBox->reset();

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::closeDialog()
{
	stop(true);
}

void ccClippingBoxTool::setTopView()
{
    setView(CC_TOP_VIEW);
}

void ccClippingBoxTool::setBottomView()
{
    setView(CC_BOTTOM_VIEW);
}

void ccClippingBoxTool::setFrontView()
{
    setView(CC_FRONT_VIEW);
}

void ccClippingBoxTool::setBackView()
{
    setView(CC_BACK_VIEW);
}

void ccClippingBoxTool::setLeftView()
{
    setView(CC_LEFT_VIEW);
}

void ccClippingBoxTool::setRightView()
{
    setView(CC_RIGHT_VIEW);
}

void ccClippingBoxTool::setView(CC_VIEW_ORIENTATION orientation)
{
    if (!m_associatedWin)
        return;

    //m_associatedWin->blockSignals(true);
    m_associatedWin->setView(orientation,false);
	if (m_clipBox && m_clipBox->isGLTransEnabled())
	{
		ccViewportParameters params = m_associatedWin->getViewportParameters();
		const ccGLMatrix& glMat = m_clipBox->getGLTransformation();

		ccGLMatrix rotMat = glMat; rotMat.setTranslation(CCVector3(0.0,0.0,0.0));

		//CCVector3 T = CCVector3(glMat.getTranslation()) - params.pivotPoint;
		//rotMat.inverse().apply(T);
		//T += params.pivotPoint;
		//params.viewMat.apply(T);
		//params.cameraCenter -= T;

		params.viewMat = params.viewMat * rotMat.inverse();
		m_associatedWin->setViewportParameters(params);

		//ccLog::Print(QString("X(%1,%2,%3)").arg(glMat.getColumn(0)[0]).arg(glMat.getColumn(0)[1]).arg(glMat.getColumn(0)[2]));
		//ccLog::Print(QString("Y(%1,%2,%3)").arg(glMat.getColumn(1)[0]).arg(glMat.getColumn(1)[1]).arg(glMat.getColumn(1)[2]));
		//ccLog::Print(QString("Z(%1,%2,%3)").arg(glMat.getColumn(2)[0]).arg(glMat.getColumn(2)[1]).arg(glMat.getColumn(2)[2]));
	}
    //m_associatedWin->blockSignals(false);
    m_associatedWin->redraw();
}
