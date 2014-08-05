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

#include <ccPointPairRegistrationDlg.h>

//Local
#include <mainwindow.h>
#include <ccGLWindow.h>
#include <ccAskThreeDoubleValuesDlg.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <cc2DLabel.h>
#include <ccPointCloud.h>

//qCC_io
#include <ccGlobalShiftManager.h>

//CC_FBO
#include <ccGlFilter.h>

//CCLib
#include <RegistrationTools.h>

//Qt
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QToolButton>

//default position of each columns in the aligned and ref. table widgets
static const int XYZ_COL_INDEX			= 0;
static const int RMS_COL_INDEX			= 3;
static const int DEL_BUTTON_COL_INDEX	= 4;

ccPointPairRegistrationDlg::ccPointPairRegistrationDlg(QWidget* parent/*=0*/)
	: ccOverlayDialog(parent)
	, m_aligned(0)
	, m_alignedPoints("aligned points")
	, m_reference(0)
	, m_refPoints("reference points")
	, m_paused(false)
{
	setupUi(this);
	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(showAlignedCheckBox,	SIGNAL(toggled(bool)),	this,	SLOT(showAlignedCloud(bool)));
	connect(showReferenceCheckBox,	SIGNAL(toggled(bool)),	this,	SLOT(showReferenceCloud(bool)));

	connect(typeAlignToolButton,	SIGNAL(clicked()),		this,	SLOT(addManualAlignedPoint()));
	connect(typeRefToolButton,		SIGNAL(clicked()),		this,	SLOT(addManualRefPoint()));

	connect(unstackAlignToolButton,	SIGNAL(clicked()),		this,	SLOT(unstackAligned()));
	connect(unstackRefToolButton,	SIGNAL(clicked()),		this,	SLOT(unstackRef()));

	connect(alignToolButton,		SIGNAL(clicked()),		this,	SLOT(align()));
	connect(resetToolButton,		SIGNAL(clicked()),		this,	SLOT(reset()));

	connect(validToolButton,		SIGNAL(clicked()),		this,	SLOT(apply()));
	connect(cancelToolButton,		SIGNAL(clicked()),		this,	SLOT(cancel()));

	m_alignedPoints.setEnabled(true);
	m_alignedPoints.setVisible(false);

	m_refPoints.setEnabled(true);
	m_refPoints.setVisible(false);
}

ccPointPairRegistrationDlg::cloudContext::cloudContext(ccGenericPointCloud* entity)
	: cloud(entity)
	, originalDisplay(entity ? entity->getDisplay() : 0)
	, wasVisible(entity ? entity->isVisible() : false)
	, wasEnabled(entity ? entity->isEnabled() : false)
	, wasSelected(entity ? entity->isSelected() : false)
{
}

void ccPointPairRegistrationDlg::cloudContext::restore()
{
	if (!cloud)
		return;

	cloud->setDisplay(originalDisplay);
	if (originalDisplay)
		originalDisplay->redraw();
	cloud->setVisible(wasVisible);
	cloud->setEnabled(wasEnabled);
	cloud->setSelected(wasSelected);
}

void ccPointPairRegistrationDlg::clear()
{
	alignToolButton->setEnabled(false);
	validToolButton->setEnabled(false);

	while (alignedPointsTableWidget->rowCount() != 0)
		alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount()-1);
	while (refPointsTableWidget->rowCount() != 0)
		refPointsTableWidget->removeRow(refPointsTableWidget->rowCount()-1);

	m_alignedPoints.removeAllChildren();
	m_alignedPoints.clear();
	m_alignedPoints.setGlobalShift(0,0,0);
	m_alignedPoints.setGlobalScale(1.0);
	m_aligned.cloud = 0;
	m_refPoints.removeAllChildren();
	m_refPoints.clear();
	m_refPoints.setGlobalShift(0,0,0);
	m_refPoints.setGlobalScale(1.0);
	m_reference.cloud = 0;
}

bool ccPointPairRegistrationDlg::linkWith(ccGLWindow* win)
{
	ccGLWindow* oldWin = m_associatedWin;
	if (oldWin)
	{
		if (oldWin != win)
			disconnect(oldWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));

		oldWin->removeFromOwnDB(&m_alignedPoints);
		m_alignedPoints.setDisplay(0);
		oldWin->removeFromOwnDB(&m_refPoints);
		m_refPoints.setDisplay(0);

		oldWin->lockPickingMode(false);
		oldWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
	}

	if (!ccOverlayDialog::linkWith(win))
		return false;

	m_aligned.restore();
	m_reference.restore();

	if (oldWin && MainWindow::TheInstance())
	{
		QMdiSubWindow* subWindow = MainWindow::TheInstance()->getMDISubWindow(oldWin);
		if (subWindow)
			subWindow->close();
	}

	if (m_associatedWin)
	{
		m_associatedWin->setPickingMode(ccGLWindow::POINT_PICKING);
		m_associatedWin->lockPickingMode(true);
		connect(m_associatedWin, SIGNAL(pointPicked(int, unsigned, int, int)), this, SLOT(processPickedPoint(int, unsigned, int, int)));

		m_associatedWin->addToOwnDB(&m_alignedPoints);
		m_associatedWin->addToOwnDB(&m_refPoints);

		m_associatedWin->displayNewMessage(QString(),ccGLWindow::LOWER_LEFT_MESSAGE);
		m_associatedWin->displayNewMessage("(you can add points 'manually' if necessary)",ccGLWindow::LOWER_LEFT_MESSAGE,true,3600);
		m_associatedWin->displayNewMessage("Pick equivalent points on both clouds (at least 4 pairs - mind the order)",ccGLWindow::LOWER_LEFT_MESSAGE,true,3600);
	}

	return true;
}

bool ccPointPairRegistrationDlg::start()
{
	assert(m_aligned.cloud);
	return ccOverlayDialog::start();
}

void ccPointPairRegistrationDlg::stop(bool state)
{
	reset();
	ccOverlayDialog::stop(state);
}

static void SetEnabled_recursive(ccHObject* ent)
{
	assert(ent);
	ent->setEnabled(true);
	if (ent->getParent())
		SetEnabled_recursive(ent->getParent());
}

//static void SetVisible_recursive(ccHObject* ent)
//{
//	assert(ent);
//	ent->setVisible(true);
//	if (ent->getParent())
//		SetVisible_recursive(ent->getParent());
//}

bool ccPointPairRegistrationDlg::init(	ccGLWindow* win,
										ccGenericPointCloud* aligned,
										ccGenericPointCloud* reference/*=0*/)
{
	assert(win);
	assert(aligned);
	
	clear();

	if (!aligned)
	{
		ccLog::Error("[PointPairRegistration] Need an aligned cloud at least!");
		return false;
	}

	//create dedicated 3D view
	if (!m_associatedWin)
	{
		//import GL filter so as to get the same rendering aspect!
		{
			ccGenericGLDisplay* sourceDisplay = aligned->getDisplay();
			if (!sourceDisplay && reference)
				sourceDisplay = reference->getDisplay();
			if (sourceDisplay)
			{
				ccGlFilter* filter = static_cast<ccGLWindow*>(sourceDisplay)->getGlFilter();
				if (filter)
					win->setGlFilter(filter->clone());
			}
		}
		linkWith(win);
		assert(m_associatedWin);
	}

	m_aligned = cloudContext(aligned);
	m_reference = cloudContext(reference);

	if (aligned->getDisplay())
	{
		const ccViewportParameters& vParams = aligned->getDisplay()->getViewportParameters();
		m_associatedWin->setViewportParameters(vParams);
	}

	//add aligned cloud to display
	if (aligned)
	{
		m_associatedWin->addToOwnDB(aligned);
		aligned->setVisible(true);
		aligned->setSelected(false);
		SetEnabled_recursive(aligned);
		//SetVisible_recursive(aligned);
	}

	//add reference cloud (if any) to display
	if (reference)
	{
		m_associatedWin->addToOwnDB(reference);
		reference->setVisible(true);
		reference->setSelected(false);
		SetEnabled_recursive(reference);
		//SetVisible_recursive(reference);
	}

	showReferenceCheckBox->setChecked(reference != 0);
	showReferenceCheckBox->setEnabled(reference != 0);
	showAlignedCheckBox->setChecked(true);

	m_associatedWin->showMaximized();
	m_associatedWin->zoomGlobal();
	m_associatedWin->redraw();
	m_associatedWin->displayNewMessage("[Point-pair registration]",ccGLWindow::UPPER_CENTER_MESSAGE,true,3600);

	onPointCountChanged();
	
	return true;
}

static QString s_aligned_tooltip("Whether the point is expressed in the cloud original coordinate system (before being shifted by CC) or not");
static double s_last_ax = 0;
static double s_last_ay = 0;
static double s_last_az = 0;
static bool s_last_a_isGlobal = true;
void ccPointPairRegistrationDlg::addManualAlignedPoint()
{
	ccAskThreeDoubleValuesDlg ptsDlg("x","y","z",-1.0e9,1.0e9,s_last_ax,s_last_ay,s_last_az,8,"Add aligned point",this);

	//if the aligned cloud is shifted, the user has the choice to input virtual point either
	//in the original coordinate system or the shifted one
	bool alignIsShifted = (m_aligned.cloud && m_aligned.cloud->isShifted());
	if (alignIsShifted)
		ptsDlg.showCheckbox("Not shifted",s_last_a_isGlobal,&s_aligned_tooltip);

	if (!ptsDlg.exec())
		return;

	//save values for current session
	s_last_ax = ptsDlg.doubleSpinBox1->value();
	s_last_ay = ptsDlg.doubleSpinBox2->value();
	s_last_az = ptsDlg.doubleSpinBox3->value();
	bool shifted = true;
	if (alignIsShifted)
	{
		s_last_a_isGlobal = ptsDlg.getCheckboxState();
		shifted = !s_last_a_isGlobal;
	}

	CCVector3d P(s_last_ax,s_last_ay,s_last_az);

	addAlignedPoint(P,0,shifted);
}

static double s_last_rx = 0;
static double s_last_ry = 0;
static double s_last_rz = 0;
static bool s_last_r_isGlobal = true;
void ccPointPairRegistrationDlg::addManualRefPoint()
{
	ccAskThreeDoubleValuesDlg ptsDlg("x","y","z",-1.0e9,1.0e9,s_last_rx,s_last_ry,s_last_rz,8,"Add reference point",this);

	//if the reference cloud is shifted, the user has the choice to input virtual
	//points either in the original coordinate system or the shifted one
	//(if there's no reference cloud, we use a 'global'	one by default)
	bool refIsShifted = (m_reference.cloud && m_reference.cloud->isShifted());
	if (refIsShifted)
		ptsDlg.showCheckbox("Not shifted",s_last_r_isGlobal,&s_aligned_tooltip);

	if (!ptsDlg.exec())
		return;

	//save values for current session
	s_last_rx = ptsDlg.doubleSpinBox1->value();
	s_last_ry = ptsDlg.doubleSpinBox2->value();
	s_last_rz = ptsDlg.doubleSpinBox3->value();
	bool shifted = (m_reference.cloud != 0);
	if (refIsShifted)
	{
		s_last_r_isGlobal = ptsDlg.getCheckboxState();
		shifted = !s_last_r_isGlobal;
	}

	CCVector3d P(s_last_rx,s_last_ry,s_last_rz);

	addReferencePoint(P,0,shifted);
}

void ccPointPairRegistrationDlg::pause(bool state)
{
	m_paused = state;
	setDisabled(state);
}

void ccPointPairRegistrationDlg::processPickedPoint(int cloudUniqueID, unsigned pointIndex, int, int)
{
	if (!m_associatedWin)
		return;
	//no point picking when paused!
	if (m_paused)
		return;

	ccHObject* db = m_associatedWin->getOwnDB();
	if (!db)
		return;

	ccHObject* obj = db->find(cloudUniqueID);
	if (obj && obj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
		const CCVector3* P = cloud->getPoint(pointIndex);

		if (P)
		{
			CCVector3d pin = CCVector3d::fromArray(P->u);

			if (cloud == m_aligned.cloud)
				addAlignedPoint(pin,m_aligned.cloud,true); //picked points are always shifted by default
			else if (cloud == m_reference.cloud)
				addReferencePoint(pin,m_reference.cloud,true); //picked points are always shifted by default
			else
			{
				assert(false);
				return;
			}

			m_associatedWin->redraw();
		}
	}
}

void ccPointPairRegistrationDlg::onPointCountChanged()
{
	bool canAlign = (m_alignedPoints.size() == m_refPoints.size() && m_refPoints.size() >= 4); //we need at least 4 points
	alignToolButton->setEnabled(canAlign);
	validToolButton->setEnabled(canAlign);

	unstackAlignToolButton->setEnabled(m_alignedPoints.size() != 0);
	unstackRefToolButton->setEnabled(m_refPoints.size() != 0);
}

static QToolButton* CreateDeleteButton()
{
	QToolButton* delButton = new QToolButton();
	delButton->setIcon(QIcon(":/CC/images/smallCancel.png"));
	return delButton;
}

static 	cc2DLabel* CreateLabel(ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = 0)
{
	cc2DLabel* label = new cc2DLabel();
	label->addPoint(cloud,pointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->setDisplayedIn3D(true);
	label->setDisplay(display);

	return label;
}

void ccPointPairRegistrationDlg::onDelButtonPushed()
{
	QObject* senderButton = sender();

	//go through all the buttons and find which one has been pushed!
	bool alignedPoint = true;
	int pointIndex = -1;
	//test 'aligned' buttons first
	{
		for (int i=0; i<alignedPointsTableWidget->rowCount(); ++i)
		{
			if (alignedPointsTableWidget->cellWidget(i,DEL_BUTTON_COL_INDEX) == senderButton)
			{
				pointIndex = i;
				break;
			}
		}
	}

	if (pointIndex < 0)
	{
		//test reference points if necessary
		alignedPoint = false;
		for (int i=0; i<refPointsTableWidget->rowCount(); ++i)
		{
			if (refPointsTableWidget->cellWidget(i,DEL_BUTTON_COL_INDEX) == senderButton)
			{
				pointIndex = i;
				break;
			}
		}
	}

	if (pointIndex < 0)
	{
		assert(false);
		return;
	}

	if (alignedPoint)
		removeAlignedPoint(pointIndex);
	else
		removeRefPoint(pointIndex);
}

void ccPointPairRegistrationDlg::addPointToTable(QTableWidget* tableWidget, int rowIndex, const CCVector3d& P, QString pointName)
{
	assert(tableWidget);
	if (!tableWidget)
		return;

	//add corresponding row in table
	tableWidget->setRowCount(std::max<int>(rowIndex+1,tableWidget->rowCount()));
	tableWidget->setVerticalHeaderItem(rowIndex,new QTableWidgetItem(pointName));

	//add point coordinates
	for (int d=0; d<3; ++d)
	{
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setData(Qt::EditRole, P.u[d]); 
		tableWidget->setItem(rowIndex, XYZ_COL_INDEX+d, item);
	}

	//add 'remove' button
	{
		if (rowIndex == 0)
			tableWidget->setColumnWidth(DEL_BUTTON_COL_INDEX, 20);
		//QTableWidgetItem* item = new QTableWidgetItem();
		//tableWidget->setItem(rowIndex, DEL_BUTTON_COL_INDEX, item);
		QToolButton* delButton = CreateDeleteButton();
		connect(delButton, SIGNAL(clicked()), this, SLOT(onDelButtonPushed()));
		tableWidget->setCellWidget(rowIndex, DEL_BUTTON_COL_INDEX, delButton);
	}
}

bool ccPointPairRegistrationDlg::addAlignedPoint(CCVector3d& Pin, ccGenericPointCloud* cloud/*=0*/, bool shifted/*=0*/)
{
	//if the input point is not shifted, we shift it to the aligned coordinate system
	assert(cloud == 0 || cloud == m_aligned.cloud);

	//first point?
	if (m_alignedPoints.size() == 0)
	{
		assert(m_aligned.cloud);
		//simply copy the cloud global shift/scale
		m_alignedPoints.setGlobalScale(m_aligned.cloud->getGlobalScale());
		m_alignedPoints.setGlobalShift(m_aligned.cloud->getGlobalShift());
	}

	//transform the input point in the 'global world' by default
	if (shifted)
		Pin = m_alignedPoints.toGlobal3d<double>(Pin);

	//check that we don't duplicate points
	for (unsigned i=0; i<m_alignedPoints.size(); ++i)
	{
		CCVector3d Pi = m_alignedPoints.toGlobal3d<PointCoordinateType>(*m_alignedPoints.getPoint(i));
		if ((Pi-Pin).norm() < ZERO_TOLERANCE)
		{
			ccLog::Error("Point already picked or too close to an already selected one!");
			return false;
		}
	}

	unsigned newPointIndex = m_alignedPoints.size();
	if (newPointIndex == m_alignedPoints.capacity() && !m_alignedPoints.reserve(newPointIndex+1))
	{
		ccLog::Error("Not enough memory?!");
		return false;
	}

	//shift point to the local coordinate system before pushing it
	CCVector3 P = m_alignedPoints.toLocal3pc<double>(Pin);
	m_alignedPoints.addPoint(P);
	
	QString pointName = QString("A%1").arg(newPointIndex);
	
	//add corresponding row in table
	addPointToTable(alignedPointsTableWidget,newPointIndex,Pin,pointName);

	//add a label!
	cc2DLabel* label = CreateLabel(&m_alignedPoints,newPointIndex,pointName,m_associatedWin);
	m_alignedPoints.addChild(label);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	return true;
}

void ccPointPairRegistrationDlg::unstackAligned()
{
	unsigned pointCount = m_alignedPoints.size();
	if (pointCount == 0) //nothing to do
		return;

	assert(alignedPointsTableWidget->rowCount()>0);
	alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount()-1);

	//remove label
	assert(m_alignedPoints.getChildrenNumber() == pointCount);
	m_alignedPoints.removeChild(pointCount-1);
	//remove point
	m_alignedPoints.resize(pointCount-1);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::removeAlignedPoint(int index, bool autoRemoveDualPoint/*=false*/)
{
	if (index >= static_cast<int>(m_alignedPoints.size()))
	{
		ccLog::Error("[ccPointPairRegistrationDlg::removeAlignedPoint] Invalid index!");
		assert(false);
		return;
	}

	int pointCount = static_cast<int>(m_alignedPoints.size());
	//remove all labels above this index
	assert(m_alignedPoints.getChildrenNumber() == pointCount);
	{
		for (int i=pointCount-1; i>=index; --i) //downward for more efficiency
		{
			assert(m_alignedPoints.getChild(i) && m_alignedPoints.getChild(i)->isA(CC_TYPES::LABEL_2D));
			m_alignedPoints.removeChild(i);
		}
	}
	//remove array row
	alignedPointsTableWidget->removeRow(index);

	//shift points & rename labels
	for (int i=index+1; i<pointCount; ++i)
	{
		*const_cast<CCVector3*>(m_alignedPoints.getPoint(i-1)) = *m_alignedPoints.getPoint(i);

		//new name
		QString pointName = QString("A%1").arg(i-1);
		//create new label
		cc2DLabel* label = CreateLabel(&m_alignedPoints,static_cast<unsigned>(i-1),pointName,m_associatedWin);
		m_alignedPoints.addChild(label);
		//update array
		alignedPointsTableWidget->setVerticalHeaderItem(i-1,new QTableWidgetItem(pointName));
	}
	m_alignedPoints.invalidateBoundingBox();
	
	pointCount--;
	assert(pointCount >= 0);
	m_alignedPoints.resize(static_cast<unsigned>(pointCount));

	if (m_alignedPoints.size() == 0)
	{
		//reset global shift (if any)
		m_alignedPoints.setGlobalShift(0,0,0);
		m_alignedPoints.setGlobalScale(1.0);
		return;
	}

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	//auto-remove the other point?
	if (	autoRemoveDualPoint
		&&	index < static_cast<int>(m_refPoints.size())
		&&	QMessageBox::question(0,"Remove dual point","Remove the equivalent reference point as well?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes )
	{
		removeRefPoint(index,false);
	}
}

bool ccPointPairRegistrationDlg::addReferencePoint(CCVector3d& Pin, ccGenericPointCloud* cloud/*=0*/, bool shifted/*=true*/)
{
	assert(cloud == 0 || cloud == m_reference.cloud);

	//first point?
	if (m_refPoints.size() == 0)
	{
		if (cloud) //picked point
		{
			//simply copy the cloud global shift/scale
			m_refPoints.setGlobalScale(cloud->getGlobalScale());
			m_refPoints.setGlobalShift(cloud->getGlobalShift());
		}
		else //virtual point
		{
			m_refPoints.setGlobalScale(1.0);
			m_refPoints.setGlobalShift(0,0,0);
			
			if (!shifted)
			{
				//test that the input point has not too big coordinates
				bool shiftEnabled = false;
				CCVector3d Pshift(0,0,0);
				double scale = 1.0;
				//we use the aligned shift by default (if any)
				if (m_aligned.cloud && m_aligned.cloud->isShifted())
				{
					Pshift = m_aligned.cloud->getGlobalShift();
					scale = m_aligned.cloud->getGlobalScale();
					shiftEnabled = true;
				}
				if (ccGlobalShiftManager::Handle(Pin,0,ccGlobalShiftManager::DIALOG_IF_NECESSARY,shiftEnabled,Pshift,&scale))
				{
					m_refPoints.setGlobalShift(Pshift);
					m_refPoints.setGlobalScale(scale);
				}
			}
		}
	}

	//transform the input point in the 'global world' by default
	if (shifted)
		Pin = cloud->toGlobal3d<double>(Pin);

	//check that we don't duplicate points
	for (unsigned i=0; i<m_refPoints.size(); ++i)
	{
		//express the 'Pi' point in the current global coordinate system
		CCVector3d Pi = m_refPoints.toGlobal3d<PointCoordinateType>(*m_refPoints.getPoint(i));
		if ((Pi-Pin).norm() < ZERO_TOLERANCE)
		{
			ccLog::Error("Point already picked or too close to an already selected one!");
			return false;
		}
	}

	//add point to the 'reference' set
	unsigned newPointIndex = m_refPoints.size();
	if (newPointIndex == m_refPoints.capacity() && !m_refPoints.reserve(newPointIndex+1))
	{
		ccLog::Error("Not enough memory?!");
		return false;
	}
	
	//shift point to the local coordinate system before pushing it
	CCVector3 P = m_refPoints.toLocal3pc<double>(Pin);
	m_refPoints.addPoint(P);
	
	QString pointName = QString("R%1").arg(newPointIndex);
	
	//add corresponding row in table
	addPointToTable(refPointsTableWidget,newPointIndex,Pin,pointName);

	//add a label!
	cc2DLabel* label = CreateLabel(&m_refPoints,newPointIndex,pointName,m_associatedWin);
	m_refPoints.addChild(label);

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	return true;
}

void ccPointPairRegistrationDlg::unstackRef()
{
	unsigned pointCount = m_refPoints.size();
	if (pointCount == 0)
		return;

	assert(refPointsTableWidget->rowCount()>0);
	refPointsTableWidget->removeRow(refPointsTableWidget->rowCount()-1);

	//remove label
	assert(m_refPoints.getChildrenNumber() == pointCount);
	pointCount--;
	m_refPoints.removeChild(pointCount);
	//remove point
	m_refPoints.resize(pointCount);

	if (pointCount == 0)
	{
		//reset global shift (if any)
		m_refPoints.setGlobalShift(0,0,0);
		m_refPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::removeRefPoint(int index, bool autoRemoveDualPoint/*=false*/)
{
	if (index >= static_cast<int>(m_refPoints.size()))
	{
		ccLog::Error("[ccPointPairRegistrationDlg::removeRefPoint] Invalid index!");
		assert(false);
		return;
	}

	int pointCount = static_cast<int>(m_refPoints.size());
	//remove all labels above this index
	assert(m_refPoints.getChildrenNumber() == pointCount);
	{
		for (int i=pointCount-1; i>=index; --i) //downward for more efficiency
		{
			assert(m_refPoints.getChild(i) && m_refPoints.getChild(i)->isA(CC_TYPES::LABEL_2D));
			m_refPoints.removeChild(i);
		}
	}
	//remove array row
	refPointsTableWidget->removeRow(index);

	//shift points & rename labels
	for (int i=index+1; i<pointCount; ++i)
	{
		*const_cast<CCVector3*>(m_refPoints.getPoint(i-1)) = *m_refPoints.getPoint(i);

		//new name
		QString pointName = QString("R%1").arg(i-1);
		//create new label
		cc2DLabel* label = CreateLabel(&m_refPoints,static_cast<unsigned>(i-1),pointName,m_associatedWin);
		m_refPoints.addChild(label);
		//update array
		refPointsTableWidget->setVerticalHeaderItem(i-1,new QTableWidgetItem(pointName));
	}
	m_refPoints.invalidateBoundingBox();
	
	pointCount--;
	assert(pointCount >= 0);
	m_refPoints.resize(static_cast<unsigned>(pointCount));

	if (m_refPoints.size() == 0)
	{
		//reset global shift (if any)
		m_refPoints.setGlobalShift(0,0,0);
		m_refPoints.setGlobalScale(1.0);
		return;
	}

	if (m_associatedWin)
		m_associatedWin->redraw();

	onPointCountChanged();

	//auto-remove the other point?
	if (	autoRemoveDualPoint
		&&	index < static_cast<int>(m_alignedPoints.size())
		&&	QMessageBox::question(0,"Remove dual point","Remove the equivalent aligned point as well?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes )
	{
		removeAlignedPoint(index,false);
	}
}

void ccPointPairRegistrationDlg::showAlignedCloud(bool state)
{
	if (!m_aligned.cloud)
		return;

	m_aligned.cloud->setVisible(state);
	m_alignedPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

void ccPointPairRegistrationDlg::showReferenceCloud(bool state)
{
	if (!m_reference.cloud)
		return;

	m_reference.cloud->setVisible(state);
	m_refPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}
}

bool ccPointPairRegistrationDlg::callHornRegistration(CCLib::PointProjectionTools::Transformation& trans, double& rms)
{
	assert(m_aligned.cloud);
	if (!m_aligned.cloud)
		return false;

	if (m_alignedPoints.size() != m_refPoints.size() || m_refPoints.size() < 4)
	{
		assert(false);
		ccLog::Error("Need at least 4 points for each cloud (and the same number of points in both clouds)!");
		return false;
	}

	//fixed scale?
	bool adjustScale = adjustScaleCheckBox->isChecked();

	//call Horn registration method
	if (!CCLib::HornRegistrationTools::FindAbsoluteOrientation(&m_alignedPoints, &m_refPoints, trans, !adjustScale))
	{
		ccLog::Error("Registration failed! (points are aligned?)");
		return false;
	}

	//apply constraints (if any)
	{
		int filters = 0;
		switch (rotComboBox->currentIndex())
		{
		case 1:
			filters |= CCLib::RegistrationTools::SKIP_RYZ;
			break;
		case 2:
			filters |= CCLib::RegistrationTools::SKIP_RXZ;
			break;
		case 3:
			filters |= CCLib::RegistrationTools::SKIP_RXY;
			break;
		default:
			//nothing to do
			break;
		}

		if (!TxCheckBox->isChecked())
			filters |= CCLib::RegistrationTools::SKIP_TX;
		if (!TyCheckBox->isChecked())
			filters |= CCLib::RegistrationTools::SKIP_TY;
		if (!TzCheckBox->isChecked())
			filters |= CCLib::RegistrationTools::SKIP_TZ;

		if (filters != 0)
		{
			CCLib::RegistrationTools::FilterTransformation(trans,filters,trans);
		}
	}

	//compute RMS
	rms = CCLib::HornRegistrationTools::ComputeRMS(&m_alignedPoints, &m_refPoints, trans);

	return true;
}

void ccPointPairRegistrationDlg::clearRMSColumns()
{
	for (int i=0; alignedPointsTableWidget->rowCount(); ++i)
		alignedPointsTableWidget->setItem(i,RMS_COL_INDEX,new QTableWidgetItem());
	for (int i=0; refPointsTableWidget->rowCount(); ++i)
		refPointsTableWidget->setItem(i,RMS_COL_INDEX,new QTableWidgetItem());
}

void ccPointPairRegistrationDlg::align()
{
	CCLib::PointProjectionTools::Transformation trans;
	double rms;

	if (callHornRegistration(trans,rms))
	{
		if (rms >= 0)
		{
			QString rmsString = QString("Current RMS: %1").arg(rms);
			ccLog::Print(QString("[PointPairRegistration] ")+rmsString);
			m_associatedWin->displayNewMessage(rmsString,ccGLWindow::UPPER_CENTER_MESSAGE,true);

			//display resulting RMS in colums
			{
				//add 'remove' button
				assert(m_alignedPoints.size() == m_refPoints.size());
				for (unsigned i=0; i<m_alignedPoints.size(); ++i)
				{
					const CCVector3* Ri = m_refPoints.getPoint(i);
					const CCVector3* Li = m_alignedPoints.getPoint(i);
					CCVector3 Lit = (trans.R.isValid() ? trans.R * (*Li) : (*Li))*trans.s + trans.T;
					PointCoordinateType dist = (*Ri-Lit).norm();

					QTableWidgetItem* itemA = new QTableWidgetItem();
					itemA->setData(Qt::EditRole, dist); 
					alignedPointsTableWidget->setItem(i, RMS_COL_INDEX, itemA);
					QTableWidgetItem* itemR = new QTableWidgetItem();
					itemR->setData(Qt::EditRole, dist); 
					refPointsTableWidget->setItem(i, RMS_COL_INDEX, itemR);
				}
			}
		}
		else
		{
			//clear RMS columns
			clearRMSColumns();
		}

		//fixed scale?
		bool adjustScale = adjustScaleCheckBox->isChecked();

		//apply (scaled) transformation...
		if (adjustScale)
		{
			if (trans.R.isValid())
				trans.R.scale(trans.s);

			QString scaleString = QString("Scale: %1").arg(trans.s);
			ccLog::Print(QString("[PointPairRegistration] ")+scaleString);
		}
		else
		{
			ccLog::Print(QString("[PointPairRegistration] Scale: fixed (1.0)"));
		}

		ccGLMatrix transMat = FromCCLibMatrix<PointCoordinateType,float>(trans.R,trans.T);
		//...virtually
		m_aligned.cloud->setGLTransformation(transMat);
		m_alignedPoints.setGLTransformation(transMat);

		if (m_associatedWin)
		{
			if (autoZoomCheckBox->isChecked())
				m_associatedWin->zoomGlobal();
			m_associatedWin->redraw();
		}

		resetToolButton->setEnabled(true);
	}
}

void ccPointPairRegistrationDlg::reset()
{
	if (!m_aligned.cloud)
		return;

	m_aligned.cloud->enableGLTransformation(false);
	m_alignedPoints.enableGLTransformation(false);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
			m_associatedWin->zoomGlobal();
		m_associatedWin->redraw();
	}

	resetToolButton->setEnabled(false);
}

void ccPointPairRegistrationDlg::apply()
{
	CCLib::PointProjectionTools::Transformation trans;
	double rms = -1.0;
	
	if (callHornRegistration(trans,rms))
	{
		QStringList summary;
		if (rms >= 0)
		{
			QString rmsString = QString("Final RMS: %1").arg(rms);
			ccLog::Print(QString("[PointPairRegistration] ")+rmsString);
			summary << rmsString;
			summary << "----------------";
		}

		//fixed scale?
		bool adjustScale = adjustScaleCheckBox->isChecked();
		//apply (scaled) transformation...
		if (adjustScale && trans.R.isValid())
			trans.R.scale(trans.s);
		ccGLMatrix transMat = FromCCLibMatrix<PointCoordinateType,float>(trans.R,trans.T);
		//...for real this time!
		assert(m_aligned.cloud);
		m_aligned.cloud->applyGLTransformation_recursive();
		m_alignedPoints.setGLTransformation(transMat);

		QString matString = transMat.toString();
		summary << QString("Transformation matrix");
		summary << transMat.toString(3,'\t'); //low precision, just for display
		summary << "----------------";

		ccLog::Print("[PointPairRegistration] Applied transformation matrix:");
		ccLog::Print(transMat.toString(12,' ')); //full precision
		
		if (adjustScale)
		{
			QString scaleString = QString("Scale: %1 (already integrated in above matrix!)").arg(trans.s);
			ccLog::Warning(QString("[PointPairRegistration] ")+scaleString);
			summary << scaleString;
		}
		else
		{
			ccLog::Print(QString("[PointPairRegistration] Scale: fixed (1.0)"));
			summary << "Scale: fixed (1.0)";
		}
		summary << "----------------";

		//pop-up summary
		summary << "Refer to Console (F8) for more details";
		QMessageBox::information(this,"Align info",summary.join("\n"));

		//don't forget global shift
		if (m_refPoints.isShifted())
		{
			const CCVector3d& Pshift = m_refPoints.getGlobalShift();
			const double& scale = m_refPoints.getGlobalScale();
			m_aligned.cloud->setGlobalShift(Pshift);
			m_aligned.cloud->setGlobalScale(scale);
			ccLog::Warning(QString("[PointPairRegistration] Aligned cloud global shift has been updated to match the reference: (%1,%2,%3) [x%4]").arg(Pshift.x).arg(Pshift.y).arg(Pshift.z).arg(scale));
		}
		else if (m_aligned.cloud->isShifted()) //we'll ask the user first before dropping the shift information on the aligned cloud
		{
			if (QMessageBox::question(this, "Drop shift information?", "Aligned cloud is shifted but reference cloud is not: drop global shift information?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
			{
				m_aligned.cloud->setGlobalShift(0,0,0);
				m_aligned.cloud->setGlobalScale(1.0);
				ccLog::Warning(QString("[PointPairRegistration] Aligned cloud global shift has been reset to match the reference!"));
			}
		}
	}
	else
	{
		ccLog::Warning(QString("[PointPairRegistration] Failed to register entities?!"));
	}

	stop(true);
}

void ccPointPairRegistrationDlg::cancel()
{
	if (m_aligned.cloud)
		m_aligned.cloud->enableGLTransformation(false);

	stop(false);
}
