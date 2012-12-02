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
//$Rev:: 2242                                                              $
//$LastChangedDate:: 2012-09-22 18:54:21 +0200 (sam., 22 sept. 2012)       $
//**************************************************************************
//

#include "ccPointListPickingDlg.h"

//Qt
#include <QSettings>
#include <QFileDialog>
#include <QMenu>

//qCC_db
#include <ccPointCloud.h>
#include <cc2DLabel.h>

//local
#include "ccGLWindow.h"
#include "ccGuiParameters.h"
#include "fileIO/FileIOFilter.h"
#include "mainwindow.h"
#include "db_tree/ccDBRoot.h"

//system
#include <assert.h>

ccPointListPickingDlg::ccPointListPickingDlg(QWidget* parent)
	: ccPointPickingGenericInterface(parent)
	, Ui::PointListPickingDlg()
	, m_associatedCloud(0)
	, m_orderedLabelsContainer(0)
	, m_toBeDeleted(0)
	, m_toBeAdded(0)
	, m_lastPreviousID(0)
{
	setupUi(this);
	setWindowFlags(Qt::FramelessWindowHint |Qt::Tool);

	exportToolButton->setPopupMode(QToolButton::MenuButtonPopup);
	QMenu* menu = new QMenu(exportToolButton);
	QAction* exportASCII_xyz = menu->addAction("x,y,z");
	QAction* exportASCII_ixyz = menu->addAction("local index,x,y,z");
	QAction* exportToNewCloud = menu->addAction("new cloud");
	exportToolButton->setMenu(menu);

	tableWidget->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);

	const ccGui::ParamStruct& guiParams = ccGui::Parameters();
	markerSizeSpinBox->setValue(guiParams.pickedPointsSize);
	startIndexSpinBox->setValue(guiParams.pickedPointsStartIndex);

	connect(cancelToolButton,       SIGNAL(clicked()),          this,				SLOT(cancelAndExit()));
	connect(revertToolButton,       SIGNAL(clicked()),          this,				SLOT(removeLastEntry()));
	connect(validToolButton,        SIGNAL(clicked()),          this,				SLOT(applyAndExit()));
	connect(exportToolButton,       SIGNAL(clicked()),          exportToolButton,	SLOT(showMenu()));
	connect(exportASCII_xyz,        SIGNAL(triggered()),        this,				SLOT(exportToASCII_xyz()));
	connect(exportASCII_ixyz,       SIGNAL(triggered()),        this,				SLOT(exportToASCII_ixyz()));
	connect(exportToNewCloud,		SIGNAL(triggered()),        this,				SLOT(exportToNewCloud()));
	connect(markerSizeSpinBox,      SIGNAL(valueChanged(int)),  this,				SLOT(markerSizeChanged(int)));
	connect(startIndexSpinBox,      SIGNAL(valueChanged(int)),  this,				SLOT(startIndexChanged(int)));

	m_toBeDeleted = new ccHObject("To be deleted");
	m_toBeAdded = new ccHObject("To be Added");

	updateList();
}

unsigned ccPointListPickingDlg::getPickedPoints(std::vector<cc2DLabel*>& pickedPoints)
{
	pickedPoints.clear();

	if (m_orderedLabelsContainer)
	{
		//get all labels
		ccHObject::Container labels;
		unsigned count = m_orderedLabelsContainer->filterChildren(labels,false,CC_2D_LABEL);
		//find highest unique ID among the VISIBLE labels
		pickedPoints.reserve(count);
		for (unsigned i=0;i<count;++i)
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(labels[i]);
			if (label->isVisible() && label->size()==1)
				pickedPoints.push_back(label);
		}
	}

	return pickedPoints.size();
}

void ccPointListPickingDlg::linkWithCloud(ccPointCloud* cloud)
{
	m_associatedCloud = cloud;
	m_lastPreviousID = 0;
	if (m_associatedCloud)
	{
		//find default container
		m_orderedLabelsContainer = 0;
		for (unsigned i=0;i<m_associatedCloud->getChildrenNumber();++i)
		{
			ccHObject* child = m_associatedCloud->getChild(i);
			if (child->isA(CC_HIERARCHY_OBJECT) && child->getName() == "Picked points list")
			{
				m_orderedLabelsContainer = child;
				break;
			}
		}

		std::vector<cc2DLabel*> previousPickedPoints;
		unsigned count = getPickedPoints(previousPickedPoints);
		//find highest unique ID among the VISIBLE labels
		for (unsigned i=0;i<count;++i)
		{
			if (previousPickedPoints[i]->getUniqueID() > m_lastPreviousID)
				m_lastPreviousID = previousPickedPoints[i]->getUniqueID();
		}
	}
	updateList();
}

void ccPointListPickingDlg::cancelAndExit()
{
	if (m_orderedLabelsContainer)
	{
		//Restore previous state
		for (unsigned i=0;i<m_toBeAdded->getChildrenNumber();++i)
			m_orderedLabelsContainer->removeChild(m_toBeAdded->getChild(i));
		for (unsigned j=0;j<m_toBeDeleted->getChildrenNumber();++j)
			m_toBeDeleted->getChild(j)->setVisible(true);
		if (m_orderedLabelsContainer->getChildrenNumber() == 0)
		{
			m_associatedCloud->removeChild(m_orderedLabelsContainer);
			m_orderedLabelsContainer=0;
		}
	}

	m_toBeDeleted->removeAllChildren();
	m_toBeAdded->removeAllChildren();
	m_associatedCloud = 0;
	m_orderedLabelsContainer = 0;

	updateList();

	stop(false);
}

void ccPointListPickingDlg::exportToNewCloud()
{
	if (!m_associatedCloud)
		return;

	//get all labels
	std::vector<cc2DLabel*> labels;
	unsigned count = getPickedPoints(labels);
	if (count>0)
	{
		ccPointCloud* cloud = new ccPointCloud();
		if (cloud->reserve(count))
		{
			cloud->setName("Picking list");
			for (unsigned i=0;i<count;++i)
			{
				const cc2DLabel::PickedPoint& PP = labels[i]->getPoint(0);
				const CCVector3* P = PP.cloud->getPoint(PP.index);
				cloud->addPoint(*P);
			}

			cloud->setDisplay(m_associatedCloud->getDisplay());
			MainWindow::TheInstance()->db()->addElement(cloud);
		}
		else
		{
			ccConsole::Error("Couldn't export picked points as point cloud: not enough memory!");
			delete cloud;
			cloud=0;
		}
	}
	else
	{
		ccConsole::Error("Pick some points first!");
	}
}

void ccPointListPickingDlg::applyAndExit()
{
	if (m_associatedCloud)
	{
		//Apply modifications
		{
			for (unsigned i=0;i<m_toBeDeleted->getChildrenNumber();++i)
				MainWindow::TheInstance()->db()->removeElement(m_toBeDeleted->getChild(i));
		}
		m_associatedCloud = 0;
	}

	m_toBeDeleted->removeAllChildren();
	m_toBeAdded->removeAllChildren();
	m_orderedLabelsContainer = 0;

	updateList();

	stop(true);
}

void ccPointListPickingDlg::removeLastEntry()
{
	if (!m_associatedCloud)
		return;

	//get all labels
	std::vector<cc2DLabel*> labels;
	unsigned count = getPickedPoints(labels);
	if (count==0)
		return;

	ccHObject* lastVisibleLabel = labels.back();
	if (lastVisibleLabel->getUniqueID() <= m_lastPreviousID)
	{
		//old label: hide it and add it to the 'to be deleted' list (will be restored if process s cancelled)
		lastVisibleLabel->setVisible(false);
		m_toBeDeleted->addChild(lastVisibleLabel,false);
	}
	else
	{
		m_associatedCloud->removeChild(lastVisibleLabel);
	}

	updateList();

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccPointListPickingDlg::startIndexChanged(int value)
{
	ccGui::ParamStruct guiParams = ccGui::Parameters();

	if (guiParams.pickedPointsStartIndex != (unsigned)value)
	{
		guiParams.pickedPointsStartIndex = (unsigned)value;
		ccGui::Set(guiParams);

		updateList();
		if (m_associatedWin)
			m_associatedWin->redraw();
	}
}

void ccPointListPickingDlg::markerSizeChanged(int size)
{
	if (size<1)
		return;

	ccGui::ParamStruct guiParams = ccGui::Parameters();

	if (guiParams.pickedPointsSize != (unsigned)size)
	{
		guiParams.pickedPointsSize = (unsigned)size;
		ccGui::Set(guiParams);
		if (m_associatedWin)
			m_associatedWin->redraw();
	}
}

void ccPointListPickingDlg::exportToASCII_xyz()
{
	exportToASCII(PLP_ASCII_EXPORT_XYZ);
}

void ccPointListPickingDlg::exportToASCII_ixyz()
{
	exportToASCII(PLP_ASCII_EXPORT_IXYZ);
}

void ccPointListPickingDlg::exportToASCII(ExportFormat format)
{
	if (!m_associatedCloud)
		return;

	//get all labels
	std::vector<cc2DLabel*> labels;
	unsigned i,count = getPickedPoints(labels);
	if (count==0)
		return;

	QSettings settings;
	settings.beginGroup("PointListPickingDlg");
	QString filename = settings.value("filename", "picking_list.txt").toString();
	settings.endGroup();

	filename = QFileDialog::getSaveFileName(this,
		"Export to ASCII",
		filename,
		CC_FILE_TYPE_FILTERS[ASCII]);

	if (filename.isEmpty())
		return;

	settings.beginGroup("PointListPickingDlg");
	settings.setValue("filename", filename);
	settings.endGroup();

	FILE* fp = fopen(qPrintable(filename),"wt");
	if (!fp)
	{
		ccConsole::Error(QString("Failed to open file '%1' for saving!").arg(filename));
		return;
	}

	//starting index
	int startIndex = startIndexSpinBox->value();

	for (i=0;i<count;++i)
	{
		const cc2DLabel::PickedPoint& PP = labels[i]->getPoint(0);
		const CCVector3* P = PP.cloud->getPoint(PP.index);
		switch(format)
		{
		case PLP_ASCII_EXPORT_XYZ:
			fprintf(fp,"%f,%f,%f\n",P->x,P->y,P->z);
			break;
		case PLP_ASCII_EXPORT_IXYZ:
			fprintf(fp,"%i,%f,%f,%f\n",i+startIndex,P->x,P->y,P->z);
			break;
		}
	}

	fclose(fp);

	ccConsole::Print(QString("File '%1' successfully saved!").arg(filename));
}

void ccPointListPickingDlg::updateList()
{
	//get all labels
	std::vector<cc2DLabel*> labels;
	unsigned count = getPickedPoints(labels);

	revertToolButton->setEnabled(count);
	validToolButton->setEnabled(count);
	exportToolButton->setEnabled(count);
	countLineEdit->setText(QString::number(count));
	tableWidget->setRowCount(count);

	if (!count)
		return;

	//starting index
	int startIndex = startIndexSpinBox->value();
	int precision = ccGui::Parameters().displayedNumPrecision;

	for (unsigned i=0;i<count;++i)
	{
		const cc2DLabel::PickedPoint& PP = labels[i]->getPoint(0);
		const CCVector3* P = PP.cloud->getPoint(PP.index);

		//point index in list
		tableWidget->setVerticalHeaderItem(i,new QTableWidgetItem(QString("%1").arg(i+startIndex)));
		//update name as well
		labels[i]->setName(QString("Point #%1").arg(i+startIndex));
		//point absolute index (in cloud)
		tableWidget->setItem(i,0,new QTableWidgetItem(QString("%1").arg(PP.index)));

		for (unsigned j=0;j<3;++j)
			tableWidget->setItem(i,j+1,new QTableWidgetItem(QString("%1").arg(P->u[j],0,'f',precision)));
	}

	tableWidget->scrollToBottom();
}

void ccPointListPickingDlg::processPickedPoint(ccPointCloud* cloud, unsigned pointIndex, int x, int y)
{
	if (cloud != m_associatedCloud || !cloud)
		return;

	cc2DLabel* newLabel = new cc2DLabel();
	ccGenericGLDisplay* display = m_associatedCloud->getDisplay();
	newLabel->setDisplay(display);
	newLabel->addPoint(cloud,pointIndex);
	newLabel->setVisible(true);
	newLabel->setDisplayedIn2D(false);
	newLabel->setCollapsed(true);
	if (display)
	{
		int vp[4];
		display->getViewportArray(vp);
		newLabel->setPosition((float)(x+20)/(float)(vp[2]-vp[0]),(float)(y+20)/(float)(vp[3]-vp[1]));
	}

	//add default container if necessary
	if (!m_orderedLabelsContainer)
	{
		m_orderedLabelsContainer = new ccHObject("Picked points list");
		m_associatedCloud->addChild(m_orderedLabelsContainer,true);
	}
	assert(m_orderedLabelsContainer);
	m_orderedLabelsContainer->addChild(newLabel,true);

	m_toBeAdded->addChild(newLabel,false);

	updateList();

	if (m_associatedWin)
		m_associatedWin->redraw();
}
