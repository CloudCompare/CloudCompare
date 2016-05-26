//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qCSF                        #
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
//#                      COPYRIGHT: Qi jianbo ; Wan peng                   #
//#                                                                        #
//##########################################################################

#include "qCSF.h"

//Qt
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>
#include <QComboBox>


//Dialog

#include "ccCSFDlg.h"

//System
#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccHObjectCaster.h>
#include <ccOctree.h>

//CSF
#include <CSF.h>

//Default constructor: should mainly be used to initialize
//actions (pointers) and other members
qCSF::qCSF(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}	

//This method should enable or disable each plugin action
//depending on the currently selected entities ('selectedEntities').
//For example: if none of the selected entities is a cloud, and your
//plugin deals only with clouds, call 'm_action->setEnabled(false)'
void qCSF::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

//This method returns all 'actions' of your plugin.
//It will be called only once, when plugin is loaded.
void qCSF::getActions(QActionGroup& group)
{
	//default action (if it has not been already created, it's the moment to do it)
	if (!m_action)
	{
		//here we use the default plugin name, description and icon,
		//but each action can have its own!
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect appropriate signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

//This is an example of an action's slot called when the corresponding action
//is triggered (i.e. the corresponding icon or menu entry is clicked in CC
//main's interface). You can access to most of CC components (database,
//3D views, console, etc.) via the 'm_app' attribute (ccMainAppInterface
//object).

//initialize the dialog state.
static bool csf_postprocessing = false;
static bool csf_rig1=false;
static bool csf_rig2=false;
static bool csf_rig3=true;
static int MaxIteration = 500;
static double cloth_resolution = 2;
static double class_threshold = 0.5;

//main function of CSF
void qCSF::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum!=1)
	{
		m_app->dispToConsole("Select only one cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//to get the point cloud from selected entity.
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
	

	////Convert CC point cloud to CSF type
	unsigned count = pc->size();
	PointCloud csfPC;
	for(int i=0;i<count;i++)
	{
            wl:LASPoint tmpPoint;
			const CCVector3* P = pc->getPoint(i);
			tmpPoint.x = P->x;
			tmpPoint.y = P->y;
			tmpPoint.z = P->z;
			csfPC.push_back(tmpPoint);
	}

	/*** HERE STARTS THE ACTION ***/
	// display the dialog
	ccCSFDlg csfDlg(m_app->getMainWindow());
	csfDlg.rig1->setChecked(csf_rig1);
	csfDlg.rig2->setChecked(csf_rig2);
	csfDlg.rig3->setChecked(csf_rig3);
	csfDlg.MaxIterationSpinBox->setValue(MaxIteration);
	csfDlg.cloth_resolutionSpinBox->setValue(cloth_resolution);
	csfDlg.class_thresholdSpinBox->setValue(class_threshold);
	if (!csfDlg.exec())
		return;

	//display the progress dialog
	QProgressDialog pDlg;
	pDlg.setWindowTitle("CSF");
	pDlg.setLabelText("Computing....");
	pDlg.setCancelButton(0);
	pDlg.show();
	QApplication::processEvents();

	//instantiation a CSF class
	CSF csf;
	csf.setPointCloud(csfPC);//Class PointCloud csfPC

	// setup parameter
	csf.params.bSloopSmooth = csfDlg.postprocessingcheckbox->isChecked();
	csf.params.class_threshold = csfDlg.class_thresholdSpinBox->value();
	
	csf.params.cloth_resolution = csfDlg.cloth_resolutionSpinBox->value();
	csf.params.interations = csfDlg.MaxIterationSpinBox->value();
	csf.params.k_nearest_points = 1;
	if (csfDlg.rig1->isChecked())
	{
		csf.params.rigidness = 1;
	}
	else if (csfDlg.rig2->isChecked())
	{
		csf.params.rigidness = 2;
	}
	else 
	{
		csf.params.rigidness = 3;
	}
	csf.params.time_step = 0.75;  

	//to do filtering
	std::vector<vector<int>> segIndex = csf.do_filtering(count);

	//输出地面点和非地面点
	CCLib::ReferenceCloud* groundpc = new CCLib::ReferenceCloud(pc);
	CCLib::ReferenceCloud* offgroundpc = new CCLib::ReferenceCloud(pc);

	  //输出地面点
	for (unsigned j=0; j<segIndex[1].size(); ++j)
	{
		groundpc->addPointIndex(segIndex[1][j]);	
	}

	  //输出非地面点
	for (unsigned k=0;k<segIndex[2].size();++k)
	{
			offgroundpc->addPointIndex(segIndex[2][k]);
	}	
	ccPointCloud* groundpoint = pc->partialClone(groundpc);
	ccPointCloud* offgroundpoint = pc->partialClone(offgroundpc);

	//新建点云容器
	ccHObject* cloudContainer = new ccHObject(pc->getName()+QString("_csf"));

	pDlg.hide();
	QApplication::processEvents();
	
	//隐藏原始点云
	pc->setEnabled(false);

	//we add new group to DB/display（滤波结果显示）
	groundpoint->setVisible(true);
	groundpoint->setName("ground points");

	offgroundpoint->setVisible(true);
	offgroundpoint->setName("off-ground points");

	cloudContainer->addChild(groundpoint);
	cloudContainer->addChild(offgroundpoint);

	m_app->addToDB(cloudContainer);
	m_app->refreshAll();
	/*** HERE ENDS THE ACTION ***/
}


//This method should return the plugin icon (it will be used mainly
//if your plugin as several actions in which case CC will create a
//dedicated sub-menu entry with this icon.
QIcon qCSF::getIcon() const
{
	//open qCSF.qrc (text file), update the "prefix" and the
	//icon(s) filename(s). Then save it with the right name (yourPlugin.qrc).
	//(eventually, remove the original qCSF.qrc file!)
	return QIcon(":/CC/plugin/qCSF/icon.png");
}

#ifndef CC_QT5
Q_EXPORT_PLUGIN2(qCSF,qCSF);
#endif
