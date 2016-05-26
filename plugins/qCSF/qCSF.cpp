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

qCSF::qCSF(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}	

void qCSF::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

void qCSF::getActions(QActionGroup& group)
{
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect appropriate signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

//initialize the dialog state.
static bool csf_postprocessing = false;
static bool csf_rig1=false;
static bool csf_rig2=false;
static bool csf_rig3=true;
static int MaxIteration = 500;
static double cloth_resolution = 2;
static double class_threshold = 0.5;

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

	//Convert CC point cloud to CSF type
	unsigned count = pc->size();
	PointCloud csfPC;
	try
	{
		csfPC.reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	for (int i = 0; i < count; i++)
	{
		const CCVector3* P = pc->getPoint(i);
		wl:LASPoint tmpPoint;
		//tmpPoint.x = P->x;
		//tmpPoint.y = P->y;
		//tmpPoint.z = P->z;
		tmpPoint.x =  P->x;
		tmpPoint.y = -P->z;
		tmpPoint.z =  P->y;
		csfPC.push_back(tmpPoint);
	}

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
	CSF csf(csfPC);

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
	std::vector< vector<int> > segIndex;
	if (!csf.do_filtering(count, segIndex))
	{
		m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//extract ground subset
	ccPointCloud* groundpoint = 0;
	{
		CCLib::ReferenceCloud groundpc(pc);
		if (groundpc.reserve(static_cast<unsigned>(segIndex[1].size())))
		{
			for (unsigned j = 0; j < segIndex[1].size(); ++j)
			{
				groundpc.addPointIndex(segIndex[1][j]);
			}
			groundpoint = pc->partialClone(&groundpc);
		}
	}
	if (!groundpoint)
	{
		m_app->dispToConsole("Failed to extract the ground subset (not enough memory)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	}

	//extract off-ground subset
	ccPointCloud* offgroundpoint = 0;
	{
		CCLib::ReferenceCloud offgroundpc(pc);
		if (offgroundpc.reserve(static_cast<unsigned>(segIndex[2].size())))
		{
			for (unsigned k = 0; k < segIndex[2].size(); ++k)
			{
				offgroundpc.addPointIndex(segIndex[2][k]);
			}
			offgroundpoint = pc->partialClone(&offgroundpc);
		}
	}
	if (!offgroundpoint)
	{
		m_app->dispToConsole("Failed to extract the off-ground subset (not enough memory)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		if (!groundpoint)
		{
			//nothing to do!
			return;
		}
	}

	pDlg.hide();
	QApplication::processEvents();
	
	//hide the original cloud
	pc->setEnabled(false);

	//we add new group to DB/display
	ccHObject* cloudContainer = new ccHObject(pc->getName() + QString("_csf"));
	if (groundpoint)
	{
		groundpoint->setVisible(true);
		groundpoint->setName("ground points");
		cloudContainer->addChild(groundpoint);
	}

	if (offgroundpoint)
	{
		offgroundpoint->setVisible(true);
		offgroundpoint->setName("off-ground points");
		cloudContainer->addChild(offgroundpoint);
	}

	m_app->addToDB(cloudContainer);
	m_app->refreshAll();
}

QIcon qCSF::getIcon() const
{
	return QIcon(":/CC/plugin/qCSF/icon.png");
}
