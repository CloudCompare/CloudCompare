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
#include <QElapsedTimer>

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
#include <ccMesh.h>

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

void qCSF::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//to get the point cloud from selected entity.
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	//Convert CC point cloud to CSF type
	unsigned count = pc->size();
	wl::PointCloud csfPC;
	try
	{
		csfPC.reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	for (unsigned i = 0; i < count; i++)
	{
		const CCVector3* P = pc->getPoint(i);
		wl::Point tmpPoint;
		//tmpPoint.x = P->x;
		//tmpPoint.y = P->y;
		//tmpPoint.z = P->z;
		tmpPoint.x =  P->x;
		tmpPoint.y = -P->z;
		tmpPoint.z =  P->y;
		csfPC.push_back(tmpPoint);
	}

	//initial dialog parameters
	static bool csf_postprocessing = false;
	static double cloth_resolution = 2;
	static double class_threshold = 0.5;
	static int csf_rigidness = 2;
	static int MaxIteration = 500;
	static bool ExportClothMesh = false;

	// display the dialog
	{
		ccCSFDlg csfDlg(m_app->getMainWindow());
		csfDlg.postprocessingcheckbox->setChecked(csf_postprocessing);
		csfDlg.rig1->setChecked(csf_rigidness == 1);
		csfDlg.rig2->setChecked(csf_rigidness == 2);
		csfDlg.rig3->setChecked(csf_rigidness == 3);
		csfDlg.MaxIterationSpinBox->setValue(MaxIteration);
		csfDlg.cloth_resolutionSpinBox->setValue(cloth_resolution);
		csfDlg.class_thresholdSpinBox->setValue(class_threshold);
		csfDlg.exportClothMeshCheckBox->setChecked(ExportClothMesh);

		if (!csfDlg.exec())
		{
			return;
		}

		//save the parameters for next time
		csf_postprocessing = csfDlg.postprocessingcheckbox->isChecked();
		if (csfDlg.rig1->isChecked())
			csf_rigidness = 1;
		else if (csfDlg.rig2->isChecked())
			csf_rigidness = 2;
		else
			csf_rigidness = 3;
		MaxIteration = csfDlg.MaxIterationSpinBox->value();
		cloth_resolution = csfDlg.cloth_resolutionSpinBox->value();
		class_threshold = csfDlg.class_thresholdSpinBox->value();
		ExportClothMesh = csfDlg.exportClothMeshCheckBox->isChecked();
	}

	//display the progress dialog
	QProgressDialog pDlg;
	pDlg.setWindowTitle("CSF");
	pDlg.setLabelText("Computing....");
	pDlg.setCancelButton(0);
	pDlg.show();
	QApplication::processEvents();

	QElapsedTimer timer;
	timer.start();

	//instantiation a CSF class
	CSF csf(csfPC);

	// setup parameter
	csf.params.k_nearest_points = 1;
	csf.params.bSloopSmooth = csf_postprocessing;
	csf.params.time_step = 0.75;
	csf.params.class_threshold = class_threshold;
	csf.params.cloth_resolution = cloth_resolution;
	csf.params.rigidness = csf_rigidness;
	csf.params.iterations = MaxIteration;

	//to do filtering
	std::vector<int> groundIndexes, offGroundIndexes;
	ccMesh* clothMesh = 0;
	if (!csf.do_filtering(groundIndexes, offGroundIndexes, ExportClothMesh, clothMesh, m_app))
	{
		m_app->dispToConsole("Process failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	m_app->dispToConsole(QString("[CSF] %1% of points classified as ground points").arg((groundIndexes.size() * 100.0) / count, 0, 'f', 2), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	m_app->dispToConsole(QString("[CSF] Timing: %1 s.").arg(timer.elapsed() / 1000.0, 0, 'f', 1), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//extract ground subset
	ccPointCloud* groundpoint = 0;
	{
		CCLib::ReferenceCloud groundpc(pc);
		if (groundpc.reserve(static_cast<unsigned>(groundIndexes.size())))
		{
			for (unsigned j = 0; j < groundIndexes.size(); ++j)
			{
				groundpc.addPointIndex(groundIndexes[j]);
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
		if (offgroundpc.reserve(static_cast<unsigned>(offGroundIndexes.size())))
		{
			for (unsigned k = 0; k < offGroundIndexes.size(); ++k)
			{
				offgroundpc.addPointIndex(offGroundIndexes[k]);
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

	if (clothMesh)
	{
		clothMesh->computePerVertexNormals();
		clothMesh->showNormals(true);
		cloudContainer->addChild(clothMesh);
	}

	m_app->addToDB(cloudContainer);
	m_app->refreshAll();
}

QIcon qCSF::getIcon() const
{
	return QIcon(":/CC/plugin/qCSF/icon.png");
}
