//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper, If you use this plugin in your work.        #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

// A mex version for programming in Matlab is at File Exchange of Mathworks website:
// http://www.mathworks.com/matlabcentral/fileexchange/58139-csf--ground-filtering-of-point-cloud-based-on-cloth-simulation

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

qCSF::qCSF(QObject* parent)
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qCSF/info.json" )
	, m_action( nullptr )
{
}	

void qCSF::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

QList<QAction *> qCSF::getActions()
{
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect appropriate signal
		connect(m_action, &QAction::triggered, this, &qCSF::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qCSF::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

	if ( !m_app->haveOneSelection() )
	{
		m_app->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

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
	pDlg.setCancelButton(nullptr);
	pDlg.show();
	QApplication::processEvents();

	QElapsedTimer timer;
	timer.start();
	//instantiation a CSF class
	CSF csf(csfPC);

	// setup parameter
	csf.params.k_nearest_points = 1;
	csf.params.bSloopSmooth = csf_postprocessing;
	csf.params.time_step = 0.65;
	csf.params.class_threshold = class_threshold;
	csf.params.cloth_resolution = cloth_resolution;
	csf.params.rigidness = csf_rigidness;
	csf.params.iterations = MaxIteration;
	//to do filtering
	std::vector<int> groundIndexes;
	std::vector<int> offGroundIndexes;
	ccMesh* clothMesh = nullptr;
	if (!csf.do_filtering(groundIndexes, offGroundIndexes, ExportClothMesh, clothMesh, m_app))
	{
		m_app->dispToConsole("Process failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	m_app->dispToConsole(QString("[CSF] %1% of points classified as ground points").arg((groundIndexes.size() * 100.0) / count, 0, 'f', 2), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	m_app->dispToConsole(QString("[CSF] Timing: %1 s.").arg(timer.elapsed() / 1000.0, 0, 'f', 1), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//extract ground subset
	ccPointCloud* groundpoint = nullptr;
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
	ccPointCloud* offgroundpoint = nullptr;
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
