//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qTreeIso                          #
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
//#        Please cite the following paper if you find this tool helpful                #
//#                                                                                     #
//#        Xi, Z.; Hopkinson, C. 3D Graph-Based Individual-Tree Isolation (Treeiso)     #
//#        from Terrestrial Laser Scanning Point Clouds. Remote Sens. 2022, 14, 6116.   #
//#        https://doi.org/10.3390/rs14236116                                           #
//#                                                                                     #
//#		   Our work relies on the cut-pursuit algorithm, please also consider citing:   #
//#        Landrieu, L.; Obozinski, G. Cut Pursuit: Fast Algorithms to Learn Piecewise  #
//#        Constant Functions on General Weighted Graphs. SIAM J. Imaging Sci.          #
//#        2017, 10, 1724–1766.                                                         #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#                  Artemis Lab, Department of Geography & Environment                 #
//#                            University of Lethbridge, Canada                         #
//#                                                                                     #
//#                                                                                     #
//#                           Zhouxin Xi and Chris Hopkinson;                           #
//#                    truebelief2010@gmail.com; c.hopkinson@uleth.ca                   #
//#                                                                                     #
//#######################################################################################


#pragma once
// A Matlab version shared via:
// https://github.com/truebelief/artemis_treeiso

#include "qTreeIso.h"

//Qt
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>
#include <QComboBox>
#include <QElapsedTimer>

//Local
#include "qTreeIsoCommands.h"

//System
#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccOctree.h>
#include <ccMesh.h>


#include <QMessageBox>


qTreeIso::qTreeIso(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qTreeIso/info.json")
	, m_action(nullptr)
{
}	

void qTreeIso::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		bool hasCloud = false;
		for (ccHObject* entity : selectedEntities)
		{
			if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
			{
				hasCloud = true;
				break;
			}
		}
		m_action->setEnabled(hasCloud);
	}
	//m_action->setEnabled(true);
}

QList<QAction *> qTreeIso::getActions()
{
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());

		//connect appropriate signal
		connect(m_action, &QAction::triggered, this, &qTreeIso::doAction);
	}

	return { m_action };
}

bool qTreeIso::init() {	
	m_treeiso=new TreeIso();

	m_treeiso_dlg=new ccTreeIsoDlg(m_app->getMainWindow());
	connect(m_treeiso_dlg->pushButtonInitSeg, &QPushButton::clicked, this, &qTreeIso::init_segs);
	connect(m_treeiso_dlg->pushButtonInterSeg, &QPushButton::clicked, this, &qTreeIso::intermediate_segs);
	connect(m_treeiso_dlg->pushButtonReseg, &QPushButton::clicked, this, &qTreeIso::final_segs);
	m_treeiso_dlg->pushButtonInitSeg->setEnabled(true);
	return true;
}


void qTreeIso::doAction()
{
	// m_app should have already been initialized by CC when plugin is loaded!
	if (!m_app)
	{
		assert(false);
		return;
	}
	if (!init()) {
		return;
	}
	if (!m_treeiso_dlg->exec())
	{
		return;
	}
	m_app->refreshAll();
}


void qTreeIso::init_segs()
{
	m_progress_dlg = new QProgressDialog(m_app->getMainWindow());

	m_treeiso->setProgressDialog(m_progress_dlg);
	m_progress_dlg->setWindowTitle("TreeIso Step 1. Initial segmention");
	m_progress_dlg->setLabelText(tr("Computing...."));
	m_progress_dlg->setCancelButton(nullptr);
	m_progress_dlg->setRange(0, 0); // infinite progress bar
	m_progress_dlg->show();

	m_treeiso_parameters.min_nn1 = m_treeiso_dlg->spinBoxK1->value();
	m_treeiso_parameters.reg_strength1 = m_treeiso_dlg->doubleSpinBoxLambda1->value();;
	m_treeiso_parameters.decimate_res1 = m_treeiso_dlg->doubleSpinBoxDecRes1->value();

	if (!m_treeiso->init_seg(m_treeiso_parameters.min_nn1, m_treeiso_parameters.reg_strength1, m_treeiso_parameters.decimate_res1, m_app, nullptr))
	{
		m_app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	m_progress_dlg->hide();

	QApplication::processEvents();
	m_app->updateUI();
	m_app->refreshAll();

}



void qTreeIso::intermediate_segs()
{

	m_progress_dlg = new QProgressDialog(m_app->getMainWindow());
	m_treeiso->setProgressDialog(m_progress_dlg);
	// display the progress dialog
	m_progress_dlg->setWindowTitle("TreeIso Step 2. Interim segmention");
	m_progress_dlg->setLabelText(tr("Computing...."));
	m_progress_dlg->setCancelButton(nullptr);
	m_progress_dlg->setRange(0, 0); // infinite progress bar
	m_progress_dlg->show();
	
	m_treeiso_parameters.min_nn2 = m_treeiso_dlg->spinBoxK2->value();
	m_treeiso_parameters.reg_strength2 = m_treeiso_dlg->doubleSpinBoxLambda2->value();
	m_treeiso_parameters.decimate_res2 = m_treeiso_dlg->doubleSpinBoxDecRes2->value();
	m_treeiso_parameters.max_gap = m_treeiso_dlg->doubleSpinBoxMaxGap->value();

	if (!m_treeiso->intermediate_seg(m_treeiso_parameters.min_nn2, m_treeiso_parameters.reg_strength2, m_treeiso_parameters.decimate_res2, m_treeiso_parameters.max_gap, m_app, nullptr))
	{
		m_progress_dlg->hide();
		QApplication::processEvents();
		m_app->updateUI();
		m_app->refreshAll();
		return;
	}
	m_progress_dlg->hide();
	QApplication::processEvents();
	m_app->updateUI();
	m_app->refreshAll();

}


void qTreeIso::final_segs()
{
	m_progress_dlg = new QProgressDialog(m_app->getMainWindow());
	m_treeiso->setProgressDialog(m_progress_dlg);
	// display the progress dialog
	m_progress_dlg->setWindowTitle("TreeIso Step 3. Final segmention");
	m_progress_dlg->setLabelText(tr("Computing...."));
	m_progress_dlg->setCancelButton(nullptr);
	m_progress_dlg->setRange(0, 0); // infinite progress bar
	m_progress_dlg->show();

	m_treeiso_parameters.rel_height_length_ratio = m_treeiso_dlg->doubleSpinBoxRelHLRatio->value();
	m_treeiso_parameters.vertical_weight = m_treeiso_dlg->doubleSpinBoxVWeight->value();

	if (!m_treeiso->final_seg(m_treeiso_parameters.min_nn2, m_treeiso_parameters.rel_height_length_ratio, m_treeiso_parameters.vertical_weight, m_app, nullptr))
	{
		m_progress_dlg->hide();
		QApplication::processEvents();
		m_app->updateUI();
		m_app->refreshAll();
		return;
	}

	m_progress_dlg->hide();
	QApplication::processEvents();
	m_app->updateUI();
	m_app->refreshAll();

}

void qTreeIso::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandTreeIso));
}
