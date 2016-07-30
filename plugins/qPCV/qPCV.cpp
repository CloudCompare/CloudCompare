//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCV                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qPCV.h"
#include "ccPcvDlg.h"

//CCLib
#include <ScalarField.h>
#include <PCV.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>

#include <QtGui>
#include <QMainWindow>

#ifndef CC_PCV_FIELD_LABEL_NAME
#define CC_PCV_FIELD_LABEL_NAME "Illuminance (PCV)"
#endif

qPCV::qPCV(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qPCV::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1);
}

void qPCV::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

//persistent settings during a single session
static bool s_firstLaunch				= true;
static int s_raysSpinBoxValue			= 256;
static int s_resSpinBoxValue			= 1024;
static bool s_mode180CheckBoxState		= true;
static bool s_closedMeshCheckBoxState	= false;

void qPCV::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Select only one cloud or one mesh!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* ent = selectedEntities[0];

	ccGenericPointCloud* cloud = NULL;
	ccGenericMesh* mesh = NULL;
	if (ent->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		cloud = ccHObjectCaster::ToGenericPointCloud(ent);
	}
	else if (ent->isKindOf(CC_TYPES::MESH))
	{
		mesh = static_cast<ccGenericMesh*>(ent);
		cloud = mesh->getAssociatedCloud();
	}
	else
	{
		m_app->dispToConsole("Select a point cloud or a mesh!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (!cloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
	{
		m_app->dispToConsole("Select a real point cloud (or a mesh associated to a real point cloud)!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

	ccPcvDlg dlg(m_app->getMainWindow());

	//restore previous dialog state
	if (!s_firstLaunch)
	{
		dlg.raysSpinBox->setValue(s_raysSpinBoxValue);
		dlg.mode180CheckBox->setChecked(s_mode180CheckBoxState);
		dlg.resSpinBox->setValue(s_resSpinBoxValue);
		dlg.closedMeshCheckBox->setChecked(s_closedMeshCheckBoxState);
	}

	//for meshes only
	if (!mesh)
		dlg.closedMeshCheckBox->setEnabled(false);

	//for using clouds normals as rays
	std::vector<ccGenericPointCloud*> cloudsWithNormals;
	ccHObject* root = m_app->dbRootObject();
	if (root)
	{
		ccHObject::Container clouds;
		root->filterChildren(clouds,true,CC_TYPES::POINT_CLOUD);
		for (size_t i=0; i<clouds.size(); ++i)
		{
			//we keep only clouds with normals
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(clouds[i]);
			if (cloud && cloud->hasNormals())
			{
				cloudsWithNormals.push_back(cloud);
				QString cloudTitle = QString("%1 - %2 points").arg(cloud->getName()).arg(cloud->size());
				if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
					cloudTitle.append(QString(" (%1)").arg(cloud->getParent()->getName()));

				dlg.cloudsComboBox->addItem(cloudTitle);
			}
		}
	}
	if (cloudsWithNormals.empty())
		dlg.useCloudRadioButton->setEnabled(false);

	if (!dlg.exec())
		return;

	//save dialog state
	{
		s_firstLaunch				= false;
		s_raysSpinBoxValue			= dlg.raysSpinBox->value();
		s_mode180CheckBoxState		= dlg.mode180CheckBox->isChecked();
		s_resSpinBoxValue			= dlg.resSpinBox->value();
		s_closedMeshCheckBoxState	= dlg.closedMeshCheckBox->isChecked();
	}

	//we get the PCV field if it already exists
	int sfIdx = pc->getScalarFieldIndexByName(CC_PCV_FIELD_LABEL_NAME);
	//otherwise we create it
	if (sfIdx < 0)
		sfIdx = pc->addScalarField(CC_PCV_FIELD_LABEL_NAME);
	if (sfIdx < 0)
	{
		m_app->dispToConsole("Couldn't allocate a new scalar field for computing PCV field ! Try to free some memory ...",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	pc->setCurrentScalarField(sfIdx);

	unsigned raysNumber = dlg.raysSpinBox->value();
	unsigned res = dlg.resSpinBox->value();
	bool meshIsClosed = (mesh ? dlg.closedMeshCheckBox->checkState()==Qt::Checked : false);
	bool mode360 = !dlg.mode180CheckBox->isChecked();

	//progress dialog
	ccProgressDialog progressCb(true,m_app->getMainWindow());

	//PCV type ShadeVis
	bool success = false;
	if (!cloudsWithNormals.empty() && dlg.useCloudRadioButton->isChecked())
	{
		//Version with cloud normals as light rays
		assert(dlg.cloudsComboBox->currentIndex() < (int)cloudsWithNormals.size());
		ccGenericPointCloud* pc = cloudsWithNormals[dlg.cloudsComboBox->currentIndex()];
		std::vector<CCVector3> rays;
		unsigned count = pc->size();
		rays.resize(count);
		for (unsigned i=0; i<count; ++i)
			rays[i] = CCVector3(pc->getPointNormal(i));

		success = PCV::Launch(rays,cloud,mesh,meshIsClosed,res,res,&progressCb);
	}
	else
	{
		//Version with rays sampled on a sphere
		success = (PCV::Launch(raysNumber,cloud,mesh,meshIsClosed,mode360,res,res,&progressCb) > 0);
	}

	if (!success)
	{
		pc->deleteScalarField(sfIdx);
		m_app->dispToConsole("En error occurred during the PCV field computation!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	else
	{
		pc->getCurrentInScalarField()->computeMinAndMax();
		pc->setCurrentDisplayedScalarField(sfIdx);
		ccScalarField* sf = static_cast<ccScalarField*>(pc->getScalarField(sfIdx));
		if (sf)
			sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
		ent->showSF(true);
		ent->showNormals(false);
		ent->prepareDisplayForRefresh_recursive();
	}

	//currently selected entities parameters may have changed!
	m_app->updateUI();
	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}

QIcon qPCV::getIcon() const
{
	return QIcon(QString::fromUtf8(":/CC/plugin/qPCV/cc_ShadeVisIcon.png"));
}
