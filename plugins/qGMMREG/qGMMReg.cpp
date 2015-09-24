//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qGMMReg                       #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qGMMReg.h"

//Qt
#include <QtGui>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//GMMReg
#include <gmmreg_tps.h>

//Default constructor: should mainly be used to initialize
//actions (pointers) and other members
qGMMRegPlugin::qGMMRegPlugin(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qGMMRegPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size() == 2);
}

void qGMMRegPlugin::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect appropriate signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

static bool CloudToVNLMatrix(const ccGenericPointCloud* cloud, vnl_matrix<double>& matrix)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	unsigned pointCount = cloud->size();
	try
	{
		matrix.set_size(pointCount,3);
	}
	catch (...)
	{
		//not enough memory?
		return false;
	}

	for (unsigned i=0; i<pointCount; ++i)
	{
		const CCVector3* P = const_cast<ccGenericPointCloud*>(cloud)->getPoint(i);
		matrix(i,0) = P->x;
		matrix(i,1) = P->y;
		matrix(i,2) = P->z;
	}

	return true;
}

void qGMMRegPlugin::doAction()
{
	//m_app should have already been initialized by CC when plugin is loaded!
	//(--> pure internal check)
	assert(m_app);
	if (!m_app)
		return;

	/*** HERE STARTS THE ACTION ***/
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	bool validSelection = true;
	if (selectedEntities.size() == 2)
	{
		for (size_t i = 0; i < selectedEntities.size(); ++i)
		{
			if (	!selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD)
				&&	!selectedEntities[i]->isKindOf(CC_TYPES::MESH) )
			{
				validSelection = false;
				break;
			}
		}
	}
	else
	{
		validSelection = false;
	}
	
	if (!validSelection)
	{
		assert(false);
		if (m_app)
			m_app->dispToConsole("Select two entities (clouds or meshes)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* data = selectedEntities[0];
	ccHObject* model = selectedEntities[1];

	//TODO: ask the user which one is the 'data' entity and which one is the 'model' entity
	if (model->isKindOf(CC_TYPES::MESH))
	{
		//for now we always take the mesh as 'data'
		std::swap(data, model);
	}

	//we can now prepare GMMReg
	//gmmreg::TpsRegistration_L2 reg;
	gmmreg::TpsRegistration_KC reg;
	{
		vnl_matrix<double> dataPts;
		{
			const ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(data);
			if (!CloudToVNLMatrix(cloud, dataPts))
			{
				if (m_app)
					m_app->dispToConsole("Failed to convert data entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}

		vnl_matrix<double> modelPts;
		{
			const ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(model);
			if (!CloudToVNLMatrix(cloud, modelPts))
			{
				if (m_app)
					m_app->dispToConsole("Failed to convert model entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
  
		if (reg.Run(dataPts, modelPts, 1) < 0)
		{
			if (m_app)
				m_app->dispToConsole("GMM registration failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}
	
	const vnl_matrix<double>& outputMatrix = reg.transformedModel();

	ccHObject* clonedData = 0;
	if (data->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		clonedData = static_cast<ccGenericPointCloud*>(data)->clone();
	}
	else if (data->isKindOf(CC_TYPES::MESH))
	{
		if (data->isA(CC_TYPES::MESH))
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(data);
			clonedData = mesh->cloneMesh();
		}
		else
		{
			if (m_app)
				m_app->dispToConsole("Can't duplicate sub-meshes (vertices will be updated directly!)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			clonedData = data;
		}
	}

	if (!clonedData)
	{
		if (m_app)
			m_app->dispToConsole("Can't duplicate the input entity (not enough memory?). Vertices will be updated directly!)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		clonedData = data;
	}

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(clonedData);
	if (cloud && cloud->size() == outputMatrix.rows())
	{
		for (unsigned i = 0; i < outputMatrix.rows(); ++i)
		{
			CCVector3* P = const_cast<CCVector3*>(cloud->getPoint(i));
			P->x = static_cast<PointCoordinateType>(outputMatrix(i, 0));
			P->y = static_cast<PointCoordinateType>(outputMatrix(i, 1));
			P->z = static_cast<PointCoordinateType>(outputMatrix(i, 2));
		}
		if (cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			static_cast<ccPointCloud*>(cloud)->invalidateBoundingBox();
		}
		clonedData->redrawDisplay();

		if (clonedData != data)
		{
			data->setEnabled(false);
			m_app->addToDB(clonedData);
		}
	}
	else
	{
		if (clonedData != data)
			delete clonedData;

		assert(false);
	}

	/*** HERE ENDS THE ACTION ***/
}

//This method should return the plugin icon (it will be used mainly
//if your plugin as several actions in which case CC will create a
//dedicated sub-menu entry with this icon.
QIcon qGMMRegPlugin::getIcon() const
{
	return QIcon(":/CC/plugin/qGMMRegPlugin/icon.png");
}

#ifndef CC_QT5
Q_EXPORT_PLUGIN2(qGMMRegPlugin,qGMMRegPlugin);
#endif
