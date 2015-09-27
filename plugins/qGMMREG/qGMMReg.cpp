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

//qCC
#include <ccOrderChoiceDlg.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//GMMReg
#include <gmmreg_tps.h>

//Qt
#include <QtGui>
#include <QInputDialog>

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

static bool CloudToVNLMatrix(const ccGenericPointCloud* cloud, vnl_matrix<double>& matrix, const CCVector3& C)
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
		CCVector3 P = *const_cast<ccGenericPointCloud*>(cloud)->getPoint(i) - C;
		matrix(i,0) = P.x;
		matrix(i,1) = P.y;
		matrix(i,2) = P.z;
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

	if (model->isKindOf(CC_TYPES::MESH))
	{
		//by default we always set the mesh (if any) as 'data'
		std::swap(data, model);
	}

	//ask the user which one is the 'data' entity and which one is the 'model' entity
	ccOrderChoiceDlg ocDlg(data, "data (deformed)", model, "reference (rigid)", m_app);
	if (!ocDlg.exec())
	{
		return;
	}
	data = ocDlg.getFirstEntity();
	model = ocDlg.getSecondEntity();

	//we can now prepare GMMReg
	gmmreg::TpsRegistration_L2 reg;
	CCVector3 C(0,0,0); 
	//gmmreg::TpsRegistration_KC reg;
	{
		ccGenericPointCloud* d = ccHObjectCaster::ToGenericPointCloud(data);
		ccGenericPointCloud* m = ccHObjectCaster::ToGenericPointCloud(model);
		ccBBox box = d->getOwnBB();
		box += m->getOwnBB();
		C = box.getCenter();

		vnl_matrix<double> dataPts;
		{
			if (!CloudToVNLMatrix(d, dataPts, C))
			{
				if (m_app)
					m_app->dispToConsole("Failed to convert data entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}

		vnl_matrix<double> modelPts;
		{
			if (!CloudToVNLMatrix(m, modelPts, C))
			{
				if (m_app)
					m_app->dispToConsole("Failed to convert model entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}

		vnl_matrix<double> ctrlPts;
		bool useControlPoints = true;
		if (useControlPoints)
		{
			//ask for the control points number
			bool ok = false;
			int count = QInputDialog::getInt(m_app ? m_app->getMainWindow() : 0, "Control points", "Control points =", 500, 100, 1000000, 100, &ok);
			if (!ok)
				return;
			
			CCVector3 diag = box.getDiagVec();
			CCVector3 bbMin = box.minCorner();

			//try to get as close as possible with a regular grid
			PointCoordinateType l = pow((diag.x * diag.y * diag.z) / count, 1.0/3.0);
			unsigned nx = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.x / l)));
			unsigned ny = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.y / l)));
			unsigned nz = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.z / l)));
			count = nx * ny * nz;

			try
			{
				ctrlPts.set_size(count,3);
			}
			catch (...)
			{
				//not enough memory?
				if (m_app)
					m_app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}

			ccPointCloud* test = 0;
#ifdef EXPORT_CONTROL_POINTS
			test = new ccPointCloud("grid");
			test->reserve(count);
#endif
			unsigned n = 0;
			for (unsigned k=0; k<nz; ++k)
			{
				CCVector3 P(0,0,bbMin.z + k * diag.z / (nz-1));
				for (unsigned j=0; j<ny; ++j)
				{
					P.y = bbMin.y + j * diag.y / (ny-1);
					for (unsigned i=0; i<nx; ++i, ++n)
					{
						P.x = bbMin.x + i * diag.x / (nx-1);
						ctrlPts(n,0) = P.x;
						ctrlPts(n,1) = P.y;
						ctrlPts(n,2) = P.z;
#ifdef EXPORT_CONTROL_POINTS
						if (test)
							test->addPoint(P);
#endif
					}
				}
			}

#ifdef EXPORT_CONTROL_POINTS
			if (test)
			{
				if (m_app)
					m_app->addToDB(test);
				else
					delete test;
			}
#endif
		}
		
		//parameters
		unsigned level = 2; //<=3
		{
			bool ok = false;
			level = QInputDialog::getInt(m_app ? m_app->getMainWindow() : 0, "Steps", "steps =", 2, 1, 3, 100, &ok);
			if (!ok)
				return;
		}
		
		std::vector<float> v_scale(3);
		v_scale[0] = 0.5f;
		v_scale[1] = 0.1f;
		v_scale[2] = 0.02f;
		std::vector<int> v_func_evals(3);
		v_func_evals[0] = 50;
		v_func_evals[1] = 50;
		v_func_evals[2] = 100;
		std::vector<float> v_lambda(3);
		v_lambda[0] = 0.1;
		v_lambda[1] = 0;
		v_lambda[2] = 0;
		std::vector<int> v_affine(3);
		v_affine[0] = 0;
		v_affine[1] = 0;
		v_affine[2] = 0;

		//ask for the scale(s)
		for (unsigned i=0; i<level; ++i)
		{
			bool ok = false;
			double val = QInputDialog::getDouble(m_app ? m_app->getMainWindow() : 0, "Scale", QString("scale(%1) =").arg(i), v_scale[i], 1.0e-3, 10.0, 3, &ok);
			if (!ok)
				return;
			v_scale[i] = static_cast<float>(val);

			int eval = QInputDialog::getInt(m_app ? m_app->getMainWindow() : 0, "Eval. count", QString("eval(%1) =").arg(i), v_func_evals[i], 10, 1000, 100, &ok);
			if (!ok)
				return;
			v_func_evals[i] = eval;
		}
		
		//multi-scale options
		if (reg.MultiScaleOptions(level, v_scale, v_func_evals) < 0)
		{
			if (m_app)
				m_app->dispToConsole("Mutli-scale options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		//TPS options
		if (reg.PrepareOwnOptions(v_lambda, v_affine) < 0)
		{
			if (m_app)
				m_app->dispToConsole("TPS options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
  
		//initialization
		if (reg.Initialize(dataPts, modelPts, useControlPoints ? &ctrlPts : 0, 0) < 0)
		{
			if (m_app)
				m_app->dispToConsole("GMM initialization failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
		
		try
		{
			//eventually we can run the registration
			if (reg.Run() < 0)
			{
				if (m_app)
					m_app->dispToConsole("GMM registration failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
		catch (...)
		{
			if (m_app)
				m_app->dispToConsole("Unknown exception caught", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
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
			P->x = static_cast<PointCoordinateType>(outputMatrix(i, 0)) + C.x;
			P->y = static_cast<PointCoordinateType>(outputMatrix(i, 1)) + C.y;
			P->z = static_cast<PointCoordinateType>(outputMatrix(i, 2)) + C.z;
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
