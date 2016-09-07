//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qHoughNormals                   #
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

#include "qHoughNormals.h"

//
#include "normals_Hough/Normals.h"

//qCC_db
#include <ccPointCloud.h>
//#include <ccHObjectCaster.h>
//#include <ccGenericPointCloud.h>
//#include <ccPointCloud.h>
//#include <ccGenericMesh.h>
//#include <ccProgressDialog.h>
//#include <ccScalarField.h>
//#include <ccColorScalesManager.h>

//Qt
#include <QtGui>
#include <QMainWindow>

//system
#include <assert.h>

qHoughNormals::qHoughNormals(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qHoughNormals::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		for (ccHObject* entity : selectedEntities)
		{
			//if we have found at least one cloud
			if (entity && entity->isA(CC_TYPES::POINT_CLOUD))
			{
				m_action->setEnabled(true);
				return;
			}
		}

		//no cloud?
		m_action->setEnabled(false);
	}
}

void qHoughNormals::getActions(QActionGroup& group)
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
struct Parameters
{
	int K = 100;
	int T = 1000;
	int n_phi = 15;
	int n_rot = 5;
	bool ua = false;
	float tol_angle_rad = 0.79f;
	int k_density = 5;
};
static Parameters s_params;

void qHoughNormals::doAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		assert(false);
		return;
	}

	try
	{
		for (ccHObject* entity : selectedEntities)
		{
			if (!entity || !entity->isA(CC_TYPES::POINT_CLOUD))
			{
				continue;
			}

			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);

			size_t pointCount = cloud->size();
			Eigen::MatrixX3d pc;
			pc.resize(pointCount, 3);
			for (size_t i = 0; i < pointCount; ++i)
			{
				const CCVector3* P = cloud->getPoint(static_cast<unsigned>(i));
				pc.row(i) = Eigen::Vector3d(P->x, P->y, P->z);
			}

			//Create estimator
			Eigen::MatrixX3d normals;
			Eigen_Normal_Estimator ne(pc, normals);
			ne.get_K() = s_params.K;
			ne.get_T() = s_params.T;
			ne.density_sensitive() = s_params.ua;
			ne.get_n_phi() = s_params.n_phi;
			ne.get_n_rot() = s_params.n_rot;
			ne.get_tol_angle_rad() = s_params.tol_angle_rad;
			ne.get_K_density() = s_params.k_density;

			//Estimate
			ne.estimate_normals();

			if (!cloud->resizeTheNormsTable())
			{
				ccLog::Error("Not enough memory");
				break;
			}

			for (size_t i = 0; i < pointCount; ++i)
			{
				const Eigen::Vector3d& n = normals.row(i);
				CCVector3 N(static_cast<PointCoordinateType>(n.x()),
							static_cast<PointCoordinateType>(n.y()),
							static_cast<PointCoordinateType>(n.z()));
				cloud->setPointNormal(static_cast<unsigned>(i), N);
			}

			cloud->showNormals(true);
			cloud->prepareDisplayForRefresh_recursive();
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory");
	}

	//currently selected entities parameters may have changed!
	m_app->updateUI();
	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}

QIcon qHoughNormals::getIcon() const
{
	return QIcon(QString::fromUtf8(":/CC/plugin/qHoughNormals/normal.png"));
}
