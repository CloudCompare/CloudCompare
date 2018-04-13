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
#include "qHoughNormalsDialog.h"

//Hough Normals library
#include "normals_Hough/Normals.h"

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>
#include <QProgressDialog>

//system
#include <assert.h>

qHoughNormals::qHoughNormals(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qHoughNormals/info.json" )
	, m_action( nullptr ){
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

QList<QAction *> qHoughNormals::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, &QAction::triggered, this, &qHoughNormals::doAction);
	}

	return QList<QAction *>{ m_action };
}

//persistent settings during a single session
qHoughNormalsDialog::Parameters s_params;

void qHoughNormals::doAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	if (!m_app->haveSelection())
	{
		assert(false);
		return;
	}

	qHoughNormalsDialog dlg(m_app->getMainWindow());
	if (!dlg.exec())
	{
		//cancelled
		return;
	}

	try
	{
		for (ccHObject* entity : m_app->getSelectedEntities())
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
			ne.density_sensitive() = s_params.use_density;
			ne.get_n_phi() = s_params.n_phi;
			ne.get_n_rot() = s_params.n_rot;
			ne.get_tol_angle_rad() = s_params.tol_angle_rad;
			ne.get_K_density() = s_params.k_density;

			int maxProgress = ne.maxProgressCounter();
			int stepProgress = std::max(1, maxProgress / 100);
			QProgressDialog pDlg("Computing normals...", QString(), 0, maxProgress, m_app->getMainWindow());
			pDlg.show();
			QCoreApplication::processEvents();
			
			std::function<void(int)> progressLambda =
				[&](int value)
				{
					if ((value % stepProgress) == 0)
					{
						QMetaObject::invokeMethod(&pDlg, "setValue", Qt::QueuedConnection, Q_ARG(int, value));
						QCoreApplication::processEvents();
					}
				};
			ne.setProgressCallback(progressLambda);

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
