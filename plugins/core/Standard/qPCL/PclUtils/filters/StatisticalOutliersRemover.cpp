//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "StatisticalOutliersRemover.h"

//Local
#include "dialogs/StatisticalOutliersRemoverDlg.h"
#include "../utils/cc2sm.h"
#include "../utils/sm2cc.h"

//PCL
#include <pcl/filters/statistical_outlier_removal.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>

//Boost
#include <boost/make_shared.hpp>

StatisticalOutliersRemover::StatisticalOutliersRemover()
	: BaseFilter(FilterDescription("Statistical Outlier Removal",
									"Filter outlier data based on point neighborhood statistics",
									"Filter the points that are farther of their neighbors than the average (plus a number of times the standard deviation)",
									":/toolbar/PclUtils/icons/sor_outlier_remover.png"))
	, m_kNN(0)
	, m_std(0.0f)
{
}

StatisticalOutliersRemover::~StatisticalOutliersRemover()
{
}

int StatisticalOutliersRemover::compute()
{
	//get selected as pointcloud
	ccPointCloud* cloud = getFirstSelectedEntityAsCCPointCloud();
	if (!cloud)
	{
		return InvalidInput;
	}

	//now as sensor message
	PCLCloud::Ptr tmp_cloud = cc2smReader(cloud).getAsSM();
	if (!tmp_cloud)
	{
		return ComputationError;
	}

	PCLCloud outcloud;

	pcl::StatisticalOutlierRemoval<PCLCloud> remover;
	remover.setInputCloud(tmp_cloud);
	remover.setMeanK(m_kNN);
	remover.setStddevMulThresh(m_std);
	remover.filter(outcloud);

	//get back outcloud as a ccPointCloud
	ccPointCloud* final_cloud = pcl2cc::Convert(outcloud);
	if (!final_cloud)
	{
		return ComputationError;
	}

	//create a suitable name for the entity
	final_cloud->setName(QString("%1_k%2_std%3").arg(cloud->getName()).arg(m_kNN).arg(m_std));
	final_cloud->setDisplay(cloud->getDisplay());
	//copy global shift & scale
	final_cloud->copyGlobalShiftAndScale(*cloud);

	//disable original cloud
	cloud->setEnabled(false);
	if (cloud->getParent())
	{
		cloud->getParent()->addChild(final_cloud);
	}

	Q_EMIT newEntity(final_cloud);

	return Success;
}

int StatisticalOutliersRemover::getParametersFromDialog()
{
	SORDialog dialog(m_app ? m_app->getMainWindow() : nullptr);
	if (!dialog.exec())
	{
		return CancelledByUser;
	}

	//get values from dialog
	m_kNN = dialog.spinK->value();
	m_std = static_cast<float>(dialog.spinStd->value());

	return Success;
}
