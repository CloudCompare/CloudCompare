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

void removeOutliersStatistical(const PCLCloud::ConstPtr incloud, int knn, double nSigma, PCLCloud::Ptr outcloud)
{
	pcl::StatisticalOutlierRemoval<PCLCloud> remover;
	remover.setInputCloud(incloud);
	remover.setMeanK(knn);
	remover.setStddevMulThresh(nSigma);
	remover.filter(*outcloud);
}

StatisticalOutliersRemover::StatisticalOutliersRemover()
	: BaseFilter(FilterDescription("Statistical Outlier Removal",
									"Filter outlier data based on point neighborhood statistics",
									"Filter the points that are farther of their neighbors than the average (plus a number of times the standard deviation)",
									":/toolbar/PclUtils/icons/sor_outlier_remover.png"))
	, m_dialog(nullptr)
	, m_k(0)
	, m_std(0.0f)
{
}

StatisticalOutliersRemover::~StatisticalOutliersRemover()
{
	//we must delete parent-less dialogs ourselves!
	if (!m_dialogHasParent && m_dialog && m_dialog->parent() == nullptr)
		delete m_dialog;
}

int StatisticalOutliersRemover::compute()
{
	//get selected as pointcloud
	ccPointCloud* cloud = this->getSelectedEntityAsCCPointCloud();
	if (!cloud)
		return -1;

	//now as sensor message
	PCLCloud::Ptr tmp_cloud = cc2smReader(cloud).getAsSM();
	if (!tmp_cloud)
		return -1;

	PCLCloud::Ptr outcloud ( new PCLCloud);
	removeOutliersStatistical(tmp_cloud, m_k, m_std, outcloud);

	//get back outcloud as a ccPointCloud
	ccPointCloud* final_cloud = sm2ccConverter(outcloud).getCloud();
	if (!final_cloud)
		return -1;

	//create a suitable name for the entity
	final_cloud->setName(QString("%1_k%2_std%3").arg(cloud->getName()).arg(m_k).arg(m_std));
	final_cloud->setDisplay(cloud->getDisplay());
	//copy global shift & scale
	final_cloud->setGlobalScale(cloud->getGlobalScale());
	final_cloud->setGlobalShift(cloud->getGlobalShift());

	//disable original cloud
	cloud->setEnabled(false);
	if (cloud->getParent())
		cloud->getParent()->addChild(final_cloud);

	emit newEntity(cloud);

	return 1;
}

int StatisticalOutliersRemover::openInputDialog()
{
	if (!m_dialog)
	{
		m_dialog = new SORDialog(m_app ? m_app->getMainWindow() : nullptr);
		m_dialogHasParent = (m_dialog->parent() != nullptr);
	}

	return m_dialog->exec() ? 1 : 0;
}

void StatisticalOutliersRemover::getParametersFromDialog()
{
	//get values from dialog
	if (m_dialog)
	{
		m_k = m_dialog->spinK->value();
		m_std = static_cast<float>(m_dialog->spinStd->value());
	}
}
