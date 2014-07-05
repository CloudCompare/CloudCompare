//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#include "StatisticalOutliersRemover.h"

//Local
#include "StatisticalOutliersRemoverDlg.h"
#include "ccPointCloud.h"
#include "cc2sm.h"
#include "sm2cc.h"
#include "filtering.h"

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QMainWindow>

//Boost
#include <boost/make_shared.hpp>

StatisticalOutliersRemover::StatisticalOutliersRemover()
	: BaseFilter(FilterDescription("Statistical Outliers Remover",
									"Remove Outliers Using statistical Approach",
									"Remove Outliers out of a given distance from the point, expressed as sigma of mean distances",
									":/toolbar/PclUtils/icons/sor_outlier_remover.png"))
	, m_dialog(0)
	, m_k(0)
	, m_std(0.0f)
{
}

StatisticalOutliersRemover::~StatisticalOutliersRemover()
{
	//we must delete parent-less dialogs ourselves!
	if (m_dialog && m_dialog->parent() == 0)
		delete m_dialog;
}

int StatisticalOutliersRemover::compute()
{
	//get selected as pointcloud
	ccPointCloud * cloud = this->getSelectedEntityAsCCPointCloud();
	PCLCloud::Ptr  tmp_cloud (new PCLCloud);

	//now as sensor message
	cc2smReader converter;
	converter.setInputCloud(cloud);
	converter.getAsSM(*tmp_cloud);

	PCLCloud::Ptr outcloud ( new PCLCloud);
	removeOutliersStatistical(tmp_cloud, m_k, m_std, outcloud);

	//get back outcloud as a ccPointCloud
	ccPointCloud* final_cloud = sm2ccConverter(outcloud).getCloud();
	if(!final_cloud)
		return -1;

	//create a suitable name for the entity
	final_cloud->setName(QString("%1_k%2_std%3").arg(cloud->getName()).arg(m_k).arg(m_std));
	final_cloud->setDisplay(cloud->getDisplay());

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
		m_dialog = new SORDialog(m_app ? m_app->getMainWindow() : 0);
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
