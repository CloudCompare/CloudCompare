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
#include "StatisticalOutliersRemoverDlg.h"
#include "ccPointCloud.h"
#include "cc2sm.h"
#include "sm2cc.h"
#include "filtering.h"

#include <boost/make_shared.hpp>

StatisticalOutliersRemover::StatisticalOutliersRemover()
    : BaseFilter(FilterDescription("Statistical Outliers Remover",
                                   "Remove Outliers Using statistical Approach",
                                   "Remove Outliers out of a given distance from the point, expressed as sigma of mean distances",
                                   ":/toolbar/PclUtils/icons/sor_outlier_remover.png",
                                   true)),
      m_dialog(0)
{
}

int StatisticalOutliersRemover::compute()
{

    //get selected as pointcloud
    ccPointCloud * cloud = this->getSelectedEntityAsCCPointCloud();
    sensor_msgs::PointCloud2::Ptr  tmp_cloud (new sensor_msgs::PointCloud2);

    //now as sensor message
    cc2smReader converter;
    converter.setInputCloud(cloud);
    converter.getAsSM(*tmp_cloud);

    sensor_msgs::PointCloud2Ptr outcloud ( new sensor_msgs::PointCloud2);
    removeOutliersStatistical(tmp_cloud, m_k, m_std, outcloud);

    //get back outcloud as a ccPointCloud
    sm2ccReader converterBack = sm2ccReader();
    converterBack.setInputCloud(outcloud);
    ccPointCloud * final_cloud = new ccPointCloud;

    if(!converterBack.getAsCC(final_cloud))
        return 0;

    //create a suitable name for the entity
    final_cloud->setName(QString("%1_k%2_std%3").arg(cloud->getName()).arg(m_k).arg(m_std));

    ccHObject* cloud_container = new ccHObject();
    QString container_name = "Statistical Outlier Remover Output";
    cloud_container->setName(qPrintable(container_name));
    cloud_container->addChild(final_cloud);

    //also disable original cloud
    cloud->setEnabled(false);
    emit newEntity(cloud_container);
    return 1;
}


int StatisticalOutliersRemover::openDialog()
{
    if (!m_dialog)
    {
        m_dialog = new ComputeSPINImages;
    }

    return m_dialog->exec() ? 1 : 0;
}

void StatisticalOutliersRemover::getParametersFromDialog()
{
    //get values from dialog
    m_k = m_dialog->spinK->value();
    m_std = m_dialog->spinStd->value();
}
