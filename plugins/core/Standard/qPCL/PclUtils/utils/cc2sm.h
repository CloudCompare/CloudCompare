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
#ifndef Q_PCL_PLUGIN_CC2SM_H
#define Q_PCL_PLUGIN_CC2SM_H

//Local
#include "PCLCloud.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Qt
#include <QStringList>

//system
#include <set>
#include <string>

class ccPointCloud;

//! CC to PCL cloud converter
class cc2smReader
{
public:
	explicit cc2smReader(ccPointCloud* ccCloud);

	//! Converts the ccPointCloud to a pcl::PointCloud2 cloud
	/** This is useful for saving a ccPointCloud into a PCD file.
		For pcl filters other methods are suggested (to get only the necessary bits of data)
	**/
	PCLCloud::Ptr getAsSM() const;
	PCLCloud::Ptr getAsSM(bool xyz, bool normals, bool rgbColors, const QStringList& scalarFields) const;

	//! Converts the ccPointCloud to a 'pcl::PointXYZ' cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr getRawXYZ() const;

	//! Converts the ccPointCloud to a 'pcl::PointNormal' cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr getAsPointNormal() const;

	static std::string GetSimplifiedSFName(const QString& ccSfName);

protected:

	PCLCloud::Ptr getXYZ() const;
	PCLCloud::Ptr getNormals() const;
	PCLCloud::Ptr getColors() const;
	PCLCloud::Ptr getFloatScalarField(const QString& sfName) const;

	//! Associated cloud
	const ccPointCloud* m_ccCloud;
};

#endif // Q_PCL_PLUGIN_CC2SM_H
