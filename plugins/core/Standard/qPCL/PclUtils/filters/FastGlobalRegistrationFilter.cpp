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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "FastGlobalRegistrationFilter.h"
#include "FastGlobalRegistration.h"
#include "FastGlobalRegistrationDlg.h"

//Local
#include "../utils/cc2sm.h"
#include "../utils/sm2cc.h"

//PCL
#include <pcl/features/fpfh_omp.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QMainWindow>

//Boost
#include <boost/make_shared.hpp>

FastGlobalRegistrationFilter::FastGlobalRegistrationFilter()
	: BaseFilter(FilterDescription("Fast Global Registration",
									"Fast Global Registration, by Zhou et al.",
									"Attempts to automatically register two clouds with normals without initial alignment",
									":/toolbar/PclUtils/icons/fastGlobalRegistration.png"))
	, m_alignedCloud(nullptr)
	, m_referenceCloud(nullptr)
	, m_featureRadius(0)
{
}

FastGlobalRegistrationFilter::~FastGlobalRegistrationFilter()
{
}

bool FastGlobalRegistrationFilter::checkSelected() const
{
	if (m_selectedEntities.size() != 2)
	{
		return false;
	}

	return m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
		&& m_selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD);
}

static bool ComputeFeatures(ccPointCloud* cloud, fgr::Features& features, double radius)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	unsigned pointCount = cloud->size();
	if (pointCount == 0)
	{
		ccLog::Warning("Cloud is empty");
		return false;
	}
	
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_cloud = cc2smReader(cloud).getAsPointNormal();
	if (!tmp_cloud)
	{
		ccLog::Warning("Failed to convert CC cloud to PCL cloud");
		return false;
	}

	pcl::PointCloud<pcl::FPFHSignature33> objectFeatures;
	try
	{
		pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> featEstimation;
		featEstimation.setRadiusSearch(radius);
		featEstimation.setInputCloud(tmp_cloud);
		featEstimation.setInputNormals(tmp_cloud);
		featEstimation.compute(objectFeatures);
	}
	catch (...)
	{
		ccLog::Warning("Failed to compute FPFH feature descriptors");
		return false;
	}

	try
	{
		features.resize(pointCount, Eigen::VectorXf(33));
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const pcl::FPFHSignature33& feature = objectFeatures.points[i];
			memcpy(features[i].data(), feature.histogram, sizeof(float) * 33);
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory");
		return false;
	}

	return true;
}

static bool ConverFromTo(const ccPointCloud& cloud, fgr::Points& points)
{
	unsigned pointCount = cloud.size();
	if (pointCount == 0)
	{
		ccLog::Warning("Cloud is empty");
		return false;
	}

	try
	{
		points.resize(pointCount);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = cloud.getPoint(i);
			points[i] = Eigen::Vector3f(P->x, P->y, P->z);
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory");
		return false;
	}

	return true;
}

int FastGlobalRegistrationFilter::getParametersFromDialog()
{
	//get selected pointclouds
	ccHObject::Container clouds;
	getSelectedEntitiesThatAreCCPointCloud(clouds);
	if (clouds.size() != 2)
	{
		return InvalidInput;
	}

	ccPointCloud* alignedCloud = ccHObjectCaster::ToPointCloud(clouds[0]);
	ccPointCloud* referenceCloud = ccHObjectCaster::ToPointCloud(clouds[1]);

	if (!alignedCloud || !referenceCloud)
	{
		assert(false);
		return InvalidInput;
	}
	if (!alignedCloud->hasNormals() || !referenceCloud->hasNormals())
	{
		ccLog::Error(tr("Clouds must have normals"));
		return InvalidInput;
	}

	FastGlobalRegistrationDialog dialog(alignedCloud, referenceCloud, m_app ? m_app->getMainWindow() : nullptr);

	if (!dialog.exec())
	{
		return CancelledByUser;
	}

	m_alignedCloud = dialog.getAlignedCloud();
	m_referenceCloud = dialog.getReferenceCloud();
	m_featureRadius = dialog.getFeatureRadius();

	dialog.saveParameters();

	return Success;
}

int FastGlobalRegistrationFilter::compute()
{
	if (!m_alignedCloud || !m_referenceCloud || m_featureRadius <= 0)
	{
		assert(false);
		return InvalidInput;
	}
	if (!m_alignedCloud->hasNormals() || !m_referenceCloud->hasNormals())
	{
		assert(false);
		return InvalidInput;
	}

	//compute the feature vector for each cloud
	fgr::Features alignedFeatures, referenceFeatures;
	if (	!ComputeFeatures(m_alignedCloud, alignedFeatures, m_featureRadius)
		||	!ComputeFeatures(m_referenceCloud, referenceFeatures, m_featureRadius) )
	{
		ccLog::Error("Failed to compute the point feature descriptors");
		return ComputationError;
	}

	//now we need to convert the clouds to vectors of Eigen::Vector3f
	fgr::Points alignedPoints, referencePoints;
	if (	!ConverFromTo(*m_alignedCloud, alignedPoints)
		||	!ConverFromTo(*m_referenceCloud, referencePoints) )
	{
		ccLog::Error("Not enough memory");
		return NotEnoughMemory;
	}


	ccGLMatrix ccTrans;
	try
	{
		fgr::CApp fgrProcess;
		fgrProcess.LoadFeature(referencePoints, referenceFeatures);
		fgrProcess.LoadFeature(alignedPoints, alignedFeatures);
		fgrProcess.NormalizePoints();
		fgrProcess.AdvancedMatching();
		if (!fgrProcess.OptimizePairwise(true))
		{
			ccLog::Error("Failed to perform pair-wise optimization (not enough points)");
			return ComputationError;
		}

		Eigen::Matrix4f trans = fgrProcess.GetOutputTrans();
		for (int i = 0; i < 16; ++i)
		{
			// both ccGLMatrix and Eigen::Matrix4f should use column-major storage
			ccTrans.data()[i] = trans.data()[i];
		}
	}
	catch (...)
	{
		ccLog::Error("Failed to determine the Global Registration matrix");
		return ComputationError;
	}

	m_alignedCloud->applyRigidTransformation(ccTrans);

	ccLog::Print(tr("[Fast Global Registration] Resulting matrix:"));
	ccLog::Print(ccTrans.toString(12, ' ')); //full precision
	ccLog::Print(tr("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool"));

	emit entityHasChanged(m_alignedCloud);

	return Success;
}
