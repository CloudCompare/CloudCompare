//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccRegistrationTools.h"

//CCLib
#include <MeshSamplingTools.h>
#include <GenericIndexedCloudPersist.h>
#include <PointCloud.h>
#include <RegistrationTools.h>
#include <DistanceComputationTools.h>
#include <CloudSamplingTools.h>
#include <Garbage.h>
#include <ParallelSort.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccLog.h>

//system
#include <set>

//! Default number of points sampled on the 'data' mesh (if any)
static const unsigned s_defaultSampledPointsOnDataMesh = 50000;
//! Default temporary registration scalar field
static const char REGISTRATION_DISTS_SF[] = "RegistrationDistances";

bool ccRegistrationTools::ICP(	ccHObject* data,
								ccHObject* model,
								ccGLMatrix& transMat,
								double &finalScale,
								double& finalRMS,
								unsigned& finalPointCount,
								double minRMSDecrease,
								unsigned maxIterationCount,
								unsigned randomSamplingLimit,
								bool removeFarthestPoints,
								CCLib::ICPRegistrationTools::CONVERGENCE_TYPE method,
								bool adjustScale,
								double finalOverlapRatio/*=1.0*/,
								bool useDataSFAsWeights/*=false*/,
								bool useModelSFAsWeights/*=false*/,
								int filters/*=CCLib::ICPRegistrationTools::SKIP_NONE*/,
								int maxThreadCount/*=0*/,
								QWidget* parent/*=0*/)
{
	//progress bar
	ccProgressDialog pDlg(false, parent);

	Garbage<CCLib::GenericIndexedCloudPersist> cloudGarbage;

	//if the 'model' entity is a mesh, we need to sample points on it
	CCLib::GenericIndexedCloudPersist* modelCloud = 0;
	ccGenericMesh* modelMesh = 0;
	if (model->isKindOf(CC_TYPES::MESH))
	{
		modelMesh = ccHObjectCaster::ToGenericMesh(model);
		modelCloud = modelMesh->getAssociatedCloud();
	}
	else
	{
		modelCloud = ccHObjectCaster::ToGenericPointCloud(model);
	}

	//if the 'data' entity is a mesh, we need to sample points on it
	CCLib::GenericIndexedCloudPersist* dataCloud = 0;
	if (data->isKindOf(CC_TYPES::MESH))
	{
		dataCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(data), s_defaultSampledPointsOnDataMesh, &pDlg);
		if (!dataCloud)
		{
			ccLog::Error("[ICP] Failed to sample points on 'data' mesh!");
			return false;
		}
		cloudGarbage.add(dataCloud);
	}
	else
	{
		dataCloud = ccHObjectCaster::ToGenericPointCloud(data);
	}

	//we activate a temporary scalar field for registration distances computation
	CCLib::ScalarField* dataDisplayedSF = 0;
	int oldDataSfIdx = -1, dataSfIdx = -1;

	//if the 'data' entity is a real ccPointCloud, we can even create a proper temporary SF for registration distances
	if (data->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(data);
		dataDisplayedSF = pc->getCurrentDisplayedScalarField();
		oldDataSfIdx = pc->getCurrentInScalarFieldIndex();
		dataSfIdx = pc->getScalarFieldIndexByName(REGISTRATION_DISTS_SF);
		if (dataSfIdx < 0)
			dataSfIdx = pc->addScalarField(REGISTRATION_DISTS_SF);
		if (dataSfIdx >= 0)
			pc->setCurrentScalarField(dataSfIdx);
		else
		{
			ccLog::Error("[ICP] Couldn't create temporary scalar field! Not enough memory?");
			return false;
		}
	}
	else
	{
		if (!dataCloud->enableScalarField())
		{
			ccLog::Error("[ICP] Couldn't create temporary scalar field! Not enough memory?");
			return false;
		}
	}

	//add a 'safety' margin to input ratio
	static double s_overlapMarginRatio = 0.2;
	finalOverlapRatio = std::max(finalOverlapRatio, 0.01); //1% minimum
	//do we need to reduce the input point cloud (so as to be close
	//to the theoretical number of overlapping points - but not too
	//low so as we are not registered yet ;)
	if (finalOverlapRatio < 1.0 - s_overlapMarginRatio)
	{
		//DGM we can now use 'approximate' distances as SAITO algorithm is exact (but with a coarse resolution)
		//level = 7 if < 1.000.000
		//level = 8 if < 10.000.000
		//level = 9 if > 10.000.000
		int gridLevel = static_cast<int>(floor(log10(static_cast<double>(std::max(dataCloud->size(), modelCloud->size()))))) + 2;
		    gridLevel = std::min(std::max(gridLevel, 7), 9);
		int result = -1;
		if (modelMesh)
		{
			CCLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams c2mParams;
			c2mParams.octreeLevel = gridLevel;
			c2mParams.maxSearchDist = 0;
			c2mParams.useDistanceMap = true,
			c2mParams.signedDistances = false;
			c2mParams.flipNormals = false;
			c2mParams.multiThread = false;
			result = CCLib::DistanceComputationTools::computeCloud2MeshDistance(dataCloud, modelMesh, c2mParams, &pDlg);
		}
		else
		{
			result = CCLib::DistanceComputationTools::computeApproxCloud2CloudDistance(	dataCloud,
																						modelCloud,
																						gridLevel,
																						-1,
																						&pDlg);
		}

		if (result < 0)
		{
			ccLog::Error("Failed to determine the max (overlap) distance (not enough memory?)");
			return false;
		}

		//determine the max distance that (roughly) corresponds to the input overlap ratio
		ScalarType maxSearchDist = 0;
		{
			unsigned count = dataCloud->size();
			std::vector<ScalarType> distances;
			try
			{
				distances.resize(count);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Error("Not enough memory!");
				return false;
			}
			for (unsigned i=0; i<count; ++i)
			{
				distances[i] = dataCloud->getPointScalarValue(i);
			}
			
			ParallelSort(distances.begin(), distances.end());
			
			//now look for the max value at 'finalOverlapRatio+margin' percent
			maxSearchDist = distances[static_cast<unsigned>(std::max(1.0,count*(finalOverlapRatio+s_overlapMarginRatio)))-1];
		}

		//evntually select the points with distance below 'maxSearchDist'
		//(should roughly correspond to 'finalOverlapRatio + margin' percent)
		{
			CCLib::ReferenceCloud* refCloud = new CCLib::ReferenceCloud(dataCloud);
			cloudGarbage.add(refCloud);
			unsigned countBefore = dataCloud->size();
			unsigned baseIncrement = static_cast<unsigned>(std::max(100.0,countBefore*finalOverlapRatio*0.05));
			for (unsigned i=0; i<countBefore; ++i)
			{
				if (dataCloud->getPointScalarValue(i) <= maxSearchDist)
				{
					if (	refCloud->size() == refCloud->capacity()
						&&	!refCloud->reserve(refCloud->size() + baseIncrement) )
					{
						ccLog::Error("Not enough memory!");
						return false;
					}
					refCloud->addPointIndex(i);
				}
			}
			refCloud->resize(refCloud->size());
			dataCloud = refCloud;

			unsigned countAfter = dataCloud->size();
			double keptRatio = static_cast<double>(countAfter)/countBefore;
			ccLog::Print(QString("[ICP][Partial overlap] Selecting %1 points out of %2 (%3%) for registration").arg(countAfter).arg(countBefore).arg(static_cast<int>(100*keptRatio)));

			//update the relative 'final overlap' ratio
			finalOverlapRatio /= keptRatio;
		}
	}

	//weights
	CCLib::ScalarField* modelWeights = 0;
	CCLib::ScalarField* dataWeights = 0;
	{
		if (!modelMesh && useModelSFAsWeights)
		{
			if (modelCloud == dynamic_cast<CCLib::GenericIndexedCloudPersist*>(model) && model->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(model);
				modelWeights = pc->getCurrentDisplayedScalarField();
				if (!modelWeights)
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but model has no displayed scalar field!");
			}
			else
			{
				ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but only point clouds scalar fields can be used as weights!");
			}
		}

		if (useDataSFAsWeights)
		{
			if (!dataDisplayedSF)
			{
				if (dataCloud == (CCLib::GenericIndexedCloudPersist*)data && data->isA(CC_TYPES::POINT_CLOUD))
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but data has no displayed scalar field!");
				else
					ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but inly point clouds scalar fields can be used as weights!");
			}
			else
				dataWeights = dataDisplayedSF;
		}
	}

	CCLib::ICPRegistrationTools::RESULT_TYPE result;
	CCLib::PointProjectionTools::Transformation transform;
	CCLib::ICPRegistrationTools::Parameters params;
	{
		params.convType = method;
		params.minRMSDecrease = minRMSDecrease;
		params.nbMaxIterations = maxIterationCount;
		params.adjustScale = adjustScale;
		params.filterOutFarthestPoints = removeFarthestPoints;
		params.samplingLimit = randomSamplingLimit;
		params.finalOverlapRatio = finalOverlapRatio;
		params.modelWeights = modelWeights;
		params.dataWeights = dataWeights;
		params.transformationFilters = filters;
		params.maxThreadCount = maxThreadCount;
	}

	result = CCLib::ICPRegistrationTools::Register(	modelCloud,
													modelMesh,
													dataCloud,
													params,
													transform,
													finalRMS,
													finalPointCount,
													static_cast<CCLib::GenericProgressCallback*>(&pDlg));

	if (result >= CCLib::ICPRegistrationTools::ICP_ERROR)
	{
		ccLog::Error("Registration failed: an error occurred (code %i)",result);
	}
	else if (result == CCLib::ICPRegistrationTools::ICP_APPLY_TRANSFO)
	{
		transMat = FromCCLibMatrix<PointCoordinateType, float>(transform.R, transform.T, transform.s);
		finalScale = transform.s;
	}

	//remove temporary SF (if any)
	if (dataSfIdx >= 0)
	{
		assert(data->isA(CC_TYPES::POINT_CLOUD));
		ccPointCloud* pc = static_cast<ccPointCloud*>(data);
		pc->setCurrentScalarField(oldDataSfIdx);
		pc->deleteScalarField(dataSfIdx);
		dataSfIdx = -1;
	}

	return (result < CCLib::ICPRegistrationTools::ICP_ERROR);
}
