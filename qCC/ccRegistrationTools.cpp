//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccRegistrationTools.h"

//Local
#include "ccRegistrationDlg.h"

//CCLib
#include <MeshSamplingTools.h>
#include <GenericIndexedCloudPersist.h>
#include <SimpleCloud.h>
#include <RegistrationTools.h>
#include <DistanceComputationTools.h>
#include <CloudSamplingTools.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccLog.h>

//system
#include <set>

//! Default number of points sampled on the 'model' mesh (if any)
static const unsigned s_defaultSampledPointsOnModelMesh = 100000;
//! Default number of points sampled on the 'data' mesh (if any)
static const unsigned s_defaultSampledPointsOnDataMesh = 50000;
//! Default temporary registration scalar field
static const char REGISTRATION_DISTS_SF[] = "RegistrationDistances";

template<class C> class Garbage
{
public:
	inline void add(C* entity)
	{
		try
		{
			m_entities.insert(entity);
		}
		catch(std::bad_alloc)
		{
			//what can we do?!
		}
	}

	inline void remove(C* entity)
	{
		m_entities.erase(entity);
	}

	~Garbage()
	{
		//dispose of left over
		for (std::set<C*>::iterator it = m_entities.begin(); it != m_entities.end(); ++it)
			delete *it;
		m_entities.clear();
	}
	std::set<C*> m_entities;
};

bool ccRegistrationTools::ICP(	ccHObject* data,
								ccHObject* model,
								ccGLMatrix& transMat,
								double &finalScale,
								double& finalError,
								double minErrorDecrease,
								unsigned maxIterationCount,
								unsigned randomSamplingLimit,
								bool removeFarthestPoints,
								CCLib::ICPRegistrationTools::CONVERGENCE_TYPE method,
								bool adjustScale,
								double finalOverlapRatio/*=1.0*/,
								bool useDataSFAsWeights/*=false*/,
								bool useModelSFAsWeights/*=false*/,
								int filters/*=CCLib::ICPRegistrationTools::SKIP_NONE*/,
								QWidget* parent/*=0*/)
{
	//progress bar
	ccProgressDialog pDlg(false,parent);

	Garbage<CCLib::GenericIndexedCloudPersist> cloudGarbage;

	//if the 'model' entity is a mesh, we need to sample points on it
	CCLib::GenericIndexedCloudPersist* modelCloud = 0;
	if (model->isKindOf(CC_TYPES::MESH))
	{
		modelCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(model),s_defaultSampledPointsOnModelMesh,&pDlg);
		if (!modelCloud)
		{
			ccLog::Error("[ICP] Failed to sample points on 'model' mesh!");
			return false;
		}
		cloudGarbage.add(modelCloud);
	}
	else
	{
		modelCloud = ccHObjectCaster::ToGenericPointCloud(model);
	}

	//if the 'data' entity is a mesh, we need to sample points on it
	CCLib::GenericIndexedCloudPersist* dataCloud = 0;
	if (data->isKindOf(CC_TYPES::MESH))
	{
		dataCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(data),s_defaultSampledPointsOnDataMesh,&pDlg);
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

	//unsigned maxRegistrationCount = randomSamplingLimit;
	if (finalOverlapRatio < 1.0)
	{
		//randomly pick points and deduce some stats on the average distances
		const unsigned c_randomProbingCount = 5000;
		CCLib::ReferenceCloud* realSampledCloud = 0;
		CCLib::GenericIndexedCloudPersist* sampledCloud = dataCloud;
		//do we really need to subsample the cloud?
		if (dataCloud->size() > c_randomProbingCount)
		{
			realSampledCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(dataCloud,c_randomProbingCount);
			if (realSampledCloud)
			{
				sampledCloud = realSampledCloud;
				cloudGarbage.add(realSampledCloud);
			}
			else
			{
				ccLog::Warning("[ICP][Partial overlap] Failed to subsample the data cloud (will have to go the long way...)");
			}
		}

		//compute distances on a (significant) subset of the input cloud
		CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
		params.multiThread = true;
		params.octreeLevel = 6;
		int result = CCLib::DistanceComputationTools::computeHausdorffDistance(sampledCloud,modelCloud,params,&pDlg);
		if (result < 0)
		{
			ccLog::Error("Failed to determine the max (overlap) distance (not enough memory?)");
			return false;
		}

		//add 5% margin to input ratio
		//finalOverlapRatio = std::min(finalOverlapRatio+0.05,1.0);

		//determine the max distance that (roughly) corresponds to the input overlap ratio
		ScalarType maxSearchDist = static_cast<ScalarType>(-1.0);
		{
			unsigned count = sampledCloud->size();
			std::vector<ScalarType> distances;
			try
			{
				distances.resize(count);
			}
			catch(std::bad_alloc)
			{
				ccLog::Error("Not enough memory!");
				return false;
			}
			for (unsigned i=0; i<count; ++i)
				distances[i] = sampledCloud->getPointScalarValue(i);
			std::sort(distances.begin(),distances.end());
			//now look for the max value at 'finalOverlapRatio' * count
			maxSearchDist = distances[static_cast<unsigned>((count-1)*finalOverlapRatio)];
		}

		if (realSampledCloud)
		{
			assert(sampledCloud != dataCloud);
			//free some memory right now
			cloudGarbage.remove(realSampledCloud);
			delete realSampledCloud;
			realSampledCloud = 0;
			//we now have to compute the (bounded) distances on the real cloud
			sampledCloud = dataCloud;
			params.maxSearchDist = static_cast<ScalarType>(maxSearchDist * 1.05); //safety margin of 5% (+ it helps to differentiate the 'max' value from the real values!)
			result = CCLib::DistanceComputationTools::computeHausdorffDistance(dataCloud,modelCloud,params,&pDlg);
			if (result < 0)
			{
				ccLog::Error("Failed to determine the max (overlap) distance (not enough memory?)");
				return false;
			}

			CCLib::ReferenceCloud* refCloud = new CCLib::ReferenceCloud(dataCloud);
			cloudGarbage.add(refCloud);
			unsigned count = dataCloud->size();
			unsigned baseIncrement = static_cast<unsigned>(std::max(100.0,count*finalOverlapRatio*0.05));
			for (unsigned i=0; i<count; ++i)
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

			ccLog::Print(QString("[ICP][Partial overlap] Will use %1 points out of %2 (%3%%)").arg(dataCloud->size()).arg(count).arg(100.0*static_cast<double>(dataCloud->size())/count));

			removeFarthestPoints = true;
		}
	}

	//parameters
	CCLib::PointProjectionTools::Transformation transform;

	CCLib::ScalarField* modelWeights = 0;
	if (useModelSFAsWeights)
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

	CCLib::ScalarField* dataWeights = 0;
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

	CCLib::ICPRegistrationTools::RESULT_TYPE result;

	result = CCLib::ICPRegistrationTools::RegisterClouds(	modelCloud,
															dataCloud,
															transform,
															method,
															minErrorDecrease,
															maxIterationCount,
															finalError,
															adjustScale,
															static_cast<CCLib::GenericProgressCallback*>(&pDlg),
															removeFarthestPoints,
															randomSamplingLimit,
															modelWeights,
															dataWeights,
															filters);

	if (result >= CCLib::ICPRegistrationTools::ICP_ERROR)
	{
		ccLog::Error("Registration failed: an error occurred (code %i)",result);
	}
	else if (result == CCLib::ICPRegistrationTools::ICP_APPLY_TRANSFO)
	{
		transMat = FromCCLibMatrix<PointCoordinateType,float>(transform.R, transform.T, transform.s);
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
