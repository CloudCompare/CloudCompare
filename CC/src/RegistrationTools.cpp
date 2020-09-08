//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include <RegistrationTools.h>

//local
#include <CloudSamplingTools.h>
#include <DistanceComputationTools.h>
#include <Garbage.h>
#include <GenericProgressCallback.h>
#include <GeometricalAnalysisTools.h>
#include <Jacobi.h>
#include <KdTree.h>
#include <ManualSegmentationTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <ScalarFieldTools.h>

//system
#include <ctime>

using namespace CCLib;

void RegistrationTools::FilterTransformation(	const ScaledTransformation& inTrans,
												int filters,
												ScaledTransformation& outTrans )
{
	outTrans = inTrans;

	//filter translation
	if (filters & SKIP_TRANSLATION)
	{
		if (filters & SKIP_TX)
			outTrans.T.x = 0;
		if (filters & SKIP_TY)
			outTrans.T.y = 0;
		if (filters & SKIP_TZ)
			outTrans.T.z = 0;
	}

	//filter rotation
	if (inTrans.R.isValid() && (filters & SKIP_ROTATION))
	{
		const CCLib::SquareMatrix R(inTrans.R); //copy it in case inTrans and outTrans are the same!
		outTrans.R.toIdentity();
		if (filters & SKIP_RYZ) //keep only the rotation component around X
		{
			//we use a specific Euler angles convention here
			if (R.getValue(0,2) < 1.0)
			{
				PointCoordinateType phi = -asin(R.getValue(0,2));
				PointCoordinateType cos_phi = cos(phi);
				PointCoordinateType theta = atan2(R.getValue(1,2)/cos_phi,R.getValue(2,2)/cos_phi);
				PointCoordinateType cos_theta = cos(theta);
				PointCoordinateType sin_theta = sin(theta);

				outTrans.R.setValue(1,1,cos_theta);
				outTrans.R.setValue(2,2,cos_theta);
				outTrans.R.setValue(2,1,sin_theta);
				outTrans.R.setValue(1,2,-sin_theta);
			}
			else
			{
				//simpler/faster to ignore this (very) specific case!
			}
		}
		else if (filters & SKIP_RXZ) //keep only the rotation component around Y
		{
			//we use a specific Euler angles convention here
			if (R.getValue(2,1) < 1.0)
			{
				PointCoordinateType theta = asin(R.getValue(2,1));
				PointCoordinateType cos_theta = cos(theta);
				PointCoordinateType phi = atan2(-R.getValue(2,0)/cos_theta,R.getValue(2,2)/cos_theta);
				PointCoordinateType cos_phi = cos(phi);
				PointCoordinateType sin_phi = sin(phi);

				outTrans.R.setValue(0,0,cos_phi);
				outTrans.R.setValue(2,2,cos_phi);
				outTrans.R.setValue(0,2,sin_phi);
				outTrans.R.setValue(2,0,-sin_phi);
			}
			else
			{
				//simpler/faster to ignore this (very) specific case!
			}
		}
		else if (filters & SKIP_RXY) //keep only the rotation component around Z
		{
			//we use a specific Euler angles convention here
			if (R.getValue(2,0) < 1.0)
			{
				PointCoordinateType theta_rad = -asin(R.getValue(2,0));
				PointCoordinateType cos_theta = cos(theta_rad);
				PointCoordinateType phi_rad = atan2(R.getValue(1,0)/cos_theta, R.getValue(0,0)/cos_theta);
				PointCoordinateType cos_phi	= cos(phi_rad);
				PointCoordinateType sin_phi	= sin(phi_rad);

				outTrans.R.setValue(0,0,cos_phi);
				outTrans.R.setValue(1,1,cos_phi);
				outTrans.R.setValue(1,0,sin_phi);
				outTrans.R.setValue(0,1,-sin_phi);
			}
			else
			{
				//simpler/faster to ignore this (very) specific case!
			}
		}
	}
}

struct ModelCloud
{
	ModelCloud() : cloud(nullptr), weights(nullptr) {}
	ModelCloud(const ModelCloud& m) = default;
	GenericIndexedCloudPersist* cloud;
	ScalarField* weights;
};

struct DataCloud
{
	DataCloud() : cloud(nullptr), rotatedCloud(nullptr), weights(nullptr), CPSetRef(nullptr), CPSetPlain(nullptr) {}
	
	ReferenceCloud* cloud;
	PointCloud* rotatedCloud;
	ScalarField* weights;
	ReferenceCloud* CPSetRef;
	PointCloud* CPSetPlain;
};

ICPRegistrationTools::RESULT_TYPE ICPRegistrationTools::Register(	GenericIndexedCloudPersist* inputModelCloud,
																	GenericIndexedMesh* inputModelMesh,
																	GenericIndexedCloudPersist* inputDataCloud,
																	const Parameters& params,
																	ScaledTransformation& transform,
																	double& finalRMS,
																	unsigned& finalPointCount,
																	GenericProgressCallback* progressCb/*=0*/)
{
	if (!inputModelCloud || !inputDataCloud)
	{
		assert(false);
		return ICP_ERROR_INVALID_INPUT;
	}


	//hopefully the user will understand it's not possible ;)
	finalRMS = -1.0;

	Garbage<GenericIndexedCloudPersist> cloudGarbage;
	Garbage<ScalarField> sfGarbage;

	//DATA CLOUD (will move)
	DataCloud data;
	{
		//we also want to use the same number of points for registration as initially defined by the user!
		unsigned dataSamplingLimit = params.finalOverlapRatio != 1.0 ? static_cast<unsigned>(params.samplingLimit / params.finalOverlapRatio) : params.samplingLimit;

		//we resample the cloud if it's too big (speed increase)
		if (inputDataCloud->size() > dataSamplingLimit)
		{
			data.cloud = CloudSamplingTools::subsampleCloudRandomly(inputDataCloud, dataSamplingLimit);
			if (!data.cloud)
			{
				return ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			cloudGarbage.add(data.cloud);

			//if we need to resample the weights as well
			if (params.dataWeights)
			{
				data.weights = new ScalarField("ResampledDataWeights");
				sfGarbage.add(data.weights);
				
				unsigned destCount = data.cloud->size();
				if (data.weights->resizeSafe(destCount))
				{
					for (unsigned i = 0; i < destCount; ++i)
					{
						unsigned pointIndex = data.cloud->getPointGlobalIndex(i);
						data.weights->setValue(i, params.dataWeights->getValue(pointIndex));
					}
					data.weights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					return ICP_ERROR_NOT_ENOUGH_MEMORY;
				}
			}
		}
		else //no need to resample
		{
			//we still create a 'fake' reference cloud with all the points
			data.cloud = new ReferenceCloud(inputDataCloud);
			cloudGarbage.add(data.cloud);
			if (!data.cloud->addPointIndex(0, inputDataCloud->size()))
			{
				//not enough memory
				return ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			//we use the input weights
			data.weights = params.dataWeights;
		}

		//eventually we'll need a scalar field on the data cloud
		if (!data.cloud->enableScalarField())
		{
			//not enough memory
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}
	}
	assert(data.cloud);

	//octree level for cloud/mesh distances computation
	unsigned char meshDistOctreeLevel = 8;

	//MODEL ENTITY (reference, won't move)
	ModelCloud model;
	if (inputModelMesh)
	{
		assert(!params.modelWeights);

		//we'll use the mesh vertices to estimate the right octree level
		DgmOctree dataOctree(data.cloud);
		DgmOctree modelOctree(inputModelCloud);
		if (dataOctree.build() < static_cast<int>(data.cloud->size()) || modelOctree.build() < static_cast<int>(inputModelCloud->size()))
		{
			//an error occurred during the octree computation: probably there's not enough memory
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}

		meshDistOctreeLevel = dataOctree.findBestLevelForComparisonWithOctree(&modelOctree);
	}
	else /*if (inputModelCloud)*/
	{
		//we resample the cloud if it's too big (speed increase)
		if (inputModelCloud->size() > params.samplingLimit)
		{
			ReferenceCloud* subModelCloud = CloudSamplingTools::subsampleCloudRandomly(inputModelCloud, params.samplingLimit);
			if (!subModelCloud)
			{
				//not enough memory
				return ICP_ERROR_NOT_ENOUGH_MEMORY;
			}
			cloudGarbage.add(subModelCloud);
			
			//if we need to resample the weights as well
			if (params.modelWeights)
			{
				model.weights = new ScalarField("ResampledModelWeights");
				sfGarbage.add(model.weights);

				unsigned destCount = subModelCloud->size();
				if (model.weights->resizeSafe(destCount))
				{
					for (unsigned i = 0; i < destCount; ++i)
					{
						unsigned pointIndex = subModelCloud->getPointGlobalIndex(i);
						model.weights->setValue(i, params.modelWeights->getValue(pointIndex));
					}
					model.weights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					return ICP_ERROR_NOT_ENOUGH_MEMORY;
				}
			}
			model.cloud = subModelCloud;
		}
		else
		{
			//we use the input cloud and weights
			model.cloud = inputModelCloud;
			model.weights = params.modelWeights;
		}
		assert(model.cloud);
	}

	//for partial overlap
	unsigned maxOverlapCount = 0;
	std::vector<ScalarType> overlapDistances;
	if (params.finalOverlapRatio < 1.0)
	{
		//we pre-allocate the memory to sort distance values later
		try
		{
			overlapDistances.resize(data.cloud->size());
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}
		maxOverlapCount = static_cast<unsigned>(params.finalOverlapRatio*data.cloud->size());
		assert(maxOverlapCount != 0);
	}

	//Closest Point Set (see ICP algorithm)
	if (inputModelMesh)
	{
		data.CPSetPlain = new PointCloud;
		cloudGarbage.add(data.CPSetPlain);
	}
	else
	{
		data.CPSetRef = new ReferenceCloud(model.cloud);
		cloudGarbage.add(data.CPSetRef);
	}

	//per-point couple weights
	ScalarField* coupleWeights = nullptr;
	if (model.weights || data.weights)
	{
		coupleWeights = new ScalarField("CoupleWeights");
		sfGarbage.add(coupleWeights);
	}

	//we compute the initial distance between the two clouds (and the CPSet by the way)
	//data.cloud->forEach(ScalarFieldTools::SetScalarValueToNaN); //DGM: done automatically in computeCloud2CloudDistance now
	if (inputModelMesh)
	{
		assert(data.CPSetPlain);
		DistanceComputationTools::Cloud2MeshDistanceComputationParams c2mDistParams;
		c2mDistParams.octreeLevel = meshDistOctreeLevel;
		c2mDistParams.CPSet = data.CPSetPlain;
		c2mDistParams.maxThreadCount = params.maxThreadCount;
		if (DistanceComputationTools::computeCloud2MeshDistance(data.cloud, inputModelMesh, c2mDistParams, progressCb) < 0)
		{
			//an error occurred during distances computation...
			return ICP_ERROR_DIST_COMPUTATION;
		}
	}
	else if (inputModelCloud)
	{
		assert(data.CPSetRef);
		DistanceComputationTools::Cloud2CloudDistanceComputationParams c2cDistParams;
		c2cDistParams.CPSet = data.CPSetRef;
		c2cDistParams.maxThreadCount = params.maxThreadCount;
		if (DistanceComputationTools::computeCloud2CloudDistance(data.cloud, model.cloud, c2cDistParams, progressCb) < 0)
		{
			//an error occurred during distances computation...
			return ICP_ERROR_DIST_COMPUTATION;
		}
	}
	else
	{
		assert(false);
	}

	FILE* fTraceFile = nullptr;
#ifdef CC_DEBUG
	fTraceFile = fopen("registration_trace_log.csv","wt");
	if (fTraceFile)
		fprintf(fTraceFile,"Iteration; RMS; Point count;\n");
#endif

	double lastStepRMS = -1.0;
	double initialDeltaRMS = -1.0;
	ScaledTransformation currentTrans;
	RESULT_TYPE result = ICP_ERROR;

	for (unsigned iteration = 0 ;; ++iteration)
	{
		if (progressCb && progressCb->isCancelRequested())
		{
			result = ICP_ERROR_CANCELED_BY_USER;
			break;
		}

		//shall we remove the farthest points?
		bool pointOrderHasBeenChanged = false;
		if (params.filterOutFarthestPoints)
		{
			NormalDistribution N;
			N.computeParameters(data.cloud);
			if (N.isValid())
			{
				ScalarType mu;
				ScalarType sigma2;
				N.getParameters(mu,sigma2);
				ScalarType maxDistance = static_cast<ScalarType>(mu + 2.5*sqrt(sigma2));

				DataCloud filteredData;
				filteredData.cloud = new ReferenceCloud(data.cloud->getAssociatedCloud());
				cloudGarbage.add(filteredData.cloud);

				if (data.CPSetRef)
				{
					filteredData.CPSetRef = new ReferenceCloud(data.CPSetRef->getAssociatedCloud()); //we must also update the CPSet!
					cloudGarbage.add(filteredData.CPSetRef);
				}
				else if (data.CPSetPlain)
				{
					filteredData.CPSetPlain = new PointCloud; //we must also update the CPSet!
					cloudGarbage.add(filteredData.CPSetPlain);
				}

				if (data.weights)
				{
					filteredData.weights = new ScalarField("ResampledDataWeights");
					sfGarbage.add(filteredData.weights);
				}

				unsigned pointCount = data.cloud->size();
				if (	!filteredData.cloud->reserve(pointCount)
					||	(filteredData.CPSetRef && !filteredData.CPSetRef->reserve(pointCount))
					||	(filteredData.CPSetPlain && !filteredData.CPSetPlain->reserve(pointCount))
					||	(filteredData.weights && !filteredData.weights->reserveSafe(pointCount)))
				{
					//not enough memory
					result = ICP_ERROR_NOT_ENOUGH_MEMORY;
					break;
				}

				//we keep only the points with "not too high" distances
				for (unsigned i=0; i<pointCount; ++i)
				{
					if (data.cloud->getPointScalarValue(i) <= maxDistance)
					{
						filteredData.cloud->addPointIndex(data.cloud->getPointGlobalIndex(i));
						if (filteredData.CPSetRef)
							filteredData.CPSetRef->addPointIndex(data.CPSetRef->getPointGlobalIndex(i));
						else if (filteredData.CPSetPlain)
							filteredData.CPSetPlain->addPoint(*(data.CPSetPlain->getPoint(i)));
						if (filteredData.weights)
							filteredData.weights->addElement(data.weights->getValue(i));
					}
				}

				//resize should be ok as we have called reserve first
				filteredData.cloud->resize(filteredData.cloud->size()); //should always be ok as current size < pointCount
				if (filteredData.CPSetRef)
					filteredData.CPSetRef->resize(filteredData.CPSetRef->size());
				else if (filteredData.CPSetPlain)
					filteredData.CPSetPlain->resize(filteredData.CPSetPlain->size());
				if (filteredData.weights)
					filteredData.weights->resize(filteredData.weights->currentSize());

				//replace old structures by new ones
				cloudGarbage.destroy(data.cloud);
				if (data.CPSetRef)
					cloudGarbage.destroy(data.CPSetRef);
				else if (data.CPSetPlain)
					cloudGarbage.destroy(data.CPSetPlain);
				if (data.weights)
					sfGarbage.destroy(data.weights);
				data = filteredData;

				pointOrderHasBeenChanged = true;
			}
		}

		//shall we ignore/remove some points based on their distance?
		DataCloud trueData;
		unsigned pointCount = data.cloud->size();
		if (maxOverlapCount != 0 && pointCount > maxOverlapCount)
		{
			assert(overlapDistances.size() >= pointCount);
			for (unsigned i=0; i<pointCount; ++i)
			{
				overlapDistances[i] = data.cloud->getPointScalarValue(i);
				assert(overlapDistances[i] == overlapDistances[i]);
			}
			
			ParallelSort(overlapDistances.begin(), overlapDistances.begin() + pointCount);

			assert(maxOverlapCount != 0);
			ScalarType maxOverlapDist = overlapDistances[maxOverlapCount-1];

			DataCloud filteredData;
			filteredData.cloud = new ReferenceCloud(data.cloud->getAssociatedCloud());
			if (data.CPSetRef)
			{
				filteredData.CPSetRef = new ReferenceCloud(data.CPSetRef->getAssociatedCloud()); //we must also update the CPSet!
				cloudGarbage.add(filteredData.CPSetRef);
			}
			else if (data.CPSetPlain)
			{
				filteredData.CPSetPlain = new PointCloud; //we must also update the CPSet!
				cloudGarbage.add(filteredData.CPSetPlain);
			}
			cloudGarbage.add(filteredData.cloud);
			if (data.weights)
			{
				filteredData.weights = new ScalarField("ResampledDataWeights");
				sfGarbage.add(filteredData.weights);
			}

			if (	!filteredData.cloud->reserve(pointCount) //should be maxOverlapCount in theory, but there may be several points with the same value as maxOverlapDist!
				||	(filteredData.CPSetRef && !filteredData.CPSetRef->reserve(pointCount))
				||	(filteredData.CPSetPlain && !filteredData.CPSetPlain->reserve(pointCount))
				||	(filteredData.weights && !filteredData.weights->reserveSafe(pointCount)))
			{
				//not enough memory
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}

			//we keep only the points with "not too high" distances
			for (unsigned i=0; i<pointCount; ++i)
			{
				if (data.cloud->getPointScalarValue(i) <= maxOverlapDist)
				{
					filteredData.cloud->addPointIndex(data.cloud->getPointGlobalIndex(i));
					if (filteredData.CPSetRef)
						filteredData.CPSetRef->addPointIndex(data.CPSetRef->getPointGlobalIndex(i));
					else if (filteredData.CPSetPlain)
						filteredData.CPSetPlain->addPoint(*(data.CPSetPlain->getPoint(i)));
					if (filteredData.weights)
						filteredData.weights->addElement(data.weights->getValue(i));
				}
			}
			assert(filteredData.cloud->size() >= maxOverlapCount);

			//resize should be ok as we have called reserve first
			filteredData.cloud->resize(filteredData.cloud->size()); //should always be ok as current size < pointCount
			if (filteredData.CPSetRef)
				filteredData.CPSetRef->resize(filteredData.CPSetRef->size());
			else if (filteredData.CPSetPlain)
				filteredData.CPSetPlain->resize(filteredData.CPSetPlain->size());
			if (filteredData.weights)
				filteredData.weights->resize(filteredData.weights->currentSize());

			//(temporarily) replace old structures by new ones
			trueData = data;
			data = filteredData;
		}

		//update couple weights (if any)
		if (coupleWeights)
		{
			assert(model.weights || data.weights);
			unsigned count = data.cloud->size();
			assert(!model.weights || (data.CPSetRef && data.CPSetRef->size() == count));

			if (coupleWeights->currentSize() != count && !coupleWeights->resizeSafe(count))
			{
				//not enough memory to store weights
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
			for (unsigned i = 0; i<count; ++i)
			{
				ScalarType wd = (data.weights ? data.weights->getValue(i) : static_cast<ScalarType>(1.0));
				ScalarType wm = (model.weights ? model.weights->getValue(data.CPSetRef->getPointGlobalIndex(i)) : static_cast<ScalarType>(1.0)); //model weights are only support with a reference cloud!
				coupleWeights->setValue(i, wd*wm);
			}
			coupleWeights->computeMinAndMax();
		}

		//we can now compute the best registration transformation for this step
		//(now that we have selected the points that will be used for registration!)
		{
			//if we use weights, we have to compute weighted RMS!!!
			double meanSquareValue = 0.0;
			double wiSum = 0.0; //we normalize the weights by their sum

			for (unsigned i = 0; i < data.cloud->size(); ++i)
			{
				ScalarType V = data.cloud->getPointScalarValue(i);
				if (ScalarField::ValidValue(V))
				{
					double wi = 1.0;
					if (coupleWeights)
					{
						ScalarType w = coupleWeights->getValue(i);
						if (!ScalarField::ValidValue(w))
							continue;
						wi = std::abs(w);
					}
					double Vd = wi * V;
					wiSum += wi * wi;
					meanSquareValue += Vd * Vd;
				}
			}

			//12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
			double meanSquareError = (wiSum != 0 ? static_cast<ScalarType>(meanSquareValue / wiSum) : 0);

			double rms = sqrt(meanSquareError);

#ifdef CC_DEBUG
			if (fTraceFile)
				fprintf(fTraceFile, "%u; %f; %u;\n", iteration, rms, data.cloud->size());
#endif
			if (iteration == 0)
			{
				//progress notification
				if (progressCb)
				{
					//on the first iteration, we init/show the dialog
					if (progressCb->textCanBeEdited())
					{
						progressCb->setMethodTitle("Clouds registration");
						char buffer[256];
						sprintf(buffer, "Initial RMS = %f\n", rms);
						progressCb->setInfo(buffer);
					}
					progressCb->update(0);
					progressCb->start();
				}

				finalRMS = rms;
				finalPointCount = data.cloud->size();

				if (rms < ZERO_TOLERANCE)
				{
					//nothing to do
					result = ICP_NOTHING_TO_DO;
					break;
				}
			}
			else
			{
				assert(lastStepRMS >= 0.0); 
				
				if (rms > lastStepRMS) //error increase!
				{
					result = iteration == 1 ? ICP_NOTHING_TO_DO : ICP_APPLY_TRANSFO;
					break;
				}

				//error update (RMS)
				double deltaRMS = lastStepRMS - rms;
				//should be better!
				assert(deltaRMS >= 0.0);

				//we update the global transformation matrix
				if (currentTrans.R.isValid())
				{
					if (transform.R.isValid())
						transform.R = currentTrans.R * transform.R;
					else
						transform.R = currentTrans.R;

					transform.T = currentTrans.R * transform.T;
				}

				if (params.adjustScale)
				{
					transform.s *= currentTrans.s;
					transform.T *= currentTrans.s;
				}

				transform.T += currentTrans.T;

				finalRMS = rms;
				finalPointCount = data.cloud->size();

				//stop criterion
				if (	(params.convType == MAX_ERROR_CONVERGENCE && deltaRMS < params.minRMSDecrease) //convergence reached
					||	(params.convType == MAX_ITER_CONVERGENCE && iteration >= params.nbMaxIterations) //max iteration reached
					)
				{
					result = ICP_APPLY_TRANSFO;
					break;
				}

				//progress notification
				if (progressCb)
				{

					if (progressCb->textCanBeEdited())
					{
						char buffer[256];
						sprintf(buffer, "RMS = %f [-%f]\n", rms, deltaRMS);
						progressCb->setInfo(buffer);
					}
					if (iteration == 1)
					{
						initialDeltaRMS = deltaRMS;
						progressCb->update(0);
					}
					else
					{
						assert(initialDeltaRMS >= 0.0);
						float progressPercent = static_cast<float>((initialDeltaRMS - deltaRMS) / (initialDeltaRMS - params.minRMSDecrease)*100.0);
						progressCb->update(progressPercent);
					}
				}
			}

			lastStepRMS = rms;
		}

		//single iteration of the registration procedure
		currentTrans = ScaledTransformation();
		if (!RegistrationTools::RegistrationProcedure(	data.cloud,
														data.CPSetRef ? static_cast<CCLib::GenericCloud*>(data.CPSetRef) : static_cast<CCLib::GenericCloud*>(data.CPSetPlain),
														currentTrans,
														params.adjustScale,
														coupleWeights))
		{
			result = ICP_ERROR_REGISTRATION_STEP;
			break;
		}

		//restore original data sets (if any were stored)
		if (trueData.cloud)
		{
			cloudGarbage.destroy(data.cloud);
			if (data.CPSetRef)
				cloudGarbage.destroy(data.CPSetRef);
			else if (data.CPSetPlain)
				cloudGarbage.destroy(data.CPSetPlain);
			if (data.weights)
				sfGarbage.destroy(data.weights);
			data = trueData;
		}

		//shall we filter some components of the resulting transformation?
		if (params.transformationFilters != SKIP_NONE)
		{
			//filter translation (in place)
			FilterTransformation(currentTrans, params.transformationFilters, currentTrans);
		}

		//get rotated data cloud
		if (!data.rotatedCloud || pointOrderHasBeenChanged)
		{
			//we create a new structure, with rotated points
			PointCloud* rotatedDataCloud = PointProjectionTools::applyTransformation(data.cloud, currentTrans);
			if (!rotatedDataCloud)
			{
				//not enough memory
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
			//replace data.rotatedCloud
			if (data.rotatedCloud)
				cloudGarbage.destroy(data.rotatedCloud);
			data.rotatedCloud = rotatedDataCloud;
			cloudGarbage.add(data.rotatedCloud);

			//update data.cloud
			data.cloud->clear();
			data.cloud->setAssociatedCloud(data.rotatedCloud);
			if (!data.cloud->addPointIndex(0, data.rotatedCloud->size()))
			{
				//not enough memory
				result = ICP_ERROR_NOT_ENOUGH_MEMORY;
				break;
			}
		}
		else
		{
			//we simply have to rotate the existing temporary cloud
			currentTrans.apply(*data.rotatedCloud);
			data.rotatedCloud->invalidateBoundingBox(); //invalidate bb

			//DGM: warning, we must manually invalidate the ReferenceCloud bbox after rotation!
			data.cloud->invalidateBoundingBox();
		}

		//compute (new) distances to model
		if (inputModelMesh)
		{
			DistanceComputationTools::Cloud2MeshDistanceComputationParams c2mDistParams;
			c2mDistParams.octreeLevel = meshDistOctreeLevel;
			c2mDistParams.CPSet = data.CPSetPlain;
			c2mDistParams.maxThreadCount = params.maxThreadCount;
			if (DistanceComputationTools::computeCloud2MeshDistance(data.cloud, inputModelMesh, c2mDistParams) < 0)
			{
				//an error occurred during distances computation...
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}
		}
		else if (inputDataCloud)
		{
			DistanceComputationTools::Cloud2CloudDistanceComputationParams c2cDistParams;
			c2cDistParams.CPSet = data.CPSetRef;
			c2cDistParams.maxThreadCount = params.maxThreadCount;
			if (DistanceComputationTools::computeCloud2CloudDistance(data.cloud, model.cloud, c2cDistParams) < 0)
			{
				//an error occurred during distances computation...
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}
		}
		else
		{
			assert(false);
		}
	}

	//end of tracefile
	if (fTraceFile)
	{
		fclose(fTraceFile);
		fTraceFile = nullptr;
	}

	//end of progress notification
	if (progressCb)
	{
		progressCb->stop();
	}

	return result;
}

bool HornRegistrationTools::FindAbsoluteOrientation(GenericCloud* lCloud,
													GenericCloud* rCloud,
													ScaledTransformation& trans,
													bool fixedScale/*=false*/)
{
	return RegistrationProcedure(lCloud, rCloud, trans, !fixedScale);
}

double HornRegistrationTools::ComputeRMS(GenericCloud* lCloud,
										 GenericCloud* rCloud,
										 const ScaledTransformation& trans)
{
	assert(rCloud && lCloud);
	if (!rCloud || !lCloud || rCloud->size() != lCloud->size() || rCloud->size() < 3)
		return false;

	double rms = 0.0;

	rCloud->placeIteratorAtBeginning();
	lCloud->placeIteratorAtBeginning();
	unsigned count = rCloud->size();
			
	for (unsigned i=0; i<count; i++)
	{
		const CCVector3* Ri = rCloud->getNextPoint();
		const CCVector3* Li = lCloud->getNextPoint();
		CCVector3 Lit = (trans.R.isValid() ? trans.R * (*Li) : (*Li))*trans.s + trans.T;

//#ifdef CC_DEBUG
//		double dist = (*Ri-Lit).norm();
//#endif

		rms += (*Ri - Lit).norm2();
	}

	return sqrt(rms / count);
}

bool RegistrationTools::RegistrationProcedure(	GenericCloud* P, //data
												GenericCloud* X, //model
												ScaledTransformation& trans,
												bool adjustScale/*=false*/,
												ScalarField* coupleWeights/*=0*/,
												PointCoordinateType aPrioriScale/*=1.0f*/)
{
	//resulting transformation (R is invalid on initialization, T is (0,0,0) and s==1)
	trans.R.invalidate();
	trans.T = CCVector3(0, 0, 0);
	trans.s = PC_ONE;

	if (P == nullptr || X == nullptr || P->size() != X->size() || P->size() < 3)
		return false;

	//centers of mass
	CCVector3 Gp = coupleWeights ? GeometricalAnalysisTools::ComputeWeightedGravityCenter(P, coupleWeights) : GeometricalAnalysisTools::ComputeGravityCenter(P);
	CCVector3 Gx = coupleWeights ? GeometricalAnalysisTools::ComputeWeightedGravityCenter(X, coupleWeights) : GeometricalAnalysisTools::ComputeGravityCenter(X);

	//specific case: 3 points only
	//See section 5.A in Horn's paper
	if (P->size() == 3)
	{
		//compute the first set normal
		P->placeIteratorAtBeginning();
		const CCVector3* Ap = P->getNextPoint();
		const CCVector3* Bp = P->getNextPoint();
		const CCVector3* Cp = P->getNextPoint();
		CCVector3 Np(0, 0, 1);
		{
			Np = (*Bp - *Ap).cross(*Cp - *Ap);
			double norm = Np.normd();
			if (norm < ZERO_TOLERANCE)
				return false;
			Np /= static_cast<PointCoordinateType>(norm);
		}
		//compute the second set normal
		X->placeIteratorAtBeginning();
		const CCVector3* Ax = X->getNextPoint();
		const CCVector3* Bx = X->getNextPoint();
		const CCVector3* Cx = X->getNextPoint();
		CCVector3 Nx(0, 0, 1);
		{
			Nx = (*Bx - *Ax).cross(*Cx - *Ax);
			double norm = Nx.normd();
			if (norm < ZERO_TOLERANCE)
				return false;
			Nx /= static_cast<PointCoordinateType>(norm);
		}
		//now the rotation is simply the rotation from Nx to Np, centered on Gx
		CCVector3 a = Np.cross(Nx);
		if (a.norm() < ZERO_TOLERANCE)
		{
			trans.R = CCLib::SquareMatrix(3);
			trans.R.toIdentity();
			if (Np.dot(Nx) < 0)
			{
				trans.R.scale(-PC_ONE);
			}
		}
		else
		{
			double cos_t = Np.dot(Nx);
			assert(cos_t > -1.0 && cos_t < 1.0); //see above
			double s = sqrt((1 + cos_t) * 2);
			double q[4] = { s / 2, a.x / s, a.y / s, a.z / s }; //don't forget to normalize the quaternion
			double qnorm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
			assert(qnorm >= ZERO_TOLERANCE);
			qnorm = sqrt(qnorm);
			q[0] /= qnorm;
			q[1] /= qnorm;
			q[2] /= qnorm;
			q[3] /= qnorm;
			trans.R.initFromQuaternion(q);
		}

		if (adjustScale)
		{
			double sumNormP = (*Bp - *Ap).norm() + (*Cp - *Bp).norm() + (*Ap - *Cp).norm();
			sumNormP *= aPrioriScale;
			if (sumNormP < ZERO_TOLERANCE)
				return false;
			double sumNormX = (*Bx - *Ax).norm() + (*Cx - *Bx).norm() + (*Ax - *Cx).norm();
			trans.s = static_cast<PointCoordinateType>(sumNormX / sumNormP); //sumNormX / (sumNormP * Sa) in fact
		}

		//we deduce the first translation
		trans.T = Gx - (trans.R*Gp) * (aPrioriScale*trans.s); //#26 in Besl paper, modified with the scale as in jschmidt

		//we need to find the rotation in the (X) plane now
		{
			CCVector3 App = trans.apply(*Ap);
			CCVector3 Bpp = trans.apply(*Bp);
			CCVector3 Cpp = trans.apply(*Cp);

			double C = 0;
			double S = 0;
			CCVector3 Ssum(0, 0, 0);
			CCVector3 rx;
			CCVector3 rp;

			rx = *Ax - Gx;
			rp = App - Gx;
			C = rx.dot(rp);
			Ssum = rx.cross(rp);

			rx = *Bx - Gx;
			rp = Bpp - Gx;
			C += rx.dot(rp);
			Ssum += rx.cross(rp);

			rx = *Cx - Gx;
			rp = Cpp - Gx;
			C += rx.dot(rp);
			Ssum += rx.cross(rp);

			S = Ssum.dot(Nx);
			double Q = sqrt(S*S + C * C);
			if (Q < ZERO_TOLERANCE)
				return false;

			PointCoordinateType sin_t = static_cast<PointCoordinateType>(S / Q);
			PointCoordinateType cos_t = static_cast<PointCoordinateType>(C / Q);
			PointCoordinateType inv_cos_t = 1 - cos_t;

			const PointCoordinateType& l1 = Nx.x;
			const PointCoordinateType& l2 = Nx.y;
			const PointCoordinateType& l3 = Nx.z;

			PointCoordinateType l1_inv_cos_t = l1 * inv_cos_t;
			PointCoordinateType l3_inv_cos_t = l3 * inv_cos_t;

			SquareMatrix R(3);
			//1st column
			R.m_values[0][0] = cos_t + l1 * l1_inv_cos_t;
			R.m_values[0][1] = l2 * l1_inv_cos_t + l3 * sin_t;
			R.m_values[0][2] = l3 * l1_inv_cos_t - l2 * sin_t;

			//2nd column
			R.m_values[1][0] = l2 * l1_inv_cos_t - l3 * sin_t;
			R.m_values[1][1] = cos_t + l2 * l2*inv_cos_t;
			R.m_values[1][2] = l2 * l3_inv_cos_t + l1 * sin_t;

			//3rd column
			R.m_values[2][0] = l3 * l1_inv_cos_t + l2 * sin_t;
			R.m_values[2][1] = l2 * l3_inv_cos_t - l1 * sin_t;
			R.m_values[2][2] = cos_t + l3 * l3_inv_cos_t;

			trans.R = R * trans.R;
			trans.T = Gx - (trans.R*Gp) * (aPrioriScale*trans.s); //update T as well
		}
	}
	else
	{
		CCVector3 bbMin;
		CCVector3 bbMax;
		X->getBoundingBox(bbMin, bbMax);

		//if the data cloud is equivalent to a single point (for instance
		//it's the case when the two clouds are very far away from
		//each other in the ICP process) we try to get the two clouds closer
		CCVector3 diag = bbMax - bbMin;
		if (std::abs(diag.x) + std::abs(diag.y) + std::abs(diag.z) < ZERO_TOLERANCE)
		{
			trans.T = Gx - Gp * aPrioriScale;
			return true;
		}

		//Cross covariance matrix, eq #24 in Besl92 (but with weights, if any)
		SquareMatrixd Sigma_px = (coupleWeights ? GeometricalAnalysisTools::ComputeWeightedCrossCovarianceMatrix(P, X, Gp, Gx, coupleWeights)
			: GeometricalAnalysisTools::ComputeCrossCovarianceMatrix(P, X, Gp, Gx));
		if (!Sigma_px.isValid())
			return false;

		//transpose sigma_px
		SquareMatrixd Sigma_px_t = Sigma_px.transposed();

		SquareMatrixd Aij = Sigma_px - Sigma_px_t;

		double trace = Sigma_px.trace(); //that is the sum of diagonal elements of sigma_px

		SquareMatrixd traceI3(3); //create the I matrix with eigvals equal to trace
		traceI3.m_values[0][0] = trace;
		traceI3.m_values[1][1] = trace;
		traceI3.m_values[2][2] = trace;

		SquareMatrixd bottomMat = Sigma_px + Sigma_px_t - traceI3;

		//we build up the registration matrix (see ICP algorithm)
		SquareMatrixd QSigma(4); //#25 in the paper (besl)

		QSigma.m_values[0][0] = trace;

		QSigma.m_values[0][1] = QSigma.m_values[1][0] = Aij.m_values[1][2];
		QSigma.m_values[0][2] = QSigma.m_values[2][0] = Aij.m_values[2][0];
		QSigma.m_values[0][3] = QSigma.m_values[3][0] = Aij.m_values[0][1];

		QSigma.m_values[1][1] = bottomMat.m_values[0][0];
		QSigma.m_values[1][2] = bottomMat.m_values[0][1];
		QSigma.m_values[1][3] = bottomMat.m_values[0][2];

		QSigma.m_values[2][1] = bottomMat.m_values[1][0];
		QSigma.m_values[2][2] = bottomMat.m_values[1][1];
		QSigma.m_values[2][3] = bottomMat.m_values[1][2];

		QSigma.m_values[3][1] = bottomMat.m_values[2][0];
		QSigma.m_values[3][2] = bottomMat.m_values[2][1];
		QSigma.m_values[3][3] = bottomMat.m_values[2][2];

		//we compute its eigenvalues and eigenvectors
		CCLib::SquareMatrixd eigVectors;
		std::vector<double> eigValues;
		if (!Jacobi<double>::ComputeEigenValuesAndVectors(QSigma, eigVectors, eigValues, false))
		{
			//failure
			return false;
		}

		//as Besl says, the best rotation corresponds to the eigenvector associated to the biggest eigenvalue
		double qR[4];
		double maxEigValue = 0;
		Jacobi<double>::GetMaxEigenValueAndVector(eigVectors, eigValues, maxEigValue, qR);

		//these eigenvalue and eigenvector correspond to a quaternion --> we get the corresponding matrix
		trans.R.initFromQuaternion(qR);

		if (adjustScale)
		{
			//two accumulators
			double acc_num = 0.0;
			double acc_denom = 0.0;

			//now deduce the scale (refer to "Point Set Registration with Integrated Scale Estimation", Zinsser et. al, PRIP 2005)
			X->placeIteratorAtBeginning();
			P->placeIteratorAtBeginning();

			unsigned count = X->size();
			assert(P->size() == count);
			for (unsigned i = 0; i < count; ++i)
			{
				//'a' refers to the data 'A' (moving) = P
				//'b' refers to the model 'B' (not moving) = X
				CCVector3 a_tilde = trans.R * (*(P->getNextPoint()) - Gp);	// a_tilde_i = R * (a_i - a_mean)
				CCVector3 b_tilde = (*(X->getNextPoint()) - Gx);			// b_tilde_j =     (b_j - b_mean)

				acc_num += b_tilde.dot(a_tilde);
				acc_denom += a_tilde.dot(a_tilde);
			}

			//DGM: acc_2 can't be 0 because we already have checked that the bbox is not a single point!
			assert(acc_denom > 0.0);
			trans.s = static_cast<PointCoordinateType>(std::abs(acc_num / acc_denom));
		}

		//and we deduce the translation
		trans.T = Gx - (trans.R*Gp) * (aPrioriScale*trans.s); //#26 in besl paper, modified with the scale as in jschmidt
	}

	return true;
}

bool FPCSRegistrationTools::RegisterClouds(	GenericIndexedCloud* modelCloud,
											GenericIndexedCloud* dataCloud,
											ScaledTransformation& transform,
											ScalarType delta,
											ScalarType beta,
											PointCoordinateType overlap,
											unsigned nbBases,
											unsigned nbTries,
											GenericProgressCallback* progressCb,
											unsigned nbMaxCandidates)
{
	//DGM: KDTree::buildFromCloud will call reset right away!
	//if (progressCb)
	//{
	//	if (progressCb->textCanBeEdited())
	//	{
	//		progressCb->setMethodTitle("Clouds registration");
	//		progressCb->setInfo("Starting 4PCS");
	//	}
	//	progressCb->update(0);
	//	progressCb->start();
	//}

	//Initialize random seed with current time
	srand(static_cast<unsigned>(time(nullptr)));

	unsigned bestScore = 0;
	unsigned score = 0;
	transform.R.invalidate();
	transform.T = CCVector3(0,0,0);

	//Adapt overlap to the model cloud size
	{
		CCVector3 bbMin;
		CCVector3 bbMax;
		modelCloud->getBoundingBox(bbMin, bbMax);
		CCVector3 diff = bbMax - bbMin;
		overlap *= diff.norm() / 2;
	}

	//Build the associated KDtrees
	KDTree* dataTree = new KDTree();
	if (!dataTree->buildFromCloud(dataCloud, progressCb))
	{
		delete dataTree;
		return false;
	}
	KDTree* modelTree = new KDTree();
	if (!modelTree->buildFromCloud(modelCloud, progressCb))
	{
		delete dataTree;
		delete modelTree;
		return false;
	}

	//if (progressCb)
	//    progressCb->stop();

	for (unsigned i=0; i<nbBases; i++)
	{
		//Randomly find the current reference base
		Base reference;
		if (!FindBase(modelCloud, overlap, nbTries, reference))
			continue;

		//Search for all the congruent bases in the second cloud
		std::vector<Base> candidates;
		unsigned count = dataCloud->size();
		candidates.reserve(count);
		if (candidates.capacity() < count) //not enough memory
		{
			delete dataTree;
			delete modelTree;
			transform.R = SquareMatrix();
			return false;
		}
		const CCVector3* referenceBasePoints[4];
		{
			for(unsigned j=0; j<4; j++)
				referenceBasePoints[j] = modelCloud->getPoint(reference.getIndex(j));
		}
		int result = FindCongruentBases(dataTree, beta, referenceBasePoints, candidates);
		if (result == 0)
			continue;
		else if (result < 0) //something bad happened!
		{
			delete dataTree;
			delete modelTree;
			transform.R = SquareMatrix();
			return false;
		}

		//Compute rigid transforms and filter bases if necessary
		{
			std::vector<ScaledTransformation> transforms;
			if (!FilterCandidates(modelCloud, dataCloud, reference, candidates, nbMaxCandidates, transforms))
			{
				delete dataTree;
				delete modelTree;
				transform.R = SquareMatrix();
				return false;
			}

			for(unsigned j=0; j<candidates.size(); j++)
			{
				//Register the current candidate base with the reference base
				const ScaledTransformation& RT = transforms[j];
				//Apply the rigid transform to the data cloud and compute the registration score
				if (RT.R.isValid())
				{
					score = ComputeRegistrationScore(modelTree, dataCloud, delta, RT);

					//Keep parameters that lead to the best result
					if (score > bestScore)
					{
						transform.R = RT.R;
						transform.T = RT.T;
						bestScore = score;
					}
				}
			}
		}

		if (progressCb)
		{
			if (progressCb->textCanBeEdited())
			{
				char buffer[256];
				sprintf(buffer, "Trial %u/%u [best score = %u]\n", i + 1, nbBases, bestScore);
				progressCb->setInfo(buffer);
				progressCb->update(((i + 1)*100.0f) / nbBases);
			}

			if (progressCb->isCancelRequested())
			{
				delete dataTree;
				delete modelTree;
				transform.R = SquareMatrix();
				return false;
			}
		}
	}

	delete dataTree;
	delete modelTree;

	if (progressCb)
	{
		progressCb->stop();
	}

	return (bestScore > 0);
}


 unsigned FPCSRegistrationTools::ComputeRegistrationScore(	KDTree *modelTree,
															GenericIndexedCloud *dataCloud,
															ScalarType delta,
															const ScaledTransformation& dataToModel)
{
	CCVector3 Q;

	unsigned score = 0;

	unsigned count = dataCloud->size();
	for (unsigned i=0; i<count; ++i)
	{
		dataCloud->getPoint(i,Q);
		//Apply rigid transform to each point
		Q = dataToModel.R * Q + dataToModel.T;
		//Check if there is a point in the model cloud that is close enough to q
		if (modelTree->findPointBelowDistance(Q.u, delta))
			score++;
	}

	return score;
 }

bool FPCSRegistrationTools::FindBase(	GenericIndexedCloud* cloud,
										PointCoordinateType overlap,
										unsigned nbTries,
										Base &base)
{
	unsigned a;
	unsigned b;
	unsigned c;
	unsigned d;
	unsigned i;
	unsigned size;
	PointCoordinateType f;
	PointCoordinateType best;
	PointCoordinateType d0;
	PointCoordinateType d1;
	PointCoordinateType d2;
	PointCoordinateType x;
	PointCoordinateType y;
	PointCoordinateType z;
	PointCoordinateType w;
	const CCVector3 *p0 = nullptr;
	const CCVector3 *p1 = nullptr;
	const CCVector3 *p2 = nullptr;
	const CCVector3 *p3 = nullptr;
	CCVector3 normal;
	CCVector3 u;
	CCVector3 v;

	overlap *= overlap;
	size = cloud->size();
	best = 0.;
	b = c = 0;
	a = rand() % size;
	p0 = cloud->getPoint(a);
	//Randomly pick 3 points as sparsed as possible
	for (i = 0; i < nbTries; i++)
	{
		unsigned t1 = (rand() % size);
		unsigned t2 = (rand() % size);
		if (t1 == a || t2 == a || t1 == t2)
			continue;

		p1 = cloud->getPoint(t1);
		p2 = cloud->getPoint(t2);
		//Checked that the selected points are not more than overlap-distant from p0
		u = *p1 - *p0;
		if (u.norm2() > overlap)
			continue;
		u = *p2 - *p0;
		if (u.norm2() > overlap)
			continue;

		//compute [p0, p1, p2] area thanks to cross product
		x = ((p1->y - p0->y)*(p2->z - p0->z)) - ((p1->z - p0->z)*(p2->y - p0->y));
		y = ((p1->z - p0->z)*(p2->x - p0->x)) - ((p1->x - p0->x)*(p2->z - p0->z));
		z = ((p1->x - p0->x)*(p2->y - p0->y)) - ((p1->y - p0->y)*(p2->x - p0->x));
		//don't need to compute the true area : f=(area²)*2 is sufficient for comparison
		f = x * x + y * y + z * z;
		if (f > best)
		{
			b = t1;
			c = t2;
			best = f;
			normal.x = x;
			normal.y = y;
			normal.z = z;
		}
	}

	if (b == c)
		return false;

	//Once we found the points, we have to search for a fourth coplanar point
	f = normal.norm();
	if (f <= 0)
		return false;
	normal *= 1.0f / f;
	//plane equation : p lies in the plane if x*p[0] + y*p[1] + z*p[2] + w = 0
	x = normal.x;
	y = normal.y;
	z = normal.z;
	w = -(x*p0->x) - (y*p0->y) - (z*p0->z);
	d = a;
	best = -1.;
	p1 = cloud->getPoint(b);
	p2 = cloud->getPoint(c);
	for(i=0; i<nbTries; i++)
	{
		unsigned t1 = (rand() % size);
		if (t1 == a || t1 == b || t1 == c)
			continue;
		p3 = cloud->getPoint(t1);
		//p3 must be close enough to at least two other points (considering overlap)
		d0 = (*p3 - *p0).norm2();
		d1 = (*p3 - *p1).norm2();
		d2 = (*p3 - *p2).norm2();
		if ((d0 >= overlap && d1 >= overlap) || (d0 >= overlap && d2 >= overlap) || (d1 >= overlap && d2 >= overlap))
			continue;
		//Compute distance to the plane (cloud[a], cloud[b], cloud[c])
		f = std::abs((x*p3->x) + (y*p3->y) + (z*p3->z) + w);
		//keep the point which is the closest to the plane, while being as far as possible from the other three points
		f = (f + 1.0f) / (sqrt(d0) + sqrt(d1) + sqrt(d2));
		if ((best < 0.) || (f < best))
		{
			d = t1;
			best = f;
		}
	}

	//Store the result in the base parameter
	if (d != a)
	{
		//Find the points order in the quadrilateral
		p0 = cloud->getPoint(a);
		p1 = cloud->getPoint(b);
		p2 = cloud->getPoint(c);
		p3 = cloud->getPoint(d);
		//Search for the diagonnals of the convexe hull (3 tests max)
		//Note : if the convexe hull is made of 3 points, the points order has no importance
		u = (*p1-*p0)*(*p2-*p0);
		v = (*p1-*p0)*(*p3-*p0);
		if (u.dot(v) <= 0)
		{
			//p2 and p3 lie on both sides of [p0, p1]
			base.init(a, b, c, d);
			return true;
		}
		u = (*p2-*p1)*(*p0-*p1);
		v = (*p2-*p1)*(*p3-*p1);
		if (u.dot(v) <= 0)
		{
			//p0 and p3 lie on both sides of [p2, p1]
			base.init(b, c, d, a);
			return true;
		}
		base.init(a, c, b, d);
		return true;
	}

	return false;
}

//pair of indexes
using IndexPair = std::pair<unsigned,unsigned>;

int FPCSRegistrationTools::FindCongruentBases(KDTree* tree,
												ScalarType delta,
												const CCVector3* base[4],
												std::vector<Base>& results)
{
	//Compute reference base invariants (r1, r2)
	PointCoordinateType r1;
	PointCoordinateType r2;
	PointCoordinateType d1;
	PointCoordinateType d2;
	{
		const CCVector3* p0 = base[0];
		const CCVector3* p1 = base[1];
		const CCVector3* p2 = base[2];
		const CCVector3* p3 = base[3];

		d1 = (*p1-*p0).norm();
		d2 = (*p3-*p2).norm();

		CCVector3 inter;
		if (!LinesIntersections(*p0, *p1, *p2, *p3, inter, r1, r2))
			return 0;
	}

	GenericIndexedCloud* cloud = tree->getAssociatedCloud();

	//Find all pairs which are d1-appart and d2-appart
	std::vector<IndexPair> pairs1;
	std::vector<IndexPair> pairs2;
	{
		unsigned count = static_cast<unsigned>(cloud->size());
		std::vector<unsigned> pointsIndexes;
		try
		{
			pointsIndexes.reserve(count);
		}
		catch(...)
		{
			//not enough memory
			return -1;
		}

		for (unsigned i = 0; i < count; i++)
		{
			const CCVector3 *q0 = cloud->getPoint(i);
			IndexPair idxPair;
			idxPair.first = i;
			//Extract all points from the cloud which are d1-appart (up to delta) from q0
			pointsIndexes.clear();
			tree->findPointsLyingToDistance(q0->u, static_cast<ScalarType>(d1), delta, pointsIndexes);
			{
				for (std::size_t j = 0; j < pointsIndexes.size(); j++)
				{
					//As ||pi-pj|| = ||pj-pi||, we only take care of pairs that verify i<j
					if (pointsIndexes[j] > i)
					{
						idxPair.second = pointsIndexes[j];
						pairs1.push_back(idxPair);
					}
				}
			}
			//Extract all points from the cloud which are d2-appart (up to delta) from q0
			pointsIndexes.clear();
			tree->findPointsLyingToDistance(q0->u, static_cast<ScalarType>(d2), delta, pointsIndexes);
			{
				for (std::size_t j = 0; j < pointsIndexes.size(); j++)
				{
					if (pointsIndexes[j] > i)
					{
						idxPair.second = pointsIndexes[j];
						pairs2.push_back(idxPair);
					}
				}
			}
		}
	}

	//Select among the pairs the ones that can be congruent to the base "base"
	std::vector<IndexPair> match;
	{
		PointCloud tmpCloud1;
		PointCloud tmpCloud2;
		{
			unsigned count = static_cast<unsigned>(pairs1.size());
			if (!tmpCloud1.reserve(count * 2)) //not enough memory
				return -2;
			for (unsigned i = 0; i < count; i++)
			{
				//generate the two intermediate points from r1 in pairs1[i]
				const CCVector3 *q0 = cloud->getPoint(pairs1[i].first);
				const CCVector3 *q1 = cloud->getPoint(pairs1[i].second);
				CCVector3 P1 = *q0 + r1*(*q1-*q0);
				tmpCloud1.addPoint(P1);
				CCVector3 P2 = *q1 + r1*(*q0-*q1);
				tmpCloud1.addPoint(P2);
			}
		}
	
		{
			unsigned count = static_cast<unsigned>(pairs2.size());
			if (!tmpCloud2.reserve(count*2)) //not enough memory
				return -3;
			for(unsigned i=0; i<count; i++)
			{
				//generate the two intermediate points from r2 in pairs2[i]
				const CCVector3 *q0 = cloud->getPoint(pairs2[i].first);
				const CCVector3 *q1 = cloud->getPoint(pairs2[i].second);
				CCVector3 P1 = *q0 + r2*(*q1-*q0);
				tmpCloud2.addPoint(P1);
				CCVector3 P2 = *q1 + r2*(*q0-*q1);
				tmpCloud2.addPoint(P2);
			}
		}

		//build kdtree for nearest neighbour fast research
		KDTree intermediateTree;
		if (!intermediateTree.buildFromCloud(&tmpCloud1))
			return -4;

		//Find matching (up to delta) intermediate points in tmpCloud1 and tmpCloud2
		{
			unsigned count = static_cast<unsigned>(tmpCloud2.size());
			match.reserve(count);
			if (match.capacity() < count)	//not enough memory
				return -5;
		
			for(unsigned i=0; i<count; i++)
			{
				const CCVector3 *q0 = tmpCloud2.getPoint(i);
				unsigned a;
				if (intermediateTree.findNearestNeighbour(q0->u, a, delta))
				{
					IndexPair idxPair;
					idxPair.first = i;
					idxPair.second = a;
					match.push_back(idxPair);
				}
			}
		}
	}

	//Find bases from matching intermediate points indexes
	{
		results.resize(0);
		std::size_t count = match.size();
		if (count > 0)
		{
			try
			{
				results.reserve(count);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return -6;
			}
			for (std::size_t i = 0; i < count; i++)
			{
				Base quad;
				unsigned b = match[i].second / 2;
				if ((match[i].second % 2) == 0)
				{
					quad.a = pairs1[b].first;
					quad.b = pairs1[b].second;
				}
				else
				{
					quad.a = pairs1[b].second;
					quad.b = pairs1[b].first;
				}

				unsigned a = match[i].first / 2;
				if ((match[i].first % 2) == 0)
				{
					quad.c = pairs2[a].first;
					quad.d = pairs2[a].second;
				}
				else
				{
					quad.c = pairs2[a].second;
					quad.d = pairs2[a].first;
				}
				results.push_back(quad);
			}
		}
	}

	return static_cast<int>(results.size());
}


bool FPCSRegistrationTools::LinesIntersections(	const CCVector3 &p0,
												const CCVector3 &p1,
												const CCVector3 &p2,
												const CCVector3 &p3,
												CCVector3 &inter,
												PointCoordinateType& lambda,
												PointCoordinateType& mu)
{
	CCVector3 p02;
	CCVector3 p32;
	CCVector3 p10;
	CCVector3 A;
	CCVector3 B;
	PointCoordinateType num;
	PointCoordinateType denom;

	//Find lambda and mu such that :
	//A = p0+lambda(p1-p0)
	//B = p2+mu(p3-p2)
	//(lambda, mu) = argmin(||A-B||²)
	p02 = p0-p2;
	p32 = p3-p2;
	p10 = p1-p0;
	num = (p02.dot(p32) * p32.dot(p10)) - (p02.dot(p10) * p32.dot(p32));
	denom = (p10.dot(p10) * p32.dot(p32)) - (p32.dot(p10) * p32.dot(p10));
	if (std::abs(denom) < 0.00001)
		return false;
	lambda = num / denom;
	num = p02.dot(p32) + (lambda*p32.dot(p10));
	denom = p32.dot(p32);
	if (std::abs(denom) < 0.00001)
		return false;
	mu = num / denom;
	A.x = p0.x + (lambda*p10.x);
	A.y = p0.y + (lambda*p10.y);
	A.z = p0.z + (lambda*p10.z);
	B.x = p2.x + (mu*p32.x);
	B.y = p2.y + (mu*p32.y);
	B.z = p2.z + (mu*p32.z);
	inter.x = (A.x + B.x) / 2.0f;
	inter.y = (A.y + B.y) / 2.0f;
	inter.z = (A.z + B.z) / 2.0f;

	return true;
}

bool FPCSRegistrationTools::FilterCandidates(	GenericIndexedCloud *modelCloud,
												GenericIndexedCloud *dataCloud,
												Base& reference,
												std::vector<Base>& candidates,
												unsigned nbMaxCandidates,
												std::vector<ScaledTransformation>& transforms)
{
	std::vector<Base> table;
	std::vector<float> scores;
	std::vector<float> sortedscores;
	const CCVector3* p[4];
	ScaledTransformation t;
	std::vector<ScaledTransformation> tarray;
	PointCloud referenceBaseCloud;
	PointCloud dataBaseCloud;

	unsigned candidatesCount = static_cast<unsigned>(candidates.size());
	if (candidatesCount == 0)
		return false;

	bool filter = (nbMaxCandidates>0 && candidatesCount > nbMaxCandidates);
	{
		try
		{
			table.resize(candidatesCount);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}
		for (unsigned i=0; i<candidatesCount; i++)
			table[i].copy(candidates[i]);
	}

	if (!referenceBaseCloud.reserve(4)) //we never know ;)
		return false;

	{
		for (unsigned j=0; j<4; j++)
		{
			p[j] = modelCloud->getPoint(reference.getIndex(j));
			referenceBaseCloud.addPoint(*p[j]);
		}
	}

	try
	{
		scores.reserve(candidatesCount);
		sortedscores.reserve(candidatesCount);
		tarray.reserve(candidatesCount);
		transforms.reserve(candidatesCount);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	//enough memory?
	if (	scores.capacity() < candidatesCount 
		||	sortedscores.capacity() < candidatesCount 
		||	tarray.capacity() < candidatesCount 
		||	transforms.capacity() < candidatesCount)
	{
		return false;
	}

	{
		for (unsigned i=0; i<table.size(); i++)
		{
			dataBaseCloud.reset();
			if (!dataBaseCloud.reserve(4)) //we never know ;)
				return false;
			for (unsigned j=0; j<4; j++)
				dataBaseCloud.addPoint(*dataCloud->getPoint(table[i].getIndex(j)));

			if (!RegistrationTools::RegistrationProcedure(&dataBaseCloud, &referenceBaseCloud, t, false))
				return false;

			tarray.push_back(t);
			if (filter)
			{
				float score = 0;
				GenericIndexedCloud* b = PointProjectionTools::applyTransformation(&dataBaseCloud, t);
				if (!b)
					return false; //not enough memory
				for (unsigned j=0; j<4; j++)
				{
					const CCVector3* q = b->getPoint(j);
					score += static_cast<float>((*q - *(p[j])).norm());
				}
				delete b;
				scores.push_back(score);
				sortedscores.push_back(score);
			}
		}
	}

	if (filter)
	{
		transforms.resize(0);
		try
		{
			candidates.resize(nbMaxCandidates);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}

		//Sort the scores in ascending order and only keep the nbMaxCandidates smallest scores
		sort(sortedscores.begin(), sortedscores.end());
		float score = sortedscores[nbMaxCandidates - 1];
		unsigned j = 0;
		for (unsigned i = 0; i < scores.size(); i++)
		{
			if (scores[i] <= score && j < nbMaxCandidates)
			{
				candidates[i].copy(table[i]);
				transforms.push_back(tarray[i]);
				j++;
			}
		}
	}
	else
	{
		transforms = tarray;
	}

	return true;
}
