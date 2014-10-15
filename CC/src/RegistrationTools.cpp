//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "RegistrationTools.h"

//local
#include "Matrix.h"
#include "GenericProgressCallback.h"
#include "GenericCloud.h"
#include "GenericIndexedCloudPersist.h"
#include "ReferenceCloud.h"
#include "DgmOctree.h"
#include "DistanceComputationTools.h"
#include "CCConst.h"
#include "CloudSamplingTools.h"
#include "ScalarFieldTools.h"
#include "NormalDistribution.h"
#include "ManualSegmentationTools.h"
#include "GeometricalAnalysisTools.h"
#include "KdTree.h"
#include "SimpleCloud.h"

//system
#include <time.h>
#include <algorithm>
#include <assert.h>

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

ICPRegistrationTools::RESULT_TYPE ICPRegistrationTools::RegisterClouds(	GenericIndexedCloudPersist* _modelCloud,
																		GenericIndexedCloudPersist* _dataCloud,
																		ScaledTransformation& transform,
																		CONVERGENCE_TYPE convType,
																		double minErrorDecrease,
																		unsigned nbMaxIterations,
																		double& finalError,
																		bool adjustScale/*=false*/,
																		GenericProgressCallback* progressCb/*=0*/,
																		bool filterOutFarthestPoints/*=false*/,
																		unsigned samplingLimit/*=20000*/,
																		ScalarField* modelWeights/*=0*/,
																		ScalarField* dataWeights/*=0*/,
																		int filters/*=SKIP_NONE*/)
{
	assert(_modelCloud && _dataCloud);

	finalError = -1.0;

	//MODEL CLOUD (reference, won't move)
	GenericIndexedCloudPersist* modelCloud = _modelCloud;
	ScalarField* _modelWeights = modelWeights;
	{
		if (_modelCloud->size() > samplingLimit) //shall we resample the clouds? (speed increase)
		{
			ReferenceCloud* subModelCloud = CloudSamplingTools::subsampleCloudRandomly(_modelCloud,samplingLimit);
			if (subModelCloud && modelWeights)
			{
				_modelWeights = new ScalarField("ResampledModelWeights");
				unsigned realCount = subModelCloud->size();
				if (_modelWeights->reserve(realCount))
				{
					for (unsigned i=0; i<realCount; ++i)
						_modelWeights->addElement(modelWeights->getValue(subModelCloud->getPointGlobalIndex(i)));
					_modelWeights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					delete subModelCloud;
					subModelCloud = 0;
				}
			}
			modelCloud = subModelCloud;
		}
		if (!modelCloud) //something bad happened
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
	}

	//DATA CLOUD (will move)
	ReferenceCloud* dataCloud = 0;
	ScalarField* _dataWeights = dataWeights;
	SimpleCloud* rotatedDataCloud = 0; //temporary structure (rotated vertices)
	{
		if (_dataCloud->size() > samplingLimit) //shall we resample the clouds? (speed increase)
		{
			dataCloud = CloudSamplingTools::subsampleCloudRandomly(_dataCloud,samplingLimit);
			if (dataCloud && dataWeights)
			{
				_dataWeights = new ScalarField("ResampledDataWeights");
				unsigned realCount = dataCloud->size();
				if (_dataWeights->reserve(realCount))
				{
					for (unsigned i=0; i<realCount; ++i)
						_dataWeights->addElement(dataWeights->getValue(dataCloud->getPointGlobalIndex(i)));
					_dataWeights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					delete dataCloud;
					dataCloud = 0;
				}
			}
		}
		else
		{
			//create a 'fake' reference cloud with all points
			dataCloud = new ReferenceCloud(_dataCloud);
			if (!dataCloud->addPointIndex(0,_dataCloud->size())) //not enough memory
			{
				delete dataCloud;
				dataCloud = 0;
			}
		}

		if (!dataCloud || !dataCloud->enableScalarField()) //something bad happened
		{
			if (dataCloud)
				delete dataCloud;
			if (modelCloud && modelCloud != _modelCloud)
				delete modelCloud;
			if (_modelWeights && _modelWeights != modelWeights)
				_modelWeights->release();
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}
	}

	//Closest Point Set (see ICP algorithm)
	ReferenceCloud* CPSet = new ReferenceCloud(modelCloud);
	ScalarField* CPSetWeights = _modelWeights ? new ScalarField("CPSetWeights") : 0;

	//algorithm result
	RESULT_TYPE result = ICP_NOTHING_TO_DO;
	unsigned iteration = 0;
	double error = 0.0;

	//we compute the initial distance between the two clouds (and the CPSet by the way)
	dataCloud->forEach(ScalarFieldTools::SetScalarValueToNaN);
	DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
	params.CPSet = CPSet;
	if (DistanceComputationTools::computeHausdorffDistance(dataCloud,modelCloud,params,progressCb) >= 0)
	{
		//12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
		error = ScalarFieldTools::computeMeanSquareScalarValue(dataCloud); //we only have positive SF values as we use the Hausdorff distance!
	}
	else
	{
		//if an error occurred during distances computation...
		error = -1.0;
		result = ICP_ERROR_DIST_COMPUTATION;
	}

	if (error > 0.0)
	{
#ifdef _DEBUG
		FILE* fp = fopen("registration_trace_log.txt","wt");
		if (fp)
			fprintf(fp,"Initial error: %f\n",error);
#endif

		double lastError = error, initialErrorDelta = 0.0, errorDelta = 0.0;
		result = ICP_APPLY_TRANSFO; //as soon as we do at least one iteration, we'll have to apply a transformation

		while (true)
		{
			++iteration;

			//regarding the progress bar
			if (progressCb && iteration>1) //on the first iteration, we do... nothing
			{
				char buffer[256];
				//then on the second iteration, we init/show it
				if (iteration == 2)
				{
					initialErrorDelta = errorDelta;

					progressCb->reset();
					progressCb->setMethodTitle("Clouds registration");
					sprintf(buffer,"Initial mean square error = %f\n",lastError);
					progressCb->setInfo(buffer);
					progressCb->start();
				}
				else //and after we update it continuously
				{
					sprintf(buffer,"Mean square error = %f [%f]\n",error,-errorDelta);
					progressCb->setInfo(buffer);
					progressCb->update((float)((initialErrorDelta-errorDelta)/(initialErrorDelta-minErrorDecrease)*100.0));
				}

				if (progressCb->isCancelRequested())
					break;
			}

			//shall we remove points with distance above a given threshold?
			if (filterOutFarthestPoints)
			{
				NormalDistribution N;
				N.computeParameters(dataCloud);
				if (N.isValid())
				{
					ScalarType mu,sigma2;
					N.getParameters(mu,sigma2);

					ReferenceCloud* c = new ReferenceCloud(dataCloud->getAssociatedCloud());
					ReferenceCloud* newCPSet = new ReferenceCloud(CPSet->getAssociatedCloud()); //we must also update the CPSet!
					ScalarField* newdataWeights = (_dataWeights ? new ScalarField("ResampledDataWeights") : 0);
					//unsigned realCount = dataCloud->size();
					//if (_dataWeights->reserve(realCount))
					//{
					//	for (unsigned i=0; i<realCount; ++i)
					//		_dataWeights->addElement(dataWeights->getValue(dataCloud->getPointGlobalIndex(i)));
					//	_dataWeights->computeMinAndMax();
					//}
					//else
					//{
					//	//not enough memory
					//	delete dataCloud;
					//	dataCloud = 0;
					//}

					unsigned n = dataCloud->size();
					if (!c->reserve(n) || !newCPSet->reserve(n) || (newdataWeights && !newdataWeights->reserve(n)))
					{
						//not enough memory
						delete c;
						delete newCPSet;
						if (newdataWeights)
							newdataWeights->release();
						result = ICP_ERROR_REGISTRATION_STEP;
						break;
					}

					//we keep only the points with "not too high" distances
					ScalarType maxDist = mu+3.0f*sqrt(sigma2);
					unsigned realSize = 0;
					for (unsigned i=0; i<n; ++i)
					{
						unsigned index = dataCloud->getPointGlobalIndex(i);
						if (dataCloud->getAssociatedCloud()->getPointScalarValue(index)<maxDist)
						{
							c->addPointIndex(index); //can't fail, see above
							newCPSet->addPointIndex(CPSet->getPointGlobalIndex(i)); //can't fail, see above
							if (newdataWeights)
								newdataWeights->addElement(_dataWeights->getValue(index));
							++realSize;
						}
					}

					//resize should be ok as we have called reserve first
					c->resize(realSize); //should always be ok as counter<n
					newCPSet->resize(realSize); //idem
					if (newdataWeights)
						newdataWeights->resize(realSize); //idem

					//replace old structures by new ones
					delete CPSet;
					CPSet = newCPSet;
					delete dataCloud;
					dataCloud = c;
					if (_dataWeights)
					{
						_dataWeights->release();
						_dataWeights = newdataWeights;
					}
				}
			}

			//update CPSet weights (if any)
			if (_modelWeights)
			{
				unsigned count = CPSet->size();
				assert(CPSetWeights);
				if (CPSetWeights->currentSize() != count)
				{
					if (!CPSetWeights->resize(count))
					{
						result = ICP_ERROR_REGISTRATION_STEP;
						break;
					}
				}
				for (unsigned i=0; i<count; ++i)
					CPSetWeights->setValue(i,_modelWeights->getValue(CPSet->getPointGlobalIndex(i)));
				CPSetWeights->computeMinAndMax();
			}

			//single iteration of the registration procedure
			ScaledTransformation currentTrans;
			if (!RegistrationTools::RegistrationProcedure(dataCloud, CPSet, currentTrans, adjustScale, _dataWeights, _modelWeights))
			{
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}

			//shall we filter some components of the resulting transformation?
			if (filters != SKIP_NONE)
			{
				//filter translation (in place)
				FilterTransformation(currentTrans,filters,currentTrans);
			}

			//get rotated data cloud
			if (!rotatedDataCloud || filterOutFarthestPoints)
			{
				//we create a new structure, with rotated points
				SimpleCloud* newDataCloud = PointProjectionTools::applyTransformation(dataCloud, currentTrans);
				if (!newDataCloud)
				{
					//not enough memory
					result = ICP_ERROR_REGISTRATION_STEP;
					break;
				}
				//update dataCloud
				if (rotatedDataCloud)
					delete rotatedDataCloud;
				rotatedDataCloud = newDataCloud;
				delete dataCloud;

				dataCloud = new ReferenceCloud(rotatedDataCloud);
				if (!dataCloud->addPointIndex(0,rotatedDataCloud->size()))
				{
					//not enough memory
					delete dataCloud;
					result = ICP_ERROR_REGISTRATION_STEP;
					break;
				}
			}
			else
			{
				//we simply have to rotate the existing temporary cloud
				rotatedDataCloud->applyTransformation(currentTrans);
			}

			//compute (new) distances to model
			params.CPSet = CPSet;
			if (DistanceComputationTools::computeHausdorffDistance(dataCloud,modelCloud,params) < 0)
			{
				//an error occurred during distances computation...
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}

			lastError = error;
			//12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
			error = ScalarFieldTools::computeMeanSquareScalarValue(dataCloud); //we only have positive SF values as we use the Hausdorff distance!
			finalError = (error>0 ? sqrt(error) : error);

#ifdef _DEBUG
			if (fp)
				fprintf(fp,"Iteration #%u --> error: %f\n",iteration,error);
#endif

			//error update
			errorDelta = lastError-error;

			//is it better?
			if (errorDelta > 0.0)
			{
				//we update global transformation matrix
				if (currentTrans.R.isValid())
				{
					if (transform.R.isValid())
						transform.R = currentTrans.R * transform.R;
					else
						transform.R = currentTrans.R;

					transform.T = currentTrans.R * transform.T;
				}

				if (adjustScale)
				{
					transform.s *= currentTrans.s;
					transform.T *= currentTrans.s;
				}

				transform.T += currentTrans.T;
			}

			//stop criterion
			if ((errorDelta < 0.0) || //error increase
				(convType == MAX_ERROR_CONVERGENCE && errorDelta < minErrorDecrease) || //convergence reached
				(convType == MAX_ITER_CONVERGENCE && iteration > nbMaxIterations)) //max iteration reached
			{
				break;
			}
		}

		if (progressCb)
			progressCb->stop();

#ifdef _DEBUG
		if (fp)
		{
			fclose(fp);
			fp = 0;
		}
#endif
	}

	if (CPSet)
		delete CPSet;
	CPSet = 0;
	if (CPSetWeights)
		CPSetWeights->release();

	//release memory
	if (modelCloud && modelCloud != _modelCloud)
		delete modelCloud;
	if (_modelWeights && _modelWeights != modelWeights)
		_modelWeights->release();
	if (dataCloud)
		delete dataCloud;
	if (_dataWeights && _dataWeights != dataWeights)
		_dataWeights->release();
	if (rotatedDataCloud)
		delete rotatedDataCloud;

	return result;
}

bool HornRegistrationTools::FindAbsoluteOrientation(GenericCloud* lCloud,
													GenericCloud* rCloud,
													ScaledTransformation& trans,
													bool fixedScale/*=false*/)
{
	return RegistrationProcedure(lCloud,rCloud,trans,!fixedScale);
}

double HornRegistrationTools::ComputeRMS(GenericCloud* lCloud,
										 GenericCloud* rCloud,
										 const ScaledTransformation& trans)
{
	assert(rCloud && lCloud);
	if (!rCloud || !lCloud || rCloud->size() != lCloud->size() || rCloud->size()<3)
		return false;

	double rms = 0.0;

	rCloud->placeIteratorAtBegining();
	lCloud->placeIteratorAtBegining();
	unsigned count = rCloud->size();
			
	for (unsigned i=0; i<count; i++)
	{
		const CCVector3* Ri = rCloud->getNextPoint();
		const CCVector3* Li = lCloud->getNextPoint();
		CCVector3 Lit = (trans.R.isValid() ? trans.R * (*Li) : (*Li))*trans.s + trans.T;

#ifdef _DEBUG
		double dist = (*Ri-Lit).norm();
#endif

		rms += (*Ri-Lit).norm2();
	}

	return sqrt(rms/(double)count);
}

bool RegistrationTools::RegistrationProcedure(	GenericCloud* P,
												GenericCloud* X,
												ScaledTransformation& trans,
												bool adjustScale/*=false*/,
												ScalarField* weightsP/*=0*/,
												ScalarField* weightsX/*=0*/,
												PointCoordinateType aPrioriScale/*=1.0f*/)
{
	//resulting transformation (R is invalid on initialization, T is (0,0,0) and s==1)
	trans.R.invalidate();
	trans.T = CCVector3(0,0,0);
	trans.s = PC_ONE;

	if (P == 0 || X == 0 || P->size() != X->size() || P->size() < 4)
		return false;

	PointCoordinateType bbMin[3],bbMax[3];
	X->getBoundingBox(bbMin,bbMax);

	//BBox dimensions
	PointCoordinateType dx = bbMax[0]-bbMin[0];
	PointCoordinateType dy = bbMax[1]-bbMin[1];
	PointCoordinateType dz = bbMax[2]-bbMin[2];

	//centers of mass
	CCVector3 Gp = GeometricalAnalysisTools::computeGravityCenter(P);
	CCVector3 Gx = GeometricalAnalysisTools::computeGravityCenter(X);

	//if the data cloud is equivalent to a single point (for instance
	//it's the case when the two clouds are very far away from
	//each other in the ICP process) we try to get the two clouds closer
	if (fabs(dx)+fabs(dy)+fabs(dz) < ZERO_TOLERANCE)
	{
		trans.T = Gx - Gp*aPrioriScale;
		return true;
	}

	//Cross covariance matrix, eq #24 in Besl92 (but with weights, if any)
	SquareMatrixd Sigma_px = (weightsP || weightsX	? GeometricalAnalysisTools::computeWeightedCrossCovarianceMatrix(P,X,Gp,Gx,weightsP,weightsX)
													: GeometricalAnalysisTools::computeCrossCovarianceMatrix(P,X,Gp,Gx) );
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
	SquareMatrixd eig = QSigma.computeJacobianEigenValuesAndVectors();

	if (!eig.isValid())
		return false;

	//as Besl says, the best rotation corresponds to the eigenvector associated to the biggest eigenvalue
	double qR[4];
	eig.getMaxEigenValueAndVector(qR);

	//these eigenvalue and eigenvector correspond to a quaternion --> we get the corresponding matrix
	trans.R.initFromQuaternion(qR);

	if (adjustScale)
	{
		//two accumulators
		double acc_num = 0.0;
		double acc_denom = 0.0;

		//now deduce the scale (refer to "Point Set Registration with Integrated Scale Estimation", Zinsser et. al, PRIP 2005)
		X->placeIteratorAtBegining();
		P->placeIteratorAtBegining();

		unsigned count = X->size();
		assert(P->size() == count);
		for (unsigned i=0; i<count; ++i)
		{
			//'a' refers to the data 'A' (moving) = P
			//'b' refers to the model 'B' (not moving) = X
			CCVector3 a_tilde = trans.R * (*(P->getNextPoint()) - Gp);	// a_tilde_i = R * (a_i - a_mean)
			CCVector3 b_tilde = (*(X->getNextPoint()) - Gx);			// b_tilde_j =     (b_j - b_mean)

			acc_num += (double)b_tilde.dot(a_tilde);
			acc_denom += (double)a_tilde.dot(a_tilde);
		}

		//DGM: acc_2 can't be 0 because we already have checked that the bbox is not a single point!
		assert(acc_denom > 0.0);
		trans.s = static_cast<PointCoordinateType>(fabs(acc_num / acc_denom));
	}

	//and we deduce the translation
	trans.T = Gx - (trans.R*Gp)*(aPrioriScale*trans.s); //#26 in besl paper, modified with the scale as in jschmidt

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
	unsigned i, j;
	unsigned bestScore, score;
	Base reference;
	std::vector<Base> candidates;
	std::vector<ScaledTransformation> transforms;
	ScaledTransformation RT;
	KDTree *dataTree, *modelTree;
	CCVector3 min, max, diff;

	/*DGM: KDTree::buildFromCloud call reset right away!
	if (progressCb)
	{
	progressCb->reset();
	progressCb->setMethodTitle("Clouds registration");
	progressCb->setInfo("Starting 4PCS");
	progressCb->start();
	}
	//*/

	//Initialize random seed with current time
	srand((unsigned)time(0));

	bestScore = score = 0;
	transform.R.invalidate();
	transform.T = CCVector3(0,0,0);

	//Adapt overlap to the model cloud size
	modelCloud->getBoundingBox(min.u, max.u);
	diff = max-min;
	overlap *= diff.norm() / 2;

	//Buil the associated KDtrees
	dataTree = new KDTree();
	if (!dataTree->buildFromCloud(dataCloud, progressCb))
	{
		delete dataTree;
		return false;
	}
	modelTree = new KDTree();
	if (!modelTree->buildFromCloud(modelCloud, progressCb))
	{
		delete dataTree;
		delete modelTree;
		return false;
	}

	//if (progressCb)
	//    progressCb->stop();

	for(i=0; i<nbBases; i++)
	{
		//Randomly find the current reference base
		if (!FindBase(modelCloud, overlap, nbTries, reference))
			continue;

		//Search for all the congruent bases in the second cloud
		candidates.clear();
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
		for(j=0; j<4; j++)
			referenceBasePoints[j] = modelCloud->getPoint(reference.getIndex(j));
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
		if (!FilterCandidates(modelCloud, dataCloud, reference, candidates, nbMaxCandidates, transforms))
		{
			delete dataTree;
			delete modelTree;
			transform.R = SquareMatrix();
			return false;
		}

		for(j=0; j<candidates.size(); j++)
		{
			//Register the current candidate base with the reference base
			RT = transforms[j];
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

		if (progressCb)
		{
			char buffer[256];
			sprintf(buffer,"Trial %u/%u [best score = %u]\n",i+1,nbBases,bestScore);
			progressCb->setInfo(buffer);
			progressCb->update(((float)(i+1)*100.0f)/(float)nbBases);

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
		progressCb->stop();

	return (bestScore > 0);
}


 unsigned FPCSRegistrationTools::ComputeRegistrationScore(	KDTree *modelTree,
															GenericIndexedCloud *dataCloud,
															ScalarType delta,
															ScaledTransformation& dataToModel)
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
	unsigned a, b, c, d, t1, t2;
	unsigned i, size;
	PointCoordinateType f, best, d0, d1, d2, x, y, z, w;
	const CCVector3 *p0, *p1, *p2, *p3;
	CCVector3 normal, u, v;

	overlap *= overlap;
	size = cloud->size();
	best = 0.;
	b = c = 0;
	a = rand()%size;
	p0 = cloud->getPoint(a);
	//Randomly pick 3 points as sparsed as possible
	for(i=0; i<nbTries; i++)
	{
		t1 = rand()%size;
		t2 = rand()%size;
		if (t1 == a || t2 == a || t1 == t2)
			continue;

		p1 = cloud->getPoint(t1);
		p2 = cloud->getPoint(t2);
		//Checked that the selected points are not more than overlap-distant from p0
		u = *p1-*p0;
		if (u.norm2() > overlap)
			continue;
		u = *p2-*p0;
		if (u.norm2() > overlap)
			continue;

		//compute [p0, p1, p2] area thanks to cross product
		x = ((p1->y-p0->y)*(p2->z-p0->z))-((p1->z-p0->z)*(p2->y-p0->y));
		y = ((p1->z-p0->z)*(p2->x-p0->x))-((p1->x-p0->x)*(p2->z-p0->z));
		z = ((p1->x-p0->x)*(p2->y-p0->y))-((p1->y-p0->y)*(p2->x-p0->x));
		//don't need to compute the true area : f=(area²)*2 is sufficient for comparison
		f = x*x + y*y + z*z;
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
	if (f<=0.)
		return false;
	normal *= 1.0f/f;
	//plane equation : p lies in the plane if x*p[0] + y*p[1] + z*p[2] + w = 0
	x = normal.x;
	y = normal.y;
	z = normal.z;
	w = -(x*p0->x)-(y*p0->y)-(z*p0->z);
	d = a;
	best = -1.;
	p1 = cloud->getPoint(b);
	p2 = cloud->getPoint(c);
	for(i=0; i<nbTries; i++)
	{
		t1 = rand()%size;
		if (t1 == a || t1 == b || t1 == c)
			continue;
		p3 = cloud->getPoint(t1);
		//p3 must be close enough to at least two other points (considering overlap)
		d0 = (*p3 - *p0).norm2();
		d1 = (*p3 - *p1).norm2();
		d2 = (*p3 - *p2).norm2();
		if ((d0>=overlap && d1>=overlap) || (d0>=overlap && d2>=overlap) || (d1>=overlap && d2>=overlap))
			continue;
		//Compute distance to the plane (cloud[a], cloud[b], cloud[c])
		f = fabs((x*p3->x)+(y*p3->y)+(z*p3->z)+w);
		//keep the point which is the closest to the plane, while being as far as possible from the other three points
		f=(f+1.0f)/(sqrt(d0)+sqrt(d1)+sqrt(d2));
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
typedef std::pair<unsigned,unsigned> IndexPair;

int FPCSRegistrationTools::FindCongruentBases(KDTree* tree,
												ScalarType delta,
												const CCVector3* base[4],
												std::vector<Base>& results)
{
	//Compute reference base invariants (r1, r2)
	PointCoordinateType r1, r2, d1, d2;
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
	std::vector<IndexPair> pairs1, pairs2;
	{
		unsigned count = (unsigned)cloud->size();
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

		for (unsigned i=0; i<count; i++)
		{
			const CCVector3 *q0 = cloud->getPoint(i);
			IndexPair idxPair;
			idxPair.first = i;
			//Extract all points from the cloud which are d1-appart (up to delta) from q0
			pointsIndexes.clear();
			tree->findPointsLyingToDistance(q0->u, static_cast<ScalarType>(d1), delta, pointsIndexes);
			{
				for(size_t j=0; j<pointsIndexes.size(); j++)
				{
					//As ||pi-pj|| = ||pj-pi||, we only take care of pairs that verify i<j
					if (pointsIndexes[j]>i)
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
				for(size_t j=0; j<pointsIndexes.size(); j++)
				{
					if (pointsIndexes[j]>i)
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
		SimpleCloud tmpCloud1,tmpCloud2;
		{
			unsigned count = (unsigned)pairs1.size();
			if (!tmpCloud1.reserve(count*2)) //not enough memory
				return -2;
			for(unsigned i=0; i<count; i++)
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
			unsigned count = (unsigned)pairs2.size();
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
			unsigned count = (unsigned)tmpCloud2.size();
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
		results.clear();
		size_t count = match.size();
		if (count>0)
		{
			results.reserve(count);
			if (results.capacity() < count)		//not enough memory
				return -6;
			for(size_t i=0; i<count; i++)
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

	return (int)results.size();
}


bool FPCSRegistrationTools::LinesIntersections(	const CCVector3 &p0,
												const CCVector3 &p1,
												const CCVector3 &p2,
												const CCVector3 &p3,
												CCVector3 &inter,
												PointCoordinateType& lambda,
												PointCoordinateType& mu)
{
	CCVector3 p02, p32, p10, A, B;
	PointCoordinateType num, denom;

	//Find lambda and mu such that :
	//A = p0+lambda(p1-p0)
	//B = p2+mu(p3-p2)
	//(lambda, mu) = argmin(||A-B||²)
	p02 = p0-p2;
	p32 = p3-p2;
	p10 = p1-p0;
	num = (p02.dot(p32) * p32.dot(p10)) - (p02.dot(p10) * p32.dot(p32));
	denom = (p10.dot(p10) * p32.dot(p32)) - (p32.dot(p10) * p32.dot(p10));
	if (fabs(denom) < 0.00001)
		return false;
	lambda = num / denom;
	num = p02.dot(p32) + (lambda*p32.dot(p10));
	denom = p32.dot(p32);
	if (fabs(denom) < 0.00001)
		return false;
	mu = num / denom;
	A.x = p0.x+(lambda*p10.x);
	A.y = p0.y+(lambda*p10.y);
	A.z = p0.z+(lambda*p10.z);
	B.x = p2.x+(mu*p32.x);
	B.y = p2.y+(mu*p32.y);
	B.z = p2.z+(mu*p32.z);
	inter.x = (A.x+B.x)/2.0f;
	inter.y = (A.y+B.y)/2.0f;
	inter.z = (A.z+B.z)/2.0f;

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
	std::vector<float> scores, sortedscores;
	const CCVector3 *p[4], *q;
	unsigned i, j;
	ScaledTransformation t;
	std::vector<ScaledTransformation> tarray;
	SimpleCloud referenceBaseCloud, dataBaseCloud;

	unsigned candidatesCount = (unsigned)candidates.size();
	if (candidatesCount == 0)
		return false;

	bool filter = (nbMaxCandidates>0 && candidatesCount>nbMaxCandidates);
	try
	{
		table.resize(candidatesCount);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}
	for(i=0; i<candidatesCount; i++)
		table[i].copy(candidates[i]);

	if (!referenceBaseCloud.reserve(4)) //we never know ;)
		return false;

	for(j=0; j<4; j++)
	{
		p[j] = modelCloud->getPoint(reference.getIndex(j));
		referenceBaseCloud.addPoint(*p[j]);
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
	if (scores.capacity() < candidatesCount 
		|| sortedscores.capacity() < candidatesCount 
		|| tarray.capacity() < candidatesCount 
		|| transforms.capacity() < candidatesCount)
	{
		return false;
	}

	for(i=0; i<table.size(); i++)
	{
		dataBaseCloud.clear();
		if (!dataBaseCloud.reserve(4)) //we never know ;)
			return false;
		for(j=0; j<4; j++)
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
			for (j=0; j<4; j++)
			{
				q = b->getPoint(j);
				score += static_cast<float>((*q - *(p[j])).norm());
			}
			delete b;
			scores.push_back(score);
			sortedscores.push_back(score);
		}
	}

	if (filter)
	{
		transforms.clear();
		candidates.clear();
		try
		{
			candidates.resize(nbMaxCandidates);
		}
		catch (.../*const std::bad_alloc&*/) //out of memory
		{
			return false;
		}
		candidatesCount=nbMaxCandidates;

		//Sort the scores in ascending order and only keep the nbMaxCandidates smallest scores
		sort(sortedscores.begin(), sortedscores.end());
		float score = sortedscores[nbMaxCandidates-1];
		j = 0;
		for(i=0; i<scores.size(); i++)
		{
			if (scores[i]<=score && j<nbMaxCandidates)
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

