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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#include "RegistrationTools.h"

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

#include <time.h>
#include <algorithm>
#include <assert.h>

using namespace CCLib;

ICPRegistrationTools::CC_ICP_RESULT ICPRegistrationTools::RegisterClouds(GenericIndexedCloudPersist* _modelCloud,
																			GenericIndexedCloudPersist* _dataCloud,
																			PointProjectionTools::Transformation& transform,
																			CC_ICP_CONVERGENCE_TYPE convType,
																			double minErrorDecrease,
																			unsigned nbMaxIterations,
																			double& finalError,
																			GenericProgressCallback* progressCb/*=0*/,
																			bool filterOutFarthestPoints/*=false*/,
																			unsigned samplingLimit/*=20000*/,
																			ScalarField* modelWeights/*=0*/,
																			ScalarField* dataWeights/*=0*/)
{
    assert(_modelCloud && _dataCloud);

    finalError = -1.0;

	//MODEL CLOUD (reference, won't move)
	GenericIndexedCloudPersist* modelCloud=_modelCloud;
	ScalarField* _modelWeights=modelWeights;
	{
		if (_modelCloud->size()>samplingLimit) //shall we resample the clouds? (speed increase)
		{
			ReferenceCloud* subModelCloud = CloudSamplingTools::subsampleCloudRandomly(_modelCloud,samplingLimit);
			if (subModelCloud && modelWeights)
			{
				_modelWeights = new ScalarField("ResampledModelWeights",modelWeights->isPositive());
				unsigned realCount = subModelCloud->size();
				if (_modelWeights->reserve(realCount))
				{
					for (unsigned i=0;i<realCount;++i)
						_modelWeights->addElement(modelWeights->getValue(subModelCloud->getPointGlobalIndex(i)));
					_modelWeights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					delete subModelCloud;
					subModelCloud=0;
				}
			}
			modelCloud = subModelCloud;
		}
		if (!modelCloud) //something bad happened
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
	}

	//DATA CLOUD (will move)
	ReferenceCloud* dataCloud=0;
	ScalarField* _dataWeights=dataWeights;
	SimpleCloud* rotatedDataCloud=0; //temporary structure (rotated vertices)
	{
		if (_dataCloud->size()>samplingLimit) //shall we resample the clouds? (speed increase)
		{
			dataCloud = CloudSamplingTools::subsampleCloudRandomly(_dataCloud,samplingLimit);
			if (dataCloud && dataWeights)
			{
				_dataWeights = new ScalarField("ResampledDataWeights",dataWeights->isPositive());
				unsigned realCount = dataCloud->size();
				if (_dataWeights->reserve(realCount))
				{
					for (unsigned i=0;i<realCount;++i)
						_dataWeights->addElement(dataWeights->getValue(dataCloud->getPointGlobalIndex(i)));
					_dataWeights->computeMinAndMax();
				}
				else
				{
					//not enough memory
					delete dataCloud;
					dataCloud=0;
				}
			}
		}
		else
		{
			//create a 'fake' reference cloud with all points
			dataCloud = new ReferenceCloud(_dataCloud);
			if (dataCloud->reserve(_dataCloud->size()))
			{
				dataCloud->addPointIndex(0,_dataCloud->size());
			}
			else //not enough memory
			{
				delete dataCloud;
				dataCloud=0;
			}
		}

		if (!dataCloud || !dataCloud->enableScalarField()) //something bad happened
		{
			if (dataCloud)
				delete dataCloud;
			if (modelCloud && modelCloud != _modelCloud)
				delete modelCloud;
			if (_modelWeights && _modelWeights!=modelWeights)
				_modelWeights->release();
			return ICP_ERROR_NOT_ENOUGH_MEMORY;
		}
	}

	//Closest Point Set (see ICP algorithm)
	ReferenceCloud* CPSet = new ReferenceCloud(modelCloud);
	ScalarField* CPSetWeights = _modelWeights ? new ScalarField("CPSetWeights",_modelWeights->isPositive()) : 0;

	//algorithm result
	CC_ICP_RESULT result = ICP_NOTHING_TO_DO;
	unsigned iteration = 0;
	double error = 0.0;

    //we compute the initial distance between the two clouds (and the CPSet by the way)
    dataCloud->forEach(ScalarFieldTools::razDistsToHiddenValue);
	DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
	params.CPSet = CPSet;
	if (DistanceComputationTools::computeHausdorffDistance(dataCloud,modelCloud,params,progressCb)>=0)
	{
		//12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
		error = DistanceComputationTools::computeMeanSquareDist(dataCloud);
	}
	else
	{
	    //if an error occured during distances computation...
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

		double lastError=error,initialErrorDelta=0.0,errorDelta=0.0;
		result = ICP_APPLY_TRANSFO; //as soon as we do at least one iteration, we'll have to apply a transformation

		while (true)
		{
			++iteration;

			//regarding the progress bar
			if (progressCb && iteration>1) //on the first iteration, we do... nothing
			{
				char buffer[256];
				//then on the second iteration, we init/show it
				if (iteration==2)
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
				N.computeParameters(dataCloud,false);
				if (N.isValid())
				{
					DistanceType mu,sigma2;
					N.getParameters(mu,sigma2);

					ReferenceCloud* c = new ReferenceCloud(dataCloud->getAssociatedCloud());
					ReferenceCloud* newCPSet = new ReferenceCloud(CPSet->getAssociatedCloud()); //we must also update the CPSet!
					ScalarField* newdataWeights = (_dataWeights ? new ScalarField("ResampledDataWeights",_dataWeights->isPositive()) : 0);
				//unsigned realCount = dataCloud->size();
				//if (_dataWeights->reserve(realCount))
				//{
				//	for (unsigned i=0;i<realCount;++i)
				//		_dataWeights->addElement(dataWeights->getValue(dataCloud->getPointGlobalIndex(i)));
				//	_dataWeights->computeMinAndMax();
				//}
				//else
				//{
				//	//not enough memory
				//	delete dataCloud;
				//	dataCloud=0;
				//}

					unsigned n=dataCloud->size();
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
					DistanceType maxDist = mu+3.0f*sqrt(sigma2);
					unsigned realSize=0;
					for (unsigned i=0;i<n;++i)
					{
						unsigned index = dataCloud->getPointGlobalIndex(i);
						if (dataCloud->getAssociatedCloud()->getPointScalarValue(index)<maxDist)
						{
							c->addPointIndex(index);
							newCPSet->addPointIndex(CPSet->getPointGlobalIndex(i));
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
					CPSet=newCPSet;
					delete dataCloud;
					dataCloud=c;
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
				unsigned count=CPSet->size();
				assert(CPSetWeights);
				if (CPSetWeights->currentSize()!=count)
				{
					if (!CPSetWeights->resize(count))
					{
						result = ICP_ERROR_REGISTRATION_STEP;
						break;
					}
				}
				for (unsigned i=0;i<count;++i)
					CPSetWeights->setValue(i,_modelWeights->getValue(CPSet->getPointGlobalIndex(i)));
				CPSetWeights->computeMinAndMax();
			}

            PointProjectionTools::Transformation currentTrans;
			//if registration procedure fails
            if (!RegistrationTools::RegistrationProcedure(dataCloud, CPSet, currentTrans, _dataWeights, _modelWeights))
			{
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}

			//get rotated data cloud
			if (!rotatedDataCloud || filterOutFarthestPoints)
			{
				//we create a new structure, with rotated points
				SimpleCloud* newDataCloud = PointProjectionTools::applyTransformation(dataCloud,currentTrans);
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
				if (!dataCloud->reserve(rotatedDataCloud->size()))
				{
					//not enough  memory
					result = ICP_ERROR_REGISTRATION_STEP;
					break;
				}
				dataCloud->addPointIndex(0,rotatedDataCloud->size());
			}
			else
			{
				//we simply have to rotate the existing temporary cloud
				rotatedDataCloud->applyTransformation(currentTrans);
			}

			//compute (new) distances to model
			params.CPSet = CPSet;
			if (DistanceComputationTools::computeHausdorffDistance(dataCloud,modelCloud,params)<0)
			{
                //an error occured during distances computation...
				result = ICP_ERROR_REGISTRATION_STEP;
				break;
			}

			lastError = error;
            //12/11/2008 - A.BEY: ICP guarantees only the decrease of the squared distances sum (not the distances sum)
			error = DistanceComputationTools::computeMeanSquareDist(dataCloud);
			finalError = (error>0 ? sqrt(error) : error);

#ifdef _DEBUG
			if (fp)
			    fprintf(fp,"Iteration #%i --> error: %f\n",iteration,error);
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
			fp=0;
		}
#endif
	}

	if (CPSet)
		delete CPSet;
	CPSet=0;
	if (CPSetWeights)
		CPSetWeights->release();

	//release memory
	if (modelCloud && modelCloud != _modelCloud)
        delete modelCloud;
	if (_modelWeights && _modelWeights!=modelWeights)
		_modelWeights->release();
	if (dataCloud)
		delete dataCloud;
	if (_dataWeights && _dataWeights!=dataWeights)
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
    //resulting transformation (R is invalid on initialization and T is (0,0,0))
    trans.R.invalidate();
    trans.T = CCVector3(0,0,0);
	trans.s = (PointCoordinateType)1.0;

	assert(rCloud && lCloud);
	if (!rCloud || !lCloud || rCloud->size() != lCloud->size() || rCloud->size()<3)
		return false;
	unsigned count = rCloud->size();
	assert(count>2);

	//determine best scale?
	if (!fixedScale)
	{
		CCVector3 Gr = GeometricalAnalysisTools::computeGravityCenter(rCloud);
		CCVector3 Gl = GeometricalAnalysisTools::computeGravityCenter(lCloud);

		//we determine scale with the symmetrical form as proposed by Horn
		double lNorm2Sum = 0.0;
		{		
			lCloud->placeIteratorAtBegining();
			for (unsigned i=0;i<count;i++)
			{
				CCVector3 Pi = *lCloud->getNextPoint()-Gl;
				lNorm2Sum += Pi.dot(Pi);
			}
		}

		if (lNorm2Sum >= ZERO_TOLERANCE)
		{
			double rNorm2Sum = 0.0;
			{
				rCloud->placeIteratorAtBegining();
				for (unsigned i=0;i<count;i++)
				{
					CCVector3 Pi = *rCloud->getNextPoint()-Gr;
					rNorm2Sum += Pi.dot(Pi);
				}
			}

			//resulting scale
			trans.s = (PointCoordinateType)sqrt(rNorm2Sum/lNorm2Sum);
		}
		//else
		//{
		//	//shouldn't happen!
		//}
	}

	return RegistrationProcedure(lCloud,rCloud,trans,0,0,trans.s);
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
			
	for (unsigned i=0;i<count;i++)
	{
		const CCVector3* Ri = rCloud->getNextPoint();
		const CCVector3* Li = lCloud->getNextPoint();
		CCVector3 Lit = (trans.R.isValid() ? trans.R * (*Li) : (*Li))*trans.s + trans.T;

		rms += (*Ri-Lit).norm2();
	}

	return sqrt(rms/(double)count);
}

bool RegistrationTools::RegistrationProcedure(GenericCloud* P,
											  GenericCloud* X,
											  PointProjectionTools::Transformation& trans,
											  ScalarField* weightsP/*=0*/,
											  ScalarField* weightsX/*=0*/,
											  PointCoordinateType scale/*=1.0f*/)
{
    //resulting transformation (R is invalid on initialization and T is (0,0,0))
    trans.R.invalidate();
    trans.T = CCVector3(0,0,0);

	PointCoordinateType bbMin[3],bbMax[3];
	X->getBoundingBox(bbMin,bbMax);

	PointCoordinateType dx = bbMax[0]-bbMin[0];
	PointCoordinateType dy = bbMax[1]-bbMin[1];
	PointCoordinateType dz = bbMax[2]-bbMin[2];

	CCVector3 Gp = GeometricalAnalysisTools::computeGravityCenter(P);
	CCVector3 Gx = GeometricalAnalysisTools::computeGravityCenter(X);

	//if the cloud is equivalent to a single point (for instance
	//it's the case when the two clouds are very far away from 
	//each other in the ICP process) we try to get the two clouds closer
	if (fabs(dx)+fabs(dy)+fabs(dz) < ZERO_TOLERANCE)
	{
	    trans.T = Gx - Gp*scale;
        return true;
	}

	//Cross covariance matrix
	SquareMatrixd Sigma_px = (weightsP || weightsX) ? GeometricalAnalysisTools::computeWeightedCrossCovarianceMatrix(P,X,Gp.u,Gx.u,weightsP,weightsX) : GeometricalAnalysisTools::computeCrossCovarianceMatrix(P,X,Gp.u,Gx.u);
	if (!Sigma_px.isValid())
		return false;

	SquareMatrixd Sigma_px_t = Sigma_px;
	Sigma_px_t.transpose();

	SquareMatrixd Aij = Sigma_px-Sigma_px_t;

	double trace = Sigma_px.trace();

	SquareMatrixd traceI3(3);
	traceI3.m_values[0][0]=trace;
	traceI3.m_values[1][1]=trace;
	traceI3.m_values[2][2]=trace;

	SquareMatrixd bottomMat = Sigma_px+Sigma_px_t-traceI3;

    //we build up the registration matrix (see ICP algorithm)
	SquareMatrixd QSigma(4);

	QSigma.m_values[0][0]=trace;

	QSigma.m_values[0][1]=QSigma.m_values[1][0]=Aij.m_values[1][2];
	QSigma.m_values[0][2]=QSigma.m_values[2][0]=Aij.m_values[2][0];
	QSigma.m_values[0][3]=QSigma.m_values[3][0]=Aij.m_values[0][1];

	QSigma.m_values[1][1]=bottomMat.m_values[0][0];
	QSigma.m_values[1][2]=bottomMat.m_values[0][1];
	QSigma.m_values[1][3]=bottomMat.m_values[0][2];

	QSigma.m_values[2][1]=bottomMat.m_values[1][0];
	QSigma.m_values[2][2]=bottomMat.m_values[1][1];
	QSigma.m_values[2][3]=bottomMat.m_values[1][2];

	QSigma.m_values[3][1]=bottomMat.m_values[2][0];
	QSigma.m_values[3][2]=bottomMat.m_values[2][1];
	QSigma.m_values[3][3]=bottomMat.m_values[2][2];

    //we compute its eigenvalues and eigenvectors
	SquareMatrixd eig = QSigma.computeJacobianEigenValuesAndVectors();
	if (!eig.isValid())
        return false;

	//as Besl says, the best rotation corresponds to the eigenvector associated to the biggest eigenvalue
    double qR[4];
	eig.getMaxEigenValueAndVector(qR);

    //these eigenvalue and eigenvector correspond to a quaternion --> we get the corresponding matrix
	trans.R.initFromQuaternion(qR);

    //and we deduce the translation
	trans.T = Gx - (trans.R*Gp)*scale;

	return true;
}

bool FPCSRegistrationTools::RegisterClouds(GenericIndexedCloud* modelCloud,
                                            GenericIndexedCloud* dataCloud,
                                            PointProjectionTools::Transformation& transform,
                                            float delta,
                                            float beta,
                                            float overlap,
                                            unsigned nbBases,
                                            unsigned nbTries,
                                            GenericProgressCallback* progressCb,
                                            unsigned nbMaxCandidates)
{
    unsigned i, j;
    unsigned bestScore, score;
    Base reference;
    std::vector<Base> candidates;
    std::vector<PointProjectionTools::Transformation> transforms;
    PointProjectionTools::Transformation RT;
    KDTree *dataTree, *modelTree;
    CCVector3 min, max, diff;

    /*DGM: KDTree::BuildFromCloud call reset right away!
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
    overlap *= diff.norm()/2.0f;

    //Buil the associated KDtrees
    dataTree = new KDTree();
    if (!dataTree->BuildFromCloud(dataCloud, progressCb))
    {
        delete dataTree;
        return false;
    }
    modelTree = new KDTree();
    if (!modelTree->BuildFromCloud(modelCloud, progressCb))
    {
        delete dataTree;
        delete modelTree;
        return false;
    }

    //if(progressCb)
    //    progressCb->stop();

    for(i=0; i<nbBases; i++)
    {
        //Randomly find the current reference base
        if(!FindBase(modelCloud, overlap, nbTries, reference))
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
		else if (result<0) //something bad happened!
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
            if(RT.R.isValid())
            {
                score = ComputeRegistrationScore(modelTree, dataCloud, delta, RT);

                //Keep parameters that lead to the best result
                if(score > bestScore)
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
            sprintf(buffer,"Trial %d/%d [best score = %d]\n",i+1,nbBases,bestScore);
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

    if(progressCb)
        progressCb->stop();

    return (bestScore > 0);
}


 unsigned FPCSRegistrationTools::ComputeRegistrationScore(
        KDTree *modelTree,
        GenericIndexedCloud *dataCloud,
        DistanceType delta,
        PointProjectionTools::Transformation& dataToModel)
{
	CCVector3 Q;

	unsigned score = 0;

	unsigned i,count=dataCloud->size();
    for (i=0;i<count;++i)
    {
		dataCloud->getPoint(i,Q);
        //Apply rigid transform to each point
        Q = dataToModel.R * Q + dataToModel.T;
        //Check if there is a point in the model cloud that is close enough to q
        if(modelTree->FindPointBelowDistance(Q.u, delta))
            score++;
    }

    return score;
}

bool FPCSRegistrationTools::FindBase(GenericIndexedCloud* cloud,
                                        float overlap,
                                        unsigned nbTries,
                                        Base &base)
{
    unsigned a, b, c, d, t1, t2;
    unsigned i, size;
    float best, f, d0, d1, d2, x, y, z, w;
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
        if(t1 == a || t2 == a || t1 == t2)
            continue;

        p1 = cloud->getPoint(t1);
        p2 = cloud->getPoint(t2);
        //Checked that the selected points are not more than overlap-distant from p0
        u = *p1-*p0;
        if(u.norm2() > overlap)
            continue;
        u = *p2-*p0;
        if(u.norm2() > overlap)
            continue;

        //compute [p0, p1, p2] area thanks to cross product
        x = ((p1->y-p0->y)*(p2->z-p0->z))-((p1->z-p0->z)*(p2->y-p0->y));
        y = ((p1->z-p0->z)*(p2->x-p0->x))-((p1->x-p0->x)*(p2->z-p0->z));
        z = ((p1->x-p0->x)*(p2->y-p0->y))-((p1->y-p0->y)*(p2->x-p0->x));
        //don't need to compute the true area : f=(area²)*2 is sufficient for comparison
        f = (x*x)+(y*y)+(z*z);
        if(f > best)
        {
            b = t1;
            c = t2;
            best = f;
            normal.x = x;
            normal.y = y;
            normal.z = z;
        }
    }

    if(b == c)
        return false;

    //Once we found the points, we have to search for a fourth coplanar point
    f = normal.norm();
    if(f<=0.)
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
        if(t1 == a || t1 == b || t1 == c)
            continue;
        p3 = cloud->getPoint(t1);
        //p3 must be close enough to at least two other points (considering overlap)
        d0 = (*p3 - *p0).norm2();
        d1 = (*p3 - *p1).norm2();
        d2 = (*p3 - *p2).norm2();
        if((d0>=overlap && d1>=overlap) || (d0>=overlap && d2>=overlap) || (d1>=overlap && d2>=overlap))
            continue;
        //Compute distance to the plane (cloud[a], cloud[b], cloud[c])
        f = fabs((x*p3->x)+(y*p3->y)+(z*p3->z)+w);
        //keep the point which is the closest to the plane, while being as far as possible from the other three points
        f=(f+1.0f)/(sqrt(d0)+sqrt(d1)+sqrt(d2));
        if((best < 0.) || (f < best))
        {
            d = t1;
            best = f;
        }
    }

    //Store the result in the base parameter
    if(d != a)
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
        if(u.dot(v) <= 0)
        {
            //p2 and p3 lie on both sides of [p0, p1]
            base.init(a, b, c, d);
            return true;
        }
        u = (*p2-*p1)*(*p0-*p1);
        v = (*p2-*p1)*(*p3-*p1);
        if(u.dot(v) <= 0)
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


int FPCSRegistrationTools::FindCongruentBases(
        KDTree* tree,
        float delta,
        const CCVector3* base[4],
        std::vector<Base>& results)
{
    unsigned i, j, a, b;
    float r1, r2, d1, d2;
    CCVector3 e, r, s, u, v;
    const CCVector3 *p0, *p1, *p2, *p3, *q0, *q1;
    std::vector<unsigned> pointsIndexes;
    SimpleCloud tmpCloud1, tmpCloud2;
    Base quad;
    GenericIndexedCloud *cloud;
    std::vector<IndexPair> match, pairs1, pairs2;
    IndexPair idxPair;

    //Compute reference base invariants (r1, r2)
    p0 = base[0];
    p1 = base[1];
    p2 = base[2];
    p3 = base[3];
    e = *p1-*p0;
    d1 = e.norm();
    e = *p3-*p2;
    d2 = e.norm();
    if(!LinesIntersections(*p0, *p1, *p2, *p3, e, r1, r2))
        return 0;

    //Find all pairs which are d1-appart and d2-appart
    cloud = tree->GetAssociatedCloud();
	unsigned count = cloud->size();
    pointsIndexes.reserve(count);
	if (pointsIndexes.capacity() < count) //not enough memory
		return -1;

	pointsIndexes.clear();
    for(i=0; i<count; i++)
    {
        q0 = cloud->getPoint(i);
        idxPair.a = i;
        //Extract all points from the cloud which are d1-appart (up to delta) from q0
        tree->FindPointsLyingToDistance(q0->u, d1, delta, pointsIndexes);
        for(j=0; j<pointsIndexes.size(); j++)
        {
            //As ||pi-pj|| = ||pj-pi||,  we only take care of pairs that verify i<j
            if(pointsIndexes[j]>i)
            {
                idxPair.b = pointsIndexes[j];
                pairs1.push_back(idxPair);
            }
        }
        pointsIndexes.clear();
        //Extract all points from the cloud which are d2-appart (up to delta) from q0
        tree->FindPointsLyingToDistance(q0->u, d2, delta, pointsIndexes);
        for(j=0; j<pointsIndexes.size(); j++)
        {
            if(pointsIndexes[j]>i)
            {
                idxPair.b = pointsIndexes[j];
                pairs2.push_back(idxPair);
            }
        }
        pointsIndexes.clear();
    }

    //Select among the pairs the ones that can be congruent to the base "base"
	count = pairs1.size();
    if (!tmpCloud1.reserve(count*2)) //not enough memory
		return -2;
    for(i=0; i<count; i++)
    {
        //generate the two intermediate points from r1 in pairs1[i]
        q0 = cloud->getPoint(pairs1[i].a);
        q1 = cloud->getPoint(pairs1[i].b);
        e.x = q0->x+r1*(q1->x-q0->x);
        e.y = q0->y+r1*(q1->y-q0->y);
        e.z = q0->z+r1*(q1->z-q0->z);
        tmpCloud1.addPoint(e);
        e.x = q1->x+r1*(q0->x-q1->x);
        e.y = q1->y+r1*(q0->y-q1->y);
        e.z = q1->z+r1*(q0->z-q1->z);
        tmpCloud1.addPoint(e);
    }

	count = pairs2.size();
    if (!tmpCloud2.reserve(count*2)) //not enough memory
		return -3;
    for(i=0; i<count; i++)
    {
        //generate the two intermediate points from r2 in pairs2[i]
        q0 = cloud->getPoint(pairs2[i].a);
        q1 = cloud->getPoint(pairs2[i].b);
        e.x = q0->x+r2*(q1->x-q0->x);
        e.y = q0->y+r2*(q1->y-q0->y);
        e.z = q0->z+r2*(q1->z-q0->z);
        tmpCloud2.addPoint(e);
        e.x = q1->x+r2*(q0->x-q1->x);
        e.y = q1->y+r2*(q0->y-q1->y);
        e.z = q1->z+r2*(q0->z-q1->z);
        tmpCloud2.addPoint(e);
    }

    //build kdtree for nearest neighbour fast research
    KDTree *intermediatesTree = new KDTree();
    if (!intermediatesTree->BuildFromCloud(&tmpCloud1))
    {
        delete intermediatesTree;
        return -4;
    }

    //Find matching (up to delta) intermediate points in tmpCloud1 and tmpCloud2
	count = tmpCloud2.size();
    match.reserve(count);
	if (match.capacity() < count)  //not enough memory
	{
		delete intermediatesTree;
		return -5;
	}
    for(i=0; i<count; i++)
    {
        q0 = tmpCloud2.getPoint(i);
        if(intermediatesTree->FindNearestNeighbour(q0->u, a, delta))
        {
            idxPair.a = i;
            idxPair.b = a;
            match.push_back(idxPair);
        }
    }

    //Find bases from matching intermediate points indexes
    results.clear();
	count = match.size();
    if(count>0)
    {
        results.reserve(count);
		if (results.capacity() < count)  //not enough memory
		{
			delete intermediatesTree;
			return -6;
		}
        for(i=0; i<count; i++)
        {
            a = match[i].a / 2;
            b = match[i].b / 2;
            if((match[i].b%2) == 0)
            {
                quad.a = pairs1[b].a;
                quad.b = pairs1[b].b;
            }
            else
            {
                quad.a = pairs1[b].b;
                quad.b = pairs1[b].a;
            }
            if((match[i].a%2) == 0)
            {
                quad.c = pairs2[a].a;
                quad.d = pairs2[a].b;
            }
            else
            {
                quad.c = pairs2[a].b;
                quad.d = pairs2[a].a;
            }
            results.push_back(quad);
        }
    }

    delete intermediatesTree;
    tmpCloud1.clear();
    tmpCloud2.clear();

    return (int)results.size();
}


bool FPCSRegistrationTools::LinesIntersections(
    const CCVector3 &p0,
    const CCVector3 &p1,
    const CCVector3 &p2,
    const CCVector3 &p3,
    CCVector3 &inter,
    float& lambda,
    float& mu)
{
    CCVector3 p02, p32, p10, A, B;
    float num, denom;

    //Find lambda and mu such that :
    //A = p0+lambda(p1-p0)
    //B = p2+mu(p3-p2)
    //(lambda, mu) = argmin(||A-B||²)
    p02 = p0-p2;
    p32 = p3-p2;
    p10 = p1-p0;
    num = (p02.dot(p32) * p32.dot(p10)) - (p02.dot(p10) * p32.dot(p32));
    denom = (p10.dot(p10) * p32.dot(p32)) - (p32.dot(p10) * p32.dot(p10));
    if(fabs(denom) < 0.00001)
        return false;
    lambda = num / denom;
    num = p02.dot(p32) + (lambda*p32.dot(p10));
    denom = p32.dot(p32);
    if(fabs(denom) < 0.00001)
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

bool FPCSRegistrationTools::FilterCandidates(
        GenericIndexedCloud *modelCloud,
        GenericIndexedCloud *dataCloud,
        Base& reference,
        std::vector<Base>& candidates,
        unsigned nbMaxCandidates,
        std::vector<PointProjectionTools::Transformation>& transforms)
{
    std::vector<Base> table;
    std::vector<float> scores, sortedscores;
    const CCVector3 *p[4], *q;
    unsigned i, j;
    PointProjectionTools::Transformation t;
    std::vector<PointProjectionTools::Transformation> tarray;
    SimpleCloud referenceBaseCloud, dataBaseCloud;

	unsigned candidatesCount = candidates.size();
    if(candidates.size() == 0)
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

    scores.reserve(candidatesCount);
    sortedscores.reserve(candidatesCount);
    tarray.reserve(candidatesCount);
    transforms.reserve(candidatesCount);

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

        if (!RegistrationTools::RegistrationProcedure(&dataBaseCloud, &referenceBaseCloud, t))
            return false;

        tarray.push_back(t);
        if(filter)
        {
            float score = 0.;
			GenericIndexedCloud* b = PointProjectionTools::applyTransformation(&dataBaseCloud, t);
			if (!b)
				return false; //not enough memory
            for (j=0; j<4; j++)
            {
                q = b->getPoint(j);
                score += (*q - *(p[j])).norm();
            }
            delete b;
            scores.push_back(score);
            sortedscores.push_back(score);
        }
    }

    if(filter)
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
            if(scores[i]<=score && j<nbMaxCandidates)
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

