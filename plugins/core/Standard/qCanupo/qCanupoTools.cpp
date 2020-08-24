//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#include "qCanupoTools.h"

//Local
#include "trainer.h"

//CCLib
#include <DistanceComputationTools.h>
#include <Neighbourhood.h>
#include <ParallelSort.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccOctree.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//qCC_plugins
#include "ccMainAppInterface.h"

//Qt
#include <QApplication>
#include <QComboBox>
#include <QMainWindow>
#include <QProgressDialog>
#include <QtConcurrentMap>

//ComputeCorePointsDescriptors parameters
static struct
{
	CCLib::GenericIndexedCloud* corePoints;
	ccGenericPointCloud* sourceCloud;
	CCLib::DgmOctree* octree;
	unsigned char octreeLevel;
	CorePointDescSet* descriptors;
	bool invalidDescriptors;

	CCLib::NormalizedProgress* nProgress;
	bool processCanceled;
	bool errorOccurred;

	ScaleParamsComputer* computer; //the per-scale parameters computer 

	std::vector<ccScalarField*>* roughnessSFs; //for test


} s_computeCorePointsDescParams;

//! Per-point descriptor computer (all the parameters are stored in s_computeCorePointsDescParams)
void ComputeCorePointDescriptor(unsigned index)
{
	if (s_computeCorePointsDescParams.processCanceled)
		return;

	const CCVector3* P = s_computeCorePointsDescParams.corePoints->getPoint(index);
	CCLib::DgmOctree::NeighboursSet neighbours;

	//extract the neighbors (maximum radius)
	float maxRadius = s_computeCorePointsDescParams.descriptors->scales().front()/2;
	int n = s_computeCorePointsDescParams.octree->getPointsInSphericalNeighbourhood(*P,
																				maxRadius,
																				neighbours,
																				s_computeCorePointsDescParams.octreeLevel);

	if (n != 0)
	{
		size_t scaleCount = s_computeCorePointsDescParams.descriptors->scales().size();

		//get reference on corresponding descriptor
		assert(s_computeCorePointsDescParams.descriptors->size() > index);
		CorePointDesc& desc = s_computeCorePointsDescParams.descriptors->at(index);

		unsigned dimPerScale = s_computeCorePointsDescParams.descriptors->dimPerScale();
		assert(desc.params.size() == scaleCount*dimPerScale);

		//init the whole neighborhood subset (we will prune it each time)
		CCLib::ReferenceCloud subset(s_computeCorePointsDescParams.sourceCloud);
		{
			if (!subset.reserve(n))
			{
				//not enough memory!
				s_computeCorePointsDescParams.errorOccurred = true;
				s_computeCorePointsDescParams.processCanceled = true; //to make the loop stop!
				return;
			}

			//sort the neighbors by increasing distance
			ParallelSort(neighbours.begin(), neighbours.end(), CCLib::DgmOctree::PointDescriptor::distComp);

			for (int j = 0; j < n; ++j)
			{
				subset.addPointIndex(neighbours[j].pointIndex);
			}
		}

		s_computeCorePointsDescParams.computer->reset();

		for (size_t i=0; i<scaleCount; ++i)
		{
			const double radius = s_computeCorePointsDescParams.descriptors->scales()[i]/2; //we start from the biggest

			if (i != 0)
			{
				//trim the points that don't fall in the current neighborhood
				double squareRadius = radius*radius;
				CCLib::DgmOctree::PointDescriptor fakeDesc(nullptr,0,squareRadius);
				CCLib::DgmOctree::NeighboursSet::iterator up = std::upper_bound(neighbours.begin(),neighbours.end(),fakeDesc,CCLib::DgmOctree::PointDescriptor::distComp);
				if (up != neighbours.end())
				{
					size_t count = std::max<size_t>( 1, up - neighbours.begin() );
			
					neighbours.resize(count);
					subset.resize(static_cast<unsigned>(count));
				}
			}

			//optional: compute per-level roughness
			if (s_computeCorePointsDescParams.roughnessSFs)
			{
				ScalarType roughness = NAN_VALUE;

				if (subset.size() >= 3)
				{
					//to compute we take the nearest point to the query point as 'central' point
					//warning: it should work in most of the cases, apart if the core points have nothing to do
					//with the global cloud!!!
					unsigned lastIndex = subset.size()-1;
					subset.swap(0, lastIndex);

					//temporarily remove the central point (now at the end)
					unsigned globalIndex = subset.getPointGlobalIndex(lastIndex);
					subset.resize(lastIndex);
					
					CCLib::Neighbourhood Z(&subset);
					const PointCoordinateType* lsPlane = Z.getLSPlane();
					if (lsPlane)
					{
						//distance to the LS plane fitted on the nearest neighbors
						const CCVector3* centralPoint = s_computeCorePointsDescParams.sourceCloud->getPoint(globalIndex);
						roughness = fabs(CCLib::DistanceComputationTools::computePoint2PlaneDistance(centralPoint,lsPlane));
					}

					//put back the point at its original place!
					subset.addPointIndex(globalIndex);
					subset.swap(0, lastIndex);
				}

				assert(s_computeCorePointsDescParams.roughnessSFs->size() == scaleCount);
				ccScalarField* sf = s_computeCorePointsDescParams.roughnessSFs->at(i);
				assert(sf && sf->currentSize() > index);
				sf->setValue(index,roughness);
			}

			bool invalidScale = false;
			if (!s_computeCorePointsDescParams.computer->computeScaleParams(subset, radius, &(desc.params[i*dimPerScale]), invalidScale))
			{
				//an error occurred!
				s_computeCorePointsDescParams.errorOccurred = true;
				s_computeCorePointsDescParams.processCanceled = true; //to make the loop stop!
				return;
			}

			if (invalidScale)
			{
				s_computeCorePointsDescParams.invalidDescriptors = true;
				//no need to compute the remaining scales!
				for (size_t j=i+1; j<scaleCount; ++j)
				{
					//copy the same parameters for all scales (see CANUPO paper)
					memcpy(&(desc.params[j*dimPerScale]), &(desc.params[i*dimPerScale]), sizeof(float)*dimPerScale);
				}
				//neighbours.clear();
				//subset.clear(true);
				break;
			}
		}
	}
	else
	{
		//if the widest neighborhood has less than 3 points, we can't compute a valid descriptor!
		s_computeCorePointsDescParams.invalidDescriptors = true;
	}
	
	//progress notification
	if (s_computeCorePointsDescParams.nProgress && !s_computeCorePointsDescParams.nProgress->oneStep())
	{
		s_computeCorePointsDescParams.processCanceled = true;
	}
}

bool qCanupoTools::ComputeCorePointsDescriptors(CCLib::GenericIndexedCloud* corePoints,
												CorePointDescSet& corePointsDescriptors,
												ccGenericPointCloud* sourceCloud,
												const std::vector<float>& sortedScales,
												bool& invalidDescriptors,
												QString& error, //if any
												unsigned descriptorID/*=DESC_DIMENSIONALITY*/,
												int maxThreadCount/*=0*/,
												CCLib::GenericProgressCallback* progressCb/*=0*/,
												CCLib::DgmOctree* inputOctree/*=0*/,
												std::vector<ccScalarField*>* roughnessSFs/*=0*/)
{
	assert(corePoints && sourceCloud);
	assert(!sortedScales.empty());
	
	invalidDescriptors = true;
	error = QString();

	unsigned corePtsCount = corePoints->size();
	if (corePtsCount == 0)
	{
		error = "No core points?!";
		return false;
	}
	size_t scaleCount = sortedScales.size();
	if (scaleCount == 0)
	{
		error = "No scales?!";
		return false;
	}

	//descriptor (computer)
	s_computeCorePointsDescParams.computer = ScaleParamsComputer::GetByID(descriptorID);
	if (!s_computeCorePointsDescParams.computer)
	{
		error = QString("Unhandled descriptor ID (%1)!").arg(descriptorID);
		return false;
	}
	if (s_computeCorePointsDescParams.computer->needSF() && !corePoints->enableScalarField())
	{
		error = "Couldn't allocate a scalar field for core points!";
		return false;
	}

	corePointsDescriptors.setDescriptorID(descriptorID);
	corePointsDescriptors.setDimPerScale(s_computeCorePointsDescParams.computer->dimPerScale());

	CCLib::DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(sourceCloud);
		if (theOctree->build(progressCb) == 0)
		{
			error = "Failed to build the octree (not enough memory?)";
			delete theOctree;
			return false;
		}
	}

	CCLib::NormalizedProgress nProgress(progressCb, corePtsCount);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo(qPrintable(QString("Core points: %1\nSource points: %2").arg(corePtsCount).arg(sourceCloud->size())));
			progressCb->setMethodTitle("Computing descriptors");
		}
		progressCb->start();
		QApplication::processEvents();
	}

	//reserve memory for descriptors storage
	bool success = true;
	try
	{
		corePointsDescriptors.resize(corePtsCount);
	}
	catch (const std::bad_alloc&)
	{
		success = false;
	}

	if (success)
		success = corePointsDescriptors.setScales(sortedScales); //automatically resizes the 'params' structure for each core point
	if (!success)
	{
		error = "Not enough memory!";
		if (!inputOctree)
			delete theOctree;
		return false;
	}

	PointCoordinateType biggestRadius = sortedScales.front()/2; //we extract the biggest neighborhood
	unsigned char octreeLevel = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(biggestRadius);

	s_computeCorePointsDescParams.corePoints = corePoints;
	s_computeCorePointsDescParams.descriptors = &corePointsDescriptors;
	s_computeCorePointsDescParams.sourceCloud = sourceCloud;
	s_computeCorePointsDescParams.octree = theOctree;
	s_computeCorePointsDescParams.octreeLevel = octreeLevel;
	s_computeCorePointsDescParams.nProgress = progressCb ? &nProgress : nullptr;
	s_computeCorePointsDescParams.processCanceled = false;
	s_computeCorePointsDescParams.errorOccurred = false;
	s_computeCorePointsDescParams.invalidDescriptors = false;
	s_computeCorePointsDescParams.roughnessSFs = roughnessSFs;

	//we try the parallel way (if we have enough memory)
	bool useParallelStrategy = true;
#ifdef _DEBUG
	useParallelStrategy = false;
#endif

	std::vector<unsigned> corePointsIndexes;
	if (useParallelStrategy)
	{
		try
		{
			corePointsIndexes.resize(corePtsCount);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			useParallelStrategy = false;
		}
	}

	if (useParallelStrategy)
	{
		for (unsigned i=0; i<corePtsCount; ++i)
		{
			corePointsIndexes[i] = i;
		}

		if (maxThreadCount == 0)
		{
			maxThreadCount = QThread::idealThreadCount();
		}
		assert(maxThreadCount <= QThread::idealThreadCount());
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(corePointsIndexes, ComputeCorePointDescriptor);
	}
	else
	{
		//manually call the static per-point method!
		for (unsigned i=0; i<corePtsCount; ++i)
		{
			ComputeCorePointDescriptor(i);
		}
	}

	//output flags
	bool wasCanceled = s_computeCorePointsDescParams.processCanceled;
	bool errorOccurred = s_computeCorePointsDescParams.errorOccurred;
	if (errorOccurred)
		error = "An error occurred during descriptors computation!";
	else if (wasCanceled)
		error = "Process has been cancelled by the user";
	invalidDescriptors = s_computeCorePointsDescParams.invalidDescriptors;

	//reset static parameters (just to be clean ;)
	s_computeCorePointsDescParams.corePoints = nullptr;
	s_computeCorePointsDescParams.descriptors = nullptr;
	s_computeCorePointsDescParams.sourceCloud = nullptr;
	s_computeCorePointsDescParams.octree = nullptr;
	s_computeCorePointsDescParams.octreeLevel = 0;
	s_computeCorePointsDescParams.nProgress = nullptr;
	s_computeCorePointsDescParams.processCanceled = false;
	s_computeCorePointsDescParams.errorOccurred = false;
	s_computeCorePointsDescParams.invalidDescriptors = false;
	s_computeCorePointsDescParams.computer = nullptr;

	if (progressCb)
	{
		progressCb->stop();
	}

	if (!inputOctree)
		delete theOctree;

	return !errorOccurred && !wasCanceled;
}

bool qCanupoTools::CompareVectors(const std::vector<float>& first, const std::vector<float>& second)
{
	//check scales
	size_t firstCount = first.size();
	if (firstCount != second.size())
		return false;

	for (size_t i=0; i<firstCount; ++i)
		if (!Fpeq<float>(first[i],second[i]))
			return false;

	return true;
}

size_t qCanupoTools::TestVectorsOverlap(const std::vector<float>& first, const std::vector<float>& second)
{
	size_t size1 = first.size();
	size_t size2 = second.size();
	size_t minCount = std::min(size1, size2);

	size_t i = 0;
	for (i=0; i<minCount; ++i)
		if (!Fpeq<float>(first[size1-1-i],second[size2-1-i]))
			break;

	return i;
}


QString qCanupoTools::GetEntityName(ccHObject* obj)
{
	if (!obj)
	{
		assert(false);
		return QString();
	}

	QString name = obj->getName();
	if (name.isEmpty())
		name = "unnamed";
	name += QString(" [ID %1]").arg(obj->getUniqueID());

	return name;
}

ccPointCloud* qCanupoTools::GetCloudFromCombo(QComboBox* comboBox, ccHObject* dbRoot)
{
	assert(comboBox && dbRoot);
	if (!comboBox || !dbRoot)
	{
		assert(false);
		return nullptr;
	}

	//return the cloud currently selected in the combox box
	int index = comboBox->currentIndex();
	if (index < 0)
	{
		assert(false);
		return nullptr;
	}
	unsigned uniqueID = comboBox->itemData(index).toUInt();
	ccHObject* item = dbRoot->find(uniqueID);
	if (!item || !item->isA(CC_TYPES::POINT_CLOUD))
	{
		assert(false);
		return nullptr;
	}
	return static_cast<ccPointCloud*>(item);
}

bool qCanupoTools::EvaluateClassifier(	const Classifier& classifier,
										const CorePointDescSet& descriptors1,
										const CorePointDescSet& descriptors2,
										const std::vector<float>& scales,
										EvalParameters& params)
{
	params = EvalParameters();

	if (descriptors1.empty() || descriptors2.empty())
	{
		//empty descriptors?
		return false;
	}

	//Evaluate on 1st class
	{
		size_t nsamples1 = descriptors1.size();
		double sumd = 0;
		double sumd2 = 0;
		for (size_t i=0; i<nsamples1; ++i)
		{
			float d = classifier.classify(descriptors1[i]);
			if (d > 0)
				params.false1++;
			else
				params.true1++;

			sumd += static_cast<double>(d);
			sumd2 += static_cast<double>(d*d);
		}
		params.mu1 = sumd / static_cast<double>(nsamples1);
		params.var1 = sumd2 / static_cast<double>(nsamples1) - params.mu1*params.mu1;
	}

	//Evaluate on 2nd class
	{
		size_t nsamples2 = descriptors2.size();
		double sumd = 0;
		double sumd2 = 0;
		for (size_t i=0; i<nsamples2; ++i)
		{
			float d = classifier.classify(descriptors2[i]);
			if (d < 0)
				params.false2++;
			else
				params.true2++;

			sumd += static_cast<double>(d);
			sumd2 += static_cast<double>(d*d);
		}
		params.mu2 = sumd / static_cast<double>(nsamples2);
		params.var2 = sumd2 / static_cast<double>(nsamples2) - params.mu2*params.mu2;
	}

	return true;
}

bool qCanupoTools::TrainClassifier(	Classifier& classifier,
									const CorePointDescSet& descriptors1,
									const CorePointDescSet& descriptors2,
									const std::vector<float>& scales,
									ccPointCloud* mscCloud,
									const CorePointDescSet* evaluationDescriptors/*=0*/,
									ccMainAppInterface* app/*=0*/)
{
	//fuse both descriptor sets in a single 'dlib' structure
	size_t nsamples1 = descriptors1.size();
	size_t nsamples2 = descriptors2.size();

	if (nsamples1 == 0 || nsamples2 == 0)
	{
		if (app)
			app->dispToConsole("Invalid descriptors!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	assert(descriptors1.descriptorID() == descriptors2.descriptorID());
	classifier.descriptorID = descriptors1.descriptorID();

	unsigned dimPerScale = descriptors1.dimPerScale();
	assert(dimPerScale == descriptors2.dimPerScale());
	classifier.dimPerScale = dimPerScale;

	//we use the specified 'scales' (not necessarily all descriptors will be used!)
	assert((descriptors1.front().params.size() % dimPerScale) == 0);
	size_t paramsCount = descriptors1.front().params.size() / dimPerScale;
	size_t scaleCount = scales.size();
	assert(scaleCount <= paramsCount);
	scaleCount = std::min(scaleCount, paramsCount);

	classifier.scales = scales;
	//already set outside!
	//classifier.class1 = 1;
	//classifier.class2 = 2;

	size_t nsamples = nsamples1 + nsamples2;
	size_t fdim = scaleCount * dimPerScale;

	std::vector<LDATrainer::sample_type> samples;
	std::vector<float> labels;
	try
	{
		LDATrainer::sample_type nanSample;
		nanSample.set_size(fdim,1);
		samples.resize(nsamples,nanSample);
		labels.resize(nsamples,1); //labels for class#1 will be changed to -1 (see below)
	}
	catch (const std::bad_alloc&)
	{
		if (app)
			app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//add class #1 data
	{
		for (size_t i=0; i<nsamples1; ++i)
		{
			const CorePointDesc& desc = descriptors1[i];
			LDATrainer::sample_type& sample = samples[i];
			//assert(scaleCount <= paramsCount); //already tested above
			size_t shift = (paramsCount-scaleCount)*dimPerScale; //if we use less scales than parameters
			for (size_t j=0; j<fdim; ++j)
			{
				sample(j) = desc.params[shift+j];
			}
			//class #1 is labelled with '-1'
			labels[i] = -1;
		}
	}
	//add class #2 data
	{
		for (size_t i=0; i<nsamples2; ++i)
		{
			const CorePointDesc& desc = descriptors2[i];
			LDATrainer::sample_type& sample = samples[nsamples1+i];
			//assert(scaleCount <= paramsCount); //already tested above
			size_t shift = (paramsCount-scaleCount)*dimPerScale; //if we use less scales than parameters
			for (size_t j=0; j<fdim; ++j)
			{
				sample(j) = desc.params[shift+j];
			}
			//class #2 is labelled with '1' (already done above)
			//labels[nsamples1+i] = 1;
		}
	}

	//Computing the two best projection directions
	QMainWindow* parentWindow = (app ? app->getMainWindow() : nullptr);
	LDATrainer trainer;
	{
		QProgressDialog tempProgressDlg("LDA (step #1) in progress... please wait...",QString(),0,0,parentWindow);
		tempProgressDlg.show();
		QApplication::processEvents();

		// shuffle before internal cross-validation to spread instances of each class
		dlib::randomize_samples(samples, labels);
		try
		{
			trainer.train(10, samples, labels);
		}
		catch(...)
		{
			if (app)
				app->dispToConsole("Oups, it seems the LDA classifier just crashed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
	}

	// get the projections of each sample on the first classifier direction
	std::vector<float> proj1;
	{
		try
		{
			proj1.resize(nsamples);
		}
		catch (const std::bad_alloc&)
		{
			if (app)
				app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		try
		{
			for (size_t i=0; i<nsamples; ++i)
			{
				proj1[i] = static_cast<float>( trainer.predict(samples[i]) );
			}
		}
		catch(...)
		{
			if (app)
				app->dispToConsole("Oups, it seems the LDA classifier just crashed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		//std::pair<std::vector<float>::const_iterator, std::vector<float>::const_iterator> mm = std::minmax_element(proj1.begin(),proj1.end());
		//m_app->dispToConsole(QString("Min/max(proj1) = (%1 , %2)").arg(*mm.first).arg(*mm.second));
	}

	dlib::matrix<LDATrainer::sample_type,0,1> basis;
	{
		basis.set_size(fdim);
		for (size_t i=0; i<fdim; ++i)
		{
			basis(i).set_size(fdim);
			for (size_t j=0; j<fdim; ++j)
				basis(i)(j) = 0;
			basis(i)(i) = 1;
		}
	}
	LDATrainer::sample_type w_vect;
	{
		w_vect.set_size(fdim);
		for (size_t i=0; i<fdim; ++i)
			w_vect(i) = static_cast<float>(trainer.m_weights[i]);
	}

	GramSchmidt(basis,w_vect);

	//Determining orthogonal direction
	std::vector<LDATrainer::sample_type> samples_reduced;
	{
		try
		{
			samples_reduced.resize(nsamples);
		}
		catch (const std::bad_alloc&)
		{
			if (app)
				app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		for (size_t i=0; i<nsamples; ++i)
			samples_reduced[i].set_size(fdim-1);
		// project the data onto the hyperplane so as to get the second direction
		for (size_t si=0; si<nsamples; ++si)
			for (size_t i=1; i<fdim; ++i)
				samples_reduced[si](i-1) = dlib::dot(samples[si], basis(i));
	}

	LDATrainer orthoTrainer;
	try
	{
		QProgressDialog tempProgressDlg("LDA (step #2) in progress... please wait...",QString(),0,0,parentWindow);
		tempProgressDlg.show();
		QApplication::processEvents();

		orthoTrainer.train(10, samples_reduced, labels);
	}
	catch(...)
	{
		if (app)
			app->dispToConsole("Oups, it seems the LDA classifier just crashed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	// convert back the classifier weights into the original space
	{		
		try
		{
			orthoTrainer.m_weights.resize(fdim+1);
		}
		catch (const std::bad_alloc&)
		{
			if (app)
				app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
		orthoTrainer.m_weights[fdim] = orthoTrainer.m_weights[fdim-1];

		size_t i=0;
		for (i=0; i<fdim; ++i)
			w_vect(i) = 0;
		for (i=1; i<fdim; ++i)
			w_vect += orthoTrainer.m_weights[i-1] * basis(i);
		for (i=0; i<fdim; ++i)
			orthoTrainer.m_weights[i] = w_vect(i);
	}

	std::vector<float> proj2;
	{
		try
		{
			proj2.resize(nsamples);
		}
		catch (const std::bad_alloc&)
		{
			if (app)
				app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		try
		{
			for (size_t i=0; i<nsamples; ++i)
			{
				proj2[i] = static_cast<float>( orthoTrainer.predict(samples[i]) );
			}
		}
		catch(...)
		{
			if (app)
				app->dispToConsole("Oups, it seems the LDA classifier just crashed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		//std::pair<std::vector<float>::const_iterator, std::vector<float>::const_iterator> mm = std::minmax_element(proj2.begin(),proj2.end());
		//m_app->dispToConsole(QString("Min/max(proj2) = (%1 , %2)").arg(*mm.first).arg(*mm.second));
	}

	// compute the reference points for orienting the classifier boundaries
	// pathological cases are possible where an arbitrary point in the (>0,>0)
	// quadrant is not in the +1 class for example
	// here, just use the mean of the classes
	ComputeReferencePoints(	classifier.refPointPos,
							classifier.refPointNeg,
							proj1,
							proj2,
							labels);

	classifier.weightsAxis1 = trainer.m_weights;
	classifier.weightsAxis2 = orthoTrainer.m_weights;

	if (true)
	{
		// Same as Brodu's code:
		// Experimental: dilatation to highlight the internal data structure
		if (!DilateClassifier(	classifier,
								proj1,
								proj2,
								labels,
								samples,
								trainer,
								orthoTrainer))
		{
			if (app)
				app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
	}

	//proceed to boundary evaluation
	{
		Classifier::Point2D boundaryCenter(0,0);
		Classifier::Point2D boundaryDir(0,1);

		assert(mscCloud);
		if (!mscCloud)
		{
			if (app)
				app->dispToConsole("[Internal error] Invalid output MSC cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
		//if we have no 'evaluation' cloud, we'll add it to the sum of the two input clouds
		size_t cloudSize = nsamples;
		if (evaluationDescriptors)
			cloudSize += evaluationDescriptors->size();
		mscCloud->clear();
		if (!mscCloud->reserve(cloudSize))
		{
			if (app)
				app->dispToConsole("Not enough memory to determine the classifier behavior!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		bool hasColors = true;
		if (!mscCloud->reserveTheRGBTable())
		{
			if (app)
				app->dispToConsole("Not enough memory to display colors!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		else
		{
			mscCloud->showColors(true);
		}

		//generate the cloud of (colored) MSC "points"
		for (size_t i = 0; i < cloudSize; ++i)
		{
			LDATrainer::sample_type sample;
			sample.set_size(fdim);

			const CorePointDesc* desc = nullptr;
			const ccColor::Rgb* col = &ccColor::lightGreyRGB;

			if (i < nsamples1)
			{
				desc = &descriptors1[i];
				col = &ccColor::blueRGB;
			}
			else if (i < nsamples)
			{
				desc = &descriptors2[i-nsamples1];
				col = &ccColor::redRGB;
			}
			else if (evaluationDescriptors)
			{
				desc = &evaluationDescriptors->at(i-nsamples);
				//col = &ccColor::lightGreyRGB;
			}
			else
			{
				assert(false);
				continue;
			}

			assert(desc && col);
			size_t shift = (paramsCount - scaleCount) * dimPerScale; //if we use less scales than parameters
			for (size_t j = 0; j < fdim; ++j)
			{
				sample(j) = desc->params[shift + j];
			}

			double x = trainer.predict(sample);
			double y = orthoTrainer.predict(sample);
			mscCloud->addPoint( CCVector3(	static_cast<float>(x),
											static_cast<float>(y),
											0) );
			if (hasColors && col)
			{
				mscCloud->addColor(*col);
			}
		}

//DGM: we only use the evaluation cloud for representation now!
#if 0
		if (evaluationDescriptors)
		{
			// radius from probabilistic SVM, diameter = 90% chance of correct classif
			PointCoordinateType radius = static_cast<PointCoordinateType>(-log(1.0/0.9 - 1.0)/2.0);

			//we'll need the octree
			ccProgressDialog pDlg(true,parentWindow);
			if (!mscCloud->computeOctree(&pDlg))
			{
				if (app)
					app->dispToConsole("Not enough memory to determine the classifier boundary on evaluation cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}
			ccOctree* octree = mscCloud->getOctree();
			assert(octree);
			unsigned char octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);

			int minNeighborCount = 0;
			Classifier::Point2D bestDir(0,0);
			Classifier::Point2D bestStartingPoint(0,0);

			const int c_searchTransSteps = 13;	// 2*13=26 steps between the reference points
			const int c_searchDirCount = 90;	// from 0 to 180 each 2 degree (unoriented lines)

			CosSinTable<c_searchDirCount> tableCosSin;
			float transStep = mscCloud->getOwnBB().getMaxBoxDim() / (c_searchTransSteps*2);

			//progress notification
			pDlg.reset();
			pDlg.setCancelButton(0);
			pDlg.setWindowTitle("Determining boundary line");
			pDlg.setRange(0,c_searchTransSteps*2);
			pDlg.setInfo(qPrintable(QString("Search steps: %1").arg(c_searchTransSteps*2)));

			//we look for the line with the least number of neighbor points
			for (int td=0; td<=c_searchTransSteps*2; ++td)
			{
				//strating point
				Classifier::Point2D A = classifier.refPointNeg + (classifier.refPointPos - classifier.refPointNeg) * (static_cast<float>(td) / static_cast<float>(c_searchTransSteps*2));

				// now we swipe a decision boundary in each direction around the point
				// and look for the lowest overall density along the boundary
				int sumds[c_searchDirCount];

				// for each orientation
				int minDirIndex = 0;
				for (int sd=0; sd<c_searchDirCount; ++sd)
				{
					// unit vector in the direction of the line
					Classifier::Point2D u(	tableCosSin.cosines[sd],
											tableCosSin.sines[sd] );
											sumds[sd] = 0;
					for (int sp = -c_searchTransSteps; sp <= c_searchTransSteps; ++sp)
					{
						float s = sp * transStep;

						// use the parametric P2D = A + u*s formulation of a line
						Classifier::Point2D P2D = A + u * s;

						CCVector3 P(P2D.x,P2D.y,0);
						CCLib::DgmOctree::NeighboursSet Yk;
						int count = octree->getPointsInSphericalNeighbourhood(P,radius,Yk,octreeLevel);

						// count the number of neighbors
						sumds[sd] += count;
					}

					// keep track of the best direction for this starting point
					if (sumds[sd] < sumds[minDirIndex])
						minDirIndex = sd;
				}

				if (td == 0 || sumds[minDirIndex] < minNeighborCount)
				{
					minNeighborCount = sumds[minDirIndex];
					bestDir.x = tableCosSin.cosines[minDirIndex];
					bestDir.y = tableCosSin.sines[minDirIndex];
					bestStartingPoint = A;
				}

				//progress notification
				pDlg.setValue(td);
			}

			pDlg.close();

			// decision boundary in this 2D space
			boundaryCenter = bestStartingPoint;
			boundaryDir = bestDir;
		}
		else
#endif
		{
			//evaluate the boundary simply with the two input "class" clouds
			Classifier::Point2D c1(0,0);
			Classifier::Point2D c2(0,0);
			unsigned n1 = 0;
			unsigned n2 = 0;
			ComputeReferencePoints(c2,c1,proj1,proj2,labels,&n2,&n1);

			Classifier::Point2D w_vect = c2 - c1;
			w_vect.normalize();
			Classifier::Point2D w_orth(-w_vect.y,w_vect.x);

			double cba2_max = 0;
			const int c_searchSteps = 180;
			CosSinTable<c_searchSteps> tableCosSin;

			//progress notification
			ccProgressDialog pDlg(false,parentWindow);
			pDlg.setWindowTitle("Determining boundary line");
			pDlg.setRange(0,c_searchSteps);
			pDlg.setInfo(qPrintable(QString("Search steps: %1").arg(c_searchSteps)));

			for (int sd=1; sd<c_searchSteps; ++sd)
			{
				Classifier::Point2D v(	tableCosSin.cosines[sd],
										tableCosSin.sines[sd] );

				dlib::matrix<double,2,2> basis;
				Classifier::Point2D base_vec1 = w_vect;
				Classifier::Point2D base_vec2 = w_vect * v.x + w_orth * v.y;
				basis(0,0) = base_vec1.x; basis(0,1) = base_vec2.x;
				basis(1,0) = base_vec1.y; basis(1,1) = base_vec2.y;
				basis = inv(basis);

				double m1 = 0;
				double m2 = 0;
				std::vector<double> p1;
				std::vector<double> p2;
				p1.reserve(n1);
				p2.reserve(n2);
				for (size_t i=0; i<nsamples; ++i)
				{
					dlib::matrix<double,2,1> P;
					P(0) = proj1[i];
					P(1) = proj2[i];
					P = basis * P;
					const double& d = P(0); // projection on w_vect along the slanted direction
					if (labels[i] < 0)
					{
						p1.push_back(d);
						m1 += d;
					}
					else
					{
						p2.push_back(d);
						m2 += d;
					}
				}
				m1 /= static_cast<double>(n1);
				m2 /= static_cast<double>(n2);

				// search for optimal separation
				bool reversed = false;
				if (m1 > m2)
				{
					reversed = true;
					std::swap(m1, m2);
					p1.swap(p2);
				}
				ParallelSort(p1.begin(), p1.end());
				ParallelSort(p2.begin(), p2.end());

				for (int i = 0; i <= 100; ++i)
				{
					double pos = m1 + i * (m2 - m1) / 100.0;

					size_t idx1 = std::lower_bound(p1.begin(), p1.end(), pos) - p1.begin();
					size_t idx2 = std::lower_bound(p2.begin(), p2.end(), pos) - p2.begin();

					double pr1 = idx1 / static_cast<double>(n1);
					double pr2 = 1.0 - idx2 / static_cast<double>(n2);
					double cba2 = fabs(pr1 + pr2);
					if (cba2 > cba2_max)
					{
						cba2_max = cba2;
						double r = (pos - m1) / (m2 - m1);
						if (reversed)
							r = 1.0 - r;
						Classifier::Point2D center = c1 + (c2 - c1) * static_cast<float>(r);
						boundaryCenter = center;
						boundaryDir = base_vec2;
					}
				}

				pDlg.setValue(sd);
			}

			pDlg.close();
		}

		//update classifier info (boundary)
		{
			PointCoordinateType l = mscCloud->getOwnBB().getDiagVec().y / 2;
			classifier.path.resize(2);
			classifier.path[0] = boundaryCenter + boundaryDir * l;
			classifier.path[1] = boundaryCenter - boundaryDir * l;
		}
	}

	return true;
}