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

#include "qCanupoProcess.h"

//local
#include "qCanupoTools.h"

//CloudCompare
#include <ccMainAppInterface.h>

//CCLib
#include <CloudSamplingTools.h>
#include <ReferenceCloud.h>

//qCC_db
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Qt
#include <QApplication>
#include <QMessageBox>
#include <QStringList>

// Default SF names
#ifdef COMPILE_PRIVATE_CANUPO
static const char CANUPO_PER_LEVEL_ROUGHNESS_SF_NAME[] = "CANUPO.roughness";
#endif
static const char CANUPO_PER_LEVEL_ADDITIONAL_SF_NAME[] = "CANUPO.(x-y)";

//Reserved name for CANUPO 'MSC' meta-data
static const char s_canupoMSCMetaData[] = "CanupoMSCData";

//Tries to refine the classification (returns the new confidence if successful)
float RefinePointClassif(	const Classifier& classifier,
							const float confidence,
							float& distToBoundary,
							ccPointCloud* cloud,
							ccOctree* octree,
							unsigned char octreeLevel,
							CCLib::GenericIndexedCloudPersist* corePoints,
							CCLib::DgmOctree* corePointsOctree,
							unsigned char coreOctreeLevel,
							unsigned coreIndex,
							PointCoordinateType largestRadius,
							const std::vector<int>& corePointClasses
	)
{
	CCLib::ScalarField* sf = cloud->getCurrentDisplayedScalarField();
	if (!sf)
	{
		assert(false);
		return confidence;
	}

	try
	{
		// find all scene data around that core point
		CCLib::DgmOctree::NeighboursSet neighbors;
		int n = octree->getPointsInSphericalNeighbourhood(*corePoints->getPoint(coreIndex),
			largestRadius, //we use the biggest neighborhood
			neighbors,
			octreeLevel);

		// for each scene data point, find the corresponding core point and check if it is reliable
		std::vector<ScalarType> class1SFValues;
		std::vector<ScalarType> class2SFValues;
		for (int j = 0; j < n; ++j)
		{
			unsigned currentPointIndex = neighbors[j].pointIndex;
			//look for the nearest 'core point' neighbor
			unsigned nearestCoreIndex = 0;
			if (corePoints == cloud)
			{
				//if we used the whole cloud as core points, then each point is a core point!
				nearestCoreIndex = currentPointIndex;
			}
			else
			{
				double maxSquareDist = 0;
				CCLib::ReferenceCloud Yk(corePoints);
				if (corePointsOctree->findPointNeighbourhood(cloud->getPoint(currentPointIndex),
					&Yk,
					1,
					coreOctreeLevel,
					maxSquareDist) == 1)
				{
					nearestCoreIndex = Yk.getPointGlobalIndex(0);
				}
				else
				{
					assert(false);
					continue;
				}
			}

			int corePointClass = corePointClasses[nearestCoreIndex];
			if (corePointClass != -1)
			{
				const ScalarType& sfValue = sf->getValue(currentPointIndex);
				if (corePointClass == classifier.class1)
					class1SFValues.push_back(sfValue);
				else if (corePointClass == classifier.class2)
					class2SFValues.push_back(sfValue);
				// else the extra info is irrelevant for this classifier pair
			}
		}

		// some local info? TODO: min size for considering this information is reliable?
		size_t nsamples = class1SFValues.size() + class2SFValues.size();
		if (nsamples == 0)
		{
			//nothing to do
			return confidence;
		}

		// only one class?
		if (class1SFValues.empty())
		{
			distToBoundary = static_cast<float>(class2SFValues.size()) / n;
			if (distToBoundary < 0.5f)
			{
				//too close
				return confidence;
			}
		}
		else if (class2SFValues.empty())
		{
			distToBoundary = -static_cast<float>(class1SFValues.size()) / n; //dist(class 1) < 0
			if (distToBoundary > -0.5f)
			{
				//too close
				return confidence;
			}
		}
		else
		{
			std::sort(class1SFValues.begin(), class1SFValues.end());
			std::sort(class2SFValues.begin(), class2SFValues.end());

			std::vector<ScalarType>* smallestSet = &class1SFValues;
			std::vector<ScalarType>* largestSet = &class2SFValues;

			if (class1SFValues.size() >= class2SFValues.size())
			{
				std::swap(smallestSet, largestSet);
			}

			std::vector<ScalarType> bestSplit;
			std::vector<int> bestSplitDir;
			float bestConfidence = -1.0f;

			for (size_t k = 0; k < smallestSet->size(); ++k)
			{
				size_t nsbelow = 0;
				size_t nlabove = 0;

				// dichomed is the first index so that largestSet[dichomed] >= smallestSet[k]...
				size_t dichomed = std::upper_bound(largestSet->begin(), largestSet->end(), smallestSet->at(k)) - largestSet->begin();
				if (dichomed == 0)
				{
					nlabove = largestSet->size();
					nsbelow = k + 1;
				}
				else
				{
					nlabove = largestSet->size() - dichomed;
					nsbelow = k;
				}

				// classification on either side, take largest and reverse roles if necessary
				float c1 = static_cast<float>(nlabove) / largestSet->size() + static_cast<float>(nsbelow) / smallestSet->size();
				float c2 = static_cast<float>(largestSet->size() - nlabove) / largestSet->size() + static_cast<float>(smallestSet->size() - nsbelow) / smallestSet->size();
				float conf = std::max(c1, c2);
				// no need to average for comparison purpose
				if (bestConfidence < conf)
				{
					bestSplit.clear();
					bestSplitDir.clear();
					bestConfidence = conf;
				}

				if (qCanupoTools::Fpeq<float>(bestConfidence, conf))
				{
					bestSplit.push_back((smallestSet->at(k) + largestSet->at(std::min(dichomed, largestSet->size() - 1))) / 2);
					bestSplitDir.push_back(c1 <= c2 ? 1 : 0);
				}
			}

			bestConfidence /= 2;
			// see if we're improving estimated probability or not
			// TODO: sometimes (rarely) there are mistakes in the reference core points and we're dealing with similar classes
			// => put back these core points in the unreliable pool
			if (/*old*/confidence < bestConfidence)
			{
				// take median best split
				size_t bsi = bestSplit.size() / 2;
				distToBoundary = sf->getValue(coreIndex) - bestSplit[bsi];
				// reverse if necessary
				if (bestSplitDir[bsi] == 1)
					distToBoundary = -distToBoundary;
				// back to original vectors
				if (class1SFValues.size() >= class2SFValues.size())
					distToBoundary = -distToBoundary;
			}
			else
			{
				//no improvement
				return confidence;
			}
		}

		//update confidence
		float newConfidence = 1.0f / (exp(-fabs(distToBoundary)) + 1.0f); //in [0.5 ; 1]
		newConfidence = 2 * (newConfidence - 0.5f); //map to [0;1]

		return newConfidence;
	}
	catch (const std::bad_alloc&)
	{
		return -1.0f;
	}

	assert(false); //we shouldn't arrive here!
	return confidence;
}

bool qCanupoProcess::Classify(	QString classifierFilename,
								const ClassifyParams& params,
								ccPointCloud* cloud,
								CCLib::GenericIndexedCloudPersist* corePoints,
								CorePointDescSet& corePointsDescriptors,
								ccPointCloud* realCorePoints/*=nullptr*/,
								ccMainAppInterface* app/*=nullptr*/,
								QWidget* parentWidget/*=nullptr*/,
								bool silent/*=false*/)
{
	//core points are mandatory
	if (!cloud || !corePoints)
	{
		assert(false);
		return false;
	}

	//load the classifier file
	std::vector<Classifier> classifiers;
	std::vector<float> scales;
	unsigned descriptorID = DESC_DIMENSIONALITY;
	{
		QString error;
		if (!Classifier::Load(classifierFilename, classifiers, scales, error))
		{
			if (app)
				app->dispToConsole(QString("An error occured: ") + error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
		if (classifiers.empty())
		{
			if (app)
				app->dispToConsole(QString("Invalid classifier file!"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		//check that the descriptor ID of every classifier is handled by this version!
		for (size_t i = 0; i < classifiers.size(); ++i)
		{
			if (!ScaleParamsComputer::GetByID(classifiers[i].descriptorID))
			{
				if (app)
					app->dispToConsole(QString("Unhandled descriptor type! (ID = %1)").arg(classifiers[i].descriptorID), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}
			if (i != 0 && classifiers[i].descriptorID != classifiers[0].descriptorID)
			{
				if (app)
					app->dispToConsole(QString("Can't handle mixed descriptor types!"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}
		}
		descriptorID = classifiers[0].descriptorID;
	}

	//progress dialog
	ccProgressDialog pDlg(true, parentWidget);

	//does the core point cloud has associated meta-data?
	QVariant mscMetaData;
	bool useExistingMetaData = false;
	if (realCorePoints)
	{
		mscMetaData = realCorePoints->getMetaData(s_canupoMSCMetaData);
		if (mscMetaData.isValid())
		{
			bool validMetaData = (	mscMetaData.type() == QVariant::ByteArray
								&&	corePointsDescriptors.fromByteArray(mscMetaData.toByteArray()));

			if (validMetaData)
			{
				useExistingMetaData = true;
			}
			else
			{
				if (app)
					app->dispToConsole("[qCanupo] Failed to read core point cloud associated MSC meta data?!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				mscMetaData.clear();
			}
		}
	}

#ifdef COMPILE_PRIVATE_CANUPO
	//whether to compute per scale roughness for each core point!
	bool generateRoughnessSF = cDlg.generateRoughnessSFsCheckBox->isChecked();
	std::vector<ccScalarField*> coreRoughnessSFs;
#endif

	try
	{
		for (int step = 0; step < 1; ++step) //fake loop for easy break
		{
			//check descriptors' scales (if already loaded)
			bool computeDescriptors = true;
			if (!corePointsDescriptors.scales().empty())
			{
				if (qCanupoTools::TestVectorsOverlap(scales, corePointsDescriptors.scales()) < scales.size())
				{
					if (!silent && QMessageBox::question(	parentWidget,
															"Scales mismatch",
															"Available descriptors/scales data mismatch with classifier's! Compute new descriptors or cancel?",
															QMessageBox::Yes,
															QMessageBox::Cancel) == QMessageBox::Cancel)
					{
						//cancel process
						break;
					}
				}
				else
				{
					computeDescriptors = false; //no need to compute the descriptors as we already have them!
				}

				//check descriptor type!
				if (!computeDescriptors && corePointsDescriptors.descriptorID() != descriptorID)
				{
					if (!silent && QMessageBox::question(parentWidget,
														"Descriptor type mismatch",
														"Available descriptors have been computed with another descriptor type! Compute new descriptors or cancel?",
														QMessageBox::Yes,
														QMessageBox::Cancel) == QMessageBox::Cancel)
					{
						//cancel process
						break;
					}
					else
					{
						//force new computation
						computeDescriptors = true;
					}
				}
			}

			//compute the original cloud octree
			ccOctree::Shared octree = cloud->getOctree();
			if (!octree)
			{
				octree = cloud->computeOctree(&pDlg);
				if (!octree)
				{
					if (app)
						app->dispToConsole("Failed to compute input cloud octree!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					break;
				}
				else if (app && cloud->getParent())
				{
					app->addToDB(cloud->getOctreeProxy());
				}
			}

#ifdef COMPILE_PRIVATE_CANUPO
			computeDescriptors |= generateRoughnessSF;
#endif

			//let's compute the descriptors
			if (computeDescriptors)
			{
				//check that the selected descriptor (computer) is valid
				{
					assert(descriptorID != 0);
					ScaleParamsComputer* computer = ScaleParamsComputer::GetByID(descriptorID);
					if (!computer)
					{
						if (app)
							app->dispToConsole(QString("Internal error: unhandled descriptor ID (%1)!").arg(descriptorID), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
						break;
					}

					if (computer->needSF()
						&& ((realCorePoints		&& realCorePoints->getCurrentDisplayedScalarField() == nullptr)
						||  (!realCorePoints	&&          cloud->getCurrentDisplayedScalarField() == nullptr) //if realCorePoints == 0, it means that the subsampled cloud couldn't be converted to a real cloud!
						)
						)
					{
						if (app)
							app->dispToConsole(QString("To compute this type of descriptor, the core points cloud must have an active scalar field!"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
						break;
					}
				}

#ifdef COMPILE_PRIVATE_CANUPO
				//test: create 1 sf per scale with roughness
				if (generateRoughnessSF)
				{
					size_t scaleCount = scales.size();
					coreRoughnessSFs.resize(scaleCount,0);
					//for each scale
					for (size_t s=0; s<scaleCount; ++s)
					{
						QString sfName = QString(CANUPO_PER_LEVEL_ROUGHNESS_SF_NAME) + QString(" @ scale %1").arg(scales[s]);

						coreRoughnessSFs[s] = new ccScalarField(qPrintable(sfName));
						if (!coreRoughnessSFs[s]->resize(corePoints->size(),NAN_VALUE))
						{
							m_app->dispToConsole("Not enough memory to store per-level roughness!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
							generateRoughnessSF = false;
							break;
						}
					}
				}
#endif
				//computes the 'descriptors'
				bool invalidDescriptors = false;
				QString errorStr;
				if (!qCanupoTools::ComputeCorePointsDescriptors(corePoints,
					corePointsDescriptors,
					cloud,
					scales,
					invalidDescriptors,
					errorStr,
					descriptorID,
					params.maxThreadCount,
					&pDlg,
					octree.data()
#ifdef COMPILE_PRIVATE_CANUPO
					, generateRoughnessSF ? &coreRoughnessSFs : 0
#endif
					))
				{
					if (app)
						app->dispToConsole(QString("Failed to compute core points descriptors: %1").arg(errorStr), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					break;
				}
				else if (invalidDescriptors)
				{
					if (app)
						app->dispToConsole("[qCanupo] Some descriptors couldn't be computed (min scale may be too small)!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
			}

			//main classification process
			{
				//advanced options
				assert(!params.useActiveSFForConfidence || cloud->getCurrentDisplayedScalarField() != nullptr);

				// core points octree
				CCLib::DgmOctree* corePointsOctree = nullptr;
				if (corePoints == cloud)
				{
					corePointsOctree = octree.data();
				}
				else
				{
					corePointsOctree = new CCLib::DgmOctree(corePoints);
					if (!corePointsOctree->build(&pDlg))
					{
						if (app)
							app->dispToConsole("Failed to compute core points octree! (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
						delete corePointsOctree;
						corePointsOctree = nullptr;
						break;
					}
				}
				assert(corePointsOctree);

				const PointCoordinateType& largestRadius = scales.front() / 2; //largest scale is the first by construction in canupo
				unsigned char coreOctreeLevel = corePointsOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(params.samplingDist != 0 ? params.samplingDist : largestRadius);
				unsigned char octreeLevel = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(largestRadius);

				//core points class labels
				std::vector<int> corePointClasses;
				//core points confidence values
				std::vector<float> corePointConfidences;

				// set the class of all core points that are far enough from hyperplane
				bool processCanceled = false;

				size_t corePointCount = corePointsDescriptors.size();

				corePointClasses.resize(corePointCount, -1);
				corePointConfidences.resize(corePointCount, 0.0f);

				//number of points that couldn't be classified
				std::vector<unsigned> pendingPoints(corePointCount);
				{
					for (size_t i = 0; i < corePointCount; ++i)
						pendingPoints[i] = static_cast<unsigned>(i);
				}
				std::vector<unsigned> unreliablePointIndexes;

				CCLib::ScalarField* sf = cloud->getCurrentDisplayedScalarField();
				assert(!params.useActiveSFForConfidence || sf);

				//while unreliable points remain
				while (!pendingPoints.empty())
				{
					//progress notification
					pDlg.reset();
					pDlg.setInfo(QObject::tr("Remaining points to classify: %1\nSource points: %2").arg(pendingPoints.size()).arg(cloud->size()));
					pDlg.setMethodTitle(QObject::tr("Classification"));
					CCLib::NormalizedProgress nProgress(&pDlg, corePoints->size());
					pDlg.start();

					for (size_t i = 0; i < pendingPoints.size(); ++i)
					{
						unsigned coreIndex = pendingPoints[i];
						CorePointDesc& coreDesc = corePointsDescriptors[coreIndex];

						//most common case
						if (classifiers.size() == 1)
						{
							const Classifier& classifier = classifiers.front();
							float distToBoundary = classifier.classify(coreDesc);

							float confidence = 1.0f / (exp(-fabs(distToBoundary)) + 1.0f); //in [0.5 ; 1]
							confidence = 2 * (confidence - 0.5f); //map to [0;1]

							//unreliable point
							bool unreliable = false;
							if (confidence < params.confidenceThreshold)
							{
								unreliable = true;
								if (params.useActiveSFForConfidence)
								{
									//use the scalar field to refine the classification
									float newConfidence = RefinePointClassif(	classifier,
																				confidence,
																				distToBoundary,
																				cloud,
																				octree.data(),
																				octreeLevel,
																				corePoints,
																				corePointsOctree,
																				coreOctreeLevel,
																				coreIndex,
																				largestRadius,
																				corePointClasses);

									if (newConfidence < 0)
									{
										if (app)
											app->dispToConsole("Internal error: failed to refine the classification on the boundary (not enough memory)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
										return false;
									}
									else if (newConfidence > confidence)
									{
										unreliable = false;
										confidence = newConfidence;
									}
								}
							}

							if (!unreliable)
							{
								int theClass = (distToBoundary >= 0 ? classifier.class2 : classifier.class1);
								corePointClasses[i] = theClass;
								corePointConfidences[i] = confidence;
							}
							else if (params.useActiveSFForConfidence)
							{
								//this point can't be classified this way
								unreliablePointIndexes.push_back(static_cast<unsigned>(i));
							}
						}
						else //more than one classifier
						{
							std::map< int, int > votes;
							std::map< int, float > minConfidences;

							// apply all classifiers and look for the most represented class
							for (std::vector<Classifier>::const_iterator classifierIt = classifiers.begin(); classifierIt != classifiers.end(); ++classifierIt)
							{
								const Classifier& classifier = *classifierIt;

								// uniformize the order, distToBoundary>0 selects the larger class of both
								float distToBoundary = classifier.classify(coreDesc); //DGM: the descriptors may have more values than the number of scales!
								//if (classifier.class1 > classifier.class2)
								//	distToBoundary = -distToBoundary;

								//int minclass = std::min(classifier.class1, classifier.class2);
								//int maxclass = std::max(classifier.class1, classifier.class2);

								//int minclass = classifier.class1;
								//int maxclass = classifier.class2;

								float confidence = 1.0f / (exp(-fabs(distToBoundary)) + 1.0f); //in [0.5 ; 1]
								confidence = 2 * (confidence - 0.5f); //map to [0;1]

								//unreliable point
								if (confidence < params.confidenceThreshold)
								{
									bool unreliable = true;
									if (params.useActiveSFForConfidence)
									{
										//use the scalar field to refine the classification
										float newConfidence = RefinePointClassif(classifier,
											confidence,
											distToBoundary,
											cloud,
											octree.data(),
											octreeLevel,
											corePoints,
											corePointsOctree,
											coreOctreeLevel,
											coreIndex,
											largestRadius,
											corePointClasses);

										if (newConfidence < 0)
										{
											if (app)
												app->dispToConsole("Internal error: failed to refine the classification on the boundary (not enough memory)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
											return false;
										}
										else if (newConfidence > confidence)
										{
											unreliable = false;
											confidence = newConfidence;
										}
									}

									if (unreliable)
									{
										votes.clear();
										break;
									}
								}

								int theClass = (distToBoundary >= 0 ? classifier.class2 : classifier.class1);
								++votes[theClass];

								// maintain the min confidence for each class
								if (minConfidences.find(theClass) == minConfidences.end())
								{
									minConfidences[theClass] = confidence;
								}
								else
								{
									if (confidence < minConfidences[theClass])
										minConfidences[theClass] = confidence;
								}
							}

							if (!votes.empty())
							{
								// search for max vote
								std::vector<int> bestClasses;
								int maxVoteCount = -1;
								for (auto &vote : votes)
								{
									int voteCount = vote.second;
									if (maxVoteCount < voteCount)
									{
										bestClasses.clear();
										bestClasses.push_back(vote.first);
										maxVoteCount = voteCount;
									}
									else if (maxVoteCount == voteCount)
									{
										bestClasses.push_back(vote.first);
									}
								}

								// in case of equality, use the distances to the decision boundary
								int bestClassLabel = bestClasses.front();
								if (bestClasses.size() > 1)
								{
									for (size_t j = 1; j<bestClasses.size(); ++j)
									{
										if (minConfidences[bestClasses[j]] > minConfidences[bestClassLabel])
											bestClassLabel = bestClasses[j];
									}
								}

								corePointClasses[i] = bestClassLabel;
								corePointConfidences[i] = minConfidences[bestClassLabel];
							}
							else if (params.useActiveSFForConfidence)
							{
								//this point can't be classified this way
								unreliablePointIndexes.push_back(static_cast<unsigned>(i));
							}
						}

						//progress notification
						if (!nProgress.oneStep())
						{
							processCanceled = true;
							break;
						}
					}

					//nothing has changed?
					if (pendingPoints.size() == unreliablePointIndexes.size())
					{
						break;
					}

					pendingPoints = unreliablePointIndexes;
					unreliablePointIndexes.clear();
				}

				if (processCanceled)
				{
					if (app)
						app->dispToConsole("[qCanupo] Process cancelled by user!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
					break;
				}

				// eventually label the points
				{
					// instantiate the scalar fields
					CCLib::ScalarField* classLabelSF = nullptr;
					int classLabelSFIdx = -1;
					{
						classLabelSFIdx = cloud->getScalarFieldIndexByName("CANUPO.class");
						if (classLabelSFIdx < 0)
							classLabelSFIdx = cloud->addScalarField("CANUPO.class");
						if (classLabelSFIdx >= 0)
						{
							classLabelSF = cloud->getScalarField(classLabelSFIdx);
							classLabelSF->fill(NAN_VALUE);
						}
						else
						{
							if (app)
								app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
							break;
						}
					}

					CCLib::ScalarField* confidenceSF = nullptr;
					int confidenceSFIdx = -1;
					{
						confidenceSFIdx = cloud->getScalarFieldIndexByName("CANUPO.confidence");
						if (confidenceSFIdx < 0)
							confidenceSFIdx = cloud->addScalarField("CANUPO.confidence");
						if (confidenceSFIdx >= 0)
						{
							confidenceSF = cloud->getScalarField(confidenceSFIdx);
							confidenceSF->fill(NAN_VALUE);
						}
						else if (app)
						{
							app->dispToConsole("[qCanupo] Not enough memory to store the confidence values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
						}
					}

					//optional: create 1 sf per scale with 'x-y'
					std::vector<ccScalarField*> scaleSFs;
					bool generateAdditionalSF = params.generateAdditionalSF;
					if (generateAdditionalSF && corePointsDescriptors.dimPerScale() != 2)
					{
						if (app)
							app->dispToConsole("[qCanupo] Per-level 'x-y' values can only be extracted from descriptor with 2 dimensions per scale!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
						generateAdditionalSF = false;
					}

					if (generateAdditionalSF)
					{
						//remove any previously generated scalar field starting by CANUPO_PER_LEVEL_ADDITIONAL_SF_NAME
						{
							QStringList toDelete;
							for (unsigned i = 0; i < cloud->getNumberOfScalarFields(); ++i)
							{
								QString sfName(cloud->getScalarField(i)->getName());
								if (sfName.startsWith(CANUPO_PER_LEVEL_ADDITIONAL_SF_NAME))
									toDelete << sfName;
							}

							for (int j = 0; j < toDelete.size(); ++j)
								cloud->deleteScalarField(realCorePoints->getScalarFieldIndexByName(qPrintable(toDelete[j])));
						}

						size_t scaleCount = scales.size();
						scaleSFs.resize(scaleCount, nullptr);
						//for each scale
						for (size_t s = 0; s < scaleCount; ++s)
						{
							QString sfName = QString(CANUPO_PER_LEVEL_ADDITIONAL_SF_NAME) + QString(" @ scale %1").arg(scales[s]);


							//SF with same name (if any) should have already been removed!
							assert(cloud->getScalarFieldIndexByName(qPrintable(sfName)) < 0);

							scaleSFs[s] = new ccScalarField(qPrintable(sfName));
							if (!scaleSFs[s]->resizeSafe(cloud->size(), true, NAN_VALUE))
							{
								if (app)
									app->dispToConsole("Not enough memory to store per-level 'x-y' values!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
								while (!scaleSFs.empty())
								{
									scaleSFs.back()->release();
									scaleSFs.pop_back();
								}
								generateAdditionalSF = false;
								break;
							}
						}
					}

#ifdef COMPILE_PRIVATE_CANUPO
					//optional: create 1 sf per scale with roughness
					std::vector<ccScalarField*> roughnessSFs;
					if (generateRoughnessSF)
					{
						//remove any previously generated scalar field starting by CANUPO_PER_LEVEL_ROUGHNESS_SF_NAME
						{
							QStringList toDelete;
							for (unsigned i = 0; i < realCorePoints->getNumberOfScalarFields(); ++i)
							{
								QString sfName(cloud->getScalarField(i)->getName());
								if (sfName.startsWith(CANUPO_PER_LEVEL_ROUGHNESS_SF_NAME))
									toDelete << sfName;
							}

							for (int j = 0; j < toDelete.size(); ++j)
								cloud->deleteScalarField(cloud->getScalarFieldIndexByName(qPrintable(toDelete[j])));
						}

						//if the output cloud has the same number of points as the core points cloud, no need to duplicate the scalar fields!
						if (corePoints->size() == cloud->size())
						{
							for (size_t s = 0; s < coreRoughnessSFs.size(); ++s)
							{
								//SF with same name (if any) should have already been removed!
								assert(cloud->getScalarFieldIndexByName(coreRoughnessSFs[s]->getName()) < 0);

								coreRoughnessSFs[s]->computeMinAndMax();
								cloud->addScalarField(coreRoughnessSFs[s]);
							}
							coreRoughnessSFs.clear(); //don't want to release them anymore!
							generateRoughnessSF = false; //no need to bother anymore
						}
						else
						{
							size_t scaleCount = scales.size();
							roughnessSFs.resize(scaleCount, 0);
							assert(coreRoughnessSFs.size() == roughnessSFs.size());
							//for each scale
							for (size_t s = 0; s < scaleCount; ++s)
							{
								//same name as the per-core points version
								roughnessSFs[s] = new ccScalarField(coreRoughnessSFs[s]->getName());

								//SF with same name (if any) should have already been removed!
								assert(cloud->getScalarFieldIndexByName(roughnessSFs[s]->getName()) < 0);

								if (!roughnessSFs[s]->resize(cloud->size(), NAN_VALUE))
								{
									if (app)
										app->dispToConsole("Not enough memory to store per-level roughness!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
									while (!roughnessSFs.empty())
									{
										roughnessSFs.back()->release();
										roughnessSFs.pop_back();
									}
									generateRoughnessSF = false;
									break;
								}
							}
						}
					}
#endif

					//progress notification
					pDlg.reset();
					pDlg.setInfo(QObject::tr("Core points: %1\nSource points: %2").arg(corePoints->size()).arg(cloud->size()));
					pDlg.setMethodTitle(QObject::tr("Labelling"));
					CCLib::NormalizedProgress nProgress(&pDlg, cloud->size());
					pDlg.start();

					bool error = false;
					CCLib::ReferenceCloud Yk(corePoints);
					for (unsigned i = 0; i < cloud->size(); ++i)
					{
						const CCVector3* P = cloud->getPoint(i);
						// process this point
						// first look for the nearest neighbor in core points
						unsigned nearestCorePointIndex = 0;
						if (corePoints == cloud)
						{
							//if we used the whole cloud as core points, then each point is a core point
							nearestCorePointIndex = i;
						}
						else
						{
							double maxSquareDist = 0;
							Yk.clear(false);

							assert(corePointsOctree);
							if (corePointsOctree->findPointNeighbourhood(P,
								&Yk,
								1,
								coreOctreeLevel,
								maxSquareDist) == 1)
							{
								nearestCorePointIndex = Yk.getPointGlobalIndex(0);
							}
							else
							{
								assert(false);
								error = true;
								break;
							}
						}

						assert(classLabelSF);
						ScalarType classVal = static_cast<ScalarType>(corePointClasses[nearestCorePointIndex]);
						if (classVal >= 0)
							classLabelSF->setValue(i, classVal);
						// otherwise, it is already defaulted to NaN

						if (confidenceSF)
						{
							ScalarType confVal = static_cast<ScalarType>(corePointConfidences[nearestCorePointIndex]);
							confidenceSF->setValue(i, confVal);
						}

						//save 'x-y' values
						if (generateAdditionalSF)
						{
							unsigned dimPerScale = corePointsDescriptors.dimPerScale();
							assert(dimPerScale == 2);
							const CorePointDesc& desc = corePointsDescriptors[nearestCorePointIndex];
							assert(desc.params.size() == scaleSFs.size() * dimPerScale);
							for (size_t s = 0; s < scaleSFs.size(); ++s)
							{
								ScalarType val = (desc.params[s*dimPerScale] - desc.params[s*dimPerScale + 1]);
								scaleSFs[s]->setValue(i, val);
							}
						}

#ifdef COMPILE_PRIVATE_CANUPO
						//save roughness values
						if (generateRoughnessSF)
						{
							const CorePointDesc& desc = corePointsDescriptors[nearestCorePointIndex];
							assert(coreRoughnessSFs.size() == roughnessSFs.size());
							for (size_t s = 0; s < roughnessSFs.size(); ++s)
							{
								const ScalarType& val = coreRoughnessSFs[s]->getValue(nearestCorePointIndex);
								roughnessSFs[s]->setValue(i, val);
							}
						}
#endif

						//progress notification
						if (!nProgress.oneStep())
						{
							processCanceled = true;
							break;
						}
					}

					if (error)
					{
						if (app)
							app->dispToConsole("Internal error: failed to get nearest core points?!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
						cloud->deleteScalarField(classLabelSFIdx);
						if (confidenceSFIdx >= 0)
							cloud->deleteScalarField(confidenceSFIdx);
						while (!scaleSFs.empty())
						{
							scaleSFs.back()->release();
							scaleSFs.pop_back();
						}
					}
					else
					{
						classLabelSF->computeMinAndMax();
						cloud->setCurrentDisplayedScalarField(classLabelSFIdx);

						if (confidenceSF)
							confidenceSF->computeMinAndMax();

						if (generateAdditionalSF)
						{
							for (auto &scaleSF : scaleSFs)
							{
								scaleSF->computeMinAndMax();
								scaleSF->setSymmetricalScale(true);
								cloud->addScalarField(scaleSF);
							}
						}

#ifdef COMPILE_PRIVATE_CANUPO
						if (generateRoughnessSF)
						{
							for (size_t s = 0; s < roughnessSFs.size(); ++s)
							{
								roughnessSFs[s]->computeMinAndMax();
								cloud->addScalarField(roughnessSFs[s]);
							}
						}
#endif
					}
					cloud->showSF(cloud->getCurrentDisplayedScalarField() != nullptr);
				}

				//dispose of octree
				if (corePointsOctree != octree)
				{
					delete corePointsOctree;
					corePointsOctree = nullptr;
				}

				//save MSC data as meta-data on the core point cloud
				if (realCorePoints && !useExistingMetaData)
				{
					bool proceed = true;
					if (mscMetaData.isValid())
					{
						proceed = (silent || QMessageBox::question(	parentWidget,
																	"Overwrite MSC meta-data?",
																	"Core points cloud already has associated MSC meta-data, should we overwrite them?",
																	QMessageBox::Yes,
																	QMessageBox::No) == QMessageBox::Yes
									);
					}

					if (proceed)
					{
						mscMetaData = corePointsDescriptors.toByteArray();
						if (mscMetaData.isValid())
						{
							realCorePoints->setMetaData(s_canupoMSCMetaData, mscMetaData);
							if (app)
								app->dispToConsole(QString("[qCanupo] MSC descriptors have been saved as meta-data (cloud '%1')").arg(realCorePoints->getName()));
						}
						else
						{
							if (app)
								app->dispToConsole("[qCanupo] Failed to save MSC meta-dataa (not enough memory?)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
						}
					}
				}
			}

			break;
		}
	}
	catch (const std::bad_alloc&)
	{
		if (app)
			app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

#ifdef COMPILE_PRIVATE_CANUPO
	//release roughness SFs (if any)
	while (!coreRoughnessSFs.empty())
	{
		coreRoughnessSFs.back()->release();
		coreRoughnessSFs.pop_back();
	}
#endif

	return true;
}
