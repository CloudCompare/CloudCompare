//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qM3C2Tools.h"

//CCLib
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>
#include <Jacobi.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccNormalVectors.h>
#include <ccAdvancedTypes.h>
#include <ccScalarField.h>
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccProgressDialog.h>

//qCC
#include "ccMainAppInterface.h"

//Qt
#include <QtCore>
#include <QApplication>
#include <QMainWindow>
#include <QProgressDialog>
#include <QtConcurrentMap>

//system
#include <vector>

// ComputeCorePointNormal parameters
static struct
{
	CCLib::GenericIndexedCloud* corePoints;
	ccGenericPointCloud* sourceCloud;
	CCLib::DgmOctree* octree;
	unsigned char octreeLevel;
	std::vector<PointCoordinateType> radii;
	NormsIndexesTableType* normCodes;
	ccScalarField* normalScale;
	bool invalidNormals;

	CCLib::NormalizedProgress* nProgress;
	bool processCanceled;

} s_corePointsNormalsParams;

void ComputeCorePointNormal(unsigned index)
{
	if (s_corePointsNormalsParams.processCanceled)
		return;

	CCVector3 bestNormal(0, 0, 0);
	ScalarType bestScale = NAN_VALUE;

	const CCVector3* P = s_corePointsNormalsParams.corePoints->getPoint(index);
	CCLib::DgmOctree::NeighboursSet neighbours;
	CCLib::ReferenceCloud subset(s_corePointsNormalsParams.sourceCloud);

	int n = s_corePointsNormalsParams.octree->getPointsInSphericalNeighbourhood(*P,
																				s_corePointsNormalsParams.radii.back(), //we use the biggest neighborhood
																				neighbours,
																				s_corePointsNormalsParams.octreeLevel);
	
	//if the widest neighborhood has less than 3 points in it, there's nothing we can do for this core point!
	if (n >= 3)
	{
		size_t radiiCount = s_corePointsNormalsParams.radii.size();

		double bestPlanarityCriterion = 0;
		unsigned bestSamplePointCount = 0;

		for (size_t i = 0; i < radiiCount; ++i)
		{
			double radius = s_corePointsNormalsParams.radii[radiiCount - 1 - i]; //we start from the biggest
			double squareRadius = radius*radius;

			subset.clear(false);
			for (unsigned j = 0; j < static_cast<unsigned>(n); ++j)
				if (neighbours[j].squareDistd <= squareRadius)
					subset.addPointIndex(neighbours[j].pointIndex);

			//as we start from the biggest neighborhood, if we have less than 3 points for the current radius
			//it won't be better for the next one(s)!
			if (subset.size() < 3)
				break;
			//see the 'M3C2' article:
			//<< we ensure that a minimum of 10 points is used to compute the normal at Dopt
			//	otherwise we choose the scale immediatly larger that fulfils this requirement >>
			//if (subset.size() < 10) //DGM -> not implemented in Brodu's code?!
			//	break;

			CCLib::Neighbourhood Z(&subset);

			/*** we manually compute the least squares best fitting plane (so as to get the PCA eigen values) ***/

			//we determine the plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
			CCLib::SquareMatrixd eigVectors;
			std::vector<double> eigValues;
			if (Jacobi<double>::ComputeEigenValuesAndVectors(Z.computeCovarianceMatrix(), eigVectors, eigValues, true))
			{
				/*** code and comments below are from the original 'm3c2' code (N. Brodu) ***/

				// The most 2D scale. For the criterion for how "2D" a scale is, see canupo
				// Ideally first and second eigenvalue are equal
				// convert to percent variance explained by each dim
				double totalvar = 0;
				CCVector3d svalues;
				for (unsigned k = 0; k < 3; ++k)
				{
					// singular values are squared roots of eigenvalues
					svalues.u[k] = eigValues[k];
					svalues.u[k] = svalues.u[k] * svalues.u[k];
					totalvar += svalues.u[k];
				}
				svalues /= totalvar;
				//sort eigenvalues
				std::sort(svalues.u, svalues.u + 3);
				std::swap(svalues.x, svalues.z);

				// ideally, 2D means first and second entries are both 1/2 and third is 0
				// convert to barycentric coordinates and take the coefficient of the 2D
				// corner as a quality measure.
				// Use barycentric coordinates : a for 1D, b for 2D and c for 3D
				// Formula on wikipedia page for barycentric coordinates
				// using directly the triangle in %variance space, they simplify a lot
				// double a = svalues[0] - svalues[1];
				double b = 2 * svalues[0] + 4 * svalues[1] - 2;
				// double c = 1 - a - b; // they sum to 1
				if (bestSamplePointCount == 0 || b > bestPlanarityCriterion)
				{
					bestPlanarityCriterion = b;
					bestSamplePointCount = subset.size();

					//the smallest eigen vector corresponds to the "least square best fitting plane" normal
					double vec[3];
					double minEigValue = 0;
					Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, vec);

					CCVector3 N = CCVector3::fromArray(vec);
					N.normalize();

					bestNormal = N;
					bestScale = static_cast<ScalarType>(radius * 2);
				}
			}
		}

		if (bestSamplePointCount < 3)
		{
			s_corePointsNormalsParams.invalidNormals = true;
		}
	}
	else
	{
		s_corePointsNormalsParams.invalidNormals = true;
	}

	//compress the best normal and store it
	CompressedNormType normCode = ccNormalVectors::GetNormIndex(bestNormal.u);
	s_corePointsNormalsParams.normCodes->setValue(index, normCode);

	//if necessary, store 'best radius'
	if (s_corePointsNormalsParams.normalScale)
		s_corePointsNormalsParams.normalScale->setValue(index, bestScale);

	//progress notification
	if (s_corePointsNormalsParams.nProgress && !s_corePointsNormalsParams.nProgress->oneStep())
	{
		s_corePointsNormalsParams.processCanceled = true;
	}
}

bool qM3C2Normals::ComputeCorePointsNormals(CCLib::GenericIndexedCloud* corePoints,
											NormsIndexesTableType* corePointsNormals,
											ccGenericPointCloud* sourceCloud,
											const std::vector<PointCoordinateType>& sortedRadii,
											bool& invalidNormals,
											int maxThreadCount/*=0*/,
											ccScalarField* normalScale/*=0*/,
											CCLib::GenericProgressCallback* progressCb/*=0*/,
											CCLib::DgmOctree* inputOctree/*=0*/)
{
	assert(corePoints && sourceCloud && corePointsNormals);
	assert(!sortedRadii.empty());
	
	invalidNormals = true;

	unsigned corePtsCount = corePoints->size();
	if (corePtsCount == 0)
		return false;

	if (normalScale)
	{
		if (normalScale->currentSize() != corePtsCount && !normalScale->resizeSafe(corePtsCount))
		{
			//not enough memory
			return false;
		}
		normalScale->fill(NAN_VALUE);
	}

	CCLib::DgmOctree* theOctree = inputOctree;
	if (!theOctree)
	{
		theOctree = new CCLib::DgmOctree(sourceCloud);
		if (theOctree->build() == 0)
		{
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
			progressCb->setMethodTitle("Computing normals");
		}
		progressCb->start();
	}

	//reserve memory for normals (codes) storage
	if (!corePointsNormals->isAllocated() || corePointsNormals->currentSize() < corePtsCount)
	{
		if (!corePointsNormals->resizeSafe(corePtsCount))
		{
			if (!inputOctree)
				delete theOctree;
			return false;
		}
	}
	PointCoordinateType biggestRadius = sortedRadii.back(); //we extract the biggest neighborhood
	unsigned char octreeLevel = theOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(biggestRadius);

	s_corePointsNormalsParams.corePoints = corePoints;
	s_corePointsNormalsParams.normCodes = corePointsNormals;
	s_corePointsNormalsParams.sourceCloud = sourceCloud;
	s_corePointsNormalsParams.radii = sortedRadii;
	s_corePointsNormalsParams.octree = theOctree;
	s_corePointsNormalsParams.octreeLevel = octreeLevel;
	s_corePointsNormalsParams.nProgress = progressCb ? &nProgress : nullptr;
	s_corePointsNormalsParams.processCanceled = false;
	s_corePointsNormalsParams.invalidNormals = false;
	s_corePointsNormalsParams.normalScale = normalScale;

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
		for (unsigned i = 0; i < corePtsCount; ++i)
		{
			corePointsIndexes[i] = i;
		}

		if (maxThreadCount == 0)
		{
			maxThreadCount = QThread::idealThreadCount();
		}
		QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
		QtConcurrent::blockingMap(corePointsIndexes, ComputeCorePointNormal);
	}
	else
	{
		//manually call the static per-point method!
		for (unsigned i = 0; i < corePtsCount; ++i)
		{
			ComputeCorePointNormal(i);
		}
	}

	//output flags
	bool wasCanceled = s_corePointsNormalsParams.processCanceled;
	invalidNormals = s_corePointsNormalsParams.invalidNormals;

	if (progressCb)
	{
		progressCb->stop();
	}

	if (!inputOctree)
		delete theOctree;

	return !wasCanceled;
}

static struct
{
	NormsIndexesTableType* normsCodes;
	CCLib::GenericIndexedCloud* normCloud;
	CCLib::GenericIndexedCloud* orientationCloud;

	CCLib::NormalizedProgress* nProgress;
	bool processCanceled;

} s_normOriWithCloud;

static void OrientPointNormalWithCloud(unsigned index)
{
	if (s_normOriWithCloud.processCanceled)
		return;

	const CompressedNormType& nCode = s_normOriWithCloud.normsCodes->getValue(index);
	CCVector3 N(ccNormalVectors::GetNormal(nCode));

	//corresponding point
	const CCVector3* P = s_normOriWithCloud.normCloud->getPoint(index);

	//find nearest point in 'orientation cloud'
	//(brute force: we don't expect much points!)
	CCVector3 orientation(0, 0, 1);
	PointCoordinateType minSquareDist = 0;
	for (unsigned j = 0; j < s_normOriWithCloud.orientationCloud->size(); ++j)
	{
		const CCVector3* Q = s_normOriWithCloud.orientationCloud->getPoint(j);
		CCVector3 PQ = (*Q - *P);
		PointCoordinateType squareDist = PQ.norm2();
		if (j == 0 || squareDist < minSquareDist)
		{
			orientation = PQ;
			minSquareDist = squareDist;
		}
	}

	//we check sign
	if (N.dot(orientation) < 0)
	{
		//inverse normal and re-compress it
		N *= -1;
		s_normOriWithCloud.normsCodes->setValue(index, ccNormalVectors::GetNormIndex(N.u));
	}

	if (s_normOriWithCloud.nProgress && !s_normOriWithCloud.nProgress->oneStep())
	{
		s_normOriWithCloud.processCanceled = true;
	}
}

bool qM3C2Normals::UpdateNormalOrientationsWithCloud(	CCLib::GenericIndexedCloud* normCloud,
														NormsIndexesTableType& normsCodes,
														CCLib::GenericIndexedCloud* orientationCloud,
														int maxThreadCount/*=0*/,
														CCLib::GenericProgressCallback* progressCb/*=0*/)
{
	//input normals
	unsigned count = normsCodes.currentSize();
	if (!normCloud || normCloud->size() != count)
	{
		assert(false);
		ccLog::Warning("[qM3C2Tools::UpdateNormalOrientationsWithCloud] Cloud/normals set mismatch!");
		return false;
	}

	//orientation points
	unsigned orientationCount = orientationCloud ? orientationCloud->size() : 0;
	if (orientationCount == 0)
	{
		//nothing to do?
		assert(false);
		return true;
	}

	CCLib::NormalizedProgress nProgress(progressCb, count);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo(qPrintable(QString("Normals: %1\nOrientation points: %2").arg(count).arg(orientationCloud->size())));
			progressCb->setMethodTitle("Orienting normals");
		}
		progressCb->start();
	}

	s_normOriWithCloud.normCloud = normCloud;
	s_normOriWithCloud.orientationCloud = orientationCloud;
	s_normOriWithCloud.normsCodes = &normsCodes;
	s_normOriWithCloud.nProgress = &nProgress;
	s_normOriWithCloud.processCanceled = false;

	//we check each normal's orientation
	{
		std::vector<unsigned> pointIndexes;
		bool useParallelStrategy = true;
#ifdef _DEBUG
		useParallelStrategy = false;
#endif
		if (useParallelStrategy)
		{
			try
			{
				pointIndexes.resize(count);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				useParallelStrategy = false;
			}
		}

		if (useParallelStrategy)
		{
			for (unsigned i = 0; i < count; ++i)
			{
				pointIndexes[i] = i;
			}

			if (maxThreadCount == 0)
			{
				maxThreadCount = QThread::idealThreadCount();
			}
			QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
			QtConcurrent::blockingMap(pointIndexes, OrientPointNormalWithCloud);
		}
		else
		{
			//manually call the static per-point method!
			for (unsigned i = 0; i < count; ++i)
			{
				OrientPointNormalWithCloud(i);
			}
		}
	}

	if (progressCb)
	{
		progressCb->stop();
	}

	return true;
}

void qM3C2Normals::MakeNormalsHorizontal(NormsIndexesTableType& normsCodes)
{
	//for each normal
	unsigned count = normsCodes.currentSize();
	for (unsigned i = 0; i < count; i++)
	{
		const CompressedNormType& nCode = normsCodes.getValue(i);

		//de-compress the normal
		CCVector3 N(ccNormalVectors::GetNormal(nCode));

		N.z = 0;
		N.normalize();

		//re-compress the normal
		normsCodes.setValue(i,ccNormalVectors::GetNormIndex(N.u));
	}
}

//! Computes the median distance of a (sorted) neighbors set
/** Uses the common definition using mid-point average in the even case
	just as the original m3c2 code by N. Brodu.
**/
static double Median(const CCLib::DgmOctree::NeighboursSet& set, size_t begin = 0, size_t count = 0)
{
	if (count == 0)
	{
		count = set.size();
		if (count == 0)
			return NAN_VALUE;
	}

	size_t nd2 = count / 2;
	double midValue = set[begin + nd2].squareDistd;

	if ((count & 1) == 0) //even case
	{
		midValue = (midValue + set[begin + nd2 - 1].squareDistd) / 2;
	}

	return midValue;
}

//! Computes the interquartile range of a (sorted) neighbors set
/** Uses Mathworld's definition (http://mathworld.wolfram.com/InterquartileRange.html)
	as the original m3c2 code by N. Brodu.
	**/
double Interquartile(const CCLib::DgmOctree::NeighboursSet& set)
{
	if (set.empty())
		return NAN_VALUE;

	size_t num = set.size();
	size_t num_pts_each_half = (num + 1) / 2;
	size_t offset_second_half = num / 2;

	double q1 = Median(set, 0, num_pts_each_half);
	double q3 = Median(set, offset_second_half, num_pts_each_half);

	return q3 - q1;
}

void qM3C2Tools::ComputeStatistics(CCLib::DgmOctree::NeighboursSet& set, bool useMedian, double& meanOrMedian, double& stdDevOrIQR)
{
	size_t count = set.size();
	if (count == 0)
	{
		meanOrMedian = NAN_VALUE;
		stdDevOrIQR = 0;
		return;
	}
	else if (count == 1)
	{
		meanOrMedian = set.back().squareDistd;
		stdDevOrIQR = 0;
		return;
	}

	if (useMedian)
	{
		//sort neighbors by distance
		std::sort(set.begin(), set.end(), CCLib::DgmOctree::PointDescriptor::distComp);

		meanOrMedian = Median(set);
		stdDevOrIQR = Interquartile(set);
	}
	else
	{
		//otherwise we proceed with the 'standard' mean
		double sum = 0;
		double sum2 = 0;
		for (size_t i = 0; i < count; ++i)
		{
			const ScalarType& dist = set[i].squareDistd;
			sum += static_cast<double>(dist); //should be the projected dist in fact!
			sum2 += static_cast<double>(dist) * dist;
		}

		assert(count > 1);
		sum /= count;
		sum2 = sqrt(fabs(sum2 / count - sum*sum));

		meanOrMedian = static_cast<ScalarType>(sum);
		stdDevOrIQR = static_cast<ScalarType>(sum2);
	}
}

bool qM3C2Tools::GuessBestParams(	ccPointCloud* cloud1,
									ccPointCloud* cloud2,
									unsigned minPoints4Stats,
									qM3C2Tools::GuessedParams& params,
									bool fastMode,
									ccMainAppInterface* app/*=0*/,
									unsigned probingCount/*=1000*/)
{
	//invalid parameters?
	if (!cloud1 || !cloud2 || minPoints4Stats == 0)
		return false;

	//no need to bother for very small clouds
	unsigned count1 = cloud1->size();
	if (count1 < 50)
	{
		params.projDepth = params.normScale = params.projScale = cloud1->getOwnBB().getDiagNorm() / 2;
		return true;
	}

	//first we can do a very simple guess for normal and projection scales (default value)
	{
		double d1 = cloud1->getOwnBB().getDiagNormd();
		double d2 = cloud2->getOwnBB().getDiagNormd();
		double baseRadius = std::min(d1, d2) * 0.01;
		params.normScale = params.projScale = baseRadius;
	}

	//we could guess the max depth with a Distance Transform (as in the
	//C2C distances computation tool of CloudCompare) but it's only
	//valid for nearest neighbor distance (and not normal-based NN
	//search). Moreover it would be a bit overkill, especially as the
	//neighbor search is progressive by default...
	params.projDepth = 5 * params.projScale;

	//if possible, for normal scale we are going to look for a low roughness (1st cloud only)
	//and for projection scale we are going to look for a good density (1st cloud only)
	bool success = true;
	if (!fastMode)
	{
		ccOctree::Shared octree1 = cloud1->getOctree();
		if (!octree1)
		{
			ccProgressDialog pDlg(true, app ? app->getMainWindow() : nullptr);
			octree1 = cloud1->computeOctree(&pDlg);
			if (!octree1)
			{
				//failed to compute octree (not enough memory?)
				return true; //we have the default values
			}
			else if (app && cloud1->getParent())
			{
				app->addToDB(cloud1->getOctreeProxy());
			}
		}

		QProgressDialog pDlg("Please wait...", "Cancel", 0, 0);
		pDlg.setWindowTitle("M3C2");
		//pDlg.setModal(true);
		pDlg.show();
		QApplication::processEvents();

		//can't probe with less than 10 points
		probingCount = std::max<unsigned>(probingCount, 10);

		//starting scale = smallest octree cell size!
		PointCoordinateType baseScale = octree1->getCellSize(ccOctree::MAX_OCTREE_LEVEL);
		PointCoordinateType scale = baseScale;

		bool hasBestNormLevel = false;
		bool hasBestProjLevel = false;
		double bestMeanRoughness = -1.0;
		std::vector<unsigned> populations;
		populations.resize(probingCount, 0);

		while (!hasBestNormLevel || !hasBestProjLevel)
		{
			unsigned char level = octree1->findBestLevelForAGivenNeighbourhoodSizeExtraction(scale);
			if (app)
				app->dispToConsole(QString("[M3C2::auto] Test scale: %1 (level %2, %3 samples)").arg(scale).arg(level).arg(probingCount));

			double sumPopulation = 0;
			double sumPopulation2 = 0;
			unsigned validLSPlanes = 0;
			double sumRoughness = 0;
			CCVector3d meanNormal(0, 0, 0);
			for (unsigned i = 0; i < probingCount; ++i)
			{
				unsigned pointIndex = static_cast<unsigned>(ceil(static_cast<double>(rand()) / RAND_MAX * static_cast<double>(count1)));
				if (count1 == pointIndex)
					count1--;

				const CCVector3* P = cloud1->getPoint(pointIndex);
				ccOctree::NeighboursSet neighbors;
				octree1->getPointsInSphericalNeighbourhood(*P, scale, neighbors, level);

				size_t n = neighbors.size();
				populations[i] = static_cast<unsigned>(n);
				sumPopulation += static_cast<double>(n);
				sumPopulation2 += static_cast<double>(n*n);

				if (!hasBestNormLevel && n >= 12)
				{
					CCLib::ReferenceCloud neighborCloud(cloud1);
					if (!neighborCloud.resize(static_cast<unsigned>(n)))
					{
						//not enough memory!
						success = false;
						//for early stop
						hasBestNormLevel = hasBestProjLevel = true;
						break;
					}

					for (unsigned j = 0; j < n; ++j)
					{
						neighborCloud.setPointIndex(j, neighbors[j].pointIndex);
					}

					CCLib::Neighbourhood Yk(&neighborCloud);
					const PointCoordinateType* lsPlane = Yk.getLSPlane();
					if (lsPlane)
					{
						ScalarType d = CCLib::DistanceComputationTools::computePoint2PlaneDistance(P, lsPlane);
						//we compute relative roughness
						d /= scale;
						sumRoughness += d*d;//fabs(d);
						meanNormal += CCVector3d::fromArray(lsPlane);
						validLSPlanes++;
					}
				}

				if (i % 100 == 0)
				{
					if (pDlg.wasCanceled())
					{
						//early stop
						hasBestProjLevel = true;
						hasBestNormLevel = true;
						success = false;
						break;
					}
					pDlg.setValue(pDlg.value() + 1);
					QApplication::processEvents();
				}
			}

			//population stats
			{
				double meanPopulation = sumPopulation / static_cast<double>(probingCount);
				double stdDevPopulation = sqrt(fabs(sumPopulation2 / static_cast<double>(probingCount)-meanPopulation*meanPopulation));

				if (app)
					app->dispToConsole(QString("[M3C2::auto] \tPopulation per cell: %1 +/- %2").arg(meanPopulation, 0, 'f', 1).arg(stdDevPopulation, 0, 'f', 1));

				if (!hasBestProjLevel)
				{
					std::sort(populations.begin(), populations.begin() + probingCount);
					unsigned pop97 = populations[static_cast<unsigned>(floor(probingCount * 0.03))];
					if (app)
						app->dispToConsole(QString("[M3C2::auto] \t97% of cells above: %1 +/- %2").arg(pop97));
					if (pop97 /*meanPopulation - 2 * stdDevPopulation*/ >= minPoints4Stats)
					{
						if (app)
							app->dispToConsole(QString("[M3C2::auto] \tThis scale seems ok for projection!"));
						params.projScale = scale;
						hasBestProjLevel = true;
					}
				}
			}

			if (!hasBestNormLevel)
			{
				if (app)
					app->dispToConsole(QString("[M3C2::auto] \tValid normals: %1/%2").arg(validLSPlanes).arg(probingCount));

				double meanRoughness = sqrt(sumRoughness / static_cast<double>(std::max<unsigned>(validLSPlanes, 1)));
				if (app)
					app->dispToConsole(QString("[M3C2::auto] \tMean relative roughness: %1").arg(meanRoughness));

				if (validLSPlanes > 0.99 * static_cast<double>(probingCount)) //almost all neighbourhood must be large enough to fit a plane!
				{
					if (	bestMeanRoughness < 0
						|| (meanRoughness < bestMeanRoughness && (!hasBestProjLevel || scale < 2.1 * params.projScale)) //DGM: don't increase the normal scale more than 2 times the projection scale (if possible)
						)
					{
						bestMeanRoughness = meanRoughness;
						params.normScale = scale;

						//mean normal
						meanNormal.normalize();
						unsigned char maxDim = 0;
						if (fabs(meanNormal.x) < fabs(meanNormal.y))
							maxDim = 1;
						if (fabs(meanNormal.u[maxDim]) < fabs(meanNormal.z))
							maxDim = 2;

						params.preferredDimension = 2 * maxDim;
					}
					else
					{
						//this scale is worst than the previous one, so we stop here
						hasBestNormLevel = true;
						if (app)
							app->dispToConsole(QString("[M3C2::auto] \tThe previous scale was better for normals!"));
					}
				}
			}

			if (level == 6 && (!hasBestNormLevel || !hasBestProjLevel))
			{
				//if we have reach a big radius already and we don't have
				//good scales, we stop anyway!
				if (bestMeanRoughness < 0)
					params.normScale = scale;
				if (!hasBestProjLevel)
					params.projScale = scale;
				hasBestProjLevel = true;
				hasBestNormLevel = true;
				if (app)
					app->dispToConsole(QString("[M3C2::auto] We failed to converge for the best projection level, so we will stop here!"), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				break;
			}

			//scale += baseScale;
			scale *= 2;
			probingCount = std::max<unsigned>(10, probingCount - (probingCount / 10));
		}
	}

	return success;
}
