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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ccPointCloudInterpolator.h"

//qCC_db
#include "ccPointCloud.h"

//CCLib
#include <DgmOctree.h>
#include <DistanceComputationTools.h>
#include <GenericProgressCallback.h>
#include <ccScalarField.h>

struct SFPair
{
	SFPair(const CCLib::ScalarField* sfIn = nullptr, CCLib::ScalarField* sfOut = nullptr) : in(sfIn), out(sfOut) {}
	const CCLib::ScalarField* in;
	CCLib::ScalarField* out;
};

bool cellSFInterpolator(const CCLib::DgmOctree::octreeCell& cell,
						void** additionalParameters,
						CCLib::NormalizedProgress* nProgress/*=0*/)
{
	//additional parameters
//	const ccPointCloud* srcCloud = reinterpret_cast<ccPointCloud*>(additionalParameters[0]);
	const CCLib::DgmOctree* srcOctree = reinterpret_cast<CCLib::DgmOctree*>(additionalParameters[1]);
	std::vector<SFPair>* scalarFields = reinterpret_cast<std::vector< SFPair >*>(additionalParameters[2]);
	const ccPointCloudInterpolator::Parameters* params = reinterpret_cast<const ccPointCloudInterpolator::Parameters*>(additionalParameters[3]);
	
	bool normalDistWeighting = false;
	double interpSigma2x2 = 0;
	if (params->algo == ccPointCloudInterpolator::Parameters::NORMAL_DIST)
	{
		interpSigma2x2 = 2 * params->sigma * params->sigma;
		normalDistWeighting = (interpSigma2x2 > 0);
	}

	//structure for nearest neighbors search
	bool useKNN = (params->method == ccPointCloudInterpolator::Parameters::K_NEAREST_NEIGHBORS);
	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
	{
		nNSS.level = cell.level;
		if (useKNN)
		{
			nNSS.minNumberOfNeighbors = params->knn;
		}
		else
		{
			nNSS.prepare(static_cast<PointCoordinateType>(params->radius), cell.parentOctree->getCellSize(cell.level));
		}
		cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
		cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);
	}

	std::vector<double> sumValues;
	size_t sfCount = scalarFields->size();
	assert(sfCount != 0);
	sumValues.resize(sfCount);

	//for each point of the current cell (destination octree) we look for its nearest neighbours in the source cloud
	unsigned pointCount = cell.points->size();
	for (unsigned i = 0; i < pointCount; i++)
	{
		unsigned outPointIndex = cell.points->getPointGlobalIndex(i);
		cell.points->getPoint(i, nNSS.queryPoint);

		//look for neighbors (either inside a sphere or the k nearest ones)
		//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (neighborCount)!
		unsigned neighborCount = 0;

		if (useKNN)
		{
			neighborCount = srcOctree->findNearestNeighborsStartingFromCell(nNSS);
			neighborCount = std::min(neighborCount, params->knn);
		}
		else
		{
			neighborCount = srcOctree->findNeighborsInASphereStartingFromCell(nNSS, params->radius, false);
		}

		if (neighborCount)
		{
			if (params->algo == ccPointCloudInterpolator::Parameters::MEDIAN)
			{
				//median
				std::vector<ScalarType> values;
				values.resize(neighborCount);
				unsigned medianIndex = std::max(neighborCount / 2, 1u) - 1;

				for (unsigned j = 0; j < sfCount; ++j)
				{
					const CCLib::ScalarField* sf = scalarFields->at(j).in;
					for (unsigned k = 0; k < neighborCount; ++k)
					{
						CCLib::DgmOctree::PointDescriptor& P = nNSS.pointsInNeighbourhood[k];
						values[k] = sf->getValue(P.pointIndex);
					}
					std::sort(values.begin(), values.end());
					
					ScalarType median = values[medianIndex];
					scalarFields->at(j).out->setValue(outPointIndex, median);
				}
			}
			else //average or weighted average
			{
				double sumW = 0;
				std::fill(sumValues.begin(), sumValues.end(), 0);
				for (unsigned k = 0; k < neighborCount; ++k)
				{
					CCLib::DgmOctree::PointDescriptor& P = nNSS.pointsInNeighbourhood[k];
					double w = 1.0;
					if (normalDistWeighting)
					{
						w = exp(-P.squareDistd / interpSigma2x2);
					}
					sumW += w;
					for (unsigned j = 0; j < sfCount; ++j)
					{
						sumValues[j] += w * scalarFields->at(j).in->getValue(P.pointIndex);
					}
				}

				if (sumW > 0)
				{
					for (unsigned j = 0; j < sfCount; ++j)
					{
						ScalarType s = static_cast<ScalarType>(sumValues[j] / sumW);
						scalarFields->at(j).out->setValue(outPointIndex, s);
					}
				}
				else
				{
					//we assume the scalar fields have all been initialized to NAN_VALUE
				}
			}
		}
		else
		{
			//we assume the scalar fields have all been initialized to NAN_VALUE
		}

		if (nProgress && !nProgress->oneStep())
		{
			return false;
		}
	}

	return true;
}

bool ccPointCloudInterpolator::InterpolateScalarFieldsFrom(	ccPointCloud* destCloud,
															ccPointCloud* srcCloud,
															const std::vector<int>& inSFIndexes,
															const Parameters& params,
															CCLib::GenericProgressCallback* progressCb/*=0*/,
															unsigned char octreeLevel/*=0*/)
{
	if (!destCloud || !srcCloud || srcCloud->size() == 0 || srcCloud->getNumberOfScalarFields() == 0)
	{
		ccLog::Warning("[InterpolateScalarFieldsFrom] Invalid/empty input cloud(s)!");
		return false;
	}

	//check that both bounding boxes intersect!
	ccBBox box = destCloud->getOwnBB();
	ccBBox otherBox = srcCloud->getOwnBB();

	CCVector3 dimSum = box.getDiagVec() + otherBox.getDiagVec();
	CCVector3 dist = box.getCenter() - otherBox.getCenter();
	if (	fabs(dist.x) > dimSum.x / 2
		||	fabs(dist.y) > dimSum.y / 2
		||	fabs(dist.z) > dimSum.z / 2)
	{
		ccLog::Warning("[InterpolateScalarFieldsFrom] Clouds are too far from each other! Can't proceed.");
		return false;
	}

	//now copy the scalar fields
	bool overwrite = false;
	std::vector< SFPair > scalarFields;
	try
	{
		scalarFields.reserve(inSFIndexes.size());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory");
		return false;
	}
	for (size_t i = 0; i < inSFIndexes.size(); ++i)
	{
		int inSFIndex = inSFIndexes[i];
		if (inSFIndex < 0 || inSFIndex >= static_cast<int>(srcCloud->getNumberOfScalarFields()))
		{
			//invalid index
			ccLog::Warning(QString("[InterpolateScalarFieldsFrom] Source cloud has no scalar field with index #%1").arg(inSFIndex));
			assert(false);
			return false;
		}

		const char* sfName = srcCloud->getScalarFieldName(inSFIndex);
		int outSFIndex = destCloud->getScalarFieldIndexByName(sfName);
		if (outSFIndex < 0)
		{
			outSFIndex = destCloud->addScalarField(sfName);
			if (outSFIndex < 0)
			{
				ccLog::Error("Not enough memory!");
				return false;
			}
		}
		else
		{
			overwrite = true;
		}

		CCLib::ScalarField* inSF = srcCloud->getScalarField(inSFIndex);
		CCLib::ScalarField* outSF = destCloud->getScalarField(outSFIndex);
		scalarFields.push_back(SFPair(inSF, outSF));

		outSF->fill(NAN_VALUE);
	}

	if (params.method == Parameters::NEAREST_NEIGHBOR)
	{
		//compute the closest-point set of 'this cloud' relatively to 'input cloud'
		//(to get a mapping between the resulting vertices and the input points)
		QSharedPointer<CCLib::ReferenceCloud> CPSet = destCloud->computeCPSet(*srcCloud, progressCb, octreeLevel);
		if (!CPSet)
		{
			return false;
		}

		unsigned CPSetSize = CPSet->size();
		assert(CPSetSize == destCloud->size());

		//now copy the scalar fields
		for (SFPair& sfPair : scalarFields)
		{
			for (unsigned i = 0; i < CPSetSize; ++i)
			{
				unsigned pointIndex = CPSet->getPointGlobalIndex(i);
				sfPair.out->setValue(i, sfPair.in->getValue(pointIndex));
			}
		}
	}
	else
	{
		if ((params.method == Parameters::K_NEAREST_NEIGHBORS && params.knn == 0) ||
			(params.method == Parameters::RADIUS && params.radius <= 0))
		{
			//invalid input
			ccLog::Warning("[InterpolateScalarFieldsFrom] Invalid input");
			assert(false);
			return false;
		}

		assert(srcCloud && destCloud);

		//we spatially 'synchronize' the octrees
		CCLib::DgmOctree *_srcOctree = nullptr;
		CCLib::DgmOctree *_destOctree = nullptr;
		CCLib::DistanceComputationTools::SOReturnCode soCode = CCLib::DistanceComputationTools::synchronizeOctrees(
			srcCloud,
			destCloud,
			_srcOctree,
			_destOctree,
			/*maxSearchDist*/0,
			progressCb);
		
		QScopedPointer<CCLib::DgmOctree> srcOctree(_srcOctree);
		QScopedPointer<CCLib::DgmOctree> destOctree(_destOctree);

		if (soCode != CCLib::DistanceComputationTools::SYNCHRONIZED)
		{
			//not enough memory (or invalid input)
			ccLog::Warning("[InterpolateScalarFieldsFrom] Failed to build the octrees");
			return false;
		}

		if (octreeLevel == 0)
		{
			if (params.method == ccPointCloudInterpolator::Parameters::K_NEAREST_NEIGHBORS)
			{
				octreeLevel = srcOctree->findBestLevelForAGivenPopulationPerCell(params.knn);
			}
			else
			{
				octreeLevel = srcOctree->findBestLevelForAGivenNeighbourhoodSizeExtraction(params.radius);
			}
		}

		try
		{
			//additional parameters
			void* additionalParameters[] = {	reinterpret_cast<void*>(srcCloud),
												reinterpret_cast<void*>(srcOctree.data()),
												reinterpret_cast<void*>(&scalarFields),
												(void*)(&params)
			};

			if (destOctree->executeFunctionForAllCellsAtLevel(	octreeLevel,
																cellSFInterpolator,
																additionalParameters,
																true,
																progressCb,
																"Scalar field interpolation",
																0) == 0)
			{
				//something went wrong
				ccLog::Warning("[InterpolateScalarFieldsFrom] Failed to perform the interpolation");
				return false;
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			ccLog::Warning("[InterpolateScalarFieldsFrom] Not enough memory");
			return false;
		}
	}

	//now copy the scalar fields
	for (SFPair& sfPair : scalarFields)
	{
		sfPair.out->computeMinAndMax();
	}

	if (overwrite)
	{
		ccLog::Warning("[InterpolateScalarFieldsFrom] Some scalar fields with the same names have been overwritten");
	}

	//We must update the VBOs
	destCloud->colorsHaveChanged();
	
	return true;
}
