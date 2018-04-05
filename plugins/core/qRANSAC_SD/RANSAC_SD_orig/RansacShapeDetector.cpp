#ifdef WIN32
#include <windows.h>
#endif
#include "RansacShapeDetector.h"
#include <algorithm>
#include <functional>
#include <ctime>
#include <deque>
#include <iostream>
#include <MiscLib/Random.h>
#include "Candidate.h"
#include <MiscLib/Performance.h>
#include "Octree.h"
#include "ScorePrimitiveShapeVisitor.h"
#include "FlatNormalThreshPointCompatibilityFunc.h"
#ifdef DOPARALLEL
#include <omp.h>
#endif
#undef max
#undef min

using namespace MiscLib;

RansacShapeDetector::RansacShapeDetector()
: m_maxCandTries(20)
, m_reqSamples(0)
, m_autoAcceptSize(0)
{}

RansacShapeDetector::RansacShapeDetector(const Options &options)
: m_options(options)
, m_maxCandTries(20)
, m_reqSamples(0)
, m_autoAcceptSize(0)
{}

RansacShapeDetector::~RansacShapeDetector()
{
	for(ConstructorsType::iterator i = m_constructors.begin(),
		iend = m_constructors.end(); i != iend; ++i)
		(*i)->Release();
}

void RansacShapeDetector::Add(PrimitiveShapeConstructor *c)
{
	c->AddRef();
	m_constructors.push_back(c);
	if(c->RequiredSamples() > m_reqSamples)
		m_reqSamples = c->RequiredSamples();
}

size_t RansacShapeDetector::StatBucket(float score) const
{
	return (size_t)std::max(0.f, std::floor((std::log(score) - std::log((float)m_options.m_minSupport)) / std::log(1.21f)) + 1);
}

/*
 * Function Detect !!!!
 */

void RansacShapeDetector::UpdateLevelWeights(float factor,
	const MiscLib::Vector< std::pair< float, size_t > > &levelScores,
	MiscLib::Vector< double > *sampleLevelProbability) const
{
	MiscLib::Vector< double > newSampleLevelProbability(
		sampleLevelProbability->size());
	double newSampleLevelProbabilitySum = 0;
	for(size_t i = 0; i < newSampleLevelProbability.size(); ++i)
	{
		if((*sampleLevelProbability)[i] > 0)
			newSampleLevelProbability[i] = (levelScores[i].first / (*sampleLevelProbability)[i]);
		else
			newSampleLevelProbability[i] = 0;
		newSampleLevelProbabilitySum += newSampleLevelProbability[i];
	}
	double newSum = 0;
	for(size_t i = 0; i < newSampleLevelProbability.size(); ++i)
	{
		newSampleLevelProbability[i] = .9f * newSampleLevelProbability[i] + .1f * newSampleLevelProbabilitySum/levelScores.size();
		newSum += newSampleLevelProbability[i];
	}
	for(size_t i = 0; i < sampleLevelProbability->size(); ++i)
	{
		(*sampleLevelProbability)[i] = (1.f - factor) * (*sampleLevelProbability)[i] +
			factor * (newSampleLevelProbability[i] / newSum);
	}
}

template< class ScoreVisitorT >
void RansacShapeDetector::GenerateCandidates(
	const IndexedOctreeType &globalOctree,
	const MiscLib::Vector< ImmediateOctreeType * > &octrees,
	const PointCloud &pc, ScoreVisitorT &scoreVisitor,
	size_t currentSize, size_t numInvalid,
	const MiscLib::Vector< double > &sampleLevelProbSum,
	size_t *drawnCandidates,
	MiscLib::Vector< std::pair< float, size_t > > *sampleLevelScores,
	float *bestExpectedValue,
	CandidatesType *candidates) const
{
	size_t genCands = 0;

#ifdef DOPARALLEL
	#pragma omp parallel
#endif
	{
	ScoreVisitorT scoreVisitorCopy(scoreVisitor);
#ifdef DOPARALLEL
	#pragma omp for schedule(dynamic, 10) reduction(+:genCands)
#endif
	for(int candIter = 0; candIter < 200; ++candIter)
	{
		// pick a sample level
		double s = ((double)rand()) / (double)RAND_MAX;
		size_t sampleLevel = 0;
		for(; sampleLevel < sampleLevelProbSum.size() - 1; ++sampleLevel)
			if(sampleLevelProbSum[sampleLevel] >= s)
				break;
		// draw samples on current sample level in octree
		MiscLib::Vector< size_t > samples;
		const IndexedOctreeType::CellType *node;
		if(!DrawSamplesStratified(globalOctree, m_reqSamples, sampleLevel,
			scoreVisitorCopy.GetShapeIndex(), &samples, &node))
			continue;
		++genCands;
		// construct the candidates
		size_t c = samples.size();
		MiscLib::Vector< Vec3f > samplePoints(samples.size() << 1);
		for(size_t i = 0; i < samples.size(); ++i)
		{
			samplePoints[i] = globalOctree.at(samples[i]).pos;
			samplePoints[i + c] = globalOctree.at(samples[i]).normal;
		}
		// construct the different primitive shapes
		PrimitiveShape *shape;
		for(ConstructorsType::const_iterator i = m_constructors.begin(),
			iend = m_constructors.end(); i != iend; ++i)
		{
			if((*i)->RequiredSamples() > samples.size() 
					|| !(shape = (*i)->Construct(samplePoints)))
				continue;
			// verify shape
			std::pair< float, float > dn;
			bool verified = true;
			for(size_t i = 0; i < c; ++i)
			{
				shape->DistanceAndNormalDeviation(samplePoints[i], samplePoints[i + c], &dn);
				if(!scoreVisitorCopy.PointCompFunc()(dn.first, dn.second))
				{
					verified = false;
					break;
				}
			}
			if(!verified)
			{
				shape->Release();
				continue;
			}
			Candidate cand(shape, node->Level());
			cand.Indices(new MiscLib::RefCounted< MiscLib::Vector< size_t > >);
			cand.Indices()->Release();
			shape->Release();
			cand.ImproveBounds(octrees, pc, scoreVisitorCopy,
				currentSize, m_options.m_bitmapEpsilon, 1);
			if(cand.UpperBound() < m_options.m_minSupport)
			{
#ifdef DOPARALLEL
				#pragma omp critical
#endif
				{
				(*sampleLevelScores)[node->Level()].first += cand.ExpectedValue();
				++(*sampleLevelScores)[node->Level()].second;
				}
				continue;
			}

#ifdef DOPARALLEL
			#pragma omp critical
#endif
			{
				(*sampleLevelScores)[node->Level()].first += cand.ExpectedValue();
				++(*sampleLevelScores)[node->Level()].second;
				candidates->push_back(cand);
				if(cand.ExpectedValue() > *bestExpectedValue)
					*bestExpectedValue = cand.ExpectedValue();
			}
		}
	}
	}
	*drawnCandidates += genCands;
}

struct CandidateHeapPred
{
	bool operator()(const Candidate *a, const Candidate *b) const
	{
		return *a < *b;
	}
};

template< class ScoreVisitorT >
bool RansacShapeDetector::FindBestCandidate(CandidatesType &candidates,
	const MiscLib::Vector< ImmediateOctreeType * > &octrees, const PointCloud &pc,
	ScoreVisitorT &scoreVisitor, size_t currentSize,
	size_t drawnCandidates, size_t numInvalid, size_t minSize, float numLevels,
	float *maxForgottenCandidate, float *candidateFailProb) const
{
	if(!candidates.size())
		return false;
	size_t maxImproveSubsetDuringMaxSearch = octrees.size();
	// sort by expected value
	std::sort(candidates.begin(), candidates.end());
	// check if max is smaller than forgotten candidate
	if(candidates.size() && candidates.back().ExpectedValue() < *maxForgottenCandidate)
	{
		// drawn candidates is wrong!
		// need to correct the value
		drawnCandidates = std::max(candidates.size(), (size_t)1);
		*maxForgottenCandidate = 0;
	}

	MiscLib::Vector< Candidate * > candHeap;
	for(size_t i = candidates.size() - 1; i != -1; --i)
	{
		if(CandidateFailureProbability(
			candidates[i].ExpectedValue(),
			currentSize - numInvalid, drawnCandidates, numLevels) > m_options.m_probability)
			break;
		candHeap.push_back(&candidates[i]);
	}

	if(!candHeap.size())
	{
		return false;
	}

	std::make_heap(candHeap.begin(), candHeap.end(), CandidateHeapPred());

	MiscLib::Vector< Candidate * > beatenCands;
	Candidate *trial = candHeap.front();
	std::pop_heap(candHeap.begin(), candHeap.end(), CandidateHeapPred());
	candHeap.pop_back();
	float bestCandidateFailureProbability;
	while(candHeap.size())
	{
		if(trial->IsEquivalent(*candHeap.front(), pc, m_options.m_epsilon,
			m_options.m_normalThresh))
		{
			std::pop_heap(candHeap.begin(), candHeap.end(),
				CandidateHeapPred());
			candHeap.pop_back();
			continue;
		}
		bool isEquivalent = false;
		for(size_t j = 0; j < beatenCands.size(); ++j)
		{
			if(beatenCands[j]->IsEquivalent(*candHeap.front(), pc,
				m_options.m_epsilon, m_options.m_normalThresh))
			{
				isEquivalent = true;
				break;
			}
		}
		if(isEquivalent)
		{
			std::pop_heap(candHeap.begin(), candHeap.end(),
				CandidateHeapPred());
			candHeap.pop_back();
			continue;
		}
		bestCandidateFailureProbability = CandidateFailureProbability(
			trial->ExpectedValue(),
			currentSize - numInvalid, drawnCandidates, numLevels);
		while((bestCandidateFailureProbability <= m_options.m_probability)
			&& (*trial >= *candHeap.front())
			&& (trial->UpperBound() >= minSize)
			&& trial->ImproveBounds(octrees, pc, scoreVisitor, currentSize,
				m_options.m_bitmapEpsilon, octrees.size()))
		{
			bestCandidateFailureProbability = CandidateFailureProbability(
				trial->ExpectedValue(),
				currentSize - numInvalid, drawnCandidates, numLevels);
		}
		if(bestCandidateFailureProbability <= m_options.m_probability
			&& trial->UpperBound() >= minSize
			&& trial->ComputedSubsets() >= octrees.size()
			&& *trial >= *candHeap.front())
			break;
		if(bestCandidateFailureProbability <= m_options.m_probability
			&& trial->UpperBound() >= minSize)
		{
			candHeap.push_back(trial);
			std::push_heap(candHeap.begin(), candHeap.end(), CandidateHeapPred());
		}
		else if((int)trial->ComputedSubsets()
			> std::max(2, ((int)octrees.size()) - 2))
			beatenCands.push_back(trial);

		//nextCandidate
		trial = candHeap.front();
		std::pop_heap(candHeap.begin(), candHeap.end(), CandidateHeapPred());
		candHeap.pop_back();
	}
	bestCandidateFailureProbability = CandidateFailureProbability(
		trial->ExpectedValue(),
		currentSize - numInvalid, drawnCandidates, numLevels);
	while(bestCandidateFailureProbability <= m_options.m_probability
		&& trial->UpperBound() >= minSize
		&& trial->ImproveBounds(octrees, pc, scoreVisitor, currentSize,
			m_options.m_bitmapEpsilon, octrees.size()))
	{
		bestCandidateFailureProbability = CandidateFailureProbability(
			trial->ExpectedValue(),
			currentSize - numInvalid, drawnCandidates, numLevels);
	}
	if((bestCandidateFailureProbability > m_options.m_probability
		|| trial->UpperBound() < minSize)
		&& (!m_autoAcceptSize || trial->UpperBound() < m_autoAcceptSize))
	{
		return false;
	}
	std::sort(candidates.begin(), candidates.end());

	size_t bestCandidate = candidates.size() - 1;
	bestCandidateFailureProbability = CandidateFailureProbability(
		candidates.back().ExpectedValue(),
		currentSize - numInvalid, drawnCandidates, numLevels);

	for(size_t i = bestCandidate - 1; i != -1; --i)
	{
		float iFailProb = CandidateFailureProbability(candidates[i].ExpectedValue(),
			currentSize - numInvalid, drawnCandidates, numLevels);
		if(iFailProb > m_options.m_probability || candidates[i].UpperBound() < minSize
			|| candidates[i].UpperBound() < candidates[bestCandidate].LowerBound())
			break;
		// check if this is an identical candidate
		if(candidates[bestCandidate].IsEquivalent(candidates[i], pc,
			m_options.m_epsilon, m_options.m_normalThresh))
		{
			continue;
		}
		bool isEquivalent = false;
		for(size_t j = 0; j < beatenCands.size(); ++j)
		{
			if(beatenCands[j]->IsEquivalent(candidates[i], pc,
				m_options.m_epsilon, m_options.m_normalThresh))
			{
				isEquivalent = true;
				break;
			}
		}
		if(isEquivalent)
		{
			continue;
		}
		do
		{
			if(candidates[i].UpperBound() > candidates[bestCandidate].UpperBound()
				&& candidates[i].LowerBound() < candidates[bestCandidate].LowerBound())
			{
				bool dontBreak = candidates[i].ImproveBounds(octrees, pc, scoreVisitor,
						currentSize, m_options.m_bitmapEpsilon,
						maxImproveSubsetDuringMaxSearch);
				iFailProb = CandidateFailureProbability(candidates[i].ExpectedValue(),
					currentSize - numInvalid, drawnCandidates, numLevels);
				if(!dontBreak)
					break;
			}
			else if(candidates[bestCandidate].UpperBound() > candidates[i].UpperBound()
				&& candidates[bestCandidate].LowerBound() < candidates[i].LowerBound())
			{
				bool dontBreak = candidates[bestCandidate].ImproveBounds(octrees, pc,
					scoreVisitor, currentSize, m_options.m_bitmapEpsilon,
					maxImproveSubsetDuringMaxSearch);
				bestCandidateFailureProbability = CandidateFailureProbability(
					candidates[bestCandidate].ExpectedValue(),
					currentSize - numInvalid, drawnCandidates, numLevels);
				if(!dontBreak)
					break;
			}
			else
			{
				bool dontBreak = candidates[bestCandidate].ImproveBounds(octrees, pc,
					scoreVisitor, currentSize, m_options.m_bitmapEpsilon,
					maxImproveSubsetDuringMaxSearch);
				dontBreak = candidates[i].ImproveBounds(octrees, pc, scoreVisitor,
						currentSize, m_options.m_bitmapEpsilon,
						maxImproveSubsetDuringMaxSearch)
					|| dontBreak;
				iFailProb = CandidateFailureProbability(candidates[i].ExpectedValue(),
					currentSize - numInvalid, drawnCandidates, numLevels);
				bestCandidateFailureProbability = CandidateFailureProbability(
					candidates[bestCandidate].ExpectedValue(),
					currentSize - numInvalid, drawnCandidates, numLevels);
				if(!dontBreak)
					break;
			}
		}
		while(bestCandidateFailureProbability <= m_options.m_probability
			&& iFailProb <= m_options.m_probability
			&& candidates[i].UpperBound() >= minSize
			&& candidates[bestCandidate].UpperBound() >= minSize
			&& candidates[i].UpperBound() > candidates[bestCandidate].LowerBound()
			&& candidates[i].LowerBound() < candidates[bestCandidate].UpperBound()
			);
		if((
			candidates[i] > candidates[bestCandidate]
			|| bestCandidateFailureProbability > m_options.m_probability
			|| candidates[bestCandidate].UpperBound() < minSize)
			&& (iFailProb <= m_options.m_probability && candidates[i].UpperBound() >= minSize))
		{
			while(iFailProb <= m_options.m_probability && candidates[i].UpperBound() >= minSize
				&& candidates[i] > candidates[bestCandidate]
				&& candidates[i].ImproveBounds(octrees, pc, scoreVisitor,
					currentSize, m_options.m_bitmapEpsilon, octrees.size()))
			{
				iFailProb = CandidateFailureProbability(candidates[i].ExpectedValue(),
					currentSize - numInvalid, drawnCandidates, numLevels);
			}
			if(candidates[i] > candidates[bestCandidate])
			{
				beatenCands.push_back(&candidates[bestCandidate]);
				bestCandidate = i;
				bestCandidateFailureProbability = iFailProb;
			}
			else
				beatenCands.push_back(&candidates[i]);
		}
		else
			beatenCands.push_back(&candidates[i]);
		if(bestCandidateFailureProbability > m_options.m_probability
			|| candidates[bestCandidate].UpperBound() < minSize)
			break;
	} // end for

	while(candidates[bestCandidate].ImproveBounds(octrees, pc, scoreVisitor,
		currentSize, m_options.m_bitmapEpsilon, octrees.size()));

	bestCandidateFailureProbability = CandidateFailureProbability(
		candidates[bestCandidate].ExpectedValue(),
		currentSize - numInvalid, drawnCandidates, numLevels);

	if((bestCandidateFailureProbability <= m_options.m_probability
		&& candidates[bestCandidate].UpperBound() >= minSize)
		|| (m_autoAcceptSize && candidates[bestCandidate].UpperBound() >= m_autoAcceptSize))
	{
		std::swap(candidates.back(), candidates[bestCandidate]);
		*candidateFailProb = bestCandidateFailureProbability;
		return true;
	}
	std::sort(candidates.begin(), candidates.end()/*, std::greater< Candidate >()*/);
	return false;
}

size_t
RansacShapeDetector::Detect(PointCloud &pc, size_t beginIdx, size_t endIdx,
	MiscLib::Vector< std::pair< RefCountPtr< PrimitiveShape >, size_t > > *shapes)
{
	size_t pcSize = endIdx - beginIdx;
	/*
	 * Initialization part
	 */
	srand((unsigned int)time(NULL));
	rn_setseed((size_t)time(NULL));

	CandidatesType candidates;

	ScorePrimitiveShapeVisitor< FlatNormalThreshPointCompatibilityFunc,
		ImmediateOctreeType > subsetScoreVisitor(m_options.m_epsilon,
			m_options.m_normalThresh);
	ScorePrimitiveShapeVisitor< FlatNormalThreshPointCompatibilityFunc,
		IndexedOctreeType > globalScoreVisitor(3 * m_options.m_epsilon,
			m_options.m_normalThresh);

	// construct random subsets
	size_t subsets = std::max(int(std::floor(std::log((float)pcSize)/std::log(2.f)))-9, 2);
	GfxTL::AACube< GfxTL::Vector3Df > bcube;
	bcube.Bound(pc.begin() + beginIdx, pc.begin() + endIdx); 

	// construct stratified subsets
	MiscLib::Vector< ImmediateOctreeType * > octrees(subsets);
	for(size_t i = octrees.size(); i;)
	{
		--i;
		size_t subsetSize = pcSize;
		if(i)
		{
			subsetSize = subsetSize >> 1;
			MiscLib::Vector< size_t > subsetIndices(subsetSize);
			size_t bucketSize = pcSize / subsetSize;
			for(size_t j = 0; j < subsetSize; ++j)
			{
				size_t index = rn_rand() % bucketSize;
				index += j * bucketSize;
				if(index >= pcSize)
					index = pcSize - 1;
				subsetIndices[j] = index + beginIdx;
			}
			// move all the indices to the end
			std::sort(subsetIndices.begin(), subsetIndices.end(),
				std::greater< size_t >());
			for(size_t j = pcSize - 1, i = 0; i < subsetIndices.size(); --j, ++i)
				std::swap(pc[j + beginIdx], pc[subsetIndices[i]]);
		}
		octrees[i] = new ImmediateOctreeType;
		octrees[i]->ContainedData(&pc);
		octrees[i]->DataRange(pcSize - subsetSize + beginIdx,
			pcSize + beginIdx);
		octrees[i]->MaxBucketSize() = 20;
		octrees[i]->MaxSubdivisionLevel() = 10;
		octrees[i]->Build(bcube);
		pcSize -= subsetSize;
	}

	pcSize = endIdx - beginIdx;

	// construct one global octree
	MiscLib::Vector< size_t > globalOctreeIndices(pcSize);
	for(size_t i = 0; i < pcSize; ++i)
		globalOctreeIndices[i] = i + beginIdx;
	IndexedOctreeType globalOctree;
	globalOctree.MaxBucketSize() = 20;
	globalOctree.MaxSubdivisionLevel() = 10;
	globalOctree.IndexedData(globalOctreeIndices.begin(),
		globalOctreeIndices.end(), pc.begin());
	globalOctree.Build(bcube);
	size_t globalOctTreeMaxNodeDepth = globalOctree.MaxDepth();

	MiscLib::Vector< double > sampleLevelProbability(
		globalOctTreeMaxNodeDepth + 1);
	for(size_t i = 0; i < sampleLevelProbability.size(); ++i)
		sampleLevelProbability[i] = 1.0 / sampleLevelProbability.size();
	size_t drawnCandidates = 0; // keep track of number of candidates
		// that have been generated
	float maxForgottenCandidate = 0; // the maximum size of a canidate
		// that has been forgotten
	size_t numTries = 0; // number of loops since last successful candidate
	size_t numShapes = 0; // number of shapes found since the
		// last housekeeping
	size_t numInvalid = 0; // number of points that have been assigned
		// to a shape since the last housekeeping
	MiscLib::Vector< int > shapeIndex(pc.size(), -1); // maintains for every
		// point the index of the shape it has been assigned to or
		// -1 if the point is not assigned yet
	subsetScoreVisitor.SetShapeIndex(shapeIndex);
	globalScoreVisitor.SetShapeIndex(shapeIndex);
	size_t currentSize = pcSize;
	do
	{
		MiscLib::Vector< std::pair< float, size_t > > sampleLevelScores(
			sampleLevelProbability.size());
		for(size_t i = 0; i < sampleLevelScores.size(); ++i)
			sampleLevelScores[i] = std::make_pair(0.f, 0u);
		MiscLib::Vector< double >sampleLevelProbSum(sampleLevelProbability.size());
		sampleLevelProbSum[0] = sampleLevelProbability[0];
		for(size_t i = 1; i < sampleLevelProbSum.size() - 1; ++i)
			sampleLevelProbSum[i] = sampleLevelProbability[i] + 
				sampleLevelProbSum[i - 1];
		sampleLevelProbSum[sampleLevelProbSum.size() - 1] = 1;
		// generate candidates
		float bestExpectedValue = 0;
		if(candidates.size())
			bestExpectedValue = candidates.back().ExpectedValue();
		bestExpectedValue = std::min((float)(currentSize - numInvalid), bestExpectedValue);
		do
		{
			GenerateCandidates(globalOctree,
				octrees, pc, subsetScoreVisitor,
				currentSize, numInvalid,
				sampleLevelProbSum,
				&drawnCandidates,
				&sampleLevelScores,
				&bestExpectedValue,
				&candidates);
		}
		while(CandidateFailureProbability(bestExpectedValue,
				currentSize - numInvalid, drawnCandidates,
				globalOctTreeMaxNodeDepth) > m_options.m_probability
			&& CandidateFailureProbability(m_options.m_minSupport,
				currentSize - numInvalid, drawnCandidates,
				globalOctTreeMaxNodeDepth) > m_options.m_probability);
		// find the best candidate:
		float bestCandidateFailureProbability;
		float failureProbability = std::numeric_limits< float >::infinity();
		bool foundCandidate = false;
		size_t firstCandidateSize = 0;
		while(FindBestCandidate(candidates, octrees, pc, subsetScoreVisitor,
			currentSize, drawnCandidates, numInvalid,
			std::max((size_t)m_options.m_minSupport, (size_t)(0.8f * firstCandidateSize)),
			globalOctTreeMaxNodeDepth, &maxForgottenCandidate,
			&bestCandidateFailureProbability))
		{
			if(!foundCandidate)
			{
				// this is the first candidate
				firstCandidateSize = (size_t)candidates.back().LowerBound();
				// rescore levels
				for(size_t i = 0; i < sampleLevelScores.size(); ++i)
					sampleLevelScores[i] = std::make_pair(0.f, 0u);
				for(size_t i = 0; i < candidates.size(); ++i)
				{
					if(candidates[i].ExpectedValue() * 1.4f > candidates.back().ExpectedValue())
					{
						size_t candLevel = std::min(sampleLevelScores.size() - 1,
							candidates[i].Level());
						++sampleLevelScores[candLevel].first;
						++sampleLevelScores[candLevel].second;
					}
				}
				UpdateLevelWeights(0.5f, sampleLevelScores, &sampleLevelProbability);
			}
			foundCandidate = true;
			if(bestCandidateFailureProbability < failureProbability)
				failureProbability = bestCandidateFailureProbability;
			std::string candidateDescription;
			candidates.back().Shape()->Description(&candidateDescription);
			// do fitting
			if(m_options.m_fitting != Options::NO_FITTING)
			{
				candidates.back().GlobalScore(globalScoreVisitor, globalOctree);
				candidates.back().ConnectedComponent(pc, m_options.m_bitmapEpsilon);
				Candidate clone;
				candidates.back().Clone(&clone);
				float oldScore, newScore;
				size_t oldSize, newSize;
				// get the weight once
				newScore = clone.GlobalWeightedScore( globalScoreVisitor, globalOctree,
						pc, 3 * m_options.m_epsilon, m_options.m_normalThresh,
						m_options.m_bitmapEpsilon );
				newSize = std::max(clone.Size(), candidates.back().Size());
				bool allowDifferentShapes = false;
				size_t fittingIter = 0;
				do
				{
					++fittingIter;
					oldScore = newScore;
					oldSize = newSize;
					std::pair< size_t, float > score;
					PrimitiveShape *shape;
					if(shape = Fit(allowDifferentShapes, *clone.Shape(),
						pc, clone.Indices()->begin(), clone.Indices()->end(),
						&score))
					{
						clone.Shape(shape);
						newScore = clone.GlobalWeightedScore( globalScoreVisitor, globalOctree,
							pc, 3 * m_options.m_epsilon, m_options.m_normalThresh,
							m_options.m_bitmapEpsilon );
						newSize = clone.Size();
						shape->Release();
						if(newScore > oldScore && newSize > m_options.m_minSupport)
							clone.Clone(&candidates.back());
					}
					allowDifferentShapes = false;
				}
				while(newScore > oldScore && fittingIter < 3);
			}
			else
			{
				candidates.back().GlobalScore(globalScoreVisitor, globalOctree);
				candidates.back().ConnectedComponent(pc, m_options.m_bitmapEpsilon);
			}
			if(candidates.back().Size() == 0)
				std::cout << "ERROR: candidate size == 0 after fitting" << std::endl;
			// best candidate is ok!
			// remove the points
			shapes->push_back(std::make_pair(RefCountPtr< PrimitiveShape >(candidates.back().Shape()),
				candidates.back().Indices()->size()));
			for(size_t i = 0; i < candidates.back().Indices()->size(); ++i)
				shapeIndex[(*(candidates.back().Indices()))[i]] = numShapes;
			++numShapes;
			// update drawn candidates to reflect removal of points
			// get the percentage of candidates that are invalid
			drawnCandidates = std::pow(1.f - (candidates.back().Indices()->size() /
				float(currentSize - numInvalid)), 3.f) * drawnCandidates;
			numInvalid += candidates.back().Indices()->size();
			candidates.pop_back();
			if(numInvalid > currentSize / 4) // more than half of the points assigned?
			{
				// do a housekeeping step
				//this is a two stage procedure:
				// 1) determine the new address of each Point and store it in the shapeIndex array
				// 2) swap Points according to new addresses
				size_t begin = beginIdx, end = beginIdx + currentSize;
				
				// these hold the address ranges for the lately detected shapes
				MiscLib::Vector< size_t > shapeIterators(numShapes);
				int shapeIt = shapes->size() - numShapes;
				for(size_t i = 0; i < numShapes; ++i, ++shapeIt)
					shapeIterators[i] = end -= ((*shapes)[shapeIt]).second;

				MiscLib::Vector< size_t > subsetSizes(octrees.size(), 0);
				for(size_t i = beginIdx, j = 0; i < beginIdx + currentSize; ++i)
					if(shapeIndex[i] < 0)
					{
						if(i >= octrees[j]->end() - pc.begin() + beginIdx
							&& j < octrees.size() - 1)
							++j;
						shapeIndex[i] = begin++;
						++subsetSizes[j];
					}
					else
						shapeIndex[i] = shapeIterators[shapeIndex[i]]++;

				// check if small subsets should be merged
				size_t mergedSubsets = 0;
				if(subsetSizes[0] < 500 && subsetSizes.size() > 1)
				{
					// should be merged
					while(subsetSizes[0] < 500 && subsetSizes.size() > 1)
					{
						subsetSizes[1] += subsetSizes[0];
						subsetSizes.erase(subsetSizes.begin());
						delete octrees[0];
						octrees.erase(octrees.begin());
						++mergedSubsets;
					}
				}

				// reindex global octree
				size_t minInvalidIndex = currentSize - numInvalid + beginIdx;
				int j = 0;
#ifdef DOPARALLEL
				#pragma omp parallel for schedule(static)
#endif
				for(int i = 0; i < static_cast<int>(globalOctreeIndices.size()); ++i)
					if(shapeIndex[globalOctreeIndices[i]] < minInvalidIndex)
						globalOctreeIndices[j++] = shapeIndex[globalOctreeIndices[i]];
				globalOctreeIndices.resize(currentSize - numInvalid);

				// reindex candidates (this also recomputes the bounds)
#ifdef DOPARALLEL
				#pragma omp parallel for schedule(static)
#endif
				for(int i = 0; i < static_cast<int>(candidates.size()); ++i)
					candidates[i].Reindex(shapeIndex, minInvalidIndex, mergedSubsets,
						subsetSizes, pc, currentSize - numInvalid, m_options.m_epsilon,
						m_options.m_normalThresh, m_options.m_bitmapEpsilon);

				//regarding swapping it is best to swap both the addresses and the data
				for(size_t i = beginIdx; i < beginIdx + currentSize; ++i)
					while(i != shapeIndex[i])
					{
						pc.swapPoints(i, shapeIndex[i]);
						std::swap(shapeIndex[i], shapeIndex[shapeIndex[i]]);
					}
					
				numInvalid = 0;

				// rebuild subset octrees
				if(mergedSubsets) // the octree for the first subset has to be constructed
				{
					//std::cout << "Attention: Merged " << mergedSubsets << " subsets!" << std::endl;
					MiscLib::Vector< size_t > shuffleIndices(beginIdx + subsetSizes[0]),
						reindex(beginIdx + subsetSizes[0]);
					for(size_t i = 0; i < shuffleIndices.size(); ++i)
						shuffleIndices[i] = i;
					delete octrees[0];
					octrees[0] = new ImmediateOctreeType();
					octrees[0]->ContainedData(&pc);
					octrees[0]->DataRange(beginIdx, beginIdx + subsetSizes[0]);
					octrees[0]->MaxBucketSize() = 20;
					octrees[0]->MaxSubdivisionLevel() = 10;
					octrees[0]->ShuffleIndices(&shuffleIndices);
					octrees[0]->Build(bcube);
					octrees[0]->ShuffleIndices(NULL);
					for(size_t i = 0; i < shuffleIndices.size(); ++i)
						reindex[shuffleIndices[i]] = i;
					// reindex global octree
#ifdef DOPARALLEL
					#pragma omp parallel for schedule(static)
#endif
					for(int i = 0; i < static_cast<int>(globalOctreeIndices.size()); ++i)
						if(globalOctreeIndices[i] < reindex.size())
							globalOctreeIndices[i] = reindex[globalOctreeIndices[i]];
					// reindex candidates
#ifdef DOPARALLEL
					#pragma omp parallel for schedule(static, 100)
#endif
					for(int i = 0; i < static_cast<int>(candidates.size()); ++i)
						candidates[i].Reindex(reindex);
					for(size_t i = 1, begin = subsetSizes[0] + beginIdx;
						i < octrees.size(); begin += subsetSizes[i], ++i)
					{
						octrees[i]->DataRange(begin, begin + subsetSizes[i]);
						octrees[i]->Rebuild();
						if(octrees[i]->Root()->Size() != subsetSizes[i])
							std::cout << "ERROR IN REBUILD!!!!" << std::endl;
					}
				}
				else
					for(size_t i = 0, begin = beginIdx; i < octrees.size();
						begin += subsetSizes[i], ++i)
					{
						octrees[i]->DataRange(begin, begin + subsetSizes[i]);
						octrees[i]->Rebuild();
					}

				//so everything is in its correct place, but we need to update the global octree ranges
				currentSize = globalOctreeIndices.size();

				globalOctree.IndexedRange(globalOctreeIndices.begin(),
					globalOctreeIndices.end());
				globalOctTreeMaxNodeDepth = globalOctree.Rebuild();
				if(globalOctree.Root()->Size() != globalOctreeIndices.size())
					std::cout << "ERROR IN GLOBAL REBUILD!" << std::endl;
				sampleLevelProbability.resize(globalOctTreeMaxNodeDepth + 1);

				//shapeIndex.resize(globalOctreeIndices.size());
				std::fill(shapeIndex.begin() + beginIdx,
					shapeIndex.begin() + beginIdx + currentSize, -1);
				numShapes = 0;
			}
			else
			{
				// the bounds of the candidates have become invalid and have to be
				// recomputed
#ifdef DOPARALLEL
				#pragma omp parallel for schedule(static, 100)
#endif
				for(int i = 0; i < static_cast<int>(candidates.size()); ++i)
					candidates[i].RecomputeBounds(octrees, pc, subsetScoreVisitor,
						currentSize - numInvalid, m_options.m_epsilon,
						m_options.m_normalThresh, m_options.m_bitmapEpsilon);
			}
			// remove all candidates that have become obsolete
			std::sort(candidates.begin(), candidates.end(), std::greater< Candidate >());
			size_t remainingCandidates = 0;
			for(size_t i = 0; i < candidates.size(); ++i)
				if(candidates[i].ExpectedValue() >= m_options.m_minSupport
					&& candidates[i].Size() > 0)
					candidates[remainingCandidates++] = candidates[i];
			candidates.resize(remainingCandidates);
		} // Ende abgrasen
		if(foundCandidate)
		{
			std::sort(candidates.begin(), candidates.end(), std::greater< Candidate >());
			size_t remainingCandidates = 0;
			size_t nonConnectedCount = 0;
			for(size_t i = 0; i < candidates.size(); ++i)
				if(candidates[i].ExpectedValue() >= m_options.m_minSupport &&
					(candidates[i].ComputedSubsets() > octrees.size() - 3 ||
					nonConnectedCount++ < 500))
					candidates[remainingCandidates++] = candidates[i];
				else
					if(candidates[i].ExpectedValue() > maxForgottenCandidate)
						maxForgottenCandidate = candidates[i].ExpectedValue();
			candidates.resize(remainingCandidates);

			numTries = 0;
		}
		else
		{
			numTries++;
		}
	}
	while(CandidateFailureProbability(m_options.m_minSupport, currentSize - numInvalid,
		drawnCandidates, globalOctTreeMaxNodeDepth) > m_options.m_probability
		&& (currentSize - numInvalid) >= m_options.m_minSupport);

	if(numInvalid)
	{
		// rearrange the last shapes
		//this is a two stage procedure:
		// 1) determine the new address of each Point and store it in the shapeIndex array
		// 2) swap Points according to new addresses
		size_t begin = beginIdx, end = beginIdx + currentSize;
		// these hold the address ranges for the lately detected shapes
		MiscLib::Vector< size_t > shapeIterators(numShapes);
		int shapeIt = shapes->size() - numShapes;
		for(size_t i = 0; i < numShapes; ++i, ++shapeIt)
			shapeIterators[i] = end -= ((*shapes)[shapeIt]).second;

		for(size_t i = beginIdx; i < beginIdx + currentSize; ++i)
			if(shapeIndex[i] < 0)
				shapeIndex[i] = begin++;
			else
				shapeIndex[i] = shapeIterators[shapeIndex[i]]++;

		//regarding swapping it is best to swap both the addresses and the data
		for(size_t i = beginIdx; i < beginIdx + currentSize; ++i)
			while(i != shapeIndex[i])
			{
				pc.swapPoints(i, shapeIndex[i]);
				std::swap(shapeIndex[i], shapeIndex[shapeIndex[i]]);
			}
	}
	// clean up subset octrees
	for(size_t i = 0; i < octrees.size(); ++i)
		delete octrees[i];
	// optimize parametrizations
	size_t eidx = endIdx;
	for(size_t i = 0; i < shapes->size(); ++i)
	{
		size_t bidx = eidx - (*shapes)[i].second;
		(*shapes)[i].first->OptimizeParametrization(pc, bidx, eidx,
			m_options.m_bitmapEpsilon);
		eidx = bidx;
	}

	// prune nonsense shapes
	for(size_t i = shapes->size(); i != 0; --i)
	{
		if(shapes->at(i - 1).second == 0)
			shapes->erase(shapes->begin() + i - 1);
	}
	return currentSize - numInvalid;
}

bool RansacShapeDetector::DrawSamplesStratified(const IndexedOctreeType &oct,
	size_t numSamples, size_t depth,
	const MiscLib::Vector< int > &shapeIndex,
	MiscLib::Vector< size_t > *samples,
	const IndexedOctreeType::CellType **node) const
{
	for(size_t tries = 0; tries < m_maxCandTries; tries++)
	{
		samples->clear();
		//get first point, which also determines octree cell
		size_t first;
		do
		{
			first = oct.Dereference(rn_rand() % oct.size());
		}
		while(shapeIndex[first] != -1);
		samples->push_back(first);

		std::pair< size_t, size_t > nodeRange;
		*node = oct.NodeContainingPoint(oct.at(first), depth, numSamples,
			&nodeRange);

		if((*node)->Size() < numSamples)
			continue;

		while(samples->size() < numSamples)
		{
			size_t i, iter = 0;
			do
			{
				i = oct.Dereference(rn_rand() % (*node)->Size()
					+ nodeRange.first);
			}
			while( ( shapeIndex[i] != -1
					|| std::find(samples->begin(), samples->end(), i) != samples->end() )
					&& iter++ < 40);
			if(iter >= 40)
				break;
			samples->push_back(i);
		}

		if(samples->size() == numSamples)
			return true;
	}
	return false;
}

PrimitiveShape *RansacShapeDetector::Fit(bool allowDifferentShapes,
	const PrimitiveShape &initialShape, const PointCloud &pc,
	MiscLib::Vector< size_t >::const_iterator begin,
	MiscLib::Vector< size_t >::const_iterator end,
	std::pair< size_t, float > *score) const
{
	if(!m_constructors.size())
		return NULL;
	PrimitiveShape *bestShape = NULL;
	if(m_options.m_fitting == Options::LS_FITTING)
		bestShape = initialShape.LSFit(pc, m_options.m_epsilon,
			m_options.m_normalThresh, begin, end, score);
	return bestShape;
}
