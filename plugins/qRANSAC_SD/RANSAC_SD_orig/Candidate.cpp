#include "Candidate.h"

#if !defined(_WIN32) && !defined(WIN32)
#include <unistd.h>
#endif

Candidate::Candidate()
: m_subset(0)
, m_lowerBound(0)
, m_upperBound(0)
, m_level(0)
, m_hasConnectedComponent(false)
{}

Candidate::Candidate(PrimitiveShape *shape, size_t level)
: m_shape(shape)
, m_subset(0)
, m_lowerBound(0)
, m_upperBound(0)
, m_level(level)
, m_hasConnectedComponent(false)
{}

void Candidate::Reset()
{
	m_indices->clear();
	m_subset = 0;
	m_lowerBound = 0;
	m_upperBound = 0;
	m_hasConnectedComponent = false;
	m_score = 0;
}

void Candidate::Reindex(const MiscLib::Vector< int > &newIndices, int minInvalidIndex,
	size_t mergedSubsets, const MiscLib::Vector< size_t > &subsetSizes,
	const PointCloud &pc, size_t currentSize, float epsilon, float normalThresh,
	float bitmapEpsilon)
{
	size_t i = 0, j = 0;
	for(; i < m_indices->size(); ++i)
		if(newIndices[(*m_indices)[i]] < minInvalidIndex)
			(*m_indices)[j++] = newIndices[(*m_indices)[i]];
	if(m_subset <= mergedSubsets)
	{
		m_hasConnectedComponent = false;
		m_subset = 0;
		m_indices->clear();
		m_lowerBound = 0;
		m_upperBound = 0; // forget this candidate
		m_score = 0;
		return;
	}
	else
	{
		m_indices->resize(j);
		m_subset -= mergedSubsets;
		if(m_subset >= subsetSizes.size())
			// do connected component if all subsets have been computed
			ConnectedComponent(pc, bitmapEpsilon);		
	}
	size_t sampledPoints = 0,
		endi = std::min(m_subset, subsetSizes.size());;
	for(i = 0; i < endi; ++i)
		sampledPoints += subsetSizes[i];

	GetBounds(sampledPoints, currentSize);
}

void Candidate::Reindex(const MiscLib::Vector< size_t > &reindex)
{
	size_t reindexSize = reindex.size();
	for(size_t i = 0; i < m_indices->size(); ++i)
		if(m_indices->at(i) < reindexSize)
			m_indices->at(i) = reindex[m_indices->at(i)];
}

float Candidate::WeightedScore(const PointCloud &pc, float epsilon,
	float normalThresh) const
{
	float score = 0;
#ifdef DOPARALLEL
	#pragma omp parallel for schedule(static) reduction(+:score)
#endif
	for(intptr_t i = 0; i < (intptr_t)m_indices->size(); ++i)
		score += weigh(m_shape->Distance(pc[(*m_indices)[i]].pos), epsilon);
	return score;
}

void Candidate::ConnectedComponent(const PointCloud &pc, float bitmapEpsilon, float* borderRatio )
{
	size_t connectedSize = m_shape->ConnectedComponent(pc, bitmapEpsilon, m_indices, true, borderRatio);
	m_indices->resize(connectedSize);
	m_lowerBound = m_upperBound = (float)m_indices->size();
}

bool Candidate::IsEquivalent(const Candidate &c, const PointCloud &pc,
	float epsilon, float normalThresh) const
{
	if(m_shape->Identifier() != c.m_shape->Identifier())
		return false;
	size_t correct = 0;
	size_t size = std::min(m_indices->size(), (size_t)9);
	for(size_t i = 0; i < size; ++i)
	{
		std::pair< float, float > dn;
		size_t idx = MiscLib::rn_rand() % m_indices->size();
		c.m_shape->DistanceAndNormalDeviation(
			pc[m_indices->at(idx)].pos,
			pc[m_indices->at(idx)].normal, &dn);
		if(dn.first < epsilon && fabs(dn.second) > normalThresh)
			++correct;
	}
	size_t tested = size;
	size = std::min(c.m_indices->size(), (size_t)9);
	for(size_t i = 0; i < size; ++i)
	{
		std::pair< float, float > dn;
		size_t idx = MiscLib::rn_rand() % c.m_indices->size();
		m_shape->DistanceAndNormalDeviation(
			pc[c.m_indices->at(idx)].pos,
			pc[c.m_indices->at(idx)].normal, &dn);
		if(dn.first < epsilon && fabs(dn.second) > normalThresh)
			++correct;
	}
	tested += size;
	return correct >= 2 * tested / 3;
}

float Candidate::GetVariance( const PointCloud &pc )
{
	float variance = 0.0f;
	float dev;

	if( m_indices->size() > 0 )
	{
		// first pass - get expectancy
		float expectancy = 0.0f;
		for( int i = 0; i < m_indices->size(); ++i )
			expectancy += fabs( m_shape->NormalDeviation( pc[(*m_indices)[i]].pos, pc[(*m_indices)[i]].normal));

		expectancy /= static_cast<float>( m_indices->size());

		// second pass - get real variance
		for( int i = 0; i < m_indices->size(); ++i )
		{
			dev = fabs( m_shape->NormalDeviation( pc[(*m_indices)[i]].pos, pc[(*m_indices)[i]].normal)) - expectancy;
			variance += dev * dev;
		}

		variance /= static_cast<float>( m_indices->size());
	}
	else
		variance = 1.0f;

	return variance;
}

float Candidate::GetPseudoVariance( const PointCloud &pc )
{
	float variance = 0.0f;
	float dev;

	for( int i = 0; i < m_indices->size(); ++i )
	{
		dev = fabs( m_shape->NormalDeviation( pc[(*m_indices)[i]].pos, pc[(*m_indices)[i]].normal)) - 1.0f;
		variance += dev * dev;
	}
	variance /= static_cast<float>( m_indices->size());

	return variance;
}

void Candidate::GetScore( const PointCloud& pc, float bitmapEpsilon, bool doFiltering )
{
	GetScoreMaxCCSize( pc, bitmapEpsilon, doFiltering );
}

void Candidate::GetScoreMaxCCSize( const PointCloud& pc, float bitmapEpsilon, bool doFiltering )
{
	size_t connectedSize = m_shape->ConnectedComponent(pc, bitmapEpsilon, 
					m_indices, doFiltering );
	m_indices->resize(connectedSize);

	m_score = connectedSize;
}

void Candidate::GetScoreMaxCCMinBorder( const PointCloud& pc, float bitmapEpsilon, bool doFiltering )
{
	float borderRatio;
	size_t connectedSize = m_shape->ConnectedComponent(pc, bitmapEpsilon, 
					m_indices, doFiltering, &borderRatio);
	m_indices->resize(connectedSize);

	m_score = connectedSize * size_t(( 1.0f - borderRatio ) * ( 1.0f - GetVariance( pc )));
}
