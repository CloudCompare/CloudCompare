#ifndef RANSACSHAPEDETECTOR_HEADER
#define RANSACSHAPEDETECTOR_HEADER
#include "PointCloud.h"
#include "PrimitiveShapeConstructor.h"
#include <MiscLib/Vector.h>
#include <MiscLib/NoShrinkVector.h>
#include <utility>
#include "Candidate.h"
#include <MiscLib/RefCountPtr.h>
#include "Octree.h"
#include <GfxTL/NullClass.h>
#include <GfxTL/ImmediateTreeDataKernels.h>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE RansacShapeDetector
{
	public:
		struct Options
		{
			Options()
			: m_epsilon(0.01f)
			, m_normalThresh(0.95f)
			, m_minSupport(100)
			, m_bitmapEpsilon(0.01f)
			, m_fitting(LS_FITTING)
			, m_probability(0.001f)
			{}
			float m_epsilon;
			float m_normalThresh;
			unsigned int m_minSupport;
			float m_bitmapEpsilon;
			enum { NO_FITTING, LS_FITTING } m_fitting;
			float m_probability;
		};
		RansacShapeDetector();
		RansacShapeDetector(const Options &options);
		virtual ~RansacShapeDetector();
		void Add(PrimitiveShapeConstructor *c);
		size_t Detect(PointCloud &pc, size_t begin, size_t end,
			MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > *shapes);
		void AutoAcceptSize(size_t s) { m_autoAcceptSize = s; }
		size_t AutoAcceptSize() const { return m_autoAcceptSize; }
		const Options &GetOptions() const { return m_options; }

	private:
		typedef MiscLib::Vector< PrimitiveShapeConstructor * > ConstructorsType;
		typedef MiscLib::NoShrinkVector< Candidate > CandidatesType;
		bool DrawSamplesStratified(const IndexedOctreeType &oct,
			size_t numSamples, size_t depth,
			const MiscLib::Vector< int > &shapeIndex,
			MiscLib::Vector< size_t > *samples,
			const IndexedOctreeType::CellType **node) const;
		PrimitiveShape *Fit(bool allowDifferentShapes,
			const PrimitiveShape &initialShape, const PointCloud &pc,
			MiscLib::Vector< size_t >::const_iterator begin,
			MiscLib::Vector< size_t >::const_iterator end,
			std::pair< size_t, float > *score) const;
		float CandidateFailureProbability(float candidateSize,
			float numberOfPoints, float drawnCandidates, float levels) const
		{
			return std::min(std::pow(1.f - candidateSize
				/ (numberOfPoints * levels * (1 << (m_reqSamples - 1))),
				drawnCandidates), 1.f);
		}
		float UpdateAcceptedFailureProbability(
			float currentFailureProbability, size_t numTries) const
		{
			//currentFailureProbability *= 1.1f;//1.61;/*powf(1.05, 1<<numTries);*/ //1.1f;
			if(currentFailureProbability > 1.f)
				currentFailureProbability = 1.f;
			return currentFailureProbability;
		}
		template< class ScoreVisitorT >
		void GenerateCandidates(
			const IndexedOctreeType &globalOctTree,
			const MiscLib::Vector< ImmediateOctreeType * > &octrees,
			const PointCloud &pc, ScoreVisitorT &scoreVisitor,
			size_t currentSize, size_t numInvalid,
			const MiscLib::Vector< double > &sampleLevelProbSum,
			size_t *drawnCandidates,
			MiscLib::Vector< std::pair< float, size_t > > *sampleLevelScores,
			float *bestExpectedValue,
			CandidatesType *candidates) const;
		template< class ScoreVisitorT >
		bool FindBestCandidate(CandidatesType &candidates,
			const MiscLib::Vector< ImmediateOctreeType * > &octrees,
			const PointCloud &pc, ScoreVisitorT &scoreVisitor,
			size_t currentSize, size_t drawnCandidates,
			size_t numInvalid, size_t minSize, float numLevels,
			float *maxForgottenCandidate, float *candidateFailProb) const;
		size_t StatBucket(float score) const;
		void UpdateLevelWeights(float factor,
			const MiscLib::Vector< std::pair< float, size_t > > &levelScores,
			MiscLib::Vector< double > *sampleLevelProbability) const;
	private:
		ConstructorsType m_constructors;
		Options m_options;
		size_t m_maxCandTries;
		size_t m_reqSamples;
		size_t m_autoAcceptSize;
};

#endif
