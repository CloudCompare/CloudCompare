#include "../ChamferDistanceTransformSigned.h"

//Local
#include <GenericProgressCallback.h>

using namespace CCLib;

//! 3x3x3 neighborhood
const char Neighbours333[27][4] = {
	{-1,-1,-1, -1},
	{-1,-1, 0, -1},
	{-1,-1, 1, -1},
	{-1, 0,-1, -1},
	{-1, 0, 0, -1},
	{-1, 0, 1, -1},
	{-1, 1,-1, -1},
	{-1, 1, 0, -1},
	{-1, 1, 1, -1},
	{ 0,-1,-1, -1},
	{ 0,-1, 0, -1},
	{ 0,-1, 1, -1},
	{ 0, 0,-1, -1},
	{ 0, 0, 0, -1},
	{ 0, 0, 1, -1},
	{ 0, 1,-1, -1},
	{ 0, 1, 0, -1},
	{ 0, 1, 1, -1},
	{ 1,-1,-1, -1},
	{ 1,-1, 0, -1},
	{ 1,-1, 1, -1},
	{ 1, 0,-1, -1},
	{ 1, 0, 0, -1},
	{ 1, 0, 1, -1},
	{ 1, 1,-1, -1},
	{ 1, 1, 0, -1},
	{ 1, 1, 1, -1}
};


//inspired from ITK's FastChamferDistanceImageFilter
bool ChamferDistanceTransformSigned::propagateDistance(GenericProgressCallback* progressCb/*=0*/)
{
	const float Weights[3] = { 0.92644f, 1.34065f, 1.65849f };

	NormalizedProgress normProgress(progressCb,m_innerSize.y*m_innerSize.z*2);
	if (progressCb)
	{
		progressCb->setMethodTitle("Chamfer distance");
		char buffer[256];
		sprintf(buffer,"Box: [%u x %u x %u]",m_innerSize.x,m_innerSize.y,m_innerSize.z);
		progressCb->setInfo(buffer);
        progressCb->reset();
		progressCb->start();
	}

	/** Precomputing the neighbor types */
	std::vector<char> neighborDist(3*3*3,-1);
	{
		for (size_t i=0; i<neighborDist.size(); ++i)
		{
			const char* pos = Neighbours333[4*i];
			//neighborDist[i] = -1;
			neighborDist[i] += static_cast< char >( pos[0] != 0 );
			neighborDist[i] += static_cast< char >( pos[1] != 0 );
			neighborDist[i] += static_cast< char >( pos[2] != 0 );
		}
	}

	/** 1st Scan , using neighbors from centerNeighborIndex+1 to neighborhoodSize-1 */
	const size_t centerNeighborIndex = neighborDist.size() / 2;
	{
		size_t firstNeighborIndex = centerNeighborIndex + 1;
		size_t lastNeighborIndex  = neighborDist.size() - 1;

		for (size_t k=0; k<m_innerSize.z; ++k)
		{
			for (size_t j=0; j<m_innerSize.y; ++j)
			{
				for (size_t i=0; i<m_innerSize.x; ++i)
				{
					GridElement center_value = getValue(static_cast<int>(i),
														static_cast<int>(j),
														static_cast<int>(k));

					/** Update Positive Distance */
					if ( center_value > -Weights[0] )
					{
						float val[3] = {	center_value + Weights[0],
											center_value + Weights[1],
											center_value + Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[4*n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(neighborDist[n] >= 0 && neighborDist[n] < 3);
							if ( val[neighborDist[n]] < getValue(neighborPos) )
							{
								setValue(neighborPos,val[neighborDist[n]]);
							}
						}
					}
					/** Update Negative Distance */
					else if ( center_value < Weights[0] )
					{
						float val[3] = {	center_value - Weights[0],
											center_value - Weights[1],
											center_value - Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[4*n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(neighborDist[n] >= 0 && neighborDist[n] < 3);
							if ( val[neighborDist[n]] > getValue(neighborPos) )
							{
								setValue(neighborPos,val[neighborDist[n]]);
							}
						}
					}
				}

				if (progressCb && !normProgress.oneStep())
				{
					//process cancelled by the user
					return false;
				}
			}
		}
	}

	/** 2nd Scan , using neighbors from 0 to centerNeighborIndex-1 */
	{
		size_t firstNeighborIndex = 0;
		size_t lastNeighborIndex  = centerNeighborIndex - 1;

		for (size_t k=0; k<m_innerSize.z; ++k)
		{
			for (size_t j=0; j<m_innerSize.y; ++j)
			{
				for (size_t i=0; i<m_innerSize.x; ++i)
				{
					GridElement center_value = getValue(static_cast<int>(i),
														static_cast<int>(j),
														static_cast<int>(k));

					/** Update Positive Distance */
					if ( center_value > -Weights[0] )
					{
						float val[3] = {	center_value + Weights[0],
											center_value + Weights[1],
											center_value + Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[4*n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(neighborDist[n] >= 0 && neighborDist[n] < 3);
							if ( val[neighborDist[n]] < getValue(neighborPos) )
							{
								setValue(neighborPos,val[neighborDist[n]]);
							}
						}
					}
					/** Update Negative Distance */
					else if ( center_value < Weights[0] )
					{
						float val[3] = {	center_value - Weights[0],
											center_value - Weights[1],
											center_value - Weights[2] };

						for (size_t n=firstNeighborIndex; n<=lastNeighborIndex; ++n)
						{
							const char* pos = Neighbours333[4*n];
							Tuple3i neighborPos(static_cast<int>(i) + static_cast<int>(pos[0]),
												static_cast<int>(j) + static_cast<int>(pos[1]),
												static_cast<int>(k) + static_cast<int>(pos[2]) );
						
							assert(neighborDist[n] >= 0 && neighborDist[n] < 3);
							if ( val[neighborDist[n]] > getValue(neighborPos) )
							{
								setValue(neighborPos,val[neighborDist[n]]);
							}
						}
					}
				}

				if (progressCb && !normProgress.oneStep())
				{
					//process cancelled by the user
					return false;
				}
			}
		}
	}

	return true;
}