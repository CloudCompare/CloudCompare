#ifndef __GfxTL_NEARESTNEIGHBORS_HEADER__
#define __GfxTL_NEARESTNEIGHBORS_HEADER__
#include <limits>
#include <GfxTL/LimitedHeap.h>

namespace GfxTL
{
	template< class Scalar, class HandleT = size_t >
	struct NearestNeighbor
	{
		typedef Scalar ScalarType;
		typedef HandleT HandleType;

		NearestNeighbor()
		{}

		NearestNeighbor(HandleType i, ScalarType d)
		: index(i)
		, sqrDist(d)
		{}

		bool operator<(const NearestNeighbor &nn) const
		{
			return sqrDist < nn.sqrDist;
		}

		bool operator>(const NearestNeighbor &nn) const
		{
			return sqrDist > nn.sqrDist;
		}

		bool operator<=(const NearestNeighbor &nn) const
		{
			return sqrDist <= nn.sqrDist;
		}

		bool operator>=(const NearestNeighbor &nn) const
		{
			return sqrDist >= nn.sqrDist;
		}

		operator HandleType &()
		{
			return index;
		}

		operator const HandleType &() const
		{
			return index;
		}

		HandleType index;
		ScalarType sqrDist;
	};

	template< class NN, class NNs >
	void SortedNearestNeighborInsert(const NN &nn, unsigned int k, NNs *n)
	{
		/*typename NNs::iterator low = n->begin(), high = n->end() - 1,
			index = low, mid;
		if(n->size() > 0)
		{
			while(low <= high)
			{
				mid = low + ((high - low) >> 1);
				if(*mid <= nn)
				{
					low = mid + 1;
					index = low;
				}
				else
				{
					index = mid;
					high = mid - 1;
				}
			}
		}
		n->insert(index, nn);*/
		intptr_t low = 0, high = n->size() - 1, index = 0, mid;
		if(n->size() > 0)
		{
			while(low <= high)
			{
				mid = (low + high) >> 1;
				if((*n)[mid] <= nn)
				{
					low = mid + 1;
					index = low;
				}
				else
				{
					index = mid;
					high = mid - 1;
				}
			}
		}
		n->insert(n->begin() + index, nn);
		if(n->size() > k)
		{
			if((*n)[k] > (*n)[k - 1])
				n->resize(k);
		}
/*
		// remove all that are too far away
		typename NNs::iterator begin = n->begin() +
			Math< typename NNs::size_type >::Min(n->size() - 1, k - 1),
			nEnd = n->end();
		while(begin + 1 != nEnd && begin->sqrDist == (begin + 1)->sqrDist)
			++begin;
		if(begin + 1 != nEnd)
			n->erase(begin + 1, nEnd);*/
	}

	template< class NNs, class ScalarType >
	void FindFurthestNearestNeighbor(const NNs &n, size_t size, size_t *worst,
		ScalarType *worstDist)
	{
		size_t j = 0;
		*worst = j;
		*worstDist = n[j].sqrDist;
		++j;
		for(; j < size; ++j)
		{
			if(n[j].sqrDist > *worstDist)
			{
				*worstDist = n[j].sqrDist;
				*worst = j;
			}
		}
	}

	template< class Point, class Points >
	void NearestNeighbors(unsigned int k, const Point &p,
		const Points &points, size_t size,
		std::vector< NearestNeighbor< typename Points::size_type,
			typename Point::ScalarType > > *n)
	{
		typedef Point PointType;
		typedef typename Point::ScalarType ScalarType;
		typedef NearestNeighbor< ScalarType > NN;
		typedef typename Points::size_type size_type;
		n->reserve(k);
		size_type worst;
		ScalarType worstDist = 0;
		size_type i = 0;
		for(; i < size && i < k; ++i)
		{
			PointType diff = points[i] - p;
			n->push_back(NN(i, diff * diff));
			if(n->back().sqrDist > worstDist)
			{
				worstDist = n->back().sqrDist;
				worst = n->size() - 1;
			}
		}

		for(i = k; i < size; ++i)
		{
			PointType diff = points[i] - p;
			ScalarType dist = diff * diff;
			if(dist < worstDist)
			{
				(*n)[worst] = NN(i, dist);
				FindFurthestNearestNeighbor(*n, n->size(), &worst, &worstDist);
			}
		}
	}

	template< class Point, class Points, template< class > class ContainerT,
		class NN >
	void BruteForceNearestNeighbors(unsigned int k, const Point &p,
		const Points &points, size_t size,
		LimitedHeap< NN, std::less<NN>, ContainerT > *neighbors)
	{
		neighbors->clear();
		neighbors->Limit(k);
		for(size_t i = 0; i < size; ++i)
		{
			NN nn(p.SqrDistance(points[i]), i);
			neighbors->PushHeap(nn);
		}
	}

	template< class Point, class Points, template< class > class ContainerT,
		class NN >
	void BruteForceL1NearestNeighbors(unsigned int k, const Point &p,
		const Points &points, size_t size,
		LimitedHeap< NN, std::less<NN>, ContainerT > *neighbors)
	{
		neighbors->clear();
		neighbors->Limit(k);
		for(size_t i = 0; i < size; ++i)
		{
			NN nn(p.L1Distance(points[i]), i);
			neighbors->PushHeap(nn);
		}
	}
};

#endif
