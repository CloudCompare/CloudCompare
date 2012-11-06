
namespace GfxTL
{
	template< class ValueT, class BaseT >
	KdTreeCell< ValueT, BaseT >::KdTreeCell()
	{
		memset(&_children, NULL, sizeof(_children));
	}

	template< class ValueT, class BaseT >
	KdTreeCell< ValueT, BaseT >::KdTreeCell(
		const KdTreeCell< ValueT, BaseT > &cell)
	: BaseT(cell)
	, _split(cell._split)
	, _box(cell._box)
	{
		if(cell._children[0])
			_children[0] = new KdTreeCell< ValueT, BaseT >(*cell._children[0]);
		else
			_children[0] = NULL;
		if(cell._children[1])
			_children[1] = new KdTreeCell< ValueT, BaseT >(*cell._children[1]);
		else
			_children[1] = NULL;
	}

	template< class ValueT, class BaseT >
	KdTreeCell< ValueT, BaseT >::~KdTreeCell()
	{
		for(unsigned int i = 0; i < 2; ++i)
			if(_children[i])
				delete _children[i];
	}

	template< class ValueT, class BaseT >
	const KdTreeCell< ValueT, BaseT > *KdTreeCell< ValueT, BaseT >::operator[](
		unsigned int index) const
	{
		return _children[index];
	}

	template< class ValueT, class BaseT >
	KdTreeCell< ValueT, BaseT > *KdTreeCell< ValueT, BaseT >::operator[](
		unsigned int index)
	{
		return _children[index];
	}

	template< class ValueT, class BaseT >
	void KdTreeCell< ValueT, BaseT >::Child(unsigned int index,
		ThisType *child)
	{
		_children[index] = child;
	}

	template< class ValueT, class BaseT >
	const typename KdTreeCell< ValueT, BaseT >::SplitterType &
		KdTreeCell< ValueT, BaseT >::Split() const
	{
		return _split;
	}

	template< class ValueT, class BaseT >
	const typename KdTreeCell< ValueT, BaseT >::BoxType &
		KdTreeCell< ValueT, BaseT >::Box() const
	{
		return _box;
	}

	template< class Strategies >
	void KdTree< Strategies >::Build()
	{
		Clear();

		CellType *root = new CellType;
		CellRange range;
		RootRange(&range);
		root->_box.Infinite();
		Root(root);
		Init(range, NULL, root);

		typedef std::pair< CellType *, CellRange > Pair;
		std::deque< Pair > stack;
		stack.push_back(Pair(root, range));
		while(stack.size())
		{
			Pair p = stack.back();
			stack.pop_back();
			if(ShouldSubdivide(p.second))
			{
				Subdivide(p.first, p.second);
				if(IsLeaf(*p.first)) // couldn't subdivide?
					continue;
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
				{
					Range(*p.first, p.second, i, &range);
					//Init(range, p.first, (*p.first)[i]);
					stack.push_back(Pair((*p.first)[i], range));
				}
			}
		}
	}

	template< class Strategies >
	void KdTree< Strategies >::Insert(DereferencedType s)
	{
		CellType *cell = Root();
		CellRange range;
		RootRange(&range);
		if(!cell)
		{
			cell = new CellType;
			Root(cell);
			Init(range, NULL, cell);
		}
		Insert(cell, range, s);
	}

	template< class Strategies >
	 template< class ContainerT >
	void KdTree< Strategies >::NearestNeighbors(const PointType &p,
		unsigned int k, ContainerT *neighbors, ScalarType *dist) const
	{
		*dist = std::numeric_limits< ScalarType >::infinity();
		neighbors->resize(0);
		if(!Root())
			return;
		AABox< PointType > rootBox;
		//rootBox.Infinite();
		CellRange range;
		RootRange(&range);
		neighbors->reserve(k + 1);
		NearestNeighbors(*Root(), range, rootBox, p, k, neighbors, dist);
		if(*dist < std::numeric_limits< ScalarType >::infinity())
			*dist = std::sqrt(*dist);
	}

	template< class Strategies >
	 template< template< class > class ContainerT >
	void KdTree< Strategies >::NearestNeighborsL(const PointType &p,
		unsigned int k, LimitedHeap< NN, ContainerT > *neighbors, ScalarType *dist) const
	{
		*dist = std::numeric_limits< ScalarType >::infinity();
		neighbors->resize(0);
		if(!Root())
			return;
		neighbors->Limit(k);
		CellRange range;
		RootRange(&range);
		NearestNeighborsL(*Root(), range, p, k, neighbors, dist);
		//neighbors->sort_heap();
		//std::sort_heap(neighbors->begin(), neighbors->end());
		//if(*dist < std::numeric_limits< ScalarType >::infinity())
		//	*dist = std::sqrt(*dist);
	}

	template< class Strategies >
	 template< class ContainerT >
	void KdTree< Strategies >::ApproximateNearestNeighbors(const PointType &p,
		unsigned int k, ScalarType eps,
		ContainerT *neighbors, ScalarType *sqrDist) const
	{
		*sqrDist = std::numeric_limits< ScalarType >::infinity();
		neighbors->resize(0);
		if(!Root())
			return;
		AABox< PointType > rootBox;
		//rootBox.Infinite();
		CellRange range;
		RootRange(&range);
		neighbors->reserve(k + 1);
		eps = 1 - eps;
		ApproximateNearestNeighbors(*Root(), range, rootBox, p, k, eps * eps,
			neighbors, sqrDist);
		//if(*dist < std::numeric_limits< ScalarType >::infinity())
		//	*dist = std::sqrt(*dist);
	}

	template< class Strategies >
	 template< class ContainerT >
	void KdTree< Strategies >::PointsInSphere(const PointType &p,
		ScalarType sqrRadius, ContainerT *points) const
	{
		points->resize(0);
		if(!Root())
			return;
		CellRange range;
		RootRange(&range);
		PointsInSphere(*Root(), range, p, sqrRadius, points);
	}

	template< class Strategies >
	bool KdTree< Strategies >::Contains(const PointType &p,
		DereferencedType *d) const
	{
		if(!Root())
			return false;
		CellRange range;
		RootRange(&range);
		return Contains(*Root(), range, p, d);
	}

	template< class Strategies >
	bool KdTree< Strategies >::ShouldSubdivide(CellRange range) const
	{
		return range.second - range.first > 10;
	}

	template< class Strategies >
	void KdTree< Strategies >::Subdivide(CellType *cell, CellRange range)
	{
		PointType pmin = cell->_box.Min(), pmax = cell->_box.Max();
		/*pmax = pmin = at(Dereference(range.first));
		for(HandleType i = range.first + 1; i < range.second; ++i)
		{
			const PointType p = at(Dereference(i));
			for(unsigned int j = 0; j < Dim; ++j)
			{
				if(pmin[j] > p[j])
					pmin[j] = p[j];
				else if(pmax[j] < p[j])
					pmax[j] = p[j];
			}
		}*/

		PointType pdiff = pmax - pmin;
		unsigned int axis = 0;
		ScalarType length = pdiff[0];
		for(unsigned int j = 1; j < Dim; ++j)
		{
			if(pdiff[j] > length)
			{
				axis = j;
				length = pdiff[j];
			}
		}

		ScalarType split = (pmax[axis] + pmin[axis]) / 2;

		cell->_split.Set(axis, split);

		CellType *left, *right;
		left = new CellType;
		right = new CellType;
		Split(cell->_split, range, left, right);
		if(left->Size() == cell->Size() || right->Size() == cell->Size())
		{
			delete left;
			delete right;
			return;
		}
		//cell->_box.Split(cell->_split.Axis(), cell->_split.Value(),
		//	&left->_box, &right->_box);
		cell->Child(0, left);
		cell->Child(1, right);
		CellRange childRange;
		Range(*cell, range, 0, &childRange); 
		Init(childRange, cell, left);
		Range(*cell, range, 1, &childRange);
		Init(childRange, cell, right);
		// refine split value
		cell->_split.Value((right->Box().Min()[cell->_split.Axis()]
			+ left->Box().Max()[cell->_split.Axis()]) / 2);
	}

	template< class Strategies >
	void KdTree< Strategies >::Insert(CellType *cell, CellRange range,
		DereferencedType s)
	{
		cell->_box += at(s);
		if(IsLeaf(*cell))
		{
			BaseType::Insert(range, s);
			if(ShouldSubdivide(range))
				Subdivide(cell, range);
			return;
		}
		bool left;
		BaseType::SplitAndInsert(cell->Split(), range, s, (*cell)[0],
			(*cell)[1], &left);
		if(left)
		{
			CellRange lr;
			Range(*cell, range, 0, &lr);
			Insert((*cell)[0], lr, s);
		}
		else
		{
			CellRange rr;
			Range(*cell, range, 1, &rr);
			Insert((*cell)[1], rr, s);
		}
	}

	template< class Strategies >
	 template< class ContainerT >
	void KdTree< Strategies >::NearestNeighbors(const CellType &cell,
		CellRange range, const AABox< PointType > &box, const PointType &p,
		unsigned int k, ContainerT *neighbors, ScalarType *dist2) const
	{
		if(IsLeaf(cell))
		{
			HandleType i = range.first, iend = range.second;
			if(neighbors->size() + cell.Size() <= k)
			{
				for(; i != iend; ++i)
				{
					DereferencedType deref = Dereference(i);
					PointType diff = at(deref);
					diff -= p;
					ScalarType d2 = diff * diff;
					neighbors->push_back(NN(deref, d2));
				}
				std::sort(neighbors->begin(), neighbors->end());
				*dist2 = neighbors->back().sqrDist;
				return;
			}
			// let's find nearest neighbors from points within the cell
			for(; i != iend; ++i)
			{
				DereferencedType deref = Dereference(i);
				PointType diff = at(deref);
				diff -= p;
				ScalarType d2 = diff * diff;
				if(neighbors->size() < k ||	d2 <= *dist2)
				{
					SortedNearestNeighborInsert(NN(deref, d2), k, neighbors);
					*dist2 = neighbors->back().sqrDist;
				}
			}
			return;
		}
		// descent into subtree containing p
		unsigned int nearerChild, fartherChild;
		if(cell.Split()(p) <= 0)
		{
			nearerChild = 0;
			fartherChild = 1;
		}
		else
		{
			nearerChild = 1;
			fartherChild = 0;
		}
		/*AABox< PointType > subBoxes[2];
		box.Split(cell.Split().Axis(), cell.Split().Value(),
			&subBoxes[0], &subBoxes[1]);*/

		CellRange childRange;
		if(neighbors->size() < k || Intersect(cell[nearerChild]->Box(),
			p, *dist2))
		{
			Range(cell, range, nearerChild, &childRange);
			NearestNeighbors(*cell[nearerChild], childRange,
				box/*subBoxes[nearerChild]*/, p, k, neighbors, dist2);
		}

		// check if closer neighbor could be in other child
		if(neighbors->size() < k || Intersect(cell[fartherChild]->Box()/*subBoxes[fartherChild]*/,
			p, *dist2))
		{
			Range(cell, range, fartherChild, &childRange);
			NearestNeighbors(*cell[fartherChild], childRange,
				box/*subBoxes[fartherChild]*/, p, k, neighbors, dist2);
		}
	}

	template< class Strategies >
	 template< template< class > class ContainerT >
	void KdTree< Strategies >::NearestNeighborsL(const CellType &cell,
		const CellRange &range, const PointType &p, unsigned int k,
		LimitedHeap< NN, ContainerT > *neighbors, ScalarType *dist2) const
	{
		if(IsLeaf(cell))
		{
			HandleType i = range.first, iend = range.second;
			DereferencedType deref;
			if(neighbors->size() + cell.Size() <= k)
			{
				for(; i != iend; ++i)
				{
					deref = Dereference(i);
					neighbors->push_back(NN(deref,
						p.SqrDistance(at(deref).Point())));
				}
				neighbors->MakeHeap();
				*dist2 = neighbors->front().sqrDist;
				return;
			}
			// let's find nearest neighbors from points within the cell
			for(; i != iend; ++i)
			{
				deref = Dereference(i);
				neighbors->PushHeap(NN(deref,
					p.SqrDistance(at(deref).Point())));
				*dist2 = neighbors->front().sqrDist;
			}
			return;
		}
		// descent into subtree containing p
		unsigned int nearerChild, fartherChild;
		const CellType *nChild, *fChild;
		ScalarType splitDist = cell._split(p);
		if(splitDist <= 0)
		{
			nearerChild = 0;
			fartherChild = 1;
			nChild = cell[0];
			fChild = cell[1];
		}
		else
		{
			nearerChild = 1;
			fartherChild = 0;
			nChild = cell[1];
			fChild = cell[0];
		}

		CellRange childRange;
		//if(neighbors->size() < k || Intersect(p, *dist2, nChild->_box.Min(),
		//	nChild->_box.Max()))
		//{
			Range(cell, range, nearerChild, &childRange);
			NearestNeighborsL(*nChild, childRange, p, k, neighbors,
				dist2);
		//}

		// check if closer neighbor could be in other child
        if(neighbors->size() < k || (splitDist * splitDist < *dist2 &&
			Intersect(p, *dist2, fChild->_box.Min(), fChild->_box.Max())))
		{
			Range(cell, range, fartherChild, &childRange);
			NearestNeighborsL(*fChild, childRange, p, k, neighbors,
				dist2);
		}
	}

	template< class Strategies >
	 template< class ContainerT >
	void KdTree< Strategies >::ApproximateNearestNeighbors(
		const CellType &cell, CellRange range, const AABox< PointType > &box,
		const PointType &p, unsigned int k, ScalarType eps,
		ContainerT *neighbors, ScalarType *sqrDist) const
	{
		if(IsLeaf(cell))
		{
			if(neighbors->size() + cell.Size() <= k)
			{
				for(HandleType i = range.first; i != range.second; ++i)
				{
					DereferencedType deref = Dereference(i);
					PointType diff = at(deref);
					diff -= p;
					ScalarType d2 = diff * diff;
					neighbors->push_back(NN(deref, d2));
				}
				std::sort(neighbors->begin(), neighbors->end());
				*sqrDist = neighbors->back().sqrDist;
				return;
			}
			// let's find nearest neighbors from points within the cell
			for(HandleType i = range.first; i != range.second; ++i)
			{
				DereferencedType deref = Dereference(i);
				PointType diff = at(deref);
				diff -= p;
				ScalarType d2 = diff * diff;
				if(neighbors->size() < k ||	d2 <= *sqrDist)
				{
					SortedNearestNeighborInsert(NN(deref, d2), k, neighbors);
					*sqrDist = neighbors->back().sqrDist;
				}
			}
			return;
		}
		// descent into subtree containing p
		unsigned int nearerChild, fartherChild;
		if(cell.Split()(p) <= 0)
		{
			nearerChild = 0;
			fartherChild = 1;
		}
		else
		{
			nearerChild = 1;
			fartherChild = 0;
		}
		/*AABox< PointType > subBoxes[2];
		box.Split(cell.Split().Axis(), cell.Split().Value(),
			&subBoxes[0], &subBoxes[1]);*/

		CellRange childRange;
		if(neighbors->size() < k || Intersect(cell[nearerChild]->Box(),
			p, eps * (*sqrDist)))
		{
			Range(cell, range, nearerChild, &childRange);
			ApproximateNearestNeighbors(*cell[nearerChild], childRange,
				box/*subBoxes[nearerChild]*/, p, k, eps, neighbors, sqrDist);
		}

		// check if closer neighbor could be in other child
		if(neighbors->size() < k || Intersect(cell[fartherChild]->Box()/*subBoxes[fartherChild]*/,
			p, eps * (*sqrDist)))
		{
			Range(cell, range, fartherChild, &childRange);
			ApproximateNearestNeighbors(*cell[fartherChild], childRange,
				box/*subBoxes[fartherChild]*/, p, k, eps, neighbors, sqrDist);
		}
	}

	template< class Strategies >
	 template< class ContainerT >
	void KdTree< Strategies >::PointsInSphere(const CellType &cell,
		const CellRange &range, const PointType &p, ScalarType sqrRadius,
		ContainerT *points) const
	{
		if(IsLeaf(cell))
		{
			for(HandleType i = range.first; i < range.second; ++i)
			{
				DereferencedType deref = Dereference(i);
				ScalarType sqrDist = p.SqrDistance(at(deref));
				if(sqrDist <= sqrRadius)
					points->push_back(NN(deref, sqrDist));
			}
		}
		else
		{
			CellRange childRange;
			if(Intersect(p, sqrRadius, cell[0]->_box.Min(),
				cell[0]->_box.Max()))
			{
				Range(cell, range, 0, &childRange);
				PointsInSphere(*(cell[0]), childRange, p, sqrRadius, points);
			}
			if(Intersect(p, sqrRadius, cell[1]->_box.Min(),
				cell[1]->_box.Max()))
			{
				Range(cell, range, 1, &childRange);
				PointsInSphere(*(cell[1]), childRange, p, sqrRadius, points);
			}
		}
	}

	template< class Strategies >
	bool KdTree< Strategies >::Contains(const CellType &cell,
		const CellRange range, const PointType &p, DereferencedType *d) const
	{
		if(IsLeaf(cell))
		{
			for(HandleType i = range.first; i < range.second; ++i)
			{
				DereferencedType deref = Dereference(i);
				if(p == at(deref))
				{
					*d = deref;
					return true;
				}
			}
			return false;
		}
		// descent into subtree containing p
		unsigned int nearerChild = cell.Split()(p) <= 0? 0 : 1;
		CellRange childRange;
		Range(cell, range, nearerChild, &childRange);
		return Contains(*cell[nearerChild], childRange, p, d);
	}

	template< class Strategies >
	void KdTree< Strategies >::Init(const CellRange &range, const CellType *,
		CellType *cell)
	{
		PointType pmin, pmax;
		pmax = pmin = at(Dereference(range.first));
		for(HandleType i = range.first + 1; i != range.second; ++i)
		{
			const PointType p = at(Dereference(i));
			for(unsigned int j = 0; j < Dim; ++j)
			{
				if(pmin[j] > p[j])
					pmin[j] = p[j];
				else if(pmax[j] < p[j])
					pmax[j] = p[j];
			}
		}
		cell->_box.Min() = pmin;
		cell->_box.Max() = pmax;
	}
};
