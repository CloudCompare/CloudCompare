
namespace GfxTL
{
	//-- AAKdCell

	template< class Point, class Base >
	AAKdCell< Point, Base >::AAKdCell()
	{
		memset(_children, 0, sizeof(_children));
	};

	template< class Point, class Base >
	AAKdCell< Point, Base >::AAKdCell(const AAKdCell &cell)
	: _axis(cell._axis)
	, _splitValue(cell._splitValue)
	{
		if(cell._children[0])
			_children[0] = new AAKdCell< Point, Base >(*cell._children[0]);
		else
			_children[0] = NULL;
		if(cell._children[1])
			_children[1] = new AAKdCell< Point, Base >(*cell._children[1]);
		else
			_children[1] = NULL;
	}

	template< class Point, class Base >
	AAKdCell< Point, Base >::~AAKdCell()
	{
		for(unsigned int i = 0; i < 2; ++i)
			if(_children[i])
				delete _children[i];
	}

	template< class Point, class Base >
	const AAKdCell< Point, Base > *
		AAKdCell< Point, Base >::operator[](unsigned int index) const
	{
		return _children[index];
	}

	template< class Point, class Base >
	AAKdCell< Point, Base > *
		AAKdCell< Point, Base >::operator[](unsigned int index)
	{
		return _children[index];
	}

	template< class Point, class Base >
	void AAKdCell< Point, Base >::Child(unsigned int index, ThisType *child)
	{
		_children[index] = child;
	}

	template< class Point, class Base >
	typename AAKdCell< Point, Base >::ScalarType
		AAKdCell< Point, Base >::Split() const
	{
		return _splitValue;
	}

	template< class Point, class Base >
	void AAKdCell< Point, Base >::Split(ScalarType split)
	{
		_splitValue = split;
	}

	template< class Point, class Base >
	unsigned int AAKdCell< Point, Base >::Axis() const
	{
		return _axis;
	}

	template< class Point, class Base >
	void AAKdCell< Point, Base >::Axis(unsigned int axis)
	{
		_axis = axis;
	}

	//-- AAKdTree

	template< class Strategies >
	void AAKdTree< Strategies >::Build()
	{
		Clear();

		CellType *root = new CellType;
		RootCellData(AACube< PointType >(), root); // implemented by TreeData
		Root(root);

		std::list< CellType * > stack;
		stack.push_back(root);
		while(stack.size())
		{
			CellType *c = stack.back();
			stack.pop_back();
			if(ShouldSubdivide(c))
			{
				Subdivide(c);
				if(IsLeaf(*c)) // couldn't subdivide?
					continue;
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					stack.push_back((*c)[i]);
			}
		}
	};

	template< class Strategies >
	void AAKdTree< Strategies >::PointsInSphere(const PointType &center,
		ScalarType radius, std::vector< size_t > *points) const
	{
		points->clear();
		if(!Root())
			return;
		PointsInSphere(Root(), center, radius, points);
	};

	template< class Strategies >
	void AAKdTree< Strategies >::
		PointsInAACube(const AACube< PointType	> &cube,
			std::vector< size_t > *points) const
	{
		points->clear();
		if(!Root())
			return;
		PointType center;
		cube.Center(&center);
		PointsInAACube(Root(), center, cube.Width() / 2, points);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::PointsInAACube(const PointType &center,
		ScalarType width, std::vector< size_t > *points) const
	{
		points->clear();
		if(!Root())
			return;
		PointsInAACube(Root(), center, width / 2, points);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::
		RefreshWithNewTreeData(const AACube< PointType > &bc)
	{
		// iterate over all cells and readjust cell data
		CellType *c = Root();
		if(!c)
		{
			c = new CellType;
			Root(c);
		}
		RootCellData(AACube< PointType >(), c);
		std::list< CellType * > stack;
		stack.push_back(c);
		while(stack.size())
		{
			c = stack.back();
			stack.pop_back();
			if(!IsLeaf(c))
			{
				if(!ShouldSubdivide(c))
				{
					for(unsigned int i = 0; i < CellType::NChildren; ++i)
					{
						delete (*c)[i];
						c->Child(i, NULL);
					}
				}
				else
				{
					ReadjustData(c);
					for(unsigned int i = 0; i < CellType::NChildren; ++i)
						stack.push_back((*c)[i]);
				}
			}
			else if(ShouldSubdivide(c))
			{
				Subdivide(c);
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					stack.push_back((*c)[i]);
			}
		}
		StrategyBaseType::RefreshWithNewTreeData(bc);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::NearestNeighbor(const PointType &p,
		size_t *neighbor, ScalarType *dist) const
	{
		*dist = std::numeric_limits< ScalarType >::infinity();
		if(!Root())
			return;
		AABox< PointType > rootBox;
		rootBox.Infinite();
		NearestNeighbor(*Root(), rootBox, p, neighbor, dist);
		if(*dist < std::numeric_limits< ScalarType >::infinity())
			*dist = std::sqrt(*dist);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::KNearestNeighbors(const PointType &p,
		unsigned int k,	std::vector< NN > *neighbors,
		ScalarType *dist) const
	{
		*dist = std::numeric_limits< ScalarType >::infinity();
		if(!Root())
			return;
		size_t worstIdx = 0;
		AABox< PointType > rootBox;
		rootBox.Infinite();
		neighbors->clear();
		neighbors->reserve(k + 1);
		KNearestNeighbors(*Root(), rootBox, p, k, neighbors, &worstIdx, dist);
		if(*dist < std::numeric_limits< ScalarType >::infinity())
			*dist = std::sqrt(*dist);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::NearestNeighbor(const CellType &cell,
		const AABox< PointType > &box, const PointType &p,
		size_t *neighbor, ScalarType *dist2) const
	{
		if(IsLeaf(cell))
		{
			// let's find nearest neighbor from points within the cell
			for(size_t i = 0; i < cell.Size(); ++i)
			{
				size_t idx = Translate(cell.Points() + i);
				PointType diff = PointTranslated(idx) - p;
				ScalarType d2 = diff * diff;
				if(d2 < *dist2)
				{
					*dist2 = d2;
					*neighbor = idx;
				}
			}
			return;
		}
		// descent into subtree containing p
		unsigned int nearerChild, fartherChild;
		if(p[cell.Axis()] <= cell.Split())
		{
			nearerChild = 0;
			fartherChild = 1;
		}
		else
		{
			nearerChild = 1;
			fartherChild = 0;
		}
		AABox< PointType > subBoxes[2];
		box.Split(cell.Axis(), cell.Split(), &subBoxes[0], &subBoxes[1]);
		NearestNeighbor(*cell[nearerChild], subBoxes[nearerChild],
			p, neighbor, dist2);

		// check if closer neighbor could be in other child
		if(Intersect(subBoxes[fartherChild], p, *dist2))
			NearestNeighbor(*cell[fartherChild], subBoxes[fartherChild],
				p, neighbor, dist2);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::KNearestNeighbors(const CellType &cell,
		const AABox< PointType > &box, const PointType &p, unsigned int k,
		std::vector< NN > *neighbors, size_t *worstIdx,
		ScalarType *dist2) const
	{
		if(IsLeaf(cell))
		{
			// let's find nearest neighbors from points within the cell
			for(size_t i = 0; i < cell.Size(); ++i)
			{
				size_t idx = Translate(cell.Points() + i);
				PointType diff = PointTranslated(idx) - p;
				ScalarType d2 = diff * diff;
				if(neighbors->size() < k)
				{
					neighbors->push_back(NN(idx, d2));
					FindFurthestNearestNeighbor(*neighbors, neighbors->size(),
						worstIdx, dist2);
				}
				else if(d2 < *dist2)
				{
					(*neighbors)[*worstIdx] = NN(idx, d2);
					FindFurthestNearestNeighbor(*neighbors, neighbors->size(),
						worstIdx, dist2);
				}
			}
			return;
		}
		// descent into subtree containing p
		unsigned int nearerChild, fartherChild;
		if(p[cell.Axis()] <= cell.Split())
		{
			nearerChild = 0;
			fartherChild = 1;
		}
		else
		{
			nearerChild = 1;
			fartherChild = 0;
		}
		AABox< PointType > subBoxes[2];
		box.Split(cell.Axis(), cell.Split(), &subBoxes[0], &subBoxes[1]);

		KNearestNeighbors(*cell[nearerChild], subBoxes[nearerChild], p, k,
			neighbors, worstIdx, dist2);

		// check if closer neighbor could be in other child
		if(neighbors->size() < k || Intersect(subBoxes[fartherChild],
			p, *dist2))
			KNearestNeighbors(*cell[fartherChild], subBoxes[fartherChild],
				p, k, neighbors, worstIdx, dist2);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::ReadjustData(CellType *cell)
	{
		if(!cell->Size())
			return;
		PointType pmin, pmax;
		pmax = pmin = PointTranslated(Translate(cell->Points()));
		for(size_t i = 1; i < cell->Size(); ++i)
		{
			const PointType &p =
				PointTranslated(Translate(cell->Points() + i));
			for(unsigned int j = 0; j < Dim; ++j)
			{
				if(pmin[j] > p[j])
					pmin[j] = p[j];
				else if(pmax[j] < p[j])
					pmax[j] = p[j];
			}
		}

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

		CellType *left = (*cell)[0],
			*right = (*cell)[1];
		SplitAlongAxis(*cell, axis, split, left, right); // implemented by TreeData
		cell->Split(split);
		cell->Axis(axis);
	}

	template< class Strategies >
	void AAKdTree< Strategies >::PointsInSphere(const CellType *cell,
		const PointType &center, ScalarType radius,
		std::vector< size_t > *points) const
	{
		if(IsLeaf(cell))
		{
			ScalarType radius2 = radius * radius;
			for(size_t i = 0; i < cell->Size(); ++i)
			{
				size_t ind = Translate(cell->Points() + i);
				const PointType &p = PointTranslated(ind);
				PointType d = p - center;
				if((d * d) <= radius2)
					points->push_back(ind);
			}
		}
		else
		{
			ScalarType d = center[cell->Axis()] - cell->Split();
			if(d + radius >= 0)
				PointsInSphere((*cell)[1], center, radius, points);
			if(d - radius <= 0)
				PointsInSphere((*cell)[0], center, radius, points);
		}
	}

	template< class Strategies >
	void AAKdTree< Strategies >::PointsInAACube(const CellType *cell,
		const PointType &center, ScalarType radius,
		std::vector< HandleType > *points) const
	{
		if(IsLeaf(cell))
		{
			for(size_t i = 0; i < cell->Size(); ++i)
			{
				size_t ind = Translate(cell->Points() + i);
				const PointType &p = PointTranslated(ind);
				PointType d = p - center;
				bool isInside = true;
				for(unsigned int j = 0; j < PointType::Dim; ++j)
				{
					if(Absolute(d[j]) > radius)
					{
						isInside = false;
						break;
					}
				}
				if(isInside)
					points->push_back(ind);
			}
		}
		else
		{
			ScalarType d = center[cell->Axis()] - cell->Split();
			if(d + radius >= 0)
				PointsInAACube((*cell)[1], center, radius, points);
			if(d - radius <= 0)
				PointsInAACube((*cell)[0], center, radius, points);
		}
	}

	template< class Strategies >
	bool AAKdTree< Strategies >::ShouldSubdivide(const CellType *cell) const
	{
		if(cell->Size() > 10)
			return true;
		return false;
	}

	template< class Strategies >
	void AAKdTree< Strategies >::Subdivide(CellType *cell)
	{
		PointType pmin, pmax;
		pmax = pmin = PointTranslated(Translate(cell->Points()));
		for(size_t i = 1; i < cell->Size(); ++i)
		{
			const PointType &p =
				PointTranslated(Translate(cell->Points() + i));
			for(unsigned int j = 0; j < Dim; ++j)
			{
				if(pmin[j] > p[j])
					pmin[j] = p[j];
				else if(pmax[j] < p[j])
					pmax[j] = p[j];
			}
		}

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

		CellType *left, *right;
		left = new CellType;
		right = new CellType;
		SplitAlongAxis(*cell, axis, split, left, right); // implemented by TreeData

		if(left->Size() == cell->Size() || right->Size() == cell->Size())
		{
			delete left;
			delete right;
			return;
		}
		cell->Split(split);
		cell->Axis(axis);
		cell->Child(0, left);
		cell->Child(1, right);
	}

	template< class Strategies >
	typename AAKdTree< Strategies >::ScalarType
		AAKdTree< Strategies >::Absolute(ScalarType f) const
	{
		if(f < 0)
			return -f;
		return f;
	}

};