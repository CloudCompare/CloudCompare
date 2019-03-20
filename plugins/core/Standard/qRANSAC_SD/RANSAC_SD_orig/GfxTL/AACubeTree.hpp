namespace GfxTL
{

	//-- AACubeCell

	template< class Point, class Base >
	AACubeCell< Point, Base >::AACubeCell()
	: _parent(NULL)
	{
		memset(_children, 0, sizeof(_children));
	}

	template< class Point, class Base >
	AACubeCell< Point, Base >::AACubeCell(ThisType *parent,
		const CubeType &cube)
	: _parent(parent)
	, _cube(cube)
	{
		memset(_children, 0, sizeof(_children));
	}

	template< class Point, class Base >
	AACubeCell< Point, Base >::~AACubeCell()
	{
		for(int i = 0; i < NChildren; ++i)
		{
			if(_children[i])
				delete _children[i];
		}
	}

	template< class Point, class Base >
	const AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::operator[](unsigned int index) const
	{
		return _children[index];
	}

	template< class Point, class Base >
	AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::operator[](unsigned int index)
	{
		return _children[index];
	}

	template< class Point, class Base >
	void AACubeCell< Point, Base >::Child(unsigned int index, ThisType *child)
	{
		_children[index] = child;
	}

	template< class Point, class Base >
	typename AACubeCell< Point, Base >::CubeType &
		AACubeCell< Point, Base >::Cube()
	{
		return _cube;
	}

	template< class Point, class Base >
	const typename AACubeCell< Point, Base >::CubeType &
		AACubeCell< Point, Base >::Cube() const
	{
		return _cube;
	}

	template< class Point, class Base >
	AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::Parent()
	{
		return _parent;
	}

	template< class Point, class Base >
	const AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::Parent() const
	{
		return _parent;
	}

	template< class Point, class Base >
	void AACubeCell< Point, Base >::Parent(ThisType *parent)
	{
		_parent = parent;
	}

	template< class Point, class Base >
	AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::FaceNeighborIndexed(unsigned int index,
			unsigned int *level)
	{
		int axis = index >> 1;
		++axis;
		if(index & 1)
			axis = -axis;
		return FaceNeighbor(axis, level);
	}

	template< class Point, class Base >
	const AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::FaceNeighborIndexed(unsigned int index,
			unsigned int *level) const
	{
		int axis = index >> 1;
		++axis;
		if(index & 1)
			axis = -axis;
		return FaceNeighbor(axis, level);
	}

	template< class Point, class Base >
	AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::FaceNeighbor(int axis, unsigned int *level)
	{
		int box = SubBox();
		if(box < 0)
			return NULL;
		int a = axis;
		if(a > 0)
		{
			a -= 1;
			if(box & (1 << a))
				return (*_parent)[box & ~(1 << a)];
		}
		if(a < 0)
		{
			a = (-a) - 1;
			if(!(box & (1 << a)))
				return (*_parent)[box | (1 << a)];
		}
		unsigned int l = *level;
		++(*level);
		ThisType *c = _parent->FaceNeighbor(axis, level);
		if(!c)
			return NULL;
		int invBox = box;
		if(invBox & (1 << a))
			invBox &= ~(1 << a);
		else
			invBox |= 1 << a;
		if(*level == l + 1 && (*c)[invBox])
		{
			--(*level);
			c = (*c)[invBox];
		}
		return c;
	}

	template< class Point, class Base >
	const AACubeCell< Point, Base > *
		AACubeCell< Point, Base >::
			FaceNeighbor(int axis, unsigned int *level) const
	{
		return const_cast< ThisType * >(this)->FaceNeighbor(axis, level);
	}

	template< class Point, class Base >
	int AACubeCell< Point, Base >::SubBox() const
	{
		unsigned int box;
		if(!_parent)
			return -1;
		if(!(_parent->Cube().IsSubCube(&box, Cube())))
		{
			assert(false);
			return -1;
		}
		return (int)box;
	}

	//-- AACubeTree

	template< class Strategies >
	void AACubeTree< Strategies >::Clear()
	{
		StrategyBaseType::Clear();
		_cellCount = 0;
	}

	template< class Strategies >
	void AACubeTree< Strategies >::Build()
	{
		AABox< PointType > bbox;
		BoundingVolume(&bbox);
		PointType center;
		bbox.Center(&center);
		ScalarType w = (bbox.Max() - bbox.Min()).Length();
		PointType bbl;
		for(unsigned int i = 0; i < PointType::Dim; ++i)
			bbl[i] = center[i] - (w / 2);
		CubeType bc(bbl, w);
		/*CubeType bc;
		BoundingCube(&bc);*/
		CubeType cc(bc[CubeType::NCorners - 1] * (ScalarType)1.2,
			bc.Width() * (ScalarType)1.2);
		Build(cc);
	}

	template< class Strategies >
	void AACubeTree< Strategies >::Build(const CubeType &bc)
	{
		Clear();

		std::queue< CellType * > *q;
		std::queue< CellType * > *qq;
		std::queue< CellType * > q1, q2, level;

		CellType *c = new CellType(NULL, bc);
		RootCellData(bc, c); // implemented by TreeData
		q1.push(c);
		Root(c); // implemented by BaseTree
		InitCellData(c); // implemented by Strategies
		q = &q1;
		qq = &q2;

		do
		{
			while(q->size())
			{
				c = q->front();
				level.push(c);
				q->pop();
				//InitCellData(c); // implemented by Strategies
				if(ShouldSubdivide(*c)) // implemented by Strategies
				{
					Subdivide(c);
					for(int i = 0; i < CellType::NChildren; ++i)
						qq->push((*c)[i]);
					_cellCount += CellType::NChildren;
				}
			}
			std::queue< CellType * > *qqq = q;
			q = qq;
			qq = qqq;

			while(level.size())
			{
				c = level.front();
				level.pop();
				InitLevelDependentCellData(c); // implemented by Strategies
			}
		}
		while(q->size());
	}

	template< class Strategies >
	void AACubeTree< Strategies >::Subdivide(CellType *cell)
	{
		Subdivide(cell, 0, 0, cell);
		for(unsigned int i = 0; i < CellType::NChildren; ++i)
			InitCellData((*cell)[i]); // implemented by Strategies
	}

	template< class Strategies >
	void AACubeTree< Strategies >::RefreshWithNewTreeData(const CubeType &bc)
	{
		// iterate over all cells and readjust cell data
		CellType *c = Root();
		if(!c)
			return;
		RootCellData(c->Cube(), c);
		std::list< CellType * > stack;
		stack.push_back(c);
		while(stack.size())
		{
			c = stack.back();
			stack.pop_back();
			if(!IsLeaf(c))
			{
				ReadjustChildrenData(c, 0, 0, c);
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					stack.push_back((*c)[i]);
			}
		}
		StrategyBaseType::RefreshWithNewTreeData(bc);
	}

	template< class Strategies >
	void AACubeTree< Strategies >::ReadjustChildrenData(CellType *cell,
		unsigned int axis, unsigned int box, CellType *data)
	{
		if(axis == Dim)
		{
			(*cell)[box]->Data(*data);
		}
		else
		{
			ScalarType s;
			cell->Cube().DividingPlane(axis, &s);
			CellType left, right;
			SplitAlongAxis(*data, axis, s, &left, &right); // implemented by TreeData

			ReadjustChildrenData(cell, axis + 1, box | (1 << axis), &left);
			ReadjustChildrenData(cell, axis + 1, box & (~(1 << axis)), &right);
		}
	}

	template< class Strategies >
	void AACubeTree< Strategies >::Subdivide(CellType *cell, unsigned int axis,
		unsigned int box, CellType *data)
	{
		if(axis == Dim)
		{
			CellType *child = new CellType;
			child->Data(*data);
			child->Parent(cell);
			child->Cube() = CubeType(box, cell->Cube());
			cell->Child(box, child);
		}
		else
		{
			ScalarType s;
			cell->Cube().DividingPlane(axis, &s);
			CellType left, right;
			SplitAlongAxis(*data, axis, s, &left, &right); // implemented by TreeData

			Subdivide(cell, axis + 1, box | (1 << axis), &left);
			Subdivide(cell, axis + 1, box & (~(1 << axis)), &right);
		}
	}

};