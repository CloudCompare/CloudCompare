
namespace GfxTL
{
	template< class Cell >
	BaseTree< Cell >::BaseTree()
	: _root(NULL)
	{}

	template< class Cell >
	BaseTree< Cell >::BaseTree(const BaseTree< Cell > &bt)
	: _root(NULL)
	{
		if(bt._root)
			_root = new Cell(*bt._root);
	}

	template< class Cell >
	BaseTree< Cell >::~BaseTree()
	{
		Clear();
	}

	template< class Cell >
	void BaseTree< Cell >::Clear()
	{
		if(_root)
		{
			delete _root;
			_root = NULL;
		}
	}

	template< class Cell >
	void BaseTree< Cell >::Init()
	{}

	template< class Cell >
	typename BaseTree< Cell >::CellType *
		BaseTree< Cell >::Root()
	{
		return _root;
	}

	template< class Cell >
	const typename BaseTree< Cell >::CellType *
		BaseTree< Cell >::Root() const
	{
		return _root;
	}

	template< class Cell >
	void BaseTree< Cell >::Root(Cell *root)
	{
		_root = root;
	}

	template< class Cell >
	bool BaseTree< Cell >::IsLeaf(const CellType *cell) const
	{
		return (*cell)[0] == NULL;
	}

	template< class Cell >
	BaseTree< Cell > &BaseTree< Cell >::operator=(const BaseTree< Cell > &bt)
	{
		Clear();
		if(bt._root)
			_root = new Cell(*bt._root);
		return *this;
	}
};