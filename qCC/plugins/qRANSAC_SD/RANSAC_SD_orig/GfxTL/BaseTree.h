#ifndef GfxTL_BASETREE_HEADER__
#define GfxTL_BASETREE_HEADER__
#include <vector>
#include <utility>

namespace GfxTL
{
	template< class Cell >
	class BaseTree
	{
		public:
			typedef Cell CellType;

			BaseTree();
			BaseTree(const BaseTree< Cell > &bt);
			virtual ~BaseTree();
			virtual void Clear();
			virtual void Init();
			CellType *&Root() { return m_root; }
			const CellType *Root() const { return m_root; }
			inline bool IsLeaf(const CellType &cell) const;
			inline bool ExistChild(const CellType &cell, unsigned int i) const;
			BaseTree< Cell > &operator=(const BaseTree< Cell > &bt);
			size_t NumberOfLeaves() const;
			size_t NumberOfNodesOnLevel(size_t level) const;
			size_t MaxDepth() const;
			double AvgDepth() const;
			template< class ScalarT >
			void LeafDepthVariance(size_t *numLeaves, ScalarT *variance) const;

		protected:
			CellType *InnerNodeMarker() const { return (CellType *)1; }

		private:
			CellType *m_root;
	};

	template< class Cell >
	inline bool BaseTree< Cell >::IsLeaf(const CellType &cell) const
	{
		return &(cell[0]) == NULL;
	}

	template< class Cell >
	inline bool BaseTree< Cell >::ExistChild(const CellType &cell,
		unsigned int i) const
	{
		return &(cell[i]) > (CellType *)1;
	}

	template< class Cell >
	BaseTree< Cell >::BaseTree()
	: m_root(NULL)
	{}

	template< class Cell >
	BaseTree< Cell >::BaseTree(const BaseTree< Cell > &bt)
	: m_root(NULL)
	{
		if(bt.m_root)
			m_root = new Cell(*bt.m_root);
	}

	template< class Cell >
	BaseTree< Cell >::~BaseTree()
	{
		Clear();
	}

	template< class Cell >
	void BaseTree< Cell >::Clear()
	{
		if(m_root)
		{
			delete m_root;
			m_root = NULL;
		}
	}

	template< class Cell >
	void BaseTree< Cell >::Init()
	{}

	template< class Cell >
	BaseTree< Cell > &BaseTree< Cell >::operator=(const BaseTree< Cell > &bt)
	{
		Clear();
		if(bt.m_root)
			m_root = new Cell(*bt.m_root);
		return *this;
	}

	template< class Cell >
	size_t BaseTree< Cell >::NumberOfLeaves() const
	{
		if(!Root())
			return 0;
		size_t numLeaves = 0;
		std::vector< const CellType * > stack;
		stack.push_back(Root());
		while(stack.size())
		{
			const CellType *c = stack.back();
			stack.pop_back();
			if(IsLeaf(*c))
				++numLeaves;
			else
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					if(ExistChild(*c, i))
						stack.push_back(&((*c)[i]));
		}
		return numLeaves;
	}

	template< class Cell >
	size_t BaseTree< Cell >::NumberOfNodesOnLevel(size_t level) const
	{
		if(!Root())
			return 0;
		typedef std::pair< const CellType *, size_t > Pair;
		size_t numNodes = 0;
		std::vector< Pair > stack;
		stack.push_back(Pair(Root(), 0));
		while(stack.size())
		{
			Pair p = stack.back();
			stack.pop_back();
			if(p.second == level)
			{
				++numNodes;
				continue;
			}
			else if(IsLeaf(*p.first))
				continue;
			else
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					if(ExistChild(*p.first, i))
						stack.push_back(Pair(&((*p.first)[i]), p.second + 1));
		}
		return numNodes;
	}

	template< class Cell >
	size_t BaseTree< Cell >::MaxDepth() const
	{
		size_t maxLevel = 0;
		if(!Root())
			return maxLevel;
		typedef std::pair< const CellType *, size_t > Pair;
		std::vector< Pair > stack;
		stack.push_back(Pair(Root(), 0));
		while(stack.size())
		{
			Pair p = stack.back();
			stack.pop_back();
			if(p.second > maxLevel)
				maxLevel = p.second;
			if(IsLeaf(*p.first))
				continue;
			else
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					if(ExistChild(*p.first, i))
						stack.push_back(Pair(&((*p.first)[i]), p.second + 1));
		}
		return maxLevel;
	}

	template< class Cell >
	double BaseTree< Cell >::AvgDepth() const
	{
		if(!Root())
			return 0.0;
		size_t levelSum = 0;
		size_t leaveCount = 0;
		typedef std::pair< const CellType *, size_t > Pair;
		std::vector< Pair > stack;
		stack.push_back(Pair(Root(), 0));
		while(stack.size())
		{
			Pair p = stack.back();
			stack.pop_back();
			if(IsLeaf(*p.first))
			{
				levelSum += p.second;
				++leaveCount;
				continue;
			}
			else
				for(unsigned int i = 0; i < CellType::NChildren; ++i)
					if(ExistChild(*p.first, i))
						stack.push_back(Pair(&((*p.first)[i]), p.second + 1));
		}
		return double(levelSum) / double(leaveCount);
	}

	template< class Cell >
	 template< class ScalarT >
	void BaseTree< Cell >::LeafDepthVariance(size_t *numLeaves, ScalarT *variance) const
	{
		typedef ScalarT ScalarType;
		*numLeaves = 0;
		*variance = 0;
		if(!Root())
			return;
		ScalarType avgDepth = 0;
		std::vector< std::pair< const CellType *, size_t > > stack;
		stack.push_back(std::make_pair(Root(), size_t(0)));
		while(stack.size())
		{
			std::pair< const CellType *, size_t > c = stack.back();
			stack.pop_back();
			if(IsLeaf(*c.first))
			{
				++(*numLeaves);
				avgDepth += (ScalarType)c.second;
			}
			else
				for(unsigned int i = 0; i < Cell::NChildren; ++i)
					if(ExistChild(*c.first, i))
						stack.push_back(std::make_pair(&((*c.first)[i]), c.second + 1));
		}
		avgDepth /= ScalarType(*numLeaves);
		*variance = 0;
		stack.push_back(std::make_pair(Root(), size_t(0)));
		while(stack.size())
		{
			std::pair< const CellType *, size_t > c = stack.back();
			stack.pop_back();
			if(IsLeaf(*c.first))
			{
				ScalarType t = ScalarType(c.second) - avgDepth;
				*variance += t * t;
			}
			else
				for(unsigned int i = 0; i < Cell::NChildren; ++i)
					if(ExistChild(*c.first, i))
						stack.push_back(std::make_pair(&((*c.first)[i]), c.second + 1));
		}
		*variance /= ScalarType(*numLeaves);
	}
};

#endif
