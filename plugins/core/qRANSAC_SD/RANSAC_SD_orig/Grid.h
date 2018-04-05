#ifndef GRID_HEADER
#define GRID_HEADER

#include <unordered_map>

template< class CellT, unsigned int DimT >
class ArrayGridKernel
{
public:
	ArrayGridKernel()
	: m_array(NULL)
	{}

	ArrayGridKernel(ArrayGridKernel< CellT, DimT - 1 > *array)
	: m_array(array)
	{}

	~ArrayGridKernel()
	{
		delete[] m_array;
	}

	ArrayGridKernel< CellT, DimT - 1 > *&Data()
	{
		return m_array;
	}

protected:
	ArrayGridKernel< CellT, DimT - 1 > *m_array;
};


template< class CellT >
class ArrayGridKernel< CellT, 0 >
{
public:
	operator CellT &()
	{
		return m_cell;
	}

	operator const CellT &() const
	{
		return m_cell;
	}

	CellT &Data()
	{
		return m_cell;
	}

protected:
	CellT m_cell;
};


template< class CellT, unsigned int DimT >
class ArrayGridAccessor
{
public:
	ArrayGridAccessor(unsigned int *extent,
		ArrayGridKernel< CellT, DimT - 1 > *&array)
	: m_extent(extent)
	, m_array(array)
	{}

	ArrayGridAccessor< CellT, DimT - 1 > operator[](unsigned int i)
	{
		if(!m_array)
			m_array = new ArrayGridKernel< CellT, DimT - 1 >[*m_extent];
		return ArrayGridAccessor< CellT, DimT - 1 >(m_extent + 1,
			m_array[i].Data());
	}

private:
	unsigned int *m_extent;
	ArrayGridKernel< CellT, DimT - 1 > *&m_array;
};


template< class CellT >
class ArrayGridAccessor< CellT, 0 >
{
public:
	ArrayGridAccessor(unsigned int *, CellT &cell)
	: m_cell(cell)
	{}

	CellT &operator=(const CellT &c)
	{
		return m_cell = c;
	}

	operator CellT &()
	{
		return m_cell;
	}

private:
	CellT &m_cell;
};

template< class CellT, unsigned int DimT >
class ConstArrayGridAccessor
{
public:
	ConstArrayGridAccessor(const ArrayGridKernel< CellT, DimT - 1 > *array)
	: m_array(array)
	{}

	const ConstArrayGridAccessor< CellT, DimT - 1 > operator[](
		unsigned int i) const
	{
		return ConstArrayGridAccessor< CellT, DimT - 1 >(m_array[i].Data());
	}

private:
	const ArrayGridKernel< CellT, DimT - 1 > *m_array;
};

template< class CellT >
class ConstArrayGridAccessor< CellT, 0 >
{
public:
	ConstArrayGridAccessor(unsigned int *, const CellT &cell)
	: m_cell(cell)
	{}

	operator const CellT &() const
	{
		return m_cell;
	}

private:
	const CellT &m_cell;
};

template< class CellT, unsigned int DimT >
class ArrayGrid
: protected ArrayGridKernel< CellT, DimT >
{
public:
	ArrayGrid()
	{
		for(unsigned int i = 0; i < DimT; ++i)
			m_extent[i] = 0;
	}

	ArrayGrid(const MiscLib::Vector< unsigned int > &extent)
	{
		Extent(&extent[0]);
	}

	ArrayGrid(const unsigned int *extent)
	{
		Extent(extent);
	}

	void Clear()
	{
		delete this->m_array;
		this->m_array = NULL;
	}

	void Extent(const unsigned int *extent)
	{
		Clear();
		for(unsigned int i = 0; i < DimT; ++i)
			m_extent[i] = extent[i];
	}

	ArrayGridAccessor< CellT, DimT - 1 > operator[](unsigned int i)
	{
		return ArrayGridAccessor< CellT, DimT >(m_extent, this->m_array)[i];
	}

	const ConstArrayGridAccessor< CellT, DimT - 1 > operator[](
		unsigned int i) const
	{
		return ConstArrayGridAccessor< CellT, DimT >(this->m_array)[i];
	}

private:
	unsigned int m_extent[DimT];
};


template< class CellT, unsigned int DimT >
class HashGridAccessor
{
public:
	HashGridAccessor(const size_t *factors, size_t hashKey,
		std::unordered_map< size_t, CellT > &hash)
	: m_factors(factors)
	, m_hashKey(hashKey)
	, m_hash(hash)
	{}

	HashGridAccessor< CellT, DimT - 1 > operator[](size_t i)
	{
		return HashGridAccessor< CellT, DimT - 1 >(m_factors + 1,
			m_hashKey + (*m_factors) * i, m_hash);
	}

private:
	const size_t *m_factors;
	size_t m_hashKey;
	std::unordered_map< size_t, CellT > &m_hash;
};

template< class CellT >
class HashGridAccessor< CellT, 0 >
{
public:
	HashGridAccessor(const size_t *, size_t hashKey,
		std::unordered_map< size_t, CellT > &hash)
	: m_hashKey(hashKey)
	, m_hash(hash)
	{}

	CellT &operator=(const CellT &c)
	{
		return m_hash[m_hashKey] = c;
	}

	operator CellT &()
	{
		return m_hash[m_hashKey];
	}

	CellT &operator()()
	{
		return m_hash[m_hashKey];
	}

	CellT *operator->()
	{
		return &m_hash[m_hashKey];
	}

private:
	size_t m_hashKey;
	std::unordered_map< size_t, CellT > &m_hash;
};

template< class CellT, unsigned int DimT >
class ConstHashGridAccessor
{
public:
	ConstHashGridAccessor(const size_t *factors, size_t hashKey,
		const std::unordered_map< size_t, CellT > &hash)
	: m_factors(factors)
	, m_hashKey(hashKey)
	, m_hash(hash)
	{}

	ConstHashGridAccessor< CellT, DimT - 1 > operator[](size_t i)
	{
		return ConstHashGridAccessor< CellT, DimT - 1 >(m_factors + 1,
			m_hashKey + (*m_factors) * i, m_hash);
	}

private:
	const size_t *m_factors;
	size_t m_hashKey;
	const std::unordered_map< size_t, CellT > &m_hash;
};

template< class CellT >
class ConstHashGridAccessor< CellT, 0 >
{
public:
	ConstHashGridAccessor(const size_t *, size_t hashKey,
		const std::unordered_map< size_t, CellT > &hash)
	: m_hashKey(hashKey)
	, m_hash(hash)
	{}

	operator const CellT *()
	{
		typename std::unordered_map< size_t, CellT >::const_iterator i =
			m_hash.find(m_hashKey);
		if(i != m_hash.end())
			return &i->second;
		return NULL;
	}

private:
	size_t m_hashKey;
	const std::unordered_map< size_t, CellT > &m_hash;
};

template< class CellT, unsigned int DimT >
class HashGrid
{
public:
	typedef typename std::unordered_map< size_t, CellT >::iterator iterator;
	typedef typename std::unordered_map< size_t, CellT >::const_iterator
		const_iterator;

	HashGrid()
	{
		for(unsigned int i = 0; i < DimT; ++i)
			m_factors[i] = 0;
	}

	void Clear()
	{
		m_hash.clear();
	}

	template< class ExtentT >
	void Extent(const ExtentT &extent)
	{
		m_factors[DimT - 1] = 1;
		for(unsigned int i = DimT - 1; i != 0; --i)
			m_factors[i - 1] = m_factors[i] * extent[i];
	}

	HashGridAccessor< CellT, DimT - 1 > operator[](size_t i)
	{
		return HashGridAccessor< CellT, DimT >(m_factors, 0, m_hash)[i];
	}

	ConstHashGridAccessor< CellT, DimT - 1 > operator[](size_t i) const
	{
		return ConstHashGridAccessor< CellT, DimT >(m_factors, 0, m_hash)[i];
	}

	template< class IndexT >
	CellT &operator[](const IndexT *index)
	{
		return m_hash[HashKey(index)];
	}

	CellT &at(size_t hashKey)
	{
		return m_hash[hashKey];
	}

	const CellT &at(size_t hashKey) const
	{
		return m_hash[hashKey];
	}

	template< class IndexT >
	CellT *find(const IndexT &index)
	{
		iterator i = m_hash.find(HashKey(index));
		if(i != m_hash.end())
			return &i->second;
		return NULL;
	}

	template< class IndexT >
	const CellT *find(const IndexT &index) const
	{
		const_iterator i = m_hash.find(HashKey(index));
		if(i != m_hash.end())
			return &i->second;
		return NULL;
	}

	iterator begin() { return m_hash.begin(); }
	iterator end() { return m_hash.end(); }
	const_iterator begin() const{ return m_hash.begin(); }
	const_iterator end() const { return m_hash.end(); }
	size_t size() const { return m_hash.size(); }

private:
	template< class IndexT >
	size_t HashKey(const IndexT &index) const
	{
		size_t hashKey = m_factors[0] * index[0];
		for(unsigned int i = 1; i < DimT; ++i)
			hashKey += m_factors[i] * index[i];
		return hashKey;
	}

private:
	size_t m_factors[DimT];
	std::unordered_map< size_t, CellT > m_hash;
};

#endif
