#ifndef MiscLib__PAIR_HEADER__
#define MiscLib__PAIR_HEADER__
#include <utility>

namespace MiscLib
{
	template< class FirstT, class SecondT >
	struct Pair
	{
		typedef FirstT FirstType;
		typedef SecondT SecondType;
		Pair() {}
		Pair(FirstType &f, SecondType &s)
		: first(f)
		, second(s)
		{}
		Pair(const std::pair< FirstType, SecondType > &p)
		: first(p.first)
		, second(p.second)
		{}
		Pair< FirstType, SecondType > &operator=(
			const std::pair< FirstType, SecondType > &p)
		{
			first = p.first;
			second = p.second;
		}
		FirstType first;
		SecondType second;
	};

	template< class FirstT >
	struct Pair< FirstT, FirstT >
	{
		typedef FirstT FirstType;
		typedef FirstType SecondType;
		FirstType first, second;
		Pair() {}
		Pair(FirstType &f, SecondType &s)
		: first(f)
		, second(s)
		{}
		Pair(std::pair< FirstType, SecondType > &p)
		: first(p.first)
		, second(p.second)
		{}
		Pair< FirstType, SecondType > &operator=(
			const std::pair< FirstType, SecondType > &p)
		{
			first = p.first;
			second = p.second;
		}
		operator FirstType *() { return &first; }
		operator const FirstType *() const { return &first; }
	};

	template< class FirstT, class SecondT >
	Pair< FirstT, SecondT > MakePair(FirstT &first, SecondT &second)
	{
		return Pair< FirstT, SecondT >(first, second);
	}
};

#endif
