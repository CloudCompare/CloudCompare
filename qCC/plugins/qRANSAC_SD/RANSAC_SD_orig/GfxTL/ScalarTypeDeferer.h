#ifndef GfxTL__SCALARTYPEDEFERER_HEADER__
#define GfxTL__SCALARTYPEDEFERER_HEADER__
#include <GfxTL/VectorXD.h>
#include <GfxTL/Array.h>
#include <MiscLib/Vector.h>
#include <vector>
#include <memory>

namespace GfxTL
{
	template< class PointT >
	struct ScalarTypeDeferer
	{
		typedef typename PointT::value_type ScalarType;
	};

	template< class ScalarT >
	struct ScalarTypeDeferer< ScalarT * >
	{
		typedef ScalarT ScalarType;
	};

	template< class ScalarT >
	struct ScalarTypeDeferer< const ScalarT * >
	{
		typedef ScalarT ScalarType;
	};

	template< unsigned int DimT, class ScalarT >
	struct ScalarTypeDeferer< VectorXD< DimT, ScalarT > >
	{
		typedef ScalarT ScalarType;
	};

	template< >
	struct ScalarTypeDeferer< float >
	{
		typedef float ScalarType;
	};

	template< >
	struct ScalarTypeDeferer< double >
	{
		typedef double ScalarType;
	};

	template< >
	struct ScalarTypeDeferer< int >
	{
		typedef int ScalarType;
	};

	template< >
	struct ScalarTypeDeferer< char >
	{
		typedef char ScalarType;
	};

	template< >
	struct ScalarTypeDeferer< short >
	{
		typedef short ScalarType;
	};

	template< class IteratorT >
	struct ScalarTypeDeferer< ArrayAccessor< 1, IteratorT > >
	{
		typedef typename ArrayAccessor< 1, IteratorT >::value_type ScalarType;
	};

	template< class T >
	struct ScalarTypeDeferer< std::auto_ptr< T > >
	{
		typedef T ScalarType;
	};

	template< class T, class A >
	struct ScalarTypeDeferer< MiscLib::Vector< T, A > >
	{
		typedef T ScalarType;
	};

	template< class T, class A >
	struct ScalarTypeDeferer< std::vector< T, A > >
	{
		typedef T ScalarType;
	};
};

#endif
