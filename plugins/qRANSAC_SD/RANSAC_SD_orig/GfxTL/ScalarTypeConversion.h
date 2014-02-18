#ifndef GfxTL__SCALARTYPECONVERSION_HEADER__
#define GfxTL__SCALARTYPECONVERSION_HEADER__

namespace GfxTL
{
	template< class A, class B >
	struct ScalarTypeConversion
	{};

	template< >
	struct ScalarTypeConversion< char, char >
	{
		typedef short DifferenceType;
		typedef short AdditionType;
		typedef short MultiplicationType;
		typedef float DivisionType;
	};

	template< >
	struct ScalarTypeConversion< short, short >
	{
		typedef int DifferenceType;
		typedef int AdditionType;
		typedef int MultiplicationType;
		typedef float DivisionType;
	};

	template< >
	struct ScalarTypeConversion< int, int >
	{
		typedef int DifferenceType;
		typedef int AdditionType;
		typedef int MultiplicationType;
		typedef float DivisionType;
	};

	template< >
	struct ScalarTypeConversion< float, float >
	{
		typedef float DifferenceType;
		typedef float AdditionType;
		typedef float MultiplicationType;
		typedef float DivisionType;
	};

	template< >
	struct ScalarTypeConversion< double, double >
	{
		typedef double DifferenceType;
		typedef double AdditionType;
		typedef double MultiplicationType;
		typedef double DivisionType;
	};

	template< >
	struct ScalarTypeConversion< char, short >
	: public ScalarTypeConversion< short, short >
	{};

	template< >
	struct ScalarTypeConversion< short, char >
	: public ScalarTypeConversion< short, short >
	{};

	template< >
	struct ScalarTypeConversion< char, int >
	: public ScalarTypeConversion< int, int >
	{};

	template< >
	struct ScalarTypeConversion< int, char >
	: public ScalarTypeConversion< int, int >
	{};

	template< >
	struct ScalarTypeConversion< char, float >
	: public ScalarTypeConversion< float, float >
	{};

	template< >
	struct ScalarTypeConversion< float, char >
	: public ScalarTypeConversion< float, float >
	{};

	template< >
	struct ScalarTypeConversion< char, double >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< double, char >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< short, int >
	: public ScalarTypeConversion< int, int >
	{};

	template< >
	struct ScalarTypeConversion< int, short >
	: public ScalarTypeConversion< int, int >
	{};

	template< >
	struct ScalarTypeConversion< short, float >
	: public ScalarTypeConversion< float, float >
	{};

	template< >
	struct ScalarTypeConversion< float, short >
	: public ScalarTypeConversion< float, float >
	{};

	template< >
	struct ScalarTypeConversion< short, double >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< double, short >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< int, float >
	: public ScalarTypeConversion< float, float >
	{};

	template< >
	struct ScalarTypeConversion< float, int >
	: public ScalarTypeConversion< float, float >
	{};

	template< >
	struct ScalarTypeConversion< int, double >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< double, int >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< float, double >
	: public ScalarTypeConversion< double, double >
	{};

	template< >
	struct ScalarTypeConversion< double, float >
	: public ScalarTypeConversion< double, double >
	{};
};

#endif
