#ifndef PRIMITIVESHAPEVISITOR_HEADER
#define PRIMITIVESHAPEVISITOR_HEADER

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE PlanePrimitiveShape;
class DLL_LINKAGE SpherePrimitiveShape;
class DLL_LINKAGE CylinderPrimitiveShape;
class DLL_LINKAGE ConePrimitiveShape;
class DLL_LINKAGE TorusPrimitiveShape;

class DLL_LINKAGE PrimitiveShapeVisitor
{
public:
	virtual ~PrimitiveShapeVisitor() {}
	virtual void Visit(const PlanePrimitiveShape &plane) = 0;
	virtual void Visit(const SpherePrimitiveShape &sphere) = 0;
	virtual void Visit(const CylinderPrimitiveShape &cylinder) = 0;
	virtual void Visit(const ConePrimitiveShape &cone) = 0;
	virtual void Visit(const TorusPrimitiveShape &torus) = 0;
};

template< class BaseT >
class PrimitiveShapeVisitorShell
: public BaseT
{
public:
	PrimitiveShapeVisitorShell() {}

	template< class T >
	PrimitiveShapeVisitorShell(const T &t)
	: BaseT(t)
	{}

	template< class A, class B >
	PrimitiveShapeVisitorShell(const A &a, const B &b)
	: BaseT(a, b)
	{}

	void Visit(const PlanePrimitiveShape &plane)
	{
		BaseT::Visit(plane);
	}

	void Visit(const SpherePrimitiveShape &sphere)
	{
		BaseT::Visit(sphere);
	}

	void Visit(const CylinderPrimitiveShape &cylinder)
	{
		BaseT::Visit(cylinder);
	}

	void Visit(const ConePrimitiveShape &cone)
	{
		BaseT::Visit(cone);
	}

	void Visit(const TorusPrimitiveShape &torus)
	{
		BaseT::Visit(torus);
	}
};

#endif
