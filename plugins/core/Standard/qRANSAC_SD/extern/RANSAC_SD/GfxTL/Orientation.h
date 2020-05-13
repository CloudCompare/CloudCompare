#ifndef GfxTL__ORIENTATION_HEADER__
#define GfxTL__ORIENTATION_HEADER__

#include <GfxTL/VectorXD.h>

namespace GfxTL
{
	template< class ScalarT >
	inline ScalarT Orientation(const VectorXD< 2, ScalarT > &p1,
		const VectorXD< 2, ScalarT > &p2,
		const VectorXD< 2, ScalarT > &c)
	{
		return ((p1[0] - c[0]) * (p2[1] - c[1])) -
			((p1[1] - c[1]) * (p2[0] - c[0]));
	}
};

#endif
