#ifndef __GfxTL_MATHHELPER_HEADER__
#define __GfxTL_MATHHELPER_HEADER__
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

namespace GfxTL
{
	template< class ScalarT >
	class Math
	{
		public:
			static inline ScalarT Abs(ScalarT s)
			{
				if(s < 0)
					return -s;
				return s;
			}

			static inline ScalarT Sign(ScalarT s)
			{
				if(s < 0)
					return -1;
				else if(s > 0)
					return 1;
				return 0;
			}

			static inline ScalarT Clamp(ScalarT s, ScalarT bottom, ScalarT top)
			{
				if(s < bottom)
					return bottom;
				if(s > top)
					return top;
				return s;
			}

			static inline ScalarT Max(ScalarT a, ScalarT b)
			{
				if(a > b)
					return a;
				return b;
			}

			static inline ScalarT Min(ScalarT a, ScalarT b)
			{
				if(a < b)
					return a;
				return b;
			}
	};
};

#endif

