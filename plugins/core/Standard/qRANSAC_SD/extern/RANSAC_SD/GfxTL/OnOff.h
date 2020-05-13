#ifndef GfxTL__ONOFF_HEADER__
#define GfxTL__ONOFF_HEADER__

namespace GfxTL
{

template< bool T >
class OnOff {};

typedef OnOff< true > On;
typedef OnOff< false > Off;
typedef OnOff< true > Yes;
typedef OnOff< false > No;

};

#endif
