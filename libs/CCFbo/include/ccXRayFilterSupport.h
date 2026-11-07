#pragma once
// ##########################################################################
// #                                                                        #
// #                               CCFBO                                    #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU Library General Public License as       #
// #  published by the Free Software Foundation; version 2 or later of the  #
// #  License.                                                              #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

#include "CCFbo.h"

//! Optional interface for filters incompatible with X-ray compositing.
//! Existing filters need not implement it; ccGlFilter's layout and vtable stay unchanged.
class CCFBO_LIB_API ccXRayFilterSupport
{
  public:
	virtual ~ccXRayFilterSupport();
	virtual bool supportsXRay() const = 0;
};
