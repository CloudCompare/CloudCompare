// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                     COPYRIGHT: CloudCompare project                    #
// #                                                                        #
// ##########################################################################

#include "ccXRayDrawScope.h"

#include "ccGLDrawContext.h"

#include <cassert>

namespace
{
	thread_local const ccXRayDrawScope* s_current = nullptr;
}

ccXRayDrawScope::ccXRayDrawScope(const ccGLDrawContext& context, double exposure, bool reverse)
    : m_display(context.display)
    , m_glContext(context.qGLContext)
    , m_exposure(exposure)
    , m_reverse(reverse)
    , m_previous(s_current)
{
	s_current = this;
}

ccXRayDrawScope::~ccXRayDrawScope()
{
	assert(s_current == this);
	s_current = m_previous;
}

const ccXRayDrawScope* ccXRayDrawScope::Current(const ccGLDrawContext& context)
{
	if (!(context.drawingFlags & CC_DRAW_XRAY_ENABLED))
		return nullptr;
	for (auto* scope = s_current; scope; scope = scope->m_previous)
	{
		if (scope->m_display == context.display && scope->m_glContext == context.qGLContext)
			return scope;
	}
	return nullptr;
}
