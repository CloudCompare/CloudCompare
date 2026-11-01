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

#pragma once

#include "qCC_db.h"
struct ccGLDrawContext;
class ccGenericGLDisplay;
class QOpenGLContext;

//! Scoped state for one view's rendering pass; supports nested draws and context copies.
//! Access is per cloud, never per point. The stack lives in QCC_DB_LIB, not in each DLL.
class QCC_DB_LIB_API ccXRayDrawScope
{
  public:
	ccXRayDrawScope(const ccGLDrawContext& context, double exposure, bool reverse);
	~ccXRayDrawScope();
	ccXRayDrawScope(const ccXRayDrawScope&)            = delete;
	ccXRayDrawScope& operator=(const ccXRayDrawScope&) = delete;

	static const ccXRayDrawScope* Current(const ccGLDrawContext& context);
	double                        exposure() const
	{
		return m_exposure;
	}
	bool reverse() const
	{
		return m_reverse;
	}

  private:
	const ccGenericGLDisplay* m_display;
	const QOpenGLContext*     m_glContext;
	double                    m_exposure;
	bool                      m_reverse;
	const ccXRayDrawScope*    m_previous;
};
