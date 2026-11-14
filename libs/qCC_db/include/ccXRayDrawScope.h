#pragma once

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

#include "qCC_db.h"

struct ccGLDrawContext;
class ccGenericGLDisplay;
class QOpenGLContext;

//! Scoped state for one view's rendering pass; supports nested draws and context copies.
//! Access is per cloud, never per point. The stack lives in QCC_DB_LIB, not in each DLL.
class QCC_DB_LIB_API ccXRayDrawScope
{
  public:
	//! Constructor
	ccXRayDrawScope(const ccGLDrawContext& context, double exposure, bool reverse);
	//! Destructor
	~ccXRayDrawScope();

	// Deleted copy constructor and assignment operator to prevent copying
	ccXRayDrawScope(const ccXRayDrawScope&)            = delete;
	ccXRayDrawScope& operator=(const ccXRayDrawScope&) = delete;

	//! Returns the current scope for the given context, or nullptr if none is active.
	static const ccXRayDrawScope* Current(const ccGLDrawContext& context);

	//! Returns the exposure value for the current scope.
	double exposure() const
	{
		return m_exposure;
	}

	//! Returns the reverse flag for the current scope.
	bool reverse() const
	{
		return m_reverse;
	}

  protected:                               // members
	const ccGenericGLDisplay* m_display;   //!< The display for which this scope is active; nullptr if none.
	const QOpenGLContext*     m_glContext; //!< The OpenGL context for which this scope is active; nullptr if none.
	double                    m_exposure;  //!< The exposure value for the current scope.
	bool                      m_reverse;   //!< The reverse flag for the current scope.
	const ccXRayDrawScope*    m_previous;  //!< The previous scope in the stack; nullptr if none.
};
