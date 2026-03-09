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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// Local
#include "ccColorScale.h"

// CCFbo
#include <ccShader.h>

class QCC_DB_LIB_API ccColorRampShader : public ccShader
{
	Q_OBJECT

  public:
	//! Default constructor
	ccColorRampShader();

	//! Destructor
	~ccColorRampShader() override = default;

	//! Setups shader
	/** Shader must have already been stared!
	 **/
	bool setup(QOpenGLFunctions_2_1* glFunc, float minSatRel, float maxSatRel, unsigned colorSteps, const ccColorScale::Shared& colorScale);

	//! Returns the maximum color ramp size
	static unsigned MaxColorRampSize();

	//! Returns the minimum memory required on the shader side
	/** See GL_MAX_FRAGMENT_UNIFORM_COMPONENTS
	 **/
	static GLint MinRequiredBytes();
};
