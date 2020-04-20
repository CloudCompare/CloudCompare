//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccColorRampShader.h"

//! Maximum color ramp size
/** 252 so as to get 1024 bytes as total required memory
(see MinRequiredBytes).
**/
static const unsigned CC_MAX_SHADER_COLOR_RAMP_SIZE = 256;

//! Buffer for converting a color scale to packed values before sending it to shader
static float s_packedColormapf[CC_MAX_SHADER_COLOR_RAMP_SIZE];

unsigned ccColorRampShader::MaxColorRampSize()
{
	return CC_MAX_SHADER_COLOR_RAMP_SIZE;
}

GLint ccColorRampShader::MinRequiredBytes()
{
	return (CC_MAX_SHADER_COLOR_RAMP_SIZE + 4) * 4;
}

ccColorRampShader::ccColorRampShader()
	: ccShader()
{
}

bool ccColorRampShader::setup(QOpenGLFunctions_2_1* glFunc, float minSatRel, float maxSatRel, unsigned colorSteps, const ccColorScale::Shared& colorScale)
{
	assert(glFunc);

	if (colorSteps > CC_MAX_SHADER_COLOR_RAMP_SIZE)
	{
		colorSteps = CC_MAX_SHADER_COLOR_RAMP_SIZE;
	}

	setUniformValue("uf_minSaturation", minSatRel);
	setUniformValue("uf_maxSaturation", maxSatRel);
	setUniformValue("uf_colormapSize", static_cast<float>(colorSteps));

	static const double resolution = static_cast<double>(1 << 24);

	//set 'grayed' points color as a float-packed value
	{
		int rgb = (ccColor::lightGrey.a << 24) | (ccColor::lightGrey.r << 16) | (ccColor::lightGrey.g << 8) | ccColor::lightGrey.b;
		float packedColorGray = static_cast<float>(rgb / resolution);
		setUniformValue("uf_colorGray", packedColorGray);
	}

	//send colormap to shader
	assert(colorScale);
	for (unsigned i = 0; i < colorSteps; ++i)
	{
		const ccColor::Rgb* col = colorScale->getColorByRelativePos(static_cast<double>(i) / (colorSteps - 1), colorSteps);
		//set ramp colors as float-packed values
		int rgb = (col->r << 16) | (col->g << 8) | col->b;
		s_packedColormapf[i] = static_cast<float>(rgb / resolution);
	}
	setUniformValueArray("uf_colormapTable", s_packedColormapf, colorSteps, 1);

	return (glFunc->glGetError() == 0);
}
