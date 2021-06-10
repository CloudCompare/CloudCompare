#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_STATIC_FUNCTIONS_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_STATIC_FUNCTIONS_HPP

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ChaiScripting                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                     COPYRIGHT: Chris S Brown                           #
//#                                                                        #
//##########################################################################


#include <chaiscript/chaiscript.hpp>
#include <chaiscript/utility/utility.hpp>

#include <ccGLUtils.h>
#include <ccRenderingTools.h>


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_glWindow
			{

				ModulePtr bs_ccGLUtils(ModulePtr m = std::make_shared<Module>())
				{

					m->add(fun(static_cast<void(*)(QImage, int, int, int, int, unsigned char)>(&ccGLUtils::DisplayTexture2DPosition)), "DisplayTexture2DPosition");
					m->add(fun(static_cast<void(*)(QImage, int, int, unsigned char)>(&ccGLUtils::DisplayTexture2D)), "DisplayTexture2D");
					m->add(fun(static_cast<void(*)(GLuint, int, int, int, int, unsigned char)>(&ccGLUtils::DisplayTexture2DPosition)), "DisplayTexture2DPosition");
					m->add(fun(static_cast<void(*)(GLuint, int, int, unsigned char)>(&ccGLUtils::DisplayTexture2D)), "DisplayTexture2D");

					m->add(fun(&ccGLUtils::GenerateViewMat), "GenerateViewMat");
					return m;
				}

				ModulePtr bs_ccRenderingTools(ModulePtr m = std::make_shared<Module>())
				{
					m->add(fun(&ccRenderingTools::ShowDepthBuffer), "ShowDepthBuffer");
					m->add(fun([](ccGBLSensor* a, QWidget* b) {ccRenderingTools::ShowDepthBuffer(a, b); }), "ShowDepthBuffer");
					m->add(fun([](ccGBLSensor* a) {ccRenderingTools::ShowDepthBuffer(a); }), "ShowDepthBuffer");
					m->add(fun(static_cast<void(*)(const CC_DRAW_CONTEXT&)>(&ccRenderingTools::DrawColorRamp)), "DrawColorRamp");
					m->add(fun(static_cast<void(*)(const CC_DRAW_CONTEXT&, const ccScalarField*, ccGLWindow*, int, int, float)>(&ccRenderingTools::DrawColorRamp)), "DrawColorRamp");
					return m;
				}


				ModulePtr bootstrap_static_functions(ModulePtr m = std::make_shared<Module>())
				{
					bs_ccGLUtils(m);
					bs_ccRenderingTools(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_STATIC_FUNCTIONS_HPP