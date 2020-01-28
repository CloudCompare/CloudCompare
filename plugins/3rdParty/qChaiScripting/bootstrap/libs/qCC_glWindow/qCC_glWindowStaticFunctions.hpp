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


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_glWindow
			{




				ModulePtr bootstrap_static_functions(ModulePtr m = std::make_shared<Module>())
				{

					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_GLWINDOW_STATIC_FUNCTIONS_HPP