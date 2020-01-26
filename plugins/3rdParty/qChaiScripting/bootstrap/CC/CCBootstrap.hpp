#ifndef CHAISCRIPTING_BOOTSTRAP_CC_HPP
#define CHAISCRIPTING_BOOTSTRAP_CC_HPP

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

#include <cmath>
#include <memory>

#include <chaiscript/chaiscript.hpp>
#include "CCEnums.hpp"
#include "CCClasses.hpp"

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace CC
		{

			ModulePtr bootstrap(ModulePtr m = std::make_shared<Module>())
			{
				bootstrap_enum(m);
				bootstrap_classes(m);
				return m;
			}
		}
	}
}

#endif CHAISCRIPTING_BOOTSTRAP_CC_HPP