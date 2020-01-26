#ifndef CHAISCRIPTING_BOOTSTRAP_CC_ENUMS_HPP
#define CHAISCRIPTING_BOOTSTRAP_CC_ENUMS_HPP

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

#include <CCConst.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace CC
		{
			

			ModulePtr CCConst(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<CC_LOCAL_MODEL_TYPES>(*m,
					"CC_LOCAL_MODEL_TYPES",
					{
						{ CC_LOCAL_MODEL_TYPES::NO_MODEL, "NO_MODEL" },
						{ CC_LOCAL_MODEL_TYPES::LS, "LS" },
						{ CC_LOCAL_MODEL_TYPES::TRI, "TRI" },
						{ CC_LOCAL_MODEL_TYPES::QUADRIC, "QUADRIC" }
					}
				);

				chaiscript::utility::add_class<CC_CHAMFER_DISTANCE_TYPE>(*m,
					"CC_CHAMFER_DISTANCE_TYPE",
					{
						{ CC_CHAMFER_DISTANCE_TYPE::CHAMFER_111, "CHAMFER_111" },
						{ CC_CHAMFER_DISTANCE_TYPE::CHAMFER_345, "CHAMFER_345" }
					}
				);

				return m;
			}


			ModulePtr bootstrap_enum(ModulePtr m = std::make_shared<Module>())
			{
				CCConst(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_ENUMS_HPP