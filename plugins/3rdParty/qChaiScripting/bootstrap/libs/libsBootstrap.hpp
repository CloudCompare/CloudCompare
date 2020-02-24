#ifndef CHAISCRIPTING_BOOTSTRAP_LIBS_HPP
#define CHAISCRIPTING_BOOTSTRAP_LIBS_HPP

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
#include "qCC_db/qCC_dbBootstrap.hpp"
#include "qCC_io/qCC_ioBootstrap.hpp"
#include "qCC_glWindow/qCC_glWindowBootstrap.hpp"


namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{

			ModulePtr bootstrap(ModulePtr m = std::make_shared<Module>())
			{
				qCC_db::bootstrap(m);
				qCC_io::bootstrap(m);
				qCC_glWindow::bootstrap(m);
				return m;
			}
		}
	}
}

#endif // CHAISCRIPTING_BOOTSTRAP_LIBS_HPP