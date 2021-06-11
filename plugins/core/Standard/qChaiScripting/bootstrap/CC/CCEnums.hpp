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
#include <CloudSamplingTools.h>
#include <DgmOctree.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace CC
		{
			

			ModulePtr bs_CCConst(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<CCCoreLib::LOCAL_MODEL_TYPES>(*m,
					"LOCAL_MODEL_TYPES",
					{
						{ CCCoreLib::LOCAL_MODEL_TYPES::NO_MODEL, "NO_MODEL" },
						{ CCCoreLib::LOCAL_MODEL_TYPES::LS, "LS" },
						{ CCCoreLib::LOCAL_MODEL_TYPES::TRI, "TRI" },
						{ CCCoreLib::LOCAL_MODEL_TYPES::QUADRIC, "QUADRIC" }
					}
				);

				chaiscript::utility::add_class<CCCoreLib::CHAMFER_DISTANCE_TYPE>(*m,
					"CHAMFER_DISTANCE_TYPE",
					{
						{ CCCoreLib::CHAMFER_DISTANCE_TYPE::CHAMFER_111, "CHAMFER_111" },
						{ CCCoreLib::CHAMFER_DISTANCE_TYPE::CHAMFER_345, "CHAMFER_345" }
					}
				);

				return m;
			}


			ModulePtr bs_CloudSamplingToolsEnum(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCCoreLib;
				chaiscript::utility::add_class<CloudSamplingTools::RESAMPLING_CELL_METHOD>(*m,
					"RESAMPLING_CELL_METHOD",
					{
						{ CloudSamplingTools::RESAMPLING_CELL_METHOD::CELL_CENTER, "CELL_CENTER" },
						{ CloudSamplingTools::RESAMPLING_CELL_METHOD::CELL_GRAVITY_CENTER, "CELL_GRAVITY_CENTER" },
					}
				);

				chaiscript::utility::add_class<CloudSamplingTools::SUBSAMPLING_CELL_METHOD>(*m,
					"SUBSAMPLING_CELL_METHOD",
					{
						{ CloudSamplingTools::SUBSAMPLING_CELL_METHOD::RANDOM_POINT, "RANDOM_POINT" },
						{ CloudSamplingTools::SUBSAMPLING_CELL_METHOD::NEAREST_POINT_TO_CELL_CENTER, "NEAREST_POINT_TO_CELL_CENTER" }
					}
				);

				return m;
			}

			ModulePtr bs_DgmOctreeEnum(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCCoreLib;
				chaiscript::utility::add_class<DgmOctree::RayCastProcess>(*m,
					"RayCastProcess",
					{
						{ DgmOctree::RayCastProcess::RC_NEAREST_POINT, "RC_NEAREST_POINT" },
						{ DgmOctree::RayCastProcess::RC_CLOSE_POINTS, "RC_CLOSE_POINTS" },
					}
				);
				return m;
			}


			ModulePtr bootstrap_enum(ModulePtr m = std::make_shared<Module>())
			{
				bs_CCConst(m);
				bs_CloudSamplingToolsEnum(m);
				bs_DgmOctreeEnum(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_ENUMS_HPP