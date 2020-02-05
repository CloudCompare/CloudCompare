#ifndef CHAISCRIPTING_BOOTSTRAP_CC_STATIC_FUNCTIONS_HPP
#define CHAISCRIPTING_BOOTSTRAP_CC_STATIC_FUNCTIONS_HPP

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
#include <DistanceComputationTools.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace CC
		{
			using namespace CCLib;
			ModulePtr bs_DistanceComputationTools(ModulePtr m = std::make_shared<Module>())
			{
				m->add(fun(&DistanceComputationTools::computeCloud2CloudDistance), "computeCloud2CloudDistance");
				m->add(fun(&DistanceComputationTools::computeCloud2MeshDistance), "computeCloud2MeshDistance");
				m->add(fun(&DistanceComputationTools::computeApproxCloud2CloudDistance), "computeApproxCloud2CloudDistance");
				m->add(fun(&DistanceComputationTools::computePoint2TriangleDistance), "computePoint2TriangleDistance");
				m->add(fun(&DistanceComputationTools::computePoint2PlaneDistance), "computePoint2PlaneDistance");
				m->add(fun(&DistanceComputationTools::computePoint2LineSegmentDistSquared), "computePoint2LineSegmentDistSquared");
				m->add(fun(&DistanceComputationTools::computeCloud2ConeEquation), "computeCloud2ConeEquation");
				m->add(fun(&DistanceComputationTools::computeCloud2CylinderEquation), "computeCloud2CylinderEquation");
				m->add(fun(&DistanceComputationTools::computeCloud2SphereEquation), "computeCloud2SphereEquation");
				m->add(fun(&DistanceComputationTools::computeCloud2PlaneEquation), "computeCloud2PlaneEquation");
				m->add(fun(&DistanceComputationTools::computeCloud2RectangleEquation), "computeCloud2RectangleEquation");
				m->add(fun(&DistanceComputationTools::computeCloud2BoxEquation), "computeCloud2BoxEquation");
				m->add(fun(&DistanceComputationTools::computeCloud2PolylineEquation), "computeCloud2PolylineEquation");
				m->add(fun(&DistanceComputationTools::ComputeCloud2PlaneDistance), "ComputeCloud2PlaneDistance");
				m->add(fun(&DistanceComputationTools::ComputeCloud2PlaneRobustMax), "ComputeCloud2PlaneRobustMax");
				m->add(fun(&DistanceComputationTools::ComputeCloud2PlaneMaxDistance), "ComputeCloud2PlaneMaxDistance");
				m->add(fun(&DistanceComputationTools::computeCloud2PlaneDistanceRMS), "computeCloud2PlaneDistanceRMS");
				m->add(fun(&DistanceComputationTools::ComputeSquareDistToSegment), "ComputeSquareDistToSegment");
				m->add(fun(&DistanceComputationTools::computeGeodesicDistances), "computeGeodesicDistances");
				m->add(fun(&DistanceComputationTools::diff), "diff");
				m->add(fun(&DistanceComputationTools::synchronizeOctrees), "synchronizeOctrees");
				m->add(fun(&DistanceComputationTools::MultiThreadSupport), "MultiThreadSupport");				
				return m;
			}
			



			ModulePtr bootstrap_static_functions(ModulePtr m = std::make_shared<Module>())
			{
				bs_DistanceComputationTools(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_STATIC_FUNCTIONS_HPP