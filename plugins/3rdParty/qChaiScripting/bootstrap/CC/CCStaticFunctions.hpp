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
#include <CCMiscTools.h>
#include <AutoSegmentationTools.h>
#include <CloudSamplingTools.h>
#include <ErrorFunction.h>
#include <GeometricalAnalysisTools.h>
#include <Polyline.h>
#include <StatisticalTestingTools.h>
#include <ManualSegmentationTools.h>
#include <SimpleMesh.h>

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
			
			ModulePtr bs_CCMiscTools(ModulePtr m = std::make_shared<Module>())
			{
				m->add(fun(&CCMiscTools::EnlargeBox), "EnlargeBox");
				m->add(fun(&CCMiscTools::MakeMinAndMaxCubical), "MakeMinAndMaxCubical");
				m->add(fun(static_cast<void(*)(const CCVector3&,CCVector3&,	CCVector3&)>(&CCMiscTools::ComputeBaseVectors)), "ComputeBaseVectors");
				m->add(fun(static_cast<void(*)(const CCVector3d&, CCVector3d&, CCVector3d&)>(&CCMiscTools::ComputeBaseVectors)), "ComputeBaseVectors");
				m->add(fun(&CCMiscTools::TriBoxOverlap), "TriBoxOverlap");
				m->add(fun(&CCMiscTools::TriBoxOverlapd), "TriBoxOverlapd");
				chaiscript::bootstrap::array<CCVector3[3]>("triverts_Array", m);
				chaiscript::bootstrap::array<CCVector3d[3]>("triverts_Array", m);
				return m;
			}

			ModulePtr bs_AutoSegmentationTools(ModulePtr m = std::make_shared<Module>())
			{
				m->add(user_type<ReferenceCloudContainer>(), "ReferenceCloudContainer");
				m->add(fun(&AutoSegmentationTools::labelConnectedComponents), "labelConnectedComponents");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, unsigned char l) {return AutoSegmentationTools::labelConnectedComponents(theCloud, l); }), "labelConnectedComponents");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, unsigned char l, bool sC) {return AutoSegmentationTools::labelConnectedComponents(theCloud, l, sC); }), "labelConnectedComponents");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, unsigned char l, bool sC, GenericProgressCallback* pcb) {return AutoSegmentationTools::labelConnectedComponents(theCloud, l, sC, pcb); }), "labelConnectedComponents");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, unsigned char l, bool sC, GenericProgressCallback* pcb, DgmOctree* io) {return AutoSegmentationTools::labelConnectedComponents(theCloud, l, sC, pcb, io); }), "labelConnectedComponents");
				m->add(fun(&AutoSegmentationTools::extractConnectedComponents), "extractConnectedComponents");
				m->add(fun(&AutoSegmentationTools::frontPropagationBasedSegmentation), "frontPropagationBasedSegmentation");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, PointCoordinateType r, ScalarType m, unsigned char ol, ReferenceCloudContainer& theSL) {return AutoSegmentationTools::frontPropagationBasedSegmentation(theCloud, r, m, ol, theSL); }), "frontPropagationBasedSegmentation");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, PointCoordinateType r, ScalarType m, unsigned char ol, ReferenceCloudContainer& theSL, GenericProgressCallback* pcb) {return AutoSegmentationTools::frontPropagationBasedSegmentation(theCloud, r, m, ol, theSL, pcb); }), "frontPropagationBasedSegmentation");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, PointCoordinateType r, ScalarType m, unsigned char ol, ReferenceCloudContainer& theSL, GenericProgressCallback* pcb, DgmOctree* io) {return AutoSegmentationTools::frontPropagationBasedSegmentation(theCloud, r, m, ol, theSL, pcb, io); }), "frontPropagationBasedSegmentation");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, PointCoordinateType r, ScalarType m, unsigned char ol, ReferenceCloudContainer& theSL, GenericProgressCallback* pcb, DgmOctree* io, bool agf) {return AutoSegmentationTools::frontPropagationBasedSegmentation(theCloud, r, m, ol, theSL, pcb, io, agf); }), "frontPropagationBasedSegmentation");

				return m;
			}

			ModulePtr bs_CloudSamplingTools(ModulePtr m = std::make_shared<Module>())
			{
				m->add(user_type<CloudSamplingTools::SFModulationParams>(), "SFModulationParams");
				m->add(constructor<CloudSamplingTools::SFModulationParams(bool)>(), "SFModulationParams");
				m->add(fun(&CloudSamplingTools::SFModulationParams::enabled), "enabled");
				m->add(fun(&CloudSamplingTools::SFModulationParams::a), "a");
				m->add(fun(&CloudSamplingTools::SFModulationParams::b), "b");


				m->add(fun(&CloudSamplingTools::resampleCloudWithOctreeAtLevel), "resampleCloudWithOctreeAtLevel");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, unsigned char l, CloudSamplingTools::RESAMPLING_CELL_METHOD rm) {return CloudSamplingTools::resampleCloudWithOctreeAtLevel(theCloud, l, rm); }), "resampleCloudWithOctreeAtLevel");
				m->add(fun([](GenericIndexedCloudPersist* theCloud, unsigned char l, CloudSamplingTools::RESAMPLING_CELL_METHOD rm, GenericProgressCallback* pcb) {return CloudSamplingTools::resampleCloudWithOctreeAtLevel(theCloud, l, rm, pcb); }), "resampleCloudWithOctreeAtLevel");


				m->add(fun(&CloudSamplingTools::resampleCloudWithOctree), "resampleCloudWithOctree");
				m->add(fun(&CloudSamplingTools::subsampleCloudWithOctreeAtLevel), "subsampleCloudWithOctreeAtLevel");
				m->add(fun(&CloudSamplingTools::subsampleCloudWithOctree), "subsampleCloudWithOctree");
				m->add(fun(&CloudSamplingTools::subsampleCloudRandomly), "subsampleCloudRandomly");
				m->add(fun(&CloudSamplingTools::resampleCloudSpatially), "resampleCloudSpatially");
				m->add(fun(&CloudSamplingTools::sorFilter), "sorFilter");
				m->add(fun(&CloudSamplingTools::noiseFilter), "noiseFilter");
				
				return m;
			}

			ModulePtr bs_ErrorFunction(ModulePtr m = std::make_shared<Module>())
			{
				m->add(fun(&ErrorFunction::erfc), "ErrorFunction_erfc");
				m->add(fun(&ErrorFunction::erf), "ErrorFunction_erf");
				return m;
			}

			ModulePtr bs_GeometricalAnalysisTools(ModulePtr m = std::make_shared<Module>())
			{
				m->add(fun(&GeometricalAnalysisTools::ComputeCharactersitic), "ComputeCharactersitic");
				m->add(fun([](GeometricalAnalysisTools::GeomCharacteristic a, int b, GenericIndexedCloudPersist* c, PointCoordinateType d, GenericProgressCallback* e) {return GeometricalAnalysisTools::ComputeCharactersitic(a, b, c, d, e); }), "ComputeCharactersitic");
				m->add(fun([](GeometricalAnalysisTools::GeomCharacteristic a, int b, GenericIndexedCloudPersist* c, PointCoordinateType d) {return GeometricalAnalysisTools::ComputeCharactersitic(a, b, c, d); }), "ComputeCharactersitic");
				m->add(fun(&GeometricalAnalysisTools::ComputeLocalDensityApprox), "ComputeLocalDensityApprox");
				m->add(fun([](GenericIndexedCloudPersist* a, GeometricalAnalysisTools::Density b, GenericProgressCallback* c) {return GeometricalAnalysisTools::ComputeLocalDensityApprox(a, b, c); }), "ComputeLocalDensityApprox");
				m->add(fun([](GenericIndexedCloudPersist* a, GeometricalAnalysisTools::Density b) {return GeometricalAnalysisTools::ComputeLocalDensityApprox(a, b); }), "ComputeLocalDensityApprox");
				m->add(fun(&GeometricalAnalysisTools::ComputeGravityCenter), "ComputeGravityCenter");
				m->add(fun(&GeometricalAnalysisTools::ComputeWeightedGravityCenter), "ComputeWeightedGravityCenter");
				m->add(fun(&GeometricalAnalysisTools::ComputeCrossCovarianceMatrix), "ComputeCrossCovarianceMatrix");
				m->add(fun(&GeometricalAnalysisTools::ComputeWeightedCrossCovarianceMatrix), "ComputeWeightedCrossCovarianceMatrix");
				m->add(fun([](GenericCloud* a, GenericCloud* b, const CCVector3& c, const CCVector3& d) {return GeometricalAnalysisTools::ComputeWeightedCrossCovarianceMatrix(a, b,c,d); }), "ComputeWeightedCrossCovarianceMatrix");
				m->add(fun(&GeometricalAnalysisTools::ComputeCovarianceMatrix), "ComputeCovarianceMatrix");
				m->add(fun([](GenericCloud* a) {return GeometricalAnalysisTools::ComputeCovarianceMatrix(a); }), "ComputeCovarianceMatrix");
				m->add(fun(&GeometricalAnalysisTools::FlagDuplicatePoints), "FlagDuplicatePoints");
				m->add(fun([](GenericIndexedCloudPersist* a, double b, GenericProgressCallback* c) {return GeometricalAnalysisTools::FlagDuplicatePoints(a, b, c); }), "FlagDuplicatePoints");
				m->add(fun([](GenericIndexedCloudPersist* a, double b) {return GeometricalAnalysisTools::FlagDuplicatePoints(a, b); }), "FlagDuplicatePoints");
				m->add(fun([](GenericIndexedCloudPersist* a) {return GeometricalAnalysisTools::FlagDuplicatePoints(a); }), "FlagDuplicatePoints");
				m->add(fun(&GeometricalAnalysisTools::DetectSphereRobust), "DetectSphereRobust");
				m->add(fun([](GenericIndexedCloudPersist* a, double b, CCVector3& c, PointCoordinateType& d, double& e, GenericProgressCallback* f, double g) {return GeometricalAnalysisTools::DetectSphereRobust(a, b, c, d, e, f, g); }), "DetectSphereRobust");
				m->add(fun([](GenericIndexedCloudPersist* a, double b, CCVector3& c, PointCoordinateType& d, double& e, GenericProgressCallback* f) {return GeometricalAnalysisTools::DetectSphereRobust(a, b, c, d, e, f); }), "DetectSphereRobust");
				m->add(fun([](GenericIndexedCloudPersist* a, double b, CCVector3& c, PointCoordinateType& d, double& e) {return GeometricalAnalysisTools::DetectSphereRobust(a, b, c, d, e); }), "DetectSphereRobust");
				m->add(fun(&GeometricalAnalysisTools::ComputeSphereFrom4), "ComputeSphereFrom4");


				return m;
			}

			ModulePtr bs_ManualSegmentationTools(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<ManualSegmentationTools>(), "ManualSegmentationTools");
				m->add(fun(static_cast<ReferenceCloud*(*)(GenericIndexedCloudPersist*, const CCLib::Polyline*, bool, const float* )> (&ManualSegmentationTools::segment)), "segment");
				m->add(fun(&ManualSegmentationTools::segmentReferenceCloud), "segmentReferenceCloud");
				m->add(fun(static_cast<ReferenceCloud*(*)(GenericIndexedCloudPersist*, ScalarType, ScalarType, bool)> (&ManualSegmentationTools::segment)), "segment");
				m->add(fun(static_cast<bool(*)(const CCVector2&, const GenericIndexedCloud*)> (&ManualSegmentationTools::isPointInsidePoly)), "isPointInsidePoly");
				m->add(fun(static_cast<bool(*)(const CCVector2&, const std::vector<CCVector2>&)> (&ManualSegmentationTools::isPointInsidePoly)), "isPointInsidePoly");
				m->add(fun(&ManualSegmentationTools::segmentMesh), "segmentMesh");
				m->add(fun(&ManualSegmentationTools::segmentMeshWithAAPlane), "segmentMeshWithAAPlane");
				m->add(fun(&ManualSegmentationTools::segmentMeshWithAABox), "segmentMeshWithAABox");
				
				m->add(user_type<ManualSegmentationTools::MeshCutterParams>(), "MeshCutterParams");
				m->add(constructor<ManualSegmentationTools::MeshCutterParams()>(), "MeshCutterParams");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::insideMesh), "insideMesh");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::outsideMesh), "outsideMesh");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::generateOutsideMesh), "generateOutsideMesh");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::epsilon), "epsilon");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::planeOrthoDim), "planeOrthoDim");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::planeCoord), "planeCoord");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::bbMin), "bbMin");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::bbMax), "bbMax");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::trackOrigIndexes), "trackOrigIndexes");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::origTriIndexesMapInside), "origTriIndexesMapInside");
				m->add(fun(&ManualSegmentationTools::MeshCutterParams::origTriIndexesMapOutside), "origTriIndexesMapOutside");

				return m;
			}


			ModulePtr bs_StatisticalTestingTools(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(fun(&StatisticalTestingTools::computeAdaptativeChi2Dist), "computeAdaptativeChi2Dist");
				m->add(fun([](const GenericDistribution* a, const GenericCloud* b, unsigned c, unsigned& d, bool e, const ScalarType* f, const ScalarType* g, unsigned* h) {return StatisticalTestingTools::computeAdaptativeChi2Dist(a, b, c, d, e, f, g, h); }), "computeAdaptativeChi2Dist");
				m->add(fun([](const GenericDistribution* a, const GenericCloud* b, unsigned c, unsigned& d, bool e, const ScalarType* f, const ScalarType* g) {return StatisticalTestingTools::computeAdaptativeChi2Dist(a, b, c, d, e, f, g); }), "computeAdaptativeChi2Dist");
				m->add(fun([](const GenericDistribution* a, const GenericCloud* b, unsigned c, unsigned& d, bool e, const ScalarType* f) {return StatisticalTestingTools::computeAdaptativeChi2Dist(a, b, c, d, e, f); }), "computeAdaptativeChi2Dist");
				m->add(fun([](const GenericDistribution* a, const GenericCloud* b, unsigned c, unsigned& d, bool e) {return StatisticalTestingTools::computeAdaptativeChi2Dist(a, b, c, d, e); }), "computeAdaptativeChi2Dist");
				m->add(fun([](const GenericDistribution* a, const GenericCloud* b, unsigned c, unsigned& d) {return StatisticalTestingTools::computeAdaptativeChi2Dist(a, b, c, d); }), "computeAdaptativeChi2Dist");

				m->add(fun(&StatisticalTestingTools::computeChi2Fractile), "computeChi2Fractile");
				m->add(fun(&StatisticalTestingTools::computeChi2Probability), "computeChi2Probability");
				m->add(fun(&StatisticalTestingTools::testCloudWithStatisticalModel), "testCloudWithStatisticalModel");
				m->add(fun([](const GenericDistribution* a, GenericIndexedCloudPersist* b, unsigned c, double d, GenericProgressCallback* e) {return StatisticalTestingTools::testCloudWithStatisticalModel(a, b, c, d, e); }), "testCloudWithStatisticalModel");
				m->add(fun([](const GenericDistribution* a, GenericIndexedCloudPersist* b, unsigned c, double d) {return StatisticalTestingTools::testCloudWithStatisticalModel(a, b, c, d); }), "testCloudWithStatisticalModel");

				return m;
			}


			ModulePtr bootstrap_static_functions(ModulePtr m = std::make_shared<Module>())
			{
				bs_DistanceComputationTools(m);
				bs_CCMiscTools(m);
				bs_AutoSegmentationTools(m);
				bs_CloudSamplingTools(m);
				bs_ErrorFunction(m);
				bs_GeometricalAnalysisTools(m);
				bs_ManualSegmentationTools(m);
				bs_StatisticalTestingTools(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_STATIC_FUNCTIONS_HPP