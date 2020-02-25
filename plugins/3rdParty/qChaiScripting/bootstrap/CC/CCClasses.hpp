#ifndef CHAISCRIPTING_BOOTSTRAP_CC_CLASSES_HPP
#define CHAISCRIPTING_BOOTSTRAP_CC_CLASSES_HPP

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

#include <CCTypes.h>
#include <CCGeom.h>
#include <SquareMatrix.h>
#include <BoundingBox.h>
#include <GenericCloud.h>
#include <GenericTriangle.h>
#include <PointCloud.h>
#include <GenericIndexedCloudPersist.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <GenericMesh.h>
#include <GenericIndexedMesh.h>
#include <ChamferDistanceTransform.h>
#include <Grid3D.h>
#include <ConjugateGradient.h>
#include <Delaunay2dMesh.h>
#include <DgmOctree.h>
#include <DgmOctreeReferenceCloud.h>
#include <FastMarching.h>
#include <FastMarchingForPropagation.h>
#include <Garbage.h>
#include <GenericDistribution.h>
#include <KdTree.h>
#include <LocalModel.h>
#include <ManualSegmentationTools.h>
#include <NormalDistribution.h>
#include <WeibullDistribution.h>
#include <TrueKdTree.h>
#include <DistanceComputationTools.h>
#include <SimpleTriangle.h>
#include <SimpleMesh.h>
#include <RayAndBox.h>
#include <Jacobi.h>

namespace chaiscript
{
	namespace cloudCompare
	{
		namespace CC
		{
			template<typename T>
			ModulePtr bs_Vector2Tpl(const std::string &shortCutName, ModulePtr m = std::make_shared<Module>())
			{

				chaiscript::utility::add_class<Vector2Tpl<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<Vector2Tpl<T>()>(),
						chaiscript::constructor<Vector2Tpl<T>(T)>(),
						chaiscript::constructor<Vector2Tpl<T>(T, T)>()
					},
					{
						{ fun(&Vector2Tpl<T>::x), "x" },
						{ fun(&Vector2Tpl<T>::y), "y" },
						{ fun(&Vector2Tpl<T>::u), "u" },
						{ fun(&Vector2Tpl<T>::norm2), "norm2" },
						{ fun(&Vector2Tpl<T>::norm), "norm" },
						{ fun(&Vector2Tpl<T>::normalize), "normalize" },
						{ fun(&Vector2Tpl<T>::dot), "dot" },
						{ fun(&Vector2Tpl<T>::cross), "cross" },
						{ fun(static_cast<Vector2Tpl<T> & (Vector2Tpl<T>::*)(const Vector2Tpl<T>&)>(&Vector2Tpl<T>::operator=)), "=" },
						{ fun(static_cast<Vector2Tpl<T> & (Vector2Tpl<T>::*)(const Vector2Tpl<T>&)>(&Vector2Tpl<T>::operator+=)), "+=" },
						{ fun(static_cast<Vector2Tpl<T> & (Vector2Tpl<T>::*)(const Vector2Tpl<T>&)>(&Vector2Tpl<T>::operator-=)), "-=" },
						{ fun(static_cast<Vector2Tpl<T> & (Vector2Tpl<T>::*)(T)>(&Vector2Tpl<T>::operator*=)), "*=" },
						{ fun(static_cast<Vector2Tpl<T> & (Vector2Tpl<T>::*)(T)>(&Vector2Tpl<T>::operator/=)), "/=" },
						{ fun(static_cast<Vector2Tpl<T>(Vector2Tpl<T>::*)(const Vector2Tpl<T>&)const>(&Vector2Tpl<T>::operator+)), "+" },
						{ fun(static_cast<Vector2Tpl<T>(Vector2Tpl<T>::*)(const Vector2Tpl<T>&)const>(&Vector2Tpl<T>::operator-)), "-" },
						{ fun(static_cast<Vector2Tpl<T>(Vector2Tpl<T>::*)(T)const>(&Vector2Tpl<T>::operator*)), "*" },
						{ fun(static_cast<Vector2Tpl<T>(Vector2Tpl<T>::*)(T)const>(&Vector2Tpl<T>::operator/)), "/" },
						{ fun(static_cast<T & (Vector2Tpl<T>::*)(unsigned)>(&Vector2Tpl<T>::operator[])), "[]" },
						{ fun(static_cast<const T & (Vector2Tpl<T>::*)(unsigned)const>(&Vector2Tpl<T>::operator[])), "[]" },
					}
					);
				chaiscript::bootstrap::array<T[2]>("u_Array", m);
				return m;
			}

			template<typename T>
			ModulePtr bs_Tuple3Tpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{

				chaiscript::utility::add_class<Tuple3Tpl<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<Tuple3Tpl<T>()>(),
						chaiscript::constructor<Tuple3Tpl<T>(T, T, T)>(),
						chaiscript::constructor<Tuple3Tpl<T>(const T p[])>(),
					},
					{
						{ fun(&Tuple3Tpl<T>::x), "x" },
						{ fun(&Tuple3Tpl<T>::y), "y" },
						{ fun(&Tuple3Tpl<T>::z), "z" },
						{ fun(&Tuple3Tpl<T>::u), "u" },
						//{ fun(static_cast<Tuple3Tpl<T> (Tuple3Tpl<T>::*)()const>(&Tuple3Tpl<T>::operator-() )), "-" }, // Inverse Operator
						{ fun(static_cast<Tuple3Tpl<T> & (Tuple3Tpl<T>::*)(const Tuple3Tpl<T>&)>(&Tuple3Tpl<T>::operator+=)), "+=" },
						{ fun(static_cast<Tuple3Tpl<T> & (Tuple3Tpl<T>::*)(const Tuple3Tpl<T>&)>(&Tuple3Tpl<T>::operator-=)), "-=" },
						{ fun(static_cast<Tuple3Tpl<T> & (Tuple3Tpl<T>::*)(T)>(&Tuple3Tpl<T>::operator*=)), "*=" },
						{ fun(static_cast<Tuple3Tpl<T> & (Tuple3Tpl<T>::*)(T)>(&Tuple3Tpl<T>::operator/=)), "/=" },
						{ fun(static_cast<Tuple3Tpl<T>(Tuple3Tpl<T>::*)(const Tuple3Tpl<T>&)const>(&Tuple3Tpl<T>::operator+)), "+" },
						{ fun(static_cast<Tuple3Tpl<T>(Tuple3Tpl<T>::*)(const Tuple3Tpl<T>&)const>(&Tuple3Tpl<T>::operator-)), "-" },
						{ fun(static_cast<Tuple3Tpl<T>(Tuple3Tpl<T>::*)(T)const>(&Tuple3Tpl<T>::operator*)), "*" },
						{ fun(static_cast<Tuple3Tpl<T>(Tuple3Tpl<T>::*)(T)const>(&Tuple3Tpl<T>::operator/)), "/" },
					}
					);
				chaiscript::bootstrap::array<T[3]>("u_Array", m);
				return m;
			}


			template<typename T>
			ModulePtr bs_Vector3Tpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{

				chaiscript::utility::add_class<Vector3Tpl<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<Vector3Tpl<T>()>(),
						chaiscript::constructor<Vector3Tpl<T>(T, T, T)>(),
						chaiscript::constructor<Vector3Tpl<T>(const T p[])>(),
						chaiscript::constructor<Vector3Tpl<T>(const Vector2Tpl<T> &, T)>()
					},
					{
						{ fun(&Vector3Tpl<T>::x), "x" },
						{ fun(&Vector3Tpl<T>::y), "y" },
						{ fun(&Vector3Tpl<T>::z), "z" },
						{ fun(&Vector3Tpl<T>::u), "u" },
						{ fun(&Vector3Tpl<T>::norm2), "norm2" },
						{ fun(&Vector3Tpl<T>::vnorm2), "vnorm2" },
						{ fun(&Vector3Tpl<T>::norm2d), "norm2d" },
						//{ fun(&Vector3Tpl<T>::vnorm2d), "vnorm2d" }, Doesn't exist
						{ fun(&Vector3Tpl<T>::norm), "norm" },
						{ fun(&Vector3Tpl<T>::vnorm), "vnorm" },
						{ fun(&Vector3Tpl<T>::normd), "normd" },
						//{ fun(&Vector3Tpl<T>::vnormd), "vnormd" }, Doesn't exist
						{ fun(&Vector3Tpl<T>::vdistance), "vdistance" },
						{ fun(&Vector3Tpl<T>::vdistance2), "vdistance2" },
						{ fun(&Vector3Tpl<T>::normalize), "normalize" },
						{ fun(&Vector3Tpl<T>::orthogonal), "orthogonal" },
						{ fun(&Vector3Tpl<T>::vorthogonal), "vorthogonal" },
						{ fun(&Vector3Tpl<T>::dot), "dot" },
						{ fun(&Vector3Tpl<T>::vdot), "vdot" },
						{ fun(&Vector3Tpl<T>::vdotd), "vdotd" },
						{ fun(&Vector3Tpl<T>::cross), "cross" },
						{ fun(&Vector3Tpl<T>::vcopy), "vcopy" },
						{ fun(&Vector3Tpl<T>::vadd), "vadd" },
						{ fun(&Vector3Tpl<T>::vsubstract), "vsubstract" },
						{ fun(&Vector3Tpl<T>::vnormalize), "vnormalize" },
						{ fun(&Vector3Tpl<T>::vcross), "vcross" },
						{ fun(&Vector3Tpl<T>::angle_rad), "angle_rad" },
						{ fun(&Vector3Tpl<T>::vangle_rad), "vangle_rad" },
						//{ fun(static_cast<void(const T p[], T s, T r[])>(&Vector3Tpl<T>::vdivide)), "vdivide" },
						{ fun(static_cast<Vector3Tpl<T> & (Vector3Tpl<T>::*)(const Vector3Tpl<T>&)>(&Vector3Tpl<T>::operator=)), "=" },
						//{ fun(static_cast<Vector3Tpl<T> & (Vector3Tpl<T>::*)(const Vector3Tpl<T>&)>(&Vector3Tpl<T>::operator-())), "-" },
						{ fun(static_cast<Vector3Tpl<T> & (Vector3Tpl<T>::*)(const Vector3Tpl<T>&)>(&Vector3Tpl<T>::operator+=)), "+=" },
						{ fun(static_cast<Vector3Tpl<T> & (Vector3Tpl<T>::*)(const Vector3Tpl<T>&)>(&Vector3Tpl<T>::operator-=)), "-=" },
						{ fun(static_cast<Vector3Tpl<T> & (Vector3Tpl<T>::*)(T)>(&Vector3Tpl<T>::operator*=)), "*=" },
						{ fun(static_cast<Vector3Tpl<T> & (Vector3Tpl<T>::*)(T)>(&Vector3Tpl<T>::operator/=)), "/=" },
						{ fun(static_cast<Vector3Tpl<T>(Vector3Tpl<T>::*)(const Vector3Tpl<T>&)const>(&Vector3Tpl<T>::operator+)), "+" },
						{ fun(static_cast<Vector3Tpl<T>(Vector3Tpl<T>::*)(const Vector3Tpl<T>&)const>(&Vector3Tpl<T>::operator-)), "-" },
						{ fun(static_cast<Vector3Tpl<T>(Vector3Tpl<T>::*)(T)const>(&Vector3Tpl<T>::operator*)), "*" },
						{ fun(static_cast<Vector3Tpl<T>(Vector3Tpl<T>::*)(T)const>(&Vector3Tpl<T>::operator/)), "/" },
						{ fun(static_cast<Vector3Tpl<T>(Vector3Tpl<T>::*)(const Vector3Tpl<T>&)const>(&Vector3Tpl<T>::operator*)), "*" },
						//{ fun(static_cast<T & (Vector3Tpl<T>::*)(const Vector3Tpl<T>&)>(&Vector3Tpl<T>::operator&&)), "&&" },
						{ fun(static_cast<T & (Vector3Tpl<T>::*)(unsigned)>(&Vector3Tpl<T>::operator[])), "[]" },
						{ fun(static_cast<const T & (Vector3Tpl<T>::*)(unsigned)const>(&Vector3Tpl<T>::operator[])), "[]" },
					}
					);
				chaiscript::bootstrap::array<T[3]>("u_Array", m);
				return m;
			}

			template<typename T>
			ModulePtr bs_Tuple4Tpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{

				chaiscript::utility::add_class<Tuple4Tpl<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<Tuple4Tpl<T>()>(),
						chaiscript::constructor<Tuple4Tpl<T>(T, T, T, T)>(),
						chaiscript::constructor<Tuple4Tpl<T>(const T p[])>(),
					},
					{
						{ fun(&Tuple4Tpl<T>::x), "x" },
						{ fun(&Tuple4Tpl<T>::y), "y" },
						{ fun(&Tuple4Tpl<T>::z), "z" },
						{ fun(&Tuple4Tpl<T>::z), "w" },
						{ fun(&Tuple4Tpl<T>::u), "u" },
						{ fun(static_cast<Tuple4Tpl<T> & (Tuple4Tpl<T>::*)(const Tuple4Tpl<T>&)>(&Tuple4Tpl<T>::operator=)), "=" },
						//{ fun(static_cast<Tuple4Tpl<T> & (Tuple4Tpl<T>::*)(const Tuple4Tpl<T>&)>(&Tuple4Tpl<T>::operator-())), "-" },
						{ fun(static_cast<Tuple4Tpl<T> & (Tuple4Tpl<T>::*)(const Tuple4Tpl<T>&)>(&Tuple4Tpl<T>::operator+=)), "+=" },
						{ fun(static_cast<Tuple4Tpl<T> & (Tuple4Tpl<T>::*)(const Tuple4Tpl<T>&)>(&Tuple4Tpl<T>::operator-=)), "-=" },
						{ fun(static_cast<Tuple4Tpl<T> & (Tuple4Tpl<T>::*)(T)>(&Tuple4Tpl<T>::operator*=)), "*=" },
						{ fun(static_cast<Tuple4Tpl<T> & (Tuple4Tpl<T>::*)(T)>(&Tuple4Tpl<T>::operator/=)), "/=" },
						{ fun(static_cast<Tuple4Tpl<T>(Tuple4Tpl<T>::*)(const Tuple4Tpl<T>&)const>(&Tuple4Tpl<T>::operator+)), "+" },
						{ fun(static_cast<Tuple4Tpl<T>(Tuple4Tpl<T>::*)(const Tuple4Tpl<T>&)const>(&Tuple4Tpl<T>::operator-)), "-" },
						{ fun(static_cast<Tuple4Tpl<T>(Tuple4Tpl<T>::*)(T)const>(&Tuple4Tpl<T>::operator*)), "*" },
						{ fun(static_cast<Tuple4Tpl<T>(Tuple4Tpl<T>::*)(T)const>(&Tuple4Tpl<T>::operator/)), "/" },
					}
					);
				chaiscript::bootstrap::array<T[4]>("u_Array", m);
				return m;
			}



			template<typename T>
			ModulePtr bs_SquareMatrixTpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				chaiscript::utility::add_class<SquareMatrixTpl<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<SquareMatrixTpl<T>()>(),
						chaiscript::constructor<SquareMatrixTpl<T>(unsigned)>(),
						chaiscript::constructor<SquareMatrixTpl<T>(const SquareMatrixTpl<T>&)>(),
						chaiscript::constructor<SquareMatrixTpl<T>(const float p[], bool)>(),
						chaiscript::constructor<SquareMatrixTpl<T>(const double p[], bool)>()
					},
					{
						{ fun(&SquareMatrixTpl<T>::size), "size" },
						{ fun(&SquareMatrixTpl<T>::isValid), "isValid" },
						{ fun(&SquareMatrixTpl<T>::invalidate), "invalidate" },
						{ fun(&SquareMatrixTpl<T>::m_values), "m_values" },
						{ fun(&SquareMatrixTpl<T>::row), "row" },
						{ fun(&SquareMatrixTpl<T>::setValue), "setValue" },
						{ fun(&SquareMatrixTpl<T>::getValue), "getValue" },
						{ fun(static_cast<SquareMatrixTpl<T> & (SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)>(&SquareMatrixTpl<T>::operator=)), "=" },
						{ fun(static_cast<SquareMatrixTpl<T>(SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)const>(&SquareMatrixTpl<T>::operator+)), "+" },
						{ fun(static_cast<const SquareMatrixTpl<T> & (SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)>(&SquareMatrixTpl<T>::operator+=)), "+=" },
						{ fun(static_cast<SquareMatrixTpl<T>(SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)const>(&SquareMatrixTpl<T>::operator-)), "-" },
						{ fun(static_cast<const SquareMatrixTpl<T> & (SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)>(&SquareMatrixTpl<T>::operator-=)), "-=" },
						{ fun(static_cast<SquareMatrixTpl<T>(SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)const>(&SquareMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<CCVector3(SquareMatrixTpl<T>::*)(const CCVector3&)const>(&SquareMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<CCVector3d(SquareMatrixTpl<T>::*)(const CCVector3d&)const>(&SquareMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<const SquareMatrixTpl<T> & (SquareMatrixTpl<T>::*)(const SquareMatrixTpl<T>&)>(&SquareMatrixTpl<T>::operator*=)), "*=" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(const float in[], float out[])const>(&SquareMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(const double in[], double out[])const>(&SquareMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(const double in[], float out[])const>(&SquareMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(const float in[], double out[])const>(&SquareMatrixTpl<T>::apply)), "apply" },
						{ fun(&SquareMatrixTpl<T>::transpose), "transpose" },
						{ fun(&SquareMatrixTpl<T>::transposed), "transposed" },
						{ fun(&SquareMatrixTpl<T>::clear), "clear" },
						{ fun(&SquareMatrixTpl<T>::inv), "inv" },
						{ fun(&SquareMatrixTpl<T>::print), "print" },
						{ fun(&SquareMatrixTpl<T>::toIdentity), "toIdentity" },
						{ fun(&SquareMatrixTpl<T>::scale), "scale" },
						{ fun(&SquareMatrixTpl<T>::trace), "trace" },
						{ fun(&SquareMatrixTpl<T>::computeDet), "computeDet" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(const float q[])>(&SquareMatrixTpl<T>::initFromQuaternion)), "initFromQuaternion" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(const double q[])>(&SquareMatrixTpl<T>::initFromQuaternion)), "initFromQuaternion" },
						{ fun(&SquareMatrixTpl<T>::toQuaternion), "toQuaternion" },
						{ fun(&SquareMatrixTpl<T>::deltaDeterminant), "deltaDeterminant" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(float M16f[])const>(&SquareMatrixTpl<T>::toGlMatrix)), "toGlMatrix" },
						{ fun(static_cast<void(SquareMatrixTpl<T>::*)(double M16d[])const>(&SquareMatrixTpl<T>::toGlMatrix)), "toGlMatrix" },


					}
					);
				return m;
			}

			template<typename T>
			ModulePtr bs_PointCloudTpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				chaiscript::utility::add_class<PointCloudTpl<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<PointCloudTpl<T>()>()						
					},
					{
						{ fun(&PointCloudTpl<T>::size), "size" },
						{ fun(&PointCloudTpl<T>::forEach), "forEach" },
						{ fun(&PointCloudTpl<T>::getBoundingBox), "getBoundingBox" },
						{ fun(&PointCloudTpl<T>::enableScalarField), "enableScalarField" },
						{ fun(&PointCloudTpl<T>::isScalarFieldEnabled), "isScalarFieldEnabled" },
						{ fun(&PointCloudTpl<T>::setPointScalarValue), "setPointScalarValue" },
						{ fun(&PointCloudTpl<T>::getPointScalarValue), "getPointScalarValue" },
						

					}
					);
				
				return m;
			}

			template <typename T>
			ModulePtr bs_Grid3D(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				using GridElement = T;
				m->add(user_type<Grid3D<T>>(), shortCutName);
				m->add(constructor<Grid3D<T>()>(), shortCutName);

				m->add(fun(&Grid3D<T>::size), "size");
				m->add(fun(&Grid3D<T>::isInitialized), "isInitialized");
				m->add(fun(&Grid3D<T>::init), "init");
				m->add(user_type<typename Grid3D<T>::CellToTest>(), "CellToTest");
				m->add(fun(&Grid3D<T>::CellToTest::pos), "pos");
				m->add(fun(&Grid3D<T>::CellToTest::cellSize), "cellSize");
				m->add(fun(static_cast<bool(Grid3D<T>::*)(GenericIndexedMesh*, PointCoordinateType, const CCVector3&, GridElement, GenericProgressCallback*)>(&Grid3D<T>::intersecthWith)), "intersecthWith");
				m->add(fun(static_cast<bool(Grid3D<T>::*)(GenericCloud*, PointCoordinateType, const CCVector3&, GridElement, GenericProgressCallback*)>(&Grid3D<T>::intersecthWith)), "intersecthWith");
				m->add(fun(static_cast<void(Grid3D<T>::*)(int, int, int, GridElement)>(&Grid3D<T>::setValue)), "setValue");
				m->add(fun(static_cast<void(Grid3D<T>::*)(Tuple3i&, GridElement)>(&Grid3D<T>::setValue)), "setValue");
				m->add(fun(static_cast<const GridElement & (Grid3D<T>::*)(int, int, int)const>(&Grid3D<T>::getValue)), "getValue");
				m->add(fun(static_cast<GridElement & (Grid3D<T>::*)(int, int, int)>(&Grid3D<T>::getValue)), "getValue");
				m->add(fun(static_cast<const GridElement & (Grid3D<T>::*)(Tuple3i&)const>(&Grid3D<T>::getValue)), "getValue");
				m->add(fun(static_cast<GridElement & (Grid3D<T>::*)(Tuple3i&)>(&Grid3D<T>::getValue)), "getValue");
				m->add(fun(static_cast<const GridElement * (Grid3D<T>::*)()const>(&Grid3D<T>::data)), "data");
				m->add(fun(static_cast<GridElement * (Grid3D<T>::*)()>(&Grid3D<T>::data)), "data");
				m->add(fun(&Grid3D<T>::innerCellCount), "innerCellCount");
				m->add(fun(&Grid3D<T>::totalCellCount), "totalCellCount");



				m->add(base_class<Grid3D<unsigned short>, ChamferDistanceTransform>());

				return m;
			}

			template<typename T>
			ModulePtr bs_Ray(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				chaiscript::utility::add_class<Ray<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<Ray<T>(const Vector3Tpl<T>&, const Vector3Tpl<T>&)>(),
					},
					{
						{ fun(&Ray<T>::radialSquareDistance), "radialSquareDistance" },
						{ fun(&Ray<T>::squareDistanceToOrigin), "squareDistanceToOrigin" },
						{ fun(&Ray<T>::squareDistances), "squareDistances" },
						{ fun(&Ray<T>::dir), "dir" },
						{ fun(&Ray<T>::origin), "origin" },
						{ fun(&Ray<T>::invDir), "invDir" },
						{ fun(&Ray<T>::sign), "sign" },						
					}
					);
				return m;
			}

			template<typename T>
			ModulePtr bs_AABB(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				chaiscript::utility::add_class<AABB<T>>(*m,
					shortCutName,
					{
						chaiscript::constructor<AABB<T>(const Vector3Tpl<T>&, const Vector3Tpl<T>&)>(),
					},
					{
						{ fun(&AABB<T>::intersects), "intersects" },
						{ fun(&AABB<T>::corners), "corners" },
					}
					);
				chaiscript::bootstrap::array<Vector3Tpl<T>[2]>("corners_Array", m);
				return m;
			}

			template<typename T>
			ModulePtr bs_Jacobi(ModulePtr m = std::make_shared<Module>())
			{
				using Base = Jacobi<T>;
				m->add(fun(&Base::ComputeEigenValuesAndVectors2), "ComputeEigenValuesAndVectors2");
				m->add(fun(&Base::ComputeEigenValuesAndVectors), "ComputeEigenValuesAndVectors");
				m->add(fun(&Base::SortEigenValuesAndVectors), "SortEigenValuesAndVectors");
				m->add(fun(&Base::GetEigenVector), "GetEigenVector");
				m->add(fun(&Base::GetMaxEigenValueAndVector), "GetMaxEigenValueAndVector");
				m->add(fun(&Base::GetMinEigenValueAndVector), "GetMinEigenValueAndVector");
				return m;
			}


			ModulePtr bs_PointCloud(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<CCLib::PointCloud>(*m,
					"PointCloud",
					{
						chaiscript::constructor<CCLib::PointCloud()>()
					},
					{

						{ fun(&CCLib::PointCloud::size), "size" },
						{ fun(&CCLib::PointCloud::forEach), "forEach" },
						{ fun(&CCLib::PointCloud::getBoundingBox), "getBoundingBox" },
						{ fun(&CCLib::PointCloud::testVisibility), "testVisibility" },
						{ fun(&CCLib::PointCloud::placeIteratorAtBeginning), "placeIteratorAtBeginning" },
						{ fun(&CCLib::PointCloud::getNextPoint), "getNextPoint" },
						{ fun(&CCLib::PointCloud::enableScalarField), "enableScalarField" },
						{ fun(&CCLib::PointCloud::isScalarFieldEnabled), "isScalarFieldEnabled" },
						{ fun(&CCLib::PointCloud::setPointScalarValue), "setPointScalarValue" },
						{ fun(&CCLib::PointCloud::getPointScalarValue), "getPointScalarValue" },
						{ fun(static_cast<const CCVector3*(CCLib::PointCloud::*)(unsigned index)const>(&CCLib::PointCloud::getPoint)), "getPoint" },
						{ fun(static_cast<void(CCLib::PointCloud::*)(unsigned index, CCVector3 &)const>(&CCLib::PointCloud::getPoint)), "getPoint" },
						{ fun(&CCLib::PointCloud::getPointPersistentPtr), "getPointPersistentPtr" }, //GenericIndexedCloudPersist
						{ fun(&CCLib::PointCloud::resize), "resize" },
						{ fun(&CCLib::PointCloud::reserve), "reserve" },
						{ fun(&CCLib::PointCloud::reset), "reset" },
						{ fun(&CCLib::PointCloud::addPoint), "addPoint" },
						{ fun(&CCLib::PointCloud::invalidateBoundingBox), "invalidateBoundingBox" },
						{ fun(&CCLib::PointCloud::getNumberOfScalarFields), "getNumberOfScalarFields" },
						{ fun(&CCLib::PointCloud::getScalarField), "getScalarField" },
						{ fun(&CCLib::PointCloud::getScalarFieldName), "getScalarFieldName" },
						{ fun(&CCLib::PointCloud::getScalarFieldIndexByName), "getScalarFieldIndexByName" },
						{ fun(&CCLib::PointCloud::getCurrentInScalarField), "getCurrentInScalarField" },
						{ fun(&CCLib::PointCloud::getCurrentOutScalarField), "getCurrentOutScalarField" },
						{ fun(&CCLib::PointCloud::setCurrentInScalarField), "setCurrentInScalarField" },
						{ fun(&CCLib::PointCloud::getCurrentInScalarFieldIndex), "getCurrentInScalarFieldIndex" },
						{ fun(&CCLib::PointCloud::setCurrentOutScalarField), "setCurrentOutScalarField" },
						{ fun(&CCLib::PointCloud::getCurrentOutScalarFieldIndex), "getCurrentOutScalarFieldIndex" },
						{ fun(&CCLib::PointCloud::setCurrentScalarField), "setCurrentScalarField" },
						{ fun(&CCLib::PointCloud::addScalarField), "addScalarField" },
						{ fun(&CCLib::PointCloud::renameScalarField), "renameScalarField" },
						{ fun(&CCLib::PointCloud::deleteScalarField), "deleteScalarField" },
						{ fun(&CCLib::PointCloud::deleteAllScalarFields), "deleteAllScalarFields" },
						{ fun(&CCLib::PointCloud::capacity), "capacity" }
					}
				);
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloud, CCLib::PointCloud>());
				m->add(chaiscript::base_class< CCLib::GenericCloud, CCLib::PointCloud>());
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloudPersist, CCLib::PointCloud>());
				m->add(chaiscript::base_class<CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>, CCLib::PointCloud>());
				return m;
			}

			ModulePtr bs_ReferenceCloud(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<CCLib::ReferenceCloud>(*m,
					"ReferenceCloud",
					{
						chaiscript::constructor<CCLib::ReferenceCloud(CCLib::GenericIndexedCloudPersist*)>(),
						chaiscript::constructor<CCLib::ReferenceCloud(const CCLib::ReferenceCloud&)>()
					},
					{

						{ fun(&CCLib::ReferenceCloud::size), "size" },
						{ fun(&CCLib::ReferenceCloud::forEach), "forEach" },
						{ fun(&CCLib::ReferenceCloud::getBoundingBox), "getBoundingBox" },
						{ fun(&CCLib::ReferenceCloud::testVisibility), "testVisibility" },
						{ fun(&CCLib::ReferenceCloud::placeIteratorAtBeginning), "placeIteratorAtBeginning" },
						{ fun(&CCLib::ReferenceCloud::getNextPoint), "getNextPoint" },
						{ fun(&CCLib::ReferenceCloud::enableScalarField), "enableScalarField" },
						{ fun(&CCLib::ReferenceCloud::isScalarFieldEnabled), "isScalarFieldEnabled" },
						{ fun(&CCLib::ReferenceCloud::setPointScalarValue), "setPointScalarValue" },
						{ fun(&CCLib::ReferenceCloud::getPointScalarValue), "getPointScalarValue" },
						{ fun(static_cast<const CCVector3 * (CCLib::ReferenceCloud::*)(unsigned index)const>(&CCLib::ReferenceCloud::getPoint)), "getPoint" },
						{ fun(static_cast<void(CCLib::ReferenceCloud::*)(unsigned index, CCVector3&)const>(&CCLib::ReferenceCloud::getPoint)), "getPoint" },
						{ fun(&CCLib::ReferenceCloud::getPointPersistentPtr), "getPointPersistentPtr" }, //GenericIndexedCloudPersist
						{ fun(&CCLib::ReferenceCloud::resize), "resize" },
						{ fun(&CCLib::ReferenceCloud::reserve), "reserve" },
						{ fun(&CCLib::ReferenceCloud::invalidateBoundingBox), "invalidateBoundingBox" },						
						{ fun(&CCLib::ReferenceCloud::getPointGlobalIndex), "getPointGlobalIndex" },
						{ fun(&CCLib::ReferenceCloud::getCurrentPointCoordinates), "getCurrentPointCoordinates" },
						{ fun(&CCLib::ReferenceCloud::getCurrentPointGlobalIndex), "getCurrentPointGlobalIndex" },
						{ fun(&CCLib::ReferenceCloud::getCurrentPointScalarValue), "getCurrentPointScalarValue" },
						{ fun(&CCLib::ReferenceCloud::setCurrentPointScalarValue), "setCurrentPointScalarValue" },
						{ fun(&CCLib::ReferenceCloud::forwardIterator), "forwardIterator" },
						{ fun(&CCLib::ReferenceCloud::clear), "clear" },
						{ fun(static_cast<bool(CCLib::ReferenceCloud::*)(unsigned)>(&CCLib::ReferenceCloud::addPointIndex)), "addPointIndex" },
						{ fun(static_cast<bool(CCLib::ReferenceCloud::*)(unsigned,unsigned)>(&CCLib::ReferenceCloud::addPointIndex)), "addPointIndex" },
						{ fun(&CCLib::ReferenceCloud::setPointIndex), "setPointIndex" },
						{ fun(&CCLib::ReferenceCloud::capacity), "capacity" },
						{ fun(&CCLib::ReferenceCloud::swap), "swap" },
						{ fun(&CCLib::ReferenceCloud::removeCurrentPointGlobalIndex), "removeCurrentPointGlobalIndex" },
						{ fun(&CCLib::ReferenceCloud::removePointGlobalIndex), "removePointGlobalIndex" },
						{ fun(static_cast<CCLib::GenericIndexedCloudPersist*(CCLib::ReferenceCloud::*)()>(&CCLib::ReferenceCloud::getAssociatedCloud)), "getAssociatedCloud" },
						{ fun(static_cast<const CCLib::GenericIndexedCloudPersist*(CCLib::ReferenceCloud::*)()const>(&CCLib::ReferenceCloud::getAssociatedCloud)), "getAssociatedCloud" },
						{ fun(&CCLib::ReferenceCloud::setAssociatedCloud), "setAssociatedCloud" },
						{ fun(&CCLib::ReferenceCloud::add), "add" },
					}
					);
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloud, CCLib::ReferenceCloud>());
				m->add(chaiscript::base_class< CCLib::GenericCloud, CCLib::ReferenceCloud>());
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloudPersist, CCLib::ReferenceCloud>());
				return m;
			}

			ModulePtr bs_Polyline(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<CCLib::Polyline>(), "Polyline");
				m->add(constructor<CCLib::Polyline(GenericIndexedCloudPersist*)>(), "Polyline");

				m->add(fun(&CCLib::Polyline::isClosed), "isClosed");
				m->add(fun(&CCLib::Polyline::setClosed), "setClosed");
				m->add(fun(&CCLib::Polyline::clear), "clear");

				m->add(chaiscript::base_class< ReferenceCloud, CCLib::Polyline>());
				m->add(chaiscript::base_class< GenericIndexedCloud, CCLib::Polyline>());
				m->add(chaiscript::base_class< GenericCloud, CCLib::Polyline>());
				m->add(chaiscript::base_class< GenericIndexedCloudPersist, CCLib::Polyline>());
				return m;
			}

			ModulePtr bs_ScalarField(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<CCLib::ScalarField>(), "ScalarField");
				//m->add(chaiscript::constructor<CCLib::ScalarField(const char*)>());
				//m->add(chaiscript::constructor<CCLib::ScalarField(const CCLib::ScalarField&)>());
				m->add(fun(&CCLib::ScalarField::setName), "setName");
				m->add(fun(&CCLib::ScalarField::getName), "getName");
				m->add(fun(&CCLib::ScalarField::NaN), "NaN");
				m->add(fun(&CCLib::ScalarField::computeMeanAndVariance), "computeMeanAndVariance");
				m->add(fun(&CCLib::ScalarField::computeMinAndMax), "computeMinAndMax");
				m->add(fun(&CCLib::ScalarField::ValidValue), "ValidValue");
				m->add(fun(&CCLib::ScalarField::flagValueAsInvalid), "flagValueAsInvalid");
				m->add(fun(&CCLib::ScalarField::getMin), "getMin");
				m->add(fun(&CCLib::ScalarField::getMax), "getMax");
				m->add(fun(&CCLib::ScalarField::fill), "fill");
				m->add(fun(&CCLib::ScalarField::reserveSafe), "reserveSafe");
				m->add(fun(&CCLib::ScalarField::resizeSafe), "resizeSafe");
				m->add(fun(static_cast<ScalarType & (CCLib::ScalarField::*)(std::size_t)>(&CCLib::ScalarField::getValue)), "getValue");
				m->add(fun(static_cast<const ScalarType & (CCLib::ScalarField::*)(std::size_t)const>(&CCLib::ScalarField::getValue)), "getValue");
				m->add(fun(&CCLib::ScalarField::setValue), "setValue");
				m->add(fun(&CCLib::ScalarField::addElement), "addElement");
				m->add(fun(&CCLib::ScalarField::currentSize), "currentSize");
				m->add(fun(&CCLib::ScalarField::swap), "swap");
				m->add(chaiscript::base_class<std::vector<ScalarType>, CCLib::ScalarField>());
				m->add(chaiscript::base_class<CCShareable, CCLib::ScalarField>());

				return m;
			}

			ModulePtr bs_boundingBox(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<CCLib::BoundingBox>(*m,
					"BoundingBox",
					{
						constructor<CCLib::BoundingBox()>(),
						constructor<CCLib::BoundingBox(const CCVector3&, const CCVector3&)>()
					},
					{
						{ fun(static_cast<CCLib::BoundingBox(CCLib::BoundingBox::*)(const CCLib::BoundingBox&)const>(&CCLib::BoundingBox::operator+)), "+" },
						{ fun(static_cast<const CCLib::BoundingBox & (CCLib::BoundingBox::*)(const CCLib::BoundingBox&)>(&CCLib::BoundingBox::operator+=)), "+=" },
						{ fun(static_cast<const CCLib::BoundingBox & (CCLib::BoundingBox::*)(const CCVector3&)>(&CCLib::BoundingBox::operator+=)), "+=" },
						{ fun(static_cast<const CCLib::BoundingBox & (CCLib::BoundingBox::*)(const CCVector3&)>(&CCLib::BoundingBox::operator-=)), "-=" },
						{ fun(static_cast<const CCLib::BoundingBox & (CCLib::BoundingBox::*)(PointCoordinateType)>(&CCLib::BoundingBox::operator*=)), "*=" },
						//{ fun(static_cast<const CCLib::BoundingBox & (CCLib::BoundingBox::*)(const SquareMatrix&)>(&CCLib::BoundingBox::operator*=)), "*=" },
						{ fun(&CCLib::BoundingBox::clear), "clear" },
						{ fun(&CCLib::BoundingBox::add), "add" },
						{ fun(static_cast<const CCVector3 & (CCLib::BoundingBox::*)()const>(&CCLib::BoundingBox::minCorner)), "minCorner" },
						{ fun(static_cast<const CCVector3 & (CCLib::BoundingBox::*)()const>(&CCLib::BoundingBox::maxCorner)), "maxCorner" },
						{ fun(static_cast<CCVector3 & (CCLib::BoundingBox::*)()>(&CCLib::BoundingBox::minCorner)), "minCorner" },
						{ fun(static_cast<CCVector3 & (CCLib::BoundingBox::*)()>(&CCLib::BoundingBox::maxCorner)), "maxCorner" },
						{ fun(&CCLib::BoundingBox::getCenter), "getCenter" },
						{ fun(&CCLib::BoundingBox::getDiagVec), "getDiagVec" },
						{ fun(&CCLib::BoundingBox::getDiagNormd), "getDiagNormd" },
						{ fun(&CCLib::BoundingBox::getMinBoxDim), "getMinBoxDim" },
						{ fun(&CCLib::BoundingBox::getMaxBoxDim), "getMaxBoxDim" },
						{ fun(&CCLib::BoundingBox::computeVolume), "computeVolume" },
						{ fun(&CCLib::BoundingBox::setValidity), "setValidity" },
						{ fun(&CCLib::BoundingBox::isValid), "isValid" },
						{ fun(&CCLib::BoundingBox::minDistTo), "minDistTo" },
						{ fun(&CCLib::BoundingBox::contains), "contains" }
					}
					);
				return m;
			}

			ModulePtr bs_GenericMesh(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(chaiscript::user_type<GenericMesh>(), "GenericMesh");
				
				m->add(fun(&GenericMesh::size), "size");
				m->add(fun(&GenericMesh::forEach), "forEach");
				m->add(fun(&GenericMesh::getBoundingBox), "getBoundingBox");
				m->add(fun(&GenericMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
				m->add(fun(&GenericMesh::_getNextTriangle), "_getNextTriangle");

				return m;
			}

			ModulePtr bs_GenericCloud(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(chaiscript::user_type<GenericCloud>(), "GenericCloud");
				m->add(chaiscript::user_type<GenericCloud::genericPointAction>(), "genericPointAction");
				m->add(fun(&GenericCloud::size), "size");
				m->add(fun(&GenericCloud::forEach), "forEach");
				m->add(fun(&GenericCloud::getBoundingBox), "getBoundingBox");
				m->add(fun(&GenericCloud::testVisibility), "testVisibility");
				m->add(fun(&GenericCloud::placeIteratorAtBeginning), "placeIteratorAtBeginning");
				m->add(fun(&GenericCloud::getNextPoint), "getNextPoint");
				m->add(fun(&GenericCloud::enableScalarField), "enableScalarField");
				m->add(fun(&GenericCloud::isScalarFieldEnabled), "isScalarFieldEnabled");
				m->add(fun(&GenericCloud::setPointScalarValue), "setPointScalarValue");
				m->add(fun(&GenericCloud::getPointScalarValue), "getPointScalarValue");

				return m;
			}


			ModulePtr bs_GenericTriangle(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<GenericTriangle>(), "GenericTriangle");
				m->add(fun(&GenericTriangle::_getA), "_getA");
				m->add(fun(&GenericTriangle::_getB), "_getB");
				m->add(fun(&GenericTriangle::_getC), "_getC");				
				return m;
			}

			ModulePtr bs_SimpleTriangle(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<SimpleTriangle>(), "SimpleTriangle");
				m->add(constructor<SimpleTriangle()>(), "SimpleTriangle");
				m->add(constructor<SimpleTriangle(const CCVector3&, const CCVector3&, const CCVector3&)>(), "SimpleTriangle");
				m->add(fun(&SimpleTriangle::_getA), "_getA");
				m->add(fun(&SimpleTriangle::_getB), "_getB");
				m->add(fun(&SimpleTriangle::_getC), "_getC");
				m->add(fun(&SimpleTriangle::A), "A");
				m->add(fun(&SimpleTriangle::B), "B");
				m->add(fun(&SimpleTriangle::C), "C");
				m->add(chaiscript::base_class< GenericTriangle, SimpleTriangle>());
				return m;
			}

			ModulePtr bs_SimpleRefTriangle(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<SimpleRefTriangle>(), "SimpleRefTriangle");
				m->add(constructor<SimpleRefTriangle()>(), "SimpleRefTriangle");
				m->add(constructor<SimpleRefTriangle(const CCVector3*, const CCVector3*, const CCVector3*)>(), "SimpleRefTriangle");
				m->add(fun(&SimpleRefTriangle::_getA), "_getA");
				m->add(fun(&SimpleRefTriangle::_getB), "_getB");
				m->add(fun(&SimpleRefTriangle::_getC), "_getC");
				m->add(fun(&SimpleRefTriangle::A), "A");
				m->add(fun(&SimpleRefTriangle::B), "B");
				m->add(fun(&SimpleRefTriangle::C), "C");
				m->add(chaiscript::base_class< GenericTriangle, SimpleRefTriangle>());
				return m;
			}

			ModulePtr bs_VerticesIndexes(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(chaiscript::user_type<VerticesIndexes>(), "VerticesIndexes");
				m->add(chaiscript::constructor<VerticesIndexes()>(), "VerticesIndexes");
				m->add(chaiscript::constructor<VerticesIndexes(unsigned,unsigned,unsigned)>(), "VerticesIndexes");
				m->add(fun(&VerticesIndexes::i1), "i1");
				m->add(fun(&VerticesIndexes::i2), "i2");
				m->add(fun(&VerticesIndexes::i3), "i3");
				m->add(fun(&VerticesIndexes::i), "i");
				chaiscript::bootstrap::array<unsigned[3]>("i_Array", m);
				return m;
			}

			ModulePtr bs_GenericIndexedMesh(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(chaiscript::user_type<GenericIndexedMesh>(), "GenericIndexedMesh");

				m->add(fun(&GenericIndexedMesh::size), "size");
				m->add(fun(&GenericIndexedMesh::forEach), "forEach");
				m->add(fun(&GenericIndexedMesh::getBoundingBox), "getBoundingBox");
				m->add(fun(&GenericIndexedMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
				m->add(fun(&GenericIndexedMesh::_getNextTriangle), "_getNextTriangle");
				m->add(fun(&GenericIndexedMesh::_getTriangle), "_getTriangle");
				m->add(fun(&GenericIndexedMesh::getTriangleVertIndexes), "getTriangleVertIndexes");
				m->add(fun(&GenericIndexedMesh::getTriangleVertices), "getTriangleVertices");
				m->add(fun(&GenericIndexedMesh::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");

				m->add(chaiscript::base_class< GenericMesh, GenericIndexedMesh>());
				return m;
			}

			ModulePtr bs_SimpleMesh(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<SimpleMesh>(), "SimpleMesh");
				m->add(constructor<SimpleMesh(GenericIndexedCloud*, bool)>(), "SimpleMesh");

				m->add(fun(&SimpleMesh::forEach), "forEach");
				m->add(fun(&SimpleMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
				m->add(fun(&SimpleMesh::_getNextTriangle), "_getNextTriangle");
				m->add(fun(&SimpleMesh::_getTriangle), "_getTriangle");
				m->add(fun(&SimpleMesh::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
				m->add(fun(&SimpleMesh::getTriangleVertIndexes), "getTriangleVertIndexes");
				m->add(fun(&SimpleMesh::size), "size");
				m->add(fun(&SimpleMesh::getBoundingBox), "getBoundingBox");
				m->add(fun(&SimpleMesh::getTriangleVertices), "getTriangleVertices");
				m->add(fun(&SimpleMesh::capacity), "capacity");
				m->add(fun(&SimpleMesh::vertices), "vertices");
				m->add(fun(&SimpleMesh::clear), "clear");
				m->add(fun(&SimpleMesh::addTriangle), "addTriangle");
				m->add(fun(&SimpleMesh::reserve), "reserve");
				m->add(fun(&SimpleMesh::resize), "resize");

				m->add(chaiscript::base_class< GenericMesh, SimpleMesh>());
				m->add(chaiscript::base_class< GenericIndexedMesh, SimpleMesh>());
				return m;
			}

			ModulePtr bs_GenericIndexedCloudPersist(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(chaiscript::user_type<GenericIndexedCloudPersist>(), "GenericIndexedCloudPersist");

				m->add(fun(&GenericIndexedCloudPersist::getPointPersistentPtr), "getPointPersistentPtr");
				m->add(chaiscript::base_class< GenericIndexedCloud, GenericIndexedCloudPersist>());
				m->add(chaiscript::base_class< GenericCloud, GenericIndexedCloudPersist>());
				
				return m;
			}

			ModulePtr bs_CCShareable(ModulePtr m = std::make_shared<Module>())
			{
				m->add(chaiscript::user_type<CCShareable>(), "CCShareable");
				m->add(fun(&CCShareable::link), "link");
				m->add(fun(&CCShareable::release), "release");
				m->add(fun(&CCShareable::getLinkCount), "getLinkCount");
				#ifdef CC_TRACK_ALIVE_SHARED_OBJECTS
					m->add(fun(&CCShareable::GetAliveCount), "GetAliveCount");
				#endif

				return m;
			}

			

			ModulePtr bs_ChamferDistanceTransform(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<ChamferDistanceTransform>(), "ChamferDistanceTransform");
				m->add(fun(&ChamferDistanceTransform::init), "init");
				m->add(fun(&ChamferDistanceTransform::propagateDistance), "propagateDistance");

				m->add(base_class<Grid3D<unsigned short>, ChamferDistanceTransform>());

				return m;
			}


			template<int N, class T>
			ModulePtr bs_ConjugateGradient(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				using Base = ConjugateGradient<N, T>;
				chaiscript::utility::add_class<Base>(*m,
					shortCutName,
					{
						chaiscript::constructor<Base()>(),
					},
					{
						{ fun(&Base::A), "A" },
						{ fun(&Base::b), "b" },
						{ fun(&Base::initConjugateGradient), "initConjugateGradient" },
						{ fun(&Base::iterConjugateGradient), "iterConjugateGradient" },
					}
					);
				return m;
			}


			ModulePtr bs_Delaunay2dMesh(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<Delaunay2dMesh>(), "Delaunay2dMesh");
				m->add(constructor<Delaunay2dMesh()>(), "Delaunay2dMesh");

				m->add(fun(&Delaunay2dMesh::Available), "Delaunay2dMesh_Available");
				m->add(fun(&Delaunay2dMesh::linkMeshWith), "linkMeshWith");
				m->add(fun(static_cast<bool(Delaunay2dMesh::*)(const std::vector<CCVector2>&, std::size_t, char*)>(&Delaunay2dMesh::buildMesh)), "buildMesh");
				m->add(fun(static_cast<bool(Delaunay2dMesh::*)(const std::vector<CCVector2>&, const std::vector<int>&, char*)>(&Delaunay2dMesh::buildMesh)), "buildMesh");	
				m->add(fun(&Delaunay2dMesh::removeOuterTriangles), "removeOuterTriangles");
				m->add(fun(&Delaunay2dMesh::size), "size");
				m->add(fun(&Delaunay2dMesh::forEach), "forEach");
				m->add(fun(&Delaunay2dMesh::getBoundingBox), "getBoundingBox");
				m->add(fun(&Delaunay2dMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
				m->add(fun(&Delaunay2dMesh::_getNextTriangle), "_getNextTriangle");
				m->add(fun(&Delaunay2dMesh::_getTriangle), "_getTriangle");
				m->add(fun(&Delaunay2dMesh::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
				m->add(fun(&Delaunay2dMesh::getTriangleVertIndexes), "getTriangleVertIndexes");
				m->add(fun(&Delaunay2dMesh::getTriangleVertices), "getTriangleVertices");
				m->add(fun(&Delaunay2dMesh::getTriangleVertIndexesArray), "getTriangleVertIndexesArray");
				m->add(fun(&Delaunay2dMesh::removeTrianglesWithEdgesLongerThan), "removeTrianglesWithEdgesLongerThan");
				m->add(fun(&Delaunay2dMesh::getAssociatedCloud), "getAssociatedCloud");
				m->add(fun(static_cast<Delaunay2dMesh*(*)(const std::vector<CCVector2>&)>(&Delaunay2dMesh::TesselateContour)), "TesselateContour");
				m->add(fun(static_cast<Delaunay2dMesh*(*)(GenericIndexedCloudPersist*,int)>(&Delaunay2dMesh::TesselateContour)), "TesselateContour");
				

				
				return m;
			}


			ModulePtr bs_DgmOctreeReferenceCloud(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<DgmOctreeReferenceCloud>(), "DgmOctreeReferenceCloud");
				m->add(constructor<DgmOctreeReferenceCloud(DgmOctree::NeighboursSet*,unsigned)>(), "DgmOctreeReferenceCloud");

				m->add(fun(&DgmOctreeReferenceCloud::size), "size");
				m->add(fun(&DgmOctreeReferenceCloud::forEach), "forEach");
				m->add(fun(&DgmOctreeReferenceCloud::getBoundingBox), "getBoundingBox");
				m->add(fun(&DgmOctreeReferenceCloud::placeIteratorAtBeginning), "placeIteratorAtBeginning");
				m->add(fun(&DgmOctreeReferenceCloud::getNextPoint), "getNextPoint");
				m->add(fun(&DgmOctreeReferenceCloud::enableScalarField), "enableScalarField");
				m->add(fun(&DgmOctreeReferenceCloud::isScalarFieldEnabled), "isScalarFieldEnabled");
				m->add(fun(&DgmOctreeReferenceCloud::setPointScalarValue), "setPointScalarValue");
				m->add(fun(&DgmOctreeReferenceCloud::getPointScalarValue), "getPointScalarValue");
				m->add(fun(static_cast<const CCVector3*(DgmOctreeReferenceCloud::*)(unsigned)const>(&DgmOctreeReferenceCloud::getPoint)), "getPoint");
				m->add(fun(static_cast<void(DgmOctreeReferenceCloud::*)(unsigned, CCVector3&)const>(&DgmOctreeReferenceCloud::getPoint)), "getPoint");
				m->add(fun(&DgmOctreeReferenceCloud::getPointPersistentPtr), "getPointPersistentPtr");
				m->add(fun(&DgmOctreeReferenceCloud::forwardIterator), "forwardIterator");
				
				m->add(base_class<GenericIndexedCloud, DgmOctreeReferenceCloud>());
				m->add(base_class<GenericCloud, DgmOctreeReferenceCloud>());
				m->add(base_class<GenericIndexedCloudPersist, DgmOctreeReferenceCloud>());
				return m;
			}


			ModulePtr bs_DgmOctree(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				
				m->add(user_type<GenericOctree>(), "GenericOctree");
				m->add(user_type<DgmOctree>(), "DgmOctree");
				m->add(constructor<DgmOctree(GenericIndexedCloudPersist*)>(), "DgmOctree");

				m->add(fun(&DgmOctree::GET_BIT_SHIFT), "GET_BIT_SHIFT");
				m->add(fun(&DgmOctree::OCTREE_LENGTH), "OCTREE_LENGTH");

				m->add(user_type<DgmOctree::CellCode>(), "CellCode");
				m->add_global_const(const_var(DgmOctree::MAX_OCTREE_LEVEL), "MAX_OCTREE_LEVEL");
				m->add_global_const(const_var(DgmOctree::MAX_OCTREE_LENGTH), "MAX_OCTREE_LENGTH");
				m->add_global_const(const_var(DgmOctree::INVALID_CELL_CODE), "INVALID_CELL_CODE");
				m->add(user_type<DgmOctree::cellCodesContainer>(), "cellCodesContainer");
				m->add(user_type<DgmOctree::cellIndexesContainer>(), "cellIndexesContainer");
				m->add(user_type<DgmOctree::PointDescriptor>(), "PointDescriptor");
				m->add(constructor<DgmOctree::PointDescriptor()>(), "PointDescriptor");
				m->add(constructor<DgmOctree::PointDescriptor(const CCVector3*, unsigned)>(), "PointDescriptor");
				m->add(constructor<DgmOctree::PointDescriptor(const CCVector3*, unsigned,double)>(), "PointDescriptor");
				m->add(fun(&DgmOctree::PointDescriptor::point), "point");
				m->add(fun(&DgmOctree::PointDescriptor::pointIndex), "pointIndex");
				m->add(fun(&DgmOctree::PointDescriptor::squareDistd), "squareDistd");
				m->add(fun(&DgmOctree::PointDescriptor::distComp), "distComp");
				m->add(user_type<DgmOctree::NeighboursSet>(), "NeighboursSet");
				m->add(user_type<DgmOctree::CellDescriptor>(), "CellDescriptor");
				m->add(constructor<DgmOctree::CellDescriptor()>(), "CellDescriptor");
				m->add(constructor<DgmOctree::CellDescriptor(const CCVector3&, unsigned)>(), "CellDescriptor");
				m->add(fun(&DgmOctree::CellDescriptor::center), "center");
				m->add(fun(&DgmOctree::CellDescriptor::index), "index");
				m->add(user_type<DgmOctree::NeighbourCellsSet>(), "NeighbourCellsSet");
				m->add(user_type<DgmOctree::NearestNeighboursSearchStruct>(), "NearestNeighboursSearchStruct");
				m->add(constructor<DgmOctree::NearestNeighboursSearchStruct()>(), "NearestNeighboursSearchStruct");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::queryPoint), "queryPoint");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::level), "level");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::minNumberOfNeighbors), "minNumberOfNeighbors");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::cellPos), "cellPos");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::cellCenter), "cellCenter");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::maxSearchSquareDistd), "maxSearchSquareDistd");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::minimalCellsSetToVisit), "minimalCellsSetToVisit");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::pointsInNeighbourhood), "pointsInNeighbourhood");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::alreadyVisitedNeighbourhoodSize), "alreadyVisitedNeighbourhoodSize");
				m->add(fun(&DgmOctree::NearestNeighboursSearchStruct::theNearestPointIndex), "theNearestPointIndex");

				m->add(user_type<DgmOctree::NearestNeighboursSphericalSearchStruct>(), "NearestNeighboursSphericalSearchStruct");
				m->add(constructor<DgmOctree::NearestNeighboursSphericalSearchStruct()>(), "NearestNeighboursSphericalSearchStruct");
#ifdef TEST_CELLS_FOR_SPHERICAL_NN
				m->add(fun(&DgmOctree::NearestNeighboursSphericalSearchStruct::pointsInSphericalNeighbourhood), "pointsInSphericalNeighbourhood");
				m->add(fun(&DgmOctree::NearestNeighboursSphericalSearchStruct::cellsInNeighbourhood), "cellsInNeighbourhood");
				m->add(fun(&DgmOctree::NearestNeighboursSphericalSearchStruct::maxInD2), "maxInD2");
				m->add(fun(&DgmOctree::NearestNeighboursSphericalSearchStruct::minOutD2), "minOutD2");
#endif
				m->add(fun(&DgmOctree::NearestNeighboursSphericalSearchStruct::ready), "ready");
				m->add(fun(&DgmOctree::NearestNeighboursSphericalSearchStruct::prepare), "prepare");
				m->add(chaiscript::base_class< DgmOctree::NearestNeighboursSearchStruct, DgmOctree::NearestNeighboursSphericalSearchStruct>());
				
				m->add(user_type<DgmOctree::IndexAndCode>(), "IndexAndCode");
				m->add(constructor<DgmOctree::IndexAndCode()>(), "IndexAndCode");
				m->add(constructor<DgmOctree::IndexAndCode(unsigned, DgmOctree::CellCode)>(), "IndexAndCode");
				m->add(constructor<DgmOctree::IndexAndCode(DgmOctree::IndexAndCode&)>(), "IndexAndCode");
				m->add(fun(&DgmOctree::IndexAndCode::theIndex), "theIndex");
				m->add(fun(&DgmOctree::IndexAndCode::theCode), "theCode");
				m->add(fun(&DgmOctree::IndexAndCode::operator<), "<");
				m->add(fun(&DgmOctree::IndexAndCode::operator>), ">");
				m->add(fun(&DgmOctree::IndexAndCode::codeComp), "codeComp");
				m->add(fun(&DgmOctree::IndexAndCode::indexComp), "indexComp");

				m->add(user_type<DgmOctree::cellsContainer>(), "cellsContainer");
				m->add(user_type<DgmOctree::octreeCell>(), "octreeCell");
				//m->add(constructor<DgmOctree::octreeCell(const DgmOctree*)>(), "octreeCell");
				m->add(fun(&DgmOctree::octreeCell::parentOctree), "parentOctree");
				m->add(fun(&DgmOctree::octreeCell::truncatedCode), "truncatedCode");
				m->add(fun(&DgmOctree::octreeCell::index), "index");
				m->add(fun(&DgmOctree::octreeCell::points), "points");
				m->add(fun(&DgmOctree::octreeCell::level), "level");
				m->add(user_type<DgmOctree::octreeCellFunc>(), "octreeCellFunc");
				m->add(fun(&DgmOctree::clear), "clear");
				m->add(fun(static_cast<int(DgmOctree::*)(GenericProgressCallback*)>(&DgmOctree::build)), "build");
				m->add(fun(static_cast<int(DgmOctree::*)(const CCVector3&, const CCVector3&, const CCVector3*, const CCVector3*,GenericProgressCallback*)>(&DgmOctree::build)), "build");
				m->add(fun([](DgmOctree* dgmOT) {return dgmOT->build(); }), "build");
				m->add(fun([](DgmOctree* dgmOT, const CCVector3& a, const CCVector3& b) {return dgmOT->build(a, b); }), "build");
				m->add(fun([](DgmOctree* dgmOT, const CCVector3& a, const CCVector3& b, const CCVector3* c) {return dgmOT->build(a, b, c); }), "build");
				m->add(fun([](DgmOctree* dgmOT, const CCVector3& a, const CCVector3& b, const CCVector3* c, const CCVector3* d) {return dgmOT->build(a, b, c, d); }), "build");
				m->add(fun(&DgmOctree::getNumberOfProjectedPoints), "getNumberOfProjectedPoints");
				m->add(fun(&DgmOctree::getOctreeMins), "getOctreeMins");
				m->add(fun(&DgmOctree::getOctreeMaxs), "getOctreeMaxs");
				m->add(fun(&DgmOctree::getBoundingBox), "getBoundingBox");
				m->add(fun(&DgmOctree::getMinFillIndexes), "getMinFillIndexes");
				m->add(fun(&DgmOctree::getMaxFillIndexes), "getMaxFillIndexes");
				m->add(fun(&DgmOctree::getCellSize), "getCellSize");
				m->add(fun(static_cast<void(DgmOctree::*)(const Tuple3i&, unsigned char, int*)const>(&DgmOctree::getCellDistanceFromBorders)), "getCellDistanceFromBorders");
				m->add(fun(static_cast<void(DgmOctree::*)(const Tuple3i&, unsigned char, int, int*)const>(&DgmOctree::getCellDistanceFromBorders)), "getCellDistanceFromBorders");
				m->add(fun(&DgmOctree::getPointsInCellByCellIndex), "getPointsInCellByCellIndex");
				m->add(fun(&DgmOctree::getPointsInCell), "getPointsInCell");
				m->add(fun(&DgmOctree::getPointsInCellsWithSortedCellCodes), "getPointsInCellsWithSortedCellCodes");
				m->add(fun(&DgmOctree::findPointNeighbourhood), "findPointNeighbourhood");
				m->add(fun(&DgmOctree::findTheNearestNeighborStartingFromCell), "findTheNearestNeighborStartingFromCell");
				m->add(fun(&DgmOctree::findNearestNeighborsStartingFromCell), "findNearestNeighborsStartingFromCell");
				m->add(fun(&DgmOctree::findNeighborsInASphereStartingFromCell), "findNeighborsInASphereStartingFromCell");
				m->add(fun(&DgmOctree::getPointsInSphericalNeighbourhood), "getPointsInSphericalNeighbourhood");
				
				m->add(user_type<DgmOctree::CylindricalNeighbourhood>(), "CylindricalNeighbourhood");
				m->add(constructor<DgmOctree::CylindricalNeighbourhood()>(), "CylindricalNeighbourhood");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::center), "center");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::dir), "dir");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::radius), "radius");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::maxHalfLength), "maxHalfLength");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::neighbours), "neighbours");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::level), "level");
				m->add(fun(&DgmOctree::CylindricalNeighbourhood::onlyPositiveDir), "onlyPositiveDir");
				m->add(fun(&DgmOctree::getPointsInCylindricalNeighbourhood), "getPointsInCylindricalNeighbourhood");
				m->add(user_type<DgmOctree::ProgressiveCylindricalNeighbourhood>(), "ProgressiveCylindricalNeighbourhood");
				m->add(constructor<DgmOctree::ProgressiveCylindricalNeighbourhood()>(), "ProgressiveCylindricalNeighbourhood");
				m->add(fun(&DgmOctree::ProgressiveCylindricalNeighbourhood::currentHalfLength), "currentHalfLength");
				m->add(fun(&DgmOctree::ProgressiveCylindricalNeighbourhood::potentialCandidates), "potentialCandidates");
				m->add(fun(&DgmOctree::ProgressiveCylindricalNeighbourhood::prevMinCornerPos), "prevMinCornerPos");
				m->add(fun(&DgmOctree::ProgressiveCylindricalNeighbourhood::prevMaxCornerPos), "prevMaxCornerPos");
				m->add(chaiscript::base_class< DgmOctree::CylindricalNeighbourhood, DgmOctree::ProgressiveCylindricalNeighbourhood>());
				
				m->add(fun(&DgmOctree::getPointsInCylindricalNeighbourhoodProgressive), "getPointsInCylindricalNeighbourhoodProgressive");
				
				m->add(user_type<DgmOctree::BoxNeighbourhood>(), "BoxNeighbourhood");
				m->add(constructor<DgmOctree::BoxNeighbourhood()>(), "BoxNeighbourhood");
				m->add(fun(&DgmOctree::BoxNeighbourhood::center), "center");
				m->add(fun(&DgmOctree::BoxNeighbourhood::axes), "axes");
				m->add(fun(&DgmOctree::BoxNeighbourhood::dimensions), "dimensions");
				m->add(fun(&DgmOctree::BoxNeighbourhood::neighbours), "neighbours");
				m->add(fun(&DgmOctree::BoxNeighbourhood::level), "level");
				
				m->add(fun(&DgmOctree::getPointsInBoxNeighbourhood), "getPointsInBoxNeighbourhood");
				
				m->add(fun(static_cast<DgmOctree::CellCode(*)(const Tuple3i&, unsigned char)>(&DgmOctree::GenerateTruncatedCellCode)), "GenerateTruncatedCellCode");
				m->add(fun(static_cast<void(DgmOctree::*)(const CCVector3*, Tuple3i&)const>(&DgmOctree::getTheCellPosWhichIncludesThePoint)), "getTheCellPosWhichIncludesThePoint");
				m->add(fun(static_cast<void(DgmOctree::*)(const CCVector3*, Tuple3i&, unsigned char)const>(&DgmOctree::getTheCellPosWhichIncludesThePoint)), "getTheCellPosWhichIncludesThePoint");
				m->add(fun(static_cast<void(DgmOctree::*)(const CCVector3*, Tuple3i&, unsigned char, bool&)const>(&DgmOctree::getTheCellPosWhichIncludesThePoint)), "getTheCellPosWhichIncludesThePoint");

				m->add(fun(&DgmOctree::getCellPos), "getCellPos");
				m->add(fun(static_cast<void(DgmOctree::*)(DgmOctree::CellCode, unsigned char, CCVector3&, bool)const>(&DgmOctree::computeCellCenter)), "computeCellCenter");
				m->add(fun(static_cast<void(DgmOctree::*)(const Tuple3i&, unsigned char, CCVector3&)const>(&DgmOctree::computeCellCenter)), "computeCellCenter");
#ifndef OCTREE_CODES_64_BITS
				m->add(fun(static_cast<DgmOctree::CellCode(*)(const Tuple3s&, unsigned char)>(&DgmOctree::GenerateTruncatedCellCode)), "GenerateTruncatedCellCode");
				m->add(fun(static_cast<void(DgmOctree::*)(const Tuple3s&, unsigned char, CCVector3&)const>(&DgmOctree::computeCellCenter)), "computeCellCenter");
#endif
				m->add(fun(&DgmOctree::computeCellLimits), "computeCellLimits");
				m->add(fun(&DgmOctree::findBestLevelForAGivenNeighbourhoodSizeExtraction), "findBestLevelForAGivenNeighbourhoodSizeExtraction");
				m->add(fun(&DgmOctree::findBestLevelForComparisonWithOctree), "findBestLevelForComparisonWithOctree");
				m->add(fun(&DgmOctree::findBestLevelForAGivenPopulationPerCell), "findBestLevelForAGivenPopulationPerCell");
				m->add(fun(&DgmOctree::findBestLevelForAGivenCellNumber), "findBestLevelForAGivenCellNumber");
				m->add(fun(&DgmOctree::getCellCode), "getCellCode");
				m->add(fun(&DgmOctree::getCellCodes), "getCellCodes");
				m->add(fun(&DgmOctree::getCellIndexes), "getCellIndexes");
				m->add(fun(&DgmOctree::getCellCodesAndIndexes), "getCellCodesAndIndexes");
				m->add(fun(static_cast<void(DgmOctree::*)(const DgmOctree::cellCodesContainer&, const DgmOctree::cellCodesContainer&, DgmOctree::cellCodesContainer&, DgmOctree::cellCodesContainer&)const> (&DgmOctree::diff)), "diff");
				m->add(fun(static_cast<bool(DgmOctree::*)(unsigned char, const DgmOctree::cellsContainer&, const DgmOctree::cellsContainer&, int&, int&, int&, int&)const> (&DgmOctree::diff)), "diff");

				m->add(fun(&DgmOctree::getCellNumber), "getCellNumber");
				m->add(fun(&DgmOctree::computeMeanOctreeDensity), "computeMeanOctreeDensity");
				m->add(fun(&DgmOctree::ComputeMinDistanceToCellBorder), "ComputeMinDistanceToCellBorder");
				m->add(fun(static_cast<int(DgmOctree::*)(const DgmOctree::cellCodesContainer&, unsigned char, bool, GenericProgressCallback*)const>(&DgmOctree::extractCCs)), "extractCCs");
				m->add(fun([](DgmOctree* dgmOT, const DgmOctree::cellCodesContainer& a, unsigned char b, bool c) {return dgmOT->extractCCs(a, b, c); }), "extractCCs");
				m->add(fun(static_cast<int(DgmOctree::*)(unsigned char, bool, GenericProgressCallback*)const>(&DgmOctree::extractCCs)), "extractCCs");
				m->add(fun([](DgmOctree* dgmOT, unsigned char a, bool b) {return dgmOT->extractCCs(a, b); }), "extractCCs");
				m->add(fun(&DgmOctree::executeFunctionForAllCellsStartingAtLevel), "executeFunctionForAllCellsStartingAtLevel");
				m->add(fun(&DgmOctree::executeFunctionForAllCellsAtLevel), "executeFunctionForAllCellsAtLevel");
				m->add(fun(&DgmOctree::rayCast), "rayCast");
				m->add(fun(&DgmOctree::associatedCloud), "associatedCloud");
				m->add(fun(&DgmOctree::pointsAndTheirCellCodes), "pointsAndTheirCellCodes");
				m->add(fun(&DgmOctree::MultiThreadSupport), "MultiThreadSupport");

				m->add(chaiscript::base_class< GenericOctree, DgmOctree>());
				return m;
			}

			ModulePtr bs_KDTree(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<KDTree>(), "KDTree");
				m->add(constructor<KDTree()>(), "KDTree");
				m->add(fun(&KDTree::buildFromCloud), "buildFromCloud");
				m->add(fun(&KDTree::getAssociatedCloud), "getAssociatedCloud");
				m->add(fun(&KDTree::findNearestNeighbour), "findNearestNeighbour");
				m->add(fun(&KDTree::findPointBelowDistance), "findPointBelowDistance");
				m->add(fun(&KDTree::findPointsLyingToDistance), "findPointsLyingToDistance");
				return m;
			}

			/*ModulePtr bs_LocalModel(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<LocalModel>(), "LocalModel");
				m->add(fun(&LocalModel::New), "LocalModel_New");
				m->add(fun(&LocalModel::getType), "getType");
				m->add(fun(&LocalModel::getCenter), "getCenter");
				m->add(fun(&LocalModel::getSquareSize), "getSquareSize");
				m->add(fun(&LocalModel::computeDistanceFromModelToPoint), "computeDistanceFromModelToPoint");
				m->add(fun([](LocalModel* lm,const CCVector3* a) {return lm->computeDistanceFromModelToPoint(a); }), "computeDistanceFromModelToPoint");
				return m;
			}*/


			ModulePtr bs_FastMarching(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<FastMarching>(), "FastMarching");

				m->add(fun(&FastMarching::setSeedCell), "setSeedCell");
				m->add(fun(&FastMarching::propagate), "propagate");
				m->add(fun(&FastMarching::cleanLastPropagation), "cleanLastPropagation");
				m->add(fun(&FastMarching::getTime), "getTime");
				m->add(fun(&FastMarching::setExtendedConnectivity), "setExtendedConnectivity");
				return m;
			}

			/*ModulePtr bs_FastMarchingForPropagation(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<FastMarchingForPropagation>(), "FastMarchingForPropagation");
				m->add(constructor<FastMarchingForPropagation()>(), "FastMarchingForPropagation");
				m->add(fun(&FastMarchingForPropagation::init), "init");
				m->add(fun(&FastMarchingForPropagation::extractPropagatedPoints), "extractPropagatedPoints");
				m->add(fun(&FastMarchingForPropagation::setPropagationTimingsAsDistances), "setPropagationTimingsAsDistances");
				m->add(fun(&FastMarchingForPropagation::setDetectionThreshold), "setDetectionThreshold");
				m->add(fun(&FastMarchingForPropagation::setJumpCoef), "setJumpCoef");
				m->add(fun(&FastMarchingForPropagation::findPeaks), "findPeaks");
				m->add(fun(&FastMarchingForPropagation::propagate), "propagate");

				m->add(chaiscript::base_class< FastMarching, FastMarchingForPropagation>());

				return m;
			}*/

			template<typename T>
			ModulePtr bs_Garbage(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				using Base = Garbage<T>;
				chaiscript::utility::add_class<Base>(*m,
					shortCutName,
					{
						chaiscript::constructor<Base()>(),
					},
					{
						{ fun(&Base::add), "add" },
						{ fun(&Base::remove), "remove" },
						{ fun(&Base::destroy), "destroy" },
						{ fun(&Base::m_items), "m_items" },
					}
					);
				return m;
			}

			ModulePtr bs_GenericDistribution(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<GenericDistribution>(), "GenericDistribution");
				m->add(fun(&GenericDistribution::getName), "getName");
				m->add(fun(&GenericDistribution::isValid), "isValid");
				m->add(user_type<GenericDistribution::ScalarContainer>(), "ScalarContainer");
				m->add(fun(&GenericDistribution::computeParameters), "computeParameters");
				m->add(fun(static_cast<double(GenericDistribution::*)(ScalarType)const>(&GenericDistribution::computeP)), "computeP");
				m->add(fun(&GenericDistribution::computePfromZero), "computePfromZero");
				m->add(fun(static_cast<double(GenericDistribution::*)(ScalarType, ScalarType)const>(&GenericDistribution::computeP)), "computeP");
				m->add(fun(&GenericDistribution::computeChi2Dist), "computeChi2Dist");
				return m;
			}

			ModulePtr bs_NormalDistribution(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<NormalDistribution>(), "NormalDistribution");
				m->add(constructor<NormalDistribution()>(), "NormalDistribution");
				m->add(constructor<NormalDistribution(ScalarType, ScalarType)>(), "NormalDistribution");
				m->add(fun(static_cast<bool(NormalDistribution::*)(const GenericDistribution::ScalarContainer&)>(&NormalDistribution::computeParameters)), "computeParameters");
				m->add(fun(static_cast<double(NormalDistribution::*)(ScalarType)const>(&NormalDistribution::computeP)), "computeP");
				m->add(fun(&NormalDistribution::computePfromZero), "computePfromZero");
				m->add(fun(static_cast<double(NormalDistribution::*)(ScalarType, ScalarType)const>(&NormalDistribution::computeP)), "computeP");
				m->add(fun(&NormalDistribution::computeChi2Dist), "computeChi2Dist");
				m->add(fun(&NormalDistribution::getName), "getName");
				m->add(fun(&NormalDistribution::getParameters), "getParameters");
				m->add(fun(&NormalDistribution::setParameters), "setParameters");
				m->add(fun(&NormalDistribution::getMu), "getMu");
				m->add(fun(&NormalDistribution::getSigma2), "getSigma2");
				m->add(fun(static_cast<bool(NormalDistribution::*)(const GenericCloud*)>(&NormalDistribution::computeParameters)), "computeParameters");
				m->add(fun(&NormalDistribution::computeRobustParameters), "computeRobustParameters");
				m->add(chaiscript::base_class< GenericDistribution, NormalDistribution>());
				return m;
			}

			ModulePtr bs_WeibullDistribution(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<WeibullDistribution>(), "WeibullDistribution");
				m->add(constructor<WeibullDistribution()>(), "WeibullDistribution");
				m->add(constructor<WeibullDistribution(ScalarType, ScalarType, ScalarType)>(), "WeibullDistribution");
				m->add(fun(&WeibullDistribution::getParameters), "getParameters");
				m->add(fun(&WeibullDistribution::getOtherParameters), "getOtherParameters");
				m->add(fun(&WeibullDistribution::computeMode), "computeMode");
				m->add(fun(&WeibullDistribution::computeSkewness), "computeSkewness");
				m->add(fun(&WeibullDistribution::setParameters), "setParameters");
				m->add(fun(&WeibullDistribution::setValueShift), "setValueShift");
				m->add(fun(&WeibullDistribution::getValueShift), "getValueShift");
				m->add(fun(static_cast<bool(WeibullDistribution::*)(const GenericDistribution::ScalarContainer&)>(&WeibullDistribution::computeParameters)), "computeParameters");
				m->add(fun(static_cast<double(WeibullDistribution::*)(ScalarType)const>(&WeibullDistribution::computeP)), "computeP");
				m->add(fun(&WeibullDistribution::computePfromZero), "computePfromZero");
				m->add(fun(static_cast<double(WeibullDistribution::*)(ScalarType, ScalarType)const>(&WeibullDistribution::computeP)), "computeP");
				m->add(fun(&WeibullDistribution::computeChi2Dist), "computeChi2Dist");
				m->add(fun(&WeibullDistribution::getName), "getName");
				m->add(chaiscript::base_class< GenericDistribution, WeibullDistribution>());
				return m;
			}


			ModulePtr bs_GenericProgressCallback(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<GenericProgressCallback>(), "GenericProgressCallback");
				m->add(fun(&GenericProgressCallback::update), "update");
				m->add(fun(&GenericProgressCallback::setMethodTitle), "setMethodTitle");
				m->add(fun(&GenericProgressCallback::setInfo), "setInfo");
				m->add(fun(&GenericProgressCallback::start), "start");
				m->add(fun(&GenericProgressCallback::stop), "stop");
				m->add(fun(&GenericProgressCallback::isCancelRequested), "isCancelRequested");
				m->add(fun(&GenericProgressCallback::textCanBeEdited), "textCanBeEdited");

				m->add(user_type<NormalizedProgress>(), "GenericProgressCallback");
				m->add(constructor<NormalizedProgress(GenericProgressCallback*, unsigned, unsigned)>(), "GenericProgressCallback");
				m->add(fun(&NormalizedProgress::scale), "scale");
				m->add(fun(&NormalizedProgress::reset), "reset");
				m->add(fun(&NormalizedProgress::oneStep), "oneStep");
				m->add(fun(&NormalizedProgress::steps), "steps");
				return m;
			}

			ModulePtr bs_TrueKdTree(ModulePtr m = std::make_shared<Module>())
			{
				using namespace CCLib;
				m->add(user_type<TrueKdTree>(), "TrueKdTree");
				m->add(constructor<TrueKdTree(GenericIndexedCloudPersist*)>(), "TrueKdTree");
				m->add(fun(&TrueKdTree::associatedCloud), "associatedCloud");
				m->add(fun(&TrueKdTree::build), "build");
				m->add(fun([](TrueKdTree* tkdt, double a, DistanceComputationTools::ERROR_MEASURES b, unsigned c, unsigned d) {return tkdt->build(a, b, c, d); }), "build");
				m->add(fun([](TrueKdTree* tkdt, double a, DistanceComputationTools::ERROR_MEASURES b, unsigned c) {return tkdt->build(a, b, c); }), "build");
				m->add(fun([](TrueKdTree* tkdt, double a, DistanceComputationTools::ERROR_MEASURES b) {return tkdt->build(a, b); }), "build");
				m->add(fun([](TrueKdTree* tkdt, double a) {return tkdt->build(a); }), "build");
				m->add(fun(&TrueKdTree::clear), "clear");
				m->add(fun(&TrueKdTree::getMaxError), "getMaxError");
				m->add(fun(&TrueKdTree::getMaxErrorType), "getMaxErrorType");
				m->add(fun(&TrueKdTree::getLeaves), "getLeaves");


				m->add(user_type<TrueKdTree::BaseNode>(), "BaseNode");
				m->add(fun(&TrueKdTree::BaseNode::isNode), "isNode");
				m->add(fun(&TrueKdTree::BaseNode::isLeaf), "isLeaf");
				m->add(fun(&TrueKdTree::BaseNode::parent), "parent");
				m->add(user_type<TrueKdTree::Node>(), "Node");
				m->add(constructor<TrueKdTree::Node()>(), "Node");
				m->add(fun(&TrueKdTree::Node::splitValue), "splitValue");
				m->add(fun(&TrueKdTree::Node::leftChild), "leftChild");
				m->add(fun(&TrueKdTree::Node::rightChild), "rightChild");
				m->add(fun(&TrueKdTree::Node::splitDim), "splitDim");

				m->add(user_type<TrueKdTree::Leaf>(), "Leaf");
				m->add(constructor<TrueKdTree::Leaf(ReferenceCloud*, const PointCoordinateType [], ScalarType)>(), "Leaf");
				m->add(fun(&TrueKdTree::Leaf::points), "points");
				m->add(fun(&TrueKdTree::Leaf::planeEq), "planeEq");
				m->add(fun(&TrueKdTree::Leaf::error), "error");
				m->add(fun(&TrueKdTree::Leaf::userData), "userData");

				m->add(user_type<TrueKdTree::LeafVector>(), "LeafVector");
				m->add(chaiscript::base_class< TrueKdTree::BaseNode, TrueKdTree::Node>());
				m->add(chaiscript::base_class< TrueKdTree::BaseNode, TrueKdTree::Leaf>());


				return m;
			}

			
			ModulePtr bs_CCGeom(ModulePtr m = std::make_shared<Module>())
			{
				bs_Vector2Tpl<PointCoordinateType>("CCVector2", m);
				bs_Vector2Tpl<double>("CCVector2d", m);
				bs_Vector2Tpl<int>("CCVector2i", m);

				bs_Tuple3Tpl<unsigned char>("Tuple3ub", m);
				bs_Tuple3Tpl<short>("Tuple3s", m);
				bs_Tuple3Tpl<int>("Tuple3i", m);
				bs_Tuple3Tpl<unsigned int>("Tuple3ui", m);

				bs_Vector3Tpl<PointCoordinateType>("CCVector3", m);
				bs_Vector3Tpl<float>("CCVector3f", m);
				bs_Vector3Tpl<double>("CCVector3d", m);
				return m;
			}

			ModulePtr bs_CCTypes(ModulePtr m = std::make_shared<Module>())
			{
				m->add(user_type<ScalarType>(), "ScalarType");
				m->add(user_type<PointCoordinateType>(), "PointCoordinateType");

				return m;
			}

			ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
			{
				
				bs_CCTypes(m);
				bs_CCGeom(m);
				bs_SquareMatrixTpl<PointCoordinateType>("SquareMatrix", m);
				bs_SquareMatrixTpl<float>("SquareMatrixf", m);
				bs_SquareMatrixTpl<double>("SquareMatrixd", m);
				
				bs_Ray<float>("Rayf", m);
				bs_Ray<double>("Rayd", m);
				bs_AABB<float>("AABBf", m);
				bs_AABB<double>("AABBd", m);

				bs_Jacobi<float>(m);
				bs_Jacobi<double>(m);

				
				bs_Grid3D<unsigned short>("Grid3Dus", m);

				bs_PointCloudTpl<CCLib::GenericIndexedCloudPersist>("PointCloudTpl", m);
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloud, CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>>());
				m->add(chaiscript::base_class< CCLib::GenericCloud, CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>>());
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloudPersist, CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>>());

				bs_PointCloud(m);
				bs_ReferenceCloud(m);
				bs_ScalarField(m);
				bs_boundingBox(m);
				bs_GenericMesh(m);
				bs_GenericCloud(m);
				bs_VerticesIndexes(m);
				bs_GenericIndexedMesh(m);
				bs_ChamferDistanceTransform(m);
				bs_ConjugateGradient<6, double>("ConjugateGradient_N6", m);
				bs_ConjugateGradient<8, double>("ConjugateGradient_N8", m);
				bs_Garbage<CCLib::ScalarField>("ScalarFieldGarbage", m);
				bs_Garbage<CCLib::GenericIndexedCloudPersist>("CloudGarbage", m);
				bs_GenericDistribution(m);
				bs_NormalDistribution(m);
				bs_WeibullDistribution(m);
				bs_Delaunay2dMesh(m);
				bs_DgmOctree(m);
				bs_DgmOctreeReferenceCloud(m);
				bs_FastMarching(m);
				//bs_FastMarchingForPropagation(m);
				bs_GenericProgressCallback(m);
				//bs_LocalModel(m);
				bs_TrueKdTree(m);
				bs_GenericTriangle(m);
				bs_SimpleTriangle(m);
				bs_SimpleRefTriangle(m);
				bs_SimpleMesh(m);
				bs_KDTree(m);
				bs_Polyline(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_CLASSES_HPP