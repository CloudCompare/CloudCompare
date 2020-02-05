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

#include <CCGeom.h>
#include <SquareMatrix.h>
#include <BoundingBox.h>
#include <GenericCloud.h>
#include <PointCloud.h>
#include <GenericIndexedCloudPersist.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <GenericMesh.h>
#include <GenericIndexedMesh.h>

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
						chaiscript::constructor<CCLib::BoundingBox()>(),
						chaiscript::constructor<CCLib::BoundingBox(const CCVector3&, const CCVector3&)>()
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




			ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
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

				bs_SquareMatrixTpl<PointCoordinateType>("SquareMatrix", m);
				bs_SquareMatrixTpl<float>("SquareMatrixf", m);
				bs_SquareMatrixTpl<double>("SquareMatrixd", m);

				bs_PointCloudTpl<CCLib::GenericIndexedCloudPersist>("PointCloudTpl", m);
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloud, CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>>());
				m->add(chaiscript::base_class< CCLib::GenericCloud, CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>>());
				m->add(chaiscript::base_class< CCLib::GenericIndexedCloudPersist, CCLib::PointCloudTpl<CCLib::GenericIndexedCloudPersist>>());

				bs_PointCloud(m);
				bs_ReferenceCloud(m);
				bs_ScalarField(m);
				bs_boundingBox(m);
				bs_GenericMesh(m);
				bs_VerticesIndexes(m);
				bs_GenericIndexedMesh(m);

				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_CLASSES_HPP