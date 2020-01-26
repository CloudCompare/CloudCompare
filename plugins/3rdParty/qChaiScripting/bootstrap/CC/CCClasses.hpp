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
#include <BoundingBox.h>

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




			ModulePtr boundingBox(ModulePtr m = std::make_shared<Module>())
			{
				chaiscript::utility::add_class<CCLib::BoundingBox>(*m,
					"BoundingBox",
					{
						chaiscript::constructor<CCLib::BoundingBox()>(),
						chaiscript::constructor<CCLib::BoundingBox(const CCVector3&, const CCVector3&)>()
					},
					{
						{ fun(&CCLib::BoundingBox::clear), "clear" },
						{ fun(&CCLib::BoundingBox::add), "add" },
						{ fun(&CCLib::BoundingBox::getCenter), "getCenter" },
						{ fun(&CCLib::BoundingBox::getDiagVec), "getDiagVec" }
					}
					);
				return m;
			}


			ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
			{
				bs_Vector2Tpl<PointCoordinateType>("CCVector2", m);
				bs_Vector2Tpl<double>("CCVector2d", m);
				bs_Vector2Tpl<int>("CCVector2i", m); 

				bs_Vector3Tpl<PointCoordinateType>("CCVector3", m);
				bs_Vector3Tpl<float>("CCVector3f", m);
				bs_Vector3Tpl<double>("CCVector3d", m);

				boundingBox(m);
				return m;
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_CC_CLASSES_HPP