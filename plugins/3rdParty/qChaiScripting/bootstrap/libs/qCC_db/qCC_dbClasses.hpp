#ifndef CHAISCRIPTING_BOOTSTRAP_QCC_DB_CLASSES_HPP
#define CHAISCRIPTING_BOOTSTRAP_QCC_DB_CLASSES_HPP

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

#include <Qt>
#include <QString>
#include <QSharedPointer>
#include <QOpenGLTexture>
#include <QtGui/qopengl.h>

#include <ccGenericGLDisplay.h>
#include <ccInteractor.h>
#include <ccObject.h>
#include <ccHObject.h>
#include <ccGenericMesh.h>
#include <ccGenericPointCloud.h>
#include <ccMaterialSet.h>
#include <GenericProgressCallback.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccSubMesh.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <cc2DLabel.h>
#include <cc2DViewportLabel.h>
#include <cc2DViewportObject.h>
#include <ccCone.h>
#include <ccExtru.h>
#include <ccGBLSensor.h>
#include <ccCameraSensor.h>
#include <ccDish.h>
#include <ccFacet.h>
#include <ccGenericPrimitive.h>
#include <ccImage.h>
#include <ccIndexedTransformationBuffer.h>
#include <ccKdTree.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccPlanarEntityInterface.h>
#include <ccPlane.h>
#include <ccPolyline.h>
#include <ccSensor.h>
#include <ccShiftedObject.h>
#include <ccTorus.h>
#include <ccDrawableObject.h>
#include <ccProgressDialog.h>
#include <GenericIndexedMesh.h>
#include <ccScalarField.h>
#include <ccArray.h>
#include <ccBasicTypes.h>
#include <ccSingleton.h>
#include <ccQuadric.h>
#include <ccPointCloudInterpolator.h>
#include <ccNormalVectors.h>
#include <GeometricalAnalysisTools.h>
#include <ccMinimumSpanningTreeForNormsDirection.h>
#include <ccIncludeGL.h>
#include <ccGriddedTools.h>
#include <ccGenericGLDisplay.h>
#include <ccFlags.h>
#include <ccFileUtils.h>
#include <ccFastMarchingForNormsDirection.h>
#include <ccCustomObject.h>
#include <ccColorTypes.h>
#include <ccClipBox.h>
#include <ccColorScalesManager.h>



namespace chaiscript
{
	namespace cloudCompare
	{
		namespace libs
		{
			namespace qCC_db
			{
				ModulePtr bs_ccObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccObject>(), "ccObject" );
					m->add(fun(&ccObject::GetCurrentDBVersion), "GetCurrentDBVersion" );
					m->add(fun(&ccObject::SetUniqueIDGenerator), "SetUniqueIDGenerator" );
					m->add(fun(&ccObject::GetUniqueIDGenerator), "GetUniqueIDGenerator" );
					m->add(fun(&ccObject::getClassID), "getClassID" );
					m->add(fun(&ccObject::getName), "getName" );
					m->add(fun(&ccObject::setName), "setName" );
					m->add(fun(&ccObject::getUniqueID), "getUniqueID" );
					m->add(fun(&ccObject::setUniqueID), "setUniqueID" );
					m->add(fun(&ccObject::isEnabled), "isEnabled" );
					m->add(fun(&ccObject::setEnabled), "setEnabled" );
					m->add(fun(&ccObject::toggleActivation), "toggleActivation" );
					m->add(fun(&ccObject::isLocked), "isLocked");
					m->add(fun(&ccObject::setLocked), "setLocked");
					m->add(fun(&ccObject::isLeaf), "isLeaf");
					m->add(fun(&ccObject::isCustom), "isCustom");
					m->add(fun(&ccObject::isHierarchy), "isHierarchy");
					m->add(fun(&ccObject::isKindOf), "isKindOf");
					m->add(fun(&ccObject::isA), "isA");
					m->add(fun(&ccObject::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccObject::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccObject::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccObject::getMetaData), "getMetaData");
					m->add(fun(&ccObject::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccObject::*)(const  QString&, const QVariant&)>(&ccObject::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccObject::*)(const QVariantMap&, bool)>(&ccObject::setMetaData)), "setMetaData");
					m->add(fun(&ccObject::hasMetaData), "hasMetaData");
					m->add(fun(&ccObject::metaData), "metaData");

					m->add(chaiscript::base_class<ccSerializableObject, ccObject>());

					return m;
				}

				ModulePtr bs_ccHObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccHObject>(), "ccHObject");
					//m->add(chaiscript::user_type<ccHObject*>(), "ccHObject");
					//m->add(chaiscript::user_type<ccHObject&>(), "ccHObject");
					m->add(chaiscript::constructor<ccHObject(const QString&)>(), "ccHObject");
					m->add(chaiscript::constructor<ccHObject(const ccHObject&)>(), "ccHObject");
					m->add(fun(&ccHObject::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccHObject::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccHObject::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccHObject::getClassID), "getClassID");
					//m->add(fun(&ccHObject::getName), "getName");
					//m->add(fun(&ccHObject::setName), "setName");
					m->add(fun(&ccHObject::getUniqueID), "getUniqueID");
					m->add(fun(&ccHObject::setUniqueID), "setUniqueID");
					m->add(fun(&ccHObject::isEnabled), "isEnabled");
					m->add(fun(&ccHObject::setEnabled), "setEnabled");
					m->add(fun(&ccHObject::toggleActivation), "toggleActivation");
					m->add(fun(&ccHObject::isLocked), "isLocked");
					m->add(fun(&ccHObject::setLocked), "setLocked");
					m->add(fun(&ccHObject::isLeaf), "isLeaf");
					m->add(fun(&ccHObject::isCustom), "isCustom");
					m->add(fun(&ccHObject::isHierarchy), "isHierarchy");
					m->add(fun(&ccHObject::isKindOf), "isKindOf");
					m->add(fun(&ccHObject::isA), "isA");
					m->add(fun(&ccHObject::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccHObject::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccHObject::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccHObject::getMetaData), "getMetaData");
					m->add(fun(&ccHObject::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccHObject::*)(const  QString&, const QVariant&)>(&ccHObject::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccHObject::*)(const QVariantMap&, bool)>(&ccHObject::setMetaData)), "setMetaData");
					m->add(fun(&ccHObject::hasMetaData), "hasMetaData");
					m->add(fun(&ccHObject::metaData), "metaData");
					m->add(fun([](CC_CLASS_ENUM cce, const char* n) {return ccHObject::New(cce, n); }), "ccHObject_New");
					m->add(fun([](CC_CLASS_ENUM cce) {return ccHObject::New(cce); }), "ccHObject_New");
					m->add(fun([](const QString& pid, const QString& cid, const char* n) {return ccHObject::New(pid, cid, n); }), "ccHObject_New");
					m->add(fun([](const QString& pid, const QString& cid) {return ccHObject::New(pid, cid); }), "ccHObject_New");
					m->add(fun(&ccHObject::getClassID), "getClassID");
					m->add(fun(&ccHObject::isGroup), "isGroup");
					m->add(fun(&ccHObject::getParent), "getParent");
					m->add(fun(&ccHObject::getIcon), "getIcon");
					m->add(fun(&ccHObject::addDependency), "addDependency");
					m->add(fun(&ccHObject::getDependencyFlagsWith), "getDependencyFlagsWith");
					m->add(fun(&ccHObject::removeDependencyWith), "removeDependencyWith");
					m->add(fun(&ccHObject::removeDependencyFlag), "removeDependencyFlag");
					m->add(fun(&ccHObject::addChild), "addChild");
					m->add(fun(&ccHObject::getChildrenNumber), "getChildrenNumber");
					m->add(fun(&ccHObject::getChildCountRecursive), "getChildCountRecursive");
					m->add(fun(&ccHObject::getChild), "getChild");
					m->add(fun(&ccHObject::find), "find");
					//m->add(chaiscript::user_type<ccHObject::Container>(), "Container");
					m->add(chaiscript::vector_conversion<ccHObject::Container>());
					m->add(chaiscript::bootstrap::standard_library::vector_type<ccHObject::Container>("Container"));
					m->add(chaiscript::bootstrap::standard_library::vector_type<std::vector<std::shared_ptr<ccHObject>>>("ContainerSTDShare"));
					
					m->add(chaiscript::user_type<ccHObject::Shared>(), "Shared");
					m->add(chaiscript::user_type<ccHObject::SharedContainer>(), "SharedContainer");
					m->add(chaiscript::vector_conversion<ccHObject::SharedContainer>());
					m->add(fun(&ccHObject::filterChildren), "filterChildren");
					m->add(fun([](ccHObject* obj, ccHObject::Container& a, bool b, CC_CLASS_ENUM c, bool d) {return obj->filterChildren(a,b,c,d); }), "filterChildren");
					m->add(fun([](ccHObject* obj, ccHObject::Container& a, bool b, CC_CLASS_ENUM c) {return obj->filterChildren(a, b, c); }), "filterChildren");
					m->add(fun([](ccHObject* obj, ccHObject::Container& a, bool b) {return obj->filterChildren(a, b); }), "filterChildren");
					m->add(fun([](ccHObject* obj, ccHObject::Container& a) {return obj->filterChildren(a); }), "filterChildren");
					m->add(fun(&ccHObject::detachChild), "detachChild");
					m->add(fun(&ccHObject::detatchAllChildren), "detatchAllChildren");
					m->add(fun(static_cast<void(ccHObject::*)(ccHObject*)>(&ccHObject::removeChild)), "removeChild");
					m->add(fun(static_cast<void(ccHObject::*)(int)>(&ccHObject::removeChild)), "removeChild");
					m->add(fun(&ccHObject::removeAllChildren), "removeAllChildren");
					m->add(fun(&ccHObject::getChildIndex), "getChildIndex");
					m->add(fun(&ccHObject::swapChildren), "swapChildren");
					m->add(fun(&ccHObject::getIndex), "getIndex");
					m->add(fun(&ccHObject::transferChild), "transferChild");
					m->add(fun(&ccHObject::transferChildren), "transferChildren");
					m->add(fun(&ccHObject::getFirstChild), "getFirstChild");
					m->add(fun(&ccHObject::getLastChild), "getLastChild");
					m->add(fun(&ccHObject::isAncestorOf), "isAncestorOf");
					m->add(fun(&ccHObject::getOwnBB), "getOwnBB");
					m->add(fun(&ccHObject::getBB_recursive), "getBB_recursive");
					m->add(fun(&ccHObject::getDisplayBB_recursive), "getDisplayBB_recursive");
					m->add(fun(&ccHObject::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccHObject::getGlobalBB), "getGlobalBB");
					m->add(fun(&ccHObject::drawBB), "drawBB");
					m->add(fun(&ccHObject::draw), "draw");
					m->add(fun(&ccHObject::getAbsoluteGLTransformation), "getAbsoluteGLTransformation");
					m->add(fun(&ccHObject::isDisplayed), "isDisplayed");
					m->add(fun(&ccHObject::isDisplayedIn), "isDisplayedIn");
					m->add(fun(&ccHObject::isBranchEnabled), "isBranchEnabled");
					m->add(fun(&ccHObject::setSelected), "setSelected");
					m->add(fun(&ccHObject::setDisplay), "setDisplay");
					m->add(fun(&ccHObject::removeFromDisplay), "removeFromDisplay");
					m->add(fun(&ccHObject::prepareDisplayForRefresh), "prepareDisplayForRefresh");
					m->add(fun(&ccHObject::refreshDisplay), "refreshDisplay");
					m->add(fun(&ccHObject::resetGLTransformationHistory), "resetGLTransformationHistory");
					m->add(fun(&ccHObject::toggleActivation), "toggleActivation");
					m->add(fun(&ccHObject::toggleVisibility), "toggleVisibility");
					m->add(fun(&ccHObject::toggleColors), "toggleColors");
					m->add(fun(&ccHObject::toggleNormals), "toggleNormals");
					m->add(fun(&ccHObject::toggleSF), "toggleSF");
					m->add(fun(&ccHObject::toggleShowName), "toggleShowName");
					m->add(fun(&ccHObject::toggleMaterials), "toggleMaterials");
					m->add(fun(&ccHObject::setSelected_recursive), "setSelected_recursive");
					m->add(fun(&ccHObject::setDisplay_recursive), "setDisplay_recursive");
					m->add(fun(&ccHObject::removeFromDisplay_recursive), "removeFromDisplay_recursive");
					m->add(fun(&ccHObject::prepareDisplayForRefresh_recursive), "prepareDisplayForRefresh_recursive");
					m->add(fun(&ccHObject::refreshDisplay_recursive), "refreshDisplay_recursive");
					m->add(fun(&ccHObject::resetGLTransformationHistory_recursive), "resetGLTransformationHistory_recursive");
					m->add(fun(&ccHObject::toggleActivation_recursive), "toggleActivation_recursive");
					m->add(fun(&ccHObject::toggleVisibility_recursive), "toggleVisibility_recursive");
					m->add(fun(&ccHObject::toggleColors_recursive), "toggleColors_recursive");
					m->add(fun(&ccHObject::toggleNormals_recursive), "toggleNormals_recursive");
					m->add(fun(&ccHObject::toggleSF_recursive), "toggleSF_recursive");
					m->add(fun(&ccHObject::toggleShowName_recursive), "toggleShowName_recursive");
					m->add(fun(&ccHObject::toggleMaterials_recursive), "toggleMaterials_recursive");
					m->add(fun(&ccHObject::transferDisplay), "transferDisplay");
					m->add(fun(&ccHObject::findMaxUniqueID_recursive), "findMaxUniqueID_recursive");
					m->add(fun(&ccHObject::applyGLTransformation_recursive), "applyGLTransformation_recursive");
					m->add(fun(&ccHObject::notifyGeometryUpdate), "notifyGeometryUpdate");
					m->add(fun(&ccHObject::isSerializable), "isSerializable");
					m->add(fun(&ccHObject::toFile), "toFile");
					m->add(fun(&ccHObject::fromFile), "fromFile");
					m->add(fun(&ccHObject::fromFileNoChildren), "fromFileNoChildren");
					m->add(fun(&ccHObject::isShareable), "isShareable");
					m->add(fun(&ccHObject::setSelectionBehavior), "setSelectionBehavior");
					m->add(fun(&ccHObject::getSelectionBehavior), "getSelectionBehavior");
					m->add(fun(&ccHObject::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccHObject::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccHObject::setGLTransformationHistory), "setGLTransformationHistory");
					m->add(fun(&ccHObject::resetGLTransformationHistory), "resetGLTransformationHistory");
					

					m->add(chaiscript::base_class<ccObject, ccHObject>());
					m->add(chaiscript::base_class<ccSerializableObject, ccHObject>());
					m->add(chaiscript::base_class<ccDrawableObject, ccHObject>());

					return m;
				}

				ModulePtr bs_ccCustomHObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccCustomHObject>(), "ccCustomHObject");
					
					m->add(chaiscript::constructor<ccCustomHObject(const QString&)>(), "ccCustomHObject");
					m->add(chaiscript::constructor<ccCustomHObject(const ccCustomHObject&)>(), "ccCustomHObject");
					m->add(fun(&ccCustomHObject::isSerializable), "isSerializable");
					m->add(fun(&ccCustomHObject::getClassID), "getClassID");
					m->add(fun(&ccCustomHObject::DefautMetaDataClassName), "DefautMetaDataClassName");
					m->add(fun(&ccCustomHObject::DefautMetaDataPluginName), "DefautMetaDataPluginName");

					m->add(chaiscript::base_class<ccObject, ccCustomHObject>());
					m->add(chaiscript::base_class<ccSerializableObject, ccCustomHObject>());
					m->add(chaiscript::base_class<ccDrawableObject, ccCustomHObject>());
					m->add(chaiscript::base_class<ccHObject, ccCustomHObject>());

					return m;
				}

				ModulePtr bs_ccCustomLeafObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccCustomLeafObject>(), "ccCustomLeafObject");
					m->add(chaiscript::constructor<ccCustomLeafObject(const QString&)>(), "ccCustomHObject");
					m->add(fun(&ccCustomLeafObject::getClassID), "getClassID");
					
					m->add(chaiscript::base_class<ccObject, ccCustomLeafObject>());
					m->add(chaiscript::base_class<ccSerializableObject, ccCustomLeafObject>());
					m->add(chaiscript::base_class<ccDrawableObject, ccCustomLeafObject>());
					m->add(chaiscript::base_class<ccHObject, ccCustomLeafObject>());
					m->add(chaiscript::base_class<ccCustomHObject, ccCustomLeafObject>());


					return m;
				}


					

				template<typename Type, int N, class ComponentType>
				ModulePtr bs_ccArray(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
				{
					using Base = ccArray<Type, N, ComponentType>;
					m->add(chaiscript::user_type<Base>(), shortCutName);
					m->add(fun(&Base::clone), "clone");
					m->add(fun(&Base::copy), "copy");
					m->add(fun(&Base::reserveSafe), "reserveSafe");
					m->add(fun(&Base::isAllocated), "isAllocated");
					m->add(fun(&Base::resizeSafe), "resizeSafe");
					m->add(fun(&Base::getClassID), "getClassID");
					m->add(fun(&Base::isShareable), "isShareable");
					m->add(fun(&Base::isSerializable), "isSerializable");
					m->add(fun(static_cast<Type&(Base::*)(size_t)>(&Base::getValue)), "getValue");
					m->add(fun(static_cast<const Type&(Base::*)(size_t)const>(&Base::getValue)), "getValue");
					m->add(fun(&Base::setValue), "setValue");
					m->add(fun(&Base::addElement), "addElement");
					m->add(fun(&Base::fill), "fill");
					m->add(fun(&Base::currentSize), "currentSize");
					m->add(fun(&Base::clear), "clear");
					m->add(fun(&Base::swap), "swap");
					return m;
				}

				ModulePtr bs_NormsIndexesTableType(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<NormsIndexesTableType>(), "NormsIndexesTableType");
					m->add(chaiscript::constructor<NormsIndexesTableType()>(), "NormsIndexesTableType");
					m->add(fun(&NormsIndexesTableType::getClassID), "getClassID");
					m->add(fun(&NormsIndexesTableType::clone), "clone");
					m->add(fun(&NormsIndexesTableType::fromFile_MeOnly), "fromFile_MeOnly");

					m->add(chaiscript::base_class<ccHObject, NormsIndexesTableType>());
					m->add(chaiscript::base_class<ccObject, NormsIndexesTableType>());
					m->add(chaiscript::base_class<ccSerializableObject, NormsIndexesTableType>());
					m->add(chaiscript::base_class<ccDrawableObject, NormsIndexesTableType>());
					m->add(chaiscript::base_class<CCShareable, NormsIndexesTableType>());
					m->add(chaiscript::base_class<std::vector<CompressedNormType>, NormsIndexesTableType>());

					return m;
				}

				ModulePtr bs_NormsTableType(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<NormsTableType>(), "NormsIndexesTableType");
					m->add(chaiscript::constructor<NormsTableType()>(), "NormsIndexesTableType");
					m->add(fun(&NormsTableType::getClassID), "getClassID");
					m->add(fun(&NormsTableType::clone), "clone");

					m->add(chaiscript::base_class<ccHObject, NormsTableType>());
					m->add(chaiscript::base_class<ccObject, NormsTableType>());
					m->add(chaiscript::base_class<ccSerializableObject, NormsTableType>());
					m->add(chaiscript::base_class<ccDrawableObject, NormsTableType>());
					m->add(chaiscript::base_class<CCShareable, NormsTableType>());
					m->add(chaiscript::base_class<std::vector<CCVector3>, NormsTableType>());
					m->add(chaiscript::vector_conversion<std::vector<CCVector3>>());
					return m;
				}

				ModulePtr bs_ColorsTableType(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ColorsTableType>(), "ColorsTableType");
					m->add(chaiscript::constructor<ColorsTableType()>(), "ColorsTableType");
					m->add(fun(&ColorsTableType::getClassID), "getClassID");
					m->add(fun(&ColorsTableType::clone), "clone");

					m->add(chaiscript::base_class<ccHObject, ColorsTableType>());
					m->add(chaiscript::base_class<ccObject, ColorsTableType>());
					m->add(chaiscript::base_class<ccSerializableObject, ColorsTableType>());
					m->add(chaiscript::base_class<ccDrawableObject, ColorsTableType>());
					m->add(chaiscript::base_class<CCShareable, ColorsTableType>());
					m->add(chaiscript::base_class<std::vector<ccColor::Rgb>, ColorsTableType>());
					m->add(chaiscript::vector_conversion<std::vector<ccColor::Rgb>>());

					return m;
				}

				ModulePtr bs_TexCoords2D(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<TexCoords2D>(), "TexCoords2D");
					m->add(chaiscript::constructor<TexCoords2D()>(), "TexCoords2D");
					m->add(chaiscript::constructor<TexCoords2D(float, float)>(), "TexCoords2D");
					m->add(fun(&TexCoords2D::tx), "tx");
					m->add(fun(&TexCoords2D::ty), "ty");
					m->add(fun(&TexCoords2D::t), "t");
					chaiscript::bootstrap::array<float[2]>("t_Array", m);
					return m;
				}

				ModulePtr bs_TextureCoordsContainer(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<TextureCoordsContainer>(), "TextureCoordsContainer");
					m->add(chaiscript::constructor<TextureCoordsContainer()>(), "TextureCoordsContainer");
					m->add(fun(&TextureCoordsContainer::getClassID), "getClassID");
					m->add(fun(&TextureCoordsContainer::clone), "clone");

					m->add(chaiscript::base_class<ccHObject, TextureCoordsContainer>());
					m->add(chaiscript::base_class<ccObject, TextureCoordsContainer>());
					m->add(chaiscript::base_class<ccSerializableObject, TextureCoordsContainer>());
					m->add(chaiscript::base_class<ccDrawableObject, TextureCoordsContainer>());
					m->add(chaiscript::base_class<CCShareable, TextureCoordsContainer>());
					m->add(chaiscript::base_class<std::vector<TexCoords2D>, TextureCoordsContainer>());
					m->add(chaiscript::vector_conversion<std::vector<TexCoords2D>>());
					return m;
				}

				template<typename T>
				ModulePtr bs_ccGLMatrixTpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
				{
					chaiscript::utility::add_class<ccGLMatrixTpl<T>>(*m,
						shortCutName,
						{
							chaiscript::constructor<ccGLMatrixTpl<T>()>(),
							chaiscript::constructor<ccGLMatrixTpl<T>(ccGLMatrixTpl<T>)>(),
							chaiscript::constructor<ccGLMatrixTpl<T>(const float*)>(),
							chaiscript::constructor<ccGLMatrixTpl<T>(const double*)>(),
							chaiscript::constructor<ccGLMatrixTpl<T>(const Vector3Tpl<T>&, const Vector3Tpl<T>&, const Vector3Tpl<T>&, const Vector3Tpl<T>&)>()
						},
					{
						{ fun(&ccGLMatrixTpl<T>::Interpolate), "Interpolate" },
						{ fun(&ccGLMatrixTpl<T>::FromToRotation), "FromToRotation" },
						//{ fun(static_cast<ccGLMatrixTpl<T>(ccGLMatrixTpl<T>::*)(const float*)>(&ccGLMatrixTpl<T>::FromQuaternion)), "FromQuaternion" },
						//{ fun(static_cast<ccGLMatrixTpl<T>(ccGLMatrixTpl<T>::*)(const double*)>(&ccGLMatrixTpl<T>::FromQuaternion)), "FromQuaternion" },
						{ fun(&ccGLMatrixTpl<T>::FromViewDirAndUpDir), "FromViewDirAndUpDir" },
						{ fun(&ccGLMatrixTpl<T>::FromString), "FromString" },
						{ fun(&ccGLMatrixTpl<T>::toString), "toString" },
						{ fun([](ccGLMatrixTpl<T>* mat, int p1) {return mat->toString(p1).toLocal8Bit().constData();; }), "toString"},
						{ fun([](ccGLMatrixTpl<T>* mat) {return mat->toString(); }), "toString"},
						{ fun([](ccGLMatrixTpl<T>* mat) {std::string ret = mat->toString().toLocal8Bit().constData(); return ret; }), "to_string"}, //Chai script calls "to_string" to try to convert to std::string
						{ fun(&ccGLMatrixTpl<T>::toAsciiFile), "toAsciiFile" },
						{ fun(&ccGLMatrixTpl<T>::fromAsciiFile), "fromAsciiFile" },
						{ fun(&ccGLMatrixTpl<T>::xRotation), "xRotation" },
						{ fun(&ccGLMatrixTpl<T>::yRotation), "yRotation" },
						{ fun(&ccGLMatrixTpl<T>::zRotation), "zRotation" },
						{ fun(&ccGLMatrixTpl<T>::toZero), "toZero" },
						{ fun(&ccGLMatrixTpl<T>::toIdentity), "toIdentity" },
						{ fun(&ccGLMatrixTpl<T>::clearTranslation), "clearTranslation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(T,const Vector3Tpl<T>&,const Vector3Tpl<T>&)>(&ccGLMatrixTpl<T>::initFromParameters)), "initFromParameters" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(T,T,T,const Vector3Tpl<T>&)>(&ccGLMatrixTpl<T>::initFromParameters)), "initFromParameters" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(T&,Vector3Tpl<T>&,Vector3Tpl<T>&)const>(&ccGLMatrixTpl<T>::getParameters)), "getParameters" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(T&,T&,T&,Vector3Tpl<T>&)const>(&ccGLMatrixTpl<T>::getParameters)), "getParameters" },
						{ fun(static_cast<T*(ccGLMatrixTpl<T>::*)()>(&ccGLMatrixTpl<T>::data)), "data" },
						{ fun(static_cast<const T*(ccGLMatrixTpl<T>::*)()const>(&ccGLMatrixTpl<T>::data)), "data" },
						{ fun(static_cast<T*(ccGLMatrixTpl<T>::*)()>(&ccGLMatrixTpl<T>::getTranslation)), "getTranslation" },
						{ fun(static_cast<const T*(ccGLMatrixTpl<T>::*)()const>(&ccGLMatrixTpl<T>::getTranslation)), "getTranslation" },
						{ fun(&ccGLMatrixTpl<T>::getTranslationAsVec3D), "getTranslationAsVec3D" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)>(&ccGLMatrixTpl<T>::setTranslation)), "setTranslation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)>(&ccGLMatrixTpl<T>::setTranslation)), "setTranslation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(const float Tr[3])>(&ccGLMatrixTpl<T>::setTranslation)), "setTranslation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(const double Tr[3])>(&ccGLMatrixTpl<T>::setTranslation)), "setTranslation" },
						{ fun(static_cast<T*(ccGLMatrixTpl<T>::*)(unsigned)>(&ccGLMatrixTpl<T>::getColumn)), "getColumn" },
						{ fun(static_cast<const T*(ccGLMatrixTpl<T>::*)(unsigned)const>(&ccGLMatrixTpl<T>::getColumn)), "getColumn" },
						{ fun(&ccGLMatrixTpl<T>::getColumnAsVec3D), "getColumnAsVec3D" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(unsigned, const Vector3Tpl<T>&)>(&ccGLMatrixTpl<T>::setColumn)), "setColumn" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(unsigned, const Tuple4Tpl<T>&)>(&ccGLMatrixTpl<T>::setColumn)), "setColumn" },
						{ fun(static_cast<ccGLMatrixTpl<T> (ccGLMatrixTpl<T>::*)(const ccGLMatrixTpl<T>&)const>(&ccGLMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<Vector3Tpl<float>(ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)const>(&ccGLMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<Vector3Tpl<double>(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)const>(&ccGLMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<Tuple4Tpl<float>(ccGLMatrixTpl<T>::*)(const Tuple4Tpl<float>&)const>(&ccGLMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<Tuple4Tpl<double>(ccGLMatrixTpl<T>::*)(const Tuple4Tpl<double>&)const>(&ccGLMatrixTpl<T>::operator*)), "*" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const ccGLMatrixTpl<T>&)>(&ccGLMatrixTpl<T>::operator+=)), "+=" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const ccGLMatrixTpl<T>&)>(&ccGLMatrixTpl<T>::operator-=)), "-=" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const ccGLMatrixTpl<T>&)>(&ccGLMatrixTpl<T>::operator*=)), "*=" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)>(&ccGLMatrixTpl<T>::operator+=)), "+=" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)>(&ccGLMatrixTpl<T>::operator+=)), "+=" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)>(&ccGLMatrixTpl<T>::operator-=)), "-=" },
						{ fun(static_cast<ccGLMatrixTpl<T>&(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)>(&ccGLMatrixTpl<T>::operator-=)), "-=" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(Vector3Tpl<float>&)const> (&ccGLMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(Vector3Tpl<double>&)const>(&ccGLMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(Tuple4Tpl<float>&)const> (&ccGLMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(Tuple4Tpl<double>&)const> (&ccGLMatrixTpl<T>::apply)), "apply" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyX)), "applyX" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyX)), "applyX" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyY)), "applyY" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyY)), "applyY" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Vector3Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyZ)), "applyZ" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Vector3Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyZ)), "applyZ" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Tuple4Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyX)), "applyX" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Tuple4Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyX)), "applyX" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Tuple4Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyY)), "applyY" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Tuple4Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyY)), "applyY" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Tuple4Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyZ)), "applyZ" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Tuple4Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyZ)), "applyZ" },
						{ fun(static_cast<float (ccGLMatrixTpl<T>::*)(const Tuple4Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyW)), "applyW" },
						{ fun(static_cast<double(ccGLMatrixTpl<T>::*)(const Tuple4Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyW)), "applyW" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(Vector3Tpl<float>&)const> (&ccGLMatrixTpl<T>::applyRotation)), "applyRotation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(Vector3Tpl<double>&)const>(&ccGLMatrixTpl<T>::applyRotation)), "applyRotation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(float vec[3])const> (&ccGLMatrixTpl<T>::applyRotation)), "applyRotation" },
						{ fun(static_cast<void(ccGLMatrixTpl<T>::*)(double vec[3])const>(&ccGLMatrixTpl<T>::applyRotation)), "applyRotation" },
						{ fun(&ccGLMatrixTpl<T>::shiftRotationCenter), "shiftRotationCenter" },
						{ fun(&ccGLMatrixTpl<T>::transpose), "transpose" },
						{ fun(&ccGLMatrixTpl<T>::transposed), "transposed" },
						{ fun(&ccGLMatrixTpl<T>::invert), "invert" },
						{ fun(&ccGLMatrixTpl<T>::inverse), "inverse" },
						{ fun(&ccGLMatrixTpl<T>::scaleRotation), "scaleRotation" },
						{ fun(&ccGLMatrixTpl<T>::scaleRow), "scaleRow" },
						{ fun(&ccGLMatrixTpl<T>::scaleColumn), "scaleColumn" },
						{ fun(&ccGLMatrixTpl<T>::isSerializable), "isSerializable" },
						{ fun(&ccGLMatrixTpl<T>::toFile), "toFile" },
						{ fun(&ccGLMatrixTpl<T>::fromFile), "fromFile" },
					
					}
					);
					m->add(chaiscript::base_class<ccSerializableObject, ccGLMatrixTpl<T>>());
					return m;
				}


				ModulePtr bs_ccGLMatrix(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGLMatrix>(), "ccGLMatrix");
					m->add(chaiscript::constructor<ccGLMatrix()>(), "ccGLMatrix");
					m->add(chaiscript::constructor<ccGLMatrix(const ccGLMatrixTpl<float>&)>(), "ccGLMatrix");
					m->add(chaiscript::constructor<ccGLMatrix(const float*)>(), "ccGLMatrix");
					m->add(chaiscript::constructor<ccGLMatrix(const double*)>(), "ccGLMatrix");
					m->add(chaiscript::constructor<ccGLMatrix(const Vector3Tpl<float>&, const Vector3Tpl<float>&, const Vector3Tpl<float>&, const Vector3Tpl<float>&)>(), "ccGLMatrix");
					
					m->add(chaiscript::base_class<ccGLMatrixTpl<float>, ccGLMatrix>());
					
					return m;
				}

				ModulePtr bs_ccGLMatrixd(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGLMatrixd>(), "ccGLMatrixd");
					m->add(chaiscript::constructor<ccGLMatrixd()>(), "ccGLMatrixd");
					m->add(chaiscript::constructor<ccGLMatrixd(const ccGLMatrixTpl<double>&)>(), "ccGLMatrixd");
					m->add(chaiscript::constructor<ccGLMatrixd(const float*)>(), "ccGLMatrixd");
					m->add(chaiscript::constructor<ccGLMatrixd(const double*)>(), "ccGLMatrixd");
					m->add(chaiscript::constructor<ccGLMatrixd(const Vector3Tpl<double>&, const Vector3Tpl<double>&, const Vector3Tpl<double>&, const Vector3Tpl<double>&)>(), "ccGLMatrixd");

					m->add(chaiscript::base_class<ccGLMatrixTpl<double>, ccGLMatrixd>());

					return m;
				}

				ModulePtr bs_ccInteractor(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccInteractor>(), "ccInteractor");
					m->add(fun(&ccInteractor::acceptClick), "acceptClick");
					m->add(fun(&ccInteractor::move2D), "move2D");
					m->add(fun(&ccInteractor::move3D), "move3D");
					return m;
				}

				ModulePtr bs_cc2DLabel(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<cc2DLabel>(), "cc2DLabel");
					m->add(chaiscript::constructor<cc2DLabel(QString)>(), "cc2DLabel");
					m->add(fun(&cc2DLabel::acceptClick), "acceptClick");
					m->add(fun(&cc2DLabel::move2D), "move2D");
					m->add(fun(&cc2DLabel::move3D), "move3D");
					m->add(fun(&cc2DLabel::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&cc2DLabel::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&cc2DLabel::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&cc2DLabel::getClassID), "getClassID");
					m->add(fun(&cc2DLabel::getName), "getName");
					m->add(fun(&cc2DLabel::setName), "setName");
					m->add(fun(&cc2DLabel::getUniqueID), "getUniqueID");
					m->add(fun(&cc2DLabel::setUniqueID), "setUniqueID");
					m->add(fun(&cc2DLabel::isEnabled), "isEnabled");
					m->add(fun(&cc2DLabel::setEnabled), "setEnabled");
					m->add(fun(&cc2DLabel::toggleActivation), "toggleActivation");
					m->add(fun(&cc2DLabel::isLocked), "isLocked");
					m->add(fun(&cc2DLabel::setLocked), "setLocked");
					m->add(fun(&cc2DLabel::isLeaf), "isLeaf");
					m->add(fun(&cc2DLabel::isCustom), "isCustom");
					m->add(fun(&cc2DLabel::isHierarchy), "isHierarchy");
					m->add(fun(&cc2DLabel::isKindOf), "isKindOf");
					m->add(fun(&cc2DLabel::isA), "isA");
					m->add(fun(&cc2DLabel::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&cc2DLabel::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&cc2DLabel::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&cc2DLabel::getMetaData), "getMetaData");
					m->add(fun(&cc2DLabel::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(cc2DLabel::*)(const  QString&, const QVariant&)>(&cc2DLabel::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(cc2DLabel::*)(const QVariantMap&, bool)>(&cc2DLabel::setMetaData)), "setMetaData");
					m->add(fun(&cc2DLabel::hasMetaData), "hasMetaData");
					m->add(fun(&cc2DLabel::metaData), "metaData");
					m->add(fun(&cc2DLabel::getRawName), "getRawName");
					m->add(fun(&cc2DLabel::getLabelContent), "getLabelContent");
					m->add(fun(&cc2DLabel::getTitle), "getTitle");
					m->add(fun(&cc2DLabel::setPosition), "setPosition");
					m->add(fun(&cc2DLabel::getPosition), "getPosition");
					m->add(fun(&cc2DLabel::clear), "clear");
					m->add(fun(&cc2DLabel::size), "size");
					m->add(fun(static_cast<bool(cc2DLabel::*)(ccGenericPointCloud*, unsigned, bool)>(&cc2DLabel::addPickedPoint)), "addPickedPoint");
					m->add(fun(static_cast<bool(cc2DLabel::*)(ccGenericMesh*, unsigned, const CCVector2d&, bool)>(&cc2DLabel::addPickedPoint)), "addPickedPoint");
					m->add(fun(&cc2DLabel::setCollapsed), "setCollapsed");
					m->add(fun(&cc2DLabel::isCollapsed), "isCollapsed");
					m->add(fun(&cc2DLabel::displayPointLegend), "displayPointLegend");
					m->add(fun(&cc2DLabel::isPointLegendDisplayed), "isPointLegendDisplayed");
					m->add(fun(&cc2DLabel::setDisplayedIn2D), "setDisplayedIn2D");
					m->add(fun(&cc2DLabel::isDisplayedIn2D), "isDisplayedIn2D");
						m->add(chaiscript::user_type<cc2DLabel::PickedPoint>(), "PickedPoint");
						m->add(chaiscript::constructor<cc2DLabel::PickedPoint()>(), "PickedPoint");
						m->add(chaiscript::constructor<cc2DLabel::PickedPoint(ccGenericPointCloud*, unsigned, bool)>(), "PickedPoint");
						m->add(chaiscript::constructor<cc2DLabel::PickedPoint(ccGenericMesh*, unsigned, const CCVector2d&, bool)>(), "PickedPoint");
						m->add(fun(&cc2DLabel::PickedPoint::_cloud), "_cloud");
						m->add(fun(&cc2DLabel::PickedPoint::_mesh), "_mesh");
						m->add(fun(&cc2DLabel::PickedPoint::index), "index");
						m->add(fun(&cc2DLabel::PickedPoint::pos2D), "pos2D");
						m->add(fun(&cc2DLabel::PickedPoint::uv), "uv");
						m->add(fun(&cc2DLabel::PickedPoint::entityCenterPoint), "entityCenterPoint");
						m->add(fun(&cc2DLabel::PickedPoint::getPointPosition), "getPointPosition");
						m->add(fun(&cc2DLabel::PickedPoint::cloudOrVertices), "cloudOrVertices");
						m->add(fun(&cc2DLabel::PickedPoint::getUniqueID), "getUniqueID");
						m->add(fun(&cc2DLabel::PickedPoint::entity), "entity");
						m->add(fun(&cc2DLabel::PickedPoint::itemTitle), "itemTitle");
						m->add(fun(&cc2DLabel::PickedPoint::prefix), "prefix");
					m->add(fun(static_cast<bool(cc2DLabel::*)(const cc2DLabel::PickedPoint&)>(&cc2DLabel::addPickedPoint)), "addPickedPoint");
					m->add(fun(&cc2DLabel::getPickedPoint), "getPickedPoint");
					m->add(fun(&cc2DLabel::setRelativeMarkerScale), "setRelativeMarkerScale");

					m->add(chaiscript::base_class<ccInteractor, cc2DLabel>());
					m->add(chaiscript::base_class<ccHObject, cc2DLabel>());
					m->add(chaiscript::base_class<ccObject, cc2DLabel>());
					m->add(chaiscript::base_class<ccSerializableObject, cc2DLabel>());
					m->add(chaiscript::base_class<ccDrawableObject, cc2DLabel>());

					return m;
				}

				ModulePtr bs_cc2DViewportObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<cc2DViewportObject>(), "cc2DViewportObject");
					m->add(chaiscript::constructor<cc2DViewportObject(QString)>(), "cc2DViewportObject");
					m->add(fun(&cc2DViewportObject::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&cc2DViewportObject::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&cc2DViewportObject::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&cc2DViewportObject::getClassID), "getClassID");
					m->add(fun(&cc2DViewportObject::getName), "getName");
					m->add(fun(&cc2DViewportObject::setName), "setName");
					m->add(fun(&cc2DViewportObject::getUniqueID), "getUniqueID");
					m->add(fun(&cc2DViewportObject::setUniqueID), "setUniqueID");
					m->add(fun(&cc2DViewportObject::isEnabled), "isEnabled");
					m->add(fun(&cc2DViewportObject::setEnabled), "setEnabled");
					m->add(fun(&cc2DViewportObject::toggleActivation), "toggleActivation");
					m->add(fun(&cc2DViewportObject::isLocked), "isLocked");
					m->add(fun(&cc2DViewportObject::setLocked), "setLocked");
					m->add(fun(&cc2DViewportObject::isLeaf), "isLeaf");
					m->add(fun(&cc2DViewportObject::isCustom), "isCustom");
					m->add(fun(&cc2DViewportObject::isHierarchy), "isHierarchy");
					m->add(fun(&cc2DViewportObject::isKindOf), "isKindOf");
					m->add(fun(&cc2DViewportObject::isA), "isA");
					m->add(fun(&cc2DViewportObject::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&cc2DViewportObject::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&cc2DViewportObject::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&cc2DViewportObject::getMetaData), "getMetaData");
					m->add(fun(&cc2DViewportObject::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(cc2DViewportObject::*)(const  QString&, const QVariant&)>(&cc2DViewportObject::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(cc2DViewportObject::*)(const QVariantMap&, bool)>(&cc2DViewportObject::setMetaData)), "setMetaData");
					m->add(fun(&cc2DViewportObject::hasMetaData), "hasMetaData");
					m->add(fun(&cc2DViewportObject::metaData), "metaData");
					m->add(fun(&cc2DViewportObject::setParameters), "setParameters");
					m->add(fun(&cc2DViewportObject::getParameters), "getParameters");

					m->add(chaiscript::base_class<ccHObject, cc2DViewportObject>());
					m->add(chaiscript::base_class<ccObject, cc2DViewportObject>());
					m->add(chaiscript::base_class<ccSerializableObject, cc2DViewportObject>());
					m->add(chaiscript::base_class<ccDrawableObject, cc2DViewportObject>());

					return m;
				}

				ModulePtr bs_cc2DViewportLabel(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<cc2DViewportLabel>(), "cc2DViewportLabel");
					m->add(chaiscript::constructor<cc2DViewportLabel(QString)>(), "cc2DViewportLabel");
					m->add(fun(&cc2DViewportLabel::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&cc2DViewportLabel::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&cc2DViewportLabel::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&cc2DViewportLabel::getClassID), "getClassID");
					m->add(fun(&cc2DViewportLabel::getName), "getName");
					m->add(fun(&cc2DViewportLabel::setName), "setName");
					m->add(fun(&cc2DViewportLabel::getUniqueID), "getUniqueID");
					m->add(fun(&cc2DViewportLabel::setUniqueID), "setUniqueID");
					m->add(fun(&cc2DViewportLabel::isEnabled), "isEnabled");
					m->add(fun(&cc2DViewportLabel::setEnabled), "setEnabled");
					m->add(fun(&cc2DViewportLabel::toggleActivation), "toggleActivation");
					m->add(fun(&cc2DViewportLabel::isLocked), "isLocked");
					m->add(fun(&cc2DViewportLabel::setLocked), "setLocked");
					m->add(fun(&cc2DViewportLabel::isLeaf), "isLeaf");
					m->add(fun(&cc2DViewportLabel::isCustom), "isCustom");
					m->add(fun(&cc2DViewportLabel::isHierarchy), "isHierarchy");
					m->add(fun(&cc2DViewportLabel::isKindOf), "isKindOf");
					m->add(fun(&cc2DViewportLabel::isA), "isA");
					m->add(fun(&cc2DViewportLabel::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&cc2DViewportLabel::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&cc2DViewportLabel::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&cc2DViewportLabel::getMetaData), "getMetaData");
					m->add(fun(&cc2DViewportLabel::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(cc2DViewportLabel::*)(const  QString&, const QVariant&)>(&cc2DViewportLabel::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(cc2DViewportLabel::*)(const QVariantMap&, bool)>(&cc2DViewportLabel::setMetaData)), "setMetaData");
					m->add(fun(&cc2DViewportLabel::hasMetaData), "hasMetaData");
					m->add(fun(&cc2DViewportLabel::metaData), "metaData");
					m->add(fun(&cc2DViewportLabel::setParameters), "setParameters");
					m->add(fun(&cc2DViewportLabel::getParameters), "getParameters");
					m->add(fun(&cc2DViewportLabel::roi), "roi");
					m->add(fun(&cc2DViewportLabel::setRoi), "setRoi");

					m->add(chaiscript::base_class<cc2DViewportObject, cc2DViewportLabel>());
					m->add(chaiscript::base_class<ccHObject, cc2DViewportLabel>());
					m->add(chaiscript::base_class<ccObject, cc2DViewportLabel>());
					m->add(chaiscript::base_class<ccSerializableObject, cc2DViewportLabel>());
					m->add(chaiscript::base_class<ccDrawableObject, cc2DViewportLabel>());

					return m;
				}

				ModulePtr bs_ccSensor(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccSensor>(), "ccSensor");
					m->add(chaiscript::constructor<ccSensor(const QString&)>(), "ccSensor");
					m->add(chaiscript::constructor<ccSensor(const ccSensor&)>(), "ccSensor");
					m->add(fun(&ccSensor::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccSensor::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccSensor::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccSensor::getClassID), "getClassID");
					m->add(fun(&ccSensor::getName), "getName");
					m->add(fun(&ccSensor::setName), "setName");
					m->add(fun(&ccSensor::getUniqueID), "getUniqueID");
					m->add(fun(&ccSensor::setUniqueID), "setUniqueID");
					m->add(fun(&ccSensor::isEnabled), "isEnabled");
					m->add(fun(&ccSensor::setEnabled), "setEnabled");
					m->add(fun(&ccSensor::toggleActivation), "toggleActivation");
					m->add(fun(&ccSensor::isLocked), "isLocked");
					m->add(fun(&ccSensor::setLocked), "setLocked");
					m->add(fun(&ccSensor::isLeaf), "isLeaf");
					m->add(fun(&ccSensor::isCustom), "isCustom");
					m->add(fun(&ccSensor::isHierarchy), "isHierarchy");
					m->add(fun(&ccSensor::isKindOf), "isKindOf");
					m->add(fun(&ccSensor::isA), "isA");
					m->add(fun(&ccSensor::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccSensor::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccSensor::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccSensor::getMetaData), "getMetaData");
					m->add(fun(&ccSensor::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccSensor::*)(const  QString&, const QVariant&)>(&ccSensor::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccSensor::*)(const QVariantMap&, bool)>(&ccSensor::setMetaData)), "setMetaData");
					m->add(fun(&ccSensor::hasMetaData), "hasMetaData");
					m->add(fun(&ccSensor::metaData), "metaData");
					m->add(fun(&ccSensor::getType), "getType");
					m->add(fun(&ccSensor::checkVisibility), "checkVisibility");
					m->add(fun(static_cast<ccIndexedTransformationBuffer*(ccSensor::*)()>(&ccSensor::getPositions)), "getPositions");
					m->add(fun(static_cast<const ccIndexedTransformationBuffer*(ccSensor::*)()const>(&ccSensor::getPositions)), "getPositions");
					m->add(fun(&ccSensor::setPositions), "setPositions");
					m->add(fun(&ccSensor::addPosition), "addPosition");
					m->add(fun(&ccSensor::getAbsoluteTransformation), "getAbsoluteTransformation");
					m->add(fun(&ccSensor::getActiveAbsoluteTransformation), "getActiveAbsoluteTransformation");
					m->add(fun(&ccSensor::getActiveAbsoluteCenter), "getActiveAbsoluteCenter");
					m->add(fun(&ccSensor::getActiveAbsoluteRotation), "getActiveAbsoluteRotation");
					m->add(fun(&ccSensor::setRigidTransformation), "setRigidTransformation");
					m->add(fun(static_cast<ccGLMatrix& (ccSensor::*)()>(&ccSensor::getRigidTransformation)), "getRigidTransformation");
					m->add(fun(static_cast<const ccGLMatrix& (ccSensor::*)()const>(&ccSensor::getRigidTransformation)), "getRigidTransformation");
					m->add(fun(&ccSensor::getIndexBounds), "getIndexBounds");
					m->add(fun(&ccSensor::getActiveIndex), "getActiveIndex");
					m->add(fun(&ccSensor::setActiveIndex), "setActiveIndex");
					m->add(fun(&ccSensor::setGraphicScale), "setGraphicScale");
					m->add(fun(&ccSensor::getGraphicScale), "getGraphicScale");
					m->add(fun(&ccSensor::applyViewport), "applyViewport");
					m->add(fun(&ccSensor::applyGLTransformation), "applyGLTransformation");

					m->add(chaiscript::base_class<ccHObject, ccSensor>());
					m->add(chaiscript::base_class<ccObject, ccSensor>());
					m->add(chaiscript::base_class<ccSerializableObject, ccSensor>());
					m->add(chaiscript::base_class<ccDrawableObject, ccSensor>());

					return m;
				}

				ModulePtr bs_ccCameraSensor(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccCameraSensor>(), "ccCameraSensor");
					m->add(chaiscript::constructor<ccCameraSensor()>(), "ccCameraSensor");
					m->add(chaiscript::constructor<ccCameraSensor(const ccCameraSensor&)>(), "ccCameraSensor");
					m->add(chaiscript::constructor<ccCameraSensor(const ccCameraSensor::IntrinsicParameters&)>(), "ccCameraSensor");
					m->add(fun(&ccCameraSensor::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccCameraSensor::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccCameraSensor::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccCameraSensor::getClassID), "getClassID");
					m->add(fun(&ccCameraSensor::getName), "getName");
					m->add(fun(&ccCameraSensor::setName), "setName");
					m->add(fun(&ccCameraSensor::getUniqueID), "getUniqueID");
					m->add(fun(&ccCameraSensor::setUniqueID), "setUniqueID");
					m->add(fun(&ccCameraSensor::isEnabled), "isEnabled");
					m->add(fun(&ccCameraSensor::setEnabled), "setEnabled");
					m->add(fun(&ccCameraSensor::toggleActivation), "toggleActivation");
					m->add(fun(&ccCameraSensor::isLocked), "isLocked");
					m->add(fun(&ccCameraSensor::setLocked), "setLocked");
					m->add(fun(&ccCameraSensor::isLeaf), "isLeaf");
					m->add(fun(&ccCameraSensor::isCustom), "isCustom");
					m->add(fun(&ccCameraSensor::isHierarchy), "isHierarchy");
					m->add(fun(&ccCameraSensor::isKindOf), "isKindOf");
					m->add(fun(&ccCameraSensor::isA), "isA");
					m->add(fun(&ccCameraSensor::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccCameraSensor::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccCameraSensor::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccCameraSensor::getMetaData), "getMetaData");
					m->add(fun(&ccCameraSensor::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccCameraSensor::*)(const  QString&, const QVariant&)>(&ccCameraSensor::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccCameraSensor::*)(const QVariantMap&, bool)>(&ccCameraSensor::setMetaData)), "setMetaData");
					m->add(fun(&ccCameraSensor::hasMetaData), "hasMetaData");
					m->add(fun(&ccCameraSensor::metaData), "metaData");
					m->add(fun(&ccCameraSensor::getType), "getType");
					m->add(fun(&ccCameraSensor::checkVisibility), "checkVisibility");
					m->add(fun(static_cast<ccIndexedTransformationBuffer * (ccCameraSensor::*)()>(&ccCameraSensor::getPositions)), "getPositions");
					m->add(fun(static_cast<const ccIndexedTransformationBuffer * (ccCameraSensor::*)()const>(&ccCameraSensor::getPositions)), "getPositions");
					m->add(fun(&ccCameraSensor::setPositions), "setPositions");
					m->add(fun(&ccCameraSensor::addPosition), "addPosition");
					m->add(fun(&ccCameraSensor::getAbsoluteTransformation), "getAbsoluteTransformation");
					m->add(fun(&ccCameraSensor::getActiveAbsoluteTransformation), "getActiveAbsoluteTransformation");
					m->add(fun(&ccCameraSensor::getActiveAbsoluteCenter), "getActiveAbsoluteCenter");
					m->add(fun(&ccCameraSensor::getActiveAbsoluteRotation), "getActiveAbsoluteRotation");
					m->add(fun(&ccCameraSensor::setRigidTransformation), "setRigidTransformation");
					m->add(fun(static_cast<ccGLMatrix & (ccCameraSensor::*)()>(&ccCameraSensor::getRigidTransformation)), "getRigidTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccCameraSensor::*)()const>(&ccCameraSensor::getRigidTransformation)), "getRigidTransformation");
					m->add(fun(&ccCameraSensor::getIndexBounds), "getIndexBounds");
					m->add(fun(&ccCameraSensor::getActiveIndex), "getActiveIndex");
					m->add(fun(&ccCameraSensor::setActiveIndex), "setActiveIndex");
					m->add(fun(&ccCameraSensor::setGraphicScale), "setGraphicScale");
					m->add(fun(&ccCameraSensor::getGraphicScale), "getGraphicScale");
					m->add(fun(&ccCameraSensor::applyViewport), "applyViewport");
					m->add(fun(&ccCameraSensor::applyGLTransformation), "applyGLTransformation");
						m->add(chaiscript::user_type<ccCameraSensor::IntrinsicParameters>(), "IntrinsicParameters");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::GetKinectDefaults), "GetKinectDefaults");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::vertFocal_pix), "vertFocal_pix");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::pixelSize_mm), "pixelSize_mm");						
						m->add(fun(&ccCameraSensor::IntrinsicParameters::skew), "skew");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::vFOV_rad), "vFOV_rad");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::zNear_mm), "zNear_mm");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::zFar_mm), "zFar_mm");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::arrayWidth), "arrayWidth");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::arrayHeight), "arrayHeight");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::principal_point), "principal_point");
						m->add(fun(&ccCameraSensor::IntrinsicParameters::horizFocal_pix), "horizFocal_pix");
						m->add(chaiscript::user_type<ccCameraSensor::LensDistortionParameters>(), "LensDistortionParameters");
						m->add(fun(&ccCameraSensor::LensDistortionParameters::getModel), "getModel");
						m->add(chaiscript::user_type<ccCameraSensor::LensDistortionParameters::Shared>(), "LensDistortionParameters_Shared");
						
						m->add(chaiscript::user_type<ccCameraSensor::RadialDistortionParameters>(), "RadialDistortionParameters");
						m->add(fun(&ccCameraSensor::RadialDistortionParameters::getModel), "getModel");
						m->add(chaiscript::user_type<ccCameraSensor::RadialDistortionParameters::Shared>(), "RadialDistortionParameters_Shared");
						m->add(fun(&ccCameraSensor::RadialDistortionParameters::k1), "k1");
						m->add(fun(&ccCameraSensor::RadialDistortionParameters::k2), "k2");
						m->add(chaiscript::base_class<ccCameraSensor::LensDistortionParameters, ccCameraSensor::RadialDistortionParameters>());

						m->add(chaiscript::user_type<ccCameraSensor::ExtendedRadialDistortionParameters>(), "ExtendedRadialDistortionParameters");
						m->add(fun(&ccCameraSensor::ExtendedRadialDistortionParameters::getModel), "getModel");
						m->add(chaiscript::user_type<ccCameraSensor::ExtendedRadialDistortionParameters::Shared>(), "ExtendedRadialDistortionParameters_Shared");
						m->add(fun(&ccCameraSensor::ExtendedRadialDistortionParameters::k1), "k1");
						m->add(fun(&ccCameraSensor::ExtendedRadialDistortionParameters::k2), "k2");
						m->add(fun(&ccCameraSensor::ExtendedRadialDistortionParameters::k3), "k3");
						m->add(chaiscript::base_class<ccCameraSensor::LensDistortionParameters, ccCameraSensor::ExtendedRadialDistortionParameters>());
						m->add(chaiscript::base_class<ccCameraSensor::RadialDistortionParameters, ccCameraSensor::ExtendedRadialDistortionParameters>());

						m->add(chaiscript::user_type<ccCameraSensor::BrownDistortionParameters>(), "BrownDistortionParameters");
						m->add(fun(&ccCameraSensor::BrownDistortionParameters::getModel), "getModel");
						m->add(chaiscript::user_type<ccCameraSensor::BrownDistortionParameters::Shared>(), "BrownDistortionParameters_Shared");
						m->add(fun(&ccCameraSensor::BrownDistortionParameters::GetKinectDefaults), "GetKinectDefaults");
						m->add(fun(&ccCameraSensor::BrownDistortionParameters::principalPointOffset), "principalPointOffset");
						m->add(fun(&ccCameraSensor::BrownDistortionParameters::linearDisparityParams), "linearDisparityParams");
						m->add(fun(&ccCameraSensor::BrownDistortionParameters::K_BrownParams), "K_BrownParams");
						m->add(fun(&ccCameraSensor::BrownDistortionParameters::P_BrownParams), "P_BrownParams");
						m->add(chaiscript::base_class<ccCameraSensor::LensDistortionParameters, ccCameraSensor::BrownDistortionParameters>());

						m->add(chaiscript::user_type<ccCameraSensor::FrustumInformation>(), "FrustumInformation");
						m->add(fun(&ccCameraSensor::FrustumInformation::initFrustumCorners), "initFrustumCorners");
						m->add(fun(&ccCameraSensor::FrustumInformation::initFrustumHull), "initFrustumHull");
						m->add(fun(&ccCameraSensor::FrustumInformation::isComputed), "isComputed");
						m->add(fun(&ccCameraSensor::FrustumInformation::drawFrustum), "drawFrustum");
						m->add(fun(&ccCameraSensor::FrustumInformation::drawSidePlanes), "drawSidePlanes");
						m->add(fun(&ccCameraSensor::FrustumInformation::frustumCorners), "frustumCorners");
						m->add(fun(&ccCameraSensor::FrustumInformation::frustumHull), "frustumHull");
						m->add(fun(&ccCameraSensor::FrustumInformation::center), "center");
					m->add(fun(&ccCameraSensor::setVertFocal_pix), "setVertFocal_pix");
					m->add(fun(&ccCameraSensor::getVertFocal_pix), "getVertFocal_pix");
					m->add(fun(&ccCameraSensor::getHorizFocal_pix), "getHorizFocal_pix");
					m->add(fun(&ccCameraSensor::setVerticalFov_rad), "setVerticalFov_rad");
					m->add(fun(&ccCameraSensor::getVerticalFov_rad), "getVerticalFov_rad");
					m->add(fun(&ccCameraSensor::getIntrinsicParameters), "getIntrinsicParameters");
					m->add(fun(&ccCameraSensor::setIntrinsicParameters), "setIntrinsicParameters");
					m->add(fun(&ccCameraSensor::getDistortionParameters), "getDistortionParameters");
					m->add(fun(&ccCameraSensor::setDistortionParameters), "setDistortionParameters");
					m->add(fun(&ccCameraSensor::getProjectionMatrix), "getProjectionMatrix");
					m->add(fun(&ccCameraSensor::frustumIsDrawn), "frustumIsDrawn");
					m->add(fun(&ccCameraSensor::drawFrustum), "drawFrustum");
					m->add(fun(&ccCameraSensor::frustumPlanesAreDrawn), "frustumPlanesAreDrawn");
					m->add(fun(&ccCameraSensor::drawFrustumPlanes), "drawFrustumPlanes");
					m->add(fun(&ccCameraSensor::fromLocalCoordToGlobalCoord), "fromLocalCoordToGlobalCoord");
					m->add(fun(&ccCameraSensor::fromGlobalCoordToLocalCoord), "fromGlobalCoordToLocalCoord");
					m->add(fun(&ccCameraSensor::fromLocalCoordToImageCoord), "fromLocalCoordToImageCoord");
					m->add(fun(&ccCameraSensor::fromImageCoordToLocalCoord), "fromImageCoordToLocalCoord");
					m->add(fun(&ccCameraSensor::fromGlobalCoordToImageCoord), "fromGlobalCoordToImageCoord");
					m->add(fun(&ccCameraSensor::fromImageCoordToGlobalCoord), "fromImageCoordToGlobalCoord");
					m->add(fun(&ccCameraSensor::fromRealImCoordToIdealImCoord), "fromRealImCoordToIdealImCoord");
					m->add(fun(&ccCameraSensor::orthoRectifyAsCloud), "orthoRectifyAsCloud");
					m->add(fun(&ccCameraSensor::orthoRectifyAsImage), "orthoRectifyAsImage");
					m->add(fun(&ccCameraSensor::orthoRectifyAsImageDirect), "orthoRectifyAsImageDirect");
					m->add(fun(&ccCameraSensor::OrthoRectifyAsImages), "OrthoRectifyAsImages");
					m->add(fun(&ccCameraSensor::computeOrthoRectificationParams), "computeOrthoRectificationParams");
					m->add(fun(static_cast<bool(ccCameraSensor::*)(const CCVector2&, const float, Vector3Tpl<ScalarType>&)const>(&ccCameraSensor::computeUncertainty)), "computeUncertainty");
					m->add(fun(static_cast<bool(ccCameraSensor::*)(CCLib::ReferenceCloud*, std::vector<Vector3Tpl<ScalarType>>&)>(&ccCameraSensor::computeUncertainty)), "computeUncertainty");
					m->add(fun(static_cast<QImage(ccCameraSensor::*)(const QImage&)const>(&ccCameraSensor::undistort)), "undistort");
					m->add(fun(static_cast<ccImage*(ccCameraSensor::*)(ccImage*, bool)const>(&ccCameraSensor::undistort)), "undistort");
					m->add(fun(&ccCameraSensor::isGlobalCoordInFrustum), "isGlobalCoordInFrustum");
					m->add(fun(&ccCameraSensor::computeGlobalPlaneCoefficients), "computeGlobalPlaneCoefficients");
					m->add(fun(&ccCameraSensor::ConvertFocalPixToMM), "ConvertFocalPixToMM");
					m->add(fun(&ccCameraSensor::ConvertFocalMMToPix), "ConvertFocalMMToPix");
					m->add(fun(&ccCameraSensor::ComputeFovRadFromFocalPix), "ComputeFovRadFromFocalPix");
					m->add(fun(&ccCameraSensor::ComputeFovRadFromFocalMm), "ComputeFovRadFromFocalMm");
					
					m->add(chaiscript::base_class<ccHObject, ccCameraSensor>());
					m->add(chaiscript::base_class<ccObject, ccCameraSensor>());
					m->add(chaiscript::base_class<ccSerializableObject, ccCameraSensor>());
					m->add(chaiscript::base_class<ccDrawableObject, ccCameraSensor>());
					m->add(chaiscript::base_class<ccSensor, ccCameraSensor>());

					return m;
				}

				
				ModulePtr bs_ccGBLSensor(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGBLSensor>(), "ccGBLSensor");
					m->add(chaiscript::constructor<ccGBLSensor()>(), "ccGBLSensor");
					m->add(chaiscript::constructor<ccGBLSensor(const ccGBLSensor&)>(), "ccGBLSensor");
					m->add(chaiscript::constructor<ccGBLSensor(const ccGBLSensor::ROTATION_ORDER)>(), "ccGBLSensor");
					m->add(fun(&ccGBLSensor::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccGBLSensor::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccGBLSensor::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccGBLSensor::getClassID), "getClassID");
					m->add(fun(&ccGBLSensor::getName), "getName");
					m->add(fun(&ccGBLSensor::setName), "setName");
					m->add(fun(&ccGBLSensor::getUniqueID), "getUniqueID");
					m->add(fun(&ccGBLSensor::setUniqueID), "setUniqueID");
					m->add(fun(&ccGBLSensor::isEnabled), "isEnabled");
					m->add(fun(&ccGBLSensor::setEnabled), "setEnabled");
					m->add(fun(&ccGBLSensor::toggleActivation), "toggleActivation");
					m->add(fun(&ccGBLSensor::isLocked), "isLocked");
					m->add(fun(&ccGBLSensor::setLocked), "setLocked");
					m->add(fun(&ccGBLSensor::isLeaf), "isLeaf");
					m->add(fun(&ccGBLSensor::isCustom), "isCustom");
					m->add(fun(&ccGBLSensor::isHierarchy), "isHierarchy");
					m->add(fun(&ccGBLSensor::isKindOf), "isKindOf");
					m->add(fun(&ccGBLSensor::isA), "isA");
					m->add(fun(&ccGBLSensor::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccGBLSensor::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccGBLSensor::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccGBLSensor::getMetaData), "getMetaData");
					m->add(fun(&ccGBLSensor::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccGBLSensor::*)(const  QString&, const QVariant&)>(&ccGBLSensor::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccGBLSensor::*)(const QVariantMap&, bool)>(&ccGBLSensor::setMetaData)), "setMetaData");
					m->add(fun(&ccGBLSensor::hasMetaData), "hasMetaData");
					m->add(fun(&ccGBLSensor::metaData), "metaData");
					m->add(fun(&ccGBLSensor::getType), "getType");
					m->add(fun(&ccGBLSensor::checkVisibility), "checkVisibility");
					m->add(fun(static_cast<ccIndexedTransformationBuffer * (ccGBLSensor::*)()>(&ccGBLSensor::getPositions)), "getPositions");
					m->add(fun(static_cast<const ccIndexedTransformationBuffer * (ccGBLSensor::*)()const>(&ccGBLSensor::getPositions)), "getPositions");
					m->add(fun(&ccGBLSensor::setPositions), "setPositions");
					m->add(fun(&ccGBLSensor::addPosition), "addPosition");
					m->add(fun(&ccGBLSensor::getAbsoluteTransformation), "getAbsoluteTransformation");
					m->add(fun(&ccGBLSensor::getActiveAbsoluteTransformation), "getActiveAbsoluteTransformation");
					m->add(fun(&ccGBLSensor::getActiveAbsoluteCenter), "getActiveAbsoluteCenter");
					m->add(fun(&ccGBLSensor::getActiveAbsoluteRotation), "getActiveAbsoluteRotation");
					m->add(fun(&ccGBLSensor::setRigidTransformation), "setRigidTransformation");
					m->add(fun(static_cast<ccGLMatrix & (ccGBLSensor::*)()>(&ccGBLSensor::getRigidTransformation)), "getRigidTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccGBLSensor::*)()const>(&ccGBLSensor::getRigidTransformation)), "getRigidTransformation");
					m->add(fun(&ccGBLSensor::getIndexBounds), "getIndexBounds");
					m->add(fun(&ccGBLSensor::getActiveIndex), "getActiveIndex");
					m->add(fun(&ccGBLSensor::setActiveIndex), "setActiveIndex");
					m->add(fun(&ccGBLSensor::setGraphicScale), "setGraphicScale");
					m->add(fun(&ccGBLSensor::getGraphicScale), "getGraphicScale");
					m->add(fun(&ccGBLSensor::applyViewport), "applyViewport");
					m->add(fun(&ccGBLSensor::applyGLTransformation), "applyGLTransformation");


					m->add(chaiscript::base_class<ccHObject, ccGBLSensor>());
					m->add(chaiscript::base_class<ccObject, ccGBLSensor>());
					m->add(chaiscript::base_class<ccSerializableObject, ccGBLSensor>());
					m->add(chaiscript::base_class<ccDrawableObject, ccGBLSensor>());
					m->add(chaiscript::base_class<ccSensor, ccGBLSensor>());
					

					return m;
				}

				ModulePtr bs_ccPlanarEntityInterface(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccPlanarEntityInterface>(), "ccPlanarEntityInterface");

					m->add(fun(&ccPlanarEntityInterface::showNormalVector), "showNormalVector");
					m->add(fun(&ccPlanarEntityInterface::normalVectorIsShown), "normalVectorIsShown");
					m->add(fun(&ccPlanarEntityInterface::getNormal), "getNormal");
					return m;
				}


				ModulePtr bs_ccDrawableObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccClipPlane>(), "ccClipPlane");
					m->add(fun(&ccClipPlane::equation), "equation");
					m->add(chaiscript::vector_conversion<ccClipPlaneSet>());
					m->add(chaiscript::user_type<ccClipPlaneSet>(), "ccClipPlaneSet");


					m->add(chaiscript::user_type<ccDrawableObject>(), "ccDrawableObject");
					m->add(fun(&ccDrawableObject::draw), "draw");
					m->add(fun(&ccDrawableObject::isVisible), "isVisible");
					m->add(fun(&ccDrawableObject::setVisible), "setVisible");
					m->add(fun(&ccDrawableObject::toggleVisibility), "toggleVisibility");
					m->add(fun(&ccDrawableObject::isVisiblityLocked), "isVisiblityLocked");
					m->add(fun(&ccDrawableObject::lockVisibility), "lockVisibility");
					m->add(fun(&ccDrawableObject::isSelected), "isSelected");
					m->add(fun(&ccDrawableObject::setSelected), "setSelected");
					m->add(fun(&ccDrawableObject::getDrawingParameters), "getDrawingParameters");
					m->add(fun(&ccDrawableObject::hasColors), "hasColors");
					m->add(fun(&ccDrawableObject::colorsShown), "colorsShown");
					m->add(fun(&ccDrawableObject::showColors), "showColors");
					m->add(fun(&ccDrawableObject::toggleColors), "toggleColors");
					m->add(fun(&ccDrawableObject::hasNormals), "hasNormals");
					m->add(fun(&ccDrawableObject::normalsShown), "normalsShown");
					m->add(fun(&ccDrawableObject::showNormals), "showNormals");
					m->add(fun(&ccDrawableObject::toggleNormals), "toggleNormals");
					m->add(fun(&ccDrawableObject::hasDisplayedScalarField), "hasDisplayedScalarField");
					m->add(fun(&ccDrawableObject::hasScalarFields), "hasScalarFields");
					m->add(fun(&ccDrawableObject::showSF), "showSF");
					m->add(fun(&ccDrawableObject::toggleSF), "toggleSF");
					m->add(fun(&ccDrawableObject::sfShown), "sfShown");
					m->add(fun(&ccDrawableObject::toggleMaterials), "toggleMaterials");
					m->add(fun(&ccDrawableObject::showNameIn3D), "showNameIn3D");
					m->add(fun(&ccDrawableObject::nameShownIn3D), "nameShownIn3D");
					m->add(fun(&ccDrawableObject::toggleShowName), "toggleShowName");
					m->add(fun(&ccDrawableObject::isColorOverriden), "isColorOverriden");
					m->add(fun(&ccDrawableObject::getTempColor), "getTempColor");
					m->add(fun(static_cast<void(ccDrawableObject::*)(const ccColor::Rgba&, bool)>(&ccDrawableObject::setTempColor)), "setTempColor");
					m->add(fun(static_cast<void(ccDrawableObject::*)(const ccColor::Rgb&, bool)>(&ccDrawableObject::setTempColor)), "setTempColor");
					m->add(fun(&ccDrawableObject::enableTempColor), "enableTempColor");
					m->add(fun(&ccDrawableObject::removeFromDisplay), "removeFromDisplay");
					m->add(fun(&ccDrawableObject::setDisplay), "setDisplay");
					m->add(fun(&ccDrawableObject::getDisplay), "getDisplay");
					m->add(fun(&ccDrawableObject::redrawDisplay), "redrawDisplay");
					m->add(fun(&ccDrawableObject::prepareDisplayForRefresh), "prepareDisplayForRefresh");
					m->add(fun(&ccDrawableObject::refreshDisplay), "refreshDisplay");
					m->add(fun(&ccDrawableObject::setGLTransformation), "setGLTransformation");
					m->add(fun(&ccDrawableObject::enableGLTransformation), "enableGLTransformation");
					m->add(fun(&ccDrawableObject::isGLTransEnabled), "isGLTransEnabled");
					m->add(fun(&ccDrawableObject::getGLTransformation), "getGLTransformation");
					m->add(fun(&ccDrawableObject::resetGLTransformation), "resetGLTransformation");
					m->add(fun(&ccDrawableObject::rotateGL), "rotateGL");
					m->add(fun(&ccDrawableObject::translateGL), "translateGL");
					m->add(fun(&ccDrawableObject::removeAllClipPlanes), "removeAllClipPlanes");
					m->add(fun(&ccDrawableObject::addClipPlanes), "addClipPlanes");
					m->add(fun(&ccDrawableObject::toggleClipPlanes), "toggleClipPlanes");
					return m;
				}

				ModulePtr bs_ccGenericMesh(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGenericMesh>(), "ccGenericMesh");
					m->add(fun(&ccGenericMesh::size), "size");
					m->add(fun(&ccGenericMesh::forEach), "forEach");
					m->add(fun(&ccGenericMesh::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccGenericMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccGenericMesh::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccGenericMesh::_getTriangle), "_getTriangle");
					m->add(fun(&ccGenericMesh::getTriangleVertIndexes), "getTriangleVertIndexes");
					m->add(fun(&ccGenericMesh::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccGenericMesh::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccGenericMesh::showNormals), "showNormals");
					m->add(fun(&ccGenericMesh::isSerializable), "isSerializable");
					m->add(fun(&ccGenericMesh::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccGenericMesh::refreshBB), "refreshBB");
					m->add(fun(&ccGenericMesh::capacity), "capacity");
					m->add(fun(&ccGenericMesh::hasMaterials), "hasMaterials");
					m->add(fun(&ccGenericMesh::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccGenericMesh::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccGenericMesh::hasTextures), "hasTextures");
					m->add(fun(&ccGenericMesh::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccGenericMesh::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccGenericMesh::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccGenericMesh::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccGenericMesh::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccGenericMesh::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccGenericMesh::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccGenericMesh::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccGenericMesh::computeInterpolationWeights), "computeInterpolationWeights");
					m->add(fun(&ccGenericMesh::interpolateNormals), "interpolateNormals");
					m->add(fun(&ccGenericMesh::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccGenericMesh::*)(unsigned, const CCVector3&, ccColor::Rgba&)>(&ccGenericMesh::interpolateColors)), "interpolateColors");
					m->add(fun(static_cast<bool(ccGenericMesh::*)(unsigned, const CCVector3&, ccColor::Rgb&)>(&ccGenericMesh::interpolateColors)), "interpolateColors");
					m->add(fun(static_cast<bool(ccGenericMesh::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccGenericMesh::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccGenericMesh::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccGenericMesh::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccGenericMesh::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccGenericMesh::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccGenericMesh::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccGenericMesh::showWired), "showWired");
					m->add(fun(&ccGenericMesh::triNormsShown), "triNormsShown");
					m->add(fun(&ccGenericMesh::showTriNorms), "showTriNorms");
					m->add(fun(&ccGenericMesh::materialsShown), "materialsShown");
					m->add(fun(&ccGenericMesh::showMaterials), "showMaterials");
					m->add(fun(&ccGenericMesh::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccGenericMesh::enableStippling), "enableStippling");
					m->add(fun(&ccGenericMesh::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccGenericMesh::importParametersFrom), "importParametersFrom");
					//m->add(fun(&ccGenericMesh::trianglePicking), "trianglePicking"); //TODO add default version(s)
					m->add(fun(&ccGenericMesh::computePointPosition), "computePointPosition");
					
					m->add(chaiscript::base_class<ccObject, ccGenericMesh>());
					m->add(chaiscript::base_class<ccDrawableObject, ccGenericMesh>());
					m->add(chaiscript::base_class<ccHObject, ccGenericMesh>());
					m->add(chaiscript::base_class<ccSerializableObject, ccGenericMesh>());
					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccGenericMesh>());


					return m;
				}

				ModulePtr bs_ccMesh(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccMesh>(), "ccMesh");
					m->add(chaiscript::constructor<ccMesh(ccGenericPointCloud*)>(), "ccMesh");
					m->add(chaiscript::constructor<ccMesh(CCLib::GenericIndexedMesh*, ccGenericPointCloud*)>(), "ccMesh");
					m->add(fun(&ccMesh::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccMesh::cloneMesh), "cloneMesh");
					m->add(fun(&ccMesh::Triangulate), "Triangulate");
					m->add(fun(&ccMesh::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccMesh::merge), "merge");
					m->add(fun(&ccMesh::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccMesh::flipTriangles), "flipTriangles");
					m->add(fun(&ccMesh::addTriangle), "addTriangle");
					m->add(fun(&ccMesh::reserve), "reserve");
					m->add(fun(&ccMesh::resize), "resize");
					m->add(fun(&ccMesh::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccMesh::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccMesh::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccMesh::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccMesh::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccMesh::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccMesh::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccMesh::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccMesh::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccMesh::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccMesh::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccMesh::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccMesh::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccMesh::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccMesh::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccMesh::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccMesh::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccMesh::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccMesh::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccMesh::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccMesh::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccMesh::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccMesh::computeNormals), "computeNormals");
					m->add(fun(&ccMesh::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccMesh::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccMesh::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccMesh::processScalarField), "processScalarField");
					m->add(fun(&ccMesh::subdivide), "subdivide");
					m->add(fun(&ccMesh::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccMesh::swapTriangles), "swapTriangles");
					m->add(fun(&ccMesh::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccMesh::size), "size");
					m->add(fun(&ccMesh::forEach), "forEach");
					m->add(fun(&ccMesh::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccMesh::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccMesh::_getTriangle), "_getTriangle");
					m->add(fun(&ccMesh::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccMesh::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccMesh::showNormals), "showNormals");
					m->add(fun(&ccMesh::isSerializable), "isSerializable");
					m->add(fun(&ccMesh::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccMesh::refreshBB), "refreshBB");
					m->add(fun(&ccMesh::capacity), "capacity");
					m->add(fun(&ccMesh::hasMaterials), "hasMaterials");
					m->add(fun(&ccMesh::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccMesh::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccMesh::hasTextures), "hasTextures");
					m->add(fun(&ccMesh::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccMesh::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccMesh::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccMesh::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccMesh::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccMesh::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccMesh::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccMesh::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccMesh::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccMesh::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccMesh::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccMesh::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccMesh::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccMesh::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccMesh::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccMesh::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccMesh::showWired), "showWired");
					m->add(fun(&ccMesh::triNormsShown), "triNormsShown");
					m->add(fun(&ccMesh::showTriNorms), "showTriNorms");
					m->add(fun(&ccMesh::materialsShown), "materialsShown");
					m->add(fun(&ccMesh::showMaterials), "showMaterials");
					m->add(fun(&ccMesh::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccMesh::enableStippling), "enableStippling");
					m->add(fun(&ccMesh::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccMesh::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccMesh::computePointPosition), "computePointPosition");
					m->add(fun(&ccMesh::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccMesh::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccMesh::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccMesh::getClassID), "getClassID");
					m->add(fun(&ccMesh::getName), "getName");
					m->add(fun(&ccMesh::setName), "setName");
					m->add(fun(&ccMesh::getUniqueID), "getUniqueID");
					m->add(fun(&ccMesh::setUniqueID), "setUniqueID");
					m->add(fun(&ccMesh::isEnabled), "isEnabled");
					m->add(fun(&ccMesh::setEnabled), "setEnabled");
					m->add(fun(&ccMesh::toggleActivation), "toggleActivation");
					m->add(fun(&ccMesh::isLocked), "isLocked");
					m->add(fun(&ccMesh::setLocked), "setLocked");
					m->add(fun(&ccMesh::isLeaf), "isLeaf");
					m->add(fun(&ccMesh::isCustom), "isCustom");
					m->add(fun(&ccMesh::isHierarchy), "isHierarchy");
					m->add(fun(&ccMesh::isKindOf), "isKindOf");
					m->add(fun(&ccMesh::isA), "isA");
					m->add(fun(&ccMesh::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccMesh::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccMesh::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccMesh::getMetaData), "getMetaData");
					m->add(fun(&ccMesh::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccMesh::*)(const  QString&, const QVariant&)>(&ccMesh::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccMesh::*)(const QVariantMap&, bool)>(&ccMesh::setMetaData)), "setMetaData");
					m->add(fun(&ccMesh::hasMetaData), "hasMetaData");
					m->add(fun(&ccMesh::metaData), "metaData");
					m->add(fun(&ccMesh::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccMesh::getOwnBB), "getOwnBB");
					m->add(fun(&ccMesh::getGLTransformationHistory), "getGLTransformationHistory");
					
					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccMesh>());
					m->add(chaiscript::base_class<ccHObject, ccMesh>());
					m->add(chaiscript::base_class<ccObject, ccMesh>());
					m->add(chaiscript::base_class<ccDrawableObject, ccMesh>());
					m->add(chaiscript::base_class<ccSerializableObject, ccMesh>());
					m->add(chaiscript::base_class<ccGenericMesh, ccMesh>());


					return m;
				}

				ModulePtr bs_ccGenericPrimitive(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGenericPrimitive>(), "ccGenericPrimitive");
					
					m->add(fun(&ccGenericPrimitive::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccGenericPrimitive::cloneMesh), "cloneMesh");
					m->add(fun(&ccGenericPrimitive::Triangulate), "Triangulate");
					m->add(fun(&ccGenericPrimitive::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccGenericPrimitive::merge), "merge");
					m->add(fun(&ccGenericPrimitive::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccGenericPrimitive::flipTriangles), "flipTriangles");
					m->add(fun(&ccGenericPrimitive::addTriangle), "addTriangle");
					m->add(fun(&ccGenericPrimitive::reserve), "reserve");
					m->add(fun(&ccGenericPrimitive::resize), "resize");
					m->add(fun(&ccGenericPrimitive::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccGenericPrimitive::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccGenericPrimitive::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccGenericPrimitive::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccGenericPrimitive::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccGenericPrimitive::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccGenericPrimitive::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccGenericPrimitive::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccGenericPrimitive::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccGenericPrimitive::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccGenericPrimitive::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccGenericPrimitive::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccGenericPrimitive::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccGenericPrimitive::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccGenericPrimitive::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccGenericPrimitive::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccGenericPrimitive::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccGenericPrimitive::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccGenericPrimitive::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccGenericPrimitive::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccGenericPrimitive::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccGenericPrimitive::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccGenericPrimitive::computeNormals), "computeNormals");
					m->add(fun(&ccGenericPrimitive::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccGenericPrimitive::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccGenericPrimitive::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccGenericPrimitive::processScalarField), "processScalarField");
					m->add(fun(&ccGenericPrimitive::subdivide), "subdivide");
					m->add(fun(&ccGenericPrimitive::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccGenericPrimitive::swapTriangles), "swapTriangles");
					m->add(fun(&ccGenericPrimitive::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccGenericPrimitive::size), "size");
					m->add(fun(&ccGenericPrimitive::forEach), "forEach");
					m->add(fun(&ccGenericPrimitive::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccGenericPrimitive::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccGenericPrimitive::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccGenericPrimitive::_getTriangle), "_getTriangle");
					m->add(fun(&ccGenericPrimitive::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccGenericPrimitive::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccGenericPrimitive::showNormals), "showNormals");
					m->add(fun(&ccGenericPrimitive::isSerializable), "isSerializable");
					m->add(fun(&ccGenericPrimitive::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccGenericPrimitive::refreshBB), "refreshBB");
					m->add(fun(&ccGenericPrimitive::capacity), "capacity");
					m->add(fun(&ccGenericPrimitive::hasMaterials), "hasMaterials");
					m->add(fun(&ccGenericPrimitive::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccGenericPrimitive::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccGenericPrimitive::hasTextures), "hasTextures");
					m->add(fun(&ccGenericPrimitive::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccGenericPrimitive::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccGenericPrimitive::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccGenericPrimitive::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccGenericPrimitive::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccGenericPrimitive::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccGenericPrimitive::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccGenericPrimitive::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccGenericPrimitive::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccGenericPrimitive::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccGenericPrimitive::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccGenericPrimitive::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccGenericPrimitive::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccGenericPrimitive::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccGenericPrimitive::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccGenericPrimitive::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccGenericPrimitive::showWired), "showWired");
					m->add(fun(&ccGenericPrimitive::triNormsShown), "triNormsShown");
					m->add(fun(&ccGenericPrimitive::showTriNorms), "showTriNorms");
					m->add(fun(&ccGenericPrimitive::materialsShown), "materialsShown");
					m->add(fun(&ccGenericPrimitive::showMaterials), "showMaterials");
					m->add(fun(&ccGenericPrimitive::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccGenericPrimitive::enableStippling), "enableStippling");
					m->add(fun(&ccGenericPrimitive::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccGenericPrimitive::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccGenericPrimitive::computePointPosition), "computePointPosition");
					m->add(fun(&ccGenericPrimitive::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccGenericPrimitive::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccGenericPrimitive::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccGenericPrimitive::getClassID), "getClassID");
					m->add(fun(&ccGenericPrimitive::getName), "getName");
					m->add(fun(&ccGenericPrimitive::setName), "setName");
					m->add(fun(&ccGenericPrimitive::getUniqueID), "getUniqueID");
					m->add(fun(&ccGenericPrimitive::setUniqueID), "setUniqueID");
					m->add(fun(&ccGenericPrimitive::isEnabled), "isEnabled");
					m->add(fun(&ccGenericPrimitive::setEnabled), "setEnabled");
					m->add(fun(&ccGenericPrimitive::toggleActivation), "toggleActivation");
					m->add(fun(&ccGenericPrimitive::isLocked), "isLocked");
					m->add(fun(&ccGenericPrimitive::setLocked), "setLocked");
					m->add(fun(&ccGenericPrimitive::isLeaf), "isLeaf");
					m->add(fun(&ccGenericPrimitive::isCustom), "isCustom");
					m->add(fun(&ccGenericPrimitive::isHierarchy), "isHierarchy");
					m->add(fun(&ccGenericPrimitive::isKindOf), "isKindOf");
					m->add(fun(&ccGenericPrimitive::isA), "isA");
					m->add(fun(&ccGenericPrimitive::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccGenericPrimitive::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccGenericPrimitive::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccGenericPrimitive::getMetaData), "getMetaData");
					m->add(fun(&ccGenericPrimitive::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccGenericPrimitive::*)(const  QString&, const QVariant&)>(&ccGenericPrimitive::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccGenericPrimitive::*)(const QVariantMap&, bool)>(&ccGenericPrimitive::setMetaData)), "setMetaData");
					m->add(fun(&ccGenericPrimitive::hasMetaData), "hasMetaData");
					m->add(fun(&ccGenericPrimitive::metaData), "metaData");
					m->add(fun(&ccGenericPrimitive::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccGenericPrimitive::getOwnBB), "getOwnBB");
					m->add(fun(&ccGenericPrimitive::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccGenericPrimitive::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccGenericPrimitive::getTypeName), "getTypeName");
					m->add(fun(&ccGenericPrimitive::clone), "clone");
					m->add(fun(&ccGenericPrimitive::setColor), "setColor");
					m->add(fun(&ccGenericPrimitive::operator+=), "+=");
					m->add(fun(&ccGenericPrimitive::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccGenericPrimitive::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccGenericPrimitive::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix&(ccGenericPrimitive::*)()>(&ccGenericPrimitive::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix&(ccGenericPrimitive::*)()const>(&ccGenericPrimitive::getTransformation)), "getTransformation");
					m->add(fun(&ccGenericPrimitive::getGLTransformationHistory), "getGLTransformationHistory");
					
					
					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccHObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccSerializableObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccDrawableObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccMesh, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccGenericMesh, ccGenericPrimitive>());
					
					return m;
				}

				ModulePtr bs_ccImage(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccImage>(), "ccImage");
					m->add(chaiscript::constructor<ccImage()>(), "ccImage");
					m->add(chaiscript::constructor<ccImage(const QImage&, const QString&)>(), "ccImage");

					m->add(fun(&ccImage::isSerializable), "isSerializable");
					m->add(fun(&ccImage::getClassID), "getClassID");
					m->add(fun(&ccImage::load), "load");
					m->add(fun(static_cast<QImage&(ccImage::*)()>(&ccImage::data)), "data");
					m->add(fun(static_cast<const QImage&(ccImage::*)()const>(&ccImage::data)), "data");
					m->add(fun(&ccImage::setData), "setData");
					m->add(fun(&ccImage::getW), "getW");
					m->add(fun(&ccImage::getH), "getH");
					m->add(fun(&ccImage::setAlpha), "setAlpha");
					m->add(fun(&ccImage::getAlpha), "getAlpha");
					m->add(fun(&ccImage::setAspectRatio), "setAspectRatio");
					m->add(fun(&ccImage::getAspectRatio), "getAspectRatio");
					m->add(fun(&ccImage::setAssociatedSensor), "setAssociatedSensor");
					m->add(fun(static_cast<ccCameraSensor*(ccImage::*)()>(&ccImage::getAssociatedSensor)), "getAssociatedSensor");
					m->add(fun(static_cast<const ccCameraSensor*(ccImage::*)()const>(&ccImage::getAssociatedSensor)), "getAssociatedSensor");
					m->add(fun(&ccImage::getOwnFitBB), "getOwnFitBB");

					m->add(chaiscript::base_class<ccHObject, ccImage>());
					m->add(chaiscript::base_class<ccObject, ccImage>());
					m->add(chaiscript::base_class<ccDrawableObject, ccImage>());
					m->add(chaiscript::base_class<ccSerializableObject, ccImage>());
					
					return m;
				}

				ModulePtr bs_ccPlane(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccPlane>(), "ccPlane");
					m->add(chaiscript::constructor<ccPlane(PointCoordinateType, PointCoordinateType, const ccGLMatrix*, QString)>(), "ccPlane");
					m->add(chaiscript::constructor<ccPlane(QString)>(), "ccPlane");
					
					m->add(fun([](PointCoordinateType x, PointCoordinateType y, const ccGLMatrix* mtrx, QString name) {return new ccPlane(x,y,mtrx,name); }),"create_ccPlane");
					m->add(fun(&ccPlane::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccPlane::cloneMesh), "cloneMesh");
					m->add(fun(&ccPlane::Triangulate), "Triangulate");
					m->add(fun(&ccPlane::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccPlane::merge), "merge");
					m->add(fun(&ccPlane::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccPlane::flipTriangles), "flipTriangles");
					m->add(fun(&ccPlane::addTriangle), "addTriangle");
					m->add(fun(&ccPlane::reserve), "reserve");
					m->add(fun(&ccPlane::resize), "resize");
					m->add(fun(&ccPlane::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccPlane::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccPlane::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccPlane::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccPlane::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccPlane::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccPlane::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccPlane::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccPlane::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccPlane::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccPlane::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccPlane::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccPlane::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccPlane::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccPlane::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccPlane::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccPlane::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccPlane::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccPlane::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccPlane::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccPlane::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccPlane::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccPlane::computeNormals), "computeNormals");
					m->add(fun(&ccPlane::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccPlane::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccPlane::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccPlane::processScalarField), "processScalarField");
					m->add(fun(&ccPlane::subdivide), "subdivide");
					m->add(fun(&ccPlane::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccPlane::swapTriangles), "swapTriangles");
					m->add(fun(&ccPlane::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccPlane::size), "size");
					m->add(fun(&ccPlane::forEach), "forEach");
					m->add(fun(&ccPlane::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccPlane::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccPlane::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccPlane::_getTriangle), "_getTriangle");
					m->add(fun(&ccPlane::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccPlane::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccPlane::showNormals), "showNormals");
					m->add(fun(&ccPlane::isSerializable), "isSerializable");
					m->add(fun(&ccPlane::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccPlane::refreshBB), "refreshBB");
					m->add(fun(&ccPlane::capacity), "capacity");
					m->add(fun(&ccPlane::hasMaterials), "hasMaterials");
					m->add(fun(&ccPlane::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccPlane::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccPlane::hasTextures), "hasTextures");
					m->add(fun(&ccPlane::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccPlane::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccPlane::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccPlane::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccPlane::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccPlane::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccPlane::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccPlane::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccPlane::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccPlane::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccPlane::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccPlane::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccPlane::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccPlane::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccPlane::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccPlane::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccPlane::showWired), "showWired");
					m->add(fun(&ccPlane::triNormsShown), "triNormsShown");
					m->add(fun(&ccPlane::showTriNorms), "showTriNorms");
					m->add(fun(&ccPlane::materialsShown), "materialsShown");
					m->add(fun(&ccPlane::showMaterials), "showMaterials");
					m->add(fun(&ccPlane::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccPlane::enableStippling), "enableStippling");
					m->add(fun(&ccPlane::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccPlane::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccPlane::computePointPosition), "computePointPosition");
					m->add(fun(&ccPlane::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccPlane::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccPlane::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccPlane::getClassID), "getClassID");
					//m->add(fun(&ccPlane::getName), "getName");
					//m->add(fun(&ccPlane::setName), "setName");
					m->add(fun(&ccPlane::getUniqueID), "getUniqueID");
					m->add(fun(&ccPlane::setUniqueID), "setUniqueID");
					m->add(fun(&ccPlane::isEnabled), "isEnabled");
					m->add(fun(&ccPlane::setEnabled), "setEnabled");
					m->add(fun(&ccPlane::toggleActivation), "toggleActivation");
					m->add(fun(&ccPlane::isLocked), "isLocked");
					m->add(fun(&ccPlane::setLocked), "setLocked");
					m->add(fun(&ccPlane::isLeaf), "isLeaf");
					m->add(fun(&ccPlane::isCustom), "isCustom");
					m->add(fun(&ccPlane::isHierarchy), "isHierarchy");
					m->add(fun(&ccPlane::isKindOf), "isKindOf");
					m->add(fun(&ccPlane::isA), "isA");
					m->add(fun(&ccPlane::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccPlane::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccPlane::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccPlane::getMetaData), "getMetaData");
					m->add(fun(&ccPlane::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccPlane::*)(const  QString&, const QVariant&)>(&ccPlane::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccPlane::*)(const QVariantMap&, bool)>(&ccPlane::setMetaData)), "setMetaData");
					m->add(fun(&ccPlane::hasMetaData), "hasMetaData");
					m->add(fun(&ccPlane::metaData), "metaData");
					m->add(fun(&ccPlane::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccPlane::getOwnBB), "getOwnBB");
					m->add(fun(&ccPlane::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccPlane::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccPlane::getTypeName), "getTypeName");
					m->add(fun(&ccPlane::clone), "clone");
					m->add(fun(&ccPlane::setColor), "setColor");
					m->add(fun(&ccPlane::operator+=), "+=");
					m->add(fun(&ccPlane::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccPlane::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccPlane::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix & (ccPlane::*)()>(&ccPlane::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccPlane::*)()const>(&ccPlane::getTransformation)), "getTransformation");
					m->add(fun(&ccPlane::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccPlane::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccPlane::getXWidth), "getXWidth");
					m->add(fun(&ccPlane::getYWidth), "getYWidth");
					m->add(fun(&ccPlane::getCenter), "getCenter");
					m->add(fun([](ccPlane* pln, PointCoordinateType c) {pln->setXWidth(c); }), "setXWidth");
					m->add(fun(&ccPlane::setXWidth), "setXWidth"); 
					m->add(fun([](ccPlane* pln, PointCoordinateType c) {pln->setYWidth(c); }), "setYWidth");
					m->add(fun(&ccPlane::setYWidth), "setYWidth"); 
					m->add(fun(&ccPlane::getNormal), "getNormal");
					m->add(fun([](ccPlane* pln, QImage i) {return pln->setAsTexture(i); }), "setAsTexture");
					m->add(fun(&ccPlane::setAsTexture), "setAsTexture");
					m->add(fun([](ccPlane* pln, ccMesh* msh, QImage i) {return pln->SetQuadTexture(msh, i); }), "SetQuadTexture");
					m->add(fun(&ccPlane::SetQuadTexture), "SetQuadTexture");
					m->add(fun([](ccPlane* pln, CCLib::GenericIndexedCloudPersist* cld) {pln->Fit(cld); }), "Fit");
					m->add(fun(&ccPlane::Fit), "Fit");
					m->add(fun(static_cast<void(ccPlane::*)(CCVector3&, PointCoordinateType&)const>(&ccPlane::getEquation)), "getEquation");
					m->add(fun(static_cast<const PointCoordinateType*(ccPlane::*)()>(&ccPlane::getEquation)), "getEquation");
					m->add(fun(&ccPlane::flip), "flip");

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccPlane>());
					m->add(chaiscript::base_class<ccHObject, ccPlane>());
					m->add(chaiscript::base_class<ccObject, ccPlane>());
					m->add(chaiscript::base_class<ccDrawableObject, ccPlane>());
					m->add(chaiscript::base_class<ccMesh, ccPlane>());
					m->add(chaiscript::base_class<ccGenericMesh, ccPlane>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccPlane>());
					m->add(chaiscript::base_class<ccPlanarEntityInterface, ccPlane>());
					m->add(chaiscript::base_class<ccSerializableObject, ccPlane>());

					return m;
				}

				ModulePtr bs_ccSphere(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccSphere>(), "ccSphere");
					m->add(chaiscript::constructor<ccSphere(PointCoordinateType, const ccGLMatrix*, QString, unsigned)>(), "ccSphere");
					m->add(chaiscript::constructor<ccSphere(QString)>(), "ccSphere");

					m->add(fun([](PointCoordinateType rad, const ccGLMatrix* mtrx, QString name, unsigned p) {return new ccSphere(rad, mtrx, name, p); }), "create_ccSphere");
					m->add(fun(&ccSphere::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccSphere::cloneMesh), "cloneMesh");
					m->add(fun(&ccSphere::Triangulate), "Triangulate");
					m->add(fun(&ccSphere::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccSphere::merge), "merge");
					m->add(fun(&ccSphere::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccSphere::flipTriangles), "flipTriangles");
					m->add(fun(&ccSphere::addTriangle), "addTriangle");
					m->add(fun(&ccSphere::reserve), "reserve");
					m->add(fun(&ccSphere::resize), "resize");
					m->add(fun(&ccSphere::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccSphere::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccSphere::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccSphere::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccSphere::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccSphere::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccSphere::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccSphere::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccSphere::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccSphere::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccSphere::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccSphere::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccSphere::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccSphere::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccSphere::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccSphere::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccSphere::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccSphere::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccSphere::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccSphere::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccSphere::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccSphere::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccSphere::computeNormals), "computeNormals");
					m->add(fun(&ccSphere::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccSphere::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccSphere::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccSphere::processScalarField), "processScalarField");
					m->add(fun(&ccSphere::subdivide), "subdivide");
					m->add(fun(&ccSphere::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccSphere::swapTriangles), "swapTriangles");
					m->add(fun(&ccSphere::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccSphere::size), "size");
					m->add(fun(&ccSphere::forEach), "forEach");
					m->add(fun(&ccSphere::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccSphere::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccSphere::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccSphere::_getTriangle), "_getTriangle");
					m->add(fun(&ccSphere::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccSphere::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccSphere::showNormals), "showNormals");
					m->add(fun(&ccSphere::isSerializable), "isSerializable");
					m->add(fun(&ccSphere::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccSphere::refreshBB), "refreshBB");
					m->add(fun(&ccSphere::capacity), "capacity");
					m->add(fun(&ccSphere::hasMaterials), "hasMaterials");
					m->add(fun(&ccSphere::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccSphere::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccSphere::hasTextures), "hasTextures");
					m->add(fun(&ccSphere::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccSphere::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccSphere::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccSphere::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccSphere::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccSphere::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccSphere::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccSphere::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccSphere::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccSphere::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccSphere::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccSphere::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccSphere::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccSphere::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccSphere::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccSphere::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccSphere::showWired), "showWired");
					m->add(fun(&ccSphere::triNormsShown), "triNormsShown");
					m->add(fun(&ccSphere::showTriNorms), "showTriNorms");
					m->add(fun(&ccSphere::materialsShown), "materialsShown");
					m->add(fun(&ccSphere::showMaterials), "showMaterials");
					m->add(fun(&ccSphere::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccSphere::enableStippling), "enableStippling");
					m->add(fun(&ccSphere::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccSphere::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccSphere::computePointPosition), "computePointPosition");
					m->add(fun(&ccSphere::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccSphere::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccSphere::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccSphere::getClassID), "getClassID");
					m->add(fun(&ccSphere::getName), "getName");
					m->add(fun(&ccSphere::setName), "setName");
					m->add(fun(&ccSphere::getUniqueID), "getUniqueID");
					m->add(fun(&ccSphere::setUniqueID), "setUniqueID");
					m->add(fun(&ccSphere::isEnabled), "isEnabled");
					m->add(fun(&ccSphere::setEnabled), "setEnabled");
					m->add(fun(&ccSphere::toggleActivation), "toggleActivation");
					m->add(fun(&ccSphere::isLocked), "isLocked");
					m->add(fun(&ccSphere::setLocked), "setLocked");
					m->add(fun(&ccSphere::isLeaf), "isLeaf");
					m->add(fun(&ccSphere::isCustom), "isCustom");
					m->add(fun(&ccSphere::isHierarchy), "isHierarchy");
					m->add(fun(&ccSphere::isKindOf), "isKindOf");
					m->add(fun(&ccSphere::isA), "isA");
					m->add(fun(&ccSphere::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccSphere::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccSphere::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccSphere::getMetaData), "getMetaData");
					m->add(fun(&ccSphere::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccSphere::*)(const  QString&, const QVariant&)>(&ccSphere::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccSphere::*)(const QVariantMap&, bool)>(&ccSphere::setMetaData)), "setMetaData");
					m->add(fun(&ccSphere::hasMetaData), "hasMetaData");
					m->add(fun(&ccSphere::metaData), "metaData");
					m->add(fun(&ccSphere::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccSphere::getOwnBB), "getOwnBB");
					m->add(fun(&ccSphere::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccSphere::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccSphere::getTypeName), "getTypeName");
					m->add(fun(&ccSphere::clone), "clone");
					m->add(fun(&ccSphere::setColor), "setColor");
					m->add(fun(&ccSphere::operator+=), "+=");
					m->add(fun(&ccSphere::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccSphere::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccSphere::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix & (ccSphere::*)()>(&ccSphere::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccSphere::*)()const>(&ccSphere::getTransformation)), "getTransformation");
					m->add(fun(&ccSphere::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccSphere::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccSphere::getRadius), "getRadius");
					m->add(fun(&ccSphere::setRadius), "setRadius");

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccSphere>());
					m->add(chaiscript::base_class<ccHObject, ccSphere>());
					m->add(chaiscript::base_class<ccObject, ccSphere>());
					m->add(chaiscript::base_class<ccSerializableObject, ccSphere>());
					m->add(chaiscript::base_class<ccDrawableObject, ccSphere>());
					m->add(chaiscript::base_class<ccMesh, ccSphere>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccSphere>());
					m->add(chaiscript::base_class<ccGenericMesh, ccSphere>());

					return m;
				}

				ModulePtr bs_ccCone(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccCone>(), "ccCone");
					m->add(chaiscript::constructor<ccCone(PointCoordinateType, PointCoordinateType, PointCoordinateType, PointCoordinateType, PointCoordinateType, const ccGLMatrix*, QString, unsigned)>(), "ccCone");
					m->add(chaiscript::constructor<ccCone(QString)>(), "ccCone");
					m->add(fun([](PointCoordinateType br, PointCoordinateType tr, PointCoordinateType h) {return new ccCone(br, tr, h); }), "create_ccCone");
					m->add(fun([](PointCoordinateType br, PointCoordinateType tr, PointCoordinateType h, PointCoordinateType xOff) {return new ccCone(br, tr, h, xOff); }), "create_ccCone");
					m->add(fun([](PointCoordinateType br, PointCoordinateType tr, PointCoordinateType h, PointCoordinateType xOff, PointCoordinateType yOff) {return new ccCone(br, tr, h, xOff, yOff); }), "create_ccCone");
					m->add(fun([](PointCoordinateType br, PointCoordinateType tr, PointCoordinateType h, PointCoordinateType xOff, PointCoordinateType yOff, const ccGLMatrix* transMat) {return new ccCone(br, tr, h, xOff, yOff, transMat); }), "create_ccCone");
					m->add(fun([](PointCoordinateType br, PointCoordinateType tr, PointCoordinateType h, PointCoordinateType xOff, PointCoordinateType yOff, const ccGLMatrix* transMat, QString name) {return new ccCone(br, tr, h, xOff, yOff, transMat, name); }), "create_ccCone");
					m->add(fun([](PointCoordinateType br, PointCoordinateType tr, PointCoordinateType h, PointCoordinateType xOff, PointCoordinateType yOff, const ccGLMatrix* transMat, QString name, unsigned p) {return new ccCone(br, tr, h, xOff, yOff,transMat, name, p); }), "create_ccCone");
					m->add(fun(&ccCone::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccCone::cloneMesh), "cloneMesh");
					m->add(fun(&ccCone::Triangulate), "Triangulate");
					m->add(fun(&ccCone::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccCone::merge), "merge");
					m->add(fun(&ccCone::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccCone::flipTriangles), "flipTriangles");
					m->add(fun(&ccCone::addTriangle), "addTriangle");
					m->add(fun(&ccCone::reserve), "reserve");
					m->add(fun(&ccCone::resize), "resize");
					m->add(fun(&ccCone::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccCone::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccCone::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccCone::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccCone::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccCone::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccCone::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccCone::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccCone::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccCone::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccCone::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccCone::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccCone::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccCone::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccCone::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccCone::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccCone::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccCone::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccCone::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccCone::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccCone::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccCone::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccCone::computeNormals), "computeNormals");
					m->add(fun(&ccCone::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccCone::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccCone::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccCone::processScalarField), "processScalarField");
					m->add(fun(&ccCone::subdivide), "subdivide");
					m->add(fun(&ccCone::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccCone::swapTriangles), "swapTriangles");
					m->add(fun(&ccCone::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccCone::size), "size");
					m->add(fun(&ccCone::forEach), "forEach");
					m->add(fun(&ccCone::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccCone::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccCone::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccCone::_getTriangle), "_getTriangle");
					m->add(fun(&ccCone::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccCone::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccCone::showNormals), "showNormals");
					m->add(fun(&ccCone::isSerializable), "isSerializable");
					m->add(fun(&ccCone::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccCone::refreshBB), "refreshBB");
					m->add(fun(&ccCone::capacity), "capacity");
					m->add(fun(&ccCone::hasMaterials), "hasMaterials");
					m->add(fun(&ccCone::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccCone::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccCone::hasTextures), "hasTextures");
					m->add(fun(&ccCone::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccCone::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccCone::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccCone::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccCone::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccCone::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccCone::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccCone::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccCone::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccCone::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccCone::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccCone::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccCone::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccCone::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccCone::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccCone::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccCone::showWired), "showWired");
					m->add(fun(&ccCone::triNormsShown), "triNormsShown");
					m->add(fun(&ccCone::showTriNorms), "showTriNorms");
					m->add(fun(&ccCone::materialsShown), "materialsShown");
					m->add(fun(&ccCone::showMaterials), "showMaterials");
					m->add(fun(&ccCone::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccCone::enableStippling), "enableStippling");
					m->add(fun(&ccCone::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccCone::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccCone::computePointPosition), "computePointPosition");
					m->add(fun(&ccCone::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccCone::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccCone::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccCone::getClassID), "getClassID");
					m->add(fun(&ccCone::getName), "getName");
					m->add(fun(&ccCone::setName), "setName");
					m->add(fun(&ccCone::getUniqueID), "getUniqueID");
					m->add(fun(&ccCone::setUniqueID), "setUniqueID");
					m->add(fun(&ccCone::isEnabled), "isEnabled");
					m->add(fun(&ccCone::setEnabled), "setEnabled");
					m->add(fun(&ccCone::toggleActivation), "toggleActivation");
					m->add(fun(&ccCone::isLocked), "isLocked");
					m->add(fun(&ccCone::setLocked), "setLocked");
					m->add(fun(&ccCone::isLeaf), "isLeaf");
					m->add(fun(&ccCone::isCustom), "isCustom");
					m->add(fun(&ccCone::isHierarchy), "isHierarchy");
					m->add(fun(&ccCone::isKindOf), "isKindOf");
					m->add(fun(&ccCone::isA), "isA");
					m->add(fun(&ccCone::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccCone::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccCone::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccCone::getMetaData), "getMetaData");
					m->add(fun(&ccCone::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccCone::*)(const  QString&, const QVariant&)>(&ccCone::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccCone::*)(const QVariantMap&, bool)>(&ccCone::setMetaData)), "setMetaData");
					m->add(fun(&ccCone::hasMetaData), "hasMetaData");
					m->add(fun(&ccCone::metaData), "metaData");
					m->add(fun(&ccCone::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccCone::getOwnBB), "getOwnBB");
					m->add(fun(&ccCone::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccCone::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccCone::getTypeName), "getTypeName");
					m->add(fun(&ccCone::clone), "clone");
					m->add(fun(&ccCone::setColor), "setColor");
					m->add(fun(&ccCone::operator+=), "+=");
					m->add(fun(&ccCone::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccCone::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccCone::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix & (ccCone::*)()>(&ccCone::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccCone::*)()const>(&ccCone::getTransformation)), "getTransformation");
					m->add(fun(&ccCone::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccCone::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccCone::getHeight), "getHeight");
					m->add(fun(&ccCone::setHeight), "setHeight");
					m->add(fun(&ccCone::getBottomRadius), "getBottomRadius");
					m->add(fun(&ccCone::setBottomRadius), "setBottomRadius");
					m->add(fun(&ccCone::getTopRadius), "getTopRadius");
					m->add(fun(&ccCone::setTopRadius), "setTopRadius");
					m->add(fun(&ccCone::getBottomCenter), "getBottomCenter");
					m->add(fun(&ccCone::getTopCenter), "getTopCenter");
					m->add(fun(&ccCone::getSmallCenter), "getSmallCenter");
					m->add(fun(&ccCone::getLargeCenter), "getLargeCenter");
					m->add(fun(&ccCone::getSmallRadius), "getSmallRadius");
					m->add(fun(&ccCone::getLargeRadius), "getLargeRadius");
					m->add(fun(&ccCone::isSnoutMode), "isSnoutMode");

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccCone>());
					m->add(chaiscript::base_class<ccHObject, ccCone>());
					m->add(chaiscript::base_class<ccObject, ccCone>());
					m->add(chaiscript::base_class<ccSerializableObject, ccCone>());
					m->add(chaiscript::base_class<ccDrawableObject, ccCone>());
					m->add(chaiscript::base_class<ccMesh, ccCone>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccCone>());
					m->add(chaiscript::base_class<ccGenericMesh, ccCone>());

					return m;
				}

				ModulePtr bs_ccCylinder(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccCylinder>(), "ccCylinder");
					m->add(chaiscript::constructor<ccCylinder(PointCoordinateType, PointCoordinateType, const ccGLMatrix*, QString, unsigned)>(), "ccCylinder");
					m->add(chaiscript::constructor<ccCylinder(QString)>(), "ccCylinder");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h) {return new ccCylinder(r, h); }), "create_ccCylinder");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h, const ccGLMatrix* transMat) {return new ccCylinder(r, h, transMat); }), "create_ccCylinder");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h, const ccGLMatrix* transMat, QString name) {return new ccCylinder(r, h, transMat, name); }), "create_ccCylinder");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h, const ccGLMatrix* transMat, QString name, unsigned p) {return new ccCylinder(r, h, transMat, name, p); }), "create_ccCylinder");
					m->add(fun(&ccCylinder::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccCylinder::cloneMesh), "cloneMesh");
					m->add(fun(&ccCylinder::Triangulate), "Triangulate");
					m->add(fun(&ccCylinder::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccCylinder::merge), "merge");
					m->add(fun(&ccCylinder::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccCylinder::flipTriangles), "flipTriangles");
					m->add(fun(&ccCylinder::addTriangle), "addTriangle");
					m->add(fun(&ccCylinder::reserve), "reserve");
					m->add(fun(&ccCylinder::resize), "resize");
					m->add(fun(&ccCylinder::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccCylinder::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccCylinder::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccCylinder::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccCylinder::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccCylinder::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccCylinder::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccCylinder::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccCylinder::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccCylinder::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccCylinder::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccCylinder::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccCylinder::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccCylinder::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccCylinder::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccCylinder::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccCylinder::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccCylinder::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccCylinder::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccCylinder::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccCylinder::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccCylinder::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccCylinder::computeNormals), "computeNormals");
					m->add(fun(&ccCylinder::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccCylinder::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccCylinder::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccCylinder::processScalarField), "processScalarField");
					m->add(fun(&ccCylinder::subdivide), "subdivide");
					m->add(fun(&ccCylinder::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccCylinder::swapTriangles), "swapTriangles");
					m->add(fun(&ccCylinder::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccCylinder::size), "size");
					m->add(fun(&ccCylinder::forEach), "forEach");
					m->add(fun(&ccCylinder::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccCylinder::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccCylinder::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccCylinder::_getTriangle), "_getTriangle");
					m->add(fun(&ccCylinder::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccCylinder::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccCylinder::showNormals), "showNormals");
					m->add(fun(&ccCylinder::isSerializable), "isSerializable");
					m->add(fun(&ccCylinder::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccCylinder::refreshBB), "refreshBB");
					m->add(fun(&ccCylinder::capacity), "capacity");
					m->add(fun(&ccCylinder::hasMaterials), "hasMaterials");
					m->add(fun(&ccCylinder::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccCylinder::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccCylinder::hasTextures), "hasTextures");
					m->add(fun(&ccCylinder::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccCylinder::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccCylinder::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccCylinder::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccCylinder::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccCylinder::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccCylinder::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccCylinder::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccCylinder::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccCylinder::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccCylinder::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccCylinder::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccCylinder::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccCylinder::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccCylinder::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccCylinder::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccCylinder::showWired), "showWired");
					m->add(fun(&ccCylinder::triNormsShown), "triNormsShown");
					m->add(fun(&ccCylinder::showTriNorms), "showTriNorms");
					m->add(fun(&ccCylinder::materialsShown), "materialsShown");
					m->add(fun(&ccCylinder::showMaterials), "showMaterials");
					m->add(fun(&ccCylinder::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccCylinder::enableStippling), "enableStippling");
					m->add(fun(&ccCylinder::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccCylinder::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccCylinder::computePointPosition), "computePointPosition");
					m->add(fun(&ccCylinder::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccCylinder::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccCylinder::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccCylinder::getClassID), "getClassID");
					m->add(fun(&ccCylinder::getName), "getName");
					m->add(fun(&ccCylinder::setName), "setName");
					m->add(fun(&ccCylinder::getUniqueID), "getUniqueID");
					m->add(fun(&ccCylinder::setUniqueID), "setUniqueID");
					m->add(fun(&ccCylinder::isEnabled), "isEnabled");
					m->add(fun(&ccCylinder::setEnabled), "setEnabled");
					m->add(fun(&ccCylinder::toggleActivation), "toggleActivation");
					m->add(fun(&ccCylinder::isLocked), "isLocked");
					m->add(fun(&ccCylinder::setLocked), "setLocked");
					m->add(fun(&ccCylinder::isLeaf), "isLeaf");
					m->add(fun(&ccCylinder::isCustom), "isCustom");
					m->add(fun(&ccCylinder::isHierarchy), "isHierarchy");
					m->add(fun(&ccCylinder::isKindOf), "isKindOf");
					m->add(fun(&ccCylinder::isA), "isA");
					m->add(fun(&ccCylinder::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccCylinder::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccCylinder::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccCylinder::getMetaData), "getMetaData");
					m->add(fun(&ccCylinder::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccCylinder::*)(const  QString&, const QVariant&)>(&ccCylinder::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccCylinder::*)(const QVariantMap&, bool)>(&ccCylinder::setMetaData)), "setMetaData");
					m->add(fun(&ccCylinder::hasMetaData), "hasMetaData");
					m->add(fun(&ccCylinder::metaData), "metaData");
					m->add(fun(&ccCylinder::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccCylinder::getOwnBB), "getOwnBB");
					m->add(fun(&ccCylinder::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccCylinder::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccCylinder::getTypeName), "getTypeName");
					m->add(fun(&ccCylinder::clone), "clone");
					m->add(fun(&ccCylinder::setColor), "setColor");
					m->add(fun(&ccCylinder::operator+=), "+=");
					m->add(fun(&ccCylinder::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccCylinder::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccCylinder::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix & (ccCylinder::*)()>(&ccCylinder::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccCylinder::*)()const>(&ccCylinder::getTransformation)), "getTransformation");
					m->add(fun(&ccCylinder::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccCylinder::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccCylinder::getHeight), "getHeight");
					m->add(fun(&ccCylinder::setHeight), "setHeight");
					m->add(fun(&ccCylinder::getBottomRadius), "getBottomRadius");
					m->add(fun(&ccCylinder::getTopRadius), "getTopRadius");
					m->add(fun(&ccCylinder::setTopRadius), "setTopRadius");
					m->add(fun(&ccCylinder::getBottomCenter), "getBottomCenter");
					m->add(fun(&ccCylinder::getTopCenter), "getTopCenter");
					m->add(fun(&ccCylinder::getSmallCenter), "getSmallCenter");
					m->add(fun(&ccCylinder::getLargeCenter), "getLargeCenter");
					m->add(fun(&ccCylinder::getSmallRadius), "getSmallRadius");
					m->add(fun(&ccCylinder::getLargeRadius), "getLargeRadius");
					m->add(fun(&ccCylinder::isSnoutMode), "isSnoutMode");

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccCylinder>());
					m->add(chaiscript::base_class<ccHObject, ccCylinder>());
					m->add(chaiscript::base_class<ccObject, ccCylinder>());
					m->add(chaiscript::base_class<ccSerializableObject, ccCylinder>());
					m->add(chaiscript::base_class<ccDrawableObject, ccCylinder>());
					m->add(chaiscript::base_class<ccMesh, ccCylinder>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccCylinder>());
					m->add(chaiscript::base_class<ccGenericMesh, ccCylinder>());
					m->add(chaiscript::base_class<ccCone, ccCylinder>());

					return m;
				}

				ModulePtr bs_ccDish(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccDish>(), "ccDish");
					m->add(chaiscript::constructor<ccDish(PointCoordinateType, PointCoordinateType, PointCoordinateType, const ccGLMatrix*, QString, unsigned)>(), "ccCylinder");
					m->add(chaiscript::constructor<ccDish(QString)>(), "ccDish");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h) {return new ccDish(r, h); }), "create_ccDish");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h, PointCoordinateType r2, const ccGLMatrix* transMat) {return new ccDish(r, h, r2, transMat); }), "create_ccDish");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h, PointCoordinateType r2, const ccGLMatrix* transMat, QString name) {return new ccDish(r, h, r2, transMat, name); }), "create_ccDish");
					m->add(fun([](PointCoordinateType r, PointCoordinateType h, PointCoordinateType r2, const ccGLMatrix* transMat, QString name, unsigned p) {return new ccDish(r, h, r2, transMat, name, p); }), "create_ccDish");
					m->add(fun(&ccDish::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccDish::cloneMesh), "cloneMesh");
					m->add(fun(&ccDish::Triangulate), "Triangulate");
					m->add(fun(&ccDish::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccDish::merge), "merge");
					m->add(fun(&ccDish::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccDish::flipTriangles), "flipTriangles");
					m->add(fun(&ccDish::addTriangle), "addTriangle");
					m->add(fun(&ccDish::reserve), "reserve");
					m->add(fun(&ccDish::resize), "resize");
					m->add(fun(&ccDish::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccDish::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccDish::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccDish::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccDish::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccDish::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccDish::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccDish::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccDish::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccDish::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccDish::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccDish::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccDish::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccDish::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccDish::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccDish::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccDish::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccDish::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccDish::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccDish::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccDish::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccDish::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccDish::computeNormals), "computeNormals");
					m->add(fun(&ccDish::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccDish::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccDish::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccDish::processScalarField), "processScalarField");
					m->add(fun(&ccDish::subdivide), "subdivide");
					m->add(fun(&ccDish::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccDish::swapTriangles), "swapTriangles");
					m->add(fun(&ccDish::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccDish::size), "size");
					m->add(fun(&ccDish::forEach), "forEach");
					m->add(fun(&ccDish::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccDish::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccDish::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccDish::_getTriangle), "_getTriangle");
					m->add(fun(&ccDish::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccDish::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccDish::showNormals), "showNormals");
					m->add(fun(&ccDish::isSerializable), "isSerializable");
					m->add(fun(&ccDish::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccDish::refreshBB), "refreshBB");
					m->add(fun(&ccDish::capacity), "capacity");
					m->add(fun(&ccDish::hasMaterials), "hasMaterials");
					m->add(fun(&ccDish::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccDish::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccDish::hasTextures), "hasTextures");
					m->add(fun(&ccDish::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccDish::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccDish::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccDish::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccDish::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccDish::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccDish::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccDish::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccDish::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccDish::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccDish::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccDish::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccDish::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccDish::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccDish::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccDish::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccDish::showWired), "showWired");
					m->add(fun(&ccDish::triNormsShown), "triNormsShown");
					m->add(fun(&ccDish::showTriNorms), "showTriNorms");
					m->add(fun(&ccDish::materialsShown), "materialsShown");
					m->add(fun(&ccDish::showMaterials), "showMaterials");
					m->add(fun(&ccDish::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccDish::enableStippling), "enableStippling");
					m->add(fun(&ccDish::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccDish::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccDish::computePointPosition), "computePointPosition");
					m->add(fun(&ccDish::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccDish::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccDish::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccDish::getClassID), "getClassID");
					m->add(fun(&ccDish::getName), "getName");
					m->add(fun(&ccDish::setName), "setName");
					m->add(fun(&ccDish::getUniqueID), "getUniqueID");
					m->add(fun(&ccDish::setUniqueID), "setUniqueID");
					m->add(fun(&ccDish::isEnabled), "isEnabled");
					m->add(fun(&ccDish::setEnabled), "setEnabled");
					m->add(fun(&ccDish::toggleActivation), "toggleActivation");
					m->add(fun(&ccDish::isLocked), "isLocked");
					m->add(fun(&ccDish::setLocked), "setLocked");
					m->add(fun(&ccDish::isLeaf), "isLeaf");
					m->add(fun(&ccDish::isCustom), "isCustom");
					m->add(fun(&ccDish::isHierarchy), "isHierarchy");
					m->add(fun(&ccDish::isKindOf), "isKindOf");
					m->add(fun(&ccDish::isA), "isA");
					m->add(fun(&ccDish::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccDish::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccDish::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccDish::getMetaData), "getMetaData");
					m->add(fun(&ccDish::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccDish::*)(const  QString&, const QVariant&)>(&ccDish::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccDish::*)(const QVariantMap&, bool)>(&ccDish::setMetaData)), "setMetaData");
					m->add(fun(&ccDish::hasMetaData), "hasMetaData");
					m->add(fun(&ccDish::metaData), "metaData");
					m->add(fun(&ccDish::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccDish::getOwnBB), "getOwnBB");
					m->add(fun(&ccDish::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccDish::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccDish::getTypeName), "getTypeName");
					m->add(fun(&ccDish::clone), "clone");
					m->add(fun(&ccDish::setColor), "setColor");
					m->add(fun(&ccDish::operator+=), "+=");
					m->add(fun(&ccDish::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccDish::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccDish::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix & (ccDish::*)()>(&ccDish::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccDish::*)()const>(&ccDish::getTransformation)), "getTransformation");
					m->add(fun(&ccDish::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccDish::getOwnFitBB), "getOwnFitBB");

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccDish>());
					m->add(chaiscript::base_class<ccHObject, ccDish>());
					m->add(chaiscript::base_class<ccObject, ccDish>());
					m->add(chaiscript::base_class<ccSerializableObject, ccDish>());
					m->add(chaiscript::base_class<ccDrawableObject, ccDish>());
					m->add(chaiscript::base_class<ccMesh, ccDish>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccDish>());
					m->add(chaiscript::base_class<ccGenericMesh, ccDish>());

					return m;
				}

				ModulePtr bs_ccExtru(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccExtru>(), "ccExtru");
					m->add(chaiscript::constructor<ccExtru(const std::vector<CCVector2>&, PointCoordinateType, const ccGLMatrix*, QString)>(), "ccExtru");
					m->add(chaiscript::constructor<ccExtru(QString)>(), "ccExtru");
					m->add(fun([](const std::vector<CCVector2>& profile, PointCoordinateType h) {return new ccExtru(profile, h); }), "create_ccExtru");
					m->add(fun([](const std::vector<CCVector2>& profile, PointCoordinateType h, const ccGLMatrix* transMat) {return new ccExtru(profile, h, transMat); }), "create_ccExtru");
					m->add(fun([](const std::vector<CCVector2>& profile, PointCoordinateType h, const ccGLMatrix* transMat, QString name) {return new ccExtru(profile, h, transMat, name); }), "create_ccExtru");
					m->add(fun(&ccExtru::setAssociatedCloud), "setAssociatedCloud");
					m->add(fun(&ccExtru::cloneMesh), "cloneMesh");
					m->add(fun(&ccExtru::Triangulate), "Triangulate");
					m->add(fun(&ccExtru::TriangulateTwoPolylines), "TriangulateTwoPolylines");
					m->add(fun(&ccExtru::merge), "merge");
					m->add(fun(&ccExtru::shiftTriangleIndexes), "shiftTriangleIndexes");
					m->add(fun(&ccExtru::flipTriangles), "flipTriangles");
					m->add(fun(&ccExtru::addTriangle), "addTriangle");
					m->add(fun(&ccExtru::reserve), "reserve");
					m->add(fun(&ccExtru::resize), "resize");
					m->add(fun(&ccExtru::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccExtru::setTriNormsTable), "setTriNormsTable");
					m->add(fun(&ccExtru::clearTriNormals), "clearTriNormals");
					m->add(fun(&ccExtru::arePerTriangleNormalsEnabled), "arePerTriangleNormalsEnabled");
					m->add(fun(&ccExtru::reservePerTriangleNormalIndexes), "reservePerTriangleNormalIndexes");
					m->add(fun(&ccExtru::addTriangleNormalIndexes), "addTriangleNormalIndexes");
					m->add(fun(&ccExtru::setTriangleNormalIndexes), "setTriangleNormalIndexes");
					m->add(fun(&ccExtru::removePerTriangleNormalIndexes), "removePerTriangleNormalIndexes");
					m->add(fun(&ccExtru::convertMaterialsToVertexColors), "convertMaterialsToVertexColors");
					m->add(fun(&ccExtru::hasPerTriangleMtlIndexes), "hasPerTriangleMtlIndexes");
					m->add(fun(&ccExtru::reservePerTriangleMtlIndexes), "reservePerTriangleMtlIndexes");
					m->add(fun(&ccExtru::removePerTriangleMtlIndexes), "removePerTriangleMtlIndexes");
					m->add(fun(&ccExtru::addTriangleMtlIndex), "addTriangleMtlIndex");
					m->add(fun(&ccExtru::setTriangleMtlIndexesTable), "setTriangleMtlIndexesTable");
					m->add(fun(&ccExtru::getTriangleMtlIndexesTable), "getTriangleMtlIndexesTable");
					m->add(fun(&ccExtru::setTriangleMtlIndex), "setTriangleMtlIndex");
					m->add(fun(&ccExtru::setMaterialSet), "setMaterialSet"); //TODO add default version(s)
					m->add(fun(&ccExtru::setTexCoordinatesTable), "setTexCoordinatesTable");
					m->add(fun(&ccExtru::reservePerTriangleTexCoordIndexes), "reservePerTriangleTexCoordIndexes");
					m->add(fun(&ccExtru::removePerTriangleTexCoordIndexes), "removePerTriangleTexCoordIndexes");
					m->add(fun(&ccExtru::addTriangleTexCoordIndexes), "addTriangleTexCoordIndexes");
					m->add(fun(&ccExtru::setTriangleTexCoordIndexes), "setTriangleTexCoordIndexes");
					m->add(fun(&ccExtru::computeNormals), "computeNormals");
					m->add(fun(&ccExtru::computePerVertexNormals), "computePerVertexNormals");
					m->add(fun(&ccExtru::computePerTriangleNormals), "computePerTriangleNormals");
					m->add(fun(&ccExtru::laplacianSmooth), "laplacianSmooth");
					m->add(fun(&ccExtru::processScalarField), "processScalarField");
					m->add(fun(&ccExtru::subdivide), "subdivide");
					m->add(fun(&ccExtru::createNewMeshFromSelection), "createNewMeshFromSelection");
					m->add(fun(&ccExtru::swapTriangles), "swapTriangles");
					m->add(fun(&ccExtru::transformTriNormals), "transformTriNormals");
					m->add(fun(&ccExtru::size), "size");
					m->add(fun(&ccExtru::forEach), "forEach");
					m->add(fun(&ccExtru::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccExtru::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccExtru::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccExtru::_getTriangle), "_getTriangle");
					m->add(fun(&ccExtru::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccExtru::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccExtru::showNormals), "showNormals");
					m->add(fun(&ccExtru::isSerializable), "isSerializable");
					m->add(fun(&ccExtru::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccExtru::refreshBB), "refreshBB");
					m->add(fun(&ccExtru::capacity), "capacity");
					m->add(fun(&ccExtru::hasMaterials), "hasMaterials");
					m->add(fun(&ccExtru::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccExtru::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccExtru::hasTextures), "hasTextures");
					m->add(fun(&ccExtru::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccExtru::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccExtru::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccExtru::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccExtru::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccExtru::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccExtru::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccExtru::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccExtru::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccExtru::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccExtru::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccExtru::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccExtru::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccExtru::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccExtru::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccExtru::isShownAsWire), "isShownAsWire");
					m->add(fun(&ccExtru::showWired), "showWired");
					m->add(fun(&ccExtru::triNormsShown), "triNormsShown");
					m->add(fun(&ccExtru::showTriNorms), "showTriNorms");
					m->add(fun(&ccExtru::materialsShown), "materialsShown");
					m->add(fun(&ccExtru::showMaterials), "showMaterials");
					m->add(fun(&ccExtru::stipplingEnabled), "stipplingEnabled");
					m->add(fun(&ccExtru::enableStippling), "enableStippling");
					m->add(fun(&ccExtru::samplePoints), "samplePoints"); //TODO add default version(s)
					m->add(fun(&ccExtru::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccExtru::computePointPosition), "computePointPosition");
					m->add(fun(&ccExtru::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccExtru::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccExtru::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccExtru::getClassID), "getClassID");
					m->add(fun(&ccExtru::getName), "getName");
					m->add(fun(&ccExtru::setName), "setName");
					m->add(fun(&ccExtru::getUniqueID), "getUniqueID");
					m->add(fun(&ccExtru::setUniqueID), "setUniqueID");
					m->add(fun(&ccExtru::isEnabled), "isEnabled");
					m->add(fun(&ccExtru::setEnabled), "setEnabled");
					m->add(fun(&ccExtru::toggleActivation), "toggleActivation");
					m->add(fun(&ccExtru::isLocked), "isLocked");
					m->add(fun(&ccExtru::setLocked), "setLocked");
					m->add(fun(&ccExtru::isLeaf), "isLeaf");
					m->add(fun(&ccExtru::isCustom), "isCustom");
					m->add(fun(&ccExtru::isHierarchy), "isHierarchy");
					m->add(fun(&ccExtru::isKindOf), "isKindOf");
					m->add(fun(&ccExtru::isA), "isA");
					m->add(fun(&ccExtru::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccExtru::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccExtru::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccExtru::getMetaData), "getMetaData");
					m->add(fun(&ccExtru::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccExtru::*)(const  QString&, const QVariant&)>(&ccExtru::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccExtru::*)(const QVariantMap&, bool)>(&ccExtru::setMetaData)), "setMetaData");
					m->add(fun(&ccExtru::hasMetaData), "hasMetaData");
					m->add(fun(&ccExtru::metaData), "metaData");
					m->add(fun(&ccExtru::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccExtru::getOwnBB), "getOwnBB");
					m->add(fun(&ccExtru::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccExtru::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccExtru::getTypeName), "getTypeName");
					m->add(fun(&ccExtru::clone), "clone");
					m->add(fun(&ccExtru::setColor), "setColor");
					m->add(fun(&ccExtru::operator+=), "+=");
					m->add(fun(&ccExtru::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccExtru::setDrawingPrecision), "setDrawingPrecision");
					m->add(fun(&ccExtru::getDrawingPrecision), "getDrawingPrecision");
					m->add(fun(static_cast<ccGLMatrix & (ccExtru::*)()>(&ccExtru::getTransformation)), "getTransformation");
					m->add(fun(static_cast<const ccGLMatrix & (ccExtru::*)()const>(&ccExtru::getTransformation)), "getTransformation");
					m->add(fun(&ccExtru::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccExtru::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccExtru::getThickness), "getThickness");
					m->add(fun(&ccExtru::getProfile), "getProfile");

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccExtru>());
					m->add(chaiscript::base_class<ccHObject, ccExtru>());
					m->add(chaiscript::base_class<ccObject, ccExtru>());
					m->add(chaiscript::base_class<ccSerializableObject, ccExtru>());
					m->add(chaiscript::base_class<ccDrawableObject, ccExtru>());
					m->add(chaiscript::base_class<ccMesh, ccExtru>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccExtru>());
					m->add(chaiscript::base_class<ccGenericMesh, ccExtru>());

					return m;
				}

				ModulePtr bs_ccFacet(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::constructor<ccFacet(PointCoordinateType, QString)>(), "ccFacet");
					m->add(fun([]() {return new ccFacet(); }), "create_ccFacet");
					m->add(fun([](CCLib::GenericIndexedCloudPersist* cloud) {return ccFacet::Create(cloud); }), "create_ccFacet");
					m->add(fun([](CCLib::GenericIndexedCloudPersist* cloud, PointCoordinateType maxEL) {return ccFacet::Create(cloud, maxEL); }), "create_ccFacet");
					m->add(fun([](CCLib::GenericIndexedCloudPersist* cloud,PointCoordinateType maxEL, bool to) {return ccFacet::Create(cloud, maxEL, to); }), "create_ccFacet");
					m->add(fun(&ccFacet::Create), "Create");
					m->add(fun(&ccFacet::showNormals), "showNormals");
					m->add(fun(&ccFacet::isSerializable), "isSerializable");					
					m->add(fun(&ccFacet::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccFacet::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccFacet::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccFacet::getClassID), "getClassID");
					m->add(fun(&ccFacet::getName), "getName");
					m->add(fun(&ccFacet::setName), "setName");
					m->add(fun(&ccFacet::getUniqueID), "getUniqueID");
					m->add(fun(&ccFacet::setUniqueID), "setUniqueID");
					m->add(fun(&ccFacet::isEnabled), "isEnabled");
					m->add(fun(&ccFacet::setEnabled), "setEnabled");
					m->add(fun(&ccFacet::toggleActivation), "toggleActivation");
					m->add(fun(&ccFacet::isLocked), "isLocked");
					m->add(fun(&ccFacet::setLocked), "setLocked");
					m->add(fun(&ccFacet::isLeaf), "isLeaf");
					m->add(fun(&ccFacet::isCustom), "isCustom");
					m->add(fun(&ccFacet::isHierarchy), "isHierarchy");
					m->add(fun(&ccFacet::isKindOf), "isKindOf");
					m->add(fun(&ccFacet::isA), "isA");
					m->add(fun(&ccFacet::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccFacet::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccFacet::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccFacet::getMetaData), "getMetaData");
					m->add(fun(&ccFacet::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccFacet::*)(const  QString&, const QVariant&)>(&ccFacet::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccFacet::*)(const QVariantMap&, bool)>(&ccFacet::setMetaData)), "setMetaData");
					m->add(fun(&ccFacet::hasMetaData), "hasMetaData");
					m->add(fun(&ccFacet::metaData), "metaData");
					m->add(fun(&ccFacet::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccFacet::getOwnBB), "getOwnBB");
					m->add(fun(&ccFacet::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccFacet::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccFacet::clone), "clone");
					m->add(fun(&ccFacet::setColor), "setColor");
					m->add(fun(&ccFacet::getGLTransformationHistory), "getGLTransformationHistory");
					m->add(fun(&ccFacet::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccFacet::getCenter), "getCenter");				
					m->add(fun(&ccFacet::getNormal), "getNormal");
					m->add(fun(&ccFacet::getRMS), "getRMS");
					m->add(fun(&ccFacet::getSurface), "getSurface");
					m->add(fun(&ccFacet::getPlaneEquation), "getPlaneEquation");
					m->add(fun(&ccFacet::invertNormal), "invertNormal");
					m->add(fun(&ccFacet::getCenter), "getCenter");
					m->add(fun(static_cast<ccMesh*(ccFacet::*)()>(&ccFacet::getPolygon)), "getPolygon");
					m->add(fun(static_cast<const ccMesh*(ccFacet::*)()const>(&ccFacet::getPolygon)), "getPolygon");
					m->add(fun(static_cast<ccPolyline*(ccFacet::*)()>(&ccFacet::getContour)), "getContour");
					m->add(fun(static_cast<const ccPolyline*(ccFacet::*)()const>(&ccFacet::getContour)), "getContour");
					m->add(fun(static_cast<ccPointCloud*(ccFacet::*)()>(&ccFacet::getContourVertices)), "getContourVertices");
					m->add(fun(static_cast<const ccPointCloud*(ccFacet::*)()const>(&ccFacet::getContourVertices)), "getContourVertices");
					m->add(fun(static_cast<ccPointCloud*(ccFacet::*)()>(&ccFacet::getOriginPoints)), "getOriginPoints");
					m->add(fun(static_cast<const ccPointCloud*(ccFacet::*)()const>(&ccFacet::getOriginPoints)), "getOriginPoints");
					m->add(fun(&ccFacet::setPolygon), "setPolygon");
					m->add(fun(&ccFacet::setContour), "setContour");
					m->add(fun(&ccFacet::setContourVertices), "setContourVertices");
					m->add(fun(&ccFacet::setOriginPoints), "setOriginPoints");
					m->add(fun(&ccFacet::setColor), "setColor");

					m->add(chaiscript::base_class<ccHObject, ccFacet>());
					m->add(chaiscript::base_class<ccObject, ccFacet>());
					m->add(chaiscript::base_class<ccSerializableObject, ccFacet>());
					m->add(chaiscript::base_class<ccDrawableObject, ccFacet>());
					m->add(chaiscript::base_class<ccPlanarEntityInterface, ccFacet>());

					return m;
				}

				ModulePtr bs_ccShiftedObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccShiftedObject>(), "ccShiftedObject");
					m->add(chaiscript::constructor<ccShiftedObject(QString)>(), "ccShiftedObject");
					m->add(chaiscript::constructor<ccShiftedObject(const ccShiftedObject&)>(), "ccShiftedObject");
					m->add(fun(&ccShiftedObject::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccShiftedObject::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccShiftedObject::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccShiftedObject::getClassID), "getClassID");
					m->add(fun(&ccShiftedObject::getName), "getName");
					m->add(fun(&ccShiftedObject::setName), "setName");
					m->add(fun(&ccShiftedObject::getUniqueID), "getUniqueID");
					m->add(fun(&ccShiftedObject::setUniqueID), "setUniqueID");
					m->add(fun(&ccShiftedObject::isEnabled), "isEnabled");
					m->add(fun(&ccShiftedObject::setEnabled), "setEnabled");
					m->add(fun(&ccShiftedObject::toggleActivation), "toggleActivation");
					m->add(fun(&ccShiftedObject::isLocked), "isLocked");
					m->add(fun(&ccShiftedObject::setLocked), "setLocked");
					m->add(fun(&ccShiftedObject::isLeaf), "isLeaf");
					m->add(fun(&ccShiftedObject::isCustom), "isCustom");
					m->add(fun(&ccShiftedObject::isHierarchy), "isHierarchy");
					m->add(fun(&ccShiftedObject::isKindOf), "isKindOf");
					m->add(fun(&ccShiftedObject::isA), "isA");
					m->add(fun(&ccShiftedObject::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccShiftedObject::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccShiftedObject::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccShiftedObject::getMetaData), "getMetaData");
					m->add(fun(&ccShiftedObject::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccShiftedObject::*)(const  QString&, const QVariant&)>(&ccShiftedObject::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccShiftedObject::*)(const QVariantMap&, bool)>(&ccShiftedObject::setMetaData)), "setMetaData");
					m->add(fun(&ccShiftedObject::hasMetaData), "hasMetaData");
					m->add(fun(&ccShiftedObject::metaData), "metaData");
					m->add(fun(static_cast<void(ccShiftedObject::*)(const CCVector3d&)>(&ccShiftedObject::setGlobalShift)), "setGlobalShift");
					m->add(fun(static_cast<void(ccShiftedObject::*)(double, double, double)>(&ccShiftedObject::setGlobalShift)), "setGlobalShift");
					m->add(fun(&ccShiftedObject::getGlobalShift), "getGlobalShift");
					m->add(fun(&ccShiftedObject::setGlobalScale), "setGlobalScale");
					m->add(fun(&ccShiftedObject::isShifted), "isShifted");
					m->add(fun(&ccShiftedObject::getGlobalScale), "getGlobalScale");
					m->add(fun(static_cast<CCVector3d(ccShiftedObject::*)(const Vector3Tpl<float>&)const>(&ccShiftedObject::toGlobal3d)), "toGlobal3d");
					m->add(fun(static_cast<CCVector3d(ccShiftedObject::*)(const Vector3Tpl<double>&)const>(&ccShiftedObject::toGlobal3d)), "toGlobal3d");
					m->add(fun(static_cast<CCVector3d(ccShiftedObject::*)(const Vector3Tpl<float>&)const>(&ccShiftedObject::toLocal3d)), "toLocal3d");
					m->add(fun(static_cast<CCVector3d(ccShiftedObject::*)(const Vector3Tpl<double>&)const>(&ccShiftedObject::toLocal3d)), "toLocal3d");
					m->add(fun(static_cast<CCVector3(ccShiftedObject::*)(const Vector3Tpl<float>&)const>(&ccShiftedObject::toLocal3pc)), "toLocal3pc");
					m->add(fun(static_cast<CCVector3(ccShiftedObject::*)(const Vector3Tpl<double>&)const>(&ccShiftedObject::toLocal3pc)), "toLocal3pc");
					m->add(fun(&ccShiftedObject::getGlobalBB), "getGlobalBB");		

					m->add(chaiscript::base_class<ccHObject, ccShiftedObject>());
					m->add(chaiscript::base_class<ccObject, ccShiftedObject>());
					m->add(chaiscript::base_class<ccSerializableObject, ccShiftedObject>());
					m->add(chaiscript::base_class<ccDrawableObject, ccShiftedObject>());

					return m;
				}

				ModulePtr bs_ccGenericPointCloud(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGenericPointCloud>(), "ccGenericPointCloud");
					m->add(fun(&ccGenericPointCloud::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccGenericPointCloud::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccGenericPointCloud::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccGenericPointCloud::getClassID), "getClassID");
					m->add(fun(&ccGenericPointCloud::getName), "getName");
					m->add(fun(&ccGenericPointCloud::setName), "setName");
					m->add(fun(&ccGenericPointCloud::getUniqueID), "getUniqueID");
					m->add(fun(&ccGenericPointCloud::setUniqueID), "setUniqueID");
					m->add(fun(&ccGenericPointCloud::isEnabled), "isEnabled");
					m->add(fun(&ccGenericPointCloud::setEnabled), "setEnabled");
					m->add(fun(&ccGenericPointCloud::toggleActivation), "toggleActivation");
					m->add(fun(&ccGenericPointCloud::isLocked), "isLocked");
					m->add(fun(&ccGenericPointCloud::setLocked), "setLocked");
					m->add(fun(&ccGenericPointCloud::isLeaf), "isLeaf");
					m->add(fun(&ccGenericPointCloud::isCustom), "isCustom");
					m->add(fun(&ccGenericPointCloud::isHierarchy), "isHierarchy");
					m->add(fun(&ccGenericPointCloud::isKindOf), "isKindOf");
					m->add(fun(&ccGenericPointCloud::isA), "isA");
					m->add(fun(&ccGenericPointCloud::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccGenericPointCloud::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccGenericPointCloud::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccGenericPointCloud::getMetaData), "getMetaData");
					m->add(fun(&ccGenericPointCloud::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccGenericPointCloud::*)(const  QString&, const QVariant&)>(&ccGenericPointCloud::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccGenericPointCloud::*)(const QVariantMap&, bool)>(&ccGenericPointCloud::setMetaData)), "setMetaData");
					m->add(fun(&ccGenericPointCloud::hasMetaData), "hasMetaData");
					m->add(fun(&ccGenericPointCloud::metaData), "metaData");
					m->add(fun(&ccGenericPointCloud::clone), "clone");
					m->add(fun(&ccGenericPointCloud::clear), "clear");
					m->add(fun(&ccGenericPointCloud::computeOctree), "computeOctree");
					m->add(fun(&ccGenericPointCloud::getOctree), "getOctree");
					m->add(fun(&ccGenericPointCloud::setOctree), "setOctree");
					m->add(fun(&ccGenericPointCloud::getOctreeProxy), "getOctreeProxy");
					m->add(fun(&ccGenericPointCloud::deleteOctree), "deleteOctree");
					m->add(fun(&ccGenericPointCloud::geScalarValueColor), "geScalarValueColor");
					m->add(fun(&ccGenericPointCloud::getPointScalarValueColor), "getPointScalarValueColor");
					m->add(fun(&ccGenericPointCloud::getPointDisplayedDistance), "getPointDisplayedDistance");
					m->add(fun(&ccGenericPointCloud::getPointColor), "getPointColor");
					m->add(fun(&ccGenericPointCloud::getPointNormalIndex), "getPointNormalIndex");
					m->add(fun(&ccGenericPointCloud::getPointNormal), "getPointNormal");
					m->add(chaiscript::user_type<ccGenericPointCloud::VisibilityTableType>(), "VisibilityTableType");
					m->add(fun(static_cast<ccGenericPointCloud::VisibilityTableType&(ccGenericPointCloud::*)()>(&ccGenericPointCloud::getTheVisibilityArray)), "getTheVisibilityArray");
					m->add(fun(static_cast<const ccGenericPointCloud::VisibilityTableType& (ccGenericPointCloud::*)()const>(&ccGenericPointCloud::getTheVisibilityArray)), "getTheVisibilityArray");
					m->add(fun(&ccGenericPointCloud::getTheVisiblePoints), "getTheVisiblePoints");
					m->add(fun(&ccGenericPointCloud::isVisibilityTableInstantiated), "isVisibilityTableInstantiated");
					m->add(fun(&ccGenericPointCloud::resetVisibilityArray), "resetVisibilityArray");
					m->add(fun(&ccGenericPointCloud::invertVisibilityArray), "invertVisibilityArray");
					m->add(fun(&ccGenericPointCloud::unallocateVisibilityArray), "unallocateVisibilityArray");
					m->add(fun(&ccGenericPointCloud::getOwnBB), "getOwnBB");
					m->add(fun(&ccGenericPointCloud::refreshBB), "refreshBB");
					m->add(fun(&ccGenericPointCloud::createNewCloudFromVisibilitySelection), "createNewCloudFromVisibilitySelection");
					m->add(fun(&ccGenericPointCloud::applyRigidTransformation), "applyRigidTransformation");
					m->add(fun(&ccGenericPointCloud::crop), "crop");
					m->add(fun(&ccGenericPointCloud::scale), "scale");
					m->add(fun(&ccGenericPointCloud::isSerializable), "isSerializable");
					m->add(fun(&ccGenericPointCloud::setPointSize), "setPointSize");
					m->add(fun(&ccGenericPointCloud::getPointSize), "getPointSize");
					m->add(fun(&ccGenericPointCloud::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccGenericPointCloud::pointPicking), "pointPicking");

					m->add(chaiscript::base_class<ccHObject, ccGenericPointCloud>());
					m->add(chaiscript::base_class<ccObject, ccGenericPointCloud>());
					m->add(chaiscript::base_class<ccSerializableObject, ccGenericPointCloud>());
					m->add(chaiscript::base_class<ccDrawableObject, ccGenericPointCloud>());
					m->add(chaiscript::base_class<ccShiftedObject, ccGenericPointCloud>());
					m->add(chaiscript::base_class<CCLib::GenericIndexedCloudPersist, ccGenericPointCloud>());


					return m;
				}


				ModulePtr bs_ccPointCloud(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccPointCloud>(), "ccPointCloud");
					m->add(fun(&ccPointCloud::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccPointCloud::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccPointCloud::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccPointCloud::getClassID), "getClassID");
					m->add(fun(&ccPointCloud::getName), "getName");
					m->add(fun(&ccPointCloud::setName), "setName");
					m->add(fun(&ccPointCloud::getUniqueID), "getUniqueID");
					m->add(fun(&ccPointCloud::setUniqueID), "setUniqueID");
					m->add(fun(&ccPointCloud::isEnabled), "isEnabled");
					m->add(fun(&ccPointCloud::setEnabled), "setEnabled");
					m->add(fun(&ccPointCloud::toggleActivation), "toggleActivation");
					m->add(fun(&ccPointCloud::isLocked), "isLocked");
					m->add(fun(&ccPointCloud::setLocked), "setLocked");
					m->add(fun(&ccPointCloud::isLeaf), "isLeaf");
					m->add(fun(&ccPointCloud::isCustom), "isCustom");
					m->add(fun(&ccPointCloud::isHierarchy), "isHierarchy");
					m->add(fun(&ccPointCloud::isKindOf), "isKindOf");
					m->add(fun(&ccPointCloud::isA), "isA");
					m->add(fun(&ccPointCloud::GetNextUniqueID), "GetNextUniqueID");
					m->add(fun(&ccPointCloud::GetLastUniqueID), "GetLastUniqueID");
					m->add(fun(&ccPointCloud::ReadClassIDFromFile), "ReadClassIDFromFile");
					m->add(fun(&ccPointCloud::getMetaData), "getMetaData");
					m->add(fun(&ccPointCloud::removeMetaData), "removeMetaData");
					m->add(fun(static_cast<void(ccPointCloud::*)(const  QString&, const QVariant&)>(&ccPointCloud::setMetaData)), "setMetaData");
					m->add(fun(static_cast<void(ccPointCloud::*)(const QVariantMap&, bool)>(&ccPointCloud::setMetaData)), "setMetaData");
					m->add(fun(&ccPointCloud::hasMetaData), "hasMetaData");
					m->add(fun(&ccPointCloud::metaData), "metaData");

					//m->add(fun(static_cast<ccPointCloud*(ccPointCloud::*)(const CCLib::GenericIndexedCloud*, const ccGenericPointCloud*)>(&ccPointCloud::From)), "From");
					//m->add(fun(static_cast<ccPointCloud* (ccPointCloud::*)(const CCLib::GenericCloud*, const ccGenericPointCloud*)>(&ccPointCloud::From)), "From");

					m->add(fun(&ccPointCloud::partialClone), "partialClone");
					m->add(fun(&ccPointCloud::cloneThis), "cloneThis");
					m->add(fun(&ccPointCloud::clone), "clone");
					m->add(fun(&ccPointCloud::operator+=), "+=");
					m->add(fun(&ccPointCloud::clear), "clear");
					m->add(fun(&ccPointCloud::unallocatePoints), "unallocatePoints");
					m->add(fun(&ccPointCloud::unallocateColors), "unallocateColors");
					m->add(fun(&ccPointCloud::unallocateNorms), "unallocateNorms");
					m->add(fun(&ccPointCloud::colorsHaveChanged), "colorsHaveChanged");
					m->add(fun(&ccPointCloud::normalsHaveChanged), "normalsHaveChanged");
					m->add(fun(&ccPointCloud::pointsHaveChanged), "pointsHaveChanged");
					m->add(fun(&ccPointCloud::reserveThePointsTable), "reserveThePointsTable");
					m->add(fun(&ccPointCloud::reserveTheRGBTable), "reserveTheRGBTable");
					m->add(fun(&ccPointCloud::resizeTheRGBTable), "resizeTheRGBTable");
					m->add(fun(&ccPointCloud::reserveTheNormsTable), "reserveTheNormsTable");
					m->add(fun(&ccPointCloud::resizeTheNormsTable), "resizeTheNormsTable");
					m->add(fun(&ccPointCloud::reserve), "reserve");
					m->add(fun(&ccPointCloud::resize), "resize");
					m->add(fun(&ccPointCloud::shrinkToFit), "shrinkToFit");
					m->add(fun(&ccPointCloud::getCurrentDisplayedScalarField), "getCurrentDisplayedScalarField");
					m->add(fun(&ccPointCloud::getCurrentDisplayedScalarFieldIndex), "getCurrentDisplayedScalarFieldIndex");
					m->add(fun(&ccPointCloud::setCurrentDisplayedScalarField), "setCurrentDisplayedScalarField");
					m->add(fun(&ccPointCloud::deleteScalarField), "deleteScalarField");
					m->add(fun(&ccPointCloud::deleteAllScalarFields), "deleteAllScalarFields");
					m->add(fun(static_cast<int(ccPointCloud::*)(const char*)>(&ccPointCloud::addScalarField)), "addScalarField");
					m->add(fun(&ccPointCloud::sfColorScaleShown), "sfColorScaleShown");
					m->add(fun(&ccPointCloud::showSFColorsScale), "showSFColorsScale");
					//GRID
					m->add(chaiscript::user_type<ccPointCloud::Grid>(), "Grid");
					m->add(chaiscript::constructor<ccPointCloud::Grid()>(), "Grid");
					m->add(chaiscript::constructor<ccPointCloud::Grid(const ccPointCloud::Grid&)>(), "Grid");
					m->add(fun(&ccPointCloud::Grid::toImage), "toImage");
					m->add(fun(&ccPointCloud::Grid::w), "w");
					m->add(fun(&ccPointCloud::Grid::h), "h");
					m->add(fun(&ccPointCloud::Grid::validCount), "validCount");
					m->add(fun(&ccPointCloud::Grid::minValidIndex), "minValidIndex");
					m->add(fun(&ccPointCloud::Grid::maxValidIndex), "maxValidIndex");
					m->add(fun(&ccPointCloud::Grid::indexes), "indexes");
					m->add(chaiscript::vector_conversion<std::vector<ccColor::Rgb>>());
					m->add(fun(&ccPointCloud::Grid::sensorPosition), "sensorPosition");

					m->add(fun(&ccPointCloud::gridCount), "gridCount");
					m->add(fun(static_cast<ccPointCloud::Grid::Shared&(ccPointCloud::*)(size_t)>(&ccPointCloud::grid)), "grid");
					m->add(fun(static_cast<const ccPointCloud::Grid::Shared&(ccPointCloud::*)(size_t)const>(&ccPointCloud::grid)), "grid");
					m->add(fun(&ccPointCloud::addGrid), "addGrid");
					m->add(fun(&ccPointCloud::removeGrids), "removeGrids");
					m->add(fun(&ccPointCloud::triangulateGrid), "triangulateGrid");
					m->add(fun(&ccPointCloud::computeNormalsWithGrids), "computeNormalsWithGrids");
					m->add(fun(&ccPointCloud::orientNormalsWithGrids), "orientNormalsWithGrids");
					m->add(fun(&ccPointCloud::orientNormalsTowardViewPoint), "orientNormalsTowardViewPoint");
					m->add(fun(&ccPointCloud::computeNormalsWithOctree), "computeNormalsWithOctree");
					m->add(fun(&ccPointCloud::orientNormalsWithMST), "orientNormalsWithMST");
					m->add(fun(&ccPointCloud::orientNormalsWithFM), "orientNormalsWithFM");
					m->add(fun(&ccPointCloud::hasFWF), "hasFWF");
					m->add(fun(&ccPointCloud::waveformProxy), "waveformProxy");
					m->add(chaiscript::user_type<ccPointCloud::FWFDescriptorSet>(), "FWFDescriptorSet");
					m->add(chaiscript::user_type<ccPointCloud::FWFDataContainer>(), "FWFDescriptorSet");
					m->add(chaiscript::user_type<ccPointCloud::SharedFWFDataContainer>(), "FWFDescriptorSet");
					m->add(chaiscript::vector_conversion<std::vector<uint8_t>>());
					m->add(fun(static_cast<ccPointCloud::FWFDescriptorSet&(ccPointCloud::*)()>(&ccPointCloud::fwfDescriptors)), "fwfDescriptors");
					m->add(fun(static_cast<const ccPointCloud::FWFDescriptorSet & (ccPointCloud::*)()const>(&ccPointCloud::fwfDescriptors)), "fwfDescriptors");
					m->add(fun(static_cast<std::vector<ccWaveform> & (ccPointCloud::*)()>(&ccPointCloud::waveforms)), "waveforms");
					m->add(fun(static_cast<const std::vector<ccWaveform> & (ccPointCloud::*)()const>(&ccPointCloud::waveforms)), "waveforms");
					m->add(chaiscript::vector_conversion<std::vector<ccWaveform>>());
					m->add(fun(&ccPointCloud::reserveTheFWFTable), "reserveTheFWFTable");
					m->add(fun(&ccPointCloud::resizeTheFWFTable), "resizeTheFWFTable");
					m->add(fun(static_cast<ccPointCloud::SharedFWFDataContainer& (ccPointCloud::*)()>(&ccPointCloud::fwfData)), "fwfData");
					m->add(fun(static_cast<const ccPointCloud::SharedFWFDataContainer& (ccPointCloud::*)()const>(&ccPointCloud::fwfData)), "fwfData");
					m->add(fun(&ccPointCloud::compressFWFData), "compressFWFData");
					m->add(fun(&ccPointCloud::computeFWFAmplitude), "computeFWFAmplitude");
					m->add(fun(&ccPointCloud::clearFWFData), "clearFWFData");
					m->add(fun(&ccPointCloud::computeGravityCenter), "computeGravityCenter");
					m->add(fun(&ccPointCloud::invalidateBoundingBox), "invalidateBoundingBox");
					m->add(fun(&ccPointCloud::getDrawingParameters), "getDrawingParameters");
					m->add(fun(&ccPointCloud::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccPointCloud::hasColors), "hasColors");
					m->add(fun(&ccPointCloud::hasNormals), "hasNormals");
					m->add(fun(&ccPointCloud::hasScalarFields), "hasScalarFields");
					m->add(fun(&ccPointCloud::hasDisplayedScalarField), "hasDisplayedScalarField");
					m->add(fun(&ccPointCloud::removeFromDisplay), "removeFromDisplay");
					m->add(fun(&ccPointCloud::setDisplay), "setDisplay");
					m->add(fun(&ccPointCloud::testVisibility), "testVisibility");
					m->add(fun(&ccPointCloud::geScalarValueColor), "geScalarValueColor");
					m->add(fun(&ccPointCloud::getPointScalarValueColor), "getPointScalarValueColor");
					m->add(fun(&ccPointCloud::getPointDisplayedDistance), "getPointDisplayedDistance");
					m->add(fun(&ccPointCloud::getPointColor), "getPointColor");
					m->add(fun(&ccPointCloud::getPointNormalIndex), "getPointNormalIndex");
					m->add(fun(&ccPointCloud::getPointNormal), "getPointNormal");
					m->add(fun(&ccPointCloud::crop), "crop");
					m->add(fun(&ccPointCloud::scale), "scale");
					m->add(fun(&ccPointCloud::createNewCloudFromVisibilitySelection), "createNewCloudFromVisibilitySelection");
					m->add(fun(&ccPointCloud::applyRigidTransformation), "applyRigidTransformation");
					m->add(fun(&ccPointCloud::refreshBB), "refreshBB");
					m->add(fun(&ccPointCloud::enableVisibilityCheck), "enableVisibilityCheck");
					m->add(fun(&ccPointCloud::hasSensor), "hasSensor");
					m->add(fun(&ccPointCloud::computeCPSet), "computeCPSet");
					m->add(chaiscript::user_type<QSharedPointer<CCLib::ReferenceCloud>>(), "ReferenceCloud");
					m->add(fun(&ccPointCloud::interpolateColorsFrom), "interpolateColorsFrom");
					m->add(fun(static_cast<void(ccPointCloud::*)(unsigned, const ccColor::Rgb&)> (&ccPointCloud::setPointColor)), "setPointColor");
					m->add(fun(static_cast<void(ccPointCloud::*)(unsigned, const ccColor::Rgba&)> (&ccPointCloud::setPointColor)), "setPointColor");
					m->add(fun(&ccPointCloud::setPointNormalIndex), "setPointNormalIndex");
					m->add(fun(&ccPointCloud::setPointNormal), "setPointNormal");
					m->add(fun(&ccPointCloud::addNormIndex), "addNormIndex");
					m->add(fun(&ccPointCloud::addNorm), "addNorm");
					m->add(fun(&ccPointCloud::addNormAtIndex), "addNormAtIndex");
					m->add(fun(&ccPointCloud::setNormsTable), "setNormsTable");
					m->add(fun(&ccPointCloud::convertNormalToRGB), "convertNormalToRGB");
					m->add(fun(&ccPointCloud::convertNormalToDipDirSFs), "convertNormalToDipDirSFs");
					m->add(fun(static_cast<void(ccPointCloud::*)(const ccColor::Rgba&)>(&ccPointCloud::addColor)), "addColor");
					m->add(fun(static_cast<void(ccPointCloud::*)(const ccColor::Rgb&)>(&ccPointCloud::addColor)), "addColor");
					m->add(fun(static_cast<void(ccPointCloud::*)(ColorCompType, ColorCompType, ColorCompType, ColorCompType)>(&ccPointCloud::addColor)), "addColor");
					m->add(fun(&ccPointCloud::addGreyColor), "addGreyColor");
					m->add(fun(&ccPointCloud::convertRGBToGreyScale), "convertRGBToGreyScale");
					m->add(fun(&ccPointCloud::colorize), "colorize");
					m->add(fun(&ccPointCloud::setRGBColorByHeight), "setRGBColorByHeight");
					m->add(fun(&ccPointCloud::setRGBColorByBanding), "setRGBColorByBanding");
					m->add(fun(&ccPointCloud::convertCurrentScalarFieldToColors), "convertCurrentScalarFieldToColors");
					m->add(fun(static_cast<bool(ccPointCloud::*)(ColorCompType, ColorCompType, ColorCompType, ColorCompType)>(&ccPointCloud::setColor)), "setColor");
					m->add(fun(static_cast<bool(ccPointCloud::*)(const ccColor::Rgb&)>(&ccPointCloud::setColor)), "setColor");
					m->add(fun(static_cast<bool(ccPointCloud::*)(const ccColor::Rgba&)>(&ccPointCloud::setColor)), "setColor");
					m->add(fun(&ccPointCloud::invertNormals), "invertNormals");
					m->add(fun(&ccPointCloud::translate), "translate");
					m->add(fun(&ccPointCloud::filterPointsByScalarValue), "filterPointsByScalarValue");
					m->add(fun(&ccPointCloud::hidePointsByScalarValue), "hidePointsByScalarValue");
					///UNROLL STUFF
					m->add(chaiscript::user_type<ccPointCloud::UnrollBaseParams>(), "UnrollBaseParams");
					m->add(fun(&ccPointCloud::UnrollBaseParams::radius), "radius");
					m->add(fun(&ccPointCloud::UnrollBaseParams::axisDim), "axisDim");
					m->add(chaiscript::user_type<ccPointCloud::UnrollCylinderParams>(), "UnrollCylinderParams");
					m->add(fun(&ccPointCloud::UnrollCylinderParams::center), "center");
					m->add(chaiscript::user_type<ccPointCloud::UnrollConeParams>(), "UnrollConeParams");
					m->add(fun(&ccPointCloud::UnrollConeParams::apex), "apex");
					m->add(fun(&ccPointCloud::UnrollConeParams::coneAngle_deg), "coneAngle_deg");
					m->add(chaiscript::base_class<ccPointCloud::UnrollBaseParams, ccPointCloud::UnrollCylinderParams>());
					m->add(chaiscript::base_class<ccPointCloud::UnrollBaseParams, ccPointCloud::UnrollConeParams>());

					m->add(fun(&ccPointCloud::unroll), "unroll");
					m->add(fun(&ccPointCloud::addColorRampInfo), "addColorRampInfo");
					m->add(fun(static_cast<int(ccPointCloud::*)(ccScalarField*)>(&ccPointCloud::addScalarField)), "addScalarField");
					m->add(fun(&ccPointCloud::rgbaColors), "rgbaColors");
					m->add(fun(&ccPointCloud::normals), "normals");
					m->add(fun(&ccPointCloud::crop2D), "crop2D");
					m->add(fun(&ccPointCloud::append), "append");
					m->add(fun(&ccPointCloud::enhanceRGBWithIntensitySF), "enhanceRGBWithIntensitySF");
					m->add(fun(&ccPointCloud::exportCoordToSF), "exportCoordToSF");
					m->add(fun(&ccPointCloud::exportNormalToSF), "exportNormalToSF");

					m->add(chaiscript::base_class<ccHObject, ccPointCloud>());
					m->add(chaiscript::base_class<ccObject, ccPointCloud>());
					m->add(chaiscript::base_class<ccSerializableObject, ccPointCloud>());
					m->add(chaiscript::base_class<ccDrawableObject, ccPointCloud>());
					m->add(chaiscript::base_class<ccShiftedObject, ccPointCloud>());
					m->add(chaiscript::base_class<CCLib::PointCloudTpl<ccGenericPointCloud>, ccPointCloud>());
					m->add(chaiscript::base_class<ccGenericPointCloud, ccPointCloud>());
					m->add(chaiscript::base_class<CCLib::GenericIndexedCloudPersist, ccPointCloud>());

					return m;
				}


				ModulePtr bs_ccKdTree(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccKdTree>(), "ccKdTree");
					m->add(chaiscript::constructor<ccKdTree(ccGenericPointCloud*)>(), "ccKdTree");
					m->add(fun(&ccKdTree::multiplyBoundingBox), "multiplyBoundingBox");
					m->add(fun(&ccKdTree::translateBoundingBox), "translateBoundingBox");
					m->add(fun(&ccKdTree::getClassID), "getClassID");
					m->add(fun(&ccKdTree::getOwnBB), "getOwnBB");
					m->add(fun(&ccKdTree::convertCellIndexToSF), "convertCellIndexToSF");
					m->add(fun(&ccKdTree::convertCellIndexToRandomColor), "convertCellIndexToRandomColor");
					m->add(fun(&ccKdTree::getCellBBox), "getCellBBox");
					m->add(chaiscript::user_type<ccKdTree::LeafSet>(), "LeafSet");
					m->add(fun(&ccKdTree::getNeighborLeaves), "getNeighborLeaves");
					m->add(fun(&ccKdTree::associatedGenericCloud), "associatedGenericCloud");


					m->add(chaiscript::base_class<ccObject, ccKdTree>());
					m->add(chaiscript::base_class<ccDrawableObject, ccKdTree>());
					m->add(chaiscript::base_class<ccHObject, ccKdTree>());
					m->add(chaiscript::base_class<ccSerializableObject, ccKdTree>());
					m->add(chaiscript::base_class<CCLib::TrueKdTree, ccKdTree>());


					return m;
				}


				ModulePtr bs_ccOctree(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccOctree>(), "ccOctree");
					m->add(chaiscript::user_type<ccOctree::Shared>(), "Shared");
					m->add(chaiscript::constructor<ccOctree(ccGenericPointCloud*)>(), "ccOctree");
					m->add(fun(&ccOctree::multiplyBoundingBox), "multiplyBoundingBox");
					m->add(fun(&ccOctree::translateBoundingBox), "translateBoundingBox");
					m->add(fun(&ccOctree::getSquareBB), "getSquareBB");
					m->add(fun(&ccOctree::getPointsBB), "getPointsBB");
					m->add(fun(&ccOctree::clear), "clear");
					m->add(fun(&ccOctree::getDisplayedLevel), "getDisplayedLevel");
					m->add(fun(&ccOctree::setDisplayedLevel), "setDisplayedLevel");
					m->add(fun(&ccOctree::getDisplayMode), "getDisplayMode");
					m->add(fun(&ccOctree::setDisplayMode), "setDisplayMode");
					m->add(fun(&ccOctree::draw), "draw");
					m->add(fun(&ccOctree::intersectWithFrustum), "intersectWithFrustum");
					m->add(fun(&ccOctree::pointPicking), "pointPicking");
					m->add(fun(&ccOctree::ComputeAverageColor), "ComputeAverageColor");
					m->add(fun(&ccOctree::ComputeAverageNorm), "ComputeAverageNorm");
					m->add(fun(&ccOctree::updated), "updated");
					

					m->add(chaiscript::base_class<QObject, ccOctree>());
					m->add(chaiscript::base_class<CCLib::DgmOctree, ccOctree>());
					


					return m;
				}

				ModulePtr bs_ccOctreeProxy(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccOctreeProxy>(), "ccOctreeProxy");

					m->add(chaiscript::constructor<ccOctreeProxy(ccOctree::Shared)>(), "ccOctreeProxy");
					m->add(fun(&ccOctreeProxy::setOctree), "setOctree");
					m->add(fun(&ccOctreeProxy::getOctree), "getOctree");
					m->add(fun(&ccOctreeProxy::getClassID), "getClassID");
					m->add(fun(&ccOctreeProxy::getOwnBB), "getOwnBB");
					

					//m->add(chaiscript::base_class<QObject, ccOctreeProxy>());
					//m->add(chaiscript::base_class<CCLib::DgmOctree, ccOctreeProxy>());
					m->add(chaiscript::base_class<ccHObject, ccOctreeProxy>());
					m->add(chaiscript::base_class<ccObject, ccOctreeProxy>());
					m->add(chaiscript::base_class<ccSerializableObject, ccOctreeProxy>());
					m->add(chaiscript::base_class<ccDrawableObject, ccOctreeProxy>());


					return m;
				}

				ModulePtr bs_ccPolyline(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccPolyline>(), "ccPolyline");

					m->add(chaiscript::constructor<ccPolyline(CCLib::GenericIndexedCloudPersist*)>(), "ccPolyline");
					m->add(chaiscript::constructor<ccPolyline(const ccPolyline&)>(), "ccPolyline");
					m->add(fun(&ccPolyline::getClassID), "getClassID");
					m->add(fun(&ccPolyline::isSerializable), "isSerializable");
					m->add(fun(&ccPolyline::hasColors), "hasColors");
					m->add(fun(&ccPolyline::applyGLTransformation), "applyGLTransformation");
					m->add(fun(&ccPolyline::getUniqueIDForDisplay), "getUniqueIDForDisplay");
					m->add(fun(&ccPolyline::setGlobalShift), "setGlobalShift");
					m->add(fun(&ccPolyline::setGlobalScale), "setGlobalScale");
					m->add(fun(&ccPolyline::set2DMode), "set2DMode");
					m->add(fun(&ccPolyline::is2DMode), "is2DMode");
					m->add(fun(&ccPolyline::setForeground), "setForeground");
					m->add(fun(&ccPolyline::setColor), "setColor");
					m->add(fun(&ccPolyline::setWidth), "setWidth");
					m->add(fun(&ccPolyline::getWidth), "getWidth");
					m->add(fun(&ccPolyline::getColor), "getColor");
					m->add(fun(&ccPolyline::getOwnBB), "getOwnBB");
					m->add(fun(&ccPolyline::drawBB), "drawBB");
					m->add(fun(&ccPolyline::split), "split");
					m->add(fun(&ccPolyline::computeLength), "computeLength");
					m->add(fun(&ccPolyline::showVertices), "showVertices");
					m->add(fun(&ccPolyline::verticesShown), "verticesShown");
					m->add(fun(&ccPolyline::setVertexMarkerWidth), "setVertexMarkerWidth");
					m->add(fun(&ccPolyline::getVertexMarkerWidth), "getVertexMarkerWidth");
					m->add(fun(&ccPolyline::initWith), "initWith");
					m->add(fun(&ccPolyline::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccPolyline::showArrow), "showArrow");
					m->add(fun(&ccPolyline::segmentCount), "segmentCount");
					m->add(fun(&ccPolyline::samplePoints), "samplePoints");
					m->add(fun(&ccPolyline::MetaKeyUpDir), "MetaKeyUpDir");
					m->add(fun(&ccPolyline::MetaKeyConstAltitude), "MetaKeyConstAltitude");
					m->add(fun(&ccPolyline::MetaKeyAbscissa), "MetaKeyAbscissa");
					m->add(fun(&ccPolyline::MetaKeyPrefixCenter), "MetaKeyPrefixCenter");
					m->add(fun(&ccPolyline::MetaKeyPrefixDirection), "MetaKeyPrefixDirection");
				

					
					m->add(chaiscript::base_class<CCLib::Polyline, ccPolyline>());
					m->add(chaiscript::base_class<ccHObject, ccPolyline>());
					m->add(chaiscript::base_class<ccObject, ccPolyline>());
					m->add(chaiscript::base_class<ccSerializableObject, ccPolyline>());
					m->add(chaiscript::base_class<ccDrawableObject, ccPolyline>());
					m->add(chaiscript::base_class<ccShiftedObject, ccPolyline>());

					return m;
				}

				ModulePtr bs_ccSubMesh(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccSubMesh>(), "ccSubMesh");

					m->add(chaiscript::constructor<ccSubMesh(ccMesh*)>(), "ccSubMesh");
					m->add(fun(&ccSubMesh::getClassID), "getClassID");
					m->add(fun(&ccSubMesh::getOwnBB), "getOwnBB");
					m->add(fun(&ccSubMesh::isSerializable), "isSerializable");
					m->add(fun(&ccSubMesh::getAssociatedCloud), "getAssociatedCloud");
					m->add(fun(&ccSubMesh::refreshBB), "refreshBB");
					m->add(fun(&ccSubMesh::interpolateNormals), "interpolateNormals");
					m->add(fun(&ccSubMesh::interpolateNormalsBC), "interpolateNormalsBC");
					m->add(fun(static_cast<bool(ccSubMesh::*)(unsigned, const CCVector3&, ccColor::Rgba&)>(&ccSubMesh::interpolateColors)), "interpolateColors");
					m->add(fun(static_cast<bool(ccSubMesh::*)(unsigned, const CCVector3&, ccColor::Rgb&)>(&ccSubMesh::interpolateColors)), "interpolateColors");
					m->add(fun(static_cast<bool(ccSubMesh::*)(unsigned, const CCVector3d&, ccColor::Rgba&)>(&ccSubMesh::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(static_cast<bool(ccSubMesh::*)(unsigned, const CCVector3d&, ccColor::Rgb&)>(&ccSubMesh::interpolateColorsBC)), "interpolateColorsBC");
					m->add(fun(&ccSubMesh::getColorFromMaterial), "getColorFromMaterial");
					m->add(fun(&ccSubMesh::getVertexColorFromMaterial), "getVertexColorFromMaterial");
					m->add(fun(&ccSubMesh::hasMaterials), "hasMaterials");
					m->add(fun(&ccSubMesh::getMaterialSet), "getMaterialSet");
					m->add(fun(&ccSubMesh::getTriangleMtlIndex), "getTriangleMtlIndex");
					m->add(fun(&ccSubMesh::hasTextures), "hasTextures");
					m->add(fun(&ccSubMesh::getTexCoordinatesTable), "getTexCoordinatesTable");
					m->add(fun(&ccSubMesh::getTriangleTexCoordinates), "getTriangleTexCoordinates");
					m->add(fun(&ccSubMesh::hasPerTriangleTexCoordIndexes), "hasPerTriangleTexCoordIndexes");
					m->add(fun(&ccSubMesh::getTriangleTexCoordinatesIndexes), "getTriangleTexCoordinatesIndexes");
					m->add(fun(&ccSubMesh::hasTriNormals), "hasTriNormals");
					m->add(fun(&ccSubMesh::getTriangleNormalIndexes), "getTriangleNormalIndexes");
					m->add(fun(&ccSubMesh::getTriangleNormals), "getTriangleNormals");
					m->add(fun(&ccSubMesh::getTriNormsTable), "getTriNormsTable");
					m->add(fun(&ccSubMesh::capacity), "capacity");
					m->add(fun(&ccSubMesh::hasColors), "hasColors");
					m->add(fun(&ccSubMesh::hasNormals), "hasNormals");
					m->add(fun(&ccSubMesh::hasScalarFields), "hasScalarFields");
					m->add(fun(&ccSubMesh::hasDisplayedScalarField), "hasDisplayedScalarField");
					m->add(fun(&ccSubMesh::normalsShown), "normalsShown");
					m->add(fun(&ccSubMesh::size), "size");
					m->add(fun(&ccSubMesh::forEach), "forEach");
					m->add(fun(&ccSubMesh::placeIteratorAtBeginning), "placeIteratorAtBeginning");
					m->add(fun(&ccSubMesh::_getNextTriangle), "_getNextTriangle");
					m->add(fun(&ccSubMesh::_getTriangle), "_getTriangle");
					m->add(fun(&ccSubMesh::getNextTriangleVertIndexes), "getNextTriangleVertIndexes");
					m->add(fun(&ccSubMesh::getTriangleVertIndexes), "getTriangleVertIndexes");
					m->add(fun(&ccSubMesh::getTriangleVertices), "getTriangleVertices");
					m->add(fun(&ccSubMesh::getBoundingBox), "getBoundingBox");
					m->add(fun(&ccSubMesh::getTriGlobalIndex), "getTriGlobalIndex");
					m->add(fun(&ccSubMesh::getCurrentTriGlobalIndex), "getCurrentTriGlobalIndex");
					m->add(fun(&ccSubMesh::forwardIterator), "forwardIterator");
					m->add(fun(&ccSubMesh::clear), "clear");
					m->add(fun(static_cast<bool(ccSubMesh::*)(unsigned)>(&ccSubMesh::addTriangleIndex)), "addTriangleIndex");
					m->add(fun(static_cast<bool(ccSubMesh::*)(unsigned,unsigned)>(&ccSubMesh::addTriangleIndex)), "addTriangleIndex");
					m->add(fun(&ccSubMesh::setTriangleIndex), "setTriangleIndex");
					m->add(fun(&ccSubMesh::reserve), "reserve");
					m->add(fun(&ccSubMesh::resize), "resize");
					m->add(fun(static_cast<ccMesh*(ccSubMesh::*)()>(&ccSubMesh::getAssociatedMesh)), "getAssociatedMesh");
					m->add(fun(static_cast<const ccMesh*(ccSubMesh::*)()const>(&ccSubMesh::getAssociatedMesh)), "getAssociatedMesh");
					m->add(fun(&ccSubMesh::setAssociatedMesh), "setAssociatedMesh");
					m->add(chaiscript::user_type<ccSubMesh::IndexMap>(), "IndexMap");
					m->add(fun(&ccSubMesh::createNewSubMeshFromSelection), "createNewSubMeshFromSelection");



					m->add(chaiscript::base_class<ccGenericMesh, ccSubMesh>());
					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccSubMesh>());
					m->add(chaiscript::base_class<ccHObject, ccSubMesh>());
					m->add(chaiscript::base_class<ccObject, ccSubMesh>());
					m->add(chaiscript::base_class<ccSerializableObject, ccSubMesh>());
					m->add(chaiscript::base_class<ccDrawableObject, ccSubMesh>());

					return m;
				}


				ModulePtr bs_ccScalarField(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccScalarField>(), "ccScalarField");

					//protected destructor
					//m->add(chaiscript::constructor<ccScalarField(const char*)>(), "ccScalarField");
					//m->add(chaiscript::constructor<ccScalarField(const ccScalarField&)>(), "ccScalarField");

					m->add(chaiscript::user_type<ccScalarField::Range>(), "Range");
					m->add(chaiscript::constructor<ccScalarField::Range()>(), "Range");
					m->add(fun(&ccScalarField::Range::min), "min");
					m->add(fun(&ccScalarField::Range::start), "start");
					m->add(fun(&ccScalarField::Range::stop), "stop");
					m->add(fun(&ccScalarField::Range::max), "max");
					m->add(fun(&ccScalarField::Range::range), "range");
					m->add(fun(&ccScalarField::Range::maxRange), "maxRange");
					m->add(fun(&ccScalarField::Range::setBounds), "setBounds");
					m->add(fun(&ccScalarField::Range::setStart), "setStart");
					m->add(fun(&ccScalarField::Range::setStop), "setStop");
					m->add(fun(&ccScalarField::Range::inbound), "inbound");
					m->add(fun(&ccScalarField::Range::isInbound), "isInbound");
					m->add(fun(&ccScalarField::Range::isInRange), "isInRange");
					m->add(fun(&ccScalarField::displayRange), "displayRange");
					m->add(fun(&ccScalarField::saturationRange), "saturationRange");
					m->add(fun(&ccScalarField::logSaturationRange), "logSaturationRange");
					m->add(fun(&ccScalarField::setMinDisplayed), "setMinDisplayed");
					m->add(fun(&ccScalarField::setMaxDisplayed), "setMaxDisplayed");
					m->add(fun(&ccScalarField::setSaturationStart), "setSaturationStart");
					m->add(fun(&ccScalarField::setSaturationStop), "setSaturationStop");
					m->add(fun(&ccScalarField::getColor), "getColor");
					m->add(fun(&ccScalarField::getValueColor), "getValueColor");
					m->add(fun(&ccScalarField::showNaNValuesInGrey), "showNaNValuesInGrey");
					m->add(fun(&ccScalarField::areNaNValuesShownInGrey), "areNaNValuesShownInGrey");
					m->add(fun(&ccScalarField::alwaysShowZero), "alwaysShowZero");
					m->add(fun(&ccScalarField::isZeroAlwaysShown), "isZeroAlwaysShown");
					m->add(fun(&ccScalarField::setSymmetricalScale), "setSymmetricalScale");
					m->add(fun(&ccScalarField::symmetricalScale), "symmetricalScale");
					m->add(fun(&ccScalarField::setLogScale), "setLogScale");
					m->add(fun(&ccScalarField::logScale), "logScale");
					m->add(fun(&ccScalarField::computeMinAndMax), "computeMinAndMax");
					m->add(fun(&ccScalarField::getColorScale), "getColorScale");
					m->add(fun(&ccScalarField::setColorScale), "setColorScale");
					m->add(fun(&ccScalarField::getColorRampSteps), "getColorRampSteps");
					m->add(fun(&ccScalarField::setColorRampSteps), "setColorRampSteps");
					m->add(chaiscript::user_type<ccScalarField::Histogram>(), "Histogram");
					m->add(fun(&ccScalarField::Histogram::maxValue), "maxValue");
					m->add(chaiscript::base_class<std::vector<unsigned>, ccScalarField::Histogram>());

					m->add(fun(&ccScalarField::getHistogram), "getHistogram");
					m->add(fun(&ccScalarField::mayHaveHiddenValues), "mayHaveHiddenValues");
					m->add(fun(&ccScalarField::setModificationFlag), "setModificationFlag");
					m->add(fun(&ccScalarField::getModificationFlag), "getModificationFlag");
					m->add(fun(&ccScalarField::importParametersFrom), "importParametersFrom");
					m->add(fun(&ccScalarField::isSerializable), "isSerializable");
					m->add(fun(&ccScalarField::toFile), "toFile");
					m->add(fun(&ccScalarField::fromFile), "fromFile");
					m->add(fun(&ccScalarField::getGlobalShift), "getGlobalShift");
					m->add(fun(&ccScalarField::setGlobalShift), "setGlobalShift");

					m->add(chaiscript::base_class<CCLib::ScalarField, ccScalarField>());
					m->add(chaiscript::base_class<ccSerializableObject, ccScalarField>());

					return m;
				}


				ModulePtr bs_ccQuadric(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccQuadric>(), "ccQuadric");
					m->add(chaiscript::constructor<ccQuadric(QString)>(), "ccQuadric");
					m->add(chaiscript::constructor<ccQuadric(CCVector2,	CCVector2, const PointCoordinateType [6],const Tuple3ub*,const ccGLMatrix*,QString,unsigned)>(), "ccQuadric");
					//m->add(fun(&ccQuadric::DEFAULT_DRAWING_PRECISION), "DEFAULT_DRAWING_PRECISION");
					m->add(fun(&ccQuadric::getClassID), "getClassID");
					m->add(fun(&ccQuadric::getTypeName), "getTypeName");
					m->add(fun(&ccQuadric::hasDrawingPrecision), "hasDrawingPrecision");
					m->add(fun(&ccQuadric::clone), "clone");
					m->add(fun(&ccQuadric::getOwnFitBB), "getOwnFitBB");
					m->add(fun(&ccQuadric::getMinCorner), "getMinCorner");
					m->add(fun(&ccQuadric::getMaxCorner), "getMaxCorner");
					m->add(fun(&ccQuadric::getEquationCoefs), "getEquationCoefs");
					m->add(fun(&ccQuadric::getEquationDims), "getEquationDims");
					m->add(fun(&ccQuadric::projectOnQuadric), "projectOnQuadric");
					m->add(fun(&ccQuadric::getEquationString), "getEquationString");
					m->add(fun(&ccQuadric::Fit), "Fit");

					chaiscript::bootstrap::array<PointCoordinateType[6]>("eq_Array", m);
					chaiscript::bootstrap::array<const PointCoordinateType[6]>("eq_Array", m);

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccQuadric>());
					m->add(chaiscript::base_class<ccHObject, ccQuadric>());
					m->add(chaiscript::base_class<ccObject, ccQuadric>());
					m->add(chaiscript::base_class<ccSerializableObject, ccQuadric>());
					m->add(chaiscript::base_class<ccDrawableObject, ccQuadric>());
					m->add(chaiscript::base_class<ccMesh, ccQuadric>());
					m->add(chaiscript::base_class<ccGenericMesh, ccQuadric>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccQuadric>());


					return m;
				}

				ModulePtr bs_ccProgressDialog(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccProgressDialog>(), "ccProgressDialog");
					m->add(chaiscript::constructor<ccProgressDialog(bool, QWidget*)>(), "ccProgressDialog");
					m->add(fun(&ccProgressDialog::update), "update");
					m->add(fun(static_cast<void(ccProgressDialog::*)(const char*)>(&ccProgressDialog::setMethodTitle)), "setMethodTitle");
					m->add(fun(static_cast<void(ccProgressDialog::*)(const char*)>(&ccProgressDialog::setInfo)), "setInfo");
					m->add(fun(&ccProgressDialog::isCancelRequested), "isCancelRequested");
					m->add(fun(&ccProgressDialog::start), "start");
					m->add(fun(&ccProgressDialog::stop), "stop");
					m->add(fun(static_cast<void(ccProgressDialog::*)(QString)>(&ccProgressDialog::setMethodTitle)), "setMethodTitle");
					m->add(fun(static_cast<void(ccProgressDialog::*)(QString)>(&ccProgressDialog::setInfo)), "setInfo");
					m->add(fun(&ccProgressDialog::scheduleRefresh), "scheduleRefresh");
					



					m->add(chaiscript::base_class<CCLib::GenericProgressCallback, ccProgressDialog>());
					m->add(chaiscript::base_class<QProgressDialog, ccProgressDialog>());



					return m;
				}

				ModulePtr bs_ccPointCloudInterpolator(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccPointCloudInterpolator>(), "ccPointCloudInterpolator");
					m->add(chaiscript::user_type<ccPointCloudInterpolator::Parameters>(), "Parameters");
					m->add(fun(&ccPointCloudInterpolator::Parameters::method), "method");
					m->add(fun(&ccPointCloudInterpolator::Parameters::algo), "algo");
					m->add(fun(&ccPointCloudInterpolator::Parameters::knn), "knn");
					m->add(fun(&ccPointCloudInterpolator::Parameters::radius), "radius");
					m->add(fun(&ccPointCloudInterpolator::Parameters::sigma), "sigma");

					m->add(fun(&ccPointCloudInterpolator::InterpolateScalarFieldsFrom), "InterpolateScalarFieldsFrom");
					




					m->add(chaiscript::base_class<CCLib::GenericProgressCallback, ccProgressDialog>());
					m->add(chaiscript::base_class<QProgressDialog, ccProgressDialog>());



					return m;
				}

				ModulePtr bs_ccNormalVectors(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccNormalVectors>(), "ccNormalVectors");
					
					m->add(fun(&ccNormalVectors::GetUniqueInstance), "GetUniqueInstance");
					m->add(fun(&ccNormalVectors::ReleaseUniqueInstance), "ReleaseUniqueInstance");
					m->add(fun(&ccNormalVectors::GetNumberOfVectors), "GetNumberOfVectors");
					m->add(fun(&ccNormalVectors::GetNormal), "GetNormal");
					m->add(fun(&ccNormalVectors::getNormal), "getNormal");
					m->add(fun([](ccNormalVectors* nvs, const PointCoordinateType N[]) {return nvs->GetNormIndex(N); }), "GetNormIndex");
					m->add(fun([](ccNormalVectors* nvs, const CCVector3& N) {return nvs->GetNormIndex(N); }), "GetNormIndex");
					m->add(fun(&ccNormalVectors::ComputeCloudNormals), "ComputeCloudNormals");
					m->add(fun(&ccNormalVectors::GuessNaiveRadius), "GuessNaiveRadius");
					m->add(fun(&ccNormalVectors::GuessBestRadius), "GuessBestRadius");
					m->add(fun(&ccNormalVectors::UpdateNormalOrientations), "UpdateNormalOrientations");
					m->add(fun(&ccNormalVectors::ConvertNormalToStrikeAndDip), "ConvertNormalToStrikeAndDip");
					m->add(fun(&ccNormalVectors::ConvertNormalToDipAndDipDir), "ConvertNormalToDipAndDipDir");
					m->add(fun(&ccNormalVectors::ConvertDipAndDipDirToNormal), "ConvertDipAndDipDirToNormal");
					m->add(fun(&ccNormalVectors::ConvertStrikeAndDipToString), "ConvertStrikeAndDipToString");
					m->add(fun(&ccNormalVectors::ConvertDipAndDipDirToString), "ConvertDipAndDipDirToString");
					m->add(fun(&ccNormalVectors::ConvertNormalToHSV), "ConvertNormalToHSV");
					m->add(fun(&ccNormalVectors::ConvertNormalToRGB), "ConvertNormalToRGB");
					m->add(fun(&ccNormalVectors::enableNormalHSVColorsArray), "enableNormalHSVColorsArray");
					m->add(fun(&ccNormalVectors::getNormalHSVColor), "getNormalHSVColor");
					m->add(fun(&ccNormalVectors::getNormalHSVColorArray), "getNormalHSVColorArray");
					m->add(fun(&ccNormalVectors::ComputeNormalWithLS), "ComputeNormalWithLS");
					m->add(fun(&ccNormalVectors::ComputeNormalWithTri), "ComputeNormalWithTri");
					m->add(fun(&ccNormalVectors::ComputeNormalWithQuadric), "ComputeNormalWithQuadric");

					return m;
				}

				ModulePtr bs_ccNormalCompressor(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccNormalCompressor>(), "ccNormalCompressor");

					m->add(fun(&ccNormalCompressor::Compress), "Compress");
					m->add(fun(&ccNormalCompressor::Decompress), "Decompress");
					m->add(fun(&ccNormalCompressor::InvertNormal), "InvertNormal");
					//m->add(const_var(ccNormalCompressor::QUANTIZE_LEVEL), "QUANTIZE_LEVEL"); // TODO fix these constant vals
					//m->add(const_var(ccNormalCompressor::MAX_VALID_NORM_CODE), "MAX_VALID_NORM_CODE");
					//m->add(const_var(ccNormalCompressor::NULL_NORM_CODE), "NULL_NORM_CODE");
					return m;
				}

				/*
				ModulePtr bs_ccMinimumSpanningTreeForNormsDirection(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccMinimumSpanningTreeForNormsDirection>(), "ccMinimumSpanningTreeForNormsDirection");
					m->add(fun(&ccMinimumSpanningTreeForNormsDirection::OrientNormals), "OrientNormals");
					return m;
				}

				ModulePtr bs_ccFastMarchingForNormsDirection(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccFastMarchingForNormsDirection>(), "ccFastMarchingForNormsDirection");
					m->add(chaiscript::constructor<ccFastMarchingForNormsDirection()>(), "ccFastMarchingForNormsDirection");
					m->add(fun(&ccFastMarchingForNormsDirection::OrientNormals), "OrientNormals");
					m->add(fun(&ccFastMarchingForNormsDirection::init), "init");
					m->add(fun(&ccFastMarchingForNormsDirection::updateResolvedTable), "updateResolvedTable");
					m->add(fun(&ccFastMarchingForNormsDirection::propagate), "propagate");
					
					m->add(chaiscript::base_class<CCLib::FastMarching, ccFastMarchingForNormsDirection>());

					return m;
				}*/

				ModulePtr bs_ccMaterialSet(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccMaterialSet>(), "ccMaterialSet");
					m->add(fun(&ccMaterialSet::getClassID), "getClassID");
					m->add(fun(&ccMaterialSet::isShareable), "isShareable");
					m->add(fun(&ccMaterialSet::findMaterialByName), "findMaterialByName");
					m->add(fun(&ccMaterialSet::findMaterialByUniqueID), "findMaterialByUniqueID");
					m->add(fun(&ccMaterialSet::addMaterial), "addMaterial");
					m->add(fun(&ccMaterialSet::ParseMTL), "ParseMTL");
					m->add(fun(&ccMaterialSet::saveAsMTL), "saveAsMTL");
					m->add(fun(&ccMaterialSet::clone), "clone");
					m->add(fun(&ccMaterialSet::append), "append");
					m->add(fun(&ccMaterialSet::isSerializable), "isSerializable");

					m->add(chaiscript::vector_conversion<std::vector<ccMaterial::CShared>>());

					m->add(chaiscript::base_class<std::vector<ccMaterial::CShared>, ccMaterialSet>());
					m->add(chaiscript::base_class<ccHObject, ccMaterialSet>());
					m->add(chaiscript::base_class<ccObject, ccMaterialSet>());
					m->add(chaiscript::base_class<ccSerializableObject, ccMaterialSet>());
					m->add(chaiscript::base_class<ccDrawableObject, ccMaterialSet>());
					m->add(chaiscript::base_class<CCShareable, ccMaterialSet>());


					return m;
				}

				ModulePtr bs_ccMaterial(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccMaterial>(), "ccMaterial");
					m->add(chaiscript::constructor<ccMaterial(QString)>(), "ccMaterial");
					m->add(chaiscript::constructor<ccMaterial(const ccMaterial&)>(), "ccMaterial");
					m->add(chaiscript::user_type<ccMaterial::CShared>(), "CShared");
					m->add(chaiscript::user_type<ccMaterial::Shared>(), "Shared");

					m->add(fun(&ccMaterial::getName), "getName");
					m->add(fun(&ccMaterial::getTextureFilename), "getTextureFilename");
					m->add(fun(&ccMaterial::setName), "setName");
					m->add(fun(&ccMaterial::setDiffuse), "setDiffuse");
					m->add(fun(&ccMaterial::setDiffuseFront), "setDiffuseFront");
					m->add(fun(&ccMaterial::setDiffuseBack), "setDiffuseBack");
					m->add(fun(&ccMaterial::getDiffuseFront), "getDiffuseFront");
					m->add(fun(&ccMaterial::getDiffuseBack), "getDiffuseBack");
					m->add(fun(&ccMaterial::setAmbient), "setAmbient");
					m->add(fun(&ccMaterial::getAmbient), "getAmbient");
					m->add(fun(&ccMaterial::setSpecular), "setSpecular");
					m->add(fun(&ccMaterial::getSpecular), "getSpecular");
					m->add(fun(&ccMaterial::setEmission), "setEmission");
					m->add(fun(&ccMaterial::getEmission), "getEmission");
					m->add(fun(&ccMaterial::setShininess), "setShininess");
					m->add(fun(&ccMaterial::setShininessFront), "setShininessFront");
					m->add(fun(&ccMaterial::setShininessBack), "setShininessBack");
					m->add(fun(&ccMaterial::getShininessFront), "getShininessFront");
					m->add(fun(&ccMaterial::getShininessBack), "getShininessBack");
					m->add(fun(&ccMaterial::setTransparency), "setTransparency");
					m->add(fun(&ccMaterial::applyGL), "applyGL");
					m->add(fun(&ccMaterial::hasTexture), "hasTexture");
					m->add(fun(&ccMaterial::setTexture), "setTexture");
					m->add(fun(&ccMaterial::loadAndSetTexture), "loadAndSetTexture");
					m->add(fun(&ccMaterial::getTexture), "getTexture");
					m->add(fun(&ccMaterial::getTextureID), "getTextureID");
					m->add(fun(&ccMaterial::MakeLightsNeutral), "MakeLightsNeutral");
					m->add(fun(&ccMaterial::GetTexture), "GetTexture");
					m->add(fun(&ccMaterial::AddTexture), "AddTexture");
					m->add(fun(&ccMaterial::ReleaseTextures), "ReleaseTextures");
					m->add(fun(&ccMaterial::releaseTexture), "releaseTexture");
					m->add(fun(&ccMaterial::compare), "compare");
					m->add(fun(&ccMaterial::isSerializable), "isSerializable");
					m->add(fun(&ccMaterial::toFile), "toFile");
					m->add(fun(&ccMaterial::fromFile), "fromFile");
					m->add(fun(&ccMaterial::getUniqueIdentifier), "getUniqueIdentifier");
					m->add(fun(&ccMaterial::setTextureMinMagFilters), "setTextureMinMagFilters");


					m->add(chaiscript::base_class<ccSerializableObject, ccMaterial>());



					return m;
				}


				ModulePtr bs_ccLog(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccLog>(), "ccLog");					
					m->add(fun(&ccLog::TheInstance), "TheInstance");
					m->add(fun(&ccLog::RegisterInstance), "RegisterInstance");
					m->add(fun(&ccLog::EnableMessageBackup), "EnableMessageBackup");
					m->add(fun(&ccLog::LogMessage), "LogMessage");
					m->add(fun(&ccLog::Print), "Print");
					m->add(fun(&ccLog::PrintDebug), "PrintDebug");
					m->add(fun(&ccLog::Warning), "Warning");
					m->add(fun(&ccLog::WarningDebug), "WarningDebug");
					m->add(fun(&ccLog::Error), "Error");
					m->add(fun(&ccLog::ErrorDebug), "ErrorDebug");
					return m;
				}

				ModulePtr bs_ccIndexedTransformation(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccIndexedTransformation>(), "ccIndexedTransformation");
					m->add(chaiscript::constructor<ccIndexedTransformation()>(), "ccIndexedTransformation");
					m->add(chaiscript::constructor<ccIndexedTransformation(const ccGLMatrix&)>(), "ccIndexedTransformation");
					m->add(chaiscript::constructor<ccIndexedTransformation(const ccGLMatrix&, double)>(), "ccIndexedTransformation");
					m->add(chaiscript::constructor<ccIndexedTransformation(const ccIndexedTransformation&)>(), "ccIndexedTransformation");
					m->add(fun(&ccIndexedTransformation::getIndex), "getIndex");
					m->add(fun(&ccIndexedTransformation::setIndex), "setIndex");
					m->add(fun(&ccIndexedTransformation::Interpolate), "Interpolate");
					m->add(fun(&ccIndexedTransformation::operator*), "*");
					m->add(fun(&ccIndexedTransformation::operator*=), "*=");
					m->add(fun(&ccIndexedTransformation::operator+=), "+=");
					m->add(fun(&ccIndexedTransformation::operator-=), "-=");
					m->add(fun(&ccIndexedTransformation::transposed), "transposed");
					m->add(fun(&ccIndexedTransformation::inverse), "inverse");
					m->add(fun(&ccIndexedTransformation::toAsciiFile), "toAsciiFile");
					m->add(fun(&ccIndexedTransformation::fromAsciiFile), "fromAsciiFile");
					m->add(fun(&ccIndexedTransformation::isSerializable), "isSerializable");
					m->add(fun(&ccIndexedTransformation::toFile), "toFile");
					m->add(fun(&ccIndexedTransformation::fromFile), "fromFile");


					m->add(chaiscript::base_class<ccGLMatrixTpl<float>, ccIndexedTransformation>());
					m->add(chaiscript::base_class<ccGLMatrix, ccIndexedTransformation>());

					return m;
				}

				ModulePtr bs_ccIndexedTransformationBuffer(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccIndexedTransformationBuffer>(), "ccIndexedTransformationBuffer");
					m->add(chaiscript::constructor<ccIndexedTransformationBuffer(QString)>(), "ccIndexedTransformationBuffer");
					m->add(chaiscript::constructor<ccIndexedTransformationBuffer(const ccIndexedTransformationBuffer&)>(), "ccIndexedTransformationBuffer");
					m->add(fun(&ccIndexedTransformationBuffer::getClassID), "getClassID");
					m->add(fun(&ccIndexedTransformationBuffer::isSerializable), "isSerializable");
					m->add(fun(&ccIndexedTransformationBuffer::sort), "sort");
					m->add(fun(&ccIndexedTransformationBuffer::findNearest), "findNearest");
					m->add(fun(&ccIndexedTransformationBuffer::getInterpolatedTransformation), "getInterpolatedTransformation");
					m->add(fun(&ccIndexedTransformationBuffer::triherdonsShown), "triherdonsShown");
					m->add(fun(&ccIndexedTransformationBuffer::showTriherdons), "showTriherdons");
					m->add(fun(&ccIndexedTransformationBuffer::triherdonsDisplayScale), "triherdonsDisplayScale");
					m->add(fun(&ccIndexedTransformationBuffer::setTriherdonsDisplayScale), "setTriherdonsDisplayScale");
					m->add(fun(&ccIndexedTransformationBuffer::isPathShownAsPolyline), "isPathShownAsPolyline");
					m->add(fun(&ccIndexedTransformationBuffer::showPathAsPolyline), "showPathAsPolyline");
					m->add(fun(&ccIndexedTransformationBuffer::invalidateBoundingBox), "invalidateBoundingBox");
					m->add(fun(&ccIndexedTransformationBuffer::getOwnBB), "getOwnBB");
					m->add(chaiscript::vector_conversion<std::vector<ccIndexedTransformation>>());


					m->add(chaiscript::base_class<std::vector<ccIndexedTransformation>, ccIndexedTransformationBuffer>());
					m->add(chaiscript::base_class<ccHObject, ccIndexedTransformationBuffer>());
					m->add(chaiscript::base_class<ccObject, ccIndexedTransformationBuffer>());
					m->add(chaiscript::base_class<ccSerializableObject, ccIndexedTransformationBuffer>());
					m->add(chaiscript::base_class<ccDrawableObject, ccIndexedTransformationBuffer>());

					return m;
				}

				ModulePtr bs_ccGL(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGL>(), "ccGL");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, const float*)>(&ccGL::Vertex3v)), "Vertex3v");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, const double*)>(&ccGL::Vertex3v)), "Vertex3v");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, float, float, float)>(&ccGL::Vertex3)), "Vertex3");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, double, double, double)>(&ccGL::Vertex3)), "Vertex3");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, float, float, float)>(&ccGL::Scale)), "Scale");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, double, double, double)>(&ccGL::Scale)), "Scale");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, const float*)>(&ccGL::Normal3v)), "Normal3v");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, const double*)>(&ccGL::Normal3v)), "Normal3v");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, float, float, float, float)>(&ccGL::Rotate)), "Rotate");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, double, double, double, double)>(&ccGL::Rotate)), "Rotate");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, float, float, float)>(&ccGL::Translate)), "Translate");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, double, double, double)>(&ccGL::Translate)), "Translate");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, const unsigned char*)>(&ccGL::Color3v)), "Color3v");
					m->add(fun(static_cast<void (*)(QOpenGLFunctions_2_1*, const float*)>(&ccGL::Color3v)), "Color3v");
					m->add(fun(&ccGL::Frustum), "Frustum");
					m->add(fun(&ccGL::Perspective), "Perspective");
					m->add(fun(&ccGL::Ortho), "Ortho");
					m->add(fun(&ccGL::Project<float, float>), "Project");
					m->add(fun(&ccGL::Project<float, double>), "Project");
					m->add(fun(&ccGL::Project<double, double>), "Project");
					m->add(fun(&ccGL::Project<double, float>), "Project");
					m->add(fun(static_cast<double(*)(const double*,int,int)>(&ccGL::MAT)), "MAT");
					m->add(fun(static_cast<float(*)(const float*,int,int)>(&ccGL::MAT)), "MAT");
					m->add(fun(static_cast<double&(*)(double*, int, int) > (&ccGL::MAT)), "MAT");
					m->add(fun(static_cast<float&(*)(float*,int,int)>(&ccGL::MAT)), "MAT");
					m->add(fun(&ccGL::InvertMatrix<float>), "InvertMatrix");
					m->add(fun(&ccGL::InvertMatrix<double>), "InvertMatrix");
					m->add(fun(&ccGL::Unproject<float, float>), "Unproject");
					m->add(fun(&ccGL::Unproject<float, double>), "Unproject");
					m->add(fun(&ccGL::Unproject<double, double>), "Unproject");
					m->add(fun(&ccGL::Unproject<double, float>), "Unproject");
					m->add(fun(&ccGL::PickMatrix), "PickMatrix");

					return m;
				}

				ModulePtr bs_ccGriddedTools(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccGriddedTools>(), "ccGriddedTools");
					m->add(chaiscript::user_type<ccGriddedTools::GridParameters>(), "GridParameters");
					m->add(chaiscript::constructor<ccGriddedTools::GridParameters()>(), "GridParameters");
					m->add(fun(&ccGriddedTools::GridParameters::minPhi), "minPhi");
					m->add(fun(&ccGriddedTools::GridParameters::maxPhi), "maxPhi");
					m->add(fun(&ccGriddedTools::GridParameters::minTheta), "minTheta");
					m->add(fun(&ccGriddedTools::GridParameters::maxTheta), "maxTheta");
					m->add(fun(&ccGriddedTools::GridParameters::deltaPhiRad), "deltaPhiRad");
					m->add(fun(&ccGriddedTools::GridParameters::deltaThetaRad), "deltaThetaRad");
					m->add(fun(&ccGriddedTools::GridParameters::maxRange), "maxRange");
					m->add(fun(&ccGriddedTools::DetectParameters), "DetectParameters");
					m->add(fun(&ccGriddedTools::ComputeBestSensor), "ComputeBestSensor");
					

					return m;
				}


				ModulePtr bs_ccViewportParameters(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccViewportParameters>(), "ccViewportParameters");
					m->add(constructor<ccViewportParameters()>(), "ccViewportParameters");
					m->add(constructor<ccViewportParameters(const ccViewportParameters&)>(), "ccViewportParameters");
					m->add(fun(&ccViewportParameters::isSerializable), "isSerializable");
					m->add(fun(&ccViewportParameters::toFile), "toFile");
					m->add(fun(&ccViewportParameters::fromFile), "fromFile");
					m->add(fun(&ccViewportParameters::pixelSize), "pixelSize");
					m->add(fun(&ccViewportParameters::zoom), "zoom");
					m->add(fun(&ccViewportParameters::viewMat), "viewMat");
					m->add(fun(&ccViewportParameters::defaultPointSize), "defaultPointSize");
					m->add(fun(&ccViewportParameters::defaultLineWidth), "defaultLineWidth");
					m->add(fun(&ccViewportParameters::perspectiveView), "perspectiveView");
					m->add(fun(&ccViewportParameters::objectCenteredView), "objectCenteredView");
					m->add(fun(&ccViewportParameters::zNearCoef), "zNearCoef");
					m->add(fun(&ccViewportParameters::zNear), "zNear");
					m->add(fun(&ccViewportParameters::zFar), "zFar");
					m->add(fun(&ccViewportParameters::pivotPoint), "pivotPoint");
					m->add(fun(&ccViewportParameters::cameraCenter), "cameraCenter");
					m->add(fun(&ccViewportParameters::fov), "fov");
					m->add(fun(&ccViewportParameters::perspectiveAspectRatio), "perspectiveAspectRatio");
					m->add(fun(&ccViewportParameters::orthoAspectRatio), "orthoAspectRatio");
					m->add(fun(&ccViewportParameters::IncrementToZNearCoef), "IncrementToZNearCoef");
					m->add(fun(&ccViewportParameters::ZNearCoefToIncrement), "ZNearCoefToIncrement");

					m->add(chaiscript::base_class<ccSerializableObject, ccViewportParameters>());

					return m;
				}

				ModulePtr bs_ccGLCameraParameters(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccGLCameraParameters>(), "ccGLCameraParameters");
					m->add(constructor<ccGLCameraParameters()>(), "ccGLCameraParameters");
					m->add(fun(static_cast<bool(ccGLCameraParameters::*)(const CCVector3d&, CCVector3d&, bool)const>(&ccGLCameraParameters::project)), "project");
					m->add(fun(static_cast<bool(ccGLCameraParameters::*)(const CCVector3&, CCVector3d&, bool)const>(&ccGLCameraParameters::project)), "project");
					m->add(fun(static_cast<bool(ccGLCameraParameters::*)(const CCVector3d&, CCVector3d&)const>(&ccGLCameraParameters::unproject)), "unproject");
					m->add(fun(static_cast<bool(ccGLCameraParameters::*)(const CCVector3&, CCVector3d&)const>(&ccGLCameraParameters::unproject)), "unproject");
					m->add(fun(&ccGLCameraParameters::modelViewMat), "modelViewMat");
					m->add(fun(&ccGLCameraParameters::projectionMat), "projectionMat");
					m->add(fun(&ccGLCameraParameters::viewport), "viewport");
					m->add(fun(&ccGLCameraParameters::perspective), "perspective");
					m->add(fun(&ccGLCameraParameters::fov_deg), "fov_deg");
					m->add(fun(&ccGLCameraParameters::pixelSize), "pixelSize");
					return m;
				}

				ModulePtr bs_ccGenericGLDisplay(ModulePtr m = std::make_shared<Module>())
				{
					m->add(user_type<ccGenericGLDisplay>(), "ccGenericGLDisplay");

					m->add(fun(&ccGenericGLDisplay::getScreenSize), "getScreenSize");
					m->add(fun(&ccGenericGLDisplay::redraw), "redraw");
					m->add(fun(&ccGenericGLDisplay::toBeRefreshed), "toBeRefreshed");
					m->add(fun(&ccGenericGLDisplay::refresh), "refresh");
					m->add(fun(&ccGenericGLDisplay::invalidateViewport), "invalidateViewport");
					m->add(fun(&ccGenericGLDisplay::deprecate3DLayer), "deprecate3DLayer");
					m->add(fun(&ccGenericGLDisplay::getTextDisplayFont), "getTextDisplayFont");
					m->add(fun(&ccGenericGLDisplay::getLabelDisplayFont), "getLabelDisplayFont");
					m->add(fun(&ccGenericGLDisplay::displayText), "displayText");
					m->add(fun([](ccGenericGLDisplay* obj, QString a, int b, int c, unsigned char d, float e, const ccColor::Rgba* f) {obj->displayText(a, b, c, d, e, f); }), "displayText");
					m->add(fun([](ccGenericGLDisplay* obj, QString a, int b, int c, unsigned char d, float e) {obj->displayText(a, b, c, d, e); }), "displayText");
					m->add(fun([](ccGenericGLDisplay* obj, QString a, int b, int c, unsigned char d) {obj->displayText(a, b, c, d); }), "displayText");
					m->add(fun([](ccGenericGLDisplay* obj, QString a, int b, int c) {obj->displayText(a, b, c); }), "displayText");
					m->add(fun(&ccGenericGLDisplay::display3DLabel), "display3DLabel");
					m->add(fun([](ccGenericGLDisplay* obj, const QString& a, const CCVector3& b, const ccColor::Rgba* c) {obj->display3DLabel(a, b, c); }), "display3DLabel");
					m->add(fun([](ccGenericGLDisplay* obj, const QString& a, const CCVector3& b) {obj->display3DLabel(a, b); }), "display3DLabel");
					m->add(fun(&ccGenericGLDisplay::getGLCameraParameters), "getGLCameraParameters");
					m->add(fun(&ccGenericGLDisplay::toCenteredGLCoordinates), "toCenteredGLCoordinates");
					m->add(fun(&ccGenericGLDisplay::toCornerGLCoordinates), "toCornerGLCoordinates");
					m->add(fun(&ccGenericGLDisplay::getViewportParameters), "getViewportParameters");
					m->add(fun(&ccGenericGLDisplay::setupProjectiveViewport), "setupProjectiveViewport");
					m->add(fun([](ccGenericGLDisplay* obj, const ccGLMatrixd& a, float b, float c, bool d) {obj->setupProjectiveViewport(a, b, c, d); }), "setupProjectiveViewport");
					m->add(fun([](ccGenericGLDisplay* obj, const ccGLMatrixd& a, float b, float c) {obj->setupProjectiveViewport(a, b, c); }), "setupProjectiveViewport");
					m->add(fun([](ccGenericGLDisplay* obj, const ccGLMatrixd& a, float b) {obj->setupProjectiveViewport(a, b); }), "setupProjectiveViewport");
					m->add(fun([](ccGenericGLDisplay* obj, const ccGLMatrixd& a) {obj->setupProjectiveViewport(a); }), "setupProjectiveViewport");
					m->add(fun(&ccGenericGLDisplay::asWidget), "asWidget");

					return m;
				}


				ModulePtr bs_ccFlags(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccFlags>(), "ccFlags");

					m->add(fun(&ccFlags::reset), "reset");
					m->add(fun(&ccFlags::fromByte), "fromByte");
					m->add(fun(&ccFlags::toByte), "toByte");
					m->add(fun(&ccFlags::table), "table");
				
					chaiscript::bootstrap::array<bool[8]>("bool_table_Array", m);


					return m;
				}

				ModulePtr bs_ccFileUtils(ModulePtr m = std::make_shared<Module>())
				{
					m->add(fun(&ccFileUtils::defaultDocPath), "defaultDocPath");
					return m;
				}

				ModulePtr bs_ccDepthBuffer(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccDepthBuffer>(), "ccDepthBuffer");
					m->add(chaiscript::constructor<ccDepthBuffer()>(), "ccDepthBuffer");
					
					m->add(fun(&ccDepthBuffer::zBuff), "zBuff");
					m->add(fun(&ccDepthBuffer::deltaPhi), "deltaPhi");
					m->add(fun(&ccDepthBuffer::deltaTheta), "deltaTheta");
					m->add(fun(&ccDepthBuffer::width), "width");
					m->add(fun(&ccDepthBuffer::height), "height");
					m->add(fun(&ccDepthBuffer::clear), "clear");
					m->add(fun(&ccDepthBuffer::fillHoles), "fillHoles");
										
					return m;
				}

				

				template<typename T>
				ModulePtr bs_RgbTpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
				{
					chaiscript::utility::add_class<ccColor::RgbTpl<T>>(*m,
						shortCutName,
						{
							chaiscript::constructor<ccColor::RgbTpl<T>()>(),
							chaiscript::constructor<ccColor::RgbTpl<T>(T,T,T)>(),
							chaiscript::constructor<ccColor::RgbTpl<T>(const T c[3])>(),
						},
					{
						{ fun(&ccColor::RgbTpl<T>::r), "r" },
						{ fun(&ccColor::RgbTpl<T>::g), "g" },
						{ fun(&ccColor::RgbTpl<T>::b), "b" },
						{ fun(&ccColor::RgbTpl<T>::rgb), "rgb" },
						{ fun(&ccColor::RgbTpl<T>::operator!=), "!=" },
					}
					);
					chaiscript::bootstrap::array<T[3]>("rgb_Array", m);
				
					return m;
				}

				template<typename T>
				ModulePtr bs_RgbaTpl(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
				{
					chaiscript::utility::add_class<ccColor::RgbaTpl<T>>(*m,
						shortCutName,
						{
							chaiscript::constructor<ccColor::RgbaTpl<T>()>(),
							chaiscript::constructor<ccColor::RgbaTpl<T>(T,T,T,T)>(),
							chaiscript::constructor<ccColor::RgbaTpl<T>(const T c[4])>(),
							chaiscript::constructor<ccColor::RgbaTpl<T>(const T c[3])>(),
							chaiscript::constructor<ccColor::RgbaTpl<T>(const ccColor::RgbTpl<T>, T)>(),
						},
					{
						{ fun(&ccColor::RgbaTpl<T>::r), "r" },
						{ fun(&ccColor::RgbaTpl<T>::g), "g" },
						{ fun(&ccColor::RgbaTpl<T>::b), "b" },
						{ fun(&ccColor::RgbaTpl<T>::a), "a" },
						{ fun(&ccColor::RgbaTpl<T>::rgba), "rgba" },
						{ fun(&ccColor::RgbaTpl<T>::operator!=), "!=" },
					}
					);
					chaiscript::bootstrap::array<T[4]>("rgba_Array", m);
					m->add(chaiscript::type_conversion<ccColor::RgbaTpl<T>, ccColor::RgbTpl<T>>());
					return m;
				}

				ModulePtr bs_ccColor(ModulePtr m = std::make_shared<Module>())
				{
					bs_RgbTpl<float>("Rgbf", m);
					bs_RgbTpl<unsigned char>("Rgbub", m);
					bs_RgbTpl<ColorCompType>("Rgb", m);

					bs_RgbaTpl<float>("Rgbaf", m);
					bs_RgbaTpl<unsigned char>("Rgbaub", m);
					bs_RgbaTpl<ColorCompType>("Rgba", m);

					m->add(chaiscript::user_type<ccColor::Generator>(), "Generator");
					m->add(fun(&ccColor::Generator::Random), "Random");

					//m->add(chaiscript::user_type<ccColor::Convert>(), "Convert");
					m->add(fun(&ccColor::Convert::hsl2rgb), "hsl2rgb");
					m->add(fun(&ccColor::Convert::hsv2rgb), "hsv2rgb");
					m->add(fun(&ccColor::FromRgbfToRgb), "FromRgbfToRgb");
					m->add(fun(&ccColor::FromRgbafToRgb), "FromRgbafToRgb");
					m->add(fun(&ccColor::FromRgbafToRgba), "FromRgbafToRgba");
					m->add(fun(&ccColor::FromQRgb), "FromQRgb");
					m->add(fun(&ccColor::FromQRgba), "FromQRgba");
					m->add(fun(&ccColor::FromQColor), "FromQColor");
					m->add(fun(&ccColor::FromQColora), "FromQColora");
					m->add(fun(&ccColor::FromQColorf), "FromQColorf");
					m->add(fun(&ccColor::FromQColoraf), "FromQColoraf");
					return m;


				}




				ModulePtr bs_ccColorScalesManager(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccColorScalesManager>(), "ccColorScalesManager");


					m->add(fun(&ccColorScalesManager::GetUniqueInstance), "GetUniqueInstance");
					m->add(fun(&ccColorScalesManager::ReleaseUniqueInstance), "ReleaseUniqueInstance");
					m->add(fun(&ccColorScalesManager::GetDefaultScaleUUID), "GetDefaultScaleUUID");
					m->add(fun(&ccColorScalesManager::GetDefaultScale), "GetDefaultScale");
					m->add(fun(&ccColorScalesManager::getDefaultScale), "getDefaultScale");
					m->add(fun(&ccColorScalesManager::getScale), "getScale");
					m->add(fun(&ccColorScalesManager::addScale), "addScale");
					m->add(fun(&ccColorScalesManager::removeScale), "removeScale");
					m->add(chaiscript::user_type<ccColorScalesManager::ScalesMap>(), "ScalesMap");
					m->add(fun(static_cast<ccColorScalesManager::ScalesMap&(ccColorScalesManager::*)()>(&ccColorScalesManager::map)), "map");
					m->add(fun(static_cast<const ccColorScalesManager::ScalesMap&(ccColorScalesManager::*)()const>(&ccColorScalesManager::map)), "map");
					m->add(fun(&ccColorScalesManager::fromPersistentSettings), "fromPersistentSettings");
					m->add(fun(&ccColorScalesManager::toPersistentSettings), "toPersistentSettings");
					return m;
				}

				ModulePtr bs_ccColorScaleElement(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccColorScaleElement>(), "ccColorScaleElement");
					m->add(chaiscript::constructor<ccColorScaleElement()>(), "ccColorScaleElement");
					m->add(chaiscript::constructor<ccColorScaleElement(double,const QColor&)>(), "ccColorScaleElement");

					m->add(fun(&ccColorScaleElement::setRelativePos), "setRelativePos");
					m->add(fun(&ccColorScaleElement::getRelativePos), "getRelativePos");
					m->add(fun(&ccColorScaleElement::setColor), "setColor");
					m->add(fun(&ccColorScaleElement::getColor), "getColor");
					m->add(fun(&ccColorScaleElement::IsSmaller), "IsSmaller");
					return m;
				}

				ModulePtr bs_ccColorScale(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccColorScale>(), "ccColorScale");
					m->add(chaiscript::user_type<ccColorScale::Shared>(), "Shared");
					m->add(chaiscript::constructor<ccColorScale(QString, const QString&)>(), "ccColorScale");

					m->add(fun(&ccColorScale::Create), "Create");
					m->add(fun(&ccColorScale::getName), "getName");
					m->add(fun(&ccColorScale::setName), "setName");
					m->add(fun(&ccColorScale::getUuid), "getUuid");
					m->add(fun(&ccColorScale::setUuid), "setUuid");
					m->add(fun(&ccColorScale::generateNewUuid), "generateNewUuid");
					m->add(fun(&ccColorScale::isRelative), "isRelative");
					m->add(fun(&ccColorScale::setRelative), "setRelative");
					m->add(fun(&ccColorScale::setAbsolute), "setAbsolute");
					m->add(fun(&ccColorScale::getAbsoluteBoundaries), "getAbsoluteBoundaries");
					m->add(fun(&ccColorScale::isLocked), "isLocked");
					m->add(fun(&ccColorScale::setLocked), "setLocked");
					m->add(chaiscript::user_type<ccColorScale::LabelSet>(), "LabelSet");
					m->add(fun(static_cast<ccColorScale::LabelSet&(ccColorScale::*)()>(&ccColorScale::customLabels)), "customLabels");
					m->add(fun(static_cast<const ccColorScale::LabelSet & (ccColorScale::*)()const>(&ccColorScale::customLabels)), "customLabels");
					m->add(fun(&ccColorScale::setCustomLabels), "setCustomLabels");
					m->add(fun(&ccColorScale::stepCount), "stepCount");
					m->add(fun(static_cast<ccColorScaleElement& (ccColorScale::*)(int)>(&ccColorScale::step)), "step");
					m->add(fun(static_cast<const ccColorScaleElement& (ccColorScale::*)(int)const>(&ccColorScale::step)), "step");
					m->add(fun(&ccColorScale::insert), "insert");
					m->add(fun(&ccColorScale::remove), "remove");
					m->add(fun(&ccColorScale::clear), "clear");
					m->add(fun(&ccColorScale::update), "update");
					m->add(fun(&ccColorScale::getRelativePosition), "getRelativePosition");
					m->add(fun(&ccColorScale::getColorByValue), "getColorByValue");
					m->add(fun(static_cast<const ccColor::Rgb*(ccColorScale::*)(double, const ccColor::Rgb*)const>(&ccColorScale::getColorByRelativePos)), "getColorByRelativePos");
					m->add(fun(static_cast<const ccColor::Rgb*(ccColorScale::*)(double, unsigned, const ccColor::Rgb*)const>(&ccColorScale::getColorByRelativePos)), "getColorByRelativePos");
					m->add(fun(&ccColorScale::getColorByIndex), "getColorByIndex");
					m->add(fun(&ccColorScale::saveAsXML), "saveAsXML");
					m->add(fun(&ccColorScale::LoadFromXML), "LoadFromXML");
					m->add(fun(&ccColorScale::isSerializable), "isSerializable");
					m->add(fun(&ccColorScale::toFile), "toFile");
					m->add(fun(&ccColorScale::fromFile), "fromFile");

					m->add(chaiscript::base_class<ccSerializableObject, ccColorScale>());


					return m;
				}

				ModulePtr bs_ccClipBox(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccClipBox>(), "ccClipBox");
					m->add(chaiscript::constructor<ccClipBox(QString)>(), "ccClipBox");

					m->add(fun(&ccClipBox::addAssociatedEntity), "addAssociatedEntity");
					m->add(fun(&ccClipBox::releaseAssociatedEntities), "releaseAssociatedEntities");
					m->add(fun(&ccClipBox::getOwnBB), "getOwnBB");
					m->add(fun(&ccClipBox::move2D), "move2D");
					m->add(fun(&ccClipBox::move3D), "move3D");
					m->add(fun(&ccClipBox::setClickedPoint), "setClickedPoint");
					m->add(fun(&ccClipBox::setActiveComponent), "setActiveComponent");
					m->add(fun(&ccClipBox::getClassID), "getClassID");
					m->add(fun(&ccClipBox::getBox), "getBox");
					m->add(fun(&ccClipBox::showBox), "showBox");
					m->add(fun(&ccClipBox::setBox), "setBox");
					m->add(fun(&ccClipBox::shift), "shift");
					m->add(fun(&ccClipBox::flagPointsInside), "flagPointsInside");
					m->add(fun(&ccClipBox::reset), "reset");
					m->add(fun(&ccClipBox::set), "set");
					m->add(fun(&ccClipBox::get), "get");
					m->add(fun(&ccClipBox::getContainer), "getContainer");
					m->add(fun(&ccClipBox::boxModified), "boxModified");


					m->add(chaiscript::base_class<ccHObject, ccClipBox>());
					m->add(chaiscript::base_class<ccInteractor, ccClipBox>());
					m->add(chaiscript::base_class<ccObject, ccClipBox>());
					m->add(chaiscript::base_class<ccDrawableObject, ccClipBox>());
					m->add(chaiscript::base_class<ccSerializableObject, ccClipBox>());


					return m;
				}

				

				template<typename T>
				ModulePtr bs_ccSingleton(const std::string& shortCutName, ModulePtr m = std::make_shared<Module>())
				{

					m->add(chaiscript::user_type<ccSingleton<T>>(), shortCutName);
					m->add(chaiscript::constructor<ccSingleton<T>()>(), shortCutName);
					m->add(fun(&ccSingleton<T>::release), "release");
					m->add(fun(&ccSingleton<T>::instance), "instance");
					return m;
				}




				ModulePtr bs_ccHObjectCaster(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccHObjectCaster>(), "ccHObjectCaster");
					//m->add(fun(&ccHObjectCaster::ToPointCloud), "ToPointCloud");
					////TO DO decide on style for the ccHObjectCaster functions, chaiscript is not evaluating to nullptr
					//// even when conversion fails a new instance of the type is generated rather than being set to null
					//// I found that if I make the calls lambdas with an extra bool ptr to set based on success will work
					//// but it feels a bit like a hack
					m->add(fun([](ccHObject* mat, bool* l, bool* s)->ccPointCloud* {ccPointCloud* pc = ccHObjectCaster::ToPointCloud(mat, l); *s = pc != nullptr; return pc; }), "ToPointCloud");
					m->add(fun([](ccHObject* mat, bool* s)->ccPointCloud* {ccPointCloud* pc = ccHObjectCaster::ToPointCloud(mat); *s = pc != nullptr; return pc; }), "ToPointCloud");
					m->add(fun(&ccHObjectCaster::ToGenericPointCloud), "ToGenericPointCloud");
					m->add(fun([](ccHObject* mat) {return ccHObjectCaster::ToGenericPointCloud(mat); }), "ToGenericPointCloud");
					m->add(fun(&ccHObjectCaster::ToShifted), "ToShifted");
					m->add(fun([](ccHObject *mat) {return ccHObjectCaster::ToShifted(mat); }), "ToShifted");
					m->add(fun(&ccHObjectCaster::ToGenericMesh), "ToGenericMesh");
					m->add(fun(&ccHObjectCaster::ToMesh), "ToMesh");
					m->add(fun(&ccHObjectCaster::ToSubMesh), "ToSubMesh");
					m->add(fun(&ccHObjectCaster::ToPolyline), "ToPolyline");
					m->add(fun(&ccHObjectCaster::ToPlanarEntity), "ToPlanarEntity");
					m->add(fun(&ccHObjectCaster::ToPrimitive), "ToPrimitive");
					m->add(fun(&ccHObjectCaster::ToSphere), "ToSphere");
					m->add(fun(&ccHObjectCaster::ToCylinder), "ToCylinder");
					m->add(fun(&ccHObjectCaster::ToCone), "ToCone");
					m->add(fun(&ccHObjectCaster::ToPlane), "ToPlane");
					m->add(fun(&ccHObjectCaster::ToDish), "ToDish");
					m->add(fun(&ccHObjectCaster::ToExtru), "ToExtru");
					m->add(fun(&ccHObjectCaster::ToTorus), "ToTorus");
					m->add(fun(&ccHObjectCaster::ToOctreeProxy), "ToOctreeProxy");
					m->add(fun(&ccHObjectCaster::ToOctree), "ToOctree");
					m->add(fun(&ccHObjectCaster::ToKdTree), "ToKdTree");
					m->add(fun(&ccHObjectCaster::ToSensor), "ToSensor");
					m->add(fun(&ccHObjectCaster::ToGBLSensor), "ToGBLSensor");
					m->add(fun(&ccHObjectCaster::ToCameraSensor), "ToCameraSensor");
					m->add(fun(&ccHObjectCaster::ToImage), "ToImage");
					m->add(fun(&ccHObjectCaster::To2DLabel), "To2DLabel");
					m->add(fun(&ccHObjectCaster::To2DViewportLabel), "To2DViewportLabel");
					m->add(fun(&ccHObjectCaster::To2DViewportObject), "To2DViewportObject");
					m->add(fun(&ccHObjectCaster::ToTransBuffer), "ToTransBuffer");
					return m;
				}


				
				
				ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
				{
					bs_ccObject(m);
					bs_ccHObject(m);
					bs_ccCustomHObject(m);
					bs_ccCustomLeafObject(m);
					bs_ccGLMatrixTpl<float>("ccGLMatrixTplf", m);
					bs_ccGLMatrixTpl<double>("ccGLMatrixTpld", m);
					bs_ccArray<CompressedNormType, 1, CompressedNormType>("internal_compressed_normal_array", m);
					bs_ccArray<CCVector3, 3, PointCoordinateType>("internal_normal_array", m);
					bs_ccArray<ccColor::Rgb, 3, ColorCompType>("internal_rgb_array", m);
					bs_ccArray<TexCoords2D, 2, float>("internal_tex2d_array", m);
					bs_NormsIndexesTableType(m);
					bs_NormsTableType(m);
					bs_ColorsTableType(m);
					bs_TexCoords2D(m);
					bs_TextureCoordsContainer(m);
					bs_ccGLMatrix(m);
					bs_ccGLMatrixd(m);
					bs_ccInteractor(m);
					bs_ccImage(m);
					bs_cc2DLabel(m);
					bs_cc2DViewportObject(m);
					bs_cc2DViewportLabel(m);
					bs_ccSensor(m);
					bs_ccCameraSensor(m);
					bs_ccGBLSensor(m);
					bs_ccPlanarEntityInterface(m);
					bs_ccDrawableObject(m);
					bs_ccGenericMesh(m);
					bs_ccMesh(m);
					bs_ccGenericPrimitive(m);
					bs_ccPlane(m);
					bs_ccSphere(m);
					bs_ccCone(m);
					bs_ccCylinder(m);
					bs_ccDish(m);
					bs_ccExtru(m);
					bs_ccFacet(m);
					bs_ccShiftedObject(m);
					bs_ccGenericPointCloud(m);
					bs_ccPointCloud(m);
					bs_ccKdTree(m);
					bs_ccOctree(m);
					bs_ccOctreeProxy(m);
					bs_ccPolyline(m);
					bs_ccSubMesh(m);
					bs_ccScalarField(m);
					bs_ccQuadric(m);
					bs_ccProgressDialog(m);
					bs_ccPointCloudInterpolator(m);
					bs_ccNormalVectors(m);
					bs_ccNormalCompressor(m);
					//bs_ccMinimumSpanningTreeForNormsDirection(m);
					//bs_ccFastMarchingForNormsDirection(m);
					bs_ccMaterialSet(m);
					bs_ccMaterial(m);
					bs_ccLog(m);
					bs_ccIndexedTransformation(m);
					bs_ccIndexedTransformationBuffer(m);
					bs_ccGL(m);
					bs_ccGriddedTools(m);
					bs_ccViewportParameters(m);
					bs_ccGLCameraParameters(m);
					bs_ccGenericGLDisplay(m);
					bs_ccFlags(m);
					bs_ccFileUtils(m);
					bs_ccDepthBuffer(m);
					bs_ccColor(m);
					bs_ccColorScalesManager(m);
					bs_ccColorScaleElement(m);
					bs_ccColorScale(m);
					bs_ccClipBox(m);

	
					bs_ccHObjectCaster(m);
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_DB_CLASSES_HPP