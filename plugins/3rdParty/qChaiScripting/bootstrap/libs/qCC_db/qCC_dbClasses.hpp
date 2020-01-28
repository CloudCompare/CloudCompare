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

#include <QString>

#include <ccObject.h>
#include <ccHObject.h>
#include <ccGenericMesh.h>
#include <ccGenericPointCloud.h>
#include <ccMaterialSet.h>
#include <GenericProgressCallback.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccGenericPointCloud.h>
#include <ccDrawableObject.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccGenericPrimitive.h>
#include <ccPlanarEntityInterface.h>
#include <ccPlane.h>

#include <GenericIndexedMesh.h>

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
					return m;
				}

				ModulePtr bs_ccHObject(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::user_type<ccHObject>(), "ccHObject");
					m->add(chaiscript::constructor<ccHObject(const QString&)>(), "ccHObject");
					m->add(chaiscript::constructor<ccHObject(const ccHObject&)>(), "ccHObject");
					m->add(fun(&ccHObject::GetCurrentDBVersion), "GetCurrentDBVersion");
					m->add(fun(&ccHObject::SetUniqueIDGenerator), "SetUniqueIDGenerator");
					m->add(fun(&ccHObject::GetUniqueIDGenerator), "GetUniqueIDGenerator");
					m->add(fun(&ccHObject::getClassID), "getClassID");
					m->add(fun(&ccHObject::getName), "getName");
					m->add(fun(&ccHObject::setName), "setName");
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
					m->add(fun(&ccDrawableObject::setTempColor), "setTempColor");
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
					m->add(fun(&ccGenericMesh::interpolateColors), "interpolateColors");
					m->add(fun(&ccGenericMesh::interpolateColorsBC), "interpolateColorsBC");
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
					/*m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");
					m->add(fun(&ccGenericMesh::XX), "XX");*/
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
					m->add(fun(&ccMesh::interpolateColorsBC), "interpolateColorsBC");
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
					m->add(fun(&ccGenericPrimitive::interpolateColorsBC), "interpolateColorsBC");
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
					m->add(fun(&ccPlane::interpolateColorsBC), "interpolateColorsBC");
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
					m->add(fun(&ccPlane::getName), "getName");
					m->add(fun(&ccPlane::setName), "setName");
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
					return m;
				}


				ModulePtr bs_class_relationships(ModulePtr m = std::make_shared<Module>())
				{
					m->add(chaiscript::base_class<ccObject, ccHObject>());
					m->add(chaiscript::base_class<ccDrawableObject, ccHObject>());
					
					m->add(chaiscript::base_class<ccDrawableObject, ccGenericMesh>());
					m->add(chaiscript::base_class<ccHObject, ccGenericMesh>());
					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccGenericMesh>());
					
					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccMesh>());
					m->add(chaiscript::base_class<ccHObject, ccMesh>());
					m->add(chaiscript::base_class<ccObject, ccMesh>());
					m->add(chaiscript::base_class<ccDrawableObject, ccMesh>());

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccHObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccDrawableObject, ccGenericPrimitive>());
					m->add(chaiscript::base_class<ccMesh, ccGenericPrimitive>());

					m->add(chaiscript::base_class<CCLib::GenericIndexedMesh, ccPlane>());
					m->add(chaiscript::base_class<ccHObject, ccPlane>());
					m->add(chaiscript::base_class<ccObject, ccPlane>());
					m->add(chaiscript::base_class<ccDrawableObject, ccPlane>());
					m->add(chaiscript::base_class<ccMesh, ccPlane>());
					m->add(chaiscript::base_class<ccGenericPrimitive, ccPlane>());
					m->add(chaiscript::base_class<ccPlanarEntityInterface, ccPlane>());


					return m;
				}


				ModulePtr bootstrap_classes(ModulePtr m = std::make_shared<Module>())
				{
					bs_ccObject(m);
					bs_ccHObject(m);
					bs_ccGLMatrix(m);
					bs_ccGLMatrixd(m);
					bs_ccPlanarEntityInterface(m);
					bs_ccDrawableObject(m);
					bs_ccGenericMesh(m);
					bs_ccMesh(m);
					bs_ccGenericPrimitive(m);
					bs_ccPlane(m);


					bs_class_relationships(m);			
					return m;
				}
			}
		}
	}
}

#endif //CHAISCRIPTING_BOOTSTRAP_QCC_DB_CLASSES_HPP