#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

// qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPointCloudInterpolator.h>

class QWidget;

class ccMainAppInterface;

namespace ccEntityAction
{

	// Colours
	bool setColor(ccHObject::Container selectedEntities, bool colorize, QWidget* parent = nullptr);
	bool rgbToGreyScale(ccHObject::Container selectedEntities);
	bool setColorGradient(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool changeColorLevels(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool interpolateColors(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool convertTextureToColor(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool enhanceRGBWithIntensities(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool rgbGaussianFilter(ccHObject::Container selectedEntities, ccPointCloud::RgbFilterOptions filterParams, QWidget* parent = nullptr);

	// Scalar Fields
	bool sfGaussianFilter(ccHObject::Container selectedEntities, ccPointCloud::RgbFilterOptions filterParams, QWidget* parent = nullptr);
	bool sfConvertToRGB(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool sfConvertToRandomRGB(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool sfRename(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool sfAddIdField(ccHObject::Container selectedEntities, bool storeAsInt = false);
	bool sfSplitCloud(ccHObject::Container selectedEntities, ccMainAppInterface* app);
	bool sfSetAsCoord(ccHObject* entity, QWidget* parent = nullptr);
	bool sfSetAsCoord(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool exportCoordToSF(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool setSFsAsNormal(ccHObject* entity, QWidget* parent = nullptr);
	bool exportNormalToSF(ccHObject::Container selectedEntities, QWidget* parent = nullptr, bool* exportDimensions = nullptr);
	bool sfArithmetic(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool sfFromColor(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool sfFromColor(ccHObject::Container selectedEntities, bool exportR, bool exportG, bool exportB, bool exportAlpha, bool exportComposite);
	bool interpolateSFs(ccHObject::Container selectedEntities, ccMainAppInterface* parent);
	bool interpolateSFs(ccPointCloud* source, ccPointCloud* dst, int sfIndex, ccPointCloudInterpolator::Parameters& params, QWidget* parent = nullptr);
	bool sfAddConstant(ccPointCloud* cloud, QString sfName, bool integerValue, QWidget* parent = nullptr);

	bool processMeshSF(ccHObject::Container selectedEntities, ccMesh::MESH_SCALAR_FIELD_PROCESS process, QWidget* parent = nullptr);

	// Normals
	bool computeNormals(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool invertNormals(ccHObject::Container selectedEntities);
	bool orientNormalsFM(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool orientNormalsMST(ccHObject::Container selectedEntities, QWidget* parent = nullptr);

	//! Normals conversion destinations
	enum class NORMAL_CONVERSION_DEST
	{
		HSV_COLORS,
		DIP_DIR_SFS
	};
	//! Converts a cloud's normals
	bool convertNormalsTo(ccHObject::Container selectedEntities, NORMAL_CONVERSION_DEST dest);

	// Octrees
	bool computeOctree(ccHObject::Container selectedEntities, QWidget* parent = nullptr);

	// Properties
	enum class CLEAR_PROPERTY
	{
		COLORS = 0,
		NORMALS,
		CURRENT_SCALAR_FIELD,
		ALL_SCALAR_FIELDS
	};
	bool clearProperty(ccHObject::Container selectedEntities, CLEAR_PROPERTY property, QWidget* parent = nullptr);

	enum class TOGGLE_PROPERTY
	{
		ACTIVE = 0,
		VISIBLE,
		COLOR,
		NORMALS,
		SCALAR_FIELD,
		MATERIAL,
		NAME
	};
	bool toggleProperty(ccHObject::Container selectedEntities, TOGGLE_PROPERTY property);

	// Stats
	bool statisticalTest(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
	bool computeStatParams(ccHObject::Container selectedEntities, QWidget* parent = nullptr);
} // namespace ccEntityAction
