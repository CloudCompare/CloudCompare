#pragma once

//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPointCloudInterpolator.h>

class QWidget;

class ccMainAppInterface;

namespace ccEntityAction
{

	// Colours
	bool	setColor(ccHObject::Container selectedEntities, bool colorize, QWidget* parent = nullptr);
	bool	rgbToGreyScale(const ccHObject::Container &selectedEntities);
	bool	setColorGradient(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	changeColorLevels(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	interpolateColors(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	convertTextureToColor(const ccHObject::Container& selectedEntities, QWidget* parent = nullptr);
	bool	enhanceRGBWithIntensities(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	rgbGaussianFilter(const ccHObject::Container &selectedEntities, ccPointCloud::RgbFilterOptions filterParams, QWidget* parent = nullptr);
	
	// Scalar Fields
	bool	sfGaussianFilter(const ccHObject::Container &selectedEntities, ccPointCloud::RgbFilterOptions filterParams, QWidget* parent = nullptr);
	bool	sfConvertToRGB(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	sfRename(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	sfAddIdField(const ccHObject::Container &selectedEntities, bool storeAsInt = false);
    bool	sfSplitCloud(const ccHObject::Container &selectedEntities, ccMainAppInterface *app);
	bool	sfSetAsCoord(ccHObject* entity, QWidget* parent = nullptr);
	bool	sfSetAsCoord(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	exportCoordToSF(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	setSFsAsNormal(ccHObject* entity, QWidget* parent = nullptr);
	bool	exportNormalToSF(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr, bool* exportDimensions = nullptr);
	bool	sfArithmetic(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	sfFromColor(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	sfFromColor(const ccHObject::Container &selectedEntities, bool exportR, bool exportG, bool exportB, bool exportAlpha, bool exportComposite);
    bool	interpolateSFs(const ccHObject::Container &selectedEntities, ccMainAppInterface *parent);
    bool	interpolateSFs(ccPointCloud *source, ccPointCloud *dst, int sfIndex, ccPointCloudInterpolator::Parameters& params, QWidget* parent = nullptr);
	bool    sfAddConstant(ccPointCloud* cloud, QString sfName, bool integerValue, QWidget* parent = nullptr);

	bool	processMeshSF(const ccHObject::Container &selectedEntities, ccMesh::MESH_SCALAR_FIELD_PROCESS process, QWidget* parent = nullptr);
	
	// Normals
	bool	computeNormals(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	invertNormals(const ccHObject::Container &selectedEntities);
	bool	orientNormalsFM(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	orientNormalsMST(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	
	//! Normals conversion destinations
	enum class NORMAL_CONVERSION_DEST {
		HSV_COLORS,
		DIP_DIR_SFS
	};
	//! Converts a cloud's normals
	bool	convertNormalsTo(const ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest);
	
	// Octrees
	bool	computeOctree(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	
	// Properties
	enum class CLEAR_PROPERTY {
		COLORS = 0,
		NORMALS,
		CURRENT_SCALAR_FIELD,
		ALL_SCALAR_FIELDS
	};
	bool	clearProperty(ccHObject::Container selectedEntities, CLEAR_PROPERTY property, QWidget* parent = nullptr);
	
	enum class TOGGLE_PROPERTY {
		ACTIVE = 0,
		VISIBLE,
		COLOR,
		NORMALS,
		SCALAR_FIELD,
		MATERIAL,
		NAME
	};
	bool	toggleProperty(const ccHObject::Container &selectedEntities, TOGGLE_PROPERTY property);
	
	// Stats
	bool	statisticalTest(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
	bool	computeStatParams(const ccHObject::Container &selectedEntities, QWidget* parent = nullptr);
}
