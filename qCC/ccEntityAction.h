#ifndef CCENTITYACTION_H
#define CCENTITYACTION_H
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

#include "ccMesh.h"

class QWidget;

class ccMainAppInterface;

namespace ccEntityAction
{
	// Colours
	bool	setColor(ccHObject::Container selectedEntities, bool colorize, QWidget *parent);
	bool	rgbToGreyScale(const ccHObject::Container &selectedEntities);
	bool	setColorGradient(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	changeColorLevels(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	interpolateColors(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	convertTextureToColor(const ccHObject::Container& selectedEntities, QWidget *parent);
	bool	enhanceRGBWithIntensities(const ccHObject::Container &selectedEntities, QWidget *parent);
	
	// Scalar Fields
	bool	sfGaussianFilter(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfBilateralFilter(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfConvertToRGB(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfRename(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfAddIdField(const ccHObject::Container &selectedEntities);
	bool	sfSetAsCoord(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	exportCoordToSF(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfArithmetic(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	sfFromColor(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	interpolateSFs(const ccHObject::Container &selectedEntities, ccMainAppInterface *parent);

	bool	processMeshSF(const ccHObject::Container &selectedEntities, ccMesh::MESH_SCALAR_FIELD_PROCESS process, QWidget *parent);
	
	// Normals
	bool	computeNormals(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	invertNormals(const ccHObject::Container &selectedEntities);
	bool	orientNormalsFM(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	orientNormalsMST(const ccHObject::Container &selectedEntities, QWidget *parent);
	
	//! Normals conversion destinations
	enum class NORMAL_CONVERSION_DEST {
		HSV_COLORS,
		DIP_DIR_SFS
	};
	//! Converts a cloud's normals
	bool	convertNormalsTo(const ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest);
	
	// Octrees
	bool	computeOctree(const ccHObject::Container &selectedEntities, QWidget *parent);
	
	// Properties
	enum class CLEAR_PROPERTY {
		COLORS = 0,
		NORMALS,
		CURRENT_SCALAR_FIELD,
		ALL_SCALAR_FIELDS
	};
	bool	clearProperty(ccHObject::Container selectedEntities, CLEAR_PROPERTY property, QWidget *parent);	
	
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
	bool	statisticalTest(const ccHObject::Container &selectedEntities, QWidget *parent);
	bool	computeStatParams(const ccHObject::Container &selectedEntities, QWidget *parent);
}

#endif
