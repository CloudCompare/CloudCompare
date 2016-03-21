#ifndef CCENTITYACTION_H
#define CCENTITYACTION_H
//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccColorScale.h"
#include "ccHObject.h"

class QWidget;


namespace ccEntityAction
{
	// Colours
	void	setColor(ccHObject::Container selectedEntities, bool colorize, QWidget *parent);
	void	rgbToGreyScale(const ccHObject::Container &selectedEntities);
	void	setColorGradient(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	changeColorLevels(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	interpolateColors(const ccHObject::Container &selectedEntities, QWidget *parent);
	
	// Scalar Fields
	void	sfGaussianFilter(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	sfBilateralFilter(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	sfConvertToRGB(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	sfRename(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	sfAddIdField(const ccHObject::Container &selectedEntities);
	void	sfAsCoord(const ccHObject::Container &selectedEntities, QWidget *parent);
	
	// Normals
	void	computeNormals(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	invertNormals(const ccHObject::Container &selectedEntities);
	
	//! Normals conversion destinations
	enum class NORMAL_CONVERSION_DEST {
		HSV_COLORS,
		DIP_DIR_SFS
	};
	//! Converts a cloud's normals
	void	convertNormalsTo(const ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest);

	// Octrees
	void	computeOctree(const ccHObject::Container &selectedEntities, QWidget *parent);

	// Properties
	enum class CLEAR_PROPERTY {
		COLORS = 0,
		NORMALS,
		CURRENT_SCALAR_FIELD,
		ALL_SCALAR_FIELDS
	};
	void	clearProperty(ccHObject::Container selectedEntities, CLEAR_PROPERTY property, QWidget *parent);	

	enum class TOGGLE_PROPERTY {
		ACTIVE = 0,
		VISIBLE,
		COLOR,
		NORMALS,
		SCALAR_FIELD,
		MATERIAL,
		NAME
	};
	void	toggleProperty(const ccHObject::Container &selectedEntities, TOGGLE_PROPERTY property);
}

#endif
