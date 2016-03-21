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
	void	setColor(ccHObject::Container selectedEntities, bool colorize, QWidget *parent = nullptr);
	void	rgbToGreyScale(const ccHObject::Container &selectedEntities);
	void	setColorGradient(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	changeColorLevels(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	interpolateColors(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	
	// Scalar Fields
	void	sfGaussianFilter(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfBilateralFilter(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfConvertToRGB(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfRename(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfAddIdField(const ccHObject::Container &selectedEntities);
	void	sfAsCoord(const ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	
	// Normals
	void	computeNormals(const ccHObject::Container &selectedEntities, QWidget *parent);
	void	invertNormals(const ccHObject::Container &selectedEntities);
	
	//! Normals conversion destinations
	enum NORMAL_CONVERSION_DEST {
		HSV_COLORS,
		DIP_DIR_SFS
	};
	//! Converts a cloud's normals
	void	convertNormalsTo(const ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest);

	// Octrees
	void	computeOctree(const ccHObject::Container &selectedEntities, QWidget *parent);
}

#endif
