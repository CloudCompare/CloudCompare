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
	void	rgbToGreyScale(ccHObject::Container &selectedEntities);
	void	setColorGradient(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	changeColorLevels(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	interpolateColors(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	
	// Scalar Fields
	void	sfGaussianFilter(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfBilateralFilter(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfConvertToRGB(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfConvertToRandomRGB(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfRename(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	void	sfAddIdField(ccHObject::Container &selectedEntities);
	void	sfAsCoord(ccHObject::Container &selectedEntities, QWidget *parent = nullptr);
	
	// Normals
	void	invertNormals(ccHObject::Container &selectedEntities);
	
	//! Normals conversion destinations
	enum NORMAL_CONVERSION_DEST {
		HSV_COLORS,
		DIP_DIR_SFS
	};
	//! Converts a cloud's normals
	void	convertNormalsTo(ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest);
}

#endif
