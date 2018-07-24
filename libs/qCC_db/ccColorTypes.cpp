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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccColorTypes.h"

namespace ccColor
{
	// Predefined colors (default type)
	QCC_DB_LIB_API const Rgb white						(MAX, MAX, MAX);
	QCC_DB_LIB_API const Rgb lightGrey					(static_cast<ColorCompType>(MAX*0.8), static_cast<ColorCompType>(MAX*0.8), static_cast<ColorCompType>(MAX*0.8));
	QCC_DB_LIB_API const Rgb darkGrey					(MAX / 2, MAX / 2, MAX / 2);
	QCC_DB_LIB_API const Rgb red						(MAX, 0, 0);
	QCC_DB_LIB_API const Rgb green						(0, MAX, 0);
	QCC_DB_LIB_API const Rgb blue						(0, 0, MAX);
	QCC_DB_LIB_API const Rgb darkBlue					(0, 0, MAX / 2);
	QCC_DB_LIB_API const Rgb magenta					(MAX, 0, MAX);
	QCC_DB_LIB_API const Rgb cyan						(0, MAX, MAX);
	QCC_DB_LIB_API const Rgb orange						(MAX, MAX / 2, 0);
	QCC_DB_LIB_API const Rgb black						(0, 0, 0);
	QCC_DB_LIB_API const Rgb yellow						(MAX, MAX, 0);

	// Predefined materials (float)
	QCC_DB_LIB_API const Rgbaf bright					(1.00f, 1.00f, 1.00f, 1.00f);
	QCC_DB_LIB_API const Rgbaf lighter					(0.83f, 0.83f, 0.83f, 1.00f);
	QCC_DB_LIB_API const Rgbaf light					(0.66f, 0.66f, 0.66f, 1.00f);
	QCC_DB_LIB_API const Rgbaf middle					(0.50f, 0.50f, 0.50f, 1.00f);
	QCC_DB_LIB_API const Rgbaf dark						(0.34f, 0.34f, 0.34f, 1.00f);
	QCC_DB_LIB_API const Rgbaf darker					(0.17f, 0.17f, 0.17f, 1.00f);
	QCC_DB_LIB_API const Rgbaf darkest					(0.08f, 0.08f, 0.08f, 1.00f);
	QCC_DB_LIB_API const Rgbaf night					(0.00f, 0.00f, 0.00f, 1.00F);
	QCC_DB_LIB_API const Rgbaf defaultMeshFrontDiff		(0.00f, 0.90f, 0.27f, 1.00f);
	QCC_DB_LIB_API const Rgbaf defaultMeshBackDiff		(0.27f, 0.90f, 0.90f, 1.00f);

	// Default foreground color (unsigned byte)
	QCC_DB_LIB_API const Rgbub defaultColor				(255, 255, 255); //white
	QCC_DB_LIB_API const Rgbub defaultBkgColor			( 10, 102, 151); //dark blue
	QCC_DB_LIB_API const Rgbub defaultLabelBkgColor		(255, 255, 255); //white
	QCC_DB_LIB_API const Rgbub defaultLabelMarkerColor	(255,   0, 255); //magenta
};
