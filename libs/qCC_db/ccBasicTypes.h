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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_BASIC_TYPES_HEADER
#define CC_BASIC_TYPES_HEADER

//CCLib
#include <CCGeom.h>

/***************************************************
			Main CloudCompare types
***************************************************/

//! Compressed normals type
typedef unsigned short normsType;

//! Color components type (R,G and B)
typedef unsigned char colorType;
//! Max value of a color component
const colorType MAX_COLOR_COMP = 255;

//! Scan index type
typedef unsigned char scanIndexType;

namespace ccColor
{
    // Predefined colors
    static const colorType white[3]			=	{MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
    static const colorType lightGrey[3]		=	{200,200,200};
    static const colorType darkGrey[3]		=	{MAX_COLOR_COMP/2,MAX_COLOR_COMP/2,MAX_COLOR_COMP/2};
    static const colorType red[3]			=	{MAX_COLOR_COMP,0,0};
    static const colorType green[3]			=	{0,MAX_COLOR_COMP,0};
    static const colorType blue[3]			=	{0,0,MAX_COLOR_COMP};
    static const colorType darkBlue[3]		=	{0,0,MAX_COLOR_COMP/2};
    static const colorType magenta[3]		=	{MAX_COLOR_COMP,0,MAX_COLOR_COMP};
    static const colorType cyan[3]		    =	{0,MAX_COLOR_COMP,MAX_COLOR_COMP};
    static const colorType orange[3]		=	{MAX_COLOR_COMP,MAX_COLOR_COMP/2,0};
    static const colorType black[3]			=	{0,0,0};
    static const colorType yellow[3]		=	{MAX_COLOR_COMP,MAX_COLOR_COMP,0};

    // Predefined materials
    static const float bright[4]			    =	{1.0f,1.0f,1.0f,1.0f};
    static const float lighter[4]			    =	{0.83f,0.83f,0.83f,1.0f};
    static const float light[4]				    =	{0.66f,0.66f,0.66f,1.0f};
    static const float middle[4]			    =	{0.5f,0.5f,0.5f,1.0f};
    static const float dark[4]				    =	{0.34f,0.34f,0.34f,1.0f};
    static const float darker[4]			    =	{0.17f,0.17f,0.17f,1.0f};
    static const float darkest[4]			    =	{0.08f,0.08f,0.08f,1.0f};
    static const float night[4]				    =	{0.0F,0.0F,0.0F,1.0F};
    static const float defaultMeshFrontDiff[4]  =   {0.0f,1.0f,0.32f,1.0f};
    static const float defaultMeshBackDiff[4]   =   {0.32f,1.0f,1.0f,1.0f};

    // Default foreground color
    static const colorType defaultColor[3]			=   {MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
    static const colorType defaultBkgColor[3]		=   {10,102,151};
    static const colorType defaultHistBkgColor[3]	=   {51,0,51};
    static const colorType defaultLabelColor[3]		=   {255,255,0};
};

#endif //CC_BASIC_TYPES_HEADER
