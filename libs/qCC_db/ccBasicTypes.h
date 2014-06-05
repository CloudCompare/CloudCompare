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

//system
#include <stdlib.h>

//! Compressed normals type
typedef unsigned short normsType;

//! Color components type (R,G and B)
typedef unsigned char colorType;

//! Max value of a color component
const colorType MAX_COLOR_COMP = 255;

namespace ccColor
{
	// Predefined colors
	static const colorType white[3]					= {MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
	static const colorType lightGrey[3]				= {200,200,200};
	static const colorType darkGrey[3]				= {MAX_COLOR_COMP/2,MAX_COLOR_COMP/2,MAX_COLOR_COMP/2};
	static const colorType red[3]					= {MAX_COLOR_COMP,0,0};
	static const colorType green[3]					= {0,MAX_COLOR_COMP,0};
	static const colorType blue[3]					= {0,0,MAX_COLOR_COMP};
	static const colorType darkBlue[3]				= {0,0,MAX_COLOR_COMP/2};
	static const colorType magenta[3]				= {MAX_COLOR_COMP,0,MAX_COLOR_COMP};
	static const colorType cyan[3]					= {0,MAX_COLOR_COMP,MAX_COLOR_COMP};
	static const colorType orange[3]				= {MAX_COLOR_COMP,MAX_COLOR_COMP/2,0};
	static const colorType black[3]					= {0,0,0};
	static const colorType yellow[3]				= {MAX_COLOR_COMP,MAX_COLOR_COMP,0};

	// Predefined materials
	static const float bright[4]					= {1.0f,1.0f,1.0f,1.0f};
	static const float lighter[4]					= {0.83f,0.83f,0.83f,1.0f};
	static const float light[4]						= {0.66f,0.66f,0.66f,1.0f};
	static const float middle[4]					= {0.5f,0.5f,0.5f,1.0f};
	static const float dark[4]						= {0.34f,0.34f,0.34f,1.0f};
	static const float darker[4]					= {0.17f,0.17f,0.17f,1.0f};
	static const float darkest[4]					= {0.08f,0.08f,0.08f,1.0f};
	static const float night[4]						= {0.0F,0.0F,0.0F,1.0F};
	static const float defaultMeshFrontDiff[4]		= {0.0f,1.0f,0.32f,1.0f};
	static const float defaultMeshBackDiff[4]		= {0.32f,1.0f,1.0f,1.0f};

	// Default foreground color
	static const colorType defaultColor[3]			= {MAX_COLOR_COMP,MAX_COLOR_COMP,MAX_COLOR_COMP};
	static const colorType defaultBkgColor[3]		= {10,102,151};
	static const colorType defaultHistBkgColor[3]	= {51,0,51};
	static const colorType defaultLabelColor[3]		= {255,255,0};

	//! Colors generator
	class Generator
	{
	public:
		
		//! Generates a random color
		static void Random(colorType col[/*3*/], bool lightOnly = true)
		{
			col[0] = static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
			col[1] = static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
			if (lightOnly)
			{
				col[2] = MAX_COLOR_COMP - static_cast<colorType>((static_cast<unsigned>(col[1])+static_cast<unsigned>(col[2]))/2);
			}
			else
			{
				col[2] = static_cast<colorType>(static_cast<float>(MAX_COLOR_COMP) * static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
			}
		}
	};
};

#endif //CC_BASIC_TYPES_HEADER
