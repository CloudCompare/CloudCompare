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

#include <cmath>
#include <random>

#include "ccColorTypes.h"


namespace ccColor
{
	
	Rgb Generator::Random(bool lightOnly)
	{
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		std::uniform_int_distribution<uint16_t> dist(0, MAX); //1-byte types are not allowed
		
		Rgb col;
		col.r = static_cast<unsigned char>(dist(gen));
		col.g = static_cast<unsigned char>(dist(gen));
		if (lightOnly)
		{
			col.b = MAX - static_cast<ColorCompType>((static_cast<double>(col.r) + static_cast<double>(col.g)) / 2); //cast to double to avoid overflow (whatever the type of ColorCompType!!!)
		}
		else
		{
			col.b = static_cast<unsigned char>(dist(gen));
		}
		
		return col;
	}
	
	Rgb Convert::hsv2rgb(float H, float S, float V)
	{
		float hi = 0;
		float f = std::modf(H / 60.0f, &hi);
		
		float l = V*(1.0f - S);
		float m = V*(1.0f - f*S);
		float n = V*(1.0f - (1.0f - f)*S);
		
		Rgbf rgb(0, 0, 0);
		
		switch (static_cast<int>(hi) % 6)
		{
			case 0:
				rgb.r = V; rgb.g = n; rgb.b = l;
				break;
			case 1:
				rgb.r = m; rgb.g = V; rgb.b = l;
				break;
			case 2:
				rgb.r = l; rgb.g = V; rgb.b = n;
				break;
			case 3:
				rgb.r = l; rgb.g = m; rgb.b = V;
				break;
			case 4:
				rgb.r = n; rgb.g = l; rgb.b = V;
				break;
			case 5:
				rgb.r = V; rgb.g = l; rgb.b = m;
				break;
		}
		
		return Rgb (static_cast<ColorCompType>(rgb.r * ccColor::MAX),
					static_cast<ColorCompType>(rgb.g * ccColor::MAX),
					static_cast<ColorCompType>(rgb.b * ccColor::MAX));
	}

};
