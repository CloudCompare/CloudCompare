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

#ifndef CC_COLOR_TYPES_HEADER
#define CC_COLOR_TYPES_HEADER

//system
#include <stdlib.h>
#include <math.h> //for modf

//! Default color components type (R,G and B)
typedef unsigned char ColorCompType;

//! Colors namespace
namespace ccColor
{
	//! Max value of a single color component (default type)
	const ColorCompType MAX = 255;

	//! RGB color structure
	template <typename Type> class RgbTpl
	{
	public:
	
		//! 3-tuple as a union
		union
		{
			struct
			{
				Type r,g,b;
			};
			Type rgb[3];
		};

		//! Default constructor
		/** Inits color to (0,0,0).
		**/
		inline RgbTpl() : r(0), g(0), b(0) {}

		//! Constructor from a triplet of r,g,b values
		explicit inline RgbTpl(Type red, Type green, Type blue) : r(red), g(green), b(blue) {}

		//! Constructor from an array of 3 values
		explicit inline RgbTpl(const Type col[3]) : r(col[0]), g(col[1]), b(col[2]) {}
	
		//! Copy constructor
		inline RgbTpl(const RgbTpl& c) : r(c.r), g(c.g), b(c.b) {}

		//! Comparison operator
		inline bool operator != (const RgbTpl<Type>& t) const { return (r != t.r || g != t.g || b != t.b); }
	};

	//! 3 components, float type
	typedef RgbTpl<float> Rgbf;
	//! 3 components, unsigned byte type
	typedef RgbTpl<unsigned char> Rgbub;
	//! 3 components, default type
	typedef RgbTpl<ColorCompType> Rgb;

	//! RGBA color structure
	template <class Type> class RgbaTpl
	{
	public:
	
		// 4-tuple values as a union
		union
		{
			struct
			{
				Type r,g,b,a;
			};
			Type rgba[4];
		};

		//! Default constructor
		/** Inits color to (0,0,0,0).
		**/
		inline RgbaTpl() : r(0), g(0), b(0), a(0) {}

		//! Constructor from a triplet of r,g,b values and a transparency value
		explicit inline RgbaTpl(Type red, Type green, Type blue, Type alpha) : r(red), g(green), b(blue), a(alpha) {}

		//! RgbaTpl from an array of 4 values
		explicit inline RgbaTpl(const Type col[4]) : r(col[0]), g(col[1]), b(col[2]), a(col[3]) {}
		//! RgbaTpl from an array of 3 values and a transparency value
		explicit inline RgbaTpl(const Type col[3], Type alpha) : r(col[0]), g(col[1]), b(col[2]), a(alpha) {}
	
		//! Copy constructor
		inline RgbaTpl(const RgbaTpl<Type>& c) : r(c.r), g(c.g), b(c.b), a(c.a) {}
		//! Copy constructor
		inline RgbaTpl(const RgbTpl<Type>& c, Type alpha) : r(c.r), g(c.g), b(c.b), a(alpha) {}

		//! Cast operator
		inline operator RgbTpl<Type>() const { return RgbTpl<Type>(rgba); }
		//! Cast operator (const version)
		//inline operator const Type*() const { return rgba; }

		//! Comparison operator
		inline bool operator != (const RgbaTpl<Type>& t) const { return (r != t.r || g != t.g || b != t.b || a != t.a); }
	};

	//! 4 components, float type
	typedef RgbaTpl<float> Rgbaf;
	//! 4 components, unsigned byte type
	typedef RgbaTpl<unsigned char> Rgbaub;
	//! 4 components, default type
	typedef RgbaTpl<ColorCompType> Rgba;

	// Predefined colors (default type)
	static const Rgba white						(MAX,MAX,MAX,MAX);
	static const Rgba lightGrey					(static_cast<ColorCompType>(MAX*0.8),static_cast<ColorCompType>(MAX*0.8),static_cast<ColorCompType>(MAX*0.8),MAX);
	static const Rgba darkGrey					(MAX/2,MAX/2,MAX/2,MAX);
	static const Rgba red						(MAX,0,0,MAX);
	static const Rgba green						(0,MAX,0,MAX);
	static const Rgba blue						(0,0,MAX,MAX);
	static const Rgba darkBlue					(0,0,MAX/2,MAX);
	static const Rgba magenta					(MAX,0,MAX,MAX);
	static const Rgba cyan						(0,MAX,MAX,MAX);
	static const Rgba orange					(MAX,MAX/2,0,MAX);
	static const Rgba black						(0,0,0,MAX);
	static const Rgba yellow					(MAX,MAX,0,MAX);

	// Predefined materials (float)
	static const Rgbaf bright					(1.00f, 1.00f, 1.00f, 1.00f);
	static const Rgbaf lighter					(0.83f, 0.83f, 0.83f, 1.00f);
	static const Rgbaf light					(0.66f, 0.66f, 0.66f, 1.00f);
	static const Rgbaf middle					(0.50f, 0.50f, 0.50f, 1.00f);
	static const Rgbaf dark						(0.34f, 0.34f, 0.34f, 1.00f);
	static const Rgbaf darker					(0.17f, 0.17f, 0.17f, 1.00f);
	static const Rgbaf darkest					(0.08f, 0.08f, 0.08f, 1.00f);
	static const Rgbaf night					(0.00f, 0.00f, 0.00f, 1.00F);
	static const Rgbaf defaultMeshFrontDiff		(0.00f, 0.90f, 0.27f, 1.00f);
	static const Rgbaf defaultMeshBackDiff		(0.27f, 0.90f, 0.90f, 1.00f);

	// Default foreground color (unsigned byte)
	static const Rgbub defaultColor				(255, 255, 255); //white
	static const Rgbub defaultBkgColor			( 10, 102, 151); //dark blue
	static const Rgbub defaultLabelBkgColor		(255, 255, 255); //white
	static const Rgbub defaultLabelMarkerColor	(255,   0, 255); //magenta

	//! Colors generator
	class Generator
	{
	public:
		
		//! Generates a random color
		static Rgb Random(bool lightOnly = true)
		{
			Rgb col;
			col.r = static_cast<ColorCompType>(MAX * (static_cast<float>(rand()) / RAND_MAX));
			col.g = static_cast<ColorCompType>(MAX * (static_cast<float>(rand()) / RAND_MAX));
			if (lightOnly)
			{
				col.b = MAX - static_cast<ColorCompType>((static_cast<double>(col.r)+static_cast<double>(col.g))/2); //cast to double to avoid overflow (whatever the type of ColorCompType!!!)
			}
			else
			{
				col.b = static_cast<ColorCompType>(MAX * (static_cast<float>(rand()) / RAND_MAX));
			}

			return col;
		}
	};

	//! Color space conversion
	class Convert
	{
	public:

		//! Converts a HSL color to RGB color space
		/** \param H [out] hue [0;360[
			\param S [out] saturation [0;1]
			\param L [out] light [0;1]
			\return RGB color (unsigned byte)
		**/
		static Rgb hsl2rgb(float H, float S, float L)
		{
			H /= 360;
			float q = L < 0.5f ? L * (1.0f + S) : L + S - L * S;
			float p = 2 * L - q;
			
			float r = hue2rgb(p, q, H + 1.0f/3.0f);
			float g = hue2rgb(p, q, H);
			float b = hue2rgb(p, q, H - 1.0f/3.0f);

			return Rgb (static_cast<ColorCompType>(r * ccColor::MAX),
						static_cast<ColorCompType>(g * ccColor::MAX),
						static_cast<ColorCompType>(b * ccColor::MAX));

		}

		//! Converts a HSV color to RGB color space
		/** \param H [out] hue [0;360[
			\param S [out] saturation [0;1]
			\param V [out] value [0;1]
			\return RGB color (unsigned byte)
		**/
		static Rgb hsv2rgb(float H, float S, float V)
		{
			int hi = ((static_cast<int>(H)/60) % 6);
			double f = 0;
			modf(H/60.0, &f);
			float l = static_cast<float>(V*(1.0-S));
			float m = static_cast<float>(V*(1.0-f*S));
			float n = static_cast<float>(V*(1.0-(1.0-f)*S));

			Rgbf rgb(0,0,0);

			switch (hi)
			{
			case 0:
				rgb.r=V; rgb.g=n; rgb.b=l;
				break;
			case 1:
				rgb.r=m; rgb.g=V; rgb.b=l;
				break;
			case 2:
				rgb.r=l; rgb.g=V; rgb.b=n;
				break;
			case 3:
				rgb.r=l; rgb.g=m; rgb.b=V;
				break;
			case 4:
				rgb.r=n; rgb.g=l; rgb.b=V;
				break;
			case 5:
				rgb.r=V; rgb.g=l; rgb.b=m;
				break;
			}

			return Rgb (static_cast<ColorCompType>(rgb.r * ccColor::MAX),
						static_cast<ColorCompType>(rgb.g * ccColor::MAX),
						static_cast<ColorCompType>(rgb.b * ccColor::MAX));
		}

	protected:
		
		//! Method used by hsl2rgb
		static float hue2rgb(float m1, float m2, float hue)
		{
			if (hue < 0)
				hue += 1.0f;
            else if (hue > 1.0f)
				hue -= 1.0f;

			if (6*hue < 1.0f)
				return m1 + (m2 - m1) * hue * 6;
            else if (2 * hue < 1.0f)
				return m2;
            else if (3 * hue < 2.0f)
				return m1 + (m2 - m1) * (4.0f - hue * 6);
			else
				return m1;
        }


	};
};

#endif //CC_COLOR_TYPES_HEADER
