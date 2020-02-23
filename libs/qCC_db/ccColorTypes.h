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

#ifndef CC_COLOR_TYPES_HEADER
#define CC_COLOR_TYPES_HEADER

//Local
#include "qCC_db.h"

//Qt
#include <QColor>

//! Default color components type (R,G and B)
using ColorCompType = unsigned char;

//! Colors namespace
namespace ccColor
{
	//! Max value of a single color component (default type)
	constexpr ColorCompType MAX = 255;

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
		constexpr inline RgbTpl() : r(0), g(0), b(0) {}

		//! Constructor from a triplet of r,g,b values
		explicit constexpr inline RgbTpl(Type red, Type green, Type blue) : r(red), g(green), b(blue) {}

		//! Constructor from an array of 3 values
		explicit constexpr inline RgbTpl(const Type col[3]) : r(col[0]), g(col[1]), b(col[2]) {}

		//! Comparison operator
		inline bool operator != (const RgbTpl<Type>& t) const { return (r != t.r || g != t.g || b != t.b); }
	};

	//! 3 components, float type
	using Rgbf = RgbTpl<float>;
	//! 3 components, unsigned byte type
	using Rgbub = RgbTpl<unsigned char>;
	//! 3 components, default type
	using Rgb = RgbTpl<ColorCompType>;

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
		constexpr inline RgbaTpl() : r(0), g(0), b(0), a(0) {}

		//! Constructor from a triplet of r,g,b values and a transparency value
		explicit constexpr inline RgbaTpl(Type red, Type green, Type blue, Type alpha) : r(red), g(green), b(blue), a(alpha) {}

		//! RgbaTpl from an array of 4 values
		explicit constexpr inline RgbaTpl(const Type col[4]) : r(col[0]), g(col[1]), b(col[2]), a(col[3]) {}
		//! RgbaTpl from an array of 3 values and a transparency value
		explicit constexpr inline RgbaTpl(const Type col[3], Type alpha) : r(col[0]), g(col[1]), b(col[2]), a(alpha) {}
	
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
	using Rgbaf = RgbaTpl<float>;
	//! 4 components, unsigned byte type
	using Rgbaub = RgbaTpl<unsigned char>;
	//! 4 components, default type
	using Rgba = RgbaTpl<ColorCompType>;

	// Predefined colors (default type)
	constexpr Rgb whiteRGB					(MAX, MAX, MAX);
	constexpr Rgb lightGreyRGB				(static_cast<ColorCompType>(MAX*0.8), static_cast<ColorCompType>(MAX*0.8), static_cast<ColorCompType>(MAX*0.8));
	constexpr Rgb darkGreyRGB				(MAX / 2, MAX / 2, MAX / 2);
	constexpr Rgb redRGB					(MAX, 0, 0);
	constexpr Rgb greenRGB					(0, MAX, 0);
	constexpr Rgb blueRGB					(0, 0, MAX);
	constexpr Rgb darkBlueRGB				(0, 0, MAX / 2);
	constexpr Rgb magentaRGB				(MAX, 0, MAX);
	constexpr Rgb cyanRGB					(0, MAX, MAX);
	constexpr Rgb orangeRGB					(MAX, MAX / 2, 0);
	constexpr Rgb blackRGB					(0, 0, 0);
	constexpr Rgb yellowRGB					(MAX, MAX, 0);

	// Predefined colors (default type)
	constexpr Rgba white					(MAX, MAX, MAX, MAX);
	constexpr Rgba lightGrey				(static_cast<ColorCompType>(MAX*0.8), static_cast<ColorCompType>(MAX*0.8), static_cast<ColorCompType>(MAX*0.8), MAX);
	constexpr Rgba darkGrey					(MAX / 2, MAX / 2, MAX / 2, MAX);
	constexpr Rgba red						(MAX, 0, 0, MAX);
	constexpr Rgba green					(0, MAX, 0, MAX);
	constexpr Rgba blue						(0, 0, MAX, MAX);
	constexpr Rgba darkBlue					(0, 0, MAX / 2, MAX);
	constexpr Rgba magenta					(MAX, 0, MAX, MAX);
	constexpr Rgba cyan						(0, MAX, MAX, MAX);
	constexpr Rgba orange					(MAX, MAX / 2, 0, MAX);
	constexpr Rgba black					(0, 0, 0, MAX);
	constexpr Rgba yellow					(MAX, MAX, 0, MAX);

	// Predefined materials (float)
	constexpr Rgbaf bright					(1.00f, 1.00f, 1.00f, 1.00f);
	constexpr Rgbaf lighter					(0.83f, 0.83f, 0.83f, 1.00f);
	constexpr Rgbaf light					(0.66f, 0.66f, 0.66f, 1.00f);
	constexpr Rgbaf middle					(0.50f, 0.50f, 0.50f, 1.00f);
	constexpr Rgbaf dark					(0.34f, 0.34f, 0.34f, 1.00f);
	constexpr Rgbaf darker					(0.17f, 0.17f, 0.17f, 1.00f);
	constexpr Rgbaf darkest					(0.08f, 0.08f, 0.08f, 1.00f);
	constexpr Rgbaf night					(0.00f, 0.00f, 0.00f, 1.00F);
	constexpr Rgbaf defaultMeshFrontDiff	(0.00f, 0.90f, 0.27f, 1.00f);
	constexpr Rgbaf defaultMeshBackDiff		(0.27f, 0.90f, 0.90f, 1.00f);

	// Default foreground color (unsigned byte)
	constexpr Rgbub defaultBkgColor			( 10, 102, 151);		// dark blue
	constexpr Rgba defaultColor				(MAX, MAX, MAX, MAX);	// white
	constexpr Rgba defaultLabelBkgColor		(MAX, MAX, MAX, MAX);	// white
	constexpr Rgba defaultLabelMarkerColor	(MAX,   0, MAX, MAX);	// magenta

	//! Colors generator
	class Generator
	{
	public:
		
		//! Generates a random color
		QCC_DB_LIB_API static Rgb Random(bool lightOnly = true);
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

			float r = hue2rgb(p, q, H + 1.0f / 3.0f);
			float g = hue2rgb(p, q, H);
			float b = hue2rgb(p, q, H - 1.0f / 3.0f);

			return Rgb(	static_cast<ColorCompType>(r * ccColor::MAX),
						static_cast<ColorCompType>(g * ccColor::MAX),
						static_cast<ColorCompType>(b * ccColor::MAX) );

		}

		//! Converts a HSV color to RGB color space
		/** \param H [out] hue [0;360[
			\param S [out] saturation [0;1]
			\param V [out] value [0;1]
			\return RGB color (unsigned byte)
		**/
		QCC_DB_LIB_API static Rgb hsv2rgb(float H, float S, float V);

	private:

		//! Method used by hsl2rgb
		static float hue2rgb(float m1, float m2, float hue)
		{
			if (hue < 0)
				hue += 1.0f;
			else if (hue > 1.0f)
				hue -= 1.0f;

			if (6 * hue < 1.0f)
				return m1 + (m2 - m1) * hue * 6;
			else if (2 * hue < 1.0f)
				return m2;
			else if (3 * hue < 2.0f)
				return m1 + (m2 - m1) * (4.0f - hue * 6);
			else
				return m1;
		}
	};

	//! Conversion from Rgbf
	inline Rgb FromRgbfToRgb(const Rgbf& color)
	{
		return Rgb( static_cast<ColorCompType>(color.r * MAX),
					static_cast<ColorCompType>(color.g * MAX),
					static_cast<ColorCompType>(color.b * MAX) );
	}
	
	//! Conversion from Rgbaf to Rgb
	inline Rgb FromRgbafToRgb(const Rgbaf& color)
	{
		return Rgb( static_cast<ColorCompType>(color.r * MAX),
					static_cast<ColorCompType>(color.g * MAX),
					static_cast<ColorCompType>(color.b * MAX) );
	}
	
	//! Conversion from Rgbaf to Rgba
	inline Rgba FromRgbafToRgba(const Rgbaf& color)
	{
		return Rgba( static_cast<ColorCompType>(color.r * MAX),
					 static_cast<ColorCompType>(color.g * MAX),
					 static_cast<ColorCompType>(color.b * MAX),
					 static_cast<ColorCompType>(color.a * MAX));
	}

	//! Conversion from QRgb
	inline Rgb FromQRgb(QRgb qColor)
	{
		return Rgb( static_cast<unsigned char>(qRed(qColor)),
					static_cast<unsigned char>(qGreen(qColor)),
					static_cast<unsigned char>(qBlue(qColor)) );
	}
	
	//! Conversion from QRgb'a'
	inline Rgba FromQRgba(QRgb qColor)
	{
		return Rgba( static_cast<unsigned char>(qRed(qColor)),
					 static_cast<unsigned char>(qGreen(qColor)),
					 static_cast<unsigned char>(qBlue(qColor)),
					 static_cast<unsigned char>(qAlpha(qColor)) );
	}
	
	//! Conversion from QColor
	inline Rgb FromQColor(const QColor& qColor)
	{
		return Rgb( static_cast<unsigned char>(qColor.red()),
					static_cast<unsigned char>(qColor.green()),
					static_cast<unsigned char>(qColor.blue()) );
	}
	
	//! Conversion from QColor'a'
	inline Rgba FromQColora(const QColor& qColor)
	{
		return Rgba( static_cast<unsigned char>(qColor.red()),
					 static_cast<unsigned char>(qColor.green()),
					 static_cast<unsigned char>(qColor.blue()),
					 static_cast<unsigned char>(qColor.alpha()) );
	}
	
	//! Conversion from QColor (floating point)
	inline Rgbf FromQColorf(const QColor& qColor)
	{
		return Rgbf( static_cast<float>(qColor.redF()),
					 static_cast<float>(qColor.greenF()),
					 static_cast<float>(qColor.blueF()) );
	}
	
	//! Conversion from QColor'a' (floating point)
	inline Rgbaf FromQColoraf(const QColor& qColor)
	{
		return Rgbaf( static_cast<float>(qColor.redF()),
					  static_cast<float>(qColor.greenF()),
					  static_cast<float>(qColor.blueF()),
					  static_cast<float>(qColor.alphaF()) );
	}
};

#endif //CC_COLOR_TYPES_HEADER
