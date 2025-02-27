#ifndef CC_E57_HEADER_HEADER
#define CC_E57_HEADER_HEADER

#include <cstdint>

//! Point prototype (structure use to interrogate if standardized fields are available)
/** Taken from "E57 Simple API" by Stan Coleby
**/
class PointStandardizedFieldsAvailable
{
public:
	bool		cartesianXField = false;			//!< Indicates that the PointRecord cartesianX field is active
	bool		cartesianYField = false;			//!< Indicates that the PointRecord cartesianY field is active
	bool		cartesianZField = false;			//!< Indicates that the PointRecord cartesianZ field is active
	bool		cartesianInvalidStateField = false; //!< Indicates that the PointRecord cartesianInvalidState field is active

	bool		sphericalRangeField = false;		//!< Indicates that the PointRecord sphericalRange field is active
	bool		sphericalAzimuthField = false;		//!< Indicates that the PointRecord sphericalAzimuth field is active
	bool		sphericalElevationField = false;	//!< Indicates that the PointRecord sphericalElevation field is active
	bool		sphericalInvalidStateField = false; //!< Indicates that the PointRecord sphericalInvalidState field is active

	double		pointRangeMinimum = 0.0;			//!< Indicates that the PointRecord cartesian and range fields should be configured with this minimum value -E57_FLOAT_MAX or -E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a minimum range value.
	double		pointRangeMaximum = 0.0;			//!< Indicates that the PointRecord cartesian and range fields should be configured with this maximum value E57_FLOAT_MAX or E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a maximum range value.
	double		pointRangeScaledInteger = 0.0;		//!< Indicates that the PointRecord cartesain and range fields should be configured as a ScaledIntegerNode with this scale setting. If 0. then use FloatNode.

	bool		normXField = false;					//!< Indicates that the PointRecord normalX field is active
	bool		normYField = false;					//!< Indicates that the PointRecord normalY field is active
	bool		normZField = false;					//!< Indicates that the PointRecord normalZ field is active
	double		normRangeMinimum = 0.0;				//!< Indicates that the PointRecord normal and range fields should be configured with this minimum value -E57_FLOAT_MAX or -E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a minimum range value.
	double		normRangeMaximum = 0.0;				//!< Indicates that the PointRecord normal and range fields should be configured with this maximum value E57_FLOAT_MAX or E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a maximum range value.
	double		normRangeScaledInteger = 0.0;		//!< Indicates that the PointRecord normal and range fields should be configured as a ScaledIntegerNode with this scale setting. If 0. then use FloatNode.

	double		angleMinimum = 0.0;					//!< Indicates that the PointRecord angle fields should be configured with this minimum value -E57_FLOAT_MAX or -E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a minimum angle value.
	double		angleMaximum = 0.0;					//!< Indicates that the PointRecord angle fields should be configured with this maximum value E57_FLOAT_MAX or E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a maximum angle value.
	double		angleScaledInteger = 0.0;			//!< Indicates that the PointRecord angle fields should be configured as a ScaledIntegerNode with this scale setting. If 0. then use FloatNode.

	bool		rowIndexField = false;				//!< Indicates that the PointRecord rowIndex field is active
	uint32_t	rowIndexMaximum = 0;				//!< Indicates that the PointRecord index fields should be configured with this maximum value where the minimum will be set to 0.
	bool		columnIndexField = false;			//!< Indicates that the PointRecord columnIndex field is active
	uint32_t	columnIndexMaximum = 0;				//!< Indicates that the PointRecord index fields should be configured with this maximum value where the minimum will be set to 0.

	bool		returnIndexField = false;			//!< Indicates that the PointRecord returnIndex field is active
	bool		returnCountField = false;			//!< Indicates that the PointRecord returnCount field is active
	uint8_t		returnMaximum = 0.0;				//!< Indicates that the PointRecord return fields should be configured with this maximum value where the minimum will be set to 0.

	bool		timeStampField = false;				//!< Indicates that the PointRecord timeStamp field is active
	bool		isTimeStampInvalidField = false;	//!< Indicates that the PointRecord isTimeStampInvalid field is active
	double		timeMaximum = 0.0;					//!< Indicates that the PointRecord timeStamp fields should be configured with this maximum value. like E57_UINT32_MAX, E57_FLOAT_MAX or E57_DOUBLE_MAX

	bool		intensityField = false;				//!< Indicates that the PointRecord intensity field is active
	bool		isIntensityInvalidField = false;	//!< Indicates that the PointRecord isIntensityInvalid field is active
	double		intensityScaledInteger = 0.0;		//!< Indicates that the PointRecord intensity fields should be configured as a ScaledIntegerNode with this setting. If 0. then use FloatNode, if -1. use IntegerNode

	bool		colorRedField = false;				//!< indicates that the PointRecord colorRed field is active
	bool		colorGreenField = false;			//!< indicates that the PointRecord colorGreen field is active
	bool		colorBlueField = false;				//!< indicates that the PointRecord colorBlue field is active
	bool		isColorInvalidField = false;		//!< Indicates that the PointRecord isColorInvalid field is active
};

//! Specifies the limits for the value of signal intensity that a sensor is capable of producing
/** From "E57 Simple API" by Stan Coleby
**/
class IntensityLimits
{
public:
	double		intensityMinimum = 0.0;		//!< The minimum producible intensity value. Unit is unspecified.
	double		intensityMaximum = 0.0;		//!< The maximum producible intensity value. Unit is unspecified.
};

//! Secifies the limits for the value of red, green, and blue color that a sensor is capable of producing.
/** From "E57 Simple API" by Stan Coleby
**/
class ColorLimits
{
public:
	double		colorRedMinimum = 0.0;		//!< The minimum producible red color value. Unit is unspecified.
	double		colorRedMaximum = 0.0;		//!< The maximum producible red color value. Unit is unspecified.
	double		colorGreenMinimum = 0.0;	//!< The minimum producible green color value. Unit is unspecified.
	double		colorGreenMaximum = 0.0;	//!< The maximum producible green color value. Unit is unspecified.
	double		colorBlueMinimum = 0.0;		//!< The minimum producible blue color value. Unit is unspecified.
	double		colorBlueMaximum = 0.0;		//!< The maximum producible blue color value. Unit is unspecified.
};

//! E57 scan useful header information (minimal set for CloudCompare only ;)
struct E57ScanHeader
{
	IntensityLimits						intensityLimits;	//!< The limits for the value of signal intensity that the sensor is capable of producing.
	ColorLimits							colorLimits;		//!< The limits for the value of red, green, and blue color that the sensor is capable of producing.
	PointStandardizedFieldsAvailable	pointFields;		//!< This defines the active fields used in the WritePoints function.
};

struct E57NodeDesc
{
	QString type;
	QString content;
};

struct E57NodeMap : QMap<QString, E57NodeDesc>
{
	QStringList toStringList() const
	{
		QStringList stringList;

		for (auto it = begin(); it != end(); ++it)
		{
			stringList << it.key() + "=" + it.value().type + "=" + it.value().content;
		}

		return stringList;
	}

	static void FromStringList(E57NodeMap& stringMap, const QStringList& stringList)
	{
		stringMap.clear();

		for (const QString& string : stringList)
		{
			QStringList tokens = string.split("=", Qt::KeepEmptyParts);
			if (tokens.size() == 3)
			{
				stringMap.insert(tokens[0], { tokens[1], tokens[2] });
			}
			else
			{
				ccLog::Warning("[E57NodeMap] Unexpected string: " + string);
			}
		}
	}
};

//! Identifies the representation for the image data
/** From "E57 Simple API" by Stan Coleby
**/
enum Image2DProjection
{
	E57_NO_PROJECTION = 0,	//!< No representation for the image data is present
	E57_VISUAL = 1,			//!< VisualReferenceRepresentation for the image data
	E57_PINHOLE = 2,		//!< PinholeRepresentation for the image data
	E57_SPHERICAL = 3,		//!< SphericalRepresentation for the image data
	E57_CYLINDRICAL = 4		//!< CylindricalRepresentation for the image data
};

//! Identifies the format representation for the image data
/** From "E57 Simple API" by Stan Coleby
**/
enum Image2DType
{
	E57_NO_IMAGE = 0,		//!< No image data
	E57_JPEG_IMAGE = 1,		//!< JPEG format image data.
	E57_PNG_IMAGE = 2,		//!< PNG format image data.
	E57_PNG_IMAGE_MASK = 3	//!< PNG format image mask.
};

//! Interface for E57 camera representation
class CameraRepresentation
{
public:
	virtual ~CameraRepresentation() = default;
	virtual Image2DProjection getType() { return E57_NO_PROJECTION; }
	virtual const char* getName() const { return {}; }
	virtual QStringList toStringList() const { return {}; }
	virtual bool fromStringList(const QStringList& stringList) { return true; }
};

//! Structure that stores an image that is to be used only as a visual reference.
/** From "E57 Simple API" by Stan Coleby
**/
class VisualReferenceRepresentation : public CameraRepresentation
{
public:
	Image2DProjection getType() override { return E57_VISUAL; }
	static const char* GetName() { return "visualReferenceRepresentation"; }
	const char* getName() const override { return GetName(); }

	Image2DType	imageType = E57_NO_IMAGE;	//!< image type.
	int64_t		imageSize = 0;				//!< size of image data in BlobNode.
	int64_t		imageMaskSize = 0;			//!< size of image mask data in BlobNode (if any).
	int32_t		imageWidth = 0;				//!< image width (in pixels). Shall be positive
	int32_t		imageHeight = 0;			//!< image height (in pixels). Shall be positive

	QStringList toStringList() const override
	{
		QStringList stringList;
		stringList << "type=" + QString(getName());
		stringList << "imageWidth=" + QString::number(imageWidth);
		stringList << "imageHeight=" + QString::number(imageHeight);

		return stringList;
	}

	static bool InitDoubleFromStringList(const QString& key, const QStringList& stringList, double& destValue)
	{
		for (const QString& string : stringList)
		{
			if (string.startsWith(key))
			{
				QString valueStr = string.mid(key.length() + 1); // +1 to account for the '=' character
				bool ok = false;
				destValue = valueStr.toDouble(&ok);
				return ok;
			}
		}
		return false;
	}

	static bool InitInt32FromStringList(const QString& key, const QStringList& stringList, int32_t& destValue)
	{
		for (const QString& string : stringList)
		{
			if (string.startsWith(key))
			{
				QString valueStr = string.mid(key.length() + 1); // +1 to account for the '=' character
				bool ok = false;
				destValue = valueStr.toInt(&ok);
				return ok;
			}
		}
		return false;
	}

	bool fromStringList(const QStringList& stringList) override
	{
		// mandatory fields
		return (	InitInt32FromStringList("imageWidth", stringList, imageWidth)
				&&	InitInt32FromStringList("imageHeight", stringList, imageHeight) );
	}
};

//! Structure that stores an image that is mapped from 3D using a spherical projection model
/** From "E57 Simple API" by Stan Coleby
**/
class SphericalRepresentation : public VisualReferenceRepresentation
{
public:
	Image2DProjection getType() override { return E57_SPHERICAL; }
	static const char* GetName() { return "sphericalRepresentation"; }
	const char* getName() const override { return GetName(); }

	QStringList toStringList() const override
	{
		QStringList stringList = VisualReferenceRepresentation::toStringList();

		stringList << "pixelWidth=" + QString::number(pixelWidth, 'f', 12);
		stringList << "pixelHeight=" + QString::number(pixelHeight, 'f', 12);

		return stringList;
	}

	bool fromStringList(const QStringList& stringList) override
	{
		if (!VisualReferenceRepresentation::fromStringList(stringList))
		{
			return false;
		}

		// mandatory fields
		return (	InitDoubleFromStringList("pixelWidth", stringList, pixelWidth)
				&&	InitDoubleFromStringList("pixelHeight", stringList, pixelHeight) );
	}

	double			pixelWidth = 0;		//!< The width of a pixel in the image (in radians). Shall be positive
	double			pixelHeight = 0;	//!< The height of a pixel in the image (in radians). Shall be positive.
};

//! Structure that stores an image that is mapped from 3D using the pinhole camera projection model.
/** From "E57 Simple API" by Stan Coleby
**/
class PinholeRepresentation : public SphericalRepresentation
{
public:
	Image2DProjection getType() override { return E57_PINHOLE; }
	static const char* GetName() { return "pinholeRepresentation"; }
	const char* getName() const override { return GetName(); }

	QStringList toStringList() const override
	{
		QStringList stringList = SphericalRepresentation::toStringList();

		stringList << "focalLength=" + QString::number(focalLength, 'f', 12);
		stringList << "principalPointX=" + QString::number(principalPointX, 'f', 12);
		stringList << "principalPointY=" + QString::number(principalPointY, 'f', 12);

		return stringList;
	}

	bool fromStringList(const QStringList& stringList) override
	{
		if (!SphericalRepresentation::fromStringList(stringList))
		{
			return false;
		}

		// mandatory fields
		return (	InitDoubleFromStringList("focalLength", stringList, focalLength)
				&&	InitDoubleFromStringList("principalPointX", stringList, principalPointX)
				&&	InitDoubleFromStringList("principalPointY", stringList, principalPointY) );
	}

	double			focalLength = 0;		//!< The camera's focal length (in meters). Shall be positive
	double			principalPointX = 0;	//!< The X coordinate in the image of the principal point, (in pixels). The principal point is the intersection of the z axis of the camera coordinate frame with the image plane.
	double			principalPointY = 0;	//!< The Y coordinate in the image of the principal point (in pixels).
};

//!Structure that stores an image that is mapped from 3D using a cylindrical projection model.
/** From "E57 Simple API" by Stan Coleby
**/
class CylindricalRepresentation : public SphericalRepresentation
{
public:
	Image2DProjection getType() override { return E57_CYLINDRICAL; }
	static const char* GetName() { return "cylindricalRepresentation"; }
	const char* getName() const override { return GetName(); }

	QStringList toStringList() const override
	{
		QStringList stringList = SphericalRepresentation::toStringList();

		stringList << "radius=" + QString::number(radius, 'f', 12);
		stringList << "principalPointY=" + QString::number(principalPointY, 'f', 12);

		return stringList;
	}

	bool fromStringList(const QStringList& stringList) override
	{
		if (!SphericalRepresentation::fromStringList(stringList))
		{
			return false;
		}

		// mandatory fields
		return (	InitDoubleFromStringList("radius", stringList, radius)
				&&	InitDoubleFromStringList("principalPointY", stringList, principalPointY) );
	}

	double			radius = 0;				//!< The closest distance from the cylindrical image surface to the center of projection (that is, the radius of the cylinder) (in meters). Shall be non-negative
	double			principalPointY = 0;	//!< The Y coordinate in the image of the principal point (in pixels). This is the intersection of the z = 0 plane with the image
};

#endif
