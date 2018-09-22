#ifndef CC_E57_HEADER_HEADER
#define CC_E57_HEADER_HEADER

#include <cstdint>

//! Point prototype (structure use to interrogate if standardized fields are available)
/** Taken from "E57 Simple API" by Stan Coleby
**/
class PointStandardizedFieldsAvailable
{
public:
	bool		cartesianXField;			//!< Indicates that the PointRecord cartesianX field is active
	bool		cartesianYField;			//!< Indicates that the PointRecord cartesianY field is active
	bool		cartesianZField;			//!< Indicates that the PointRecord cartesianZ field is active
	bool		cartesianInvalidStateField; //!< Indicates that the PointRecord cartesianInvalidState field is active

	bool		sphericalRangeField;		//!< Indicates that the PointRecord sphericalRange field is active
	bool		sphericalAzimuthField;		//!< Indicates that the PointRecord sphericalAzimuth field is active
	bool		sphericalElevationField;	//!< Indicates that the PointRecord sphericalElevation field is active
	bool		sphericalInvalidStateField; //!< Indicates that the PointRecord sphericalInvalidState field is active

	double		pointRangeMinimum;			//!< Indicates that the PointRecord cartesian and range fields should be configured with this minimum value -E57_FLOAT_MAX or -E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a minimum range value.
	double		pointRangeMaximum;			//!< Indicates that the PointRecord cartesian and range fields should be configured with this maximum value E57_FLOAT_MAX or E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a maximum range value.
	double		pointRangeScaledInteger;	//!< Indicates that the PointRecord cartesain and range fields should be configured as a ScaledIntegerNode with this scale setting. If 0. then use FloatNode.

	bool		normXField;					//!< Indicates that the PointRecord normalX field is active
	bool		normYField;					//!< Indicates that the PointRecord normalY field is active
	bool		normZField;					//!< Indicates that the PointRecord normalZ field is active
	double		normRangeMinimum;			//!< Indicates that the PointRecord normal and range fields should be configured with this minimum value -E57_FLOAT_MAX or -E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a minimum range value.
	double		normRangeMaximum;			//!< Indicates that the PointRecord normal and range fields should be configured with this maximum value E57_FLOAT_MAX or E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a maximum range value.
	double		normRangeScaledInteger;		//!< Indicates that the PointRecord normal and range fields should be configured as a ScaledIntegerNode with this scale setting. If 0. then use FloatNode.

	double		angleMinimum;				//!< Indicates that the PointRecord angle fields should be configured with this minimum value -E57_FLOAT_MAX or -E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a minimum angle value.
	double		angleMaximum;				//!< Indicates that the PointRecord angle fields should be configured with this maximum value E57_FLOAT_MAX or E57_DOUBLE_MAX. If using a ScaledIntegerNode then this needs to be a maximum angle value.
	double		angleScaledInteger;			//!< Indicates that the PointRecord angle fields should be configured as a ScaledIntegerNode with this scale setting. If 0. then use FloatNode.

	bool		rowIndexField;				//!< Indicates that the PointRecord rowIndex field is active
	uint32_t	rowIndexMaximum;			//!< Indicates that the PointRecord index fields should be configured with this maximum value where the minimum will be set to 0.
	bool		columnIndexField;			//!< Indicates that the PointRecord columnIndex field is active
	uint32_t	columnIndexMaximum;			//!< Indicates that the PointRecord index fields should be configured with this maximum value where the minimum will be set to 0.

	bool		returnIndexField;			//!< Indicates that the PointRecord returnIndex field is active
	bool		returnCountField;			//!< Indicates that the PointRecord returnCount field is active
	uint8_t		returnMaximum;				//!< Indicates that the PointRecord return fields should be configured with this maximum value where the minimum will be set to 0.

	bool		timeStampField;				//!< Indicates that the PointRecord timeStamp field is active
	bool		isTimeStampInvalidField;	//!< Indicates that the PointRecord isTimeStampInvalid field is active
	double		timeMaximum;				//!< Indicates that the PointRecord timeStamp fields should be configured with this maximum value. like E57_UINT32_MAX, E57_FLOAT_MAX or E57_DOUBLE_MAX

	bool		intensityField;				//!< Indicates that the PointRecord intensity field is active
	bool		isIntensityInvalidField;	//!< Indicates that the PointRecord isIntensityInvalid field is active
	double		intensityScaledInteger;		//!< Indicates that the PointRecord intensity fields should be configured as a ScaledIntegerNode with this setting. If 0. then use FloatNode, if -1. use IntegerNode

	bool		colorRedField;				//!< indicates that the PointRecord colorRed field is active
	bool		colorGreenField;			//!< indicates that the PointRecord colorGreen field is active
	bool		colorBlueField;				//!< indicates that the PointRecord colorBlue field is active
	bool		isColorInvalidField;		//!< Indicates that the PointRecord isColorInvalid field is active
};

//! Specifies the limits for the value of signal intensity that a sensor is capable of producing
/** From "E57 Simple API" by Stan Coleby
**/
class IntensityLimits
{
public:
	double		intensityMinimum;		//!< The minimum producible intensity value. Unit is unspecified.
	double		intensityMaximum;		//!< The maximum producible intensity value. Unit is unspecified.
};

//! Secifies the limits for the value of red, green, and blue color that a sensor is capable of producing.
/** From "E57 Simple API" by Stan Coleby
**/
class ColorLimits
{
public:
	double		colorRedMinimum;		//!< The minimum producible red color value. Unit is unspecified.
	double		colorRedMaximum;		//!< The maximum producible red color value. Unit is unspecified.
	double		colorGreenMinimum;		//!< The minimum producible green color value. Unit is unspecified.
	double		colorGreenMaximum;		//!< The maximum producible green color value. Unit is unspecified.
	double		colorBlueMinimum;		//!< The minimum producible blue color value. Unit is unspecified.
	double		colorBlueMaximum;		//!< The maximum producible blue color value. Unit is unspecified.
};

//! E57 scan useful header information (minimal set for CloudCompare only ;)
struct E57ScanHeader
{
	IntensityLimits						intensityLimits;	//!< The limits for the value of signal intensity that the sensor is capable of producing.
	ColorLimits							colorLimits;		//!< The limits for the value of red, green, and blue color that the sensor is capable of producing.
	PointStandardizedFieldsAvailable	pointFields;		//!< This defines the active fields used in the WritePoints function.
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
};

//! Structure that stores an image that is to be used only as a visual reference.
/** From "E57 Simple API" by Stan Coleby
**/
class VisualReferenceRepresentation : public CameraRepresentation
{
public:
	Image2DProjection getType() override { return E57_VISUAL; }

	Image2DType	imageType;		//!< image type.
	int64_t		imageSize;		//!< size of image data in BlobNode.
	int64_t		imageMaskSize;	//!< size of image mask data in BlobNode (if any).
	int32_t		imageWidth;		//!< image width (in pixels). Shall be positive
	int32_t		imageHeight;	//!< image height (in pixels). Shall be positive
};

//! Structure that stores an image that is mapped from 3D using a spherical projection model
/** From "E57 Simple API" by Stan Coleby
**/
class SphericalRepresentation : public VisualReferenceRepresentation
{
public:
	Image2DProjection getType() override { return E57_SPHERICAL; }

	double			pixelWidth;		//!< The width of a pixel in the image (in radians). Shall be positive
	double			pixelHeight;	//!< The height of a pixel in the image (in radians). Shall be positive.
};

//! Structure that stores an image that is mapped from 3D using the pinhole camera projection model.
/** From "E57 Simple API" by Stan Coleby
**/
class PinholeRepresentation : public SphericalRepresentation
{
public:
	Image2DProjection getType() override { return E57_PINHOLE; }

	double			focalLength;	//!< The camera's focal length (in meters). Shall be positive
	double			principalPointX;//!< The X coordinate in the image of the principal point, (in pixels). The principal point is the intersection of the z axis of the camera coordinate frame with the image plane.
	double			principalPointY;//!< The Y coordinate in the image of the principal point (in pixels).
};

//!Structure that stores an image that is mapped from 3D using a cylindrical projection model.
/** From "E57 Simple API" by Stan Coleby
**/
class CylindricalRepresentation : public SphericalRepresentation
{
public:
	Image2DProjection getType() override { return E57_CYLINDRICAL; }

	double			radius;			//!< The closest distance from the cylindrical image surface to the center of projection (that is, the radius of the cylinder) (in meters). Shall be non-negative
	double			principalPointY;//!< The Y coordinate in the image of the principal point (in pixels). This is the intersection of the z = 0 plane with the image
};

#endif
