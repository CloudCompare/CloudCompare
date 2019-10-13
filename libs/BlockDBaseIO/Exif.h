#ifndef __EXIF_H
#define __EXIF_H

#include <string>

class EXIFInfo {
public:
	int parseFrom(const unsigned char *data, unsigned length);
	int parseFrom(const std::string &data);

	int parseFromEXIFSegment(const unsigned char *buf, unsigned len);

	void clear();

	char ByteAlign;                   
	std::string ImageDescription;     
	std::string Make;                 
	std::string Model;               
	unsigned short Orientation;       
	// 0: unspecified in EXIF data
	// 1: upper left of image
	// 3: lower right of image
	// 6: upper right of image
	// 8: lower left of image
	// 9: undefined
	unsigned short BitsPerSample;     // Number of bits per component
	std::string Software;             // Software used
	std::string DateTime;             // File change date and time
	std::string DateTimeOriginal;     // Original file date and time (may not exist)
	std::string DateTimeDigitized;    // Digitization date and time (may not exist)
	std::string SubSecTimeOriginal;   // Sub-second time that original picture was taken
	std::string Copyright;            // File copyright information
	double ExposureTime;              // Exposure time in seconds
	double FNumber;                   // F/stop
	unsigned short ISOSpeedRatings;   // ISO speed
	double ShutterSpeedValue;         // Shutter speed (reciprocal of exposure time)
	double ExposureBiasValue;         // Exposure bias value in EV
	double SubjectDistance;           // Distance to focus point in meters
	double FocalLength;               // Focal length of lens in millimeters
	unsigned short FocalLengthIn35mm; // Focal length in 35mm film
	char Flash;                       // 0 = no flash, 1 = flash used
	unsigned short MeteringMode;      // Metering mode
	// 1: average
	// 2: center weighted average
	// 3: spot
	// 4: multi-spot
	// 5: multi-segment
	unsigned ImageWidth;              // Image width reported in EXIF data
	unsigned ImageHeight;             // Image height reported in EXIF data
	struct Geolocation_t {            // GPS information embedded in file
		double Latitude;                  // Image latitude expressed as decimal
		double Longitude;                 // Image longitude expressed as decimal
		double Altitude;                  // Altitude in meters, relative to sea level
		char AltitudeRef;                 // 0 = above sea level, -1 = below sea level
		struct Coord_t {
			double degrees;
			double minutes;
			double seconds;
			char direction;
		} LatComponents, LonComponents;   // Latitude, Longitude expressed in deg/min/sec 
	} GeoLocation;
	EXIFInfo() {
		clear();
	}
};

// Parse was successful
#define PARSE_EXIF_SUCCESS                    0
// No JPEG markers found in buffer, possibly invalid JPEG file
#define PARSE_EXIF_ERROR_NO_JPEG              1982
// No EXIF header found in JPEG file.
#define PARSE_EXIF_ERROR_NO_EXIF              1983
// Byte alignment specified in EXIF file was unknown (not Motorola or Intel).
#define PARSE_EXIF_ERROR_UNKNOWN_BYTEALIGN    1984
// EXIF header was found, but data was corrupted.
#define PARSE_EXIF_ERROR_CORRUPT              1985

#endif
