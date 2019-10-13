#include <stdio.h>
#include <algorithm>
#include "exif.h"

using std::string;

namespace {
  // IF Entry 
  struct IFEntry {
    // Raw fields
    unsigned short tag;
    unsigned short format;
    unsigned data;
    unsigned length;
    
    // Parsed fields
    string val_string;
    unsigned short val_16;
    unsigned val_32;
    double val_rational;
    unsigned char val_byte;
  };

  // Helper functions
  unsigned int parse32(const unsigned char *buf, bool intel) {
    if (intel) 
      return ((unsigned)buf[3]<<24) | ((unsigned)buf[2]<<16) | ((unsigned)buf[1]<<8)  |  buf[0];
    return ((unsigned)buf[0]<<24) |  ((unsigned)buf[1]<<16) |  ((unsigned)buf[2]<<8)  |   buf[3];
  }

  unsigned short parse16(const unsigned char *buf, bool intel) {
    if (intel)
      return ((unsigned) buf[1]<<8) | buf[0];
    return ((unsigned) buf[0]<<8) | buf[1]; 
  }

  string parseEXIFString(const unsigned char *buf, const unsigned num_components, const unsigned data, const unsigned base, const unsigned len) {
    string value;
    if (num_components <= 4)
      value.assign( (const char*)&data, num_components );
    else {
      if (base+data+num_components <= len)
        value.assign( (const char*)(buf+base+data), num_components );
    }
    return value;
  }

  double parseEXIFRational(const unsigned char *buf, bool intel) {
    double numerator   = 0;
    double denominator = 1;

    numerator  = (double) parse32(buf, intel);
    denominator= (double) parse32(buf+4, intel);
    if(denominator < 1e-20)
      return 0;
    return numerator/denominator;
  }

  IFEntry parseIFEntry(const unsigned char *buf, const unsigned offs, const bool alignIntel, const unsigned base, const unsigned len) {
    IFEntry result;
    // Each directory entry is composed of:
    // 2 bytes: tag number (data field)
    // 2 bytes: data format
    // 4 bytes: number of components
    // 4 bytes: data value or offset to data value
    result.tag        = parse16(buf + offs, alignIntel);
    result.format     = parse16(buf + offs + 2, alignIntel);
    result.length     = parse32(buf + offs + 4, alignIntel);
    result.data       = parse32(buf + offs + 8, alignIntel);

    // Parse value in specified format
    switch (result.format) {
      case 1:
        result.val_byte = (unsigned char) *(buf + offs + 8);
        break;
      case 2:
        result.val_string = parseEXIFString(buf, result.length, result.data, base, len);
        break;
      case 3:
        result.val_16 = parse16((const unsigned char *) buf + offs + 8, alignIntel);
        break;
      case 4:
        result.val_32 = result.data;
        break;
      case 5:
        if (base + result.data + 8 <= len)
          result.val_rational = parseEXIFRational(buf + base + result.data, alignIntel);
        break;
      case 7:
      case 9:
      case 10:
        break;
      default:
        result.tag = 0xFF;
    }
    return result;
  }
}

//
// Locates the EXIF segment and parses it using parseFromEXIFSegment 
//
int EXIFInfo::parseFrom(const unsigned char *buf, unsigned len) {

  if (!buf || len < 4)
    return PARSE_EXIF_ERROR_NO_EXIF;
  if (buf[0] != 0xFF || buf[1] != 0xD8)
    return PARSE_EXIF_ERROR_NO_JPEG;
  if (buf[len-2] != 0xFF || buf[len-1] != 0xD9)
    return PARSE_EXIF_ERROR_NO_JPEG;
  clear();

  unsigned offs = 0;        // current offset into buffer
  for (offs = 0; offs < len-1; offs++) 
    if (buf[offs] == 0xFF && buf[offs+1] == 0xE1) 
      break;
  if (offs + 4 > len)
    return PARSE_EXIF_ERROR_NO_EXIF;
  offs += 2;
  unsigned short section_length = parse16(buf + offs, false); 
  if (offs + section_length > len || section_length < 16)
    return PARSE_EXIF_ERROR_CORRUPT;
  offs += 2;

  return parseFromEXIFSegment(buf + offs, len - offs);
}

int EXIFInfo::parseFrom(const string &data) {
  return parseFrom((const unsigned char *)data.data(), data.length());
}

int EXIFInfo::parseFromEXIFSegment(const unsigned char *buf, unsigned len) {
  bool alignIntel = true;     // byte alignment (defined in EXIF header)
  unsigned offs   = 0;        // current offset into buffer
  if (!buf || len < 6)
    return PARSE_EXIF_ERROR_NO_EXIF;

  if (!std::equal(buf, buf+6, "Exif\0\0"))
    return PARSE_EXIF_ERROR_NO_EXIF;
  offs += 6;

  if (offs + 8 > len)
    return PARSE_EXIF_ERROR_CORRUPT;
  unsigned tiff_header_start = offs;
  if (buf[offs] == 'I' && buf[offs+1] == 'I')
    alignIntel = true;
  else {
    if(buf[offs] == 'M' && buf[offs+1] == 'M')
      alignIntel = false;
    else 
      return PARSE_EXIF_ERROR_UNKNOWN_BYTEALIGN;
  }
  this->ByteAlign = alignIntel;
  offs += 2;
  if (0x2a != parse16(buf+offs, alignIntel))
    return PARSE_EXIF_ERROR_CORRUPT;
  offs += 2;
  unsigned first_ifd_offset = parse32(buf + offs, alignIntel);
  offs += first_ifd_offset - 4;
  if (offs >= len)
    return PARSE_EXIF_ERROR_CORRUPT;

  if (offs + 2 > len)
    return PARSE_EXIF_ERROR_CORRUPT;
  int num_entries = parse16(buf + offs, alignIntel);
  if (offs + 6 + 12 * num_entries > len)
    return PARSE_EXIF_ERROR_CORRUPT;
  offs += 2;
  unsigned exif_sub_ifd_offset = len;
  unsigned gps_sub_ifd_offset  = len;
  while (--num_entries >= 0) {
    IFEntry result = parseIFEntry(buf, offs, alignIntel, tiff_header_start, len);
    offs += 12;
    switch(result.tag) {
      case 0x102:
        // Bits per sample
        if (result.format == 3)
          this->BitsPerSample = result.val_16;
        break;

      case 0x10E:
        // Image description
        if (result.format == 2)
          this->ImageDescription = result.val_string;
        break;

      case 0x10F:
        // Digicam make
        if (result.format == 2)
          this->Make = result.val_string;
        break;

      case 0x110:
        // Digicam model
        if (result.format == 2)
          this->Model = result.val_string;
        break;

      case 0x112:
        // Orientation of image
        if (result.format == 3)
          this->Orientation = result.val_16;
        break;

      case 0x131:
        // Software used for image
        if (result.format == 2)
          this->Software = result.val_string;
        break;

      case 0x132:
        // EXIF/TIFF date/time of image modification
        if (result.format == 2)
          this->DateTime = result.val_string;
        break;

      case 0x8298:
        // Copyright information
        if (result.format == 2)
          this->Copyright = result.val_string;
        break;

      case 0x8825:
        // GPS IFS offset
        gps_sub_ifd_offset = tiff_header_start + result.data;
        break;

      case 0x8769:
        // EXIF SubIFD offset
        exif_sub_ifd_offset = tiff_header_start + result.data;
        break;
    }
  }

  if (exif_sub_ifd_offset + 4 <= len) {
    offs = exif_sub_ifd_offset;
    int num_entries = parse16(buf + offs, alignIntel);
    if (offs + 6 + 12 * num_entries > len)
      return PARSE_EXIF_ERROR_CORRUPT;
    offs += 2;
    while (--num_entries >= 0) {
      IFEntry result = parseIFEntry(buf, offs, alignIntel, tiff_header_start, len);
      switch(result.tag) {
        case 0x829a:
          // Exposure time in seconds
          if (result.format == 5)
            this->ExposureTime = result.val_rational;
          break;

        case 0x829d:
          // FNumber
          if (result.format == 5)
            this->FNumber = result.val_rational;
          break;

        case 0x8827:
          // ISO Speed Rating
          if (result.format == 3)
            this->ISOSpeedRatings = result.val_16;
          break;

        case 0x9003:
          // Original date and time
          if (result.format == 2)
            this->DateTimeOriginal = result.val_string;
          break;

        case 0x9004:
          // Digitization date and time
          if (result.format == 2)
            this->DateTimeDigitized = result.val_string;
          break;

        case 0x9201:
          // Shutter speed value
          if (result.format == 5)
            this->ShutterSpeedValue = result.val_rational;
          break;

        case 0x9204:
          // Exposure bias value 
          if (result.format == 5)
            this->ExposureBiasValue = result.val_rational;
          break;

        case 0x9206:
          // Subject distance
          if (result.format == 5)
            this->SubjectDistance = result.val_rational;
          break;

        case 0x9209:
          // Flash used
          if (result.format == 3)
            this->Flash = result.data ? 1 : 0;
          break;

        case 0x920a:
          // Focal length
          if (result.format == 5)
            this->FocalLength = result.val_rational;
          break;

        case 0x9207:
          // Metering mode
          if (result.format == 3)
            this->MeteringMode = result.val_16;
          break;

        case 0x9291:
          // Subsecond original time
          if (result.format == 2)
            this->SubSecTimeOriginal = result.val_string;
          break;

        case 0xa002:
          // EXIF Image width
          if (result.format == 4)
            this->ImageWidth = result.val_32;
          if (result.format == 3)
            this->ImageWidth = result.val_16;
          break;

        case 0xa003:
          // EXIF Image height
          if (result.format == 4)
            this->ImageHeight = result.val_32;
          if (result.format == 3)
            this->ImageHeight = result.val_16;
          break;

        case 0xa405:
          // Focal length in 35mm film
          if (result.format == 3)
            this->FocalLengthIn35mm = result.val_16;
          break;
      }
      offs += 12;
    }
  }

  if (gps_sub_ifd_offset + 4 <= len) {
    offs = gps_sub_ifd_offset;
    int num_entries = parse16(buf + offs, alignIntel);
    if (offs + 6 + 12 * num_entries > len)
      return PARSE_EXIF_ERROR_CORRUPT;
    offs += 2;
    while (--num_entries >= 0) {
      unsigned short tag    = parse16(buf + offs, alignIntel);
      unsigned short format = parse16(buf + offs + 2, alignIntel);
      unsigned length       = parse32(buf + offs + 4, alignIntel);
      unsigned data         = parse32(buf + offs + 8, alignIntel);
      switch(tag) {
        case 1:
          // GPS north or south
          this->GeoLocation.LatComponents.direction = *(buf + offs + 8);
          if ('S' == this->GeoLocation.LatComponents.direction)
            this->GeoLocation.Latitude = -this->GeoLocation.Latitude;
          break;

        case 2:
          // GPS latitude
          if (format == 5 && length == 3) {
            this->GeoLocation.LatComponents.degrees = 
              parseEXIFRational(buf + data + tiff_header_start, alignIntel);
            this->GeoLocation.LatComponents.minutes = 
              parseEXIFRational(buf + data + tiff_header_start + 8, alignIntel);
            this->GeoLocation.LatComponents.seconds = 
              parseEXIFRational(buf + data + tiff_header_start + 16, alignIntel);
            this->GeoLocation.Latitude = 
              this->GeoLocation.LatComponents.degrees +
              this->GeoLocation.LatComponents.minutes / 60 +
              this->GeoLocation.LatComponents.seconds / 3600;
            if ('S' == this->GeoLocation.LatComponents.direction)
              this->GeoLocation.Latitude = -this->GeoLocation.Latitude;
          }
          break;

        case 3:
          // GPS east or west
          this->GeoLocation.LonComponents.direction = *(buf + offs + 8);
          if ('W' == this->GeoLocation.LonComponents.direction)
            this->GeoLocation.Longitude = -this->GeoLocation.Longitude;
          break;

        case 4:
          if (format == 5 && length == 3) {
            this->GeoLocation.LonComponents.degrees = 
              parseEXIFRational(buf + data + tiff_header_start, alignIntel);
            this->GeoLocation.LonComponents.minutes = 
              parseEXIFRational(buf + data + tiff_header_start + 8, alignIntel);
            this->GeoLocation.LonComponents.seconds = 
              parseEXIFRational(buf + data + tiff_header_start + 16, alignIntel);
            this->GeoLocation.Longitude = 
              this->GeoLocation.LonComponents.degrees +
              this->GeoLocation.LonComponents.minutes / 60 +
              this->GeoLocation.LonComponents.seconds / 3600;
            if ('W' == this->GeoLocation.LonComponents.direction)
              this->GeoLocation.Longitude = -this->GeoLocation.Longitude;
          }
          break;

        case 5:
          this->GeoLocation.AltitudeRef = *(buf + offs + 8);
          if (1 == this->GeoLocation.AltitudeRef)
            this->GeoLocation.Altitude = -this->GeoLocation.Altitude;
          break;

        case 6:
          if (format == 5) {
            this->GeoLocation.Altitude = 
              parseEXIFRational(buf + data + tiff_header_start, alignIntel);
            if (1 == this->GeoLocation.AltitudeRef)
              this->GeoLocation.Altitude = -this->GeoLocation.Altitude;
          }
          break;
      }
      offs += 12;
    } 
  }

  return PARSE_EXIF_SUCCESS;
}

void EXIFInfo::clear() {
  // Strings
  ImageDescription  = "";
  Make              = "";
  Model             = "";
  Software          = "";
  DateTime          = "";
  DateTimeOriginal  = "";
  DateTimeDigitized = ""; 
  SubSecTimeOriginal= "";
  Copyright         = "";

  // Shorts / unsigned / double
  ByteAlign         = 0;
  Orientation       = 0; 

  BitsPerSample     = 0;
  ExposureTime      = 0;
  FNumber           = 0;
  ISOSpeedRatings   = 0;
  ShutterSpeedValue = 0;
  ExposureBiasValue = 0;
  SubjectDistance   = 0;
  FocalLength       = 0;
  FocalLengthIn35mm = 0;
  Flash             = 0;
  MeteringMode      = 0;
  ImageWidth        = 0;
  ImageHeight       = 0;

  // Geolocation
  GeoLocation.Latitude    = 0;
  GeoLocation.Longitude   = 0;
  GeoLocation.Altitude    = 0;
  GeoLocation.AltitudeRef = 0;
  GeoLocation.LatComponents.degrees   = 0;
  GeoLocation.LatComponents.minutes   = 0;
  GeoLocation.LatComponents.seconds   = 0;
  GeoLocation.LatComponents.direction = 0;
  GeoLocation.LonComponents.degrees   = 0;
  GeoLocation.LonComponents.minutes   = 0;
  GeoLocation.LonComponents.seconds   = 0;
  GeoLocation.LonComponents.direction = 0;
}