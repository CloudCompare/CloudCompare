//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
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
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################
#ifndef LASDETAILS_H
#define LASDETAILS_H

#include <CCTypes.h>
#include <QtGlobal>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

class ccPointCloud;
/// This namespace regroups constants for all the names we use
/// in CloudCompare ScalarField system for the standard dimensions defined by
/// the LAS Spec.
///
/// Notice that RGB and Waveforms are missing, that is normal as they
/// are not treated as scalar fields within CloudCompare.
namespace LasNames {
constexpr const char *Intensity = "Intensity";
constexpr const char *ReturnNumber = "Return Number";
constexpr const char *NumberOfReturns = "Number Of Returns";
constexpr const char *ScanDirectionFlag = "Scan Direction Flag";
constexpr const char *EdgeOfFlightLine = "Edge Of Flight Line";
constexpr const char *Classification = "Classification";
constexpr const char *SyntheticFlag = "Synthetic Flag";
constexpr const char *KeypointFlag = "Keypoint Flag";
constexpr const char *WithheldFlag = "Withheld Flag";
constexpr const char *ScanAngleRank = "Scan Angle";
constexpr const char *UserData = "User Data";
constexpr const char *PointSourceId = "Point Source ID";
constexpr const char *GpsTime = "Gps Time";

// 1.4 point format 6 stuff
constexpr const char *ScanAngle = "Scan Angle";
constexpr const char *ScannerChannel = "Scanner Channel";
constexpr const char *OverlapFlag = "Overlap Flag";
constexpr const char *NearInfrared = "Near Infrared";
} // namespace LasNames

/// Returns whether the point format supports Gps Time
constexpr bool HasGpsTime(unsigned int pointFormatId) {
  return pointFormatId == 1 || pointFormatId == 3 || pointFormatId == 5 ||
         pointFormatId >= 6;
}

/// Returns whether the point format supports RGB
constexpr bool HasRGB(unsigned int pointFormatId) {
  return pointFormatId == 2 || pointFormatId == 3 || pointFormatId == 4 ||
         pointFormatId == 5 || pointFormatId >= 7;
}

/// Returns whether the point format supports Waveforms
constexpr bool HasWaveform(unsigned int pointFormatId) {
  return pointFormatId == 4 || pointFormatId == 5 || pointFormatId >= 8;
}

/// Returns whether the point format support Near Infrared
constexpr bool HasNearInfrared(unsigned int pointFormatId) {
  return pointFormatId == 8 || pointFormatId == 10;
}

constexpr bool isPointFormatExtended(unsigned int pointFormat) {
  return pointFormat >= 6;
}
struct LasVersion {
  int pointFormat = 3;
  int minorVersion = 2;
};

LasVersion SelectBestVersion(const ccPointCloud &cloud);

#endif // LASDETAILS_H
