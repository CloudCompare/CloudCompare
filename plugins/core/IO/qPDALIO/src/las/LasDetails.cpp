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
#include "LasDetails.h"

#include "ccPointCloud.h"
#include "ccScalarField.h"

#include "LasFields.h"

LasVersion SelectBestVersion(const ccPointCloud &cloud) {
  bool isExtendedRequired{false};
  // These are exclusive to 'extended' point formats (>= 6)
  bool hasOverlapFlag =
      cloud.getScalarFieldIndexByName(LasNames::OverlapFlag) != -1;
  bool hasNIR = cloud.getScalarFieldIndexByName(LasNames::NearInfrared) != -1;
  bool hasScannerChannel =
      cloud.getScalarFieldIndexByName(LasNames::ScannerChannel) != -1;
  bool hasScanAngle =
      cloud.getScalarFieldIndexByName(LasNames::ScanAngle) != -1;

  isExtendedRequired =
      hasOverlapFlag || hasNIR || hasScannerChannel || hasScanAngle;

  if (!isExtendedRequired) {
    // We may need extended point format because some fields need to store more
    // value than what is possible on non-extended point format.
    int classificationIdx =
        cloud.getScalarFieldIndexByName(LasNames::Classification);
    if (classificationIdx != -1) {
      const CCCoreLib::ScalarField *classification =
          cloud.getScalarField(classificationIdx);
      if (classification->getMax() >
          LasField::ValueRange(LasField::Classification).max) {
        isExtendedRequired = true;
      }
    }

    int returnNumberIdx =
        cloud.getScalarFieldIndexByName(LasNames::ReturnNumber);
    if (returnNumberIdx != -1) {
      const CCCoreLib::ScalarField *returnNumber =
          cloud.getScalarField(returnNumberIdx);
      if (returnNumber->getMax() >
          LasField::ValueRange(LasField::Id::ReturnNumber).max) {
        isExtendedRequired = true;
      }
    }

    int numReturnsIdx =
        cloud.getScalarFieldIndexByName(LasNames::NumberOfReturns);
    if (numReturnsIdx != -1) {
      const CCCoreLib::ScalarField *numReturns =
          cloud.getScalarField(numReturnsIdx);
      if (numReturns->getMax() >
          LasField::ValueRange(LasField::Id::NumberOfReturns).max) {
        isExtendedRequired = true;
      }
    }
  }

  bool hasRGB = cloud.hasColors();
  bool hasWaveform = cloud.hasFWF();

  if (isExtendedRequired) {
    int pointFormat = 6;
    if (hasWaveform) {
      if (hasRGB) {
        pointFormat = 10;
      } else {
        pointFormat = 9;
      }
    } else {
      if (hasRGB && hasNIR) {
        pointFormat = 8;
      } else if (hasRGB && !hasNIR) {
        pointFormat = 7;
      }
    }
    return {pointFormat, 4};
  } else {
    bool hasGpsTime = cloud.getScalarFieldIndexByName(LasNames::GpsTime) != -1;
    int pointFormat = 0;
    if (hasWaveform) {
      if (hasGpsTime) {
        pointFormat = 4;
      } else if (hasRGB) {
        pointFormat = 5;
      }
    } else {
      if (hasGpsTime) {
        pointFormat += 1;
      }
      if (hasRGB) {
        pointFormat += 2;
      }
    }
    return {pointFormat, 2};
  }
}