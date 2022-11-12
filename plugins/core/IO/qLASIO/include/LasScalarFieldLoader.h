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

#ifndef LASSCALARFIELDLOADER_H
#define LASSCALARFIELDLOADER_H

#include "LasDetails.h"

#include <FileIOFilter.h>
#include <QFileInfo>

#include <vector>

#include <ccPointCloud.h>
#include <laszip/laszip_api.h>

/// Class with the logic on how to load LAS dimensions values
/// from the LAS file into a ccPointCloud's scalar field.
/// 
/// This also handle LAS extra scalar fields, as well as RGB.
class LasScalarFieldLoader
{
  public:
    LasScalarFieldLoader(std::vector<LasScalarField> standardScalarFields,
                         std::vector<LasExtraScalarField> extraScalarFields,
                         ccPointCloud &pointCloud);

    CC_FILE_ERROR handleScalarFields(ccPointCloud &pointCloud, const laszip_point &currentPoint);

    CC_FILE_ERROR handleRGBValue(ccPointCloud &pointCloud, const laszip_point &currentPoint);

    CC_FILE_ERROR handleExtraScalarFields(ccPointCloud &pointCloud, const laszip_point &currentPoint);

    const std::vector<LasScalarField> &standardFields() const
    {
        return m_standardFields;
    }

  private:

    /// Handles loading of LAS value into the scalar field that will be part
    /// of the pointCloud.
    ///
    /// sfInfo: Info about the current scalar field we are loading the value into
    /// pointCloud: The point cloud where the scalar field will be loaded into
    /// currentValue: The current value of the LAS field we are loading.
    template <typename T>
    CC_FILE_ERROR handleScalarField(LasScalarField &sfInfo, ccPointCloud &pointCloud, T currentValue);

    /// Same thing as `handleScalarField` but for RGB.
    ///
    /// In LAS files, the red, green and blue channels are normal LAS fiels,
    /// however in CloudCompare RGB is handled differently.
    CC_FILE_ERROR handleGpsTime(LasScalarField &sfInfo, ccPointCloud &pointCloud, double currentValue);

    /// creates the ccScalarFields that correspond to the LAS extra dimensions
    bool createScalarFieldsForExtraBytes(ccPointCloud &pointCloud);

    /// Re-interprets the bytes from the source as a value of type `T`
    /// and then cast the resulting value from the type T to ScalarType.
    template <typename T> static ScalarType ParseValueOfType(uint8_t *source);

    /// Re-interprets the bytes from the source as a value of type `T`
    /// and then cast the resulting value from the type T to a value of type `V`.
    template <typename T, typename V> static V ParseValueOfTypeAs(const uint8_t *source);

    /// Loads the values for the LAS extra field of the current point from the dataStart source.
    ///
    /// The loaded values are stored into the member variable `rawValues`
    void parseRawValues(const LasExtraScalarField &extraField, uint8_t *dataStart);

    template <typename T> void handleOptionsFor(const LasExtraScalarField &extraField, T values[3]);

  private:
    unsigned char colorCompShift{0};
    std::vector<LasScalarField> m_standardFields{};
    std::vector<LasExtraScalarField> m_extraScalarFields{};

    union
    {
        uint64_t unsignedValues[3];
        int64_t signedValues[3];
        double floatingValues[3];
    } rawValues{};
};

#endif // LASSCALARFIELDLOADER_H
