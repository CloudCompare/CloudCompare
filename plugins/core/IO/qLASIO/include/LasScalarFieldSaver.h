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

#ifndef LASSCALARFIELDSAVER_H
#define LASSCALARFIELDSAVER_H

#include "LasDetails.h"

#include <limits>
#include <vector>

class ccPointCloud;
struct laszip_point;

/// Class with the logic to save a point clouds
/// scalar field into a LAS file.
class LasScalarFieldSaver
{
  public:
    LasScalarFieldSaver(std::vector<LasScalarField> standardFields,
                        std::vector<LasExtraScalarField> extraFields);

    void handleScalarFields(size_t i, laszip_point &point);

    void handleExtraFields(size_t i, laszip_point &point);

  private:
    template <typename T> static void WriteScalarValueAs(ScalarType value, uint8_t *dest)
    {
        if (value > std::numeric_limits<T>::max())
        {
            *reinterpret_cast<T *>(dest) = std::numeric_limits<T>::max();
        }
        else if (value < std::numeric_limits<T>::min())
        {
            *reinterpret_cast<T *>(dest) = std::numeric_limits<T>::min();
        }
        else
        {
            *reinterpret_cast<T *>(dest) = static_cast<T>(value);
        }
    }

  private:
    std::vector<LasScalarField> m_standardFields;
    std::vector<LasExtraScalarField> m_extraFields;
};

#endif // LASSCALARFIELDSAVER_H
