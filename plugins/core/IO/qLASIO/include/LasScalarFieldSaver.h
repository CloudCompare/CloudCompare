#pragma once

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
#include "LasExtraScalarField.h"
#include "LasScalarField.h"

struct laszip_point;

/// Class with the logic to save a point clouds
/// scalar field into a LAS file.
class LasScalarFieldSaver
{
  public:
	LasScalarFieldSaver() = default;

	/// Creates a new LasScalarFieldSaver that will save the given
	/// standard LAS fields and the LAS extra scalar fields
	LasScalarFieldSaver(std::vector<LasScalarField>&&      standardFields,
	                    std::vector<LasExtraScalarField>&& extraFields);

	inline void setStandarFields(std::vector<LasScalarField>&& standardFields)
	{
		m_standardFields = standardFields;
	}

	inline void setExtraFields(std::vector<LasExtraScalarField>&& extraFields)
	{
		m_extraFields = extraFields;
	}

	inline const std::vector<LasExtraScalarField>& extraFields() const
	{
		return m_extraFields;
	}

	/// Saves the scalar fields values for pointIndex into the given laszip_point
	void handleScalarFields(size_t pointIndex, laszip_point& point);

	/// Saves the extra scalar fields values for pointIndex into the given laszip_point
	void handleExtraFields(size_t pointIndex, laszip_point& point);

  private:
	template <typename T>
	static void WriteScalarValueAs(ScalarType value, uint8_t* dest)
	{
		if (value > std::numeric_limits<T>::max())
		{
			*reinterpret_cast<T*>(dest) = std::numeric_limits<T>::max();
		}
		else if (value < std::numeric_limits<T>::lowest())
		{
			*reinterpret_cast<T*>(dest) = std::numeric_limits<T>::lowest();
		}
		else
		{
			*reinterpret_cast<T*>(dest) = static_cast<T>(value);
		}
	}

  private:
	std::vector<LasScalarField>      m_standardFields;
	std::vector<LasExtraScalarField> m_extraFields;
};
