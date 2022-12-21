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

#include "FileIOFilter.h"
#include "LasDetails.h"
#include "LasExtraScalarField.h"
#include "LasOpenDialog.h"

// System
#include <memory>

class LasIOFilter : public FileIOFilter
{
  public:
	LasIOFilter();

	// Inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& fileName, ccHObject& container, LoadParameters& parameters) override;
	bool          canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

  private:
	struct FileInfo
	{
		LasDetails::LasVersion           version;
		std::vector<LasExtraScalarField> extraScalarFields;

		bool operator==(const FileInfo& other)
		{
			bool versionIsSame = version.minorVersion == other.version.minorVersion && version.pointFormat == other.version.pointFormat;
			if (!versionIsSame)
			{
				return false;
			}

			if (extraScalarFields.size() != other.extraScalarFields.size())
			{
				return false;
			}

			for (size_t i = 0; i < extraScalarFields.size(); ++i)
			{
				const auto& lhs = extraScalarFields[i];
				const auto& rhs = other.extraScalarFields[i];

				if (strncmp(lhs.name, rhs.name, LasExtraScalarField::MAX_NAME_SIZE) != 0
				    || lhs.type != rhs.type
				    || lhs.byteOffset != rhs.byteOffset
				    || lhs.numElements() != rhs.numElements()
				    || lhs.options != rhs.options)
				{
					return false;
				}
			}
			return true;
		}

		bool operator!=(const FileInfo& other)
		{
			return !(*this == other);
		}
	};

	std::unique_ptr<FileInfo> m_infoOfLastOpened;
	LasOpenDialog             m_openDialog{};
};
