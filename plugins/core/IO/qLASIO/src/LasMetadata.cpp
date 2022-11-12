#include "LasMetadata.h"

#include "LasDetails.h"

#include <ccPointCloud.h>

namespace LasMetadata
{
	static constexpr size_t GUID_DATA_4_SIZE       = 8;
	static constexpr size_t SYSTEM_IDENTIFIER_SIZE = 32;

	void SaveMetadataInto(const laszip_header&                    header,
	                      ccPointCloud&                           pointCloud,
	                      const std::vector<LasExtraScalarField>& extraScalarFields)
	{
		pointCloud.setMetaData(LasMetadata::X_SCALE, header.x_scale_factor);
		pointCloud.setMetaData(LasMetadata::Y_SCALE, header.y_scale_factor);
		pointCloud.setMetaData(LasMetadata::Z_SCALE, header.z_scale_factor);

		pointCloud.setMetaData(LasMetadata::X_OFFSET, header.x_offset);
		pointCloud.setMetaData(LasMetadata::Y_OFFSET, header.y_offset);
		pointCloud.setMetaData(LasMetadata::Z_OFFSET, header.z_offset);

		pointCloud.setMetaData(LasMetadata::VERSION_MAJOR, header.version_major);
		pointCloud.setMetaData(LasMetadata::VERSION_MINOR, header.version_minor);
		pointCloud.setMetaData(LasMetadata::POINT_FORMAT, header.point_data_format);

		QByteArray projectUUID;
		projectUUID.reserve(16);
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_1), sizeof(laszip_U32));
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_2), sizeof(laszip_U16));
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_3), sizeof(laszip_U16));
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_4),
		                   sizeof(laszip_CHAR) * GUID_DATA_4_SIZE);
		assert(projectUUID.size() == 16);
		pointCloud.setMetaData(LasMetadata::PROJECT_UUID, std::move(projectUUID));

		pointCloud.setMetaData(LasMetadata::SYSTEM_IDENTIFIER,
		                       QString::fromLatin1(header.system_identifier, SYSTEM_IDENTIFIER_SIZE));

		if (header.number_of_variable_length_records > 0)
		{
			LasVlr vlrs(header);
			vlrs.extraScalarFields = std::move(extraScalarFields);
			pointCloud.setMetaData(LasMetadata::VLRS, QVariant::fromValue(vlrs));
		}
	}

	bool LoadVlrs(const ccPointCloud& pointCloud, LasVlr& vlr)
	{
		if (pointCloud.hasMetaData(LasMetadata::VLRS))
		{
			QVariant value = pointCloud.getMetaData(LasMetadata::VLRS);
			if (value.canConvert<LasVlr>())
			{
				vlr = value.value<LasVlr>();
				return true;
			}
		}
		return false;
	}

	bool LoadScalesFrom(const ccPointCloud& pointCloud, CCVector3d& scales)
	{
		bool hasScaleMetaData = false;
		scales.x              = pointCloud.getMetaData(LasMetadata::X_SCALE).toDouble(&hasScaleMetaData);
		if (hasScaleMetaData)
		{
			scales.y = pointCloud.getMetaData(LasMetadata::Y_SCALE).toDouble(&hasScaleMetaData);
			if (hasScaleMetaData)
			{
				scales.z = pointCloud.getMetaData(LasMetadata::Z_SCALE).toDouble(&hasScaleMetaData);
			}
		}
		return hasScaleMetaData;
	}

	bool LoadOffsetsFrom(const ccPointCloud& pointCloud, CCVector3d& offsets)
	{
		bool hasOffsetMetaData = false;
		offsets.x              = pointCloud.getMetaData(LasMetadata::X_OFFSET).toDouble(&hasOffsetMetaData);
		if (hasOffsetMetaData)
		{
			offsets.y = pointCloud.getMetaData(LasMetadata::Y_OFFSET).toDouble(&hasOffsetMetaData);
			if (hasOffsetMetaData)
			{
				offsets.z = pointCloud.getMetaData(LasMetadata::Z_OFFSET).toDouble(&hasOffsetMetaData);
			}
		}
		return hasOffsetMetaData;
	}

	bool LoadLasVersionFrom(const ccPointCloud& pointCloud, LasDetails::LasVersion& version)
	{
		bool ok            = false;
		int  pointFormatId = pointCloud.getMetaData(LasMetadata::POINT_FORMAT).toInt(&ok);
		if (!ok)
		{
			return false;
		}

		int versionMajor = pointCloud.getMetaData(LasMetadata::VERSION_MAJOR).toInt(&ok);
		if (!ok)
		{
			return false;
		}

		int versionMinor = pointCloud.getMetaData(LasMetadata::VERSION_MINOR).toInt(&ok);
		if (!ok)
		{
			return false;
		}
		// TODO
		// version.majorVersion = versionMajor;
		version.minorVersion = versionMinor;
		version.pointFormat  = pointFormatId;
		return true;
	}
} // namespace LasMetadata
