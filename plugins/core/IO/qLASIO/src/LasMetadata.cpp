#include "LasMetadata.h"

#include "LasDetails.h"

#include <ccPointCloud.h>

namespace LasMetadata
{
	static constexpr size_t SYSTEM_IDENTIFIER_SIZE = 32;

	//! Projection VLR
	static const char ProjectionVLR[] = "LASF_Projection";

	//! Converts a vlr to a QByteArray
	static QString ProjectionVLRToString(const laszip_vlr_struct& vlr)
	{
		if (QString(vlr.user_id) != ProjectionVLR)
		{
			ccLog::Warning("[LAS] Invalid Projection VLR");
			return {};
		}

		if (vlr.record_length_after_header < 2)
		{
			// too short
			return {};
		}

		QString wkt;
		if (vlr.record_id == 2111)
		{
			wkt = "Math Transform WKT: ";
		}
		else if (vlr.record_id == 2112)
		{
			wkt = "Coordinate System WKT: ";
		}
		else
		{
			// unhandled but silent, as this will happen a lot (for IDs 34735 to 34737 for instance)
			return {};
		}

		wkt += QByteArray(reinterpret_cast<const char*>(vlr.data), vlr.record_length_after_header).trimmed();

		return wkt;
	}

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
		pointCloud.setMetaData(LasMetadata::GLOBAL_ENCODING, header.global_encoding);

		QByteArray projectUUID;
		projectUUID.reserve(16);
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_1), 4);
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_2), 2);
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_3), 2);
		projectUUID.append(reinterpret_cast<const char*>(&header.project_ID_GUID_data_4), 8);
		assert(projectUUID.size() == 16);
		pointCloud.setMetaData(LasMetadata::PROJECT_UUID, std::move(projectUUID));

		if (header.system_identifier[0] != 0)
		{
			pointCloud.setMetaData(LasMetadata::SYSTEM_IDENTIFIER, QString::fromLatin1(header.system_identifier, SYSTEM_IDENTIFIER_SIZE));
		}

		if (header.number_of_variable_length_records > 0)
		{
			LasVlr vlrs(header);
			vlrs.extraScalarFields = std::move(extraScalarFields);
			pointCloud.setMetaData(LasMetadata::VLRS, QVariant::fromValue(vlrs));

			// specific case: save the LASF_Projection VLR has a human readable string (if possible)
			for (const laszip_vlr_struct& vlr : vlrs.vlrs)
			{
				if (QString(vlr.user_id) == ProjectionVLR)
				{
					QString proj = ProjectionVLRToString(vlr);
					if (!proj.isEmpty())
					{
						pointCloud.setMetaData("LAS.projection", proj);
					}
					break;
				}
			}
		}
	}

	bool LoadProjectUUID(const ccPointCloud& pointCloud, laszip_header& header)
	{
		if (pointCloud.hasMetaData(LasMetadata::PROJECT_UUID))
		{
			QVariant   value     = pointCloud.getMetaData(LasMetadata::PROJECT_UUID);
			QByteArray byteArray = value.toByteArray();
			if (byteArray.size() != 16)
			{
				ccLog::Warning("[LAS] Invalid project UUID meta data");
				return false;
			}

			const char* bufferData = byteArray.data();

			// 1st block (32 bits)
			header.project_ID_GUID_data_1 = *(const laszip_U32*)bufferData;
			bufferData += 4;
			// 2nd block (16 bits)
			header.project_ID_GUID_data_2 = *(const laszip_U16*)bufferData;
			bufferData += 2;
			// 3rd block (16 bits)
			header.project_ID_GUID_data_3 = *(const laszip_U16*)bufferData;
			bufferData += 2;
			// 4th block (8 * 8 bits)
			memcpy(header.project_ID_GUID_data_4, bufferData, 8);

			return true;
		}

		return false;
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

	bool LoadScaleFrom(const ccPointCloud& pointCloud, CCVector3d& scale)
	{
		bool hasScaleMetaData = false;
		scale.x               = pointCloud.getMetaData(LasMetadata::X_SCALE).toDouble(&hasScaleMetaData);
		if (hasScaleMetaData)
		{
			scale.y = pointCloud.getMetaData(LasMetadata::Y_SCALE).toDouble(&hasScaleMetaData);
			if (hasScaleMetaData)
			{
				scale.z = pointCloud.getMetaData(LasMetadata::Z_SCALE).toDouble(&hasScaleMetaData);
			}
		}
		return hasScaleMetaData;
	}

	bool LoadOffsetFrom(const ccPointCloud& pointCloud, CCVector3d& offset)
	{
		bool hasOffsetMetaData = false;
		offset.x               = pointCloud.getMetaData(LasMetadata::X_OFFSET).toDouble(&hasOffsetMetaData);
		if (hasOffsetMetaData)
		{
			offset.y = pointCloud.getMetaData(LasMetadata::Y_OFFSET).toDouble(&hasOffsetMetaData);
			if (hasOffsetMetaData)
			{
				offset.z = pointCloud.getMetaData(LasMetadata::Z_OFFSET).toDouble(&hasOffsetMetaData);
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

	bool LoadGlobalEncoding(const ccPointCloud& pointCloud, uint16_t& outGlobalEncoding)
	{
		outGlobalEncoding = 0;

		bool ok             = false;
		uint globalEncoding = pointCloud.getMetaData(LasMetadata::GLOBAL_ENCODING).toUInt(&ok);
		if (!ok)
		{
			return false;
		}

		if (globalEncoding <= 65535)
		{
			outGlobalEncoding = static_cast<uint16_t>(globalEncoding);
			return true;
		}
		else
		{
			ccLog::Warning("[LAS] Invalid global encoding value: " + QString::number(globalEncoding));
			return false;
		}
	}

} // namespace LasMetadata
