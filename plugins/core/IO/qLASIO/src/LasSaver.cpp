#include "LasSaver.h"

#include "LasExtraScalarField.h"
#include "LasMetadata.h"

// Qt
#include <QDate>
// qCC_db
#include <ccPointCloud.h>

constexpr const char* const CC_NORMAL_NAMES[3]{"Nx", "Ny", "Nz"};

LasSaver::LasSaver(ccPointCloud& cloud, Parameters parameters)
    : m_cloudToSave(cloud)
{
	// restore the global encoding (if any) - must be done before calling initLaszipHeader
	LasMetadata::LoadGlobalEncoding(cloud, m_laszipHeader.global_encoding);
	// restore the project UUID (if any)
	LasMetadata::LoadProjectUUID(cloud, m_laszipHeader);

	if (parameters.shouldSaveNormalsAsExtraScalarField && cloud.hasNormals())
	{
		// We export normals to extra scalar fields,
		// the easiest way to integrate that into the LasScalarFieldSaver system
		// is to temporarily export normals to scalar fields.
		m_originallySelectedScalarField = cloud.getCurrentDisplayedScalarFieldIndex();
		bool exportOptions[3]{false, false, false}; // Export all
		for (size_t i = 0; i < 3; ++i)
		{
			int idx          = cloud.getScalarFieldIndexByName(CC_NORMAL_NAMES[i]);
			exportOptions[i] = (idx == -1); // Only export if not already exported
		}

		bool needsToExportAtLeastOne = (exportOptions[0] | exportOptions[1] | exportOptions[2]);
		if (needsToExportAtLeastOne && !cloud.exportNormalToSF(exportOptions))
		{
			throw std::runtime_error("Failed to export normals to SF");
		}
		constexpr const char* const exportedNormalNames[3]{"NormalX", "NormalY", "NormalZ"};

		for (size_t i = 0; i < 3; ++i)
		{
			int idx = cloud.getScalarFieldIndexByName(CC_NORMAL_NAMES[i]);
			assert(idx != -1);
			auto* sf = dynamic_cast<ccScalarField*>(cloud.getScalarField(idx));
			assert(sf != nullptr);

			LasExtraScalarField field;
			strncpy(field.name, exportedNormalNames[i], LasExtraScalarField::MAX_NAME_SIZE);
			field.type            = LasExtraScalarField::DataType::f64;
			field.dimensions      = LasExtraScalarField::DimensionSize::One;
			field.scalarFields[0] = sf;

			parameters.extraFields.push_back(field);
			m_normalDimWasTemporarillyExported[i] = exportOptions[i];
		}
	}

	initLaszipHeader(parameters);

	LasExtraScalarField::UpdateByteOffsets(parameters.extraFields);
	unsigned totalExtraByteSize = LasExtraScalarField::TotalExtraBytesSize(parameters.extraFields);

	if (totalExtraByteSize > 0)
	{
		m_laszipHeader.point_data_record_length += totalExtraByteSize;

		size_t      newNumVlrs = m_laszipHeader.number_of_variable_length_records + 1;
		laszip_vlr* vlrs       = new laszip_vlr_struct[newNumVlrs];
		// Move already existing vlrs
		for (size_t i = 0; i < m_laszipHeader.number_of_variable_length_records; i++)
		{
			vlrs[i] = m_laszipHeader.vlrs[i];
		}
		LasExtraScalarField::InitExtraBytesVlr(vlrs[newNumVlrs - 1], parameters.extraFields);

		delete m_laszipHeader.vlrs;
		m_laszipHeader.vlrs                              = vlrs;
		m_laszipHeader.number_of_variable_length_records = static_cast<laszip_U32>(newNumVlrs);

		m_laszipHeader.offset_to_point_data += LasDetails::SizeOfVlrs(&vlrs[newNumVlrs - 1], 1);
	}

	m_fieldsSaver.setStandarFields(std::move(parameters.standardFields));
	m_fieldsSaver.setExtraFields(std::move(parameters.extraFields));
	m_shouldSaveRGB = parameters.shouldSaveRGB && cloud.hasColors();

	if (parameters.shouldSaveWaveform)
	{
		assert(LasDetails::HasWaveform(m_laszipHeader.point_data_format) && cloud.hasFWF());
		m_waveformSaver = std::make_unique<LasWaveformSaver>(cloud);
	}
}

void LasSaver::initLaszipHeader(const Parameters& parameters)
{
	QDate currentDate                 = QDate::currentDate();
	m_laszipHeader.file_creation_year = currentDate.year();
	m_laszipHeader.file_creation_day  = currentDate.dayOfYear();

	m_laszipHeader.version_major     = parameters.versionMajor;
	m_laszipHeader.version_minor     = parameters.versionMinor;
	m_laszipHeader.point_data_format = parameters.pointFormat;

	// TODO global encoding wkt and other
	if (LasDetails::HasWaveform(m_laszipHeader.point_data_format) && m_cloudToSave.hasFWF())
	{
		// We always store FWF externally
		m_laszipHeader.global_encoding |= 0b0000'0100;    // bit 2 = Waveform Data Packets External
		m_laszipHeader.global_encoding &= (~0b0000'0010); // bit 1 = Waveform Data Packets Internal (deprecated, we do this 'just in case')
	}

	m_laszipHeader.header_size              = LasDetails::HeaderSize(m_laszipHeader.version_minor);
	m_laszipHeader.offset_to_point_data     = m_laszipHeader.header_size;
	m_laszipHeader.point_data_record_length = LasDetails::PointFormatSize(m_laszipHeader.point_data_format);

	LasVlr vlr;
	if (LasMetadata::LoadVlrs(m_cloudToSave, vlr))
	{
		m_laszipHeader.vlrs                              = new laszip_vlr_struct[vlr.numVlrs()];
		m_laszipHeader.number_of_variable_length_records = vlr.numVlrs();
		for (laszip_U32 i = 0; i < m_laszipHeader.number_of_variable_length_records; ++i)
		{
			LasDetails::CloneVlrInto(vlr.vlrs[i], m_laszipHeader.vlrs[i]);
		}
		m_laszipHeader.offset_to_point_data += LasDetails::SizeOfVlrs(m_laszipHeader.vlrs, m_laszipHeader.number_of_variable_length_records);
	}

	// set LAS scale
	m_laszipHeader.x_scale_factor = parameters.lasScale.x;
	m_laszipHeader.y_scale_factor = parameters.lasScale.y;
	m_laszipHeader.z_scale_factor = parameters.lasScale.z;

	// set LAS offset
	m_laszipHeader.x_offset = parameters.lasOffset.x;
	m_laszipHeader.y_offset = parameters.lasOffset.y;
	m_laszipHeader.z_offset = parameters.lasOffset.z;

	strncpy(m_laszipHeader.generating_software, "CloudCompare", 32);
}
LasSaver::~LasSaver() noexcept
{
	if (m_laszipWriter)
	{
		laszip_close_writer(m_laszipWriter);
		laszip_clean(m_laszipWriter);
		laszip_destroy(m_laszipWriter);
	}

	if (m_originallySelectedScalarField != -1)
	{
		m_cloudToSave.setCurrentDisplayedScalarField(m_originallySelectedScalarField);

		// m_originallySelectedScalarField it means we did create temporary
		// Nx, Ny, Nz scalar fields, so we remove them
		for (size_t i = 0; i < 3; ++i)
		{
			if (m_normalDimWasTemporarillyExported[i])
			{
				const int idx = m_cloudToSave.getScalarFieldIndexByName(CC_NORMAL_NAMES[i]);
				if (idx != -1)
				{
					m_cloudToSave.deleteScalarField(idx);
				}
			}
		}
	}
}

CC_FILE_ERROR LasSaver::open(const QString filePath)
{
	laszip_CHAR* errorMsg{nullptr};
	if (laszip_create(&m_laszipWriter))
	{
		ccLog::Warning("[LAS] laszip failed to create the writer");
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (laszip_set_header(m_laszipWriter, &m_laszipHeader))
	{
		laszip_get_error(m_laszipWriter, &errorMsg);
		ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (laszip_open_writer(m_laszipWriter, qPrintable(filePath), filePath.endsWith("laz")))
	{
		laszip_get_error(m_laszipWriter, &errorMsg);
		ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (laszip_get_point_pointer(m_laszipWriter, &m_laszipPoint))
	{
		fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	return CC_FERR_NO_ERROR;
}

CC_FILE_ERROR LasSaver::saveNextPoint()
{
	if (!m_laszipPoint)
	{
		assert(false);
		return CC_FILE_ERROR::CC_FERR_INTERNAL;
	}

	if (m_currentPointIndex >= m_cloudToSave.size())
	{
		return CC_FERR_NO_SAVE;
	}
	laszip_CHAR* errorMsg{nullptr};

	// reset point
	laszip_I32 num_extra_bytes         = m_laszipPoint->num_extra_bytes;
	laszip_U8* extra_bytes             = m_laszipPoint->extra_bytes;
	*m_laszipPoint                     = {};
	m_laszipPoint->extra_bytes         = extra_bytes;
	m_laszipPoint->num_extra_bytes     = num_extra_bytes;
	m_laszipPoint->extended_point_type = m_laszipHeader.point_data_format >= 6;

	const CCVector3* point       = m_cloudToSave.getPoint(m_currentPointIndex);
	const CCVector3d globalPoint = m_cloudToSave.toGlobal3d<PointCoordinateType>(*point);

	if (laszip_set_coordinates(m_laszipWriter, globalPoint.u))
	{
		laszip_get_error(m_laszipWriter, &errorMsg);
		ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	m_fieldsSaver.handleScalarFields(m_currentPointIndex, *m_laszipPoint);
	m_fieldsSaver.handleExtraFields(m_currentPointIndex, *m_laszipPoint);

	if (m_waveformSaver)
	{
		m_waveformSaver->handlePoint(m_currentPointIndex, *m_laszipPoint);
	}

	if (m_shouldSaveRGB)
	{
		assert(LasDetails::HasRGB(m_laszipHeader.point_data_format) && m_cloudToSave.hasColors());
		const ccColor::Rgba& color = m_cloudToSave.getPointColor(m_currentPointIndex);
		m_laszipPoint->rgb[0]      = static_cast<laszip_U16>(color.r) << 8;
		m_laszipPoint->rgb[1]      = static_cast<laszip_U16>(color.g) << 8;
		m_laszipPoint->rgb[2]      = static_cast<laszip_U16>(color.b) << 8;
	}

	if (laszip_write_point(m_laszipWriter))
	{
		laszip_get_error(m_laszipWriter, &errorMsg);
		ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	if (laszip_update_inventory(m_laszipWriter))
	{
		laszip_get_error(m_laszipWriter, &errorMsg);
		ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
		return CC_FERR_THIRD_PARTY_LIB_FAILURE;
	}

	++m_currentPointIndex;

	return CC_FERR_NO_ERROR;
}

bool LasSaver::canSaveWaveforms() const
{
	return m_waveformSaver != nullptr;
}

QString LasSaver::getLastError() const
{
	laszip_CHAR* errorMsg{nullptr};
	laszip_get_error(m_laszipWriter, &errorMsg);

	return errorMsg;
}
