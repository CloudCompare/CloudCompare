#include "LasSaver.h"
#include "LasMetadata.h"

#include <QDate>

#include <ccGlobalShiftManager.h>
#include <ccPointCloud.h>

LasSaver::LasSaver(ccPointCloud &cloud, const LasSaveDialog &saveDlg) : m_cloudToSave(cloud)
{
    initLaszipHeader(saveDlg);

    std::vector<LasScalarField> standardFields = saveDlg.fieldsToSave();
    std::vector<LasExtraScalarField> extraFields = saveDlg.extraFieldsToSave();

    LasExtraScalarField::UpdateByteOffsets(extraFields);
    unsigned int totalExtraByteSize = LasExtraScalarField::TotalExtraBytesSize(extraFields);

    if (totalExtraByteSize > 0)
    {
        m_laszipHeader.point_data_record_length += totalExtraByteSize;

        size_t newNumVlrs = m_laszipHeader.number_of_variable_length_records + 1;
        laszip_vlr *vlrs = new laszip_vlr_struct[newNumVlrs];
        // Move already existing vlrs
        for (size_t i{0}; i < m_laszipHeader.number_of_variable_length_records; i++)
        {
            vlrs[i] = m_laszipHeader.vlrs[i];
        }
        LasExtraScalarField::InitExtraBytesVlr(vlrs[newNumVlrs - 1], extraFields);

        delete m_laszipHeader.vlrs;
        m_laszipHeader.vlrs = vlrs;
        m_laszipHeader.number_of_variable_length_records = static_cast<laszip_U32>(newNumVlrs);

        m_laszipHeader.offset_to_point_data +=
            LasDetails::SizeOfVlrs(&m_laszipHeader.vlrs[newNumVlrs - 1], 1);
    }

    m_fieldsSaver.setStandarFields(std::move(standardFields));
    m_fieldsSaver.setExtraFields(std::move(extraFields));
    m_shouldSaveRGB = saveDlg.shouldSaveRGB() && cloud.hasColors();

    if (saveDlg.shouldSaveWaveform())
    {
        Q_ASSERT(LasDetails::HasWaveform(m_laszipHeader.point_data_format) && cloud.hasFWF());
        m_waveformSaver = std::make_unique<LasWaveformSaver>(cloud);
    }
}

void LasSaver::initLaszipHeader(const LasSaveDialog &saveDialog)
{
    QDate currentDate = QDate::currentDate();
    m_laszipHeader.file_creation_year = currentDate.year();
    m_laszipHeader.file_creation_day = currentDate.dayOfYear();

    m_laszipHeader.version_major = 1;
    m_laszipHeader.version_minor = saveDialog.selectedVersionMinor();
    m_laszipHeader.point_data_format = saveDialog.selectedPointFormat();

    // TODO global encoding wkt and other
    if (LasDetails::HasWaveform(m_laszipHeader.point_data_format) && m_cloudToSave.hasFWF())
    {
        // We always store FWF externally
        m_laszipHeader.global_encoding |= 0b0000'0100;
    }

    m_laszipHeader.header_size = LasDetails::HeaderSize(m_laszipHeader.version_minor);
    m_laszipHeader.offset_to_point_data = m_laszipHeader.header_size;
    m_laszipHeader.point_data_record_length = LasDetails::PointFormatSize(m_laszipHeader.point_data_format);

    LasVlr vlr;
    if (LasMetadata::LoadVlrs(m_cloudToSave, vlr))
    {
        m_laszipHeader.vlrs = new laszip_vlr_struct[vlr.numVlrs];
        m_laszipHeader.number_of_variable_length_records = vlr.numVlrs;
        for (laszip_U32 i{0}; i < m_laszipHeader.number_of_variable_length_records; ++i)
        {
            LasDetails::CloneVlrInto(vlr.vlrs[i], m_laszipHeader.vlrs[i]);
        }
        m_laszipHeader.offset_to_point_data +=
            LasDetails::SizeOfVlrs(m_laszipHeader.vlrs, m_laszipHeader.number_of_variable_length_records);
    }

    CCVector3d lasScale = saveDialog.chosenScale();
    m_laszipHeader.x_scale_factor = lasScale.x;
    m_laszipHeader.y_scale_factor = lasScale.y;
    m_laszipHeader.z_scale_factor = lasScale.z;

    strncpy(m_laszipHeader.generating_software, "CloudCompare", 32);

    CCVector3d bbMin, bbMax;
    m_cloudToSave.getOwnGlobalBB(bbMin, bbMax);
    CCVector3d lasOffsets;
    if (LasMetadata::LoadOffsetsFrom(m_cloudToSave, lasOffsets))
    {
        // Check that the saved offset still 'works'
        if (ccGlobalShiftManager::NeedShift(bbMax - lasOffsets))
        {
            ccLog::Warning("[LAS] The former LAS_OFFSET doesn't seem to be optimal. Using the minimum "
                           "bounding-box corner instead.");
            CCVector3d globaShift =
                m_cloudToSave.getGlobalShift(); //'global shift' is the opposite of LAS offset ;)

            if (ccGlobalShiftManager::NeedShift(bbMax + globaShift))
            {
                ccLog::Warning("[LAS] Using the minimum bounding-box corner instead.");
                m_laszipHeader.x_offset = bbMin.x;
                m_laszipHeader.y_offset = bbMin.y;
                m_laszipHeader.z_offset = 0;
            }
            else
            {
                ccLog::Warning("[LAS] Using the previous Global Shift instead.");
                m_laszipHeader.x_offset = -globaShift.x;
                m_laszipHeader.y_offset = -globaShift.y;
                m_laszipHeader.z_offset = -globaShift.z;
            }
        }
        else
        {
            m_laszipHeader.x_offset = lasOffsets.x;
            m_laszipHeader.y_offset = lasOffsets.y;
            m_laszipHeader.z_offset = lasOffsets.z;
        }
    }
    else
    {
        // This point cloud does not come from a LAS file,
        // so we don't have saved offset for it.

        if (m_cloudToSave.isShifted())
        {
            m_laszipHeader.x_offset = -m_cloudToSave.getGlobalShift().x;
            m_laszipHeader.y_offset = -m_cloudToSave.getGlobalShift().y;
            m_laszipHeader.z_offset = -m_cloudToSave.getGlobalShift().z;
        }
        else if (ccGlobalShiftManager::NeedShift(bbMax))
        {
            m_laszipHeader.x_offset = bbMin.x;
            m_laszipHeader.y_offset = bbMin.y;
            m_laszipHeader.z_offset = bbMin.z;
        }
    }
}
LasSaver::~LasSaver() noexcept
{
    if (m_laszipWriter)
    {
        laszip_close_writer(m_laszipWriter);
        laszip_clean(m_laszipWriter);
        laszip_destroy(m_laszipWriter);
    }
}

CC_FILE_ERROR LasSaver::open(const QString filePath)
{
    laszip_CHAR *errorMsg{nullptr};
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
    if (m_currentPointIndex >= m_cloudToSave.size())
    {
        return CC_FERR_NO_SAVE;
    }
    laszip_CHAR *errorMsg{nullptr};

    // reset point
    int totalExtraByteSize = m_laszipPoint->num_extra_bytes;
    laszip_U8 *extra_bytes = m_laszipPoint->extra_bytes;
    *m_laszipPoint = laszip_point{};
    m_laszipPoint->extra_bytes = extra_bytes;
    m_laszipPoint->num_extra_bytes = totalExtraByteSize;

    const CCVector3 *point = m_cloudToSave.getPoint(m_currentPointIndex);
    const CCVector3d globalPoint = m_cloudToSave.toGlobal3d<PointCoordinateType>(*point);

    laszip_F64 coords[3];
    coords[0] = globalPoint.x;
    coords[1] = globalPoint.y;
    coords[2] = globalPoint.z;

    if (laszip_set_coordinates(m_laszipWriter, coords))
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
        Q_ASSERT(LasDetails::HasRGB(m_laszipHeader.point_data_format) && m_cloudToSave.hasColors());
        const ccColor::Rgba &color = m_cloudToSave.getPointColor(m_currentPointIndex);
        m_laszipPoint->rgb[0] = static_cast<laszip_U16>(color.r) << 8;
        m_laszipPoint->rgb[1] = static_cast<laszip_U16>(color.g) << 8;
        m_laszipPoint->rgb[2] = static_cast<laszip_U16>(color.b) << 8;
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

    m_currentPointIndex++;
    return CC_FERR_NO_ERROR;
}

bool LasSaver::savesWaveforms() const
{
    return m_waveformSaver != nullptr;
}
