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

#include "LasIOFilter.h"
#include "LasMetadata.h"
#include "LasOpenDialog.h"
#include "LasSaveDialog.h"
#include "LasSaver.h"
#include "LasScalarFieldLoader.h"
#include "LasScalarFieldSaver.h"
#include "LasVlr.h"
#include "LasWaveformLoader.h"
#include "LasWaveformSaver.h"

// CC
#include <GenericProgressCallback.h>
#include <ccColorScalesManager.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

// Qt
#include <QDate>
#include <QElapsedTimer>
#include <QFileInfo>

// LASzip
#include <laszip/laszip_api.h>

// System
#include <memory>
#include <numeric>
#include <utility>

constexpr const char LAS_METADATA_INFO_KEY[] = "LAS.savedInfo";

static CCVector3d GetGlobalShift(FileIOFilter::LoadParameters &parameters,
                                 bool &preserveCoordinateShift,
                                 const CCVector3d &lasOffset,
                                 const CCVector3d &firstPoint)
{
    ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
    CCVector3d shift{};
    bool useLasOffset = false;

    // set the lasOffset as default if none was provided
    if (lasOffset.norm2() != 0 &&
        (!parameters._coordinatesShiftEnabled || !*parameters._coordinatesShiftEnabled))
    {
        if (csModeBackup !=
            ccGlobalShiftManager::NO_DIALOG) // No dialog, practically means that we don't want any shift!
        {
            useLasOffset = true;
            shift = -lasOffset;
            if (csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
            {
                parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
            }
        }
    }

    bool result =
        FileIOFilter::HandleGlobalShift(firstPoint, shift, preserveCoordinateShift, parameters, useLasOffset);
    if (!result)
    {
        preserveCoordinateShift = false;
        shift = {0.0, 0.0, 0.0};
    }

    // restore previous parameters
    parameters.shiftHandlingMode = csModeBackup;
    return shift;
}

LasIOFilter::LasIOFilter()
    : FileIOFilter({"LAS IO Filter",
                    DEFAULT_PRIORITY, // priority
                    QStringList{"las", "laz"},
                    "laz",
                    QStringList{"LAS file (*.las *.laz)"},
                    QStringList{"LAS file (*.las *.laz)"},
                    Import | Export})
{
}

CC_FILE_ERROR LasIOFilter::loadFile(const QString &fileName, ccHObject &container, LoadParameters &parameters)
{
    laszip_POINTER laszipReader{};
    laszip_header *laszipHeader{nullptr};
    laszip_BOOL isCompressed{false};
    laszip_CHAR *errorMsg{nullptr};

    if (laszip_create(&laszipReader))
    {
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_open_reader(laszipReader, qPrintable(fileName), &isCompressed))
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_get_header_pointer(laszipReader, &laszipHeader))
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    laszip_U64 pointCount;
    if (laszipHeader->version_minor == 4)
    {
        pointCount = laszipHeader->extended_number_of_point_records;
    }
    else
    {
        pointCount = laszipHeader->number_of_point_records;
    }

    if (pointCount >= std::numeric_limits<unsigned int>::max())
    {
        // TODO
        ccLog::Error("Files with more that %lu points are not supported", pointCount);
        return CC_FERR_NOT_IMPLEMENTED;
    }

    auto pointCloud = std::make_unique<ccPointCloud>(QFileInfo(fileName).fileName());
    if (!pointCloud->reserve(pointCount))
    {
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_NOT_ENOUGH_MEMORY;
    }

    std::vector<LasScalarField> availableScalarFields =
        LasScalarField::ForPointFormat(laszipHeader->point_data_format);

    std::vector<LasExtraScalarField> availableEXtraScalarFields =
        LasExtraScalarField::ParseExtraScalarFields(*laszipHeader);

    std::unique_ptr<FileInfo> infoOfCurrentFile = std::make_unique<FileInfo>();
    infoOfCurrentFile->version.minorVersion = laszipHeader->version_minor;
    infoOfCurrentFile->version.pointFormat = laszipHeader->point_data_format;
    infoOfCurrentFile->extraScalarFields = availableEXtraScalarFields;

    bool fileContentIsDifferentFromPrevious =
        (m_infoOfLastOpened && (*m_infoOfLastOpened != *infoOfCurrentFile));
    if (!m_openDialog || fileContentIsDifferentFromPrevious)
    {
        m_openDialog.reset(new LasOpenDialog);
    }
    m_infoOfLastOpened = std::move(infoOfCurrentFile);
    LasOpenDialog &dialog = *m_openDialog;
    dialog.setInfo(laszipHeader->version_minor, laszipHeader->point_data_format, pointCount);
    dialog.setAvailableScalarFields(availableScalarFields, availableEXtraScalarFields);

    if (parameters.sessionStart)
    {
        // we do this AFTER restoring the previous context because it may still be
        // good that the previous configuration is restored even though the user needs
        // to confirm it
        dialog.resetShouldSkipDialog();
    }

    if (parameters.alwaysDisplayLoadDialog && !dialog.shouldSkipDialog())
    {
        dialog.exec();
        if (dialog.result() == QDialog::Rejected)
        {
            laszip_close_reader(laszipReader);
            laszip_clean(laszipReader);
            laszip_destroy(laszipReader);
            return CC_FERR_CANCELED_BY_USER;
        }
    }

    dialog.filterOutNotChecked(availableScalarFields, availableEXtraScalarFields);

    CCVector3d lasOffset(laszipHeader->x_offset,
                         laszipHeader->y_offset,
                         0.0 /*laszipHeader->z_offset*/); // it's never a good idea to shift along Z

    laszip_F64 laszipCoordinates[3];
    laszip_point *laszipPoint;
    CCVector3 currentPoint{};
    CCVector3d shift;
    bool preserveGlobalShift{true};

    if (laszip_get_point_pointer(laszipReader, &laszipPoint))
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    LasScalarFieldLoader loader(availableScalarFields, availableEXtraScalarFields, *pointCloud);
    loader.setIgnoreFieldsWithDefaultValues(dialog.shouldIgnoreFieldsWithDefaultValues());
    loader.setForce8bitRgbMode(dialog.shouldForce8bitColors());
    loader.setManualTimeShift(dialog.timeShiftValue());
    std::unique_ptr<LasWaveformLoader> waveformLoader{nullptr};
    if (LasDetails::HasWaveform(laszipHeader->point_data_format))
    {
        waveformLoader = std::make_unique<LasWaveformLoader>(*laszipHeader, fileName, *pointCloud);
    }

    QElapsedTimer timer;
    timer.start();

    ccProgressDialog progressDialog(true, parameters.parentWidget);
    progressDialog.setMethodTitle("Loading LAS points");
    progressDialog.setInfo("Loading points");
    CCCoreLib::NormalizedProgress normProgress(&progressDialog, pointCount);
    progressDialog.start();

    CC_FILE_ERROR error{CC_FERR_NO_ERROR};
    for (unsigned int i{0}; i < pointCount; ++i)
    {
        if (progressDialog.isCancelRequested())
        {
            error = CC_FERR_CANCELED_BY_USER;
            break;
        }

        if (laszip_read_point(laszipReader))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }

        if (laszip_get_coordinates(laszipReader, laszipCoordinates))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }

        if (i == 0)
        {
            CCVector3d firstPoint(laszipCoordinates[0], laszipCoordinates[1], laszipCoordinates[2]);
            shift = GetGlobalShift(parameters, preserveGlobalShift, lasOffset, firstPoint);

            if (preserveGlobalShift)
            {
                pointCloud->setGlobalShift(shift);
            }

            if (shift.norm2() != 0.0)
            {
                ccLog::Warning("[LAS] Cloud has been re-centered! Translation: (%.2f ; %.2f ; %.2f)",
                               shift.x,
                               shift.y,
                               shift.z);
            }
            pointCloud->setGlobalShift(shift);
        }

        currentPoint.x = static_cast<PointCoordinateType>(laszipCoordinates[0] + shift.x);
        currentPoint.y = static_cast<PointCoordinateType>(laszipCoordinates[1] + shift.y);
        currentPoint.z = static_cast<PointCoordinateType>(laszipCoordinates[2] + shift.z);

        pointCloud->addPoint(currentPoint);

        error = loader.handleScalarFields(*pointCloud, *laszipPoint);
        if (error != CC_FERR_NO_ERROR)
        {
            break;
        }

        error = loader.handleExtraScalarFields(*pointCloud, *laszipPoint);
        if (error != CC_FERR_NO_ERROR)
        {
            break;
        }

        if (LasDetails::HasRGB(laszipHeader->point_data_format))
        {
            error = loader.handleRGBValue(*pointCloud, *laszipPoint);
            if (error != CC_FERR_NO_ERROR)
            {
                break;
            }
        }

        if (waveformLoader)
        {
            waveformLoader->loadWaveform(*pointCloud, *laszipPoint);
        }

        normProgress.oneStep();
    }

    for (const LasScalarField &field : loader.standardFields())
    {
        if (field.sf == nullptr)
        {
            // It may be null if all values were the same
            continue;
        }
        field.sf->computeMinAndMax();
        field.sf->setSaturationStart(field.sf->getMin());
        field.sf->setSaturationStop(field.sf->getMax());
        field.sf->setMinDisplayed(field.sf->getMin());
        field.sf->setMaxDisplayed(field.sf->getMax());
        switch (field.id)
        {
        case LasScalarField::Intensity:
            field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
        case LasScalarField::ReturnNumber:
        case LasScalarField::NumberOfReturns:
        case LasScalarField::ScanDirectionFlag:
        case LasScalarField::EdgeOfFlightLine:
        case LasScalarField::Classification:
        case LasScalarField::SyntheticFlag:
        case LasScalarField::KeypointFlag:
        case LasScalarField::WithheldFlag:
        case LasScalarField::ScanAngleRank:
        case LasScalarField::UserData:
        case LasScalarField::PointSourceId:
        case LasScalarField::ExtendedScannerChannel:
        case LasScalarField::OverlapFlag:
        case LasScalarField::ExtendedClassification:
        case LasScalarField::ExtendedReturnNumber:
        case LasScalarField::ExtendedNumberOfReturns:
        case LasScalarField::NearInfrared:
        {
            auto cMin = static_cast<int64_t>(field.sf->getMin());
            auto cMax = static_cast<int64_t>(field.sf->getMax());
            int64_t steps = std::min<int64_t>(cMax - cMin + 1, 256);
            field.sf->setColorRampSteps(steps);
            break;
        }
        case LasScalarField::GpsTime:
            field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BGYR));
            break;
        case LasScalarField::ExtendedScanAngle:
            field.sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::BGYR));
            break;
        }

        pointCloud->addScalarField(field.sf);
    }

    for (const LasExtraScalarField &field : loader.extraFields())
    {
        for (size_t i{0}; i < field.numElements(); ++i)
        {
            Q_ASSERT(field.scalarFields[i] != nullptr);
            field.scalarFields[i]->computeMinAndMax();
            field.scalarFields[i]->setSaturationStart(field.scalarFields[i]->getMin());
            field.scalarFields[i]->setSaturationStop(field.scalarFields[i]->getMax());
            field.scalarFields[i]->setMinDisplayed(field.scalarFields[i]->getMin());
            field.scalarFields[i]->setMaxDisplayed(field.scalarFields[i]->getMax());
            pointCloud->addScalarField(field.scalarFields[i]);
        }
    }

    int idx = pointCloud->getScalarFieldIndexByName(LasNames::Intensity);
    if (idx != -1)
    {
        pointCloud->setCurrentDisplayedScalarField(idx);
    }
    else if (pointCloud->getNumberOfScalarFields() > 0)
    {
        pointCloud->setCurrentDisplayedScalarField(0);
    }
    pointCloud->showColors(pointCloud->hasColors());
    pointCloud->showSF(!pointCloud->hasColors() && pointCloud->hasDisplayedScalarField());

    for (LasExtraScalarField &extraField : availableEXtraScalarFields)
    {
        extraField.resetScalarFieldsPointers();
    }

    LasMetadata::SaveMetadataInto(*laszipHeader, *pointCloud, availableEXtraScalarFields);

    container.addChild(pointCloud.release());

    if (error == CC_FERR_THIRD_PARTY_LIB_FAILURE)
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
    }

    timer.elapsed();
    qint64 elapsed = timer.elapsed();
    int32_t minutes = timer.elapsed() / (1000 * 60);
    elapsed -= minutes * (1000 * 60);
    int32_t seconds = elapsed / 1000;
    elapsed -= seconds * 1000;
    ccLog::Print(QString("[LAS] File loaded in %1m%2s%3ms").arg(minutes).arg(seconds).arg(elapsed));
    return error;
}

bool LasIOFilter::canSave(CC_CLASS_ENUM type, bool &multiple, bool &exclusive) const
{
    multiple = false;
    exclusive = true;
    return type == CC_TYPES::POINT_CLOUD;
}

CC_FILE_ERROR LasIOFilter::saveToFile(ccHObject *entity,
                                      const QString &filename,
                                      const FileIOFilter::SaveParameters &parameters)
{
    if (!entity || filename.isEmpty())
    {
        return CC_FERR_BAD_ARGUMENT;
    }

    if (!entity->isA(CC_TYPES::POINT_CLOUD))
    {
        return CC_FERR_BAD_ENTITY_TYPE;
    }
    auto *pointCloud = static_cast<ccPointCloud *>(entity);

    LasSaveDialog saveDialog(pointCloud, parameters.parentWidget);

    // compute optimal scale
    // optimal scale (for accuracy) --> 1e-9 because the maximum integer is roughly +/-2e+9
    CCVector3 bbMax;
    CCVector3 bbMin;
    pointCloud->getBoundingBox(bbMin, bbMax);
    CCVector3d diag = bbMax - bbMin;

    CCVector3d optimalScale(1.0e-9 * std::max<double>(diag.x, CCCoreLib::ZERO_TOLERANCE_D),
                            1.0e-9 * std::max<double>(diag.y, CCCoreLib::ZERO_TOLERANCE_D),
                            1.0e-9 * std::max<double>(diag.z, CCCoreLib::ZERO_TOLERANCE_D));
    saveDialog.setOptimalScale(optimalScale);

    // See if we have a scale from an origin las file
    CCVector3d originalScales;
    if (LasMetadata::LoadScalesFrom(*pointCloud, originalScales))
    {
        saveDialog.setOriginalScale(originalScales);
    }

    // Find the best version for the file or try to use
    // the one from original file
    LasDetails::LasVersion bestVersion = LasDetails::SelectBestVersion(*pointCloud);
    LasDetails::LasVersion savedVersion;
    if (LasMetadata::LoadLasVersionFrom(*pointCloud, savedVersion))
    {
        // We do have a version we saved from the original file,
        // however we don't want to downgrade it.
        if (bestVersion.minorVersion < savedVersion.minorVersion &&
            bestVersion.pointFormat < savedVersion.pointFormat)
        {
            bestVersion = savedVersion;
        }
    }

    saveDialog.setVersionAndPointFormat(bestVersion);

    // Try to pre-fill in the UI any saved extra scalar fields
    LasVlr vlr;
    if (LasMetadata::LoadVlrs(*pointCloud, vlr))
    {
        LasExtraScalarField::MatchExtraBytesToScalarFields(vlr.extraScalarFields, *pointCloud);
    }
    saveDialog.setExtraScalarFields(vlr.extraScalarFields);

    if (parameters.alwaysDisplaySaveDialog)
    {
        saveDialog.exec();
        if (saveDialog.result() == QDialog::Rejected)
        {
            return CC_FERR_CANCELED_BY_USER;
        }
    }

    CC_FILE_ERROR error;
    LasSaver saver(*pointCloud, saveDialog);
    error = saver.open(filename);
    if (error != CC_FERR_NO_ERROR)
    {
        return error;
    }
    ccProgressDialog progressDialog(true, parameters.parentWidget);
    progressDialog.setMethodTitle("Saving LAS points");
    progressDialog.setInfo("Saving points");
    CCCoreLib::NormalizedProgress normProgress(&progressDialog, pointCloud->size());
    unsigned int numStepsForUpdate = 1 * pointCloud->size() / 100;
    unsigned int lastProgressUpdate = 0;
    progressDialog.start();

    for (unsigned int i{0}; i < pointCloud->size(); ++i)
    {
        error = saver.saveNextPoint();
        if (error != CC_FERR_NO_ERROR)
        {
            break;
        }

        if ((i - lastProgressUpdate) == numStepsForUpdate)
        {
            normProgress.steps(i - lastProgressUpdate);
            lastProgressUpdate += (i - lastProgressUpdate);
            QApplication::processEvents();
        }
    }

    if (error == CC_FERR_THIRD_PARTY_LIB_FAILURE)
    {
        // laszip_get_error(laszipWriter, &errorMsg);
        // ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
        return error;
    }

    if (saver.savesWaveforms())
    {
        const ccPointCloud::SharedFWFDataContainer &fwfData = pointCloud->fwfData();
        QFileInfo info(filename);
        QString wdpFilename = QString("%1/%2.wdp").arg(info.path(), info.baseName());
        QFile fwfFile(wdpFilename);

        if (!fwfFile.open(QIODevice::WriteOnly))
        {
            ccLog::Error("[LAS] Failed to write waveform data");
            error = CC_FERR_WRITING;
        }
        else
        {
            {
                LasDetails::EvlrHeader header = LasDetails::EvlrHeader::Waveform();
                QDataStream stream(&fwfFile);
                stream << header;
            }
            fwfFile.write(reinterpret_cast<const char *>(fwfData->data()), fwfData->size());
            ccLog::Print(QString("[LAS] Successfully saved FWF in external file '%1'").arg(wdpFilename));
        }
    }

    return error;
}
