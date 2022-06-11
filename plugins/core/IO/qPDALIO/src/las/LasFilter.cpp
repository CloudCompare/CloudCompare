//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "LasFilter.h"
#include "CCToPdal.h"
#include "PdalToCC.h"
#include "Compatibility.h"

// qCC_db
#include "ccHObjectCaster.h"
#include "ccLog.h"
#include "ccPointCloud.h"
#include "ccProgressDialog.h"

// CCCoreLib

// Qt
#include <QFileInfo>
#include <QFuture>
#include <QInputDialog>
#include <QtConcurrent>

// pdal
#include <memory>
#include <pdal/Dimension.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>
//#include <pdal/io/LasVLR.hpp>
#include <pdal/io/LasWriter.hpp>

// System
#include "LasFields.h"
#include "ProgressStage.h"
#include "ui/LasOpenDialog.h"
#include "ui/LasSaveDialog.h"

Q_DECLARE_METATYPE(pdal::LasHeader)

Q_DECLARE_METATYPE(SavedExtraField)

LasFilter::LasFilter()
    : FileIOFilter({"LAS Filter [PDAL]",
                    3.0f, // priority
                    QStringList{"las", "laz"}, "las",
                    QStringList{"LAS cloud (*.las *.laz)"},
                    QStringList{"LAS cloud (*.las *.laz)"}, Import | Export}) {}

bool LasFilter::canSave(CC_CLASS_ENUM type, bool &multiple,
                        bool &exclusive) const {
  if (type == static_cast<CC_CLASS_ENUM>(CC_TYPES::POINT_CLOUD)) {
    multiple = false;
    exclusive = true;
    return true;
  }
  return false;
}

// TODO progress callback
CC_FILE_ERROR LasFilter::saveToFile(ccHObject *entity, const QString &filename,
                                    const SaveParameters &parameters) {

  if (!entity || filename.isEmpty())
    return CC_FERR_BAD_ARGUMENT;

  ccPointCloud *theCloud = ccHObjectCaster::ToPointCloud(entity);
  if (!theCloud) {
    ccLog::Warning("[LAS] This filter can only save one cloud at a time");
    return CC_FERR_BAD_ENTITY_TYPE;
  }

  unsigned int numberOfPoints = theCloud->size();
  if (numberOfPoints == 0) {
    ccLog::Warning("[LAS] Cloud is empty!");
    return CC_FERR_NO_SAVE;
  }

  LasSaveDialog saveDialog(theCloud);

  if (theCloud->hasMetaData("LAS_HEADER")) {
    auto lasHeader =
        theCloud->getMetaData("LAS_HEADER").value<pdal::LasHeader>();
    QString version =
        QString("1.%1").arg(QString::number(VersionMinorFromHeader(lasHeader)));
    saveDialog.setVersionAndPointFormat(version, lasHeader.pointFormat());
    saveDialog.setSavedScale(ScalesFromHeader(lasHeader));

    auto savedExtra = theCloud->getMetaData("LAS_EXTRAS")
                          .value<std::vector<SavedExtraField>>();
    if (!savedExtra.empty()) {
      saveDialog.setExtraScalarFields(savedExtra);
    }
  } else {
    LasVersion v = SelectBestVersion(*theCloud);
    QString version = QString("1.%1").arg(QString::number(v.minorVersion));
    saveDialog.setVersionAndPointFormat(version, v.pointFormat);
  }

  if (saveDialog.exec() != QDialog::Accepted) {
    return CC_FERR_CANCELED_BY_USER;
  }

  pdal::Options writerOptions;

  try {
    writerOptions.add("filename", filename.toStdString());
    writerOptions.add("minor_version", saveDialog.selectedVersionMinor());
    writerOptions.add("dataformat_id", saveDialog.selectedPointFormat());
    writerOptions.add("extra_dims", "all");
  } catch (const pdal::pdal_error &e) {
    ccLog::Error(e.what());
    return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
  }

  if (theCloud->hasMetaData("LAS_HEADER")) {
    auto lasHeader =
        theCloud->getMetaData("LAS_HEADER").value<pdal::LasHeader>();

    for (const pdal::LasVLR &vlr : lasHeader.vlrs()) {
      if (vlr.dataLen() == 0) {
        continue;
      }
      QByteArray dataBase64 = QByteArray(vlr.data()).toBase64();
      if (dataBase64.isEmpty()) {
        ccLog::PrintDebug(
            "Vlr(%s, %d) won't be saved as it has empty data in base64",
            vlr.userId().c_str(), vlr.recordId());
        continue;
      }
      QJsonObject vlrObject;
      vlrObject["description"] = vlr.description().c_str();
      vlrObject["record_id"] = vlr.recordId();
      vlrObject["user_id"] = vlr.userId().c_str();
      vlrObject["data"] = dataBase64.data();

      writerOptions.add("vlrs",
                        QJsonDocument(vlrObject).toJson().toStdString());
    }
  }

  CCToPdal converter(theCloud);
  converter.setFieldsToSave(saveDialog.fieldsToSave());
  converter.setStoreColors(saveDialog.shouldSaveColors());

  pdal::FixedPointTable pointTable(10'00);

  QScopedPointer<ccProgressDialog> pDlg(nullptr);
  pDlg.reset(
      new ccProgressDialog(true, parameters.parentWidget)); // cancel available
  pDlg->setMethodTitle(QObject::tr("Save LAS file"));
  pDlg->setInfo(QObject::tr("Points: %L1").arg(theCloud->size()));
  pDlg->start();
  CCCoreLib::NormalizedProgress nProgress(pDlg.data(), theCloud->size());

  ProgressStage progressStage(nProgress);
  progressStage.setInput(converter);

  try {
    pdal::LasWriter writer;
    writer.setOptions(writerOptions);
    writer.setInput(progressStage);
    writer.prepare(pointTable);
    writer.execute(pointTable);
  } catch (const pdal::pdal_error &e) {
    ccLog::Error(e.what());
    return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
  }

  return CC_FERR_NO_ERROR;
}

struct DetermineLasShiftCallback {
  FileIOFilter::LoadParameters parameters;
  CCVector3d lasOffsets;

  void operator()(const CCVector3d &point, ccPointCloud &cloud) {

    // backup input global parameters
    ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
    bool useLasOffset = false;
    bool preserveCoordinateShift = true;
    CCVector3d shiftToUse;

    // set the lasOffset as default if none was provided
    if (lasOffsets.norm2() != 0 && (!parameters._coordinatesShiftEnabled ||
                                    !*parameters._coordinatesShiftEnabled)) {
      if (csModeBackup !=
          ccGlobalShiftManager::NO_DIALOG) // No dialog, practically means
      // that we don't want any shift!
      {
        useLasOffset = true;
        shiftToUse = -lasOffsets;
        if (csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT) {
          parameters.shiftHandlingMode =
              ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
        }
      }
    }

    if (FileIOFilter::HandleGlobalShift(point, shiftToUse,
                                        preserveCoordinateShift, parameters,
                                        useLasOffset)) {
      if (preserveCoordinateShift) {
        cloud.setGlobalShift(shiftToUse);
      }
      ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; "
                     "%.2f ; %.2f)",
                     shiftToUse.x, shiftToUse.y, shiftToUse.z);
    }

    // restore previous parameters
    parameters.shiftHandlingMode = csModeBackup;
  }
};

// TODO Handle name collision with extra fields
// TODO progress callback
CC_FILE_ERROR LasFilter::loadFile(const QString &filename, ccHObject &container,
                                  LoadParameters &parameters) {
  pdal::Options readerOptions;
  readerOptions.add("filename", filename.toStdString());
  // We want PDAL to read ExtraBytesVLR for las < 1.4
  readerOptions.add("use_eb_vlr", true);
  pdal::FixedPointTable pointTable(10'000);

  pdal::LasReader lasReader;
  try {
    lasReader.setOptions(readerOptions);
    lasReader.prepare(pointTable);
  } catch (const std::exception &e) {
    ccLog::Error(e.what());
    return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
  }

  pdal::LasHeader lasHeader = lasReader.header();
  if (lasHeader.pointCount() == 0) {
    auto *emptyCloud = new ccPointCloud("empty");
    container.addChild(emptyCloud);
    return CC_FERR_NO_ERROR;
  }

  if (lasHeader.pointCount() >= std::numeric_limits<unsigned int>::max()) {
    ccLog::Error("Cloud is too big");
    return CC_FERR_NOT_IMPLEMENTED;
  }

  pdal::PointLayoutPtr layout(pointTable.layout());
  LasField::Vector lasScalarFields =
      LasField::LasFieldsOfLayout(*layout, Has14PointFormat(lasHeader));

  std::vector<SavedExtraField> savedExtraFields;
  for (const LasField &field : lasScalarFields) {
    if (field.ccId == LasField::Id::Extra) {
      savedExtraFields.push_back(SavedExtraField{field.name, field.pdalType});
    }
  }

  LasOpenDialog openDialog;
  openDialog.setInfo(VersionMinorFromHeader(lasHeader), lasHeader.pointFormat(),
                     lasHeader.pointCount());
  openDialog.setAvailableScalarFields(lasScalarFields);
  openDialog.setHasColors(lasHeader.hasColor());

  if (openDialog.exec() != QDialog::Accepted) {
    return CC_FERR_CANCELED_BY_USER;
  }

  openDialog.filterOutNotChecked(lasScalarFields);

  CCVector3d lasOffsets = OffsetsFromHeader(lasHeader);

  DetermineLasShiftCallback determineLasShiftCallback{parameters, lasOffsets};

  PdalToCc converter;
  converter.setScalarFieldsToLoad(std::move(lasScalarFields));
  converter.setIs14Format(Has14PointFormat(lasHeader));
  converter.setLoadColors(openDialog.loadColors());
  converter.setDetermineShiftCallback(determineLasShiftCallback);
  converter.setInput(lasReader);

  QScopedPointer<ccProgressDialog> pDlg(nullptr);
  pDlg.reset(
      new ccProgressDialog(true, parameters.parentWidget)); // cancel available
  pDlg->setMethodTitle(QObject::tr("Open LAS file"));
  pDlg->setInfo(QObject::tr("Points: %L1").arg(lasHeader.pointCount()));
  pDlg->start();
  CCCoreLib::NormalizedProgress nProgress(pDlg.data(), lasHeader.pointCount());

  ProgressStage progressStage(nProgress);
  progressStage.setInput(converter);

  try {
    progressStage.prepare(pointTable);
    progressStage.execute(pointTable);
  } catch (const std::exception &e) {
    ccLog::Error(e.what());
    return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
  }

  std::unique_ptr<ccPointCloud> cloud = converter.loadedCloud();
  cloud->showColors(cloud->hasColors());
  for (const LasField &lasField : converter.scalarFieldsToLoad()) {
    if (!lasField.sf) {
      continue;
    }
    lasField.sf->computeMinAndMax();
    cloud->addScalarField(lasField.sf);
    Q_ASSERT_X(lasField.sf->size() == cloud->size(), __func__,
               "Scalar field does not have same size as point cloud");
  }

  cloud->setMetaData("LAS_HEADER", QVariant::fromValue(lasHeader));
  cloud->setMetaData("LAS_EXTRAS", QVariant::fromValue(savedExtraFields));

  container.addChild(cloud.release());

  return CC_FERR_NO_ERROR;
}
