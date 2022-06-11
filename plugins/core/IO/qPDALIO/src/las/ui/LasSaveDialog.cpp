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
#include "LasSaveDialog.h"

#include <QPushButton>
#include <QStringListModel>

#include "LasFields.h"
#include "ui/ExtraScalarFieldForm.h"
#include "ui/ExtraScalarFieldRow.h"
#include "ui/LasScalarFieldForm.h"
#include "ui/ScalarFieldRow.h"

#include "ccLog.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"

LasSaveDialog::LasSaveDialog(ccPointCloud *cloud, QWidget *parent)
    : QDialog(parent), m_cloud(cloud), m_comboBoxModel(new QStringListModel),
      m_ebForm(new ExtraScalarFieldForm),
      m_scalarFieldForm(new LasScalarFieldForm) {
  setupUi(this);
  origRadioButton_2->setEnabled(false);
  customScaleDoubleSpinBox_2->setEnabled(true);
  bestRadioButton_2->setChecked(true);

  for (const char *versionStr : GetAvailableVersion()) {
    versionComboBox->addItem(versionStr);
  }
  versionComboBox->setCurrentIndex(0);

  QStringList cloudScalarFieldsNames;
  cloudScalarFieldsNames << QString();
  for (unsigned int i = 0; i < m_cloud->getNumberOfScalarFields(); ++i) {
    if (strcmp(m_cloud->getScalarFieldName(i), "Default") != 0) {
      cloudScalarFieldsNames << m_cloud->getScalarFieldName(i);
    }
  }
  m_comboBoxModel->setStringList(cloudScalarFieldsNames);

  // optimal scale (for accuracy) --> 1e-9 because the maximum integer is
  // roughly +/-2e+9
  CCVector3 bbMax;
  CCVector3 bbMin;
  cloud->getBoundingBox(bbMin, bbMax);
  CCVector3d diag = bbMax - bbMin;
  CCVector3d optimalScale(
      1.0e-9 * std::max<double>(diag.x, CCCoreLib::ZERO_TOLERANCE_D),
      1.0e-9 * std::max<double>(diag.y, CCCoreLib::ZERO_TOLERANCE_D),
      1.0e-9 * std::max<double>(diag.z, CCCoreLib::ZERO_TOLERANCE_D));
  setOptimalScale(optimalScale);

  m_scalarFieldForm->m_ccScalarFieldsNamesModel = m_comboBoxModel;
  m_scalarFieldForm->m_cloud = m_cloud;
  scalarFieldLayout->addWidget(m_scalarFieldForm);

  m_ebForm->setShit(m_cloud, m_comboBoxModel);
  tabWidget->addTab(m_ebForm, "Extra Bytes Fields");

  connect(
      versionComboBox,
      (void(QComboBox::*)(const QString &))(&QComboBox::currentIndexChanged),
      this, &LasSaveDialog::handleSelectedVersionChange);

  connect(pointFormatComboBox,
          (void(QComboBox::*)(int))(&QComboBox::currentIndexChanged), this,
          &LasSaveDialog::handleSelectedPointFormatChange);
}

/// When the selected version changes, we need to update the combo box
/// of point format to match the ones supported by the version
void LasSaveDialog::handleSelectedVersionChange(const QString &version) {
  const std::vector<int> pointFormats =
      PointFormatsAvailableForVersion(qPrintable(version));
  pointFormatComboBox->clear();
  for (unsigned int fmt : pointFormats) {
    pointFormatComboBox->addItem(QString::number(fmt));
  }

  const int previousIndex = pointFormatComboBox->currentIndex();
  pointFormatComboBox->setCurrentIndex(0);
  if (previousIndex == 0) {
    // We have to force the call here
    // because the index did not change,
    // But the actual point format did
    handleSelectedPointFormatChange(0);
  }
}

/// When the user changes the point format, we need to redo the
/// scalar field form.
void LasSaveDialog::handleSelectedPointFormatChange(int index) {
  if (index < 0) {
    return;
  }

  const std::vector<int> pointFormats = PointFormatsAvailableForVersion(
      qPrintable(versionComboBox->currentText()));

  int selectedPointFormat = pointFormats[index];
  const LasField::Vector lasScalarFields =
      LasField::FieldsForPointFormat(selectedPointFormat);

  m_scalarFieldForm->reset(lasScalarFields);

  // Handle the special fields RGB, Waveform
  if (!HasRGB(selectedPointFormat) && !HasWaveform(selectedPointFormat)) {
    specialScalarFieldFrame->hide();
    waveformCheckBox->setCheckState(Qt::Unchecked);
    rgbCheckBox->setCheckState(Qt::Unchecked);
  } else {
    specialScalarFieldFrame->show();
    if (HasRGB(selectedPointFormat)) {
      rgbCheckBox->show();
      rgbCheckBox->setEnabled(m_cloud->hasColors());
      rgbCheckBox->setChecked(m_cloud->hasColors());
    } else {
      rgbCheckBox->hide();
    }

    if (HasWaveform(selectedPointFormat)) {
      waveformCheckBox->show();
      waveformCheckBox->setEnabled(m_cloud->hasFWF());
      waveformCheckBox->setChecked(m_cloud->hasFWF());
    } else {
      waveformCheckBox->hide();
    }
  }

  m_ebForm->reset();
  const QStringList cloudScalarFieldsNames = m_comboBoxModel->stringList();
  for (const QString &ccSfName : cloudScalarFieldsNames) {
    if (m_scalarFieldForm->isSelectedInOneOfTheRow(ccSfName)) {
      continue;
    }
    auto it =
        std::find_if(m_savedExtraFields.begin(), m_savedExtraFields.end(),
                     [ccSfName](const SavedExtraField &savedExtraField) {
                       return savedExtraField.name == ccSfName.toStdString();
                     });
    if (it != m_savedExtraFields.end()) {
      m_ebForm->addRowWithType(ccSfName, it->type);
    } else {
      m_ebForm->addRow(ccSfName);
    }
  }
}

void LasSaveDialog::setVersionAndPointFormat(const QString &version,
                                             unsigned int pointFormat) {
  int i = versionComboBox->findText(version);
  if (i >= 0) {
    QString fmtStr = QString::number(pointFormat);
    versionComboBox->setCurrentIndex(i);
    int j = pointFormatComboBox->findText(fmtStr);
    if (j >= 0) {
      pointFormatComboBox->setCurrentIndex(j);
    }
  }
}

void LasSaveDialog::setOptimalScale(const CCVector3d &optimalScale) {
  bestAccuracyLabel_2->setText(QString("(%1, %2,%3)")
                                   .arg(optimalScale.x)
                                   .arg(optimalScale.y)
                                   .arg(optimalScale.z));
}

void LasSaveDialog::setSavedScale(const CCVector3d &savedScale) {
  origAccuracyLabel_2->setText(QString("(%1, %2, %3)")
                                   .arg(savedScale.x)
                                   .arg(savedScale.y)
                                   .arg(savedScale.z));
  origRadioButton_2->setEnabled(true);
  origRadioButton_2->setChecked(true);
}

unsigned int LasSaveDialog::selectedPointFormat() const {
  std::string s = pointFormatComboBox->currentText().toStdString();
  return pointFormatComboBox->currentText().toUInt();
}

unsigned int LasSaveDialog::selectedVersionMinor() const {
  return versionComboBox->currentText().splitRef('.').at(1).toUInt();
}

bool LasSaveDialog::shouldSaveColors() const {
  return !rgbCheckBox->isHidden() && rgbCheckBox->isChecked();
}
//
// bool LasSaveDialog::shouldSaveWaveform() const
//{
//    return waveformCheckBox->isChecked();
//}
//
CCVector3d LasSaveDialog::chosenScale() const {
  const auto vectorFromString = [](const QString &string) -> CCVector3d {
    QVector<QStringRef> splits = string.splitRef(',');
    if (splits.size() == 3) {
      double x = splits[0].right(splits[0].size() - 1).toDouble();
      double y = splits[1].toDouble();
      double z = splits[2].left(splits[2].size() - 1).toDouble();
      return {x, y, z};
    }
    return {};
  };
  if (bestRadioButton_2->isChecked()) {
    QString text = bestAccuracyLabel_2->text();
    return vectorFromString(text);
  } else if (origRadioButton_2->isChecked()) {
    QString text = origAccuracyLabel_2->text();
    return vectorFromString(text);
  } else if (customRadioButton_2->isChecked()) {
    double value = customScaleDoubleSpinBox_2->value();
    return {value, value, value};
  }
  return {};
}

LasField::Vector LasSaveDialog::fieldsToSave() const {
  LasField::Vector fields;

  m_scalarFieldForm->fillFieldsToSave(fields);

  // TODO extra bytes to save
  return fields;
}

void LasSaveDialog::setExtraScalarFields(
    const std::vector<SavedExtraField> &savedExtraFields) {
  m_savedExtraFields = savedExtraFields;
  // we need to refresh
  handleSelectedPointFormatChange(selectedPointFormat());
}
