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

#include "LasOpenDialog.h"

static QListWidgetItem *CreateItem(const char *name) {
  auto item = new QListWidgetItem(name);
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
  item->setCheckState(Qt::Checked);
  return item;
}

static QString PrettyFormatNumber(uint64_t numPoints) {
  QString num;
  num.reserve(15);
  while (numPoints != 0) {
    if (numPoints >= 1000) {
      num.prepend(QString::number(numPoints % 1000).rightJustified(3, '0'));
    } else {
      num.prepend(QString::number(numPoints % 1000));
    }
    num.prepend(" ");
    numPoints /= 1'000;
  }
  return num;
}

bool IsCheckedIn(const QString &name, const QListWidget &list) {
  for (int i = 0; i < list.count(); ++i) {
    if (list.item(i)->text() == name) {
      return list.item(i)->checkState() == Qt::Checked;
    }
  }
  return false;
}

// TODO use std::remove_if
template <typename T, typename Pred>
void RemoveFalse(std::vector<T> &vec, Pred predicate) {
  auto firstFalse = std::partition(vec.begin(), vec.end(), predicate);

  if (firstFalse != vec.end()) {
    vec.erase(firstFalse, vec.end());
  }
}

LasOpenDialog::LasOpenDialog(QWidget *parent) : QDialog(parent) {
  setupUi(this);
  specialFieldsFrame->setHidden(true);

  connect(applyButton, &QPushButton::clicked, this, &QDialog::accept);
  connect(applyAllButton, &QPushButton::clicked, this, &QDialog::accept);
  connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

void LasOpenDialog::setInfo(int versionMinor, int pointFormatId,
                            uint64_t numPoints) {
  versionLabelValue->setText(
      QString("1.%1").arg(QString::number(versionMinor)));
  pointFormatLabelValue->setText(QString::number(pointFormatId));
  numPointsLabelValue->setText(PrettyFormatNumber(numPoints));
}

void LasOpenDialog::setAvailableScalarFields(
    const LasField::Vector &scalarFields) {
  extraScalarFieldsFrame->hide();
  for (const LasField &lasScalarField : scalarFields) {
    if (lasScalarField.ccId == LasField::Id::Extra) {
      extraScalarFieldsFrame->show();
      availableExtraScalarFields->addItem(
          CreateItem(lasScalarField.name.c_str()));
    }
    availableScalarFields->addItem(CreateItem(lasScalarField.name.c_str()));
  }

  if (!extraScalarFieldsFrame->isHidden()) {
    int size = (availableExtraScalarFields->sizeHintForRow(0) +
                2 * availableExtraScalarFields->frameWidth()) *
               availableExtraScalarFields->count();
    availableExtraScalarFields->setMaximumHeight(size);
  }
}

void LasOpenDialog::setHasColors(bool state) {
  specialFieldsFrame->setHidden(!state);
  colorsCheckbox->setEnabled(state);
  colorsCheckbox->setChecked(state);
}

bool LasOpenDialog::loadColors() const {
  return colorsCheckbox->isEnabled() && colorsCheckbox->isChecked();
}

void LasOpenDialog::filterOutNotChecked(LasField::Vector &scalarFields) {
  const auto isFieldSelected = [this](const auto &field) {
    return isChecked(field);
  };

  RemoveFalse(scalarFields, isFieldSelected);
}

bool LasOpenDialog::isChecked(const LasField &lasScalarField) const {
  QString name = lasScalarField.name.c_str();
  if (lasScalarField.ccId == LasField::Id::Extra) {
    return IsCheckedIn(name, *availableExtraScalarFields);
  } else {
    return IsCheckedIn(name, *availableScalarFields);
  }
}