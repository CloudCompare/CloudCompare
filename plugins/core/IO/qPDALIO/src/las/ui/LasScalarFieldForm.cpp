#include "LasScalarFieldForm.h"

LasScalarFieldForm::LasScalarFieldForm(QWidget *parent)
    : QWidget(parent), m_rowsLayout(new QVBoxLayout) {
  setupUi();
}

void LasScalarFieldForm::setupUi() {
  auto *centralLayout = new QVBoxLayout;

  centralLayout->addItem(m_rowsLayout);
  centralLayout->addStretch();

  setLayout(centralLayout);
}

void LasScalarFieldForm::reset(const LasField::Vector &lasScalarFields) {
  hideAllRows();
  const QStringList cloudScalarFieldsNames =
      m_ccScalarFieldsNamesModel->stringList();

  for (size_t i{0}; i < lasScalarFields.size(); ++i) {
    const LasField &field = lasScalarFields[i];
    ScalarFieldRow *row = findOrCreateUsableRow();

    int matchingCCScalarFieldIndex =
        cloudScalarFieldsNames.indexOf(field.name.c_str());

    row->m_field = field;
    row->setName(QString::fromStdString(field.name));
    row->setCurrentIndex(matchingCCScalarFieldIndex);
    row->show();
    row->setEnabled(true);
  }
}

void LasScalarFieldForm::fillFieldsToSave(
    LasField::Vector &fieldsToSave) const {
  for (const ScalarFieldRow *row : m_rows) {
    if (row->isHidden()) {
      return;
    }

    if (row->m_field.sf) {
      fieldsToSave.push_back(row->m_field);
    }
  }
}

ScalarFieldRow *LasScalarFieldForm::findOrCreateUsableRow() {
  for (ScalarFieldRow *row : m_rows) {
    if (row->isHidden()) {
      return row;
    }
  }

  return addNewRow();
}

ScalarFieldRow *LasScalarFieldForm::addNewRow() {
  auto *row = new ScalarFieldRow(this);
  row->m_ccScalarFieldNameComboBox->setModel(m_ccScalarFieldsNamesModel);
  row->m_cloud = m_cloud;

  m_rowsLayout->addWidget(row);
  m_rows.append(row);
  return row;
}

bool LasScalarFieldForm::isSelectedInOneOfTheRow(
    const QString &ccScalarFieldName) const {
  for (ScalarFieldRow *row : m_rows) {
    if (row->currentScalarFieldName() == ccScalarFieldName) {
      return true;
    }

    if (row->isHidden()) {
      break;
    }
  }
  return false;
}

void LasScalarFieldForm::hideAllRows() {
  for (ScalarFieldRow *row : m_rows) {
    row->hide();
    row->setEnabled(false);
  }
}
