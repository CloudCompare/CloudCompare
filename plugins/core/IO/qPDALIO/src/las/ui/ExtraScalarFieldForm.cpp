#include "ExtraScalarFieldForm.h"

ExtraScalarFieldForm::ExtraScalarFieldForm(QWidget *parent)
    : QWidget(parent), m_extraFieldsTypeModel(new QStringListModel),
      m_rowsLayout(new QVBoxLayout), m_addButton(new QPushButton(this)) {

  m_addButton->setText(QString::fromUtf8("Add Extra Scalar Field"));
  connect(m_addButton, &QPushButton::clicked, this,
          qOverload<>(&ExtraScalarFieldForm::addRow));

  auto *centralLayout = new QVBoxLayout;
  centralLayout->addWidget(m_addButton);
  centralLayout->addItem(m_rowsLayout);
  centralLayout->addStretch();
  setLayout(centralLayout);

  QStringList extraFieldsTypes;
  extraFieldsTypes << "uint8_t"
                   << "uint16_t"
                   << "uint32_t"
                   << "uint64_t"
                   << "int8_t"
                   << "int16_t"
                   << "int32_t"
                   << "int64_t"
                   << "float"
                   << "double";
  m_extraFieldsTypeModel->setStringList(extraFieldsTypes);
}

void ExtraScalarFieldForm::reset() {
  for (ExtraScalarFieldRow *row : m_rows) {
    row->hide();
    row->setEnabled(false);
  }
}

void ExtraScalarFieldForm::addRow(const QString &scalarFieldName) {
  ExtraScalarFieldRow *row = findOrCreateUsableRow();
  row->setCurrentScalarFieldIndex(
      m_ccScalarFieldsNamesModel->stringList().indexOf(scalarFieldName));
  row->show();
  row->setEnabled(true);
}

void ExtraScalarFieldForm::addRowWithType(const QString &scalarFieldName,
                                          pdal::Dimension::Type type) {
  ExtraScalarFieldRow *row = findOrCreateUsableRow();
  row->setCurrentScalarFieldIndex(
      m_ccScalarFieldsNamesModel->stringList().indexOf(scalarFieldName));
  row->setType(type);
  row->show();
  row->setEnabled(true);
}

ExtraScalarFieldRow *ExtraScalarFieldForm::addNewRow() {
  auto *row = new ExtraScalarFieldRow(this);
  row->setScalarFieldNamesModel(m_ccScalarFieldsNamesModel);
  row->setTypeComboBoxModel(m_extraFieldsTypeModel);
  row->setCloud(m_cloud);
  connect(row, &ExtraScalarFieldRow::deleteRequested, this,
          &ExtraScalarFieldForm::handleRemoveRow);
  m_rows.append(row);
  m_rowsLayout->addWidget(row);

  return row;
}

void ExtraScalarFieldForm::addRow() {
  addRow(m_ccScalarFieldsNamesModel->stringList().first());
}

void ExtraScalarFieldForm::handleRemoveRow() {
  auto *senderRow = qobject_cast<ExtraScalarFieldRow *>(sender());
  m_rowsLayout->removeWidget(senderRow);
  m_rowsLayout->addWidget(senderRow);

  senderRow->hide();
  senderRow->setEnabled(false);

  int idx = m_rows.indexOf(senderRow);
  Q_ASSERT(idx != -1);
  m_rows.removeAt(idx);
  m_rows.append(senderRow);
}

ExtraScalarFieldRow *ExtraScalarFieldForm::findOrCreateUsableRow() {
  for (ExtraScalarFieldRow *row : m_rows) {
    if (row->isHidden()) {
      return row;
    }
  }

  return addNewRow();
}
