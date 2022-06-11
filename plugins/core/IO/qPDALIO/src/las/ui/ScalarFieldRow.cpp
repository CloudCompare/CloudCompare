#include "ScalarFieldRow.h"

void ScalarFieldRow::handleSelectedScalarFieldChanged(
    const QString &ccScalarFieldName) {
  if (ccScalarFieldName.isEmpty()) {
    m_warningLabel->setPixmap(QPixmap());
    return;
  }

  int sfIdx = m_cloud->getScalarFieldIndexByName(
      ccScalarFieldName.toStdString().c_str());
  if (sfIdx == -1) {
    qCritical() << "Somehow the cloud doesn't have the same fields as it "
                   "used to ?\n";
    return;
  }
  CCCoreLib::ScalarField *scalarField = m_cloud->getScalarField(sfIdx);
  m_field.sf = static_cast<ccScalarField *>(scalarField);

  if (scalarField->getMin() < m_field.range.min ||
      scalarField->getMax() > m_field.range.max) {
    ccLog::PrintDebug(
        "Field %s with range [%f, %f] does not fit in range [%f, %f]",
        scalarField->getName(), scalarField->getMin(), scalarField->getMax(),
        m_field.range.min, m_field.range.max);
    m_warningLabel->setToolTip(QStringLiteral(
        "Some of the values of the selected scalar field are out "
        "of range for the selected type"));
    SetWarningIcon(*m_warningLabel);
  } else {
    SetOkIcon(*m_warningLabel);
  }
}
