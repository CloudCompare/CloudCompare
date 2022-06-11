#ifndef CLOUDCOMPAREPROJECTS_LASSCALARFIELDFORM_H
#define CLOUDCOMPAREPROJECTS_LASSCALARFIELDFORM_H

#include <QVBoxLayout>
#include <QWidget>

#include "ScalarFieldRow.h"

class LasScalarFieldForm : public QWidget {

public:
  QStringListModel *m_ccScalarFieldsNamesModel{nullptr};
  const ccPointCloud *m_cloud{nullptr};

  explicit LasScalarFieldForm(QWidget *parent = nullptr);

  void reset(const LasField::Vector &lasScalarFields);

  bool isSelectedInOneOfTheRow(const QString &ccScalarFieldName) const;

  void fillFieldsToSave(LasField::Vector &fieldsToSave) const;

private:
  void setupUi();

  ScalarFieldRow *findOrCreateUsableRow();

  ScalarFieldRow *addNewRow();

  void hideAllRows();

private:
  QList<ScalarFieldRow *> m_rows;
  QVBoxLayout *m_rowsLayout;
};

#endif // CLOUDCOMPAREPROJECTS_LASSCALARFIELDFORM_H
