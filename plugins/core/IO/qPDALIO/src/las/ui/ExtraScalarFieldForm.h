#ifndef CLOUDCOMPAREPROJECTS_EXTRASCALARFIELDFORM_H
#define CLOUDCOMPAREPROJECTS_EXTRASCALARFIELDFORM_H

#include <QList>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <ccPointCloud.h>

#include "ExtraScalarFieldRow.h"

class ExtraScalarFieldForm : public QWidget {
  Q_OBJECT
public:
  explicit ExtraScalarFieldForm(QWidget *parent = nullptr);

  // TODO rename
  void setShit(const ccPointCloud *cloud, QStringListModel *m) {
    m_cloud = cloud;
    m_ccScalarFieldsNamesModel = m;
  }

  void reset();

  void addRow(const QString &scalarFieldName);

  void addRowWithType(const QString &scalarFieldName,
                      pdal::Dimension::Type type);

private:
  ExtraScalarFieldRow *addNewRow();

  void addRow();

  void handleRemoveRow();

  ExtraScalarFieldRow *findOrCreateUsableRow();

private:
  QList<ExtraScalarFieldRow *> m_rows;
  QVBoxLayout *m_rowsLayout;
  QPushButton *m_addButton;

  QStringListModel *m_extraFieldsTypeModel;
  QStringListModel *m_ccScalarFieldsNamesModel{nullptr};
  const ccPointCloud *m_cloud{nullptr};
};

#endif // CLOUDCOMPAREPROJECTS_EXTRASCALARFIELDFORM_H
