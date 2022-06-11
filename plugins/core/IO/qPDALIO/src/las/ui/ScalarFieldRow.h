#ifndef CLOUDCOMPAREPROJECTS_SCALARFIELDROW_H
#define CLOUDCOMPAREPROJECTS_SCALARFIELDROW_H

#include <QApplication>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStringListModel>
#include <QWidget>
#include <QtDebug>

#include <ccPointCloud.h>

#include "../LasFields.h"
#include "ExtraScalarFieldRow.h"

class LasSaveDialog;
class LasScalarFieldForm;

class ScalarFieldRow : public QWidget {
  friend LasScalarFieldForm;
  Q_OBJECT
public:
  explicit ScalarFieldRow(QWidget *parent = nullptr)
      : QWidget(parent), m_nameLabel(new QLabel),
        m_ccScalarFieldNameComboBox(new QComboBox), m_warningLabel(new QLabel) {

    auto *layout = new QHBoxLayout;
    layout->setContentsMargins(0, 2, 0, 2);
    layout->addWidget(m_nameLabel);
    layout->addWidget(m_ccScalarFieldNameComboBox);
    layout->addWidget(m_warningLabel);
    setLayout(layout);

    m_warningLabel->setFixedWidth(32);

    connect(m_ccScalarFieldNameComboBox, &QComboBox::currentTextChanged, this,
            &ScalarFieldRow::handleSelectedScalarFieldChanged);
  }

  void setName(const QString &name) { m_nameLabel->setText(name); }

  QString lasScalarFieldName() const { return m_nameLabel->text(); }

  QString currentScalarFieldName() const {
    return m_ccScalarFieldNameComboBox->currentText();
  }

  void setCurrentIndex(int index) {
    m_ccScalarFieldNameComboBox->setCurrentIndex(index);
    handleSelectedScalarFieldChanged(
        m_ccScalarFieldNameComboBox->currentText());
  }

private:
  void handleSelectedScalarFieldChanged(const QString &ccScalarFieldName);

private:
  QLabel *m_nameLabel;
  QComboBox *m_ccScalarFieldNameComboBox;
  QLabel *m_warningLabel;
  const ccPointCloud *m_cloud{nullptr};
  LasField m_field;
};
#endif // CLOUDCOMPAREPROJECTS_SCALARFIELDROW_H
