#ifndef CLOUDCOMPAREPROJECTS_EXTRASCALARFIELDROW_H
#define CLOUDCOMPAREPROJECTS_EXTRASCALARFIELDROW_H

#include <QApplication>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStringListModel>
#include <QWidget>
#include <QtDebug>

#include <ccPointCloud.h>

#include <pdal/DimUtil.hpp>

#include "../LasFields.h"

inline void SetWarningIcon(QLabel &label) {
  label.setPixmap(
      QApplication::style()->standardPixmap(QStyle::SP_MessageBoxWarning));
}

inline void SetOkIcon(QLabel &label) {
  label.setPixmap(
      QApplication::style()->standardPixmap(QStyle::SP_DialogApplyButton));
}

class ExtraScalarFieldRow : public QWidget {
  Q_OBJECT
public:
  explicit ExtraScalarFieldRow(QWidget *parent = nullptr)
      : QWidget(parent), m_removeBtn(new QPushButton),
        m_ccScalarFieldNameComboBox(new QComboBox),
        m_typeComboBox(new QComboBox), m_warningLabel(new QLabel) {
    auto *layout = new QHBoxLayout;
    layout->setContentsMargins(0, 2, 0, 2);
    layout->addWidget(m_removeBtn);
    layout->addWidget(m_ccScalarFieldNameComboBox);
    layout->addWidget(m_typeComboBox);
    layout->addWidget(m_warningLabel);
    setLayout(layout);

    m_removeBtn->setIcon(
        QApplication::style()->standardPixmap(QStyle::SP_DialogCancelButton));
    m_removeBtn->setFixedSize(32, 32);

    m_warningLabel->setToolTip(QStringLiteral(
        "Some of the values of the selected scalar field are out "
        "of range for the selected type"));
    SetWarningIcon(*m_warningLabel);
    m_warningLabel->setFixedWidth(32);
    SetOkIcon(*m_warningLabel);

    connect(m_ccScalarFieldNameComboBox, &QComboBox::currentTextChanged, this,
            &ExtraScalarFieldRow::handleSelectedScalarFieldChanged);
    connect(m_typeComboBox, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &ExtraScalarFieldRow::handleSelectedTypeChanged);
    connect(m_removeBtn, &QPushButton::clicked, this,
            &ExtraScalarFieldRow::deleteRequested);
  }

  QString currentScalarFieldName() const {
    return m_ccScalarFieldNameComboBox->currentText();
  }

  pdal::Dimension::Type pdalType() const { return m_pdalType; }

  LasField::Range range() const { return m_range; }

  void setCloud(const ccPointCloud *cloud) { m_cloud = cloud; }

  void setScalarFieldNamesModel(QStringListModel *model) {
    m_ccScalarFieldNameComboBox->setModel(model);
  }

  void setTypeComboBoxModel(QStringListModel *model) {
    m_typeComboBox->setModel(model);
    m_typeComboBox->setCurrentIndex(model->stringList().indexOf("float"));
  }

  void setCurrentScalarFieldIndex(int i) {
    m_ccScalarFieldNameComboBox->setCurrentIndex(i);
  }

  void setType(pdal::Dimension::Type type) {
    m_pdalType = type;
    m_typeComboBox->setCurrentText(pdalTypeToTypeName(type));
  }

Q_SIGNALS:
  void deleteRequested();

private:
  static QString pdalTypeToTypeName(pdal::Dimension::Type type) {
    switch (type) {
    case pdal::Dimension::Type::Unsigned8:
      return "uint8_t";
    case pdal::Dimension::Type::Unsigned16:
      return "uint16_t";
    case pdal::Dimension::Type::Unsigned32:
      return "uint32_t";
    case pdal::Dimension::Type::Unsigned64:
      return "uint64_t";
    case pdal::Dimension::Type::Signed8:
      return "int8_t";
    case pdal::Dimension::Type::Signed16:
      return "int16_t";
    case pdal::Dimension::Type::Signed32:
      return "int32_t";
    case pdal::Dimension::Type::Signed64:
      return "int64_t";
    case pdal::Dimension::Type::Float:
      return "float";
    case pdal::Dimension::Type::Double:
      return "double";
    case pdal::Dimension::Type::None:
    default:
      return "";
    }
  }

  static pdal::Dimension::Type typeNameToPdalType(const QString &typeName) {
    if (typeName == "uint8_t") {
      return pdal::Dimension::Type::Unsigned8;
    }
    if (typeName == "uint16_t") {
      return pdal::Dimension::Type::Unsigned16;
    }
    if (typeName == "uint32_t") {
      return pdal::Dimension::Type::Unsigned32;
    }
    if (typeName == "uint64_t") {
      return pdal::Dimension::Type::Unsigned64;
    }

    if (typeName == "int8_t") {
      return pdal::Dimension::Type::Signed8;
    }
    if (typeName == "int16_t") {
      return pdal::Dimension::Type::Signed16;
    }
    if (typeName == "int32_t") {
      return pdal::Dimension::Type::Signed32;
    }
    if (typeName == "int64_t") {
      return pdal::Dimension::Type::Signed64;
    }

    if (typeName == "float") {
      return pdal::Dimension::Type::Float;
    }

    if (typeName == "double") {
      return pdal::Dimension::Type::Double;
    }

    qCritical() << typeName << " not handled\n";
    return pdal::Dimension::Type::Float;
  }
  void handleSelectedScalarFieldChanged(const QString &newName) {
    if (!m_cloud) {
      return;
    }

    if (newName.isEmpty()) {
      Q_EMIT deleteRequested();
      return;
    }

    // TODO autoselect type ?
    int sfIdx =
        m_cloud->getScalarFieldIndexByName(newName.toStdString().c_str());
    if (sfIdx == -1) {
      qCritical() << "Somehow the cloud doesn't have the same fields as it "
                     "used to ?\n";
      return;
    }
    m_currentCcSf =
        static_cast<ccScalarField *>(m_cloud->getScalarField(sfIdx));
    checkIsRange();
  }

  void handleSelectedTypeChanged(int index) {
    Q_UNUSED(index)
    m_pdalType = typeNameToPdalType(m_typeComboBox->currentText());
    m_range = LasField::Range::ForTypeName(m_typeComboBox->currentText());
    checkIsRange();
  }

  void checkIsRange() {
    if (!m_currentCcSf) {
      return;
    }

    if (!(m_currentCcSf->getMin() >= m_range.min &&
          m_currentCcSf->getMax() <= m_range.max)) {
      SetWarningIcon(*m_warningLabel);
    } else {
      SetOkIcon(*m_warningLabel);
    }
  }

private:
  QPushButton *m_removeBtn;
  QComboBox *m_ccScalarFieldNameComboBox;
  QComboBox *m_typeComboBox;
  QLabel *m_warningLabel;
  LasField::Range m_range = LasField::Range::ForType<ScalarType>();
  pdal::Dimension::Type m_pdalType{pdal::Dimension::Type::None};
  const ccScalarField *m_currentCcSf{nullptr};
  const ccPointCloud *m_cloud{nullptr};
};

#endif // CLOUDCOMPAREPROJECTS_EXTRASCALARFIELDROW_H
