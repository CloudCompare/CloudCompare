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
#include "LasDetails.h"

#include <QStringListModel>

#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

constexpr int ExtraScalarFieldsTabIndex = 2;

class MappingLabel : public QWidget
{
  public:
    explicit MappingLabel(QWidget *parent = nullptr)
        : QWidget(parent), m_nameLabel(new QLabel), m_statusLabel(new QLabel)
    {
        auto *layout = new QHBoxLayout;
        layout->addWidget(m_nameLabel);
        layout->addWidget(m_statusLabel);
        setLayout(layout);
    }

    void setName(const QString &name)
    {
        m_nameLabel->setText(name);
    }

    QString name() const
    {
        return m_nameLabel->text();
    }

    void setWarning(const QString &message)
    {
        m_statusLabel->setPixmap(QApplication::style()->standardPixmap(QStyle::SP_MessageBoxWarning));
        m_statusLabel->setToolTip(message);
    }

    bool hasWarning() const
    {
        return !m_statusLabel->toolTip().isEmpty();
    }

    void clearWarning()
    {
        m_statusLabel->setPixmap(QPixmap());
        m_statusLabel->setToolTip(QString());
    }

  private:
    QLabel *m_nameLabel;
    QLabel *m_statusLabel;
};

LasSaveDialog::LasSaveDialog(ccPointCloud *cloud, QWidget *parent)
    : QDialog(parent), m_cloud(cloud), m_comboBoxModel(new QStringListModel)
{
    setupUi(this);
    origRadioButton_2->setEnabled(false);
    customScaleDoubleSpinBox_2->setEnabled(true);
    bestRadioButton_2->setChecked(true);

    connect(versionComboBox,
            (void(QComboBox::*)(const QString &))(&QComboBox::currentIndexChanged),
            this,
            &LasSaveDialog::handleSelectedVersionChange);

    connect(pointFormatComboBox,
            (void(QComboBox::*)(int))(&QComboBox::currentIndexChanged),
            this,
            &LasSaveDialog::handleSelectedPointFormatChange);

    for (const char *versionStr : AvailableVersions)
    {
        versionComboBox->addItem(versionStr);
    }
    versionComboBox->setCurrentIndex(0);

    QStringList cloudScalarFieldsNames;
    cloudScalarFieldsNames << QString();
    for (unsigned int i = 0; i < m_cloud->getNumberOfScalarFields(); ++i)
    {
        if (strcmp(m_cloud->getScalarFieldName(i), "Default") != 0)
        {
            cloudScalarFieldsNames << m_cloud->getScalarFieldName(i);
        }
    }
    m_comboBoxModel->setStringList(cloudScalarFieldsNames);
}

/// When the selected version changes, we need to update the combo box
/// of point format to match the ones supported by the version
void LasSaveDialog::handleSelectedVersionChange(const QString &version)
{
    const std::vector<unsigned int> *pointFormats = PointFormatsAvailableForVersion(qPrintable(version));
    if (pointFormats)
    {
        pointFormatComboBox->clear();
        for (unsigned int fmt : *pointFormats)
        {
            pointFormatComboBox->addItem(QString::number(fmt));
        }
    }

    const int previousIndex = pointFormatComboBox->currentIndex();
    pointFormatComboBox->setCurrentIndex(0);
    if (previousIndex == 0)
    {
        // We have to force the call here
        // because the index did not change,
        // But the actual point format did
        handleSelectedPointFormatChange(0);
    }
}

/// When the user changes the ccScalarField it wants to save in the particular LAS field,
/// we check that the values are in range. If they are not we display a small warning
void LasSaveDialog::handleComboBoxChange(int index)
{
    if (index < 0)
    {
        return;
    }
    QObject *senderObject = sender();
    size_t senderIndex =
        std::distance(m_scalarFieldMapping.begin(),
                      std::find_if(m_scalarFieldMapping.begin(),
                                   m_scalarFieldMapping.end(),
                                   [senderObject](const std::pair<MappingLabel *, QComboBox *> &pair)
                                   { return pair.second == senderObject; }));

    if (qobject_cast<QComboBox *>(senderObject)->itemText(index).isEmpty())
    {
        m_scalarFieldMapping[senderIndex].first->clearWarning();
        return;
    }
    const QString scalarFieldName = m_scalarFieldMapping[senderIndex].first->name();
    const QString ccScalarFieldName = m_scalarFieldMapping[senderIndex].second->currentText();
    int sfIdx = m_cloud->getScalarFieldIndexByName(ccScalarFieldName.toStdString().c_str());
    if (sfIdx == -1)
    {
        Q_ASSERT(false);
        return;
    }
    const CCCoreLib::ScalarField *scalarField = m_cloud->getScalarField(sfIdx);

    // TODO support QString equality
    LasScalarField::Id scalarFieldId =
        LasScalarField::IdFromName(scalarFieldName.toStdString().c_str(), selectedPointFormat());
    LasScalarField::Range range = LasScalarField::ValueRange(scalarFieldId);

    if (scalarField->getMin() < range.min || scalarField->getMax() > range.max)
    {
        m_scalarFieldMapping[senderIndex].first->setWarning(
            "Some values are out of range and will be truncated");
    }
    else
    {
        m_scalarFieldMapping[senderIndex].first->clearWarning();
    }

    size_t numWarnings = std::count_if(m_scalarFieldMapping.begin(),
                                       m_scalarFieldMapping.end(),
                                       [](const std::pair<MappingLabel *, QComboBox *> &pair)
                                       { return pair.first->hasWarning(); });

    if (numWarnings > 0)
    {
        tabWidget->setTabIcon(1, QApplication::style()->standardPixmap(QStyle::SP_MessageBoxWarning));
    }
    else
    {
        tabWidget->setTabIcon(1, QIcon());
    }
}

/// When the user changes the point format, we need to redo the
/// scalar field form.
void LasSaveDialog::handleSelectedPointFormatChange(int index)
{
    if (index < 0)
    {
        return;
    }

    const std::vector<unsigned int> *pointFormats =
        PointFormatsAvailableForVersion(qPrintable(versionComboBox->currentText()));

    if (!pointFormats)
    {
        Q_ASSERT_X(false, __func__, "User was allowed to select version with to point formats");
        return;
    }

    // Hide all the rows in our form
    for (int i{0}; i < scalarFieldFormLayout->rowCount(); ++i)
    {
        auto *label = scalarFieldFormLayout->itemAt(i, QFormLayout::LabelRole);
        auto *field = scalarFieldFormLayout->itemAt(i, QFormLayout::FieldRole);
        if (label != nullptr && label->widget() != nullptr)
        {
            label->widget()->hide();
        }
        if (field != nullptr && field->widget() != nullptr)
        {
            field->widget()->hide();
        }
    }

    unsigned int selectedPointFormat = (*pointFormats)[index];
    std::vector<LasScalarField> lasScalarFields = LasScalarFieldForPointFormat(selectedPointFormat);

    int numDeltaFields = scalarFieldFormLayout->rowCount() - static_cast<int>(lasScalarFields.size());

    if (numDeltaFields < 0)
    {
        // We have fewer rows in our form, create new ones
        for (int i{numDeltaFields}; i < 0; ++i)
        {
            auto *box = new QComboBox(this);
            box->setModel(m_comboBoxModel);
            connect(box,
                    qOverload<int>(&QComboBox::currentIndexChanged),
                    this,
                    &LasSaveDialog::handleComboBoxChange);

            auto *widget = new MappingLabel(this);
            scalarFieldFormLayout->addRow(widget, box);
            m_scalarFieldMapping.emplace_back(widget, box);
        }
    }

    Q_ASSERT(lasScalarFields.size() <= scalarFieldFormLayout->rowCount());

    QStringList cloudScalarFieldsNames = m_comboBoxModel->stringList();
    for (size_t i{0}; i < lasScalarFields.size(); ++i)
    {
        const LasScalarField &field = lasScalarFields[i];
        m_scalarFieldMapping[i].first->setName(field.name());
        m_scalarFieldMapping[i].first->clearWarning();
        m_scalarFieldMapping[i].second->setCurrentIndex(cloudScalarFieldsNames.indexOf(field.name()));

        auto *label = scalarFieldFormLayout->itemAt(static_cast<int>(i), QFormLayout::LabelRole);
        auto *fieldWidget = scalarFieldFormLayout->itemAt(static_cast<int>(i), QFormLayout::FieldRole);
        if (label != nullptr && label->widget() != nullptr)
        {
            label->widget()->show();
        }
        if (fieldWidget != nullptr && fieldWidget->widget() != nullptr)
        {
            fieldWidget->widget()->show();
        }
    }


    if (!HasRGB(selectedPointFormat) && !HasWaveform(selectedPointFormat))
    {
        specialScalarFieldFrame->hide();
        waveformCheckBox->setCheckState(Qt::Unchecked);
        rgbCheckBox->setCheckState(Qt::Unchecked);
    }
    else
    {
        specialScalarFieldFrame->show();
        if (HasRGB(selectedPointFormat))
        {
            rgbCheckBox->show();
            rgbCheckBox->setEnabled(m_cloud->hasColors());
            rgbCheckBox->setChecked(m_cloud->hasColors());
        }
        else
        {
            rgbCheckBox->hide();
        }

        if (HasWaveform(selectedPointFormat))
        {
            waveformCheckBox->show();
            waveformCheckBox->setEnabled(m_cloud->hasFWF());
            waveformCheckBox->setChecked(m_cloud->hasFWF());
        }
        else
        {
            waveformCheckBox->hide();
        }
    }
}

void LasSaveDialog::setVersionAndPointFormat(const QString &version, unsigned int pointFormat)
{
    int i = versionComboBox->findText(version);
    if (i >= 0)
    {
        QString fmtStr = QString::number(pointFormat);
        versionComboBox->setCurrentIndex(i);
        int j = pointFormatComboBox->findText(fmtStr);
        if (j >= 0)
        {
            pointFormatComboBox->setCurrentIndex(j);
        }
    }
}

void LasSaveDialog::setOptimalScale(const CCVector3d &optimalScale)
{
    bestAccuracyLabel_2->setText(
        QString("(%1, %2, %3)").arg(optimalScale.x).arg(optimalScale.y).arg(optimalScale.z));
}

void LasSaveDialog::setSavedScale(const CCVector3d &savedScale)
{
    origAccuracyLabel_2->setText(
        QString("(%1, %2, %3)").arg(savedScale.x).arg(savedScale.y).arg(savedScale.z));
    origRadioButton_2->setEnabled(true);
    origRadioButton_2->setChecked(true);
}

void LasSaveDialog::setExtraScalarFields(const std::vector<LasExtraScalarField> &extraScalarFields)
{
    if (extraScalarFields.empty())
    {
        tabWidget->setTabEnabled(ExtraScalarFieldsTabIndex, false);
        return;
    }
    tabWidget->setTabEnabled(ExtraScalarFieldsTabIndex, true);

    QStringList extraFieldsName;
    for (const LasExtraScalarField &field : extraScalarFields)
    {
        extraFieldsName << field.name;
    }

    auto *model = new QStringListModel;
    model->setStringList(extraFieldsName);
    extraScalarFieldView->setModel(model);
}

unsigned int LasSaveDialog::selectedPointFormat() const
{
    return pointFormatComboBox->currentText().toUInt();
}

unsigned int LasSaveDialog::selectedVersionMinor() const
{
    return versionComboBox->currentText().splitRef('.').at(1).toUInt();
}

bool LasSaveDialog::shouldSaveRGB() const
{
    return rgbCheckBox->isChecked();
}

bool LasSaveDialog::shouldSaveWaveform() const
{
    return waveformCheckBox->isChecked();
}

CCVector3d LasSaveDialog::chosenScale() const
{
    const auto vectorFromString = [](const QString &string) -> CCVector3d
    {
        QVector<QStringRef> splits = string.splitRef(',');
        if (splits.size() == 3)
        {
            double x = splits[0].right(splits[0].size() - 1).toDouble();
            double y = splits[1].toDouble();
            double z = splits[2].left(splits[2].size() - 1).toDouble();
            return {x, y, z};
        }
        return {};
    };
    if (bestRadioButton_2->isChecked())
    {
        QString text = bestAccuracyLabel_2->text();
        return vectorFromString(text);
    }
    else if (origRadioButton_2->isChecked())
    {
        QString text = origAccuracyLabel_2->text();
        return vectorFromString(text);
    }
    else if (customRadioButton_2->isChecked())
    {
        double value = customScaleDoubleSpinBox_2->value();
        return {value, value, value};
    }
    return {};
}

std::vector<LasScalarField> LasSaveDialog::fieldsToSave() const
{
    std::vector<LasScalarField> fields;
    fields.reserve(scalarFieldFormLayout->rowCount());
    unsigned int pointFormat = selectedPointFormat();

    for (const auto &item : m_scalarFieldMapping)
    {
        ccScalarField *sf = nullptr;
        if (item.second->currentIndex() > 0)
        {
            sf = static_cast<ccScalarField *>(m_cloud->getScalarField(
                m_cloud->getScalarFieldIndexByName(qPrintable(item.second->currentText()))));
            const std::string name = item.first->name().toStdString();
            fields.emplace_back(LasScalarField::IdFromName(name.c_str(), pointFormat), sf);
        }
    }

    return fields;
}
