#include "LasExtraScalarFieldCard.h"

#include <ccPointCloud.h>
#include <ccScalarField.h>

LasExtraScalarFieldCard::LasExtraScalarFieldCard(QWidget *parent) : QWidget(parent)
{
    setupUi(this);
    scalarFieldCardFrame->setFrameShape(QFrame::Shape::Panel);
    scalarFieldCardFrame->setFrameShadow(QFrame::Shadow::Plain);
    scalarFieldCardFrame->setLineWidth(1);

    connect(radioButton1,
            &QRadioButton::clicked,
            this,
            [this](bool checked)
            {
                firstScalarFieldComboBox->setVisible(true);
                secondScalarFieldComboBox->setVisible(false);
                thirdScalarFieldComboBox->setVisible(false);

                firstScalarFieldScaleSpinBox->setEnabled(true);
                secondScalarFieldScaleSpinBox->setEnabled(false);
                thirdScalarFieldScaleSpinBox->setEnabled(false);

                firstScalarFieldOffsetSpinBox->setEnabled(true);
                secondScalarFieldOffsetSpinBox->setEnabled(false);
                thirdScalarFieldOffsetSpinBox->setEnabled(false);
            });

    connect(radioButton2,
            &QRadioButton::clicked,
            this,
            [this](bool checked)
            {
                firstScalarFieldComboBox->setVisible(true);
                secondScalarFieldComboBox->setVisible(true);
                thirdScalarFieldComboBox->setVisible(false);

                firstScalarFieldScaleSpinBox->setEnabled(true);
                secondScalarFieldScaleSpinBox->setEnabled(true);
                thirdScalarFieldScaleSpinBox->setEnabled(false);

                firstScalarFieldOffsetSpinBox->setEnabled(true);
                secondScalarFieldOffsetSpinBox->setEnabled(true);
                thirdScalarFieldOffsetSpinBox->setEnabled(false);
            });

    connect(radioButton3,
            &QRadioButton::clicked,
            this,
            [this](bool checked)
            {
                firstScalarFieldComboBox->setVisible(true);
                secondScalarFieldComboBox->setVisible(true);
                thirdScalarFieldComboBox->setVisible(true);

                firstScalarFieldScaleSpinBox->setEnabled(true);
                secondScalarFieldScaleSpinBox->setEnabled(true);
                thirdScalarFieldScaleSpinBox->setEnabled(true);

                firstScalarFieldOffsetSpinBox->setEnabled(true);
                secondScalarFieldOffsetSpinBox->setEnabled(true);
                thirdScalarFieldOffsetSpinBox->setEnabled(true);
            });

    connect(firstScalarFieldComboBox,
            &QComboBox::currentTextChanged,
            this,
            [this](const QString &text)
            {
                if (radioButton1->isChecked())
                {
                    nameEdit->setText(text);
                }
            });

    advancedOptionFrame->hide();
    scaledCheckBox->setChecked(false);
    scalingOptionGroup->setEnabled(false);
    connect(advancedOptionsButton,
            &QPushButton::clicked,
            this,
            [this]()
            {
                if (this->advancedOptionFrame->isHidden())
                {
                    this->advancedOptionFrame->show();
                }
                else
                {
                    this->advancedOptionFrame->hide();
                }
            });
    connect(scaledCheckBox, &QCheckBox::stateChanged, scalingOptionGroup, &QGroupBox::setEnabled);
    reset();
}

void LasExtraScalarFieldCard::reset()
{
    radioButton1->setChecked(true);
    emit radioButton1->clicked(true);

    nameEdit->setText("");
    if (advancedOptionsButton->isChecked())
    {
        advancedOptionsButton->setChecked(false);
        emit advancedOptionsButton->clicked();
    }

#ifdef CC_CORE_LIB_USES_DOUBLE
    const char *defaultType = "float64";
#else
    const char *defaultType = "float32";
#endif
    typeComboBox->setCurrentText(defaultType);

    firstScalarFieldComboBox->setCurrentIndex(0);
    secondScalarFieldComboBox->setCurrentIndex(0);
    thirdScalarFieldComboBox->setCurrentIndex(0);
}

void LasExtraScalarFieldCard::fillFrom(const LasExtraScalarField &field)
{
    Q_ASSERT(field.ccName.empty()); // TODO

    nameEdit->setText(field.name);
    switch (field.numElements())
    {
    case 1:
        radioButton1->setChecked(true);
        emit radioButton1->clicked(true);
        Q_ASSERT(field.scalarFields[0] != nullptr);
        firstScalarFieldComboBox->setCurrentText(field.scalarFields[0]->getName());
        break;
    case 2:
        radioButton2->setChecked(true);
        emit radioButton2->clicked(true);
        Q_ASSERT(field.scalarFields[0] != nullptr);
        firstScalarFieldComboBox->setCurrentText(field.scalarFields[0]->getName());
        Q_ASSERT(field.scalarFields[1] != nullptr);
        secondScalarFieldComboBox->setCurrentText(field.scalarFields[1]->getName());
        break;
    case 3:
        radioButton3->setChecked(true);
        emit radioButton3->clicked(true);
        firstScalarFieldComboBox->setCurrentText(field.scalarFields[0]->getName());
        Q_ASSERT(field.scalarFields[1] != nullptr);
        secondScalarFieldComboBox->setCurrentText(field.scalarFields[1]->getName());
        Q_ASSERT(field.scalarFields[1] != nullptr);
        secondScalarFieldComboBox->setCurrentText(field.scalarFields[1]->getName());
        Q_ASSERT(field.scalarFields[2] != nullptr);
        thirdScalarFieldComboBox->setCurrentText(field.scalarFields[2]->getName());
        break;
    default:
        Q_ASSERT(false);
        return;
    }

    descriptionEdit->setText(field.description);
    scaledCheckBox->setChecked(false);
    emit scaledCheckBox->stateChanged(false);

    if (field.scaleIsRelevant())
    {
        scaledCheckBox->setChecked(true);
        emit scaledCheckBox->stateChanged(true);
        switch (field.numElements())
        {
        case 3:
            thirdScalarFieldScaleSpinBox->setValue(field.scales[2]);
        case 2:
            secondScalarFieldScaleSpinBox->setValue(field.scales[1]);
        case 1:
            firstScalarFieldScaleSpinBox->setValue(field.scales[0]);
        }
    }

    if (field.offsetIsRelevant())
    {
        emit scaledCheckBox->stateChanged(true);
        switch (field.numElements())
        {
        case 3:
            thirdScalarFieldOffsetSpinBox->setValue(field.offsets[2]);
        case 2:
            secondScalarFieldOffsetSpinBox->setValue(field.offsets[1]);
        case 1:
            firstScalarFieldOffsetSpinBox->setValue(field.offsets[0]);
        }
    }
    switch (field.type)
    {
    case LasExtraScalarField::DataType::u8:
    case LasExtraScalarField::DataType::u8_2:
    case LasExtraScalarField::DataType::u8_3:
        typeComboBox->setCurrentText("uint8");
        break;
    case LasExtraScalarField::DataType::u16:
    case LasExtraScalarField::DataType::u16_2:
    case LasExtraScalarField::DataType::u16_3:
        typeComboBox->setCurrentText("uint16");
        break;
    case LasExtraScalarField::DataType::u32:
    case LasExtraScalarField::DataType::u32_2:
    case LasExtraScalarField::DataType::u32_3:
        typeComboBox->setCurrentText("uint32");
        break;

    case LasExtraScalarField::DataType::u64:
    case LasExtraScalarField::DataType::u64_2:
    case LasExtraScalarField::DataType::u64_3:
        typeComboBox->setCurrentText("uint64");
        break;
    case LasExtraScalarField::DataType::i8:
    case LasExtraScalarField::DataType::i8_2:
    case LasExtraScalarField::DataType::i8_3:
        typeComboBox->setCurrentText("int8");
        break;
    case LasExtraScalarField::DataType::i16:
    case LasExtraScalarField::DataType::i16_2:
    case LasExtraScalarField::DataType::i16_3:
        typeComboBox->setCurrentText("int16");
        break;
    case LasExtraScalarField::DataType::i32:
    case LasExtraScalarField::DataType::i32_2:
    case LasExtraScalarField::DataType::i32_3:
        typeComboBox->setCurrentText("int32");
        break;
    case LasExtraScalarField::DataType::i64:
    case LasExtraScalarField::DataType::i64_2:
    case LasExtraScalarField::DataType::i64_3:
        typeComboBox->setCurrentText("int64");
        break;
    case LasExtraScalarField::DataType::f32:
    case LasExtraScalarField::DataType::f32_2:
    case LasExtraScalarField::DataType::f32_3:
        typeComboBox->setCurrentText("float32");
        break;
    case LasExtraScalarField::DataType::f64:
    case LasExtraScalarField::DataType::f64_2:
    case LasExtraScalarField::DataType::f64_3:
        typeComboBox->setCurrentText("float64");
        break;
    default:
        typeComboBox->setCurrentText("float32");
        break;
    }
}

bool LasExtraScalarFieldCard::fillField(LasExtraScalarField &field, const ccPointCloud &pointCloud) const
{
    if (nameEdit->text().isEmpty())
    {
        return false;
    }

    const std::string stdName = nameEdit->text().toStdString();
    strncpy(field.name, stdName.c_str(), 32);

    const std::string stdDescription = descriptionEdit->text().toStdString();
    strncpy(field.description, stdDescription.c_str(), 32);

    field.type = dataType();

    if (scaledCheckBox->isChecked())
    {
        field.setScaleIsRelevant(true);
        field.setOffsetIsRelevant(true);

        if (radioButton1->isChecked() || radioButton2->isChecked() || radioButton3->isChecked())
        {
            field.scales[0] = firstScalarFieldScaleSpinBox->value();
            field.offsets[0] = firstScalarFieldOffsetSpinBox->value();
        }

        if (radioButton2->isChecked() || radioButton3->isChecked())
        {
            field.scales[1] = secondScalarFieldScaleSpinBox->value();
            field.offsets[1] = secondScalarFieldOffsetSpinBox->value();
        }

        if (radioButton3->isChecked())
        {
            field.scales[2] = thirdScalarFieldScaleSpinBox->value();
            field.offsets[2] = thirdScalarFieldOffsetSpinBox->value();
        }
    }
    else
    {
        field.setScaleIsRelevant(false);
        field.setOffsetIsRelevant(false);
    }

    if (radioButton1->isChecked() || radioButton2->isChecked() || radioButton3->isChecked())
    {
        const std::string sfName = firstScalarFieldComboBox->currentText().toStdString();
        int sfIndex = pointCloud.getScalarFieldIndexByName(sfName.c_str());
        if (sfIndex < 0)
        {
            return false;
        }
        field.scalarFields[0] = static_cast<ccScalarField *>(pointCloud.getScalarField(sfIndex));
    }

    if (radioButton2->isChecked() || radioButton3->isChecked())
    {
        const std::string sfName = secondScalarFieldComboBox->currentText().toStdString();
        int sfIndex = pointCloud.getScalarFieldIndexByName(sfName.c_str());
        if (sfIndex < 0)
        {
            return false;
        }
        field.scalarFields[1] = static_cast<ccScalarField *>(pointCloud.getScalarField(sfIndex));
    }

    if (radioButton3->isChecked())
    {
        const std::string sfName = thirdScalarFieldComboBox->currentText().toStdString();
        int sfIndex = pointCloud.getScalarFieldIndexByName(sfName.c_str());
        if (sfIndex < 0)
        {
            return false;
        }
        field.scalarFields[2] = static_cast<ccScalarField *>(pointCloud.getScalarField(sfIndex));
    }

    return true;
}

LasExtraScalarField::DataType LasExtraScalarFieldCard::dataType() const
{
    const QString selectedElementType = typeComboBox->currentText();

    if (radioButton1->isChecked())
    {
        if (selectedElementType == "uint8")
        {
            return LasExtraScalarField::DataType::u8;
        }

        if (selectedElementType == "uint16")
        {
            return LasExtraScalarField::DataType::u16;
        }

        if (selectedElementType == "uint32")
        {
            return LasExtraScalarField::DataType::u32;
        }

        if (selectedElementType == "uint64")
        {
            return LasExtraScalarField::DataType::u64;
        }

        if (selectedElementType == "int8")
        {
            return LasExtraScalarField::DataType::i8;
        }

        if (selectedElementType == "int16")
        {
            return LasExtraScalarField::DataType::i16;
        }

        if (selectedElementType == "int32")
        {
            return LasExtraScalarField::DataType::i32;
        }

        if (selectedElementType == "int64")
        {
            return LasExtraScalarField::DataType::i64;
        }

        if (selectedElementType == "float32")
        {
            return LasExtraScalarField::DataType::f32;
        }

        if (selectedElementType == "float64")
        {
            return LasExtraScalarField::DataType::f64;
        }
    }

    if (radioButton2->isChecked())
    {
        if (selectedElementType == "uint8")
        {
            return LasExtraScalarField::DataType::u8_2;
        }

        if (selectedElementType == "uint16")
        {
            return LasExtraScalarField::DataType::u16_2;
        }

        if (selectedElementType == "uint32")
        {
            return LasExtraScalarField::DataType::u32_2;
        }

        if (selectedElementType == "uint64")
        {
            return LasExtraScalarField::DataType::u64_2;
        }

        if (selectedElementType == "int8")
        {
            return LasExtraScalarField::DataType::i8_2;
        }

        if (selectedElementType == "int16")
        {
            return LasExtraScalarField::DataType::i16_2;
        }

        if (selectedElementType == "int32")
        {
            return LasExtraScalarField::DataType::i32_2;
        }

        if (selectedElementType == "int64")
        {
            return LasExtraScalarField::DataType::i64_2;
        }

        if (selectedElementType == "float32")
        {
            return LasExtraScalarField::DataType::f32_2;
        }

        if (selectedElementType == "float64")
        {
            return LasExtraScalarField::DataType::f64_2;
        }
    }

    if (radioButton3->isChecked())
    {
        if (selectedElementType == "uint8")
        {
            return LasExtraScalarField::DataType::u8_3;
        }

        if (selectedElementType == "uint16")
        {
            return LasExtraScalarField::DataType::u16_3;
        }

        if (selectedElementType == "uint32")
        {
            return LasExtraScalarField::DataType::u32_3;
        }

        if (selectedElementType == "uint64")
        {
            return LasExtraScalarField::DataType::u64_3;
        }

        if (selectedElementType == "int8")
        {
            return LasExtraScalarField::DataType::i8_3;
        }

        if (selectedElementType == "int16")
        {
            return LasExtraScalarField::DataType::i16_3;
        }

        if (selectedElementType == "int32")
        {
            return LasExtraScalarField::DataType::i32_3;
        }

        if (selectedElementType == "int64")
        {
            return LasExtraScalarField::DataType::i64_3;
        }

        if (selectedElementType == "float32")
        {
            return LasExtraScalarField::DataType::f32_3;
        }

        if (selectedElementType == "float64")
        {
            return LasExtraScalarField::DataType::f64_3;
        }
    }

    Q_ASSERT(false);
    return LasExtraScalarField::DataType::Invalid;
}
