#include "LasExtraScalarFieldCard.h"

#include <ccPointCloud.h>
#include <ccScalarField.h>

LasExtraScalarFieldCard::LasExtraScalarFieldCard(QWidget* parent)
    : QWidget(parent)
{
	setupUi(this);

	nameEdit->setMaxLength(LasExtraScalarField::MAX_NAME_SIZE);
	descriptionEdit->setMaxLength(LasExtraScalarField::MAX_DESCRIPTION_SIZE);

	m_scalarFieldsUserInputs[0] = {firstScalarFieldComboBox, firstScalarFieldScaleSpinBox, firstScalarFieldOffsetSpinBox};
	m_scalarFieldsUserInputs[1] = {secondScalarFieldComboBox, secondScalarFieldScaleSpinBox, secondScalarFieldOffsetSpinBox};
	m_scalarFieldsUserInputs[2] = {thirdScalarFieldComboBox, thirdScalarFieldScaleSpinBox, thirdScalarFieldOffsetSpinBox};

	connect(radioButton1,
	        &QRadioButton::clicked,
	        this,
	        &LasExtraScalarFieldCard::onRadioButton1Selected);

	connect(radioButton2,
	        &QRadioButton::clicked,
	        this,
	        &LasExtraScalarFieldCard::onRadioButton2Selected);

	connect(radioButton3,
	        &QRadioButton::clicked,
	        this,
	        &LasExtraScalarFieldCard::onRadioButton3Selected);

	connect(firstScalarFieldComboBox,
	        &QComboBox::currentTextChanged,
	        this,
	        [this](const QString& text)
	        {
		        if (radioButton1->isChecked())
		        {
			        nameEdit->setText(text);
		        }
	        });

	advancedOptionFrame->hide();
	scaledCheckBox->setChecked(false);
	scalingOptionGroup->setEnabled(false);

	connect(advancedOptionsButton, &QPushButton::clicked, this, &LasExtraScalarFieldCard::onToggleAdvancedOptionsClicked);
	connect(scaledCheckBox, &QCheckBox::stateChanged, scalingOptionGroup, &QGroupBox::setEnabled);

	reset();
}

void LasExtraScalarFieldCard::reset()
{
	radioButton1->setChecked(true);
	emit radioButton1->clicked(true);

	nameEdit->clear();
	if (advancedOptionsButton->isChecked())
	{
		advancedOptionsButton->setChecked(false);
		emit advancedOptionsButton->clicked();
	}

#ifdef CC_CORE_LIB_USES_DOUBLE
	const char* defaultType = "float64";
#else
	const char* defaultType = "float32";
#endif
	typeComboBox->setCurrentText(defaultType);

	firstScalarFieldComboBox->setCurrentIndex(0);
	secondScalarFieldComboBox->setCurrentIndex(0);
	thirdScalarFieldComboBox->setCurrentIndex(0);
}

void LasExtraScalarFieldCard::fillFrom(const LasExtraScalarField& field)
{
	assert(field.ccName[0] == 0); // TODO

	nameEdit->setText(field.name);
	switch (field.numElements())
	{
	case 1:
		radioButton1->setChecked(true);
		emit radioButton1->clicked(true);
		assert(field.scalarFields[0] != nullptr);
		firstScalarFieldComboBox->setCurrentText(field.scalarFields[0]->getName());
		break;
	case 2:
		radioButton2->setChecked(true);
		emit radioButton2->clicked(true);
		assert(field.scalarFields[0] != nullptr);
		firstScalarFieldComboBox->setCurrentText(field.scalarFields[0]->getName());
		assert(field.scalarFields[1] != nullptr);
		secondScalarFieldComboBox->setCurrentText(field.scalarFields[1]->getName());
		break;
	case 3:
		radioButton3->setChecked(true);
		emit radioButton3->clicked(true);
		firstScalarFieldComboBox->setCurrentText(field.scalarFields[0]->getName());
		assert(field.scalarFields[1] != nullptr);
		secondScalarFieldComboBox->setCurrentText(field.scalarFields[1]->getName());
		assert(field.scalarFields[1] != nullptr);
		secondScalarFieldComboBox->setCurrentText(field.scalarFields[1]->getName());
		assert(field.scalarFields[2] != nullptr);
		thirdScalarFieldComboBox->setCurrentText(field.scalarFields[2]->getName());
		break;
	default:
		assert(false);
		return;
	}

	descriptionEdit->setText(field.description);
	scaledCheckBox->setChecked(false);
	emit scaledCheckBox->stateChanged(false);

	if (field.scaleIsRelevant())
	{
		scaledCheckBox->setChecked(true);
		emit scaledCheckBox->stateChanged(true);
		assert(field.numElements() <= 3);
		for (size_t i = 0; i < field.numElements(); ++i)
		{
			m_scalarFieldsUserInputs[i].scaleSpinBox->setValue(field.scales[i]);
		}
	}

	if (field.offsetIsRelevant())
	{
		emit scaledCheckBox->stateChanged(true);
		assert(field.numElements() <= 3);
		for (size_t i = 0; i < field.numElements(); ++i)
		{
			m_scalarFieldsUserInputs[i].offsetSpinBox->setValue(field.offsets[i]);
		}
	}

	switch (field.type)
	{
	case LasExtraScalarField::DataType::u8:
		typeComboBox->setCurrentText("uint8");
		break;
	case LasExtraScalarField::DataType::u16:
		typeComboBox->setCurrentText("uint16");
		break;
	case LasExtraScalarField::DataType::u32:
		typeComboBox->setCurrentText("uint32");
		break;
	case LasExtraScalarField::DataType::u64:
		typeComboBox->setCurrentText("uint64");
		break;
	case LasExtraScalarField::DataType::i8:
		typeComboBox->setCurrentText("int8");
		break;
	case LasExtraScalarField::DataType::i16:
		typeComboBox->setCurrentText("int16");
		break;
	case LasExtraScalarField::DataType::i32:
		typeComboBox->setCurrentText("int32");
		break;
	case LasExtraScalarField::DataType::i64:
		typeComboBox->setCurrentText("int64");
		break;
	case LasExtraScalarField::DataType::f32:
		typeComboBox->setCurrentText("float32");
		break;
	case LasExtraScalarField::DataType::f64:
		typeComboBox->setCurrentText("float64");
		break;
	default:
		typeComboBox->setCurrentText("float32");
		break;
	}
}

bool LasExtraScalarFieldCard::fillField(LasExtraScalarField& field, const ccPointCloud& pointCloud) const
{
	if (nameEdit->text().isEmpty())
	{
		return false;
	}

	const std::string stdName = nameEdit->text().toStdString();
	strncpy(field.name, stdName.c_str(), LasExtraScalarField::MAX_NAME_SIZE);

	const std::string stdDescription = descriptionEdit->text().toStdString();
	strncpy(field.description, stdDescription.c_str(), LasExtraScalarField::MAX_DESCRIPTION_SIZE);

	// since the corresponding line edits max length are properly set
	// this should only happen in (rare) cases when converting from
	// QString utf16 to bytes yields more bytes than chars if the users
	// used  too many non ascii symbols
	if (stdName.size() > LasExtraScalarField::MAX_NAME_SIZE)
	{
		ccLog::Warning("[LAS] Extra Scalar field name '%s' is too long and will be truncated",
		               stdName.c_str());
	}
	if (stdDescription.size() > LasExtraScalarField::MAX_DESCRIPTION_SIZE)
	{
		ccLog::Warning("[LAS] Extra scalar field description '%s' is too long and will be truncated",
		               stdDescription.c_str());
	}

	field.type = dataType();
	if (radioButton1->isChecked())
	{
		field.dimensions = LasExtraScalarField::DimensionSize::One;
	}
	else if (radioButton2->isChecked())
	{
		field.dimensions = LasExtraScalarField::DimensionSize::Two;
	}
	else if (radioButton3->isChecked())
	{
		field.dimensions = LasExtraScalarField::DimensionSize::Three;
	}

	if (scaledCheckBox->isChecked())
	{
		field.setScaleIsRelevant(true);
		field.setOffsetIsRelevant(true);

		for (size_t i = 0; i < field.numElements(); i++)
		{
			field.scales[i]  = m_scalarFieldsUserInputs[i].scaleSpinBox->value();
			field.offsets[i] = m_scalarFieldsUserInputs[i].offsetSpinBox->value();
		}
	}
	else
	{
		field.setScaleIsRelevant(false);
		field.setOffsetIsRelevant(false);
	}

	for (size_t i = 0; i < field.numElements(); i++)
	{
		const std::string sfName =
		    m_scalarFieldsUserInputs[i].scalarFieldComboBox->currentText().toStdString();
		int sfIndex = pointCloud.getScalarFieldIndexByName(sfName.c_str());

		if (sfIndex < 0)
		{
			ccLog::Warning("Failed to get scalar field named '%s'", sfName.c_str());
			return false;
		}
		field.scalarFields[i] = static_cast<ccScalarField*>(pointCloud.getScalarField(sfIndex));
	}

	return true;
}

LasExtraScalarField::DataType LasExtraScalarFieldCard::dataType() const
{
	const QString selectedElementType = typeComboBox->currentText();

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

	assert(false);
	return LasExtraScalarField::DataType::Invalid;
}

void LasExtraScalarFieldCard::onNumberOfElementsSelected(unsigned numberOfElements)
{
	if (numberOfElements == 0 || numberOfElements > 3)
	{
		Q_ASSERT_X(false, "onNumberOfElementsSelected", "Invalid number of elements");
		return;
	}

	for (size_t i = 0; i < LasExtraScalarField::MAX_DIM_SIZE; i++)
	{
		ScalarFieldUserInputs& userInput = m_scalarFieldsUserInputs[i];

		const bool isPartOfSelected = i <= (numberOfElements - 1);

		userInput.scalarFieldComboBox->setVisible(isPartOfSelected);
		userInput.scaleSpinBox->setEnabled(isPartOfSelected);
		userInput.offsetSpinBox->setEnabled(isPartOfSelected);
	}
}

void LasExtraScalarFieldCard::onRadioButton1Selected(bool)
{
	onNumberOfElementsSelected(1);
}

void LasExtraScalarFieldCard::onRadioButton2Selected(bool)
{
	onNumberOfElementsSelected(2);
}

void LasExtraScalarFieldCard::onRadioButton3Selected(bool)
{
	onNumberOfElementsSelected(3);
}

void LasExtraScalarFieldCard::onToggleAdvancedOptionsClicked()
{
	if (advancedOptionFrame->isHidden())
	{
		advancedOptionFrame->show();
	}
	else
	{
		advancedOptionFrame->hide();
	}
}
