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
#include "LasExtraScalarFieldCard.h"

// Qt
#include <QLayoutItem>
#include <QStringListModel>
// qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

//! Default LAS scale
static const double DefaultLASScale = 1.0e-3;

//! Widget to map a predefined scalar field 'role' with a particular scalar field (combo box)
class MappingLabel : public QWidget
{
  public:
	explicit MappingLabel(QWidget* parent = nullptr)
	    : QWidget(parent)
	    , m_nameLabel(new QLabel)
	    , m_statusLabel(new QLabel)
	{
		auto* layout = new QHBoxLayout;
		layout->setMargin(0);
		layout->addWidget(m_nameLabel);
		layout->addWidget(m_statusLabel);
		setLayout(layout);
	}

	void setName(const QString& name)
	{
		m_nameLabel->setText(name);
	}

	QString name() const
	{
		return m_nameLabel->text();
	}

	void setWarning(const QString& message)
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
	QLabel* m_nameLabel;
	QLabel* m_statusLabel;
};

LasSaveDialog::LasSaveDialog(ccPointCloud* cloud, QWidget* parent)
    : QDialog(parent)
    , m_cloud(cloud)
    , m_scalarFieldsNamesModel(new QStringListModel)
    , m_extraFieldsDataTypesModel(new QStringListModel)
    , m_optimalScale(std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN())
    , m_originalScale(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN())
{
	setupUi(this);

	// retrieve the list of SFs (must be done first)
	QStringList cloudScalarFieldsNames;
	cloudScalarFieldsNames << QString();
	if (m_cloud)
	{
		for (unsigned i = 0; i < m_cloud->getNumberOfScalarFields(); ++i)
		{
			if (strcmp(m_cloud->getScalarFieldName(i), "Default") != 0)
			{
				cloudScalarFieldsNames << m_cloud->getScalarFieldName(i);
			}
		}
	}
	else
	{
		assert(false);
	}
	m_scalarFieldsNamesModel->setStringList(cloudScalarFieldsNames);

	bestScaleRadioButton->setChecked(false);
	originalScaleRadioButton->setEnabled(false);

	customScaleRadioButton->setEnabled(true);
	customScaleRadioButton->setChecked(false);
	QDoubleSpinBox* scaleButtons[3] = {customScaleXDoubleSpinBox, customScaleYDoubleSpinBox, customScaleZDoubleSpinBox};
	for (size_t i = 0; i < 3; i++)
	{
		QDoubleSpinBox* spinBox = scaleButtons[i];
		spinBox->setDecimals(8);
		spinBox->setMinimum(1.0e-8);
		spinBox->setMaximum(100.0);
		spinBox->setValue(DefaultLASScale);
		spinBox->setSingleStep(DefaultLASScale);
		spinBox->setEnabled(false);
	}
	connect(customScaleRadioButton, &QRadioButton::toggled, this, &LasSaveDialog::handleCustomScaleButtontoggled);

	customScaleRadioButton->setChecked(true); // should be checked by default, as it's the only one enabled by default

	for (const char* versionStr : LasDetails::AvailableVersions())
	{
		versionComboBox->addItem(versionStr);
	}
	versionComboBox->setCurrentIndex(0);

	connect(versionComboBox,
	        (void (QComboBox::*)(const QString&))(&QComboBox::currentIndexChanged),
	        this,
	        &LasSaveDialog::handleSelectedVersionChange);

	connect(pointFormatComboBox,
	        (void (QComboBox::*)(int))(&QComboBox::currentIndexChanged),
	        this,
	        &LasSaveDialog::handleSelectedPointFormatChange);

	handleSelectedVersionChange(versionComboBox->currentText()); // will call handleSelectedPointFormatChange

	QStringList extraFieldsDataTypeNames{"uint8",
	                                     "uint16",
	                                     "uint32",
	                                     "uint64",
	                                     "int8",
	                                     "int16",
	                                     "int32",
	                                     "int64",
	                                     "float32",
	                                     "float64"};
	m_extraFieldsDataTypesModel->setStringList(extraFieldsDataTypeNames);

	connect(addExtraScalarFieldButton, &QPushButton::clicked, this, &LasSaveDialog::addExtraScalarFieldCard);

	normalsCheckBox->setEnabled(cloud->hasNormals());
	normalsCheckBox->setCheckState(cloud->hasNormals() ? Qt::CheckState::Checked : Qt::Unchecked);
}

/// When the selected version changes, we need to update the combo box
/// of point format to match the ones supported by the version
void LasSaveDialog::handleSelectedVersionChange(const QString& version)
{
	pointFormatComboBox->blockSignals(true);
	pointFormatComboBox->clear();

	const std::vector<unsigned>* pointFormats = LasDetails::PointFormatsAvailableForVersion(qPrintable(version));
	if (pointFormats)
	{
		for (unsigned fmt : *pointFormats)
		{
			pointFormatComboBox->addItem(QString::number(fmt));
		}
		pointFormatComboBox->setCurrentIndex(0);

		// We have to force the call here so that the point format combo-box is always updated
		handleSelectedPointFormatChange(0);
	}

	pointFormatComboBox->blockSignals(false);
}

/// When the user changes the ccScalarField it wants to save in the particular LAS field,
/// we check that the values are in range. If they are not we display a small warning
void LasSaveDialog::handleComboBoxChange(int index)
{
	if (!m_cloud)
	{
		assert(false);
		return;
	}
	if (index < 0)
	{
		return;
	}
	QObject* senderObject = sender();
	size_t   senderIndex  = std::distance(m_scalarFieldMapping.begin(),
                                       std::find_if(m_scalarFieldMapping.begin(),
                                                    m_scalarFieldMapping.end(),
                                                    [senderObject](const std::pair<MappingLabel*, QComboBox*>& pair)
                                                    { return pair.second == senderObject; }));

	if (qobject_cast<QComboBox*>(senderObject)->itemText(index).isEmpty())
	{
		m_scalarFieldMapping[senderIndex].first->clearWarning();
		return;
	}
	const QString scalarFieldName   = m_scalarFieldMapping[senderIndex].first->name();
	const QString ccScalarFieldName = m_scalarFieldMapping[senderIndex].second->currentText();
	int           sfIdx             = m_cloud->getScalarFieldIndexByName(ccScalarFieldName.toStdString().c_str());
	if (sfIdx == -1)
	{
		assert(false);
		return;
	}
	const CCCoreLib::ScalarField* scalarField = m_cloud->getScalarField(sfIdx);
	if (!scalarField)
	{
		assert(false);
		return;
	}

	// TODO support QString equality
	LasScalarField::Id    scalarFieldId = LasScalarField::IdFromName(scalarFieldName.toStdString().c_str(), selectedPointFormat());
	LasScalarField::Range range         = LasScalarField::ValueRange(scalarFieldId);

	if (scalarField->getMin() < range.min || scalarField->getMax() > range.max)
	{
		m_scalarFieldMapping[senderIndex].first->setWarning("Some values are out of range and will be truncated");
	}
	else
	{
		m_scalarFieldMapping[senderIndex].first->clearWarning();
	}

	size_t numWarnings = std::count_if(m_scalarFieldMapping.begin(),
	                                   m_scalarFieldMapping.end(),
	                                   [](const std::pair<MappingLabel*, QComboBox*>& pair)
	                                   { return pair.first->hasWarning(); });

	if (numWarnings > 0)
	{
		tabWidget->setTabIcon(1, QApplication::style()->standardPixmap(QStyle::SP_MessageBoxWarning));
	}
	else
	{
		tabWidget->setTabIcon(1, {});
	}
}

/// When the user changes the point format, we need to update the scalar field form.
void LasSaveDialog::handleSelectedPointFormatChange(int index)
{
	if (!m_cloud)
	{
		assert(false);
		return;
	}
	if (index < 0)
	{
		return;
	}

	const std::vector<unsigned>* pointFormats = LasDetails::PointFormatsAvailableForVersion(versionComboBox->currentText());
	if (!pointFormats)
	{
		Q_ASSERT_X(false, __func__, "No point format available for the selected version");
		return;
	}

	// Hide all the rows in our form
	for (int i = 0; i < scalarFieldFormLayout->rowCount(); ++i)
	{
		auto* label = scalarFieldFormLayout->itemAt(i, QFormLayout::LabelRole);
		if (label != nullptr && label->widget() != nullptr)
		{
			label->widget()->hide();
		}

		auto* field = scalarFieldFormLayout->itemAt(i, QFormLayout::FieldRole);
		if (field != nullptr && field->widget() != nullptr)
		{
			field->widget()->hide();
		}
	}

	unsigned                    selectedPointFormat = pointFormats->at(index);
	std::vector<LasScalarField> lasScalarFields     = LasScalarField::ForPointFormat(selectedPointFormat);

	int numDeltaFields = scalarFieldFormLayout->rowCount() - static_cast<int>(lasScalarFields.size());
	if (numDeltaFields < 0)
	{
		// We have fewer rows in our form, create new ones
		for (int i = numDeltaFields; i < 0; ++i)
		{
			auto* box = new QComboBox(this);
			box->setModel(m_scalarFieldsNamesModel);
			connect(box,
			        qOverload<int>(&QComboBox::currentIndexChanged),
			        this,
			        &LasSaveDialog::handleComboBoxChange);

			auto* widget = new MappingLabel(this);
			scalarFieldFormLayout->addRow(widget, box);
			m_scalarFieldMapping.emplace_back(widget, box);
		}
	}

	assert(lasScalarFields.size() <= scalarFieldFormLayout->rowCount());

	QStringList cloudScalarFieldsNames = m_scalarFieldsNamesModel->stringList();
	for (size_t i = 0; i < lasScalarFields.size(); ++i)
	{
		const LasScalarField& field = lasScalarFields[i];
		m_scalarFieldMapping[i].first->setName(field.name());
		m_scalarFieldMapping[i].first->clearWarning();
		m_scalarFieldMapping[i].second->setCurrentIndex(cloudScalarFieldsNames.indexOf(field.name()));

		auto* label = scalarFieldFormLayout->itemAt(static_cast<int>(i), QFormLayout::LabelRole);
		if (label != nullptr && label->widget() != nullptr)
		{
			label->widget()->show();
		}

		auto* fieldWidget = scalarFieldFormLayout->itemAt(static_cast<int>(i), QFormLayout::FieldRole);
		if (fieldWidget != nullptr && fieldWidget->widget() != nullptr)
		{
			fieldWidget->widget()->show();
		}
	}

	if (!LasDetails::HasRGB(selectedPointFormat) && !LasDetails::HasWaveform(selectedPointFormat))
	{
		specialScalarFieldFrame->hide();
		waveformCheckBox->setCheckState(Qt::Unchecked);
		rgbCheckBox->setCheckState(Qt::Unchecked);
	}
	else
	{
		specialScalarFieldFrame->show();
		if (LasDetails::HasRGB(selectedPointFormat))
		{
			rgbCheckBox->show();
			rgbCheckBox->setEnabled(m_cloud->hasColors());
			rgbCheckBox->setChecked(m_cloud->hasColors());
		}
		else
		{
			rgbCheckBox->hide();
		}

		if (LasDetails::HasWaveform(selectedPointFormat))
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

void LasSaveDialog::handleCustomScaleButtontoggled(bool checked)
{
	customScaleXDoubleSpinBox->setEnabled(checked);
	customScaleYDoubleSpinBox->setEnabled(checked);
	customScaleZDoubleSpinBox->setEnabled(checked);
}

LasExtraScalarFieldCard* LasSaveDialog::createCard() const
{
	auto* card = new LasExtraScalarFieldCard;
	card->typeComboBox->setModel(m_extraFieldsDataTypesModel);
	card->firstScalarFieldComboBox->setModel(m_scalarFieldsNamesModel);
	card->secondScalarFieldComboBox->setModel(m_scalarFieldsNamesModel);
	card->thirdScalarFieldComboBox->setModel(m_scalarFieldsNamesModel);
	connect(card->removeButton, &QPushButton::clicked, card, &QWidget::hide);

	return card;
}

void LasSaveDialog::addExtraScalarFieldCard()
{
#ifdef CC_CORE_LIB_USES_DOUBLE
	const char* defaultType = "float64";
#else
	const char* defaultType = "float32";
#endif
	int defaultTypeIndex = m_extraFieldsDataTypesModel->stringList().indexOf(defaultType);
	defaultTypeIndex     = std::max(defaultTypeIndex, 0);

	int esfCount = extraScalarFieldsLayout->count();

	// firt, look if a card has already been created, but is currently hidden
	for (int i = 0; i < esfCount; ++i)
	{
		QLayoutItem* item   = extraScalarFieldsLayout->itemAt(i);
		QWidget*     widget = item->widget();
		if (widget && widget->isHidden())
		{
			// remove and insert it again at the end
			extraScalarFieldsLayout->removeItem(item);
			extraScalarFieldsLayout->insertItem(extraScalarFieldsLayout->count(), item);
			auto* card = qobject_cast<LasExtraScalarFieldCard*>(widget);
			if (!card)
			{
				assert(false);
				continue;
			}

			card->reset();
			widget->show();
			return;
		}
	}

	// else we'll create a new card
	auto* card = createCard();
	extraScalarFieldsLayout->insertWidget(esfCount, card);
}

void LasSaveDialog::setVersionAndPointFormat(const LasDetails::LasVersion versionAndFmt)
{
	const QString versionStr = QString("1.%1").arg(versionAndFmt.minorVersion);

	int versionIndex = versionComboBox->findText(versionStr);
	if (versionIndex >= 0)
	{
		QString fmtStr = QString::number(versionAndFmt.pointFormat);
		versionComboBox->setCurrentIndex(versionIndex);
		int pointFormatIndex = pointFormatComboBox->findText(fmtStr);
		if (pointFormatIndex >= 0)
		{
			pointFormatComboBox->setCurrentIndex(pointFormatIndex);
		}
	}
}

void LasSaveDialog::setOptimalScale(const CCVector3d& scale, bool autoCheck /*=false*/)
{
	m_optimalScale = scale;

	bestScaleLabel->setText(QString("(%1, %2, %3)").arg(scale.x).arg(scale.y).arg(scale.z));

	bestScaleRadioButton->setEnabled(true);
	if (autoCheck)
	{
		bestScaleRadioButton->setChecked(true);
	}
}

void LasSaveDialog::setOriginalScale(const CCVector3d& scale, bool canUseScale, bool autoCheck /*=true*/)
{
	m_originalScale = scale;

	originalScaleLabel->setText(QString("(%1, %2, %3)").arg(scale.x).arg(scale.y).arg(scale.z));

	originalScaleRadioButton->setEnabled(canUseScale);
	if (!canUseScale)
	{
		labelOriginal->setText(QObject::tr("Original scale is too small for this cloud  ")); // add two whitespaces to avoid issues with italic characters justification
		labelOriginal->setStyleSheet("color: red;");
		customScaleRadioButton->setChecked(true); // revert to the custom scale by default
	}
	else if (autoCheck)
	{
		originalScaleRadioButton->setChecked(true);
	}
}

void LasSaveDialog::setExtraScalarFields(const std::vector<LasExtraScalarField>& extraScalarFields)
{
	if (extraScalarFields.empty())
	{
		return;
	}

	for (const LasExtraScalarField& field : extraScalarFields)
	{
		auto* card = createCard();
		extraScalarFieldsLayout->insertWidget(extraScalarFieldsLayout->count(), card);
		card->fillFrom(field);
	}
}

uint8_t LasSaveDialog::selectedPointFormat() const
{
	return static_cast<uint8_t>(std::min(pointFormatComboBox->currentText().toUInt(), 255u));
}

void LasSaveDialog::selectedVersion(uint8_t& versionMajor, uint8_t& versionMinor) const
{
	versionMajor = 1;
	versionMinor = 0;

	const QString       versionString = versionComboBox->currentText();
	QVector<QStringRef> tokens        = versionString.splitRef('.');
	if (tokens.size() == 2)
	{
		versionMajor = static_cast<uint8_t>(std::min(tokens[0].toUInt(), 255u));
		versionMinor = static_cast<uint8_t>(std::min(tokens[1].toUInt(), 255u));
	}
	else
	{
		assert(false);
	}
}

bool LasSaveDialog::shouldSaveRGB() const
{
	return rgbCheckBox->isChecked();
}

bool LasSaveDialog::shouldSaveWaveform() const
{
	return waveformCheckBox->isChecked();
}

bool LasSaveDialog::shouldSaveNormalsAsExtraScalarField() const
{
	return normalsCheckBox->isChecked();
}

CCVector3d LasSaveDialog::chosenScale() const
{
	if (bestScaleRadioButton->isChecked() && bestScaleRadioButton->isEnabled())
	{
		assert(std::isfinite(m_optimalScale.x) && std::isfinite(m_optimalScale.y) && std::isfinite(m_optimalScale.z));
		return m_optimalScale;
	}
	else if (originalScaleRadioButton->isChecked() && originalScaleRadioButton->isEnabled())
	{
		assert(std::isfinite(m_originalScale.x) && std::isfinite(m_originalScale.y) && std::isfinite(m_originalScale.z));
		return m_originalScale;
	}
	else if (customScaleRadioButton->isChecked() && customScaleRadioButton->isEnabled())
	{
		return {customScaleXDoubleSpinBox->value(),
		        customScaleYDoubleSpinBox->value(),
		        customScaleZDoubleSpinBox->value()};
	}
	else
	{
		ccLog::Error("Inconsistency detected: scale option is checked but not enabled");
		assert(false);
		return {DefaultLASScale, DefaultLASScale, DefaultLASScale};
	}
}

std::vector<LasScalarField> LasSaveDialog::fieldsToSave() const
{
	if (!m_cloud)
	{
		assert(false);
		return {};
	}

	unsigned pointFormat = selectedPointFormat();

	std::vector<LasScalarField> fields;
	fields.reserve(scalarFieldFormLayout->rowCount());

	for (const auto& item : m_scalarFieldMapping)
	{
		if (item.second->currentIndex() > 0)
		{
			int sfIdx = m_cloud->getScalarFieldIndexByName(qPrintable(item.second->currentText()));
			if (sfIdx < 0)
			{
				assert(false);
				continue;
			}

			ccScalarField* sf = static_cast<ccScalarField*>(m_cloud->getScalarField(sfIdx));

			const std::string name = item.first->name().toStdString();
			fields.emplace_back(LasScalarField::IdFromName(name.c_str(), pointFormat), sf);
		}
	}

	return fields;
}

std::vector<LasExtraScalarField> LasSaveDialog::extraFieldsToSave() const
{
	if (!m_cloud)
	{
		assert(false);
		return {};
	}

	if (extraScalarFieldsLayout->count() == 0)
	{
		return {};
	}
	int esfCount = extraScalarFieldsLayout->count();

	std::vector<LasExtraScalarField> extraScalarFields;
	extraScalarFields.reserve(esfCount);

	for (int i = 0; i < esfCount; ++i)
	{
		QLayoutItem* item   = extraScalarFieldsLayout->itemAt(i);
		QWidget*     widget = item->widget();
		if (!widget || widget->isHidden())
		{
			continue;
		}
		auto* card = qobject_cast<LasExtraScalarFieldCard*>(widget);
		if (!card)
		{
			continue;
		}

		LasExtraScalarField field;
		if (!card->fillField(field, *m_cloud))
		{
			ccLog::Error("failed to convert scalar field info to something writable");
			continue;
		}
		extraScalarFields.push_back(field);
	}

	return extraScalarFields;
}
