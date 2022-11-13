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

#include "LasOpenDialog.h"

// Qt
#include <QLocale>

static QListWidgetItem* CreateItem(const char* name)
{
	auto item = new QListWidgetItem(name);
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	item->setCheckState(Qt::Checked);
	return item;
}

static bool IsCheckedIn(const QString& name, const QListWidget& list)
{
	for (int i = 0; i < list.count(); ++i)
	{
		if (list.item(i)->text() == name)
		{
			return list.item(i)->checkState() == Qt::Checked;
		}
	}
	return false;
}

// TODO use std::remove_if
template <typename T, typename Pred>
static void RemoveFalse(std::vector<T>& vec, Pred predicate)
{
	auto firstFalse = std::partition(vec.begin(), vec.end(), predicate);

	if (firstFalse != vec.end())
	{
		vec.erase(firstFalse, vec.end());
	}
}

LasOpenDialog::LasOpenDialog(QWidget* parent)
    : QDialog(parent)
{
	setupUi(this);

	connect(applyButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(applyAllButton, &QPushButton::clicked, this, &QDialog::accept);
	connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
	connect(automaticTimeShiftCheckBox, &QCheckBox::toggled, this, &LasOpenDialog::onAutomaticTimeShiftToggle);
	connect(applyAllButton, &QPushButton::clicked, this, &LasOpenDialog::onApplyAll);
	connect(selectAllToolButton, &QPushButton::clicked, [&]
	        { doSelectAll(true); });
	connect(unselectAllToolButton, &QPushButton::clicked, this, [&]
	        { doSelectAll(false); });

	connect(selectAllESFToolButton, &QPushButton::clicked, [&]
	        { doSelectAllESF(true); });
	connect(unselectAllESFToolButton, &QPushButton::clicked, this, [&]
	        { doSelectAllESF(false); });
}

void LasOpenDialog::doSelectAll(bool doSelect)
{
	if (!availableScalarFields)
	{
		assert(false);
		return;
	}

	for (int i = 0; i < availableScalarFields->count(); ++i)
	{
		availableScalarFields->item(i)->setCheckState(doSelect ? Qt::Checked : Qt::Unchecked);
	}
}

void LasOpenDialog::doSelectAllESF(bool doSelect)
{
	if (!availableExtraScalarFields)
	{
		assert(false);
		return;
	}

	for (int i = 0; i < availableExtraScalarFields->count(); ++i)
	{
		availableExtraScalarFields->item(i)->setCheckState(doSelect ? Qt::Checked : Qt::Unchecked);
	}
}

void LasOpenDialog::setInfo(int versionMinor, int pointFormatId, qulonglong numPoints)
{
	versionLabelValue->setText(QString("1.%1").arg(QString::number(versionMinor)));
	pointFormatLabelValue->setText(QString::number(pointFormatId));
	numPointsLabelValue->setText(QLocale(QLocale::English).toString(numPoints));

	force8bitColorsCheckBox->setEnabled(LasDetails::HasRGB(pointFormatId));
	timeShiftLayout->setEnabled(LasDetails::HasGpsTime(pointFormatId));
}

void LasOpenDialog::setAvailableScalarFields(const std::vector<LasScalarField>&      scalarFields,
                                             const std::vector<LasExtraScalarField>& extraScalarFields)
{
	if (!scalarFields.empty())
	{
		scalarFieldFrame->show();
		for (const LasScalarField& lasScalarField : scalarFields)
		{
			availableScalarFields->addItem(CreateItem(lasScalarField.name()));
		}
	}
	else
	{
		scalarFieldFrame->hide();
	}

	if (!extraScalarFields.empty())
	{
		extraScalarFieldsFrame->show();
		for (const LasExtraScalarField& lasExtraScalarField : extraScalarFields)
		{
			availableExtraScalarFields->addItem(CreateItem(lasExtraScalarField.name));
		}
		int height = availableExtraScalarFields->frameWidth() + (availableExtraScalarFields->sizeHintForRow(0) + availableExtraScalarFields->frameWidth()) * availableExtraScalarFields->count();
		availableExtraScalarFields->setMaximumHeight(height);
	}
	else
	{
		extraScalarFieldsFrame->hide();
	}
}

void LasOpenDialog::filterOutNotChecked(std::vector<LasScalarField>&      scalarFields,
                                        std::vector<LasExtraScalarField>& extraScalarFields)
{
	const auto isFieldSelected = [this](const auto& field)
	{ return isChecked(field); };

	RemoveFalse(scalarFields, isFieldSelected);
	RemoveFalse(extraScalarFields, isFieldSelected);
}

bool LasOpenDialog::shouldIgnoreFieldsWithDefaultValues() const
{
	return ignoreFieldsWithDefaultValuesCheckBox->isChecked();
}

bool LasOpenDialog::shouldForce8bitColors() const
{
	return force8bitColorsCheckBox->isChecked();
}

double LasOpenDialog::timeShiftValue() const
{
	if (automaticTimeShiftCheckBox->isChecked())
	{
		return std::numeric_limits<double>::quiet_NaN();
	}

	return manualTimeShiftSpinBox->value();
}

bool LasOpenDialog::isChecked(const LasExtraScalarField& lasExtraScalarField) const
{
	return IsCheckedIn(lasExtraScalarField.name, *availableExtraScalarFields);
}

bool LasOpenDialog::isChecked(const LasScalarField& lasScalarField) const
{
	return IsCheckedIn(lasScalarField.name(), *availableScalarFields);
}

void LasOpenDialog::onAutomaticTimeShiftToggle(bool checked)
{
	manualTimeShiftSpinBox->setEnabled(!checked);
}

bool LasOpenDialog::shouldSkipDialog() const
{
	return m_shouldSkipDialog;
}

void LasOpenDialog::resetShouldSkipDialog()
{
	m_shouldSkipDialog = false;
}

void LasOpenDialog::onApplyAll()
{
	m_shouldSkipDialog = true;
	accept();
}
