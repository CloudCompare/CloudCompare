// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: The CloudCompare project                           #
// #                                                                        #
// ##########################################################################

#include "ccCutPursuitDlg.h"

#include <DgmOctree.h>
#include <QSettings>
#include <ui_cutPursuitDlg.h>

static const QString s_rgbFeatureName = QObject::tr("RGB");

ccCutPursuitDlg::ccCutPursuitDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent)
    , m_ui(std::make_unique<Ui::CutPursuitDialog>())
{
	m_ui->setupUi(this);

	loadFromPersistentSettings();
}

ccCutPursuitDlg::~ccCutPursuitDlg() = default;

void ccCutPursuitDlg::setScalarFields(const QStringList& sfNames, bool includeRGB /*=false*/)
{
	m_ui->scalarFieldsListWidget->clear();

	if (includeRGB)
	{
		QListWidgetItem* item = new QListWidgetItem(s_rgbFeatureName, m_ui->scalarFieldsListWidget);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
	}

	for (const QString& name : sfNames)
	{
		QListWidgetItem* item = new QListWidgetItem(name, m_ui->scalarFieldsListWidget);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
	}
}

QStringList ccCutPursuitDlg::getSelectedScalarFields() const
{
	QStringList selected;

	for (int i = 0; i < m_ui->scalarFieldsListWidget->count(); ++i)
	{
		QListWidgetItem* item = m_ui->scalarFieldsListWidget->item(i);
		if (item && item->checkState() == Qt::Checked && item->text() != s_rgbFeatureName)
		{
			selected.push_back(item->text());
		}
	}

	return selected;
}

int ccCutPursuitDlg::getKNN()
{
	return m_ui->knnSpinBox->value();
}

double ccCutPursuitDlg::getKNNRadius()
{
	return m_ui->knnRadiusSpinBox->value();
}

double ccCutPursuitDlg::getRegularization()
{
	return m_ui->regularizationSpinBox->value();
}

double ccCutPursuitDlg::getSpatialWeight()
{
	return m_ui->spatialWeightSpinBox->value();
}

int ccCutPursuitDlg::getCutoff()
{
	return m_ui->cutoffSpinBox->value();
}

bool ccCutPursuitDlg::useRGB()
{
	for (int i = 0; i < m_ui->scalarFieldsListWidget->count(); ++i)
	{
		QListWidgetItem* item = m_ui->scalarFieldsListWidget->item(i);
		if (item && item->text() == s_rgbFeatureName)
		{
			return (item->checkState() == Qt::Checked);
		}
	}
	return false;
}

bool ccCutPursuitDlg::averageColors()
{
	return (m_ui->averageColorsCheckBox->checkState() == Qt::Checked);
}

void ccCutPursuitDlg::saveToPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("CutPursuitDialog");
	{
		settings.setValue("knn", m_ui->knnSpinBox->value());
		settings.setValue("knnRadius", m_ui->knnRadiusSpinBox->value());
		settings.setValue("regularization", m_ui->regularizationSpinBox->value());
		settings.setValue("spatialWeight", m_ui->spatialWeightSpinBox->value());
		settings.setValue("cutoff", m_ui->cutoffSpinBox->value());
		settings.setValue("averageColors", m_ui->averageColorsCheckBox->isChecked());
	}
	settings.endGroup();
}

void ccCutPursuitDlg::loadFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("CutPursuitDialog");
	{
		m_ui->knnSpinBox->setValue(settings.value("knn", m_ui->knnSpinBox->value()).toInt());
		m_ui->knnRadiusSpinBox->setValue(settings.value("knnRadius", m_ui->knnRadiusSpinBox->value()).toDouble());
		m_ui->regularizationSpinBox->setValue(settings.value("regularization", m_ui->regularizationSpinBox->value()).toDouble());
		m_ui->spatialWeightSpinBox->setValue(settings.value("spatialWeight", m_ui->spatialWeightSpinBox->value()).toDouble());
		m_ui->cutoffSpinBox->setValue(settings.value("cutoff", m_ui->cutoffSpinBox->value()).toInt());
		m_ui->averageColorsCheckBox->setChecked(settings.value("averageColors", m_ui->averageColorsCheckBox->isChecked()).toBool());
	}
	settings.endGroup();
}

void ccCutPursuitDlg::accept()
{
	saveToPersistentSettings();
	QDialog::accept();
}
