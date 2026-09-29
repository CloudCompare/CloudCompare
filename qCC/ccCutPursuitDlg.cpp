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
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccCutPursuitDlg.h"

#include <DgmOctree.h>

#include <QSettings>

static const QString s_rgbFeatureName = QObject::tr("RGB");

ccCutPursuitDlg::ccCutPursuitDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , Ui::CutPursuitDialog()
{
	setupUi(this);

	loadFromPersistentSettings();
}

void ccCutPursuitDlg::setScalarFields(const QStringList& sfNames, bool includeRGB/*=false*/)
{
	scalarFieldsListWidget->clear();

	if (includeRGB)
	{
		QListWidgetItem* item = new QListWidgetItem(s_rgbFeatureName, scalarFieldsListWidget);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
	}

	for (const QString& name : sfNames)
	{
		QListWidgetItem* item = new QListWidgetItem(name, scalarFieldsListWidget);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(Qt::Checked);
	}
}

QStringList ccCutPursuitDlg::getSelectedScalarFields() const
{
	QStringList selected;

	for (int i = 0; i < scalarFieldsListWidget->count(); ++i)
	{
		QListWidgetItem* item = scalarFieldsListWidget->item(i);
		if (item && item->checkState() == Qt::Checked && item->text() != s_rgbFeatureName)
		{
			selected.push_back(item->text());
		}
	}

	return selected;
}

int ccCutPursuitDlg::getKNN()
{
	return knnSpinBox->value();
}

double ccCutPursuitDlg::getKNNRadius()
{
	return knnRadiusSpinBox->value();
}

double ccCutPursuitDlg::getRegularization()
{
	return regularizationSpinBox->value();
}

double ccCutPursuitDlg::getSpatialWeight()
{
	return spatialWeightSpinBox->value();
}

int ccCutPursuitDlg::getCutoff()
{
	return cutoffSpinBox->value();
}

bool ccCutPursuitDlg::useRGB()
{
	for (int i = 0; i < scalarFieldsListWidget->count(); ++i)
	{
		QListWidgetItem* item = scalarFieldsListWidget->item(i);
		if (item && item->text() == s_rgbFeatureName)
		{
			return (item->checkState() == Qt::Checked);
		}
	}
	return false;
}

bool ccCutPursuitDlg::averageColors()
{
	return (averageColorsCheckBox->checkState() == Qt::Checked);
}

void ccCutPursuitDlg::saveToPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("CutPursuitDialog");
	{
		settings.setValue("knn", knnSpinBox->value());
		settings.setValue("knnRadius", knnRadiusSpinBox->value());
		settings.setValue("regularization", regularizationSpinBox->value());
		settings.setValue("spatialWeight", spatialWeightSpinBox->value());
		settings.setValue("cutoff", cutoffSpinBox->value());
		settings.setValue("averageColors", averageColorsCheckBox->isChecked());
	}
	settings.endGroup();
}

void ccCutPursuitDlg::loadFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("CutPursuitDialog");
	{
		knnSpinBox->setValue(settings.value("knn", knnSpinBox->value()).toInt());
		knnRadiusSpinBox->setValue(settings.value("knnRadius", knnRadiusSpinBox->value()).toDouble());
		regularizationSpinBox->setValue(settings.value("regularization", regularizationSpinBox->value()).toDouble());
		spatialWeightSpinBox->setValue(settings.value("spatialWeight", spatialWeightSpinBox->value()).toDouble());
		cutoffSpinBox->setValue(settings.value("cutoff", cutoffSpinBox->value()).toInt());
		averageColorsCheckBox->setChecked(settings.value("averageColors", averageColorsCheckBox->isChecked()).toBool());
	}
	settings.endGroup();
}

void ccCutPursuitDlg::accept()
{
	saveToPersistentSettings();
	QDialog::accept();
}
