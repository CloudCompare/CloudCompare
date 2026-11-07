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

#include "ccSaveIndividualFilesDlg.h"

// qCC_io
#include <FileIOFilter.h>

// CCPluginAPI
#include <ccPersistentSettings.h>

// qCC_db
#include <ccFileUtils.h>

// Qt
#include <QDir>
#include <QFileDialog>
#include <QPushButton>
#include <QSettings>

// persistent settings keys (in the ccPS::SaveFile() group)
static const QString s_folderStructureKey   = QStringLiteral("individualFilesFolderStructure");
static const QString s_overwriteKey         = QStringLiteral("individualFilesOverwrite");
static const QString s_showFilterDialogsKey = QStringLiteral("individualFilesShowFilterDialogs");

// Helper: fills a combo-box with the 'file filters' of all the I/O filters able to save a given type of entity
static void FillFormatComboBox(QComboBox* comboBox, CC_CLASS_ENUM type, const QString& defaultFilter)
{
	for (const FileIOFilter::Shared& filter : FileIOFilter::GetFilters())
	{
		bool isExclusive = true;
		bool multiple    = false;
		if (filter->canSave(type, multiple, isExclusive))
		{
			for (const QString& fileFilter : filter->getFileFilters(false))
			{
				comboBox->addItem(fileFilter);
			}
		}
	}

	int index = comboBox->findText(defaultFilter);
	if (index >= 0)
	{
		comboBox->setCurrentIndex(index);
	}
}

ccSaveIndividualFilesDlg::ccSaveIndividualFilesDlg(unsigned cloudCount, unsigned meshCount, unsigned polylineCount, QWidget* parent /*=nullptr*/)
    : QDialog(parent)
    , Ui::SaveIndividualFilesDlg()
{
	setupUi(this);

	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());

	outputFolderLineEdit->setText(QDir::toNativeSeparators(settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString()));

	FillFormatComboBox(cloudFormatComboBox, CC_TYPES::POINT_CLOUD, settings.value(ccPS::SelectedOutputFilterCloud()).toString());
	FillFormatComboBox(meshFormatComboBox, CC_TYPES::MESH, settings.value(ccPS::SelectedOutputFilterMesh()).toString());
	FillFormatComboBox(polylineFormatComboBox, CC_TYPES::POLY_LINE, settings.value(ccPS::SelectedOutputFilterPoly()).toString());

	// only show the formats of the types that will actually be saved
	cloudFormatLabel->setVisible(cloudCount != 0);
	cloudFormatComboBox->setVisible(cloudCount != 0);
	meshFormatLabel->setVisible(meshCount != 0);
	meshFormatComboBox->setVisible(meshCount != 0);
	polylineFormatLabel->setVisible(polylineCount != 0);
	polylineFormatComboBox->setVisible(polylineCount != 0);

	folderStructureComboBox->setCurrentIndex(settings.value(s_folderStructureKey, static_cast<int>(FolderStructure::FromSelection)).toInt());
	overwriteCheckBox->setChecked(settings.value(s_overwriteKey, false).toBool());
	showFilterDialogsCheckBox->setChecked(settings.value(s_showFilterDialogsKey, false).toBool());

	settings.endGroup();

	summaryLabel->setText(tr("Each entity will be saved in its own file: %1 cloud(s), %2 mesh(es), %3 polyline(s).")
	                          .arg(cloudCount)
	                          .arg(meshCount)
	                          .arg(polylineCount));

	connect(browseToolButton, &QToolButton::clicked, this, &ccSaveIndividualFilesDlg::browseOutputFolder);
	connect(outputFolderLineEdit, &QLineEdit::textChanged, this, &ccSaveIndividualFilesDlg::updateOkButton);

	updateOkButton();
}

QString ccSaveIndividualFilesDlg::outputFolder() const
{
	return QDir::fromNativeSeparators(outputFolderLineEdit->text().trimmed());
}

QString ccSaveIndividualFilesDlg::cloudFilter() const
{
	return cloudFormatComboBox->currentText();
}

QString ccSaveIndividualFilesDlg::meshFilter() const
{
	return meshFormatComboBox->currentText();
}

QString ccSaveIndividualFilesDlg::polylineFilter() const
{
	return polylineFormatComboBox->currentText();
}

ccSaveIndividualFilesDlg::FolderStructure ccSaveIndividualFilesDlg::folderStructure() const
{
	return static_cast<FolderStructure>(folderStructureComboBox->currentIndex());
}

bool ccSaveIndividualFilesDlg::overwriteExisting() const
{
	return overwriteCheckBox->isChecked();
}

bool ccSaveIndividualFilesDlg::showFilterDialogs() const
{
	return showFilterDialogsCheckBox->isChecked();
}

void ccSaveIndividualFilesDlg::saveSettings() const
{
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());

	settings.setValue(ccPS::CurrentPath(), outputFolder());
	if (!cloudFormatComboBox->isHidden())
		settings.setValue(ccPS::SelectedOutputFilterCloud(), cloudFilter());
	if (!meshFormatComboBox->isHidden())
		settings.setValue(ccPS::SelectedOutputFilterMesh(), meshFilter());
	if (!polylineFormatComboBox->isHidden())
		settings.setValue(ccPS::SelectedOutputFilterPoly(), polylineFilter());
	settings.setValue(s_folderStructureKey, folderStructureComboBox->currentIndex());
	settings.setValue(s_overwriteKey, overwriteExisting());
	settings.setValue(s_showFilterDialogsKey, showFilterDialogs());

	settings.endGroup();
}

void ccSaveIndividualFilesDlg::browseOutputFolder()
{
	QString folder = QFileDialog::getExistingDirectory(this, tr("Output folder"), outputFolder());
	if (!folder.isEmpty())
	{
		outputFolderLineEdit->setText(QDir::toNativeSeparators(folder));
	}
}

void ccSaveIndividualFilesDlg::updateOkButton()
{
	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(!outputFolder().isEmpty());
}
