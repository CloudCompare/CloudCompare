//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "LASOpenDlg.h"

//Qt
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>

//System
#include <string.h>
#include <assert.h>

LASOpenDlg::LASOpenDlg(QWidget* parent)
	: QDialog(parent)
	, Ui::OpenLASFileDialog()
	, m_autoSkip(false)
{
	setupUi(this);

	clearEVLRs();

	connect(applyAllButton, &QAbstractButton::clicked, this, &LASOpenDlg::onApplyAll);
	connect(browseToolButton, &QAbstractButton::clicked, this, &LASOpenDlg::onBrowse);
	connect(tileGroupBox, &QGroupBox::toggled, applyAllButton, &QWidget::setDisabled);

	//can't use the 'Apply all' button if tiling mode is enabled
	applyAllButton->setEnabled(!tileGroupBox->isChecked());

	if (tileGroupBox->isChecked())
	{
		tabWidget->setCurrentIndex(2);
	}
}

void LASOpenDlg::resetApplyAll()
{
	m_autoSkip = false;
}

void LASOpenDlg::onApplyAll()
{
	m_autoSkip = true;
	accept();
}

void LASOpenDlg::onBrowse()
{
	QString outputPath = QFileDialog::getExistingDirectory(this, "Output path", outputPathLineEdit->text());
	if (outputPath.isEmpty())
	{
		//cancelled
		return;
	}

	outputPathLineEdit->setText(outputPath);
}

bool FieldIsPresent(const std::vector<std::string>& dimensions, LAS_FIELDS field)
{
	for (const std::string& dimension : dimensions)
	{
		if (QString(dimension.c_str()).toUpper() == QString(LAS_FIELD_NAMES[field]).toUpper())
			return true;
	}

	return false;
}

void LASOpenDlg::setDimensions(const std::vector<std::string>& dimensions)
{
	redCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_RED));
	greenCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_GREEN));
	blueCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_BLUE));
	intensityCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_INTENSITY));

	bool hasClassif = FieldIsPresent(dimensions,LAS_CLASSIFICATION);
	classifCheckBox->setEnabled(hasClassif);
	decomposeClassifGroupBox->setEnabled(hasClassif);

	timeCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_TIME));

	returnNumberCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_RETURN_NUMBER));
	numberOfReturnsCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_NUMBER_OF_RETURNS));
	scanDirFlagCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_SCAN_DIRECTION));
	edgeOfFlightCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_FLIGHT_LINE_EDGE));
	scanAngleRankCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_SCAN_ANGLE_RANK));
	userDataCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_USER_DATA));
	pointSourceIDCheckBox->setEnabled(FieldIsPresent(dimensions,LAS_POINT_SOURCE_ID));

	//classifValueCheckBox;
	//classifSyntheticCheckBox;
	//classifKeypointCheckBox;
	//classifWithheldCheckBox;
}

bool LASOpenDlg::doLoad(LAS_FIELDS field) const
{
	switch(field)
	{
	case LAS_X:
	case LAS_Y:
	case LAS_Z:
		return true;
	case LAS_INTENSITY:
		return intensityCheckBox->isEnabled() && intensityCheckBox->isChecked();
	case LAS_RETURN_NUMBER:
		return returnNumberCheckBox->isEnabled() && returnNumberCheckBox->isChecked();
	case LAS_NUMBER_OF_RETURNS:
		return numberOfReturnsCheckBox->isEnabled() && numberOfReturnsCheckBox->isChecked();
	case LAS_SCAN_DIRECTION:
		return scanDirFlagCheckBox->isEnabled() && scanDirFlagCheckBox->isChecked();
	case LAS_FLIGHT_LINE_EDGE:
		return edgeOfFlightCheckBox->isEnabled() && edgeOfFlightCheckBox->isChecked();
	case LAS_CLASSIFICATION:
		return classifCheckBox->isEnabled() && classifCheckBox->isChecked() && !decomposeClassifGroupBox->isChecked();
	case LAS_SCAN_ANGLE_RANK:
		return scanAngleRankCheckBox->isEnabled() && scanAngleRankCheckBox->isChecked();
	case LAS_USER_DATA:
		return userDataCheckBox->isEnabled() && userDataCheckBox->isChecked();
	case LAS_POINT_SOURCE_ID:
		return pointSourceIDCheckBox->isEnabled() && pointSourceIDCheckBox->isChecked();
	case LAS_RED:
		return redCheckBox->isEnabled() && redCheckBox->isChecked();
	case LAS_GREEN:
		return greenCheckBox->isEnabled() && greenCheckBox->isChecked();
	case LAS_BLUE:
		return blueCheckBox->isEnabled() && blueCheckBox->isChecked();
	case LAS_TIME:
		return timeCheckBox->isEnabled() && timeCheckBox->isChecked();
	case LAS_EXTRA:
		return extraFieldGroupBox->isEnabled() && extraFieldGroupBox->isChecked();
	case LAS_CLASSIF_VALUE:
		return classifCheckBox->isEnabled() && classifCheckBox->isChecked() && decomposeClassifGroupBox->isChecked() && classifValueCheckBox->isChecked();
	case LAS_CLASSIF_SYNTHETIC:
		return classifCheckBox->isEnabled() && classifCheckBox->isChecked() && decomposeClassifGroupBox->isChecked() && classifSyntheticCheckBox->isChecked();
	case LAS_CLASSIF_KEYPOINT:
		return classifCheckBox->isEnabled() && classifCheckBox->isChecked() && decomposeClassifGroupBox->isChecked() && classifKeypointCheckBox->isChecked();
	case LAS_CLASSIF_WITHHELD:
		return classifCheckBox->isEnabled() && classifCheckBox->isChecked() && decomposeClassifGroupBox->isChecked() && classifWithheldCheckBox->isChecked();
	case LAS_CLASSIF_OVERLAP:
		return classifCheckBox->isEnabled() && classifCheckBox->isChecked() && decomposeClassifGroupBox->isChecked() && classifOverlapCheckBox->isChecked();
	case LAS_INVALID:
	default:
		assert(false);
		return false;
	}

	return false;
}

void LASOpenDlg::clearEVLRs()
{
	evlrListWidget->clear();
	extraFieldGroupBox->setEnabled(false);
	extraFieldGroupBox->setChecked(false);
}

void LASOpenDlg::setInfos(	QString filename,
							unsigned pointCount,
							const CCVector3d& bbMin,
							const CCVector3d& bbMax)
{
	//default output path (for tiling)
	outputPathLineEdit->setText(QFileInfo(filename).absolutePath());

	//number of points
	pointCountLineEdit->setText(QLocale().toString(pointCount));

	//bounding-box
	bbTextEdit->setText(QString("X = [%1 ; %2]\nY = [%3 ; %4]\nZ = [%5 ; %6]")
								.arg(bbMin.x, 0, 'f').arg(bbMax.x, 0, 'f')
								.arg(bbMin.y, 0, 'f').arg(bbMax.y, 0, 'f')
								.arg(bbMin.z, 0, 'f').arg(bbMax.z, 0, 'f'));
}

void LASOpenDlg::addEVLR(QString description)
{
	QListWidgetItem* item = new QListWidgetItem(description);
	evlrListWidget->addItem(item);
	//auto select the entry
	item->setSelected(true);
	//auto enable the extraFieldGroupBox
	extraFieldGroupBox->setEnabled(true);
	extraFieldGroupBox->setChecked(true);
}

bool LASOpenDlg::doLoadEVLR(size_t index) const
{
	if (!extraFieldGroupBox->isChecked())
		return false;
	
	QListWidgetItem* item = evlrListWidget->item(static_cast<int>(index));
	return item && item->isSelected();
}

bool LASOpenDlg::forced8bitRgbMode() const
{
	return force8bitRgbCheckBox->isChecked();
}
