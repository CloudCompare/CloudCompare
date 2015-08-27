//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "LASOpenDlg.h"

//Qt
#include <QMessageBox>

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

	connect(applyAllButton, SIGNAL(clicked()), this, SLOT(onApplyAll()));
}

void LASOpenDlg::onApplyAll()
{
	m_autoSkip = true;
	accept();
}

bool FieldIsPresent(const std::vector<std::string>& dimensions, LAS_FIELDS field)
{
	for (std::vector<std::string>::const_iterator it=dimensions.begin(); it!=dimensions.end(); ++it)
	{
		if (QString(it->c_str()).toUpper() == QString(LAS_FIELD_NAMES[field]).toUpper())
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
