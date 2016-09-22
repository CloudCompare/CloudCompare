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

#include "ccSectionExtractionSubDlg.h"

//system
#include <assert.h>

ccSectionExtractionSubDlg::ccSectionExtractionSubDlg(QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::SectionExtractionSubDlg()
{
	setupUi(this);
}

void ccSectionExtractionSubDlg::setActiveSectionCount(int count)
{
	activeSectionsLabel->setText(QString::number(count));
}

void ccSectionExtractionSubDlg::setSectionThickness(double t)
{
	thicknessDoubleSpinBox->setValue(t);
}

double ccSectionExtractionSubDlg::getSectionThickness() const
{
	return thicknessDoubleSpinBox->value();
}

double ccSectionExtractionSubDlg::getMaxEdgeLength() const
{
	return maxEdgeLengthDoubleSpinBox->value();
}

void ccSectionExtractionSubDlg::setMaxEdgeLength(double l)
{
	maxEdgeLengthDoubleSpinBox->setValue(l);
}

bool ccSectionExtractionSubDlg::extractClouds() const
{
	return extractCloudsGroupBox->isChecked();
}

void ccSectionExtractionSubDlg::doExtractClouds(bool state)
{
	extractCloudsGroupBox->setChecked(state);
}

bool ccSectionExtractionSubDlg::extractContours() const
{
	return extractContoursGroupBox->isChecked();
}

void ccSectionExtractionSubDlg::doExtractContours(bool state, ccContourExtractor::ContourType type)
{
	extractContoursGroupBox->setChecked(state);

	switch(type)
	{
	case ccContourExtractor::LOWER:
		contourTypeComboBox->setCurrentIndex(0);
		break;
	case ccContourExtractor::UPPER:
		contourTypeComboBox->setCurrentIndex(1);
		break;
	case ccContourExtractor::FULL:
		contourTypeComboBox->setCurrentIndex(2);
		break;
	default:
		assert(false);
		break;
	}
}

bool ccSectionExtractionSubDlg::splitContours() const
{
	return splitContourCheckBox->isChecked();
}

void ccSectionExtractionSubDlg::doSplitContours(bool state)
{
	splitContourCheckBox->setChecked(state);
}

bool ccSectionExtractionSubDlg::useMultiPass() const
{
	return multiPassCheckBox->isChecked();
}

void ccSectionExtractionSubDlg::doUseMultiPass(bool state)
{
	multiPassCheckBox->setChecked(state);
}

ccContourExtractor::ContourType ccSectionExtractionSubDlg::getContourType() const
{
	switch(contourTypeComboBox->currentIndex())
	{
	case 0:
		return ccContourExtractor::LOWER;
	case 1:
		return ccContourExtractor::UPPER;
	case 2:
		return ccContourExtractor::FULL;
	default:
		assert(false);
		break;
	}

	return ccContourExtractor::FULL;
}

bool ccSectionExtractionSubDlg::visualDebugMode() const
{
	return debugModeCheckBox->isChecked();
}
