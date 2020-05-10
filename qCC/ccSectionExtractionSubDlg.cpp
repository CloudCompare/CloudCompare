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

bool ccSectionExtractionSubDlg::extractEnvelopes() const
{
	return extractEnvelopesGroupBox->isChecked();
}

void ccSectionExtractionSubDlg::doExtractEnvelopes(bool state, ccEnvelopeExtractor::EnvelopeType type)
{
	extractEnvelopesGroupBox->setChecked(state);

	switch(type)
	{
	case ccEnvelopeExtractor::LOWER:
		envelopeTypeComboBox->setCurrentIndex(0);
		break;
	case ccEnvelopeExtractor::UPPER:
		envelopeTypeComboBox->setCurrentIndex(1);
		break;
	case ccEnvelopeExtractor::FULL:
		envelopeTypeComboBox->setCurrentIndex(2);
		break;
	default:
		assert(false);
		break;
	}
}

bool ccSectionExtractionSubDlg::splitEnvelopes() const
{
	return splitEnvelopeCheckBox->isChecked();
}

void ccSectionExtractionSubDlg::doSplitEnvelopes(bool state)
{
	splitEnvelopeCheckBox->setChecked(state);
}

bool ccSectionExtractionSubDlg::useMultiPass() const
{
	return multiPassCheckBox->isChecked();
}

void ccSectionExtractionSubDlg::doUseMultiPass(bool state)
{
	multiPassCheckBox->setChecked(state);
}

ccEnvelopeExtractor::EnvelopeType ccSectionExtractionSubDlg::getEnvelopeType() const
{
	switch(envelopeTypeComboBox->currentIndex())
	{
	case 0:
		return ccEnvelopeExtractor::LOWER;
	case 1:
		return ccEnvelopeExtractor::UPPER;
	case 2:
		return ccEnvelopeExtractor::FULL;
	default:
		assert(false);
		break;
	}

	return ccEnvelopeExtractor::FULL;
}

bool ccSectionExtractionSubDlg::visualDebugMode() const
{
	return debugModeCheckBox->isChecked();
}
