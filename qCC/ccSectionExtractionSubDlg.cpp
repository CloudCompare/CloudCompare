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

#include "ccSectionExtractionSubDlg.h"

//system
#include <assert.h>

ccSectionExtractionSubDlg::ccSectionExtractionSubDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::SectionExtractionSubDlg()
{
	setupUi(this);
	setWindowFlags(Qt::Tool);
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

void ccSectionExtractionSubDlg::doExtractContours(bool state)
{
	extractContoursGroupBox->setChecked(state);
}

ccSectionExtractionSubDlg::ContourType ccSectionExtractionSubDlg::getContourType() const
{
	switch(contourTypeComboBox->currentIndex())
	{
	case 0:
		return LOWER;
	case 1:
		return UPPER;
	case 2:
		return FULL;
	default:
		assert(false);
		break;
	}

	return FULL;
}
