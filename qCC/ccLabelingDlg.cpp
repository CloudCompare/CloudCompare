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

#include "ccLabelingDlg.h"

#include <DgmOctree.h>

ccLabelingDlg::ccLabelingDlg(QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::LabelingDialog()
{
	setupUi(this);

	octreeLevelSpinBox->setMaximum(CCLib::DgmOctree::MAX_OCTREE_LEVEL);
}

int ccLabelingDlg::getOctreeLevel()
{
	return octreeLevelSpinBox->value();
}

int ccLabelingDlg::getMinPointsNb()
{
	return minPtsSpinBox->value();
}

bool ccLabelingDlg::randomColors()
{
	return (randomColorsCheckBox->checkState()==Qt::Checked);
}
