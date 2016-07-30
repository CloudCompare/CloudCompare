//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#ifndef QFACET_CELLS_FUSION_DLG_HEADER
#define QFACET_CELLS_FUSION_DLG_HEADER

#include "ui_cellsFusionDlg.h"

//! Dialog for the extraction of facets based on a cell-fusion strategy (qFacets plugin)
class CellsFusionDlg : public QDialog, public Ui::CellsFusionDlg
{
public:

	//! Cell fusion algorithm
	enum Algorithm { ALGO_KD_TREE, ALGO_FAST_MARCHING };

	//! Default constructor
	CellsFusionDlg(Algorithm algo, QWidget* parent = 0)
		: QDialog(parent, Qt::Tool)
		, Ui::CellsFusionDlg()
	{
		setupUi(this);

		switch(algo)
		{
		case ALGO_KD_TREE:
			algoComboBox->setCurrentIndex(0);
			octreeLevelSpinBox->setEnabled(false);
			kdTreeCellFusionGroupBox->setVisible(true);
			fmCellFusionGroupBox->setVisible(false);
			break;
		case ALGO_FAST_MARCHING:
			algoComboBox->setCurrentIndex(1);
			octreeLevelSpinBox->setEnabled(true);
			kdTreeCellFusionGroupBox->setVisible(false);
			fmCellFusionGroupBox->setVisible(true);
			break;
		}
	}
};

#endif //QFACET_CELLS_FUSION_DLG_HEADER
