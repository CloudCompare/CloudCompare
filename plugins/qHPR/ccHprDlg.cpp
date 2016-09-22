//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ccHprDlg.h"

#include <ccOctree.h>

ccHprDlg::ccHprDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::HPRDialog()
{
	setupUi(this);

	octreeLevelSpinBox->setRange(2, CCLib::DgmOctree::MAX_OCTREE_LEVEL);
}
