//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2050                                                              $
//$LastChangedDate:: 2012-03-27 00:51:40 +0200 (mar., 27 mars 2012)        $
//**************************************************************************
//

#include "ccHprDlg.h"

#include <ccOctree.h>

ccHprDlg::ccHprDlg(QWidget* parent) : QDialog(parent), Ui::HPRDialog()
{
    setupUi(this);

	octreeLevelSpinBox->setRange(2,CCLib::DgmOctree::MAX_OCTREE_LEVEL);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);
}
