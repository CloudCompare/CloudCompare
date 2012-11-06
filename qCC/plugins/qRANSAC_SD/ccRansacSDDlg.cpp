//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qRANSAC_SD                    #
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
//$Rev:: 1790                                                              $
//$LastChangedDate:: 2011-02-14 17:31:03 +0100 (lun., 14 févr. 2011)      $
//**************************************************************************
//

#include "ccRansacSDDlg.h"

#include <ccOctree.h>

static double s_normalThresh = .9;	// this is the cos of the maximal normal deviation
static int s_minSupport = 500;		// this is the minimal numer of points required for a primitive
static double s_probability = .01;	// this is the "probability" with which a primitive is overlooked


ccRansacSDDlg::ccRansacSDDlg(QWidget* parent) : QDialog(parent), Ui::RansacSDDialog()
{
    setupUi(this);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	supportPointsSpinBox->setValue(s_minSupport);
	normThreshDoubleSpinBox->setValue(s_normalThresh);
	probaDoubleSpinBox->setValue(s_probability);
}

void ccRansacSDDlg::saveSettings()
{
	s_minSupport = supportPointsSpinBox->value();
	s_normalThresh = normThreshDoubleSpinBox->value();
	s_probability = probaDoubleSpinBox->value();
}