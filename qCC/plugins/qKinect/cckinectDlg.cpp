//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qKinect                     #
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

#include "ccKinectDlg.h"

#include <ccOctree.h>

ccKinectDlg::ccKinectDlg(QWidget* parent) : QDialog(parent), Ui::KinectDialog()
{
    setupUi(this);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);
}

QString ccKinectDlg::getCloudName() const
{
	return cloudNameLineEdit->text();
}

bool ccKinectDlg::grabRGBInfo()
{
	return grabRGBCheckBox->isChecked();
}

void ccKinectDlg::addMode(const QString& mode)
{
	resolutionComboBox->addItem(mode);
}

unsigned char ccKinectDlg::getFrameAveragingCount() const
{
	return (unsigned char)std::min(frameAvgSpinBox->value(),255);
}
