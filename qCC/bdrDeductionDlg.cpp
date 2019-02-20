//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qRANSAC_SD                     #
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

#include "bdrDeductionDlg.h"

//local
#include "mainwindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrDeductionDlg::bdrDeductionDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRDeductionDlg()
{
	setupUi(this);

// 	connect(PointcloudFilePathToolButton, &QAbstractButton::clicked, this, &bdr3D4EMDlg::browsePointcloudFilename);
// 	connect(OutputDirFilePathToolButton, &QAbstractButton::clicked, this, &bdr3D4EMDlg::browseOutputDirPath);
// 	connect(ConfigureFilePathToolButton, &QAbstractButton::clicked, this, &bdr3D4EMDlg::browseConfigureFilename);
// 	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}
