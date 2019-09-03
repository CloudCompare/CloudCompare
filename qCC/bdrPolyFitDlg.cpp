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

#include "bdrPolyFitDlg.h"

//local
#include "mainwindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrPolyFitDlg::bdrPolyFitDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRPolyFitDlg()
{
	setupUi(this);

// 	connect(PointcloudFilePathToolButton, &QAbstractButton::clicked, this, &bdrSettingLoD2Dlg::browsePointcloudFilename);
// 	connect(OutputDirFilePathToolButton, &QAbstractButton::clicked, this, &bdrSettingLoD2Dlg::browseOutputDirPath);
// 	connect(ConfigureFilePathToolButton, &QAbstractButton::clicked, this, &bdrSettingLoD2Dlg::browseConfigureFilename);
// 	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}
