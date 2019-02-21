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

#include "bdr3D4EMDlg.h"

//local
#include "mainwindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

#ifdef USE_STOCKER
#include "builder3d4em/builder3d4em.h"
#endif // USE_STOCKER


bdr3D4EMDlg::bdr3D4EMDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDR3D4EMDlg()
{
	setupUi(this);

	connect(PointcloudFilePathToolButton, &QAbstractButton::clicked, this, &bdr3D4EMDlg::browsePointcloudFilename);
	connect(OutputFilePathToolButton, &QAbstractButton::clicked, this, &bdr3D4EMDlg::browseOutputFilename);
	connect(ConfigureFilePathToolButton, &QAbstractButton::clicked, this, &bdr3D4EMDlg::browseConfigureFilename);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));
}

void bdr3D4EMDlg::browsePointcloudFilename()
{
	QString Filename =
		QFileDialog::getOpenFileName(this,
			"Open PointCloud file",
			PointcloudFilePathLineEdit->text(),
			"Point Cloud (*.ply)");

	if (!Filename.isEmpty())
		PointcloudFilePathLineEdit->setText(Filename);
}

void bdr3D4EMDlg::browseOutputFilename()
{
	QString Filename = 
		QFileDialog::getSaveFileName(this, 
			"Output model file", 
			ConfigureFilePathLineEdit->text(),
			"obj model file (*.obj)");

	if (!Filename.isEmpty())
		OutputFilePathLineEdit->setText(Filename);
}

void bdr3D4EMDlg::browseConfigureFilename()
{
	QString Filename =
		QFileDialog::getOpenFileName(this,
			"Open Configuration file",
			ConfigureFilePathLineEdit->text(),
			"Configuration (*.ini)");

	if (!Filename.isEmpty())
		ConfigureFilePathLineEdit->setText(Filename);
}

void bdr3D4EMDlg::saveSettings()
{
	OutputFilePathLineEdit;
	ConfigureFilePathLineEdit;
}