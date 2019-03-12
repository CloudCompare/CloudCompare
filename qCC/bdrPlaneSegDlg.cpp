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

#include "bdrPlaneSegDlg.h"

//local
#include "mainwindow.h"

#include <ccOctree.h>

bdrPlaneSegDlg::bdrPlaneSegDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRPlaneSegDlg()
{
	setupUi(this);

//	connect(buttonBox, SIGNAL(accepted()), this, SLOT(GenerateLines()));

	if (MainWindow::TheInstance()) {
		m_win = MainWindow::TheInstance();
	}
}

