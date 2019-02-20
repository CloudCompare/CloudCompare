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

#include "bdrLine3DppDlg.h"

//local
#include "mainwindow.h"

#include <ccOctree.h>

#ifdef USE_STOCKER
#include "builderline3d/builderline3d.h"
#endif // USE_STOCKER


bdrLine3DppDlg::bdrLine3DppDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRLine3DppDlg()
{
	setupUi(this);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(GenerateLines()));

	if (MainWindow::TheInstance()) {
		m_win = MainWindow::TheInstance();
	}
}

void bdrLine3DppDlg::GenerateLines()
{
	saveSettings();

#ifndef USE_STOCKER
	if (m_win) {
		m_win->dispToConsole("[Line3DPP] No stocker lib used!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
	return;
#endif // USE_STOCKER

	if (m_win) {
//		m_win->addToDB(plane);
	}
}

void bdrLine3DppDlg::saveSettings()
{

}