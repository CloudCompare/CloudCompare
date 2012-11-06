//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#include "SIFTExtractDlg.h"

SIFTExtractDlg::SIFTExtractDlg(QWidget* parent) : QDialog(parent), Ui::SIFTExtractDlg()
{
	setupUi(this);
	setWindowFlags(Qt::Tool/* Qt::Dialog /* | Qt::WindowStaysOnTopHint*/);
}

void SIFTExtractDlg::updateComboBox(std::vector<std::string> fields)
{
    intensityCombo->clear();
    for (unsigned i = 0; i < fields.size(); i++)
    {
        intensityCombo->addItem(QString(fields[i].c_str()));
    }

}


