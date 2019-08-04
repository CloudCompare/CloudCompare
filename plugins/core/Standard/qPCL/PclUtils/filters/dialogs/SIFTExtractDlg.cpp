//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "SIFTExtractDlg.h"

SIFTExtractDlg::SIFTExtractDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::SIFTExtractDlg()
{
	setupUi(this);
}

void SIFTExtractDlg::updateComboBox(const std::vector<std::string>& fields)
{
	intensityCombo->clear();
	for (size_t i = 0; i < fields.size(); i++)
	{
		intensityCombo->addItem(QString(fields[i].c_str()));
	}
}


