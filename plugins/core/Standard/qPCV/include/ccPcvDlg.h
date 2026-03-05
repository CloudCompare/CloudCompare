#pragma once

//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN                               #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include <ui_pcvDlg.h>

//! Dialog for the PCV/ShadeVis plugin
class ccPcvDlg : public QDialog, public Ui::PCVDialog
{
public:
	explicit ccPcvDlg(QWidget* parent = nullptr)
		: QDialog(parent, Qt::Tool)
		, Ui::PCVDialog()
	{
		setupUi(this);
	}
};
