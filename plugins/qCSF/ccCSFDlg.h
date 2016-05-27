//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qCSF                        #
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
//#               COPYRIGHT: Qi jianbo ; Wan peng                          #
//#                                                                        #
//##########################################################################

#ifndef CC_CSF_DLG_HEADER
#define CC_CSF_DLG_HEADER

#include "ui_CSFDlg.h"

//! Dialog for qCSF plugin
class ccCSFDlg : public QDialog, public Ui::CSFDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccCSFDlg(QWidget* parent = 0);

protected slots:

	//! Saves (temporarily) the dialog paramters on acceptation
	void saveSettings();

};

#endif
