//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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

#ifndef CC_HPR_DLG_HEADER
#define CC_HPR_DLG_HEADER

#include "ui_hprDlg.h"

//! Dialog for the qHPR plugin
class ccHprDlg : public QDialog, public Ui::HPRDialog
{
public:

	//! Default constructor
	explicit ccHprDlg(QWidget* parent = 0);
};

#endif
