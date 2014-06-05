//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_PICK_ONE_ELEMENT_DLG_HEADER
#define CC_PICK_ONE_ELEMENT_DLG_HEADER

#include <ui_pickOneElementDlg.h>

//! Minimal dialog to pick one element in a list (combox box)
class ccPickOneElementDlg : public QDialog, public Ui::PickOneElementDialog
{
public:

	//! Default constructor
	ccPickOneElementDlg(QString label,
						QString windowTitle=QString(),
						QWidget* parent = 0);

	//! Add an element to the combo box
	void addElement(QString elementName);
	//! Sets the combo box default index
	void setDefaultIndex(int index);
	//! Returns the combo box current index (after completion)
	int getSelectedIndex();
};

#endif //CC_PICK_ONE_ELEMENT_DLG_HEADER
