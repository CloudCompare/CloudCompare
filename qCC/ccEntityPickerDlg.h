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

#ifndef CC_ENTITY_PICKER_DIALOG_HEADER
#define CC_ENTITY_PICKER_DIALOG_HEADER

#include <ui_pickEntityDlg.h>

//qCC_db
#include <ccHObject.h>

//! Dialog to select an entity
class ccEntityPickerDlg : public QDialog, public Ui::PickEntityDlg
{
public:
	//! Default constructor
	ccEntityPickerDlg(	const ccHObject::Container& entities,
						int selectedIndex = 0,
						QWidget* parent = 0,
						QString label = QString());

	//! Returns selected index
	int getSelectedIndex() const;

	//! Static shortcut
	static int SelectEntity(const ccHObject::Container& entities,
							int selectedIndex = 0,
							QWidget* parent = 0,
							QString label = QString());

};

#endif //CC_ASK_TWO_DOUBLE_VALUES_DIALOG_HEADER
