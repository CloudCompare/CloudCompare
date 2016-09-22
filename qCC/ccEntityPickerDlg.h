//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_ENTITY_PICKER_DIALOG_HEADER
#define CC_ENTITY_PICKER_DIALOG_HEADER

#include <ui_pickEntityDlg.h>

//qCC_db
#include <ccHObject.h>


//! Dialog to select one or multiple entities
class ccEntityPickerDlg : public QDialog, public Ui::PickEntityDlg
{
public:
	//! Default constructor
	ccEntityPickerDlg(	const ccHObject::Container& entities,
						bool multiSelectionEnabled,
						int defaultSelectedIndex = 0,
						QWidget* parent = 0,
						QString label = QString());

	//! Returns selected index (unique selection mode)
	int getSelectedIndex() const;

	//! Returns selected indexes (multi-selection mode)
	void getSelectedIndexes(std::vector<int>& indexes) const;

	//! Static shortcut: unique selection mode
	static int SelectEntity(const ccHObject::Container& entities,
							int defaultSelectedIndex = 0,
							QWidget* parent = 0,
							QString label = QString());

	//! Static shortcut: multi-selection mode
	static bool SelectEntities(	const ccHObject::Container& entities,
								std::vector<int>& indexes,
								QWidget* parent = 0,
								QString label = QString());

};

#endif //CC_ENTITY_PICKER_DIALOG_HEADER
