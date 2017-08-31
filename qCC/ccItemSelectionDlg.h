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
//#                    COPYRIGHT: Daniel Girardeau-Montaut                 #
//#                                                                        #
//##########################################################################

#ifndef CC_ITEM_SELECTION_DIALOG_HEADER
#define CC_ITEM_SELECTION_DIALOG_HEADER

#include <ui_itemSelectionDlg.h>

//qCC_db
#include <ccHObject.h>


//! Dialog to select one or multiple items
class ccItemSelectionDlg : public QDialog, public Ui::ItemSelectionDlg
{
	Q_OBJECT

public: //static shortcuts

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


public:
	
	//! Default constructor
	ccItemSelectionDlg(	bool multiSelectionEnabled,
						QWidget* parent = 0,
						QString itemName = "entities",
						QString label = QString());

	//! Sets the list of items
	void setItems(const QStringList& items, int defaultSelectedIndex = 0);

	//! Returns selected index (unique selection mode)
	int getSelectedIndex() const;

	//! Returns selected indexes (multi-selection mode)
	void getSelectedIndexes(std::vector<int>& indexes) const;

};

#endif //CC_ITEM_SELECTION_DIALOG_HEADER
