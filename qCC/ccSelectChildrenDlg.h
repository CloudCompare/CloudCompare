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

#ifndef CC_SELECT_CHILDREN_DLG_HEADER
#define CC_SELECT_CHILDREN_DLG_HEADER

#include <ui_selectChildrenDlg.h>

//Qt
#include <QDialog>

//qCC_db
#include <ccObject.h>

//! Minimal dialog to pick one element in a list (combo box)
class ccSelectChildrenDlg : public QDialog, public Ui::SelectChildrenDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSelectChildrenDlg(QWidget* parent = 0);

	//! Add an element to the 'type' combo box
	void addType(QString typeName, CC_CLASS_ENUM type);

	//! Returns the selected type
	CC_CLASS_ENUM getSelectedType();
	//! Returns the selected name (if any)
	QString getSelectedName();
	//! Returns the state of the strict type checkbox
	bool getStrictMatchState() const;

	//! if the type checkbox is checked the children are filtered
	//! before checking the name for matches
	bool getTypeIsUsed() const;

	//! if the name must be considerd as regex
	bool getNameIsRegex() const;

	//! if performing name-match (regex or not)
	bool getNameMatchIsUsed() const;

protected slots:

	//! Called when the dialog is accepted
	void onAccept();

};

#endif //CC_SELECT_CHILDREN_DLG_HEADER
