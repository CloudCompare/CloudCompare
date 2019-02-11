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

#ifndef CC_PICK_ONE_ELEMENT_DLG_HEADER
#define CC_PICK_ONE_ELEMENT_DLG_HEADER

//Qt
#include <QDialog>

class Ui_PickOneElementDialog;

//! Minimal dialog to pick one element in a list (combox box)
class ccPickOneElementDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccPickOneElementDlg(const QString &label,
						const QString &windowTitle = QString(),
						QWidget* parent = nullptr);

	//! Destructor
	~ccPickOneElementDlg() override;

	//! Add an element to the combo box
	void addElement(const QString &elementName);
	//! Sets the combo box default index
	void setDefaultIndex(int index);
	//! Returns the combo box current index (after completion)
	int getSelectedIndex();

protected:

	//! Associated UI
	Ui_PickOneElementDialog* m_ui;
};

#endif //CC_PICK_ONE_ELEMENT_DLG_HEADER
