#pragma once
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
#include "CCAppCommon.h"

//Qt
#include <QDialog>

class Ui_PickOneElementDialog;
//! \brief Minimal dialog to pick one element in a list (combox box)
//!
//! This dialog presents a combo box containing the elements of the list
//! passed to the constructor. A label and a window title can also be
//! passed.
//!
//! Example of usage:
//! \code
//! QStringList elements = {"Element 1", "Element 2", "Element 3"};
//! ccPickOneElementDlg dlg(elements, "Pick one element", "My dialog");
//! if (dlg.exec())
//! {
//!     int index = dlg.getSelectedIndex();
//!     // Do something with the selected index
//! }
//! \endcode
class CCAPPCOMMON_LIB_API ccPickOneElementDlg : public QDialog
{
	Q_OBJECT

public:

	/**
	 * \brief Default constructor
	 *
	 * \param label the label to be displayed
	 * \param windowTitle the window title
	 * \param parent the parent widget
	 */
	ccPickOneElementDlg(const QString &label,
						const QString &windowTitle = QString(),
						QWidget* parent = nullptr);

	//! Destructor
	~ccPickOneElementDlg() override;

	/**
	 * \brief Add an element to the combo box
	 *
	 * \param elementName the element name to add
	 */
	void addElement(const QString &elementName);

	/**
	 * \brief Sets the combo box default index
	 *
	 * \param index the index to set
	 */
	void setDefaultIndex(int index);

	/**
	 * \brief Returns the combo box current index (after completion)
	 */
	int getSelectedIndex();

private:

	//! Associated UI
	Ui_PickOneElementDialog* m_ui;
};
