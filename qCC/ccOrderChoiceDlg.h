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

#ifndef CC_ORDER_CHOICE_DIALOG_HEADER
#define CC_ORDER_CHOICE_DIALOG_HEADER

//Qt
#include <QDialog>

class ccHObject;
class ccMainAppInterface;
class Ui_RoleChoiceDialog;

//! Dialog to assign roles to two entities (e.g. compared/reference)
class ccOrderChoiceDlg: public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccOrderChoiceDlg(	ccHObject* firstEntity,
						QString firstRole,
						ccHObject* secondEntity,
						QString secondRole,
						ccMainAppInterface* app = 0);

	//! Destructor
	virtual ~ccOrderChoiceDlg();

	//! Returns the first entity (new order)
	ccHObject* getFirstEntity();
	//! Returns the second entity (new order)
	ccHObject* getSecondEntity();

protected slots:

	//! Swaps the entities
	void swap();

protected:

	//! Sets the right colors to the entities and updates the dialog
	void setColorsAndLabels();

	Ui_RoleChoiceDialog* m_gui;
	ccMainAppInterface* m_app;
	ccHObject* m_firstEnt;
	ccHObject* m_secondEnt;
	bool m_useInputOrder;
};

#endif // CC_ORDER_CHOICE_DIALOG_HEADER
