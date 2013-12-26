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

#ifndef CC_ORDER_CHOICE_DIALOG_HEADER
#define CC_ORDER_CHOICE_DIALOG_HEADER

#include <QDialog>

#include <ui_roleChoiceDlg.h>

class ccHObject;

//! Dialog to assign roles to two entities (e.g. compared/reference)
class ccOrderChoiceDlg: public QDialog, public Ui::RoleChoiceDialog
{
    Q_OBJECT

public:

	ccOrderChoiceDlg(ccHObject* firstEntity, const char* firstRole, ccHObject* secondEntity, const char* secondRole, QWidget* parent = 0);
	virtual ~ccOrderChoiceDlg();

    ccHObject* getFirstEntity();
    ccHObject* getSecondEntity();

protected slots:
    void swap();

protected:

    void setColorsAndLabels();

    ccHObject *firstEnt,*secondEnt;
    bool originalOrder;
};

#endif // CC_ORDER_CHOICE_DIALOG_HEADER
