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
//#                       COPYRIGHT: SAGE INGENIERIE                       #
//#                                                                        #
//##########################################################################

#ifndef CC_PLANE_EDIT_DLG_HEADER
#define CC_PLANE_EDIT_DLG_HEADER

//Local
#include <ui_planeEditDlg.h>

//CCLib
#include <CCGeom.h>

//Qt
#include <QDialog>

class ccGLWindow;
class ccPlane;
class ccHObject;

//! Dialog to create (or edit the parameters) of a plane
class ccPlaneEditDlg : public QDialog, public Ui::PlaneEditDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccPlaneEditDlg(QWidget* parent);

	//! Destructor
	virtual ~ccPlaneEditDlg();

	//! Links this dialog with a given GL window
	void linkWith(ccGLWindow* win);

	//! Links this dialog with an existing plane
	void initWithPlane(ccPlane* plane);

	//! Updates a plane with the current parameters
	void updatePlane(ccPlane* plane);

public slots:

	void pickPointAsCenter(bool);
	void processPickedItem(ccHObject*, unsigned, int, int, const CCVector3&);

protected slots:

	void saveParamsAndAccept();

protected: //members

	//! Associated window (if any)
	ccGLWindow* m_associatedWin;

	//! Associated plane (if any)
	ccPlane* m_associatedPlane;
};

#endif
