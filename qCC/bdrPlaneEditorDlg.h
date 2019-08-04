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

#ifndef BDR_PLANE_EDITOR_DLG_HEADER
#define BDR_PLANE_EDITOR_DLG_HEADER

//Local
#include <ui_bdrPlaneEditorDlg.h>
#include "ccPickingListener.h"

//CCLib
#include <CCGeom.h>

//Qt
#include <QDialog>

class ccGLWindow;
class ccPlane;
class ccHObject;
class ccPickingHub;

//! Dialog to create (or edit the parameters) of a plane
class bdrPlaneEditorDlg : public QDialog, public ccPickingListener, public Ui::BDRPlaneEditorDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrPlaneEditorDlg(ccPickingHub* pickingHub, QWidget* parent);

	//! Destructor
	virtual ~bdrPlaneEditorDlg();

	void updateParams();

	//! Links this dialog with an existing plane
	void initWithPlane(ccPlane* plane);

	//! Updates a plane with the current parameters
	void updatePlane(ccPlane* plane);

	//! Inherited from ccPickingListener
	virtual void onItemPicked(const PickedItem& pi);

public slots:

	void pickPointAsCenter(bool);
	void onDipDirChanged(double);
	void onDipDirModified(bool);
	void onNormalChanged(double);
	void cancle();

protected slots:

	void saveParamsAndAccept();

	void preview();
	void restore();

	

protected: //members

	//! to store the initial plane parameters
	struct planeParams {
		CCVector3 normal;
		CCVector3 center;
		CCVector2 size;
	};
	planeParams m_planePara;

	//! Picking window (if any)
	ccGLWindow* m_pickingWin;

	//! Associated plane (if any)
	ccPlane* m_associatedPlane;

	//! Picking hub
	ccPickingHub* m_pickingHub;
};

#endif
