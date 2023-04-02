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

//local
#include "qCC_glWindow.h"

//qCC_db
#include <ccHObject.h>

//Qt
#include <QObject>
#include <QStringList>

//system
#include <unordered_set>

class ccGLWindowInterface;

//! ccGLWindow Signal emitter
class CCGLWINDOW_LIB_API ccGLWindowSignalEmitter : public QObject
{
	Q_OBJECT

public:
	//! Default constructor
	ccGLWindowSignalEmitter(ccGLWindowInterface* associatedWindow, QObject* parent);

	//! Returns the associated window
	inline ccGLWindowInterface* getAssociatedWindow() { return m_associatedWindow; }

Q_SIGNALS:

	//! Signal emitted when an entity is selected in the 3D view
	void entitySelectionChanged(ccHObject* entity);
	//! Signal emitted when multiple entities are selected in the 3D view
	void entitiesSelectionChanged(std::unordered_set<int> entIDs);

	//! Signal emitted when a point (or a triangle) is picked
	/** \param entity 'picked' entity
		\param subEntityID point or triangle index in entity
		\param x mouse cursor x position
		\param y mouse cursor y position
		\param P the picked point
		\param uvw barycentric coordinates of the point (if picked on a mesh)
	**/
	void itemPicked(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P, const CCVector3d& uvw);

	//! Signal emitted when an item is picked (FAST_PICKING mode only)
	/** \param entity entity
		\param subEntityID point or triangle index in entity
		\param x mouse cursor x position
		\param y mouse cursor y position
	**/
	void itemPickedFast(ccHObject* entity, int subEntityID, int x, int y);

	//! Signal emitted when fast picking is finished (FAST_PICKING mode only)
	void fastPickingFinished();

	/*** Camera link mode (interactive modifications of the view/camera are echoed to other windows) ***/

	//! Signal emitted when the window 'model view' matrix is interactively changed
	void viewMatRotated(const ccGLMatrixd& rotMat);
	//! Signal emitted when the mouse wheel is rotated
	void mouseWheelRotated(float wheelDelta_deg);

	//! Signal emitted when the perspective state changes (see setPerspectiveState)
	void perspectiveStateChanged();

	//! Signal emitted when the window 'base view' matrix is changed
	void baseViewMatChanged(const ccGLMatrixd& newViewMat);

	//! Signal emitted when the f.o.v. changes
	void fovChanged(float fov);

	//! Signal emitted when the near clipping depth has been changed
	void nearClippingDepthChanged(double depth);

	//! Signal emitted when the far clipping depth has been changed
	void farClippingDepthChanged(double depth);

	//! Signal emitted when the pivot point is changed
	void pivotPointChanged(const CCVector3d&);

	//! Signal emitted when the camera position is changed
	void cameraPosChanged(const CCVector3d&);

	//! Signal emitted when the selected object is translated by the user
	void translation(const CCVector3d& t);

	//! Signal emitted when the selected object is rotated by the user
	/** \param rotMat rotation applied to current viewport (4x4 OpenGL matrix)
	**/
	void rotation(const ccGLMatrixd& rotMat);

	//! Signal emitted when the left mouse button is clicked on the window
	/** See INTERACT_SIG_LB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void leftButtonClicked(int x, int y);

	//! Signal emitted when the right mouse button is clicked on the window
	/** See INTERACT_SIG_RB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void rightButtonClicked(int x, int y);

	//! Signal emitted when the mouse is moved
	/** See INTERACT_SIG_MOUSE_MOVED.
		The two first arguments correspond to the current cursor coordinates (x,y)
		relative to the window corner!
	**/
	void mouseMoved(int x, int y, Qt::MouseButtons buttons);

	//! Signal emitted when a mouse button is released (cursor on the window)
	/** See INTERACT_SIG_BUTTON_RELEASED.
	**/
	void buttonReleased();

	//! Signal emitted during 3D pass of OpenGL display process
	/** Any object connected to this slot can draw additional stuff in 3D.
		Depth buffering, lights and shaders are enabled by default.
	**/
	void drawing3D();

	//! Signal emitted when files are dropped on the window
	void filesDropped(const QStringList& filenames);

	//! Signal emitted when a new label is created
	void newLabel(ccHObject* obj);

	//! Signal emitted when the exclusive fullscreen is toggled
	void exclusiveFullScreenToggled(bool exclusive);

	//! Signal emitted when the middle mouse button is clicked on the window
	/** See INTERACT_SIG_MB_CLICKED.
		Arguments correspond to the clicked point coordinates (x,y) in
		pixels relative to the window corner!
	**/
	void middleButtonClicked(int x, int y);

	//! Signal emitted when the associated window is about to close
	void aboutToClose(ccGLWindowInterface*);

protected:
	ccGLWindowInterface* m_associatedWindow;
};
