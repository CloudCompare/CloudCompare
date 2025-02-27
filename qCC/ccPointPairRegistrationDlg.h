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

//Local
#include "ccMainAppInterface.h"
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"

//CCCoreLib
#include <PointProjectionTools.h>

//qCC_db
#include <ccPointCloud.h>

//Qt generated dialog
#include <ui_pointPairRegistrationDlg.h>

class ccGenericPointCloud;
class ccGenericGLDisplay;
class ccGLWindowInterface;
class cc2DLabel;
class ccPickingHub;
class QStringList;

//Dialog for the point-pair registration algorithm (Horn)
class ccPointPairRegistrationDlg : public ccOverlayDialog, public ccPickingListener, Ui::pointPairRegistrationDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccPointPairRegistrationDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent = nullptr);

	//inherited from ccOverlayDialog
	bool linkWith(ccGLWindowInterface* win) override;
	bool start() override;
	void stop(bool state) override;

	//! Inits dialog
	bool init(	ccGLWindowInterface* win,
				const ccHObject::Container& alignedEntities,
				const ccHObject::Container* referenceEntities = nullptr);

	//! Clears dialog
	void clear();

	//! Pauses the dialog
	void pause(bool state);

	//! Adds a point to the 'align' set
	bool addAlignedPoint(CCVector3d& P, ccHObject* entity = nullptr, bool shifted = true);
	//! Adds a point to the 'reference' set
	bool addReferencePoint(CCVector3d& P, ccHObject* entity = nullptr, bool shifted = true);

	//! Removes a point from the 'align' set
	void removeAlignedPoint(int index, bool autoRemoveDualPoint = true);
	//! Removes a point from the 'reference' set
	void removeRefPoint(int index, bool autoRemoveDualPoint = true);

	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

protected:

	//! Slot called to change to-be-aligned entities visibility
	void showAlignedEntities(bool);
	//! Slot called to change reference entities visibility
	void showReferenceEntities(bool);

	//! Slot called to add a manual point to the 'align' set
	void addManualAlignedPoint();
	//! Slot called to add a manual point to the 'reference' set
	void addManualRefPoint();

	//! Slot called to remove the last point on the 'align' stack
	void unstackAligned();
	//! Slot called to remove the last point on the 'reference' stack
	void unstackRef();

	//! Slot called when a "delete" button is pushed
	void onDelButtonPushed();

	//! Updates the registration info and buttons states
	void updateAlignInfo();

	void apply();
	void align();
	void reset();
	void cancel();

protected:

	//! Enables (or not) buttons depending on the number of points in both lists
	void onPointCountChanged();

	//! Calls the registration routine
	bool callRegistration(	CCCoreLib::PointProjectionTools::Transformation& trans,
							double& rms,
							bool autoUpdateTab,
							bool& withUmeyama,
							QStringList* report = nullptr );

	//! Clears the RMS rows
	void clearRMSColumns();

	//! Adds a point to one of the table (ref./to-be-aligned)
	void addPointToTable(	QTableWidget* tableWidget,
							int rowIndex,
							const CCVector3d& P,
							QString pointLabel );

	//! Converts a picked point to a sphere center (if necessary)
	/** \param P input point (may be converted to a sphere center)
		\param entity associated entity
		\param sphereRadius the detected spherer radius (or -1 if no sphere)
		\return whether the point can be used or not
	**/
	bool convertToSphereCenter(CCVector3d& P, ccHObject* entity, PointCoordinateType& sphereRadius);

	//! Resets the displayed title (3D view)
	void resetTitle();

	//! Entity original context
	struct EntityContext
	{
		//! Default constructor
		explicit EntityContext(ccHObject* ent);

		//! Restores cloud original state
		void restore();

		ccHObject* entity;
		ccGenericGLDisplay* originalDisplay;
		bool wasVisible;
		bool wasEnabled;
		bool wasSelected;
	};

	//! Set of contexts
	struct EntityContexts : public QMap< ccHObject*, EntityContext >
	{
		void fill(const ccHObject::Container& entities);

		void restoreAll()
		{
			for (EntityContext& ctx : *this)
				ctx.restore();
		}

		bool isShifted;
		CCVector3d shift;
		double scale = 1.0;
	};

	//! Removes a label (and restore its associated label if any)
	void removeLabel(	ccPointCloud& points,
						unsigned childIndex,
						const EntityContexts& entities);

protected: //members

	//! To-be-aligned entities
	EntityContexts m_alignedEntities;

	//! to-be-aligned points set
	ccPointCloud m_alignedPoints;
	
	//! Reference entities (if any)
	EntityContexts m_referenceEntities;

	//! Reference points set
	ccPointCloud m_refPoints;

	//! Whether the dialog is paused or not
	bool m_paused;

	//! Picking hub
	ccPickingHub* m_pickingHub;

	//! Main application interface
	ccMainAppInterface* m_app;
};
