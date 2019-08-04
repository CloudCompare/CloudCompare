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

#ifndef CC_POINT_PICKING_LIST_DIALOG_HEADER
#define CC_POINT_PICKING_LIST_DIALOG_HEADER

//GUI
#include <ui_pointListPickingDlg.h>

//Local
#include "ccPointPickingGenericInterface.h"

//qCC_db
#include <ccHObject.h>

class cc2DLabel;

//! Dialog/interactor to graphically pick a list of points
/** Options let the user export the list to an ASCII file, a new cloud, a polyline, etc.
**/
class ccPointListPickingDlg : public ccPointPickingGenericInterface, public Ui::PointListPickingDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccPointListPickingDlg(ccPickingHub* pickingHub, QWidget* parent);

	//! Associates dialog with a cloud or a mesh
	void linkWithEntity(ccHObject* entity);

protected slots:

	//! Applies changes and exit
	void applyAndExit();
	//! Cancels process and exit
	void cancelAndExit();
	//! Exports list to a new cloud
	void exportToNewCloud();
	//! Exports list to a polyline
	void exportToNewPolyline();
	//! Removes last inserted point from list
	void removeLastEntry();
	//! Exports list to an 'xyz' ASCII file
	inline void exportToASCII_xyz() { return exportToASCII(PLP_ASCII_EXPORT_XYZ); }
	//! Exports list to an 'ixyz' ASCII file
	inline void exportToASCII_ixyz() { return exportToASCII(PLP_ASCII_EXPORT_IXYZ); }
	//! Exports list to an 'gxyz' ASCII file
	inline void exportToASCII_gxyz() { return exportToASCII(PLP_ASCII_EXPORT_GXYZ); }
	//! Exports list to an 'lxyz' ASCII file
	inline void exportToASCII_lxyz() { return exportToASCII(PLP_ASCII_EXPORT_LXYZ); }

	//! Redraw window when marker size changes
	void markerSizeChanged(int);
	//! Redraw window when starting index changes
	void startIndexChanged(int);
	//! Updates point list widget
	void updateList();

protected:

	//inherited from ccPointPickingGenericInterface
	void processPickedPoint(const PickedItem& picked) override;

	//! Gets current (visible) picked points from the associated cloud
	unsigned getPickedPoints(std::vector<cc2DLabel*>& pickedPoints);

	//! Export format
	/** See exportToASCII.
	**/
	enum ExportFormat {	PLP_ASCII_EXPORT_XYZ,
						PLP_ASCII_EXPORT_IXYZ,
						PLP_ASCII_EXPORT_GXYZ,
						PLP_ASCII_EXPORT_LXYZ
	};

	//! Exports list to an ASCII file
	void exportToASCII(ExportFormat format);

	//! Associated cloud or mesh
	ccHObject* m_associatedEntity;

	//! Last existing label unique ID on load
	unsigned m_lastPreviousID;
	//! Ordered labels container
	ccHObject* m_orderedLabelsContainer;
	//! Existing picked points that the user wants to delete (for proper "cancel" mechanism)
	ccHObject::Container m_toBeDeleted;
	//! New picked points that the user has selected (for proper "cancel" mechanism)
	ccHObject::Container m_toBeAdded;
};

#endif
