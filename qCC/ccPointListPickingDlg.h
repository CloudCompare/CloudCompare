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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2176                                                              $
//$LastChangedDate:: 2012-06-26 22:39:51 +0200 (mar., 26 juin 2012)        $
//**************************************************************************
//

#ifndef CC_POINT_PICKING_LIST_DIALOG_HEADER
#define CC_POINT_PICKING_LIST_DIALOG_HEADER

#include <ui_pointListPickingDlg.h>

#include "ccPointPickingGenericInterface.h"

class cc2DLabel;
class ccHObject;

//! Dialog/interactor to graphically pick a list of points
/** Options let the user to export the list to an ASCII file, to a new cloud, etc.
**/
class ccPointListPickingDlg : public ccPointPickingGenericInterface, public Ui::PointListPickingDlg
{

    Q_OBJECT

public:

    //! Default constructor
	ccPointListPickingDlg(QWidget* parent);

	//! Associates dialog with cloud
	void linkWithCloud(ccPointCloud* cloud);

protected slots:

	//! Applies changes and exit
	void applyAndExit();
    //! Cancels process and exit
    void cancelAndExit();
    //! Exports list to a new cloud
    void exportToNewCloud();
    //! Removes last inserted point from list
    void removeLastEntry();
    //! Exports list to an 'xyz' ASCII file
    void exportToASCII_xyz();
    //! Exports list to an 'ixyz' ASCII file
    void exportToASCII_ixyz();

    //! Redraw window when marker size changes
    void markerSizeChanged(int);
    //! Redraw window when starting index changes
    void startIndexChanged(int);

protected:

    //inherited from ccPointPickingGenericInterface
    void processPickedPoint(ccPointCloud* cloud, unsigned pointIndex, int x, int y);

	//! Gets current (visible) picked points from the associated cloud
	unsigned getPickedPoints(std::vector<cc2DLabel*>& pickedPoints);

    //! Export format
    /** See exportToASCII.
    **/
    enum ExportFormat { PLP_ASCII_EXPORT_XYZ,
                        PLP_ASCII_EXPORT_IXYZ,
    };

    //! Exports list to an ASCII file
    void exportToASCII(ExportFormat format);

    //! Updates point list widget
	void updateList();

	//! Associated cloud
	ccPointCloud* m_associatedCloud;

	//! Last existing label unique ID on load
	unsigned m_lastPreviousID;
	//! Ordered labels container
	ccHObject* m_orderedLabelsContainer;
    //! Existing picked points that the user wants to delete (for proper "cancel" mechanism)
	ccHObject* m_toBeDeleted;
    //! New picked points that the user has selected (for proper "cancel" mechanism)
	ccHObject* m_toBeAdded;
};

#endif
