// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#ifndef CC_POINT_PICKING_GENERIC_INTERFACE_HEADER
#define CC_POINT_PICKING_GENERIC_INTERFACE_HEADER

// Local
#include "ccCommon.h"
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"

// CCCoreLib
#include <CCGeom.h>

// system
#include <vector>

class ccGLWindowInterface;
class ccPointCloud;
class ccHObject;
class ccPickingHub;

/** Generic interface for any dialog/graphical interactor that relies on point picking.
 **/
class ccPointPickingGenericInterface : public ccOverlayDialog
    , public ccPickingListener
{
	Q_OBJECT

  public:
	//! Default constructor
	explicit ccPointPickingGenericInterface(ccPickingHub* pickingHub, QWidget* parent = nullptr);
	//! Destructor
	~ccPointPickingGenericInterface() override = default;

	// inherited from ccOverlayDialog
	bool linkWith(ccGLWindowInterface* win) override;
	bool start() override;
	void stop(bool state) override;

	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

  protected:
	//! Generic method to process picked points
	virtual void processPickedPoint(const PickedItem& picked) = 0;

	//! Picking hub
	ccPickingHub* m_pickingHub;
};

#endif
