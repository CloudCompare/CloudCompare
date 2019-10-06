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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef CC_PICKING_LISTENER_HEADER
#define CC_PICKING_LISTENER_HEADER

//CCLib
#include <CCGeom.h>

//Qt
#include <QPoint>

class ccHObject;

//! Point/triangle picking listener interface
class ccPickingListener
{
public:
	virtual ~ccPickingListener() = default;
	
	//! Picked item
	struct PickedItem
	{
		PickedItem()
			: entity(nullptr)
			, itemIndex(0)
			, entityCenter(false)
		{}

		QPoint clickPoint; //position of the user click
		ccHObject* entity; //picked entity (if any)
		unsigned itemIndex; //e.g. point or triangle index
		CCVector3 P3D; //picked point in 3D (if any)
		CCVector3d uvw; //picked point barycentric coordinates (if picked on a triangle)
		bool entityCenter; //the point doesn't correspond to a real 'item' but to the entity center
	};

	//! Method called whenever an item is picked
	virtual void onItemPicked(const PickedItem& pi) = 0;
};

#endif //CC_PICKING_LISTENER_HEADER
