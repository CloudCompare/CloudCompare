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
//#                      COPYRIGHT: CloudCompare project                   #
//#                                                                        #
//##########################################################################

#include "ccColorTypes.h"

//Qt
#include <QMap>

class ccHObject;

//! RGB color based entity picking mechanism
class ccColorBasedEntityPicking
{
public:

	//! Unique ID to flag entities
	/** With 24 bits RGB, we can uniquely color up to 2^24 (= 16 777 216) different entities **/
	typedef uint32_t ID_TYPE;

	//! Default constructor
	ccColorBasedEntityPicking() : lastID(0) {}

	//! Reset the structure
	void reset()
	{
		entities.clear();
		ids.clear();
		lastID = 0;
	}

	//! Converts a unique ID to a RGB color
	static inline ccColor::Rgb IDToColor(ID_TYPE id)
	{
		assert(id < (1 << 24));
		return ccColor::Rgb(static_cast<unsigned char>( id        & 255),
							static_cast<unsigned char>((id >> 8 ) & 255),
							static_cast<unsigned char>((id >> 16) & 255));
	}

	//! Converts a RGB color to a unique ID
	static inline ID_TYPE ColorToID(const ccColor::Rgb& col)
	{
		return	  (static_cast<ID_TYPE>(col.r)      )
				| (static_cast<ID_TYPE>(col.g) <<  8)
				| (static_cast<ID_TYPE>(col.b) << 16);
	}

	//! Registeres an entity an returns its corresponding color
	ccColor::Rgb registerEntity(ccHObject* obj)
	{
		if (ids.contains(obj))
		{
			return IDToColor(ids[obj]);
		}
		entities[++lastID] = obj;
		ids[obj] = lastID;
		return IDToColor(lastID);
	}

	//! Returns the entity corresponding to a given color
	inline ccHObject* objectFromColor(ccColor::Rgb color) const { return entities[ColorToID(color)]; }

	//! Returns the last generated ID
	inline ID_TYPE getLastID() const { return lastID; }

protected:

	//! ID/object association map
	QMap<ID_TYPE, ccHObject*> entities;

	//! object/ID association map
	QMap<ccHObject*, ID_TYPE> ids;

	//! Biggest ID value used during the last picking/rendering process
	ID_TYPE lastID;
};
