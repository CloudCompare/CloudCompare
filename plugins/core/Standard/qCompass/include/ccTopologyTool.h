//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#ifndef CC_TOPOLOGYTOOL_HEADER
#define CC_TOPOLOGYTOOL_HEADER

#include "ccTool.h"
#include "ccGeoObject.h"
#include "ccTopologyRelation.h"

#include <ccColorTypes.h>
#include <DistanceComputationTools.h>

/*
Tool used to assign topology (timing) relationships between different GeoObjects.
*/
class ccTopologyTool :
	public ccTool
{
public:
	ccTopologyTool();
	virtual ~ccTopologyTool() = default;

	//called when the tool is set to active (for initialization)
	virtual void toolActivated() override;

	//called when the tool is set to disactive (for cleanup)
	virtual void toolDisactivated() override;

	//called when the selection is changed while this tool is active
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;

	//called when "Return" or "Space" is pressed, or the "Accept Button" is clicked
	void accept() override; //do nothing

	//called when the "Escape" is pressed, or the "Cancel" button is clicked
	void cancel() override; //do nothing
protected:
	int m_firstPick = -1; //first object of a (pairwise) topology relationship
public:
	static int RELATIONSHIP; //used to define the topology relationship being assigned (possible values are in ccTopologyRelation)
};

#endif
