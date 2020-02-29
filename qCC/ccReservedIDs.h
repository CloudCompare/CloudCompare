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

#ifndef CC_RESERVED_IDS_HEADER
#define CC_RESERVED_IDS_HEADER

//! Unique IDs reserved by CloudCompare for special entities (display elements, etc.)
/** They should all remain below ccUniqueIDGenerator::MinUniqueID (256)
**/
enum class ReservedIDs : unsigned
{
	CLIPPING_BOX = 1,
	INTERACTIVE_SEGMENTATION_TOOL_POLYLINE = 2,
	INTERACTIVE_SEGMENTATION_TOOL_POLYLINE_VERTICES = 3,
	TRACE_POLYLINE_TOOL_POLYLINE_TIP = 4,
	TRACE_POLYLINE_TOOL_POLYLINE_TIP_VERTICES = 5,
	TRACE_POLYLINE_TOOL_POLYLINE = 6,
	TRACE_POLYLINE_TOOL_POLYLINE_VERTICES = 7,
};

#endif //CC_RESERVED_IDS_HEADER
