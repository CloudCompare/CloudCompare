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

#ifndef CC_SHAPEFILE_FILTER_HEADER
#define CC_SHAPEFILE_FILTER_HEADER

#ifdef CC_SHP_SUPPORT

//qCC_io
#include <FileIOFilter.h>

//Qt
#include <QString>

//system
#include <vector>

class GenericDBFField;

//! ESRI Shapefile file filter
/** See http://www.esri.com/library/whitepapers/pdfs/shapefile.pdf
**/
class QCC_IO_LIB_API ShpFilter : public FileIOFilter
{
public:
	ShpFilter();
	
	//inherited from FileIOFilter
	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;

	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

	//! Special method to save multiple entities with attributes
	CC_FILE_ERROR saveToFile(ccHObject* entity, const std::vector<GenericDBFField*>& fields, const QString& filename, const SaveParameters& parameters);

	//! Sets whether to consider closed polylines as polygons or not
	void treatClosedPolylinesAsPolygons(bool state) { m_closedPolylinesAsPolygons = state; }
	//! Returns whether closed polylines are considered as polygons or not
	bool areClosedPolylinesAsPolygons() const { return m_closedPolylinesAsPolygons; }

	//! Sets whether to save polyline as 2D
	void save3DPolyAs2D(bool state) { m_save3DPolyAs2D = state; }

	//! Sets whether to save polyline's height in .dbf
	void save3DPolyHeightInDBF(bool state) { m_save3DPolyHeightInDBF = state; }

private:
	//! Whether to consider closed polylines as polygons or not
	bool m_closedPolylinesAsPolygons = false;

	//! Whether to save 3D poly as 2D
	//! Note that all Polylines from shapefiles are loaded as 3D
	bool m_save3DPolyAs2D = false;

	//! Whether to save the 3D height in .dbf file
	bool m_save3DPolyHeightInDBF = false;
};

#endif //CC_SHP_SUPPORT

#endif //CC_SHAPEFILE_FILTER_HEADER
