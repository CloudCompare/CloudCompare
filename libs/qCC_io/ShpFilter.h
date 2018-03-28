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

//! ESRI Shapefile file filter (output only)
/** See http://www.esri.com/library/whitepapers/pdfs/shapefile.pdf
**/
class QCC_IO_LIB_API ShpFilter : public FileIOFilter
{
public:
	//static accessors
	static inline QString GetFileFilter() { return "SHP entity (*.shp)"; }
	static inline QString GetDefaultExtension() { return "shp"; }

	//inherited from FileIOFilter
	virtual bool importSupported() const override { return true; }
	virtual bool exportSupported() const override { return true; }
	virtual CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;
	virtual QStringList getFileFilters(bool onImport) const override { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const override { return GetDefaultExtension(); }
	virtual bool canLoadExtension(const QString& upperCaseExt) const override;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

	//! Default constructor
	ShpFilter() : FileIOFilter(), m_closedPolylinesAsPolygons(false) {}

	//! Special method to save multiple entities with attributes
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const std::vector<GenericDBFField*>& fields, const QString& filename, const SaveParameters& parameters);

	//! Sets whether to consider closed polylines as polygons or not
	void treatClosedPolylinesAsPolygons(bool state) { m_closedPolylinesAsPolygons = state; }
	//! Returns whether closed polylines are considered as polygons or not
	bool areClosedPolylinesAsPolygons() const { return m_closedPolylinesAsPolygons; }

protected:

	//! Whether to consider closed polylines as polygons or not
	bool m_closedPolylinesAsPolygons;
};

#endif //CC_SHP_SUPPORT

#endif //CC_SHAPEFILE_FILTER_HEADER
