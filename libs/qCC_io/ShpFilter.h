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

#ifndef CC_SHAPEFILE_FILTER_HEADER
#define CC_SHAPEFILE_FILTER_HEADER

#ifdef CC_SHP_SUPPORT

//qCC_io
#include <FileIOFilter.h>

//Qt
#include <QString>

//system
#include <vector>

//Shapelib
#include <shapefil.h>

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
	virtual bool importSupported() const { return true; }
	virtual bool exportSupported() const { return true; }
	virtual CC_FILE_ERROR loadFile(QString filename, ccHObject& container, LoadParameters& parameters);
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, SaveParameters& parameters);
	virtual QStringList getFileFilters(bool onImport) const { return QStringList(GetFileFilter()); }
	virtual QString getDefaultExtension() const { return GetDefaultExtension(); }
	virtual bool canLoadExtension(QString upperCaseExt) const;
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const;

	//! Default constructor
	ShpFilter() : FileIOFilter(), m_closedPolylinesAsPolygons(false) {}

	//! Generic shapefile 'field'
	/** Fields contain one value per record (i.e. primitive)
	**/
	class GenericField
	{
	public:

		//! Default constructor
		GenericField(QString name) : m_name(name) {}

		//! Returns field name
		const QString& name() const { return m_name; }

		//! Returns whehter the field is 3D or not
		virtual bool is3D() const  { return false; }

		//to be reimplemented by siblings
		virtual DBFFieldType type() const = 0;
		virtual int width() const = 0;
		virtual int decimal() const = 0;
		virtual bool save(DBFHandle handle, int fieldIndex) const { return false; } //1D version
		virtual bool save(DBFHandle handle, int xFieldIndex, int yFieldIndex, int zFieldIndex) const { return false; }; //3D version

	protected:

		//! Field name
		QString m_name;
	};

	//! Int-valued shapefile field
	class QCC_IO_LIB_API IntegerField : public GenericField
	{
	public:

		//! Default constructor
		IntegerField(QString name) : GenericField(name) {}
		
		//inherited from GenericField
		virtual DBFFieldType type() const { return FTInteger; }
		virtual int width() const { return 6; }
		virtual int decimal() const { return 0; }
		virtual bool save(DBFHandle handle, int fieldIndex) const;

		//! Field values
		std::vector<int> values;
	};

	//! Double-valued shapefile field
	class QCC_IO_LIB_API DoubleField : public GenericField
	{
	public:

		//! Default constructor
		DoubleField(QString name) : GenericField(name) {}
		
		//inherited from GenericField
		virtual DBFFieldType type() const { return FTDouble; }
		virtual int width() const { return 8; }
		virtual int decimal() const { return 8; }
		virtual bool save(DBFHandle handle, int fieldIndex) const;

		//! Field values
		std::vector<double> values;
	};

	//! Double-valued 3D shapefile field
	class QCC_IO_LIB_API DoubleField3D : public GenericField
	{
	public:

		//! Default constructor
		DoubleField3D(QString name) : GenericField(name) {}
		
		//inherited from GenericField
		virtual bool is3D() const  { return true; }
		virtual DBFFieldType type() const { return FTDouble; }
		virtual int width() const { return 8; }
		virtual int decimal() const { return 8; }
		virtual bool save(DBFHandle handle, int xFieldIndex, int yFieldIndex, int zFieldIndex) const;

		//! Field values
		std::vector<CCVector3d> values;
	};

	//! Special method to save multiple entities with attributes
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, const std::vector<GenericField*>& fields, QString filename, SaveParameters& parameters);

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
