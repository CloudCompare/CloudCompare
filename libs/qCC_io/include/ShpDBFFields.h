#pragma once

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

#ifdef CC_SHP_SUPPORT

// local
#include "qCC_io.h"

// CCCoreLib
#include <CCGeom.h>

// Qt
#include <QString>

// system
#include <vector>

// Shapelib
#include <shapefil.h>

//! Generic shapefile 'field'
/** Fields contain one value per record (i.e. primitive)
 **/
class QCC_IO_LIB_API GenericDBFField
{
  public:
	//! Default constructor
	explicit GenericDBFField(QString name)
	    : m_name(name)
	{
	}

	//! default destructor
	virtual ~GenericDBFField() = default;

	//! Returns field name
	const QString& name() const
	{
		return m_name;
	}

	//! Returns whehter the field is 3D or not
	virtual bool is3D() const
	{
		return false;
	}

	// to be reimplemented by siblings
	virtual DBFFieldType type() const    = 0;
	virtual int          width() const   = 0;
	virtual int          decimal() const = 0;
	virtual bool         save(DBFHandle handle, int fieldIndex) const
	{
		return false;
	} // 1D version
	virtual bool save(DBFHandle handle, int xFieldIndex, int yFieldIndex, int zFieldIndex) const
	{
		return false;
	} // 3D version

  protected:
	//! Field name
	QString m_name;
};

//! Int-valued shapefile field
class QCC_IO_LIB_API IntegerDBFField : public GenericDBFField
{
  public:
	//! Default constructor
	explicit IntegerDBFField(QString name)
	    : GenericDBFField(name)
	{
	}

	// inherited from GenericDBFField
	DBFFieldType type() const override
	{
		return FTInteger;
	}
	int width() const override
	{
		return 6;
	}
	int decimal() const override
	{
		return 0;
	}
	bool save(DBFHandle handle, int fieldIndex) const override;

	//! Field values
	std::vector<int> values;
};

//! Double-valued shapefile field
class QCC_IO_LIB_API DoubleDBFField : public GenericDBFField
{
  public:
	//! Default constructor
	explicit DoubleDBFField(QString name)
	    : GenericDBFField(name)
	{
	}

	// inherited from GenericDBFField
	DBFFieldType type() const override
	{
		return FTDouble;
	}
	int width() const override
	{
		return 8;
	}
	int decimal() const override
	{
		return 8;
	}
	bool save(DBFHandle handle, int fieldIndex) const override;

	//! Field values
	std::vector<double> values;
};

//! Double-valued 3D shapefile field
class QCC_IO_LIB_API DoubleDBFField3D : public GenericDBFField
{
  public:
	//! Default constructor
	explicit DoubleDBFField3D(QString name)
	    : GenericDBFField(name)
	{
	}

	// inherited from GenericDBFField
	bool is3D() const override
	{
		return true;
	}
	DBFFieldType type() const override
	{
		return FTDouble;
	}
	int width() const override
	{
		return 8;
	}
	int decimal() const override
	{
		return 8;
	}
	bool save(DBFHandle handle, int xFieldIndex, int yFieldIndex, int zFieldIndex) const override;

	//! Field values
	std::vector<CCVector3d> values;
};

#endif // CC_SHP_SUPPORT
