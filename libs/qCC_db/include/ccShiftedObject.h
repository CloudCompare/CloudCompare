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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Local
#include "ccHObject.h"


//! Shifted entity interface
/** Shifted entities are entities which coordinates can be
	(optionally) shifted so as to reduce their amplitude and
	therefore display or accuracy issues.
**/
class QCC_DB_LIB_API ccShiftedObject : public ccHObject
{
public:

	//! Default constructor
	/** \param name cloud name (optional)
		\param uniqueID unique ID (handle with care)
	**/
	ccShiftedObject(QString name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Copy constructor
	/** \param s shifted object to copy
	**/
	ccShiftedObject(const ccShiftedObject& s) = default;

	//! Copies the Global Shift and Scale from another entity
	/** \param s shifted entity to copy information from
	**/
	void copyGlobalShiftAndScale(const ccShiftedObject& s);

	//! Sets shift applied to original coordinates (information storage only)
	/** Such a shift can typically be applied at loading time.
	**/
	virtual inline void setGlobalShift(double x, double y, double z) { return setGlobalShift(CCVector3d(x, y, z)); }

	//! Sets shift applied to original coordinates (information storage only)
	/** Such a shift can typically be applied at loading time.
		Original coordinates are equal to '(P/scale - shift)'
	**/
	virtual void setGlobalShift(const CCVector3d& shift);

	//! Returns the shift applied to original coordinates
	/** See ccGenericPointCloud::setOriginalShift
	**/
	virtual const CCVector3d& getGlobalShift() const { return m_globalShift; }

	//! Sets the scale applied to original coordinates (information storage only)
	virtual void setGlobalScale(double scale);

	//! Returns the scale applied to original coordinates
	virtual double getGlobalScale() const { return m_globalScale; }

	//! Returns whether the cloud is shifted or not
	inline bool isShifted() const
	{
		const CCVector3d& globalShift = getGlobalShift();
		return (	globalShift.x != 0
				||	globalShift.y != 0
				||	globalShift.z != 0
				||	getGlobalScale() != 1.0 );
	}

	//! Returns the point back-projected into the original coordinates system
	template<typename T> inline CCVector3d toGlobal3d(const Vector3Tpl<T>& Plocal) const
	{
		// Pglobal = Plocal/scale - shift
		return CCVector3d::fromArray(Plocal.u) / getGlobalScale() - getGlobalShift();
	}

	//! Returns the point projected into the local (shifted) coordinates system
	template<typename T> inline CCVector3d toLocal3d(const Vector3Tpl<T>& Pglobal) const
	{
		// Plocal = (Pglobal + shift) * scale
		return (CCVector3d::fromArray(Pglobal.u) + getGlobalShift()) * getGlobalScale();
	}
	//! Returns the point projected into the local (shifted) coordinates system
	template<typename T> inline CCVector3 toLocal3pc(const Vector3Tpl<T>& Pglobal) const
	{
		CCVector3d Plocal = CCVector3d::fromArray(Pglobal.u) * getGlobalScale() + getGlobalShift();
		return CCVector3::fromArray(Plocal.u);
	}

	//inherited from ccHObject
	bool getGlobalBB(CCVector3d& minCorner, CCVector3d& maxCorner) override;

protected:

	//! Serialization helper (output)
	bool saveShiftInfoToFile(QFile& out) const;
	//! Serialization helper (input)
	bool loadShiftInfoFromFile(QFile& in);

	//! Global shift (typically applied at loading time)
	CCVector3d m_globalShift;

	//! Global scale (typically applied at loading time)
	double m_globalScale;

};
