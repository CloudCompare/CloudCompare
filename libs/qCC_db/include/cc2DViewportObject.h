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

#ifndef CC_2D_VIEWPORT_OBJECT_HEADER
#define CC_2D_VIEWPORT_OBJECT_HEADER

// Local
#include "ccGenericGLDisplay.h"
#include "ccHObject.h"

//! 2D viewport object
class QCC_DB_LIB_API cc2DViewportObject : public ccHObject
{
  public:
	//! Default constructor
	cc2DViewportObject(QString name = QString());

	//! Copy constructor
	cc2DViewportObject(const cc2DViewportObject& viewport);

	// inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const override
	{
		return CC_TYPES::VIEWPORT_2D_OBJECT;
	}
	virtual bool isSerializable() const override
	{
		return true;
	}

	//! Sets perspective view state
	void setParameters(const ccViewportParameters& params)
	{
		m_params = params;
	}

	//! Gets parameters
	const ccViewportParameters& getParameters() const
	{
		return m_params;
	}

  protected:
	// inherited from ccHObject
	bool  toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool  fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;

	//! Viewport parameters
	ccViewportParameters m_params;
};

#endif // CC_2D_VIEWPORT_OBJECT_HEADER
