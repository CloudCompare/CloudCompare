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

#ifndef CC_COLOR_SCALES_MANAGER_HEADER
#define CC_COLOR_SCALES_MANAGER_HEADER

//Local
#include "ccColorScale.h"

//Qt
#include <QMap>

//! Color scales manager/container
class QCC_DB_LIB_API ccColorScalesManager
{
public:

	//! Returns unique instance
	static ccColorScalesManager* GetUniqueInstance();

	//! Releases unique instance
	static void ReleaseUniqueInstance();

	//! Destructor
	virtual ~ccColorScalesManager();

	//! Pre-defined color scales (all relative - i.e. expand to actual SF)
	enum DEFAULT_SCALES	{	BGYR			=	0,		/**< Blue-Green-Yellow-Red ramp (default for distances display) */
							GREY			=	1,		/**< Grey ramp (default for Global Illumination) */
							BWR				=	2,		/**< Blue-White-Red ramp (for signed SF)*/
							RY				=	3,		/**< Red-Yellow ramp */
							RW				=	4,		/**< Red-White ramp */
							ABS_NORM_GREY	=	5,		/**< Absolute normalized grey ramp (intensities between 0 and 1) */
							HSV_360_DEG		=	6,		/**< HSV colors between 0 and 360 degrees */
							VERTEX_QUALITY	=	7,		/**< Mesh vertex quality (see CCLib::MeshSamplingTools::VertexFlags) */
							DIP_BRYW		=	8,		/**< Dip (0 - 90 degrees) (Brown-Red-Yellow-White) */
							DIP_DIR_REPEAT	=	9,		/**< Dip direction (0 - 360 degrees) */
							VIRIDIS			=	10,		/**< matplotlib library colorscale created by Stéfan van der Walt and Nathaniel Smith */
							BROWN_YELLOW	=	11,		/**< Brown-Yellow */
							YELLOW_BROWN	=	12,		/**< Yellow-Brown */
							TOPO_LANDSERF	=	13,		/**< Topo Landserf (quartile) */
							HIGH_CONTRAST	=	14		/**< High constrast */
	};

	//! Returns a pre-defined color scale UUID
	static QString GetDefaultScaleUUID(int scale) { return QString::number(scale); }

	//! Returns a pre-defined color scale (static shortcut)
	static ccColorScale::Shared GetDefaultScale(DEFAULT_SCALES scale = BGYR)
	{
		ccColorScalesManager* instance = GetUniqueInstance();
		return instance ? instance->getDefaultScale(scale) : ccColorScale::Shared(0);
	}

	//! Returns a pre-defined color scale
	ccColorScale::Shared getDefaultScale(DEFAULT_SCALES scale) const { return getScale(GetDefaultScaleUUID(scale)); }

	//! Returns a color scale based on its UUID
	ccColorScale::Shared getScale(QString UUID) const;

	//! Adds a new color scale
	void addScale(ccColorScale::Shared scale);

	//! Removes a color scale
	/** Warning: can't remove default scales!
	**/
	void removeScale(QString UUID);

	//! Color scales map type
	typedef QMap< QString, ccColorScale::Shared > ScalesMap;

	//! Access to the internal map
	ScalesMap& map() { return m_scales; }

	//! Access to the internal map (const)
	const ScalesMap& map() const { return m_scales; }

	//! Loads custom color scales from persistent settings
	void fromPersistentSettings();

	//! Save custom color scales to persistent settings
	void toPersistentSettings() const;

protected:

	//! Default constructor
	ccColorScalesManager();

	//! Creates a pre-defined color scale
	static ccColorScale::Shared Create(DEFAULT_SCALES scaleType);

	//! Color scales
	ScalesMap m_scales;

};

#endif //CC_COLOR_SCALES_MANAGER_HEADER
