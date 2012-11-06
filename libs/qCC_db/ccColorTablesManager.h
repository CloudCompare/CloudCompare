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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef COLOR_TABLES_MANAGER_HEADER
#define COLOR_TABLES_MANAGER_HEADER

//Local
#include "ccBasicTypes.h"

//! Pre-defined color ramps
enum CC_COLOR_RAMPS	{BGYR							=	0,					/**< Blue-Green-Yellow-Red ramp (default for distances display) */
						GREY						=	1,					/**< Grey ramp (default for Global Illumination) */
						BWR							=	2,					/**< Blue-White-Red ramp (for signed SF)*/
						RY							=	3,					/**< Red-Yellow ramp */
						RW							=	4,					/**< Red-White ramp */
};

const unsigned char COLOR_RAMPS_NUMBER												=		5;
const int DEFAULT_COLOR_RAMP_SIZE													=		1024;
const CC_COLOR_RAMPS COLOR_RAMPS_ENUMS[COLOR_RAMPS_NUMBER]							=		{BGYR, GREY, BWR, RY, RW};
const char COLOR_RAMPS_TITLES[COLOR_RAMPS_NUMBER][24]								=		{"Blue>Red","Grey","Blue>White>Red","Red>Yellow","White>Red"};
const CC_COLOR_RAMPS DEFAULT_COLOR_RAMP												=		BGYR;

//! Color tables factory
/** Colors in color tables are always in the format RGBA  (to improve access speed)
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccColorTablesManager
#else
class ccColorTablesManager
#endif
{
public:

    //! Returns unique instance
    static ccColorTablesManager* GetUniqueInstance();

    //! Releases unique instance
    static void ReleaseUniqueInstance();

	inline const colorType* getColorTable(CC_COLOR_RAMPS ct)
	{
	    return theColorRamps[ct];
    }

	inline const colorType* getColor(unsigned index, CC_COLOR_RAMPS ct)
    {
        return (index>=(unsigned)DEFAULT_COLOR_RAMP_SIZE ? 0 : theColorRamps[ct]+(index<<2));
    }

	inline const colorType* getColor(float normalizedIndex,CC_COLOR_RAMPS ct)
	{
        return theColorRamps[ct]+(((unsigned)(normalizedIndex*(float)(DEFAULT_COLOR_RAMP_SIZE-1)))<<2);
	}

    inline const colorType* getColor(float normalizedIndex, unsigned steps, CC_COLOR_RAMPS ct)
    {
        //old conversion
        //normalizedIndex = (float) (normalizedIndex < 1.0 ? floor(normalizedIndex * (float)steps) : steps-1) / (float)steps;
        //return theColorRamps[ct]+3*(unsigned)floor(normalizedIndex*(float)DEFAULT_COLOR_RAMP_SIZE);

        //new conversion
		//normalizedIndex = floor(normalizedIndex * (float)(steps-1)+0.5f);
        //return theColorRamps[ct]+((unsigned)(normalizedIndex*(float)(DEFAULT_COLOR_RAMP_SIZE-1)/(float)(steps-1))<<2);

		//quantized (16 bits) version --> much faster than floor!
		unsigned index = ((unsigned)((normalizedIndex*(float)(steps-1)+0.5f)*65536.0f))>>16;
		return theColorRamps[ct]+(((index*(DEFAULT_COLOR_RAMP_SIZE-1))/(steps-1))<<2);
    }

protected:

    //! Default constructor
	ccColorTablesManager();

    //! Default destructor
	~ccColorTablesManager();

    //! Creates a table corresponding to a given color ramp
    static colorType* CreateGradientColorTable(unsigned n, CC_COLOR_RAMPS cr);

    //! Color ramps
    colorType** theColorRamps;

};

#endif
