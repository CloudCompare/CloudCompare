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

#ifndef GUI_PARAMETERS_HEADER
#define GUI_PARAMETERS_HEADER

/***************************************************
				GUI parameters
***************************************************/

//! This class manages the persistent parameters
/** Values of persistent parameters are stored by the system
	from one execution of CloudCompare to the next.
**/
class ccGui
{
public:

    //! GUI parameters
    struct ParamStruct
    {
        //! Light diffuse color (RGBA)
        float lightDiffuseColor[4];
        //! Light ambient color (RGBA)
        float lightAmbientColor[4];
        //! Light specular color (RGBA)
        float lightSpecularColor[4];

        //! Default mesh diffuse color (front)
        float meshFrontDiff[4];
        //! Default mesh diffuse color (back)
        float meshBackDiff[4];
        //! Default mesh specular color
        float meshSpecular[4];

        //! Default text color
        unsigned char textDefaultCol[3];
        //! Default 3D points color
        unsigned char pointsDefaultCol[3];
        //! Background color
        unsigned char backgroundCol[3];
        //! Bounding-boxes color
        unsigned char bbDefaultCol[3];
        //! Use background gradient
        bool drawBackgroundGradient;
        //! Decimate meshes when moved
        bool decimateMeshOnMove;
        //! Decimate clouds when moved
        bool decimateCloudOnMove;
        //! Display cross in the middle of the screen
        bool displayCross;

		//! Picked points size
		unsigned pickedPointsSize;
		//! Picked points start index (for label display)
		unsigned pickedPointsStartIndex;

		//! Color scale option: always show '0'
		bool colorScaleAlwaysShowZero;
		//! Color scale option: always symmetrical
		/** This only applies to signed scalar fields.
		**/
		bool colorScaleAlwaysSymmetrical;
		//! Color scale square size
		unsigned colorScaleSquareSize;
		
		//! Default displayed font size
		unsigned defaultFontSize;
		//! Displayed numbers precision
		unsigned displayedNumPrecision;
		//! Labels transparency
		unsigned labelsTransparency;

        //! Default constructor
        ParamStruct();

        //! Copy operator
        ParamStruct& operator =(const ParamStruct& params);

        //! Resets parameters to default values
        void reset();

        //! Loads from persistent DB
        void fromPersistentSettings();

        //! Saves to persistent DB
        void toPersistentSettings();
    };

	//! Returns the stored values of each parameter.
	static const ParamStruct& Parameters();

    //! Sets GUI parameters
	static void Set(const ParamStruct& params);

    //! Release unique instance (if any)
    static void ReleaseInstance();

protected:

    //! Parameters set
    ParamStruct params;

};

 #endif
