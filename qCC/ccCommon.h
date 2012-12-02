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
//$Rev:: 2241                                                              $
//$LastChangedDate:: 2012-09-21 23:22:39 +0200 (ven., 21 sept. 2012)       $
//**************************************************************************
//

#ifndef CC_COMMON_HEADER
#define CC_COMMON_HEADER

//STANDARD SCALAR FIELD NAMES
#define CC_CLOUD2CLOUD_DISTANCES_DEFAULT_SF_NAME "C2C distances"
#define CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME "Temp. Chamfer distances"
#define CC_TEMP_DISTANCES_DEFAULT_SF_NAME "Temp. distances"
#define CC_CLOUD2CLOUD_CHAMFER_DISTANCES_DEFAULT_SF_NAME "C2C Chamfer distances"
#define CC_CLOUD2MESH_DISTANCES_DEFAULT_SF_NAME "C2M Distances"
#define CC_CLOUD2MESH_SIGNED_DISTANCES_DEFAULT_SF_NAME "C2M Signed distances"
#define CC_CLOUD2MESH_CHAMFER_DISTANCES_DEFAULT_SF_NAME "C2M Chamfer distances"
#define CC_CHI2_DISTANCES_DEFAULT_SF_NAME "Chi2 distances"
#define CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME "CC labels"
#define CC_PCV_FIELD_LABEL_NAME "PCV"
#define CC_LOCAL_DENSITY_FIELD_NAME "Local density"
#define CC_MEAN_CURVATURE_FIELD_NAME "Mean curvature"
#define CC_ROUGHNESS_FIELD_NAME "Roughness"
#define CC_GAUSSIAN_CURVATURE_FIELD_NAME "Gaussian curvature"
#define CC_GRADIENT_NORMS_FIELD_NAME "Gradient norms"
#define CC_GEODESIC_DISTANCES_FIELD_NAME "Geodesic distances"
#define CC_HEIGHT_GRID_FIELD_NAME "Height grid values"
#define CC_SCAN_INTENSITY_FIELD_NAME "Intensity"
#define CC_SCAN_RETURN_INDEX_FIELD_NAME "Return index"
#define CC_LAS_CLASSIFICATION_FIELD_NAME "LAS classification"

//! Common parameters and other stuff
class ccCommon
{
    public:

        //! Returns current software version
        static const char* GetCCVersion();
};

#endif
