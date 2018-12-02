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

#ifndef CC_COMMON_HEADER
#define CC_COMMON_HEADER

//STANDARD SCALAR FIELD NAMES
#define CC_DEFAULT_SF_NAME "Unknown"
#define CC_CLOUD2CLOUD_DISTANCES_DEFAULT_SF_NAME "C2C absolute distances"
#define CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME "Approx. distances"
#define CC_TEMP_DISTANCES_DEFAULT_SF_NAME "Temp. approx. distances"
#define CC_CLOUD2CLOUD_APPROX_DISTANCES_DEFAULT_SF_NAME "C2C approx. distances"
#define CC_CLOUD2MESH_DISTANCES_DEFAULT_SF_NAME "C2M absolute distances"
#define CC_CLOUD2MESH_SIGNED_DISTANCES_DEFAULT_SF_NAME "C2M signed distances"
#define CC_CLOUD2MESH_APPROX_DISTANCES_DEFAULT_SF_NAME "C2M approx. distances"
#define CC_CHI2_DISTANCES_DEFAULT_SF_NAME "Chi2 distances"
#define CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME "CC labels"
#define CC_LOCAL_KNN_DENSITY_FIELD_NAME "Number of neighbors"
#define CC_LOCAL_SURF_DENSITY_FIELD_NAME "Surface density"
#define CC_LOCAL_VOL_DENSITY_FIELD_NAME "Volume density"
#define CC_ROUGHNESS_FIELD_NAME "Roughness"
#define CC_MOMENT_ORDER1_FIELD_NAME "1st order moment"
#define CC_CURVATURE_GAUSSIAN_FIELD_NAME "Gaussian curvature"
#define CC_CURVATURE_MEAN_FIELD_NAME "Mean curvature"
#define CC_CURVATURE_NORM_CHANGE_RATE_FIELD_NAME "Normal change rate"
#define CC_GRADIENT_NORMS_FIELD_NAME "Gradient norms"
#define CC_GEODESIC_DISTANCES_FIELD_NAME "Geodesic distances"
#define CC_DEFAULT_RAD_SCATTERING_ANGLES_SF_NAME "Scattering angles (rad)"
#define CC_DEFAULT_DEG_SCATTERING_ANGLES_SF_NAME "Scattering angles (deg)"
#define CC_DEFAULT_RANGES_SF_NAME "Ranges"
#define CC_DEFAULT_SQUARED_RANGES_SF_NAME "Ranges (squared)"
#define CC_DEFAULT_DIP_SF_NAME "Dip (degrees)"
#define CC_DEFAULT_DIP_DIR_SF_NAME "Dip direction (degrees)"
#define CC_DEFAULT_MESH_VERT_FLAGS_SF_NAME "Vertex type"
#define CC_DEFAULT_ID_SF_NAME "Id"
#define CC_ORIGINAL_CLOUD_INDEX_SF_NAME "Original cloud index"

#endif
