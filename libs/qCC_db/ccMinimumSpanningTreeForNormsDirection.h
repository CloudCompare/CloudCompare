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

#ifndef CC_MST_FOR_NORMS_DIRECTION_HEADER
#define CC_MST_FOR_NORMS_DIRECTION_HEADER

class ccPointCloud;
class ccProgressDialog;

//! Minimum Spanning Tree for normals direction resolution
/** See http://people.maths.ox.ac.uk/wendland/research/old/reconhtml/node3.html
**/
class ccMinimumSpanningTreeForNormsDirection
{
public:

	//! Main entry point
	static bool OrientNormals(	ccPointCloud* cloud,
								unsigned kNN = 6,
								ccProgressDialog* progressDlg = 0);
};

#endif //CC_MST_FOR_NORMS_DIRECTION_HEADER
