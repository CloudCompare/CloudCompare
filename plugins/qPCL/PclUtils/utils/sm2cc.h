//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################

#ifndef Q_PCL_PLUGIN_SM2CC_H
#define Q_PCL_PLUGIN_SM2CC_H

#include "pcl_utilities.h"

//system
#include <list>

class ccPointCloud;

//! PCL to CC cloud converter
/** NOTE: THIS METHOD HAVE SOME PROBLEMS. IT CANNOT CORRECTLY LOAD NOT-FLOAT FIELDS!
	THIS IS DUE TO THE FACT THE POINT TYPE WITH A SCALAR WE USE HERE IS FLOAT
	IF YOU TRY TO LOAD A FIELD THAT IS INT YOU GET A PCL WARN!
**/
class sm2ccConverter
{
public:

	//! Default constructor
	sm2ccConverter(PCLCloud::Ptr sm_cloud);

	//! Converts inut cloud (see constructor) to a ccPointCloud
	ccPointCloud* getCloud();

	bool addXYZ(ccPointCloud *cloud);
	bool addNormals(ccPointCloud *cloud);
	bool addRGB(ccPointCloud * cloud);
	bool addScalarField(ccPointCloud * cloud, const std::string& name, bool overwrite_if_exist = true);

private:

	size_t getNumberOfPoints();

	bool existField(std::string name) const;

	void eraseString(std::list<std::string> &fields, std::string name);

	//! Associated PCL cloud
	PCLCloud::Ptr m_sm_cloud;

};

//! Convert any given PCLCloud to ccPointCloud
/** \param[in] sm_cloud PCL cloud
	\return a pointer to a ccPointCloud
	\note Not all the fields are read, only the followings:
	- x, y, and z euclidean coordinates
	- intensity (as scalar field)
	- nx, ny, and nz normals
	- rgb colors
	\todo Conversion capabilities should be extended to:
	- curvature
	- ...
**/
ccPointCloud* sensorToCC(const PCLCloud &sm_cloud);

//! Returns the list of fields contained in a PCL cloud
/** \param[in] cloud
	\return a std::vector of std::strings
**/
std::vector<std::string> getFieldList(const PCLCloud &cloud);

//! Checks if a field exists in a PCL cloud
/** \param[in] cloud the PCL cloud
	\param[in] field_name the name of the field to be checked
	\return true if the field exists
**/
bool checkField(const PCLCloud &cloud, std::string field_name);

#endif // Q_PCL_PLUGIN_SM2CC_H
