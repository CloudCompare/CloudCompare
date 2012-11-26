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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//

#include <filtering.h>
#include <pcl/filters/statistical_outlier_removal.h>
int
removeOutliersStatistical(const sensor_msgs::PointCloud2ConstPtr incloud, const int &k, const float &nStds, sensor_msgs::PointCloud2Ptr outcloud)
{
    pcl::StatisticalOutlierRemoval<sensor_msgs::PointCloud2> remover;
    remover.setInputCloud(incloud);
    remover.setMeanK(k);
    remover.setStddevMulThresh(nStds);
    remover.filter(*outcloud);
    return 1;
}


void copyScalarFields(const ccPointCloud *inCloud, ccPointCloud *outCloud, pcl::PointIndicesPtr &in2outMapping, bool overwrite = true)
{
    int n_in = inCloud->size();
    int n_out = outCloud->size();
    assert(in2outMapping->indices.size() == outCloud->size());

    int n_scalars = inCloud->getNumberOfScalarFields();
    for (int i = 0; i < n_scalars; ++i)
      {
        CCLib::ScalarField* field = inCloud->getScalarField(i);
        const char* name = field->getName();

        //we need to verify no scalar field with the same name exists in the output cloud
        int id = outCloud->getScalarFieldIndexByName(name);
        ccScalarField * new_field = new ccScalarField;

        //resize the scalar field to the outcloud size
        new_field->reserve(outCloud->size());
        new_field->setName(name);

        if (id >= 0) //a scalar field with the same name exists
          {
           if (overwrite)
               outCloud->deleteScalarField(id);
           else
              break;
          }


        //now perform point to point copy
        for (unsigned int j = 0; j < outCloud->size(); ++j)
          {
            new_field->setValue(j, field->getValue(in2outMapping->indices.at(j)));
          }


        //recompute stats
        new_field->computeMinAndMax();
        ccScalarField * casted_field = static_cast<ccScalarField *> (new_field);
        casted_field->computeMinAndMax();



        //now put back the scalar field to the outCloud
        if (id < 0)
          outCloud->addScalarField(casted_field);


      }
}
