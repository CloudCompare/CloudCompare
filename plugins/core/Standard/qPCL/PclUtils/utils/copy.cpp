//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
//#ifdef LP_PCL_PATCH_ENABLED

#include "copy.h"

//qCC_db
#include <ccScalarField.h>
#include <ccPointCloud.h>

void copyScalarFields(const ccPointCloud *inCloud, ccPointCloud *outCloud, pcl::PointIndicesPtr &in2outMapping, bool overwrite)
{
	if (in2outMapping->indices.empty())
		return;
	assert(in2outMapping->indices.size() == outCloud->size());

	unsigned n_out = outCloud->size();

	unsigned sfCount = inCloud->getNumberOfScalarFields();
	for (unsigned i = 0; i < sfCount; ++i)
	{
		const CCLib::ScalarField* field = inCloud->getScalarField(i);
		const char* name = field->getName();

		ccScalarField* new_field = 0;

		//we need to verify no scalar field with the same name exists in the output cloud
		int id = outCloud->getScalarFieldIndexByName(name);
		if (id >= 0) //a scalar field with the same name exists
		{
			if (overwrite)
			{
				new_field = static_cast<ccScalarField*>(outCloud->getScalarField(id));
			}
			else
			{
				continue;
			}
		}
		else
		{
			new_field = new ccScalarField(name);

			//resize the scalar field to the outcloud size
			if (!new_field->resizeSafe(n_out))
			{
				//not enough memory!
				new_field->release();
				new_field = 0;
				continue;
			}
		}

		//now perform point to point copy
		for (unsigned j=0; j<n_out; ++j)
		{
			new_field->setValue(j, field->getValue(in2outMapping->indices.at(j)));
		}

		//recompute stats
		new_field->computeMinAndMax();

		//now put back the scalar field to the outCloud
		if (id < 0)
		{
			outCloud->addScalarField(new_field);
		}
	}

	outCloud->showSF(outCloud->sfShown() || inCloud->sfShown());
}

void copyRGBColors(const ccPointCloud *inCloud, ccPointCloud *outCloud, pcl::PointIndicesPtr &in2outMapping, bool overwrite)
{
	// if inCloud has no color there is nothing to do
	if (!inCloud->hasColors())
		return;

	if (in2outMapping->indices.empty())
		return;
	assert(in2outMapping->indices.size() == outCloud->size());

	if (outCloud->hasColors() && !overwrite)
		return;

	if (outCloud->reserveTheRGBTable())
	{
		//now perform point to point copy
		unsigned n_out = outCloud->size();
		for (unsigned j = 0; j < n_out; ++j)
		{
			outCloud->addColor(inCloud->getPointColor(in2outMapping->indices.at(j)));
		}
	}

	outCloud->showColors(outCloud->colorsShown() || inCloud->colorsShown());
}

//#endif // LP_PCL_PATCH_ENABLED
