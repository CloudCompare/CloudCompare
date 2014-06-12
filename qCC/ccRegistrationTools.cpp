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

#include "ccRegistrationTools.h"

//Local
#include "ccRegistrationDlg.h"

//CCLib
#include <MeshSamplingTools.h>
#include <GenericIndexedCloudPersist.h>
#include <SimpleCloud.h>
#include <RegistrationTools.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccLog.h>

//! Default number of points sampled on the 'model' mesh (if any)
static const unsigned s_defaultSampledPointsOnModelMesh = 100000;
//! Default number of points sampled on the 'data' mesh (if any)
static const unsigned s_defaultSampledPointsOnDataMesh = 50000;
//! Default temporary registration scalar field
static const char REGISTRATION_DISTS_SF[] = "RegistrationDistances";

bool ccRegistrationTools::ICP(	ccHObject* data,
								ccHObject* model,
								ccGLMatrix& transMat,
								double &finalScale,
								double& finalError,
								double minErrorDecrease,
								unsigned maxIterationCount,
								unsigned randomSamplingLimit,
								bool removeFarthestPoints,
								CCLib::ICPRegistrationTools::CONVERGENCE_TYPE method,
								bool adjustScale,
								bool useDataSFAsWeights/*=false*/,
								bool useModelSFAsWeights/*=false*/,
								int filters/*=CCLib::ICPRegistrationTools::SKIP_NONE*/,
								QWidget* parent/*=0*/)
{
	//progress bar
	ccProgressDialog pDlg(false,parent);

	//if the 'model' entity is a mesh, we need to sample points on it
	CCLib::GenericIndexedCloudPersist* modelCloud = 0;
	if (model->isKindOf(CC_TYPES::MESH))
	{
		modelCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(model),s_defaultSampledPointsOnModelMesh,&pDlg);
		if (!modelCloud)
		{
			ccLog::Error("[ICP] Failed to sample points on 'model' mesh!");
			return false;
		}
	}
	else
	{
		modelCloud = ccHObjectCaster::ToGenericPointCloud(model);
	}

	//if the 'data' entity is a mesh, we need to sample points on it
	CCLib::GenericIndexedCloudPersist* dataCloud = 0;
	if (data->isKindOf(CC_TYPES::MESH))
	{
		dataCloud = CCLib::MeshSamplingTools::samplePointsOnMesh(ccHObjectCaster::ToGenericMesh(data),s_defaultSampledPointsOnDataMesh,&pDlg);
		if (!dataCloud)
		{
			ccLog::Error("[ICP] Failed to sample points on 'data' mesh!");
			return false;
		}
	}
	else
	{
		dataCloud = ccHObjectCaster::ToGenericPointCloud(data);
	}

	//we activate a temporary scalar field for registration distances computation
	CCLib::ScalarField* dataDisplayedSF = 0;
	int oldDataSfIdx=-1, dataSfIdx=-1;

	//if the 'data' entity is a real ccPointCloud, we can even create a temporary SF for registration distances
	if (data->isA(CC_TYPES::POINT_CLOUD))
	{
		ccPointCloud* pc = static_cast<ccPointCloud*>(data);
		dataDisplayedSF = pc->getCurrentDisplayedScalarField();
		oldDataSfIdx = pc->getCurrentInScalarFieldIndex();
		dataSfIdx = pc->getScalarFieldIndexByName(REGISTRATION_DISTS_SF);
		if (dataSfIdx < 0)
			dataSfIdx = pc->addScalarField(REGISTRATION_DISTS_SF);
		if (dataSfIdx >= 0)
			pc->setCurrentScalarField(dataSfIdx);
		else
			ccLog::Warning("[ICP] Couldn't create temporary scalar field! Not enough memory?");
	}
	else
	{
		dataCloud->enableScalarField();
	}

	//parameters
	CCLib::PointProjectionTools::Transformation transform;

	CCLib::ScalarField* modelWeights = 0;
	if (useModelSFAsWeights)
	{
		if (modelCloud == dynamic_cast<CCLib::GenericIndexedCloudPersist*>(model) && model->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(model);
			modelWeights = pc->getCurrentDisplayedScalarField();
			if (!modelWeights)
				ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but model has no displayed scalar field!");
		}
		else
		{
			ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but only point clouds scalar fields can be used as weights!");
		}
	}

	CCLib::ScalarField* dataWeights = 0;
	if (useDataSFAsWeights)
	{
		if (!dataDisplayedSF)
		{
			if (dataCloud == (CCLib::GenericIndexedCloudPersist*)data && data->isA(CC_TYPES::POINT_CLOUD))
				ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but data has no displayed scalar field!");
			else
				ccLog::Warning("[ICP] 'useDataSFAsWeights' is true but inly point clouds scalar fields can be used as weights!");
		}
		else
			dataWeights = dataDisplayedSF;

	}

	CCLib::ICPRegistrationTools::RESULT_TYPE result;

	result = CCLib::ICPRegistrationTools::RegisterClouds(	modelCloud,
															dataCloud,
															transform,
															method,
															minErrorDecrease,
															maxIterationCount,
															finalError,
															adjustScale,
															static_cast<CCLib::GenericProgressCallback*>(&pDlg),
															removeFarthestPoints,
															randomSamplingLimit,
															modelWeights,
															dataWeights,
															filters);

	if (result >= CCLib::ICPRegistrationTools::ICP_ERROR)
	{
		ccLog::Error("Registration failed: an error occurred (code %i)",result);
	}
	else if (result == CCLib::ICPRegistrationTools::ICP_APPLY_TRANSFO)
	{
		transMat = FromCCLibMatrix<PointCoordinateType,float>(transform.R, transform.T, transform.s);
		finalScale = transform.s;
	}

	//if we had to sample points an the data mesh
	if (!data->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		delete dataCloud;
		dataCloud = 0;
	}
	else
	{
		if (data->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(data);
			pc->setCurrentScalarField(oldDataSfIdx);
			if (dataSfIdx >= 0)
				pc->deleteScalarField(dataSfIdx);
			dataSfIdx=-1;
		}
	}

	//if we had to sample points an the model mesh
	if (!model->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		delete modelCloud;
		modelCloud = 0;
	}

	return (result < CCLib::ICPRegistrationTools::ICP_ERROR);
}
