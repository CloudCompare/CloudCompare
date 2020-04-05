//##########################################################################
//#                                                                        #
//#                               CCPAPI                                   #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          Copyright 2020 Paul RASCLE www.openfields.fr                  #
//#                                                                        #
//##########################################################################

#include "ccPApi.h"

//CC
#include <GeometricalAnalysisTools.h>

//qCC_db
#include <ccHObjectCaster.h>
//#include <ccColorScalesManager.h>

//libs/qCC_io
#include<AsciiFilter.h>

//system
#include <unordered_set>

//plugins
#include "ccPluginInterface.h"
//#include "ccPluginManager.h"

//qCC
#include "ccCommon.h"

#define _CCDEBUG_
#include <ccTrace.h>

static ccPApi* ccPApiInternals=nullptr;

ccPApi* initCloudCompare()
{
    if (!ccPApiInternals)
    {
        CCTRACE("initCloudCompare");
        ccPApiInternals = new ccPApi;
        ccPApiInternals->m_silentMode = false;
        ccPApiInternals->m_autoSaveMode = true;
        ccPApiInternals->m_addTimestamp = true;
        ccPApiInternals->m_precision = 12;
        ccPApiInternals->m_coordinatesShiftWasEnabled = false;
        FileIOFilter::InitInternalFilters();  //load all known I/O filters (plugins will come later!)
        ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization

        //load the plugins
        //ccPluginManager::get().loadPlugins();
    }
    return ccPApiInternals;
}

ccPointCloud* loadPointCloud(const char *filename, CC_SHIFT_MODE mode,
                             int skip, double x, double y, double z)
{
    CCTRACE("Opening file: " << filename << " mode: " << mode << " skip: " << skip << " x: " << x << " y: " << y << " z: " << z);
    ccPApi* capi = initCloudCompare();
    CC_FILE_ERROR result = CC_FERR_NO_ERROR;
    ccHObject *db = nullptr;

    FileIOFilter::Shared filter = FileIOFilter::Shared(nullptr);
    QString fileName(filename);
    if (filter)
    {
        db = FileIOFilter::LoadFromFile(fileName, capi->m_loadingParameters, filter,
                result);
    }
    else
    {
        db = FileIOFilter::LoadFromFile(fileName, capi->m_loadingParameters, result,
                QString());
    }

    if (!db)
    {
        CCTRACE("LoadFromFile returns nullptr");
        return nullptr;
    }

    std::unordered_set<unsigned> verticesIDs;


    // look for the remaining clouds inside loaded DB
    ccHObject::Container clouds;
    db->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
    size_t count = clouds.size();
    CCTRACE("number of clouds: " << count);
    for (size_t i = 0; i < count; ++i)
    {
        ccPointCloud *pc = static_cast<ccPointCloud*>(clouds[i]);
        if (pc->getParent())
        {
            pc->getParent()->detachChild(pc);
        }

        //if the cloud is a set of vertices, we ignore it!
        if (verticesIDs.find(pc->getUniqueID()) != verticesIDs.end())
        {
            capi->m_orphans.addChild(pc);
            continue;
        }
        CCTRACE("Found one cloud with " << pc->size() << " points");
        capi->m_clouds.emplace_back(pc, filename,
                count == 1 ? -1 : static_cast<int>(i));
    }

    delete db;
    db = nullptr;

    if (count > 0)
        return capi->m_clouds.back().pc;
    else
        return nullptr;
}

CC_FILE_ERROR SavePointCloud(ccPointCloud* cloud,
                             const QString& filename)
{
    CCTRACE("saving cloud");
    ccPApi* capi = initCloudCompare();
    if (cloud == nullptr or filename.isEmpty())
        return CC_FERR_BAD_ARGUMENT;
    CCTRACE("cloud: " << cloud->getName().toStdString() << " file: " << filename.toStdString());
    FileIOFilter::SaveParameters parameters;
    parameters.alwaysDisplaySaveDialog = false;
    QFileInfo fi(filename);
    QString ext = fi.suffix();
    QString fileFilter = "";
    const std::vector<FileIOFilter::Shared>& filters = FileIOFilter::GetFilters();
    for(auto filter :filters)
    {
        QStringList theFilters = filter->getFileFilters(false);
        QStringList matches = theFilters.filter(ext);
        if (matches.size())
        {
            fileFilter = matches.first();
            break;
        }
    }
    CCTRACE("fileFilter: " << fileFilter.toStdString());
    CC_FILE_ERROR result  = FileIOFilter::SaveToFile(cloud,
                                                    filename,
                                                    parameters,
                                                    fileFilter); //AsciiFilter::GetFileFilter());
    return result;
}

bool computeCurvature(CurvatureType option, double radius, QList<ccPointCloud*> clouds)
{
    CCTRACE("computeCurvature mode: " << option << " radius: " << radius << " nbClouds: " << clouds.size());
    ccHObject::Container entities;
    entities.resize(clouds.size());
    for (int i = 0; i < clouds.size(); ++i)
    {
        CCTRACE("entity: "<< i << " name: " << clouds.at(i)->getName().toStdString());
        entities[i] = clouds.at(i);
    }
    ccPApiComputeGeomCharacteristic(CCLib::GeometricalAnalysisTools::Curvature, option, radius, entities);
}

bool ccPApiComputeGeomCharacteristic( CCLib::GeometricalAnalysisTools::GeomCharacteristic c,
                                      int subOption,
                                      PointCoordinateType radius,
                                      ccHObject::Container& entities)
{
    // --- from ccLibAlgorithms::ComputeGeomCharacteristic
    CCTRACE("ccPApiComputeGeomCharacteristic "<< subOption << " radius: " << radius);
    size_t selNum = entities.size();
    if (selNum < 1)
        return false;

    //generate the right SF name
    QString sfName;

    switch (c)
    {
    case CCLib::GeometricalAnalysisTools::Feature:
    {
        switch (subOption)
        {
        case CCLib::Neighbourhood::EigenValuesSum:
            sfName = "Eigenvalues sum";
            break;
        case CCLib::Neighbourhood::Omnivariance:
            sfName = "Omnivariance";
            break;
        case CCLib::Neighbourhood::EigenEntropy:
            sfName = "Eigenentropy";
            break;
        case CCLib::Neighbourhood::Anisotropy:
            sfName = "Anisotropy";
            break;
        case CCLib::Neighbourhood::Planarity:
            sfName = "Planarity";
            break;
        case CCLib::Neighbourhood::Linearity:
            sfName = "Linearity";
            break;
        case CCLib::Neighbourhood::PCA1:
            sfName = "PCA1";
            break;
        case CCLib::Neighbourhood::PCA2:
            sfName = "PCA2";
            break;
        case CCLib::Neighbourhood::SurfaceVariation:
            sfName = "Surface variation";
            break;
        case CCLib::Neighbourhood::Sphericity:
            sfName = "Sphericity";
            break;
        case CCLib::Neighbourhood::Verticality:
            sfName = "Verticality";
            break;
        case CCLib::Neighbourhood::EigenValue1:
            sfName = "1st eigenvalue";
            break;
        case CCLib::Neighbourhood::EigenValue2:
            sfName = "2nd eigenvalue";
            break;
        case CCLib::Neighbourhood::EigenValue3:
            sfName = "3rd eigenvalue";
            break;
        default:
            assert(false);
            ccLog::Error("Internal error: invalid sub option for Feature computation");
            return false;
        }

        sfName += QString(" (%1)").arg(radius);
    }
    break;

    case CCLib::GeometricalAnalysisTools::Curvature:
    {
        switch (subOption)
        {
        case CCLib::Neighbourhood::GAUSSIAN_CURV:
            sfName = CC_CURVATURE_GAUSSIAN_FIELD_NAME;
            break;
        case CCLib::Neighbourhood::MEAN_CURV:
            sfName = CC_CURVATURE_MEAN_FIELD_NAME;
            break;
        case CCLib::Neighbourhood::NORMAL_CHANGE_RATE:
            sfName = CC_CURVATURE_NORM_CHANGE_RATE_FIELD_NAME;
            break;
        default:
            assert(false);
            ccLog::Error("Internal error: invalid sub option for Curvature computation");
            return false;
        }
        sfName += QString(" (%1)").arg(radius);
    }
    break;

    case CCLib::GeometricalAnalysisTools::LocalDensity:
        sfName = ccPApiGetDensitySFName(static_cast<CCLib::GeometricalAnalysisTools::Density>(subOption), false, radius);
        break;

    case CCLib::GeometricalAnalysisTools::ApproxLocalDensity:
        sfName = ccPApiGetDensitySFName(static_cast<CCLib::GeometricalAnalysisTools::Density>(subOption), true);
        break;

    case CCLib::GeometricalAnalysisTools::Roughness:
        sfName = CC_ROUGHNESS_FIELD_NAME + QString(" (%1)").arg(radius);
        break;

    case CCLib::GeometricalAnalysisTools::MomentOrder1:
        sfName = CC_MOMENT_ORDER1_FIELD_NAME + QString(" (%1)").arg(radius);
        break;

    default:
        assert(false);
        return false;
    }

    for (size_t i = 0; i < selNum; ++i)
    {
        //is the ith selected data is eligible for processing?
        if (entities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
        {
            ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entities[i]);

            ccPointCloud* pc = 0;
            int sfIdx = -1;
            if (cloud->isA(CC_TYPES::POINT_CLOUD))
            {
                pc = static_cast<ccPointCloud*>(cloud);

                sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
                if (sfIdx < 0)
                    sfIdx = pc->addScalarField(qPrintable(sfName));
                if (sfIdx >= 0)
                    pc->setCurrentScalarField(sfIdx);
                else
                {
                    continue;
                }
            }

            ccOctree::Shared octree = cloud->getOctree();
            if (!octree)
            {
                octree = cloud->computeOctree(nullptr);
                if (!octree)
                {
                    break;
                }
            }

            CCLib::GeometricalAnalysisTools::ErrorCode result = CCLib::GeometricalAnalysisTools::ComputeCharactersitic(c, subOption, cloud, radius, nullptr, octree.data());

            if (result == CCLib::GeometricalAnalysisTools::NoError)
            {
                if (pc && sfIdx >= 0)
                {
                    pc->setCurrentDisplayedScalarField(sfIdx);
                    pc->showSF(sfIdx >= 0);
                    pc->getCurrentInScalarField()->computeMinAndMax();
                }
                cloud->prepareDisplayForRefresh();
            }
            else
            {
                QString errorMessage;
                switch (result)
                {
                case CCLib::GeometricalAnalysisTools::InvalidInput:
                    errorMessage = "Internal error (invalid input)";
                    break;
                case CCLib::GeometricalAnalysisTools::NotEnoughPoints:
                    errorMessage = "Not enough points";
                    break;
                case CCLib::GeometricalAnalysisTools::OctreeComputationFailed:
                    errorMessage = "Failed to compute octree (not enough memory?)";
                    break;
                case CCLib::GeometricalAnalysisTools::ProcessFailed:
                    errorMessage = "Process failed";
                    break;
                case CCLib::GeometricalAnalysisTools::UnhandledCharacteristic:
                    errorMessage = "Internal error (unhandled characteristic)";
                    break;
                case CCLib::GeometricalAnalysisTools::NotEnoughMemory:
                    errorMessage = "Not enough memory";
                    break;
                case CCLib::GeometricalAnalysisTools::ProcessCancelledByUser:
                    errorMessage = "Process cancelled by user";
                    break;
                default:
                    assert(false);
                    errorMessage = "Unknown error";
                    break;
                }

                if (pc && sfIdx >= 0)
                {
                    pc->deleteScalarField(sfIdx);
                    sfIdx = -1;
                }

                return false;
            }
        }
    }

    return true;
}

QString ccPApiGetDensitySFName(CCLib::GeometricalAnalysisTools::Density densityType, bool approx, double densityKernelSize)
{
    // --- from ccLibAlgorithms::GetDensitySFName
    CCTRACE("ccPApiGetDensitySFName");
    QString sfName;

    //update the name with the density type
    switch (densityType)
    {
        case CCLib::GeometricalAnalysisTools::DENSITY_KNN:
            sfName = CC_LOCAL_KNN_DENSITY_FIELD_NAME;
            break;
        case CCLib::GeometricalAnalysisTools::DENSITY_2D:
            sfName = CC_LOCAL_SURF_DENSITY_FIELD_NAME;
            break;
        case CCLib::GeometricalAnalysisTools::DENSITY_3D:
            sfName = CC_LOCAL_VOL_DENSITY_FIELD_NAME;
            break;
        default:
            assert(false);
            break;
    }

    sfName += QString(" (r=%2)").arg(densityKernelSize);

    if (approx)
        sfName += " [approx]";

    return sfName;
}


