/*
 * ccPApi.cpp
 *
 *  Created on: 14 mars 2020
 *      Author: paul
 */

#include "ccPApi.h"

//qCC_db
#include <ccHObjectCaster.h>
//#include <ccColorScalesManager.h>

#include<AsciiFilter.h>

//system
#include <unordered_set>

//plugins
#include "ccPluginInterface.h"
//#include "ccPluginManager.h"

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
        //ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

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

