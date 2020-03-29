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

ccPApi::ccPApi() :
        m_silentMode(false), m_autoSaveMode(true), m_addTimestamp(true), m_precision(
                12), m_coordinatesShiftWasEnabled(false)
{
    //global structures initialization
    FileIOFilter::InitInternalFilters(); //load all known I/O filters (plugins will come later!)
    ccNormalVectors::GetUniqueInstance(); //force pre-computed normals array initialization
    //ccColorScalesManager::GetUniqueInstance(); //force pre-computed color tables initialization

    //load the plugins
    //ccPluginManager::get().loadPlugins();
}

ccPApi::ccPApi(const ccPApi &)
{
    // Private: Avoid automatic copy constructor. Prevent instances from being copied.
}

ccPApi &ccPApi::operator=(const ccPApi &)
{
    // Private: Avoid automatic copy constructor. Prevent instances from being copied.
}

ccPApi::~ccPApi()
{

}

ccPointCloud* ccPApi::loadPointCloud(const char *filename, CC_SHIFT_MODE mode,
        int skip, double x, double y, double z)
{
    CCTRACE("Opening file: " << filename << " mode: " << mode << " skip: " << skip << " x: " << x << " y: " << y << " z: " << z);

    CC_FILE_ERROR result = CC_FERR_NO_ERROR;
    ccHObject *db = nullptr;

    FileIOFilter::Shared filter = FileIOFilter::Shared(nullptr);
    QString fileName(filename);
    if (filter)
    {
        db = FileIOFilter::LoadFromFile(fileName, m_loadingParameters, filter,
                result);
    }
    else
    {
        db = FileIOFilter::LoadFromFile(fileName, m_loadingParameters, result,
                QString());
    }

    if (!db)
    {
        CCTRACE("LoadFromFile returns nullptr");
        return nullptr;
    }

    std::unordered_set<unsigned> verticesIDs;

//	//first look for meshes inside loaded DB (so that we don't consider mesh vertices as clouds!)
//	{
//		ccHObject::Container meshes;
//		size_t count = 0;
//		//first look for all REAL meshes (so as to no consider sub-meshes)
//		if (db->filterChildren(meshes, true, CC_TYPES::MESH, true) != 0)
//		{
//			count += meshes.size();
//			for (size_t i = 0; i < meshes.size(); ++i)
//			{
//				ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(meshes[i]);
//				if (mesh->getParent())
//				{
//					mesh->getParent()->detachChild(mesh);
//				}
//
//				ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
//				if (vertices)
//				{
//					verticesIDs.insert(vertices->getUniqueID());
//					CCTRACE("Found one mesh with " << mesh->size() << " faces and " << mesh->getAssociatedCloud()->size() <<" vertices: " << mesh->getName());
//					m_meshes.emplace_back(mesh, filename, count == 1 ? -1 : static_cast<int>(i));
//				}
//				else
//				{
//					delete mesh;
//					mesh = nullptr;
//					assert(false);
//				}
//			}
//		}

//		//then look for the other meshes
//		meshes.clear();
//		if (db->filterChildren(meshes, true, CC_TYPES::MESH, false) != 0)
//		{
//			size_t countBefore = count;
//			count += meshes.size();
//			for (size_t i = 0; i < meshes.size(); ++i)
//			{
//				ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(meshes[i]);
//				if (mesh->getParent())
//					mesh->getParent()->detachChild(mesh);
//
//				ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
//				if (vertices)
//				{
//					verticesIDs.insert(vertices->getUniqueID());
//					CCTRACE("Found one kind of mesh with " << mesh->size() << " faces and " << mesh->getAssociatedCloud()->size() <<" vertices: " << mesh->getName());
//					m_meshes.emplace_back(mesh, filename, count == 1 ? -1 : static_cast<int>(countBefore + i));
//				}
//				else
//				{
//					delete mesh;
//					mesh = nullptr;
//					assert(false);
//				}
//			}
//		}
//	}

//now look for the remaining clouds inside loaded DB
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
            m_orphans.addChild(pc);
            continue;
        }
        CCTRACE("Found one cloud with " << pc->size() << " points");
        m_clouds.emplace_back(pc, filename,
                count == 1 ? -1 : static_cast<int>(i));
    }

    delete db;
    db = nullptr;

    if (count > 0)
        return m_clouds.back().pc;
    else
        return nullptr;
}

CC_FILE_ERROR ccPApi::SavePointCloud(ccPointCloud* cloud,
                                     const QString& filename)
{
    CCTRACE("saving cloud");
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

