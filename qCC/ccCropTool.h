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

#ifndef CC_CROP_TOOL_HEADER
#define CC_CROP_TOOL_HEADER

//qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccLog.h>

//CCLib
#include <ManualSegmentationTools.h>
#include <SimpleMesh.h>

//! Cropping tool
/** Handles clouds and meshes for now
**/
class ccCropTool
{
public:
	//! Crops the input entity
	/** \param entity entity to be cropped (should be a cloud or a mesh)
		\param box cropping box
		\param inside whether to keep the points/triangles inside or outside the input box
		\return cropped entity (if any)
	**/
	static ccHObject* Crop(ccHObject* entity, const ccBBox& box, bool inside = true)
	{
		assert(entity);
		if (!entity)
			return 0;

		if (entity->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);

			CCLib::ReferenceCloud* selection = cloud->crop(box, inside);
			if (!selection)
			{
				//process failed!
				ccLog::Warning(QString("[Crop] Failed to crop cloud '%1'!").arg(cloud->getName()));
				return 0;
			}

			if (selection->size() == 0)
			{
				//no points fall inside selection!
				ccLog::Warning(QString("[Crop] No point of the cloud '%1' falls %2side the input box!").arg(cloud->getName()).arg(inside ? "in" : "out"));
				delete selection;
				return 0;
			}

			//crop
			ccPointCloud* croppedEnt = cloud->partialClone(selection);
			delete selection;

			return croppedEnt;
		}
		else if (entity->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(entity);
			CCLib::ManualSegmentationTools::MeshCutterParams params;
			params.bbMin = CCVector3d::fromArray(box.minCorner().u);
			params.bbMax = CCVector3d::fromArray(box.maxCorner().u);
			params.generateOutsideMesh = !inside;

			if (!CCLib::ManualSegmentationTools::segmentMeshWitAABox(mesh, mesh->getAssociatedCloud(), params))
			{
				//process failed!
				ccLog::Warning(QString("[Crop] Failed to crop mesh '%1'!").arg(mesh->getName()));
			}

			//output
			ccMesh* croppedMesh = 0;

			CCLib::SimpleMesh* tempMesh = inside ? params.insideMesh : params.outsideMesh;
			if (tempMesh)
			{
				ccPointCloud* vertices = ccPointCloud::From(tempMesh->vertices());
				if (vertices)
				{
					ccMesh* croppedMesh = new ccMesh(tempMesh, vertices);
					croppedMesh->addChild(vertices);
					vertices->setEnabled(false);
					if (croppedMesh->size() == 0)
					{
						//no points fall inside selection!
						ccLog::Warning(QString("[Crop] No trinagle of the mesh '%1' falls %2side the input box!").arg(mesh->getName()).arg(inside ? "in" : "out"));
						delete croppedMesh;
						croppedMesh = 0;
					}
				}
				else
				{
					ccLog::Warning("[Crop] Failed to create output mesh vertices (not enough memory)");
				}
			}

			//clean memory
			if (params.insideMesh)
			{
				delete params.insideMesh;
				params.insideMesh = 0;
			}
			if (params.outsideMesh)
			{
				delete params.outsideMesh;
				params.outsideMesh = 0;
			}

			return croppedMesh;
		}

		//unhandled entity
		ccLog::Warning("[Crop] Unhandled entity type");
		return 0;
	}
};

#endif //CC_CROP_TOOL_HEADER
