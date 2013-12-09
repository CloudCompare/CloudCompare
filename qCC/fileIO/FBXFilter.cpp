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

#ifdef CC_FBX_SUPPORT

#include "FBXFilter.h"

//qCC
#include "../ccCoordinatesShiftManager.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccNormalVectors.h>

//FBX SDK
#include <fbxsdk.h>

//System
#include <vector>
#include <assert.h>

CC_FILE_ERROR FBXFilter::saveToFile(ccHObject* entity, const char* filename)
{

	return CC_FERR_NO_ERROR;
}

QString GetAttributeTypeName(FbxNodeAttribute::EType type)
{
	switch(type)
	{ 
	case FbxNodeAttribute::eUnknown: return("unidentified");
	case FbxNodeAttribute::eNull: return("null"); 
	case FbxNodeAttribute::eMarker: return("marker"); 
	case FbxNodeAttribute::eSkeleton: return("skeleton"); 
	case FbxNodeAttribute::eMesh: return("mesh"); 
	case FbxNodeAttribute::eNurbs: return("nurbs"); 
	case FbxNodeAttribute::ePatch: return("patch"); 
	case FbxNodeAttribute::eCamera: return("camera"); 
	case FbxNodeAttribute::eCameraStereo: return("stereo"); 
	case FbxNodeAttribute::eCameraSwitcher: return("camera switcher"); 
	case FbxNodeAttribute::eLight: return("light"); 
	case FbxNodeAttribute::eOpticalReference: return("optical reference"); 
	case FbxNodeAttribute::eOpticalMarker: return("marker"); 
	case FbxNodeAttribute::eNurbsCurve: return("nurbs curve"); 
	case FbxNodeAttribute::eTrimNurbsSurface: return("trim nurbs surface"); 
	case FbxNodeAttribute::eBoundary: return("boundary"); 
	case FbxNodeAttribute::eNurbsSurface: return("nurbs surface"); 
	case FbxNodeAttribute::eShape: return("shape"); 
	case FbxNodeAttribute::eLODGroup: return("lodgroup"); 
	case FbxNodeAttribute::eSubDiv: return("subdiv"); 
	default:
		break;
	}
	
	return("unknown"); 
}

//converts a FBX mesh to a CC mesh
static ccMesh* FromFbxMesh(FbxMesh* fbxMesh, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	if (!fbxMesh)
		return 0;

	int polyCount = fbxMesh->GetPolygonCount();
	unsigned triCount = 0;
	unsigned normalCount = 0;
	//as we can't load all polygons (yet ;) we already look if we can load any!
	{
		unsigned skipped = 0;
		for (int i=0; i<polyCount; ++i)
		{
			int pSize = fbxMesh->GetPolygonSize(i);

			if (pSize == 3)
			{
				++triCount;
				normalCount += 3;
			}
			else if (pSize == 4)
			{
				triCount += 2;
				normalCount += 4;
			}
			else
			{
				++skipped;
			}
		}

		if (triCount == 0)
		{
			ccLog::Warning(QString("[FBX] No triangle or quad found in mesh '%1'! (polygons with more than 4 vertices are not supported for the moment)").arg(fbxMesh->GetName()));
			return 0;
		}
		else if (skipped != 0)
		{
			ccLog::Warning(QString("[FBX] Some polygons in mesh '%1' were ignored (%2): polygons with more than 4 vertices are not supported for the moment)").arg(fbxMesh->GetName()).arg(skipped));
			return 0;
		}
	}

	int vertCount = fbxMesh->GetControlPointsCount();
	if (vertCount <= 0)
	{
		ccLog::Warning(QString("[FBX] Mesh '%1' has no vetex or no polygon?!").arg(fbxMesh->GetName()));
		return 0;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	ccMesh* mesh = new ccMesh(vertices);
	mesh->setName(fbxMesh->GetName());
	mesh->addChild(vertices);
	vertices->setEnabled(false);
	
	if (!mesh->reserve(static_cast<unsigned>(triCount)) || !vertices->reserve(vertCount))
	{
		ccLog::Warning(QString("[FBX] Not enough memory to load mesh '%1'!").arg(fbxMesh->GetName()));
		delete mesh;
		return 0;
	}

	NormsIndexesTableType* normsTable = new NormsIndexesTableType();
	if (!normsTable->reserve(normalCount) || !mesh->reservePerTriangleNormalIndexes())
	{
		ccLog::Warning(QString("[FBX] Not enough memory to load mesh '%1' normals!").arg(fbxMesh->GetName()));
		normsTable->release();
		normsTable = 0;
	}
	else
	{
		mesh->setTriNormsTable(normsTable);
		mesh->addChild(normsTable);
		vertices->showNormals(true);
		mesh->showNormals(true);
	}

	//import polygons
	{
		for (int i=0; i<polyCount; ++i)
		{
			int pSize = fbxMesh->GetPolygonSize(i);

			if (pSize > 4)
			{
				//not handled for the moment
				continue;
			}
			//we split quads into two triangles

			//vertex indices
			int i1 = fbxMesh->GetPolygonVertex(i, 0);
			int i2 = fbxMesh->GetPolygonVertex(i, 1);
			int i3 = fbxMesh->GetPolygonVertex(i, 2);
			mesh->addTriangle(i1,i2,i3);

			if (pSize == 4)
			{
				int i4 = fbxMesh->GetPolygonVertex(i, 3);
				mesh->addTriangle(i1,i3,i4);
			}

			//normals
			if (normsTable)
			{
				int nIndex = static_cast<int>(normsTable->currentSize());
				for (int j=0; j<pSize; ++j)
				{
					FbxVector4 N;
					fbxMesh->GetPolygonVertexNormal(i, j, N);
					CCVector3 Npc(	static_cast<PointCoordinateType>(N.Buffer()[0]),
						static_cast<PointCoordinateType>(N.Buffer()[1]),
						static_cast<PointCoordinateType>(N.Buffer()[2]) );
					normsTable->addElement(ccNormalVectors::GetNormIndex(Npc.u));
				}

				mesh->addTriangleNormalIndexes(nIndex,nIndex+1,nIndex+2);
				if (pSize == 4)
					mesh->addTriangleNormalIndexes(nIndex,nIndex+2,nIndex+3);
			}
		}
		
		if (mesh->size() == 0)
		{
			ccLog::Warning(QString("[FBX] No triangle found in mesh '%1'! (only triangles are supported for the moment)").arg(fbxMesh->GetName()));
			delete mesh;
			return 0;
		}
	}

	//import vertices
	{
		const FbxVector4* fbxVertices = fbxMesh->GetControlPoints();
		assert(vertices && fbxVertices);
		double Pshift[3] = {0,0,0};
		for (int i=0; i<vertCount; ++i, ++fbxVertices)
		{
			const double* P = fbxVertices->Buffer();
			assert(P[3] == 0);

			//coordinate shift management
			if (i == 0)
			{
				bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
				if (shiftAlreadyEnabled)
					memcpy(Pshift,coordinatesShift,sizeof(double)*3);
				bool applyAll=false;
				if (sizeof(PointCoordinateType) < 8 && ccCoordinatesShiftManager::Handle(P,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,applyAll))
				{
					vertices->setOriginalShift(Pshift[0],Pshift[1],Pshift[2]);
					ccLog::Warning("[FBX] Mesh has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift[0],Pshift[1],Pshift[2]);

					//we save coordinates shift information
					if (applyAll && coordinatesShiftEnabled && coordinatesShift)
					{
						*coordinatesShiftEnabled = true;
						coordinatesShift[0] = Pshift[0];
						coordinatesShift[1] = Pshift[1];
						coordinatesShift[2] = Pshift[2];
					}
				}
			}
			CCVector3 PV(	static_cast<PointCoordinateType>(P[0] + Pshift[0]),
							static_cast<PointCoordinateType>(P[1] + Pshift[1]),
							static_cast<PointCoordinateType>(P[2] + Pshift[0]) );

			vertices->addPoint(PV);
		}
	}

	return mesh;
}


CC_FILE_ERROR FBXFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, double* coordinatesShift/*=0*/)
{
	// Initialize the SDK manager. This object handles memory management.
	FbxManager* lSdkManager = FbxManager::Create();

	// Create the IO settings object.
	FbxIOSettings *ios = FbxIOSettings::Create(lSdkManager, IOSROOT);
	lSdkManager->SetIOSettings(ios);

	// Create an importer using the SDK manager.
	FbxImporter* lImporter = FbxImporter::Create(lSdkManager,"");

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	
	// Use the first argument as the filename for the importer.
	if (!lImporter->Initialize(filename, -1, lSdkManager->GetIOSettings()))
	{ 
		ccLog::Warning(QString("[FBX] Error: %1").arg(lImporter->GetStatus().GetErrorString()));
		result = CC_FERR_READING;
	}
	else
	{
		// Create a new scene so that it can be populated by the imported file.
		FbxScene* lScene = FbxScene::Create(lSdkManager,"myScene");

		// Import the contents of the file into the scene.
		if (lImporter->Import(lScene))
		{
			// Print the nodes of the scene and their attributes recursively.
			// Note that we are not printing the root node because it should
			// not contain any attributes.
			FbxNode* lRootNode = lScene->GetRootNode();
			std::vector<FbxNode*> nodes;
			nodes.push_back(lRootNode);

			while (!nodes.empty())
			{
				FbxNode* pNode = nodes.back();
				nodes.pop_back();

				const char* nodeName = pNode->GetName();
#ifdef _DEBUG
				ccLog::Print(QString("Node: %1 - %2 properties").arg(nodeName).arg(pNode->GetNodeAttributeCount()));
#endif
				//FbxDouble3 translation = pNode->LclTranslation.Get(); 
				//FbxDouble3 rotation = pNode->LclRotation.Get(); 
				//FbxDouble3 scaling = pNode->LclScaling.Get();
				
				// scan the node's attributes.
				for(int i=0; i<pNode->GetNodeAttributeCount(); i++)
				{
					FbxNodeAttribute* pAttribute = pNode->GetNodeAttributeByIndex(i);
					FbxNodeAttribute::EType type = pAttribute->GetAttributeType();
#ifdef _DEBUG
					ccLog::Print(QString("\tProp. #%1").arg(GetAttributeTypeName(type)));
#endif

					switch(type)
					{ 
					case FbxNodeAttribute::eMesh:
						{
							ccMesh* mesh = FromFbxMesh(static_cast<FbxMesh*>(pAttribute),alwaysDisplayLoadDialog,coordinatesShiftEnabled,coordinatesShift);
							if (mesh)
							{
								//apply transformation
								FbxAMatrix& transform = lScene->GetEvaluator()->GetNodeGlobalTransform(pNode);
								ccGLMatrix mat;
								float* data = mat.data();
								for (int c=0; c<4; ++c)
								{
									FbxVector4 C = transform.GetColumn(c);
									*data++ = static_cast<float>(C[0]);
									*data++ = static_cast<float>(C[1]);
									*data++ = static_cast<float>(C[2]);
									*data++ = static_cast<float>(C[3]);
								}
								mesh->applyGLTransformation_recursive(&mat);

								container.addChild(mesh);
							}
						}
						break;

					case FbxNodeAttribute::eUnknown: 
					case FbxNodeAttribute::eNull:
					case FbxNodeAttribute::eMarker:
					case FbxNodeAttribute::eSkeleton:
					case FbxNodeAttribute::eNurbs:
					case FbxNodeAttribute::ePatch:
					case FbxNodeAttribute::eCamera:
					case FbxNodeAttribute::eCameraStereo:
					case FbxNodeAttribute::eCameraSwitcher:
					case FbxNodeAttribute::eLight:
					case FbxNodeAttribute::eOpticalReference:
					case FbxNodeAttribute::eOpticalMarker:
					case FbxNodeAttribute::eNurbsCurve:
					case FbxNodeAttribute::eTrimNurbsSurface:
					case FbxNodeAttribute::eBoundary:
					case FbxNodeAttribute::eNurbsSurface:
					case FbxNodeAttribute::eShape:
					case FbxNodeAttribute::eLODGroup:
					case FbxNodeAttribute::eSubDiv:
					default:
						//not handled yet
						break;
					}
				}

				// Recursively add the children.
				for(int j=0; j<pNode->GetChildCount(); j++)
				{
					nodes.push_back(pNode->GetChild(j));
				}
			}
		}
	}

	// The file is imported, so get rid of the importer.
	lImporter->Destroy();
	// Destroy the SDK manager and all the other objects it was handling.
	lSdkManager->Destroy();

	return container.getChildrenNumber() == 0 ? CC_FERR_NO_LOAD : CC_FERR_NO_ERROR;
}

#endif
