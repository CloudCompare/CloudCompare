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
#include "ccCoordinatesShiftManager.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccNormalVectors.h>

//FBX SDK
#include <fbxsdk.h>

//System
#include <vector>
#include <assert.h>

// Converts a CC mesh to an FBX mesh
static FbxNode* ToFbxMesh(ccGenericMesh* mesh, FbxScene* pScene)
{
	if (!mesh)
		return 0;

	FbxMesh* lMesh = FbxMesh::Create(pScene, qPrintable(mesh->getName()));

	ccGenericPointCloud* cloud = mesh->getAssociatedCloud();
	if (!cloud)
		return 0;
	unsigned vertCount = cloud->size();
	unsigned faceCount = mesh->size();

	// Create control points.
	{
		lMesh->InitControlPoints(vertCount);
		FbxVector4* lControlPoints = lMesh->GetControlPoints();

		for (unsigned i=0; i<vertCount; ++i)
		{
			const CCVector3* P = cloud->getPoint(i);
			lControlPoints[i] = FbxVector4(P->x,P->y,P->z);
		}
	}

	ccMesh* asCCMesh = 0;
	if (mesh->isA(CC_TYPES::MESH))
		asCCMesh = static_cast<ccMesh*>(mesh);

	// normals
	if (mesh->hasNormals())
	{
		FbxGeometryElementNormal* lGeometryElementNormal = lMesh->CreateElementNormal();
		if (mesh->hasTriNormals())
		{
			// We want to have one normal per vertex of each polygon,
			// so we set the mapping mode to eByPolygonVertex.
			lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eByPolygonVertex);
			lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eIndexToDirect);
			lGeometryElementNormal->GetIndexArray().SetCount(faceCount*3);
			
			if (asCCMesh)
			{
				NormsIndexesTableType* triNorms = asCCMesh->getTriNormsTable();
				assert(triNorms);
				for (unsigned i=0; i<triNorms->currentSize(); ++i)
				{
					const CCVector3& N = ccNormalVectors::GetNormal(triNorms->getValue(i));
					FbxVector4 Nfbx(N.x,N.y,N.z);
					lGeometryElementNormal->GetDirectArray().Add(Nfbx);
				}
				for (unsigned j=0; j<faceCount; ++j)
				{
					int i1,i2,i3;
					asCCMesh->getTriangleNormalIndexes(j,i1,i2,i3);
					lGeometryElementNormal->GetIndexArray().SetAt(static_cast<int>(j)*3+0, i1);
					lGeometryElementNormal->GetIndexArray().SetAt(static_cast<int>(j)*3+1, i2);
					lGeometryElementNormal->GetIndexArray().SetAt(static_cast<int>(j)*3+2, i3);
				}
			}
			else
			{
				for (unsigned j=0; j<faceCount; ++j)
				{
					//we can't use the 'NormsIndexesTable' so we save all the normals of all the vertices
					CCVector3 Na,Nb,Nc;
					lGeometryElementNormal->GetDirectArray().Add(FbxVector4(Na.x,Na.y,Na.z));
					lGeometryElementNormal->GetDirectArray().Add(FbxVector4(Nb.x,Nb.y,Nb.z));
					lGeometryElementNormal->GetDirectArray().Add(FbxVector4(Nc.x,Nc.y,Nc.z));
					
					mesh->getTriangleNormals(j,Na,Nb,Nc);
					lGeometryElementNormal->GetIndexArray().SetAt(static_cast<int>(j)*3+0, static_cast<int>(j)*3+0);
					lGeometryElementNormal->GetIndexArray().SetAt(static_cast<int>(j)*3+1, static_cast<int>(j)*3+1);
					lGeometryElementNormal->GetIndexArray().SetAt(static_cast<int>(j)*3+2, static_cast<int>(j)*3+2);
				}
			}
		}
		else
		{
			// We want to have one normal for each vertex (or control point),
			// so we set the mapping mode to eByControlPoint.
			lGeometryElementNormal->SetMappingMode(FbxGeometryElement::eByControlPoint);
			// The first method is to set the actual normal value
			// for every control point.
			lGeometryElementNormal->SetReferenceMode(FbxGeometryElement::eDirect);
			for (unsigned i=0; i<vertCount; ++i)
			{
				const CCVector3& N = cloud->getPointNormal(i);
				FbxVector4 Nfbx(N.x,N.y,N.z);
				lGeometryElementNormal->GetDirectArray().Add(Nfbx);
			}
		}
	}
	else
	{
		ccLog::Warning("[FBX] Mesh has no normal! You can manually compute them (select it then call \"Edit > Normals > Compute\")");
	}

	// colors
	if (cloud->hasColors())
	{
		FbxGeometryElementVertexColor* lGeometryElementVertexColor = lMesh->CreateElementVertexColor();
		lGeometryElementVertexColor->SetMappingMode(FbxGeometryElement::eByControlPoint);
		lGeometryElementVertexColor->SetReferenceMode(FbxGeometryElement::eDirect);
		for (unsigned i=0; i<vertCount; ++i)
		{
			const colorType* C = cloud->getPointColor(i);
			FbxColor col( FbxDouble3(	static_cast<double>(C[0])/MAX_COLOR_COMP,
										static_cast<double>(C[1])/MAX_COLOR_COMP,
										static_cast<double>(C[2])/MAX_COLOR_COMP ) );
			lGeometryElementVertexColor->GetDirectArray().Add(col);
		}
	}

	// Set material mapping.
	//FbxGeometryElementMaterial* lMaterialElement = lMesh->CreateElementMaterial();
	//lMaterialElement->SetMappingMode(FbxGeometryElement::eByPolygon);
	//lMaterialElement->SetReferenceMode(FbxGeometryElement::eIndexToDirect);

	// Create polygons. Assign material indices.
	{
		for (unsigned j=0; j<faceCount; ++j)
		{
			const CCLib::TriangleSummitsIndexes* tsi = mesh->getTriangleIndexes(j);

			lMesh->BeginPolygon(static_cast<int>(j));
			lMesh->AddPolygon(tsi->i1);
			lMesh->AddPolygon(tsi->i2);
			lMesh->AddPolygon(tsi->i3);
			lMesh->EndPolygon();
		}
	}

	FbxNode* lNode = FbxNode::Create(pScene,qPrintable(mesh->getName()));

	lNode->SetNodeAttribute(lMesh);

	//CreateMaterials(pScene, lMesh);

	return lNode;
}

static bool SaveScene(FbxManager* pManager, FbxDocument* pScene, const char* pFilename, int pFileFormat = -1, bool pEmbedMedia = false)
{
	// Create an exporter
	FbxExporter* lExporter = FbxExporter::Create(pManager, "");

	if( pFileFormat < 0 || pFileFormat >= pManager->GetIOPluginRegistry()->GetWriterFormatCount() )
	{
		// Write in fall back format in less no ASCII format found
		pFileFormat = pManager->GetIOPluginRegistry()->GetNativeWriterFormat();

		//Try to export in ASCII if possible
		int lFormatIndex, lFormatCount = pManager->GetIOPluginRegistry()->GetWriterFormatCount();

		for (lFormatIndex=0; lFormatIndex<lFormatCount; lFormatIndex++)
		{
			if (pManager->GetIOPluginRegistry()->WriterIsFBX(lFormatIndex))
			{
				FbxString lDesc = pManager->GetIOPluginRegistry()->GetWriterFormatDescription(lFormatIndex);
				const char *lASCII = "ascii";
				if (lDesc.Find(lASCII)>=0)
				{
					pFileFormat = lFormatIndex;
					break;
				}
			}
		} 
	}

	// Set the export states. By default, the export states are always set to 
	// true except for the option eEXPORT_TEXTURE_AS_EMBEDDED. The code below 
	// shows how to change these states
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_MATERIAL,		true);
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_TEXTURE,			true);
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_EMBEDDED,		pEmbedMedia);
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_SHAPE,			true);
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_GOBO,			true);
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_ANIMATION,		true);
	(*(pManager->GetIOSettings())).SetBoolProp(EXP_FBX_GLOBAL_SETTINGS,	true);

	// Initialize the exporter by providing a filename
	if(lExporter->Initialize(pFilename, pFileFormat, pManager->GetIOSettings()) == false)
	{
		ccLog::Warning("[FBX] Call to FbxExporter::Initialize() failed");
		ccLog::Warning("[FBX] Error returned: %s", lExporter->GetStatus().GetErrorString());
		return false;
	}

	// Export the scene
	bool lStatus = lExporter->Export(pScene); 

	// Destroy the exporter
	lExporter->Destroy();

	return lStatus;
}

CC_FILE_ERROR FBXFilter::saveToFile(ccHObject* entity, QString filename)
{
	if (!entity)
		return CC_FERR_BAD_ARGUMENT;
	
	std::vector<ccGenericMesh*> meshes;
	if (entity->isKindOf(CC_TYPES::MESH))
	{
		meshes.push_back(static_cast<ccGenericMesh*>(entity));
	}
	else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		for (unsigned i=0; i<entity->getChildrenNumber(); ++i)
		{
			ccHObject* child = entity->getChild(i);
			if (child->isKindOf(CC_TYPES::MESH))
				meshes.push_back(static_cast<ccGenericMesh*>(child));
		}
	}

	if (meshes.empty())
	{
		return CC_FERR_NO_SAVE;
	}

	//The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
	FbxManager* lSdkManager = FbxManager::Create();
	if( !lSdkManager )
	{
		ccLog::Warning("[FBX] Error: Unable to create FBX Manager!");
		return CC_FERR_CONSOLE_ERROR;
	}
	else
	{
		ccLog::Print("[FBX] Autodesk FBX SDK version %s", lSdkManager->GetVersion());
	}

	//Create an IOSettings object. This object holds all import/export settings.
	FbxIOSettings* ios = FbxIOSettings::Create(lSdkManager, IOSROOT);
	lSdkManager->SetIOSettings(ios);

	//Load plugins from the executable directory (optional)
	//FbxString lPath = FbxGetApplicationDirectory();
	//lSdkManager->LoadPluginsDirectory(lPath.Buffer());

	//Create an FBX scene. This object holds most objects imported/exported from/to files.
	FbxScene* lScene = FbxScene::Create(lSdkManager, "My Scene");
	if( !lScene )
	{
		ccLog::Warning("[FBX] Error: Unable to create FBX scene!");
		return CC_FERR_CONSOLE_ERROR;
	}

	// create scene info
	{
		FbxDocumentInfo* sceneInfo = FbxDocumentInfo::Create(lSdkManager,"SceneInfo");
		sceneInfo->mTitle = qPrintable(QString("Mesh: ") + (meshes.size() == 1 ? meshes[0]->getName() : QString("Multiple meshes")));
		sceneInfo->mAuthor = "CloudCompare";
		sceneInfo->mRevision = "rev. 1.0";
		sceneInfo->mKeywords = "cloudcompare mesh";

		// we need to add the sceneInfo before calling AddThumbNailToScene because
		// that function is asking the scene for the sceneInfo.
		lScene->SetSceneInfo(sceneInfo);
	}

	//create thumbnail
	//{
	//	FbxThumbnail* lThumbnail = FbxThumbnail::Create(lScene,"");

	//	lThumbnail->SetDataFormat(FbxThumbnail::eRGB_24);
	//	lThumbnail->SetSize(FbxThumbnail::e64x64);
	//	lThumbnail->SetThumbnailImage(cSceneThumbnail);

	//	if (lScene->GetSceneInfo())
	//	{
	//		lScene->GetSceneInfo()->SetSceneThumbnail(lThumbnail);
	//	}
	//}

	// Build the node tree.
	FbxNode* lRootNode = lScene->GetRootNode();
	{
		for (size_t i=0; i<meshes.size(); ++i)
		{
			FbxNode* meshNode = ToFbxMesh(meshes[i],lScene);
			if (meshNode)
				lRootNode->AddChild(meshNode);
			else
				ccLog::Warning(QString("[FBX] Failed to convert mesh '%1' to FBX mesh/node!").arg(meshes[i]->getName()));
		}
	}

	// Save the scene.
	bool lResult = SaveScene(lSdkManager, lScene, qPrintable(filename));

	// Destroy all objects created by the FBX SDK.
	if( lSdkManager )
		lSdkManager->Destroy();

	return lResult ? CC_FERR_NO_ERROR : CC_FERR_CONSOLE_ERROR;
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
static ccMesh* FromFbxMesh(FbxMesh* fbxMesh, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	if (!fbxMesh)
		return 0;

	int polyCount = fbxMesh->GetPolygonCount();
	//fbxMesh->GetLayer(
	unsigned triCount = 0;
	unsigned polyVertCount = 0; //different from vertCount (vertices can be counted multiple times here!)
	//as we can't load all polygons (yet ;) we already look if we can load any!
	{
		unsigned skipped = 0;
		for (int i=0; i<polyCount; ++i)
		{
			int pSize = fbxMesh->GetPolygonSize(i);

			if (pSize == 3)
			{
				++triCount;
				polyVertCount += 3;
			}
			else if (pSize == 4)
			{
				triCount += 2;
				polyVertCount += 4;
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

	//colors
	{
		for (int l=0; l<fbxMesh->GetElementVertexColorCount(); l++)
		{
			FbxGeometryElementVertexColor* vertColor = fbxMesh->GetElementVertexColor(l);
			//CC can only handle per-vertex colors
			if (vertColor->GetMappingMode() == FbxGeometryElement::eByControlPoint)
			{
				if (vertColor->GetReferenceMode() == FbxGeometryElement::eDirect
					|| vertColor->GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
				{
					if (vertices->reserveTheRGBTable())
					{
						switch (vertColor->GetReferenceMode())
						{
						case FbxGeometryElement::eDirect:
							{
								for (int i=0; i<vertCount; ++i)
								{
									FbxColor c = vertColor->GetDirectArray().GetAt(i);
									vertices->addRGBColor(	static_cast<colorType>(c.mRed	* MAX_COLOR_COMP),
															static_cast<colorType>(c.mGreen	* MAX_COLOR_COMP),
															static_cast<colorType>(c.mBlue	* MAX_COLOR_COMP) );
								}
							}
							break;
						case FbxGeometryElement::eIndexToDirect:
							{
								for (int i=0; i<vertCount; ++i)
								{
									int id = vertColor->GetIndexArray().GetAt(i);
									FbxColor c = vertColor->GetDirectArray().GetAt(id);
									vertices->addRGBColor(	static_cast<colorType>(c.mRed	* MAX_COLOR_COMP),
															static_cast<colorType>(c.mGreen	* MAX_COLOR_COMP),
															static_cast<colorType>(c.mBlue	* MAX_COLOR_COMP) );
								}
							}
							break;
						default:
							assert(false);
							break;
						}

						vertices->showColors(true);
						mesh->showColors(true);
						break; //no need to look for other color fields (we won't be able to handle them!
					}
					else
					{
						ccLog::Warning(QString("[FBX] Not enough memory to load mesh '%1' colors!").arg(fbxMesh->GetName()));
					}
				}
				else
				{
					ccLog::Warning(QString("[FBX] Color field #%i of mesh '%1' will be ignored (unhandled type)").arg(l).arg(fbxMesh->GetName()));
				}
			}
			else
			{
				ccLog::Warning(QString("[FBX] Color field #%i of mesh '%1' will be ignored (unhandled type)").arg(l).arg(fbxMesh->GetName()));
			}
		}
	}


	//normals can be per vertices or per-triangle
	int perPointNormals = -1;
	int perVertexNormals = -1;
	int perPolygonNormals = -1;
	{
		for (int j=0; j<fbxMesh->GetElementNormalCount(); j++)
		{
			FbxGeometryElementNormal* leNormals = fbxMesh->GetElementNormal(j);
			switch(leNormals->GetMappingMode())
			{
			case FbxGeometryElement::eByControlPoint:
				perPointNormals = j;
				break;
			case FbxGeometryElement::eByPolygonVertex:
				perVertexNormals = j;
				break;
			case FbxGeometryElement::eByPolygon:
				perPolygonNormals = j;
				break;
			default:
				//not handled
				break;
			}
		}
	}

	//per-point normals
	if (perPointNormals >= 0)
	{
		FbxGeometryElementNormal* leNormals = fbxMesh->GetElementNormal(perPointNormals);
		FbxLayerElement::EReferenceMode refMode = leNormals->GetReferenceMode();
		const FbxLayerElementArrayTemplate<FbxVector4>& normals = leNormals->GetDirectArray();
		assert(normals.GetCount() == vertCount);
		if (normals.GetCount() != vertCount)
		{
			ccLog::Warning(QString("[FBX] Wrong number of normals on mesh '%1'!").arg(fbxMesh->GetName()));
			perPointNormals = -1;
		}
		else if (!vertices->reserveTheNormsTable())
		{
			ccLog::Warning(QString("[FBX] Not enough memory to load mesh '%1' normals!").arg(fbxMesh->GetName()));
			perPointNormals = -1;
		}
		else
		{
			//import normals
			for (int i=0; i<vertCount; ++i)
			{
				int id = refMode != FbxGeometryElement::eDirect ? leNormals->GetIndexArray().GetAt(i) : i;
				FbxVector4 N = normals.GetAt(id);
				//convert to CC-structure
				CCVector3 Npc(	static_cast<PointCoordinateType>(N.Buffer()[0]),
								static_cast<PointCoordinateType>(N.Buffer()[1]),
								static_cast<PointCoordinateType>(N.Buffer()[2]) );
				vertices->addNorm(Npc);
			}
			vertices->showNormals(true);
			mesh->showNormals(true);
			//no need to import the other normals (if any)
			perVertexNormals = -1;
			perPolygonNormals = -1;
		}
	}

	//per-triangle normals
	NormsIndexesTableType* normsTable = 0;
	if (perVertexNormals >= 0 || perPolygonNormals >= 0)
	{
		normsTable = new NormsIndexesTableType();
		if (!normsTable->reserve(polyVertCount) || !mesh->reservePerTriangleNormalIndexes())
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
	}

	//import textures UV
	int perVertexUV = -1;
	bool hasTexUV = false;
	{
		for (int l=0; l<fbxMesh->GetElementUVCount(); ++l)
		{
			FbxGeometryElementUV* leUV = fbxMesh->GetElementUV(l);
			//per-point UV coordinates
			if (leUV->GetMappingMode() == FbxGeometryElement::eByControlPoint)
			{
				TextureCoordsContainer* vertTexUVTable = new TextureCoordsContainer();
				if (!vertTexUVTable->reserve(vertCount) || !mesh->reservePerTriangleTexCoordIndexes())
				{
					vertTexUVTable->release();
					ccLog::Warning(QString("[FBX] Not enough memory to load mesh '%1' UV coordinates!").arg(fbxMesh->GetName()));
				}
				else
				{
					FbxLayerElement::EReferenceMode refMode = leUV->GetReferenceMode();
					for (int i=0; i<vertCount; ++i)
					{
						int id = refMode != FbxGeometryElement::eDirect ? leUV->GetIndexArray().GetAt(i) : i;
						FbxVector2 uv = leUV->GetDirectArray().GetAt(id);
						//convert to CC-structure
						float uvf[2] = {static_cast<float>(uv.Buffer()[0]),
										static_cast<float>(uv.Buffer()[1])};
						vertTexUVTable->addElement(uvf);
					}
					mesh->addChild(vertTexUVTable);
					hasTexUV = true;
				}
				perVertexUV = -1;
				break; //no need to look to the other UV fields (can't handle them!)
			}
			else if (leUV->GetMappingMode() == FbxGeometryElement::eByPolygonVertex)
			{
				//per-vertex UV coordinates
				perVertexUV = l;
			}
		}
	}

	//per-vertex UV coordinates
	TextureCoordsContainer* texUVTable = 0;
	if (perVertexUV >= 0)
	{
		texUVTable = new TextureCoordsContainer();
		if (!texUVTable->reserve(polyVertCount) || !mesh->reservePerTriangleTexCoordIndexes())
		{
			texUVTable->release();
			ccLog::Warning(QString("[FBX] Not enough memory to load mesh '%1' UV coordinates!").arg(fbxMesh->GetName()));
		}
		else
		{
			mesh->addChild(texUVTable);
			hasTexUV = true;
		}
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

			int i4 = -1;
			if (pSize == 4)
			{
				i4 = fbxMesh->GetPolygonVertex(i, 3);
				mesh->addTriangle(i1,i3,i4);
			}

			if (hasTexUV)
			{
				if (texUVTable)
				{
					assert(perVertexUV >= 0);

					int uvIndex = static_cast<int>(texUVTable->currentSize());
					for (int j=0; j<pSize; ++j)
					{
						int lTextureUVIndex = fbxMesh->GetTextureUVIndex(i, j);
						FbxGeometryElementUV* leUV = fbxMesh->GetElementUV(perVertexUV);
						FbxVector2 uv = leUV->GetDirectArray().GetAt(lTextureUVIndex);
						//convert to CC-structure
						float uvf[2] = {static_cast<float>(uv.Buffer()[0]),
										static_cast<float>(uv.Buffer()[1])};
						texUVTable->addElement(uvf);
					}
					mesh->addTriangleTexCoordIndexes(uvIndex,uvIndex+1,uvIndex+2);
					if (pSize == 4)
						mesh->addTriangleTexCoordIndexes(uvIndex,uvIndex+2,uvIndex+3);
				}
				else
				{
					mesh->addTriangleTexCoordIndexes(i1,i2,i3);
					if (pSize == 4)
						mesh->addTriangleTexCoordIndexes(i1,i3,i4);
				}
			}

			//per-triangle normals
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
		CCVector3d Pshift(0,0,0);
		for (int i=0; i<vertCount; ++i, ++fbxVertices)
		{
			const double* P = fbxVertices->Buffer();
			assert(P[3] == 0);

			//coordinate shift management
			if (i == 0)
			{
				bool shiftAlreadyEnabled = (coordinatesShiftEnabled && *coordinatesShiftEnabled && coordinatesShift);
				if (shiftAlreadyEnabled)
					Pshift = *coordinatesShift;
				bool applyAll = false;
				if (	sizeof(PointCoordinateType) < 8
					&&	ccCoordinatesShiftManager::Handle(P,0,alwaysDisplayLoadDialog,shiftAlreadyEnabled,Pshift,0,&applyAll))
				{
					vertices->setGlobalShift(Pshift);
					ccLog::Warning("[FBX] Mesh has been recentered! Translation: (%.2f,%.2f,%.2f)",Pshift.x,Pshift.y,Pshift.z);

					//we save coordinates shift information
					if (applyAll && coordinatesShiftEnabled && coordinatesShift)
					{
						*coordinatesShiftEnabled = true;
						*coordinatesShift = Pshift;
					}
				}
			}

			CCVector3 PV(	static_cast<PointCoordinateType>(P[0] + Pshift.x),
							static_cast<PointCoordinateType>(P[1] + Pshift.y),
							static_cast<PointCoordinateType>(P[2] + Pshift.z) );

			vertices->addPoint(PV);
		}
	}

	//import textures
	{
		//TODO
	}

	return mesh;
}


CC_FILE_ERROR FBXFilter::loadFile(QString filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	// Initialize the SDK manager. This object handles memory management.
	FbxManager* lSdkManager = FbxManager::Create();

	// Create the IO settings object.
	FbxIOSettings *ios = FbxIOSettings::Create(lSdkManager, IOSROOT);
	lSdkManager->SetIOSettings(ios);
	
	// Import options determine what kind of data is to be imported.
	// True is the default, but here we’ll set some to true explicitly, and others to false.
	//(*(lSdkManager->GetIOSettings())).SetBoolProp(IMP_FBX_MATERIAL,	true);
	//(*(lSdkManager->GetIOSettings())).SetBoolProp(IMP_FBX_TEXTURE,	true);
	
	// Create an importer using the SDK manager.
	FbxImporter* lImporter = FbxImporter::Create(lSdkManager,"");

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	
	// Use the first argument as the filename for the importer.
	if (!lImporter->Initialize(qPrintable(filename), -1, lSdkManager->GetIOSettings()))
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
				FbxNode* lNode = nodes.back();
				nodes.pop_back();

				const char* nodeName = lNode->GetName();
#ifdef _DEBUG
				ccLog::Print(QString("Node: %1 - %2 properties").arg(nodeName).arg(lNode->GetNodeAttributeCount()));
#endif
				// scan the node's attributes.
				for(int i=0; i<lNode->GetNodeAttributeCount(); i++)
				{
					FbxNodeAttribute* pAttribute = lNode->GetNodeAttributeByIndex(i);
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
								FbxAMatrix& transform = lNode->EvaluateGlobalTransform();
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

								if (mesh->getName().isEmpty())
									mesh->setName(nodeName);

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
				for(int j=0; j<lNode->GetChildCount(); j++)
				{
					nodes.push_back(lNode->GetChild(j));
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
