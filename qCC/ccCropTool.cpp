//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccCropTool.h"

//qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccLog.h>
#include <ccScalarField.h>
#include <ccMaterial.h>
#include <ccMaterialSet.h>

//CCLib
#include <ManualSegmentationTools.h>
#include <SimpleMesh.h>

ccHObject* ccCropTool::Crop(ccHObject* entity, const ccBBox& box, bool inside/*=true*/, const ccGLMatrix* meshRotation/*=0*/)
{
	assert(entity);
	if (!entity)
	{
		return 0;
	}

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
			ccLog::Warning(QString("[Crop] No point of the cloud '%1' falls %2side the input box!").arg(cloud->getName(), (inside ? "in" : "out")));
			delete selection;
			return 0;
		}

		//crop
		ccPointCloud* croppedEnt = cloud->partialClone(selection);
		delete selection;
		selection = 0;

		return croppedEnt;
	}
	else if (entity->isKindOf(CC_TYPES::MESH))
	{
		ccGenericMesh* mesh = static_cast<ccGenericMesh*>(entity);
		CCLib::ManualSegmentationTools::MeshCutterParams params;
		params.bbMin = CCVector3d::fromArray(box.minCorner().u);
		params.bbMax = CCVector3d::fromArray(box.maxCorner().u);
		params.generateOutsideMesh = !inside;
		params.trackOrigIndexes = mesh->hasColors() || mesh->hasScalarFields() || mesh->hasMaterials();

		ccGenericPointCloud* origVertices = mesh->getAssociatedCloud();
		assert(origVertices);
		ccGenericPointCloud* cropVertices = origVertices;
		if (meshRotation)
		{
			ccPointCloud* rotatedVertices = ccPointCloud::From(origVertices);
			if (!rotatedVertices)
			{
				ccLog::Warning(QString("[Crop] Failed to crop mesh '%1'! (not enough memory)").arg(mesh->getName()));
				return 0;
			}
			rotatedVertices->setGLTransformation(*meshRotation);
			rotatedVertices->applyGLTransformation_recursive();
			cropVertices = rotatedVertices;
		}

		if (!CCLib::ManualSegmentationTools::segmentMeshWitAABox(mesh, cropVertices, params))
		{
			//process failed!
			ccLog::Warning(QString("[Crop] Failed to crop mesh '%1'!").arg(mesh->getName()));
		}

		if (cropVertices != origVertices)
		{
			//don't need those anymore
			delete cropVertices;
			cropVertices = origVertices;
		}

		CCLib::SimpleMesh* tempMesh = inside ? params.insideMesh : params.outsideMesh;

		//output
		ccMesh* croppedMesh = 0;

		if (tempMesh)
		{
			ccPointCloud* croppedVertices = ccPointCloud::From(tempMesh->vertices());
			if (croppedVertices)
			{
				if (meshRotation)
				{
					//apply inverse transformation
					croppedVertices->setGLTransformation(meshRotation->inverse());
					croppedVertices->applyGLTransformation_recursive();
				}
				croppedMesh = new ccMesh(tempMesh, croppedVertices);
				croppedMesh->addChild(croppedVertices);
				croppedVertices->setEnabled(false);
				if (croppedMesh->size() == 0)
				{
					//no points fall inside selection!
					ccLog::Warning(QString("[Crop] No triangle of the mesh '%1' falls %2side the input box!").arg(mesh->getName(), (inside ? "in" : "out")));
					delete croppedMesh;
					croppedMesh = 0;
				}
				else
				{
					assert(origVertices);
					
					//import parameters
					croppedVertices->importParametersFrom(origVertices);
					croppedMesh->importParametersFrom(mesh);

					//compute normals if necessary
					if (mesh->hasNormals())
					{
						bool success = false;
						if (mesh->hasTriNormals())
							success = croppedMesh->computePerTriangleNormals();
						else
							success = croppedMesh->computePerVertexNormals();

						if (!success)
						{
							ccLog::Warning("[Crop] Failed to compute normals on the output mesh (not enough memory)");
						}
						croppedMesh->showNormals(success && mesh->normalsShown());
					}

					//import other features if necessary
					if (params.trackOrigIndexes)
					{
						const std::vector<unsigned>& origTriIndexes = inside ? params.origTriIndexesMapInside : params.origTriIndexesMapOutside;

						try
						{
							//per vertex features (RGB color & scalar fields)
							if (origVertices->hasColors() || origVertices->hasScalarFields())
							{
								//we use flags to avoid processing the same vertex multiple times
								std::vector<bool> vertProcessed(croppedVertices->size(), false);

								//colors
								bool importColors = false;
								if (origVertices->hasColors())
								{
									importColors = croppedVertices->resizeTheRGBTable();
									if (!importColors)
										ccLog::Warning("[Crop] Failed to transfer RGB colors on the output mesh (not enough memory)");
								}
								
								//scalar fields
								std::vector<ccScalarField*> importedSFs;
								ccPointCloud* origVertices_pc = 0;
								if (origVertices->hasScalarFields())
								{
									origVertices_pc = origVertices->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(origVertices) : 0;
									unsigned sfCount = origVertices_pc ? origVertices_pc->getNumberOfScalarFields() : 1;
									
									//now try to import each SF
									for (unsigned i = 0; i < sfCount; ++i)
									{
										int sfIdx = croppedVertices->addScalarField(origVertices_pc ? origVertices_pc->getScalarField(i)->getName() : "Scalar field");
										if (sfIdx >= 0)
										{
											ccScalarField* sf = static_cast<ccScalarField*>(croppedVertices->getScalarField(i));
											sf->fill(NAN_VALUE);
											if (origVertices_pc)
											{
												//import display parameters if possible
												ccScalarField* originSf = static_cast<ccScalarField*>(origVertices_pc->getScalarField(i));
												assert(originSf);
												//copy display parameters
												sf->importParametersFrom(originSf);
											}
											importedSFs.push_back(sf);
										}
										else
										{
											ccLog::Warning("[Crop] Failed to transfer one or several scalar fields on the output mesh (not enough memory)");
											//we can stop right now as all SFs have the same size!
											break;
										}
									}

									//default displayed SF
									if (origVertices_pc)
										croppedVertices->setCurrentDisplayedScalarField(std::max(static_cast<int>(croppedVertices->getNumberOfScalarFields())-1, origVertices_pc->getCurrentDisplayedScalarFieldIndex()));
									else
										croppedVertices->setCurrentDisplayedScalarField(0);
								}
								bool importSFs = !importedSFs.empty();

								if (importColors || importSFs)
								{
									//for each new triangle
									for (unsigned i = 0; i < croppedMesh->size(); ++i)
									{
										//get the origin triangle
										unsigned origTriIndex = origTriIndexes[i];
										const CCLib::VerticesIndexes* tsio = mesh->getTriangleVertIndexes(origTriIndex);

										//get the new triangle
										const CCLib::VerticesIndexes* tsic = croppedMesh->getTriangleVertIndexes(i);

										//we now have to test the 3 vertices of the new triangle
										for (unsigned j = 0; j < 3; ++j)
										{
											unsigned vertIndex = tsic->i[j];
											
											if (vertProcessed[vertIndex])
											{
												//vertex has already been process
												continue;
											}

											const CCVector3* Vcj = croppedVertices->getPoint(vertIndex);

											//we'll deduce its color and SFs values by triangulation
											if (importColors)
											{
												ccColor::Rgb col;
												if (mesh->interpolateColors(origTriIndex, *Vcj, col))
												{
													croppedVertices->setPointColor(vertIndex, col);
												}
											}

											if (importSFs)
											{
												CCVector3d w;
												mesh->computeInterpolationWeights(origTriIndex, *Vcj, w);

												//import SFs
												for (unsigned s = 0; s < static_cast<unsigned>(importedSFs.size()); ++s)
												{
													CCVector3d scalarValues(0, 0, 0);
													if (origVertices_pc)
													{
														const CCLib::ScalarField* sf = origVertices_pc->getScalarField(s);
														scalarValues.x = sf->getValue(tsio->i1);
														scalarValues.y = sf->getValue(tsio->i2);
														scalarValues.z = sf->getValue(tsio->i3);
													}
													else
													{
														assert(s == 0);
														scalarValues.x = origVertices->getPointScalarValue(tsio->i1);
														scalarValues.y = origVertices->getPointScalarValue(tsio->i2);
														scalarValues.z = origVertices->getPointScalarValue(tsio->i3);
													}

													ScalarType sVal = static_cast<ScalarType>(scalarValues.dot(w));
													importedSFs[s]->setValue(vertIndex,sVal);
												}
											}

											//update 'processed' flag
											vertProcessed[vertIndex] = true;
										}
									}

									for (size_t s = 0; s < importedSFs.size(); ++s)
									{
										importedSFs[s]->computeMinAndMax();
									}
								
									croppedVertices->showColors(importColors && origVertices->colorsShown());
									croppedVertices->showSF(importSFs && origVertices->sfShown());
									croppedMesh->showColors(importColors && mesh->colorsShown());
									croppedMesh->showSF(importSFs && mesh->sfShown());
								}
							}

							//per-triangle features (materials)
							if (mesh->hasMaterials())
							{
								const ccMaterialSet* origMaterialSet = mesh->getMaterialSet();
								assert(origMaterialSet);
								
								if (origMaterialSet && !origMaterialSet->empty() && croppedMesh->reservePerTriangleMtlIndexes())
								{
									std::vector<int> materialUsed(origMaterialSet->size(),-1);
									
									//per-triangle materials
									for (unsigned i = 0; i < croppedMesh->size(); ++i)
									{
										//get the origin triangle
										unsigned origTriIndex = origTriIndexes[i];
										int mtlIndex = mesh->getTriangleMtlIndex(origTriIndex);
										croppedMesh->addTriangleMtlIndex(mtlIndex);
										
										if (mtlIndex >= 0)
											materialUsed[mtlIndex] = 1;
									}

									//import materials
									{
										size_t materialUsedCount = 0;
										{
											for (size_t i = 0; i < materialUsed.size(); ++i)
												if (materialUsed[i] == 1)
													++materialUsedCount;
										}

										if (materialUsedCount == materialUsed.size())
										{
											//nothing to do, we use all input materials
											croppedMesh->setMaterialSet(origMaterialSet->clone());
										}
										else
										{
											//create a subset of the input materials
											ccMaterialSet* matSet = new ccMaterialSet(origMaterialSet->getName());
											{
												matSet->reserve(materialUsedCount);
												for (size_t i = 0; i < materialUsed.size(); ++i)
												{
													if (materialUsed[i] >= 0)
													{
														matSet->push_back(ccMaterial::Shared(new ccMaterial(*origMaterialSet->at(i))));
														//update index
														materialUsed[i] = static_cast<int>(matSet->size()) - 1;
													}
												}
											}
											croppedMesh->setMaterialSet(matSet);
											
											//and update the materials indexes!
											for (unsigned i = 0; i < croppedMesh->size(); ++i)
											{
												int mtlIndex = croppedMesh->getTriangleMtlIndex(i);
												if (mtlIndex >= 0)
												{
													assert(mtlIndex < static_cast<int>(materialUsed.size()));
													croppedMesh->setTriangleMtlIndex(i,materialUsed[mtlIndex]);
												}
											}
										}
									}

									croppedMesh->showMaterials(mesh->materialsShown());
								}
								else
								{
									ccLog::Warning("[Crop] Failed to transfer materials on the output mesh (not enough memory)");
								}

								//per-triangle texture coordinates
								if (mesh->hasPerTriangleTexCoordIndexes())
								{
									TextureCoordsContainer* texCoords = new TextureCoordsContainer;
									if (	croppedMesh->reservePerTriangleTexCoordIndexes()
										&&	texCoords->reserveSafe(croppedMesh->size()*3))
									{
										//for each new triangle
										for (unsigned i = 0; i < croppedMesh->size(); ++i)
										{
											//get the origin triangle
											unsigned origTriIndex = origTriIndexes[i];
											TexCoords2D* tx1 = 0;
											TexCoords2D* tx2 = 0;
											TexCoords2D* tx3 = 0;
											mesh->getTriangleTexCoordinates(origTriIndex, tx1, tx2, tx3);

											//get the new triangle
											const CCLib::VerticesIndexes* tsic = croppedMesh->getTriangleVertIndexes(i);

											//for each vertex of the new triangle
											int texIndexes[3] = { -1, -1, -1 };
											for (unsigned j = 0; j < 3; ++j)
											{
												unsigned vertIndex = tsic->i[j];
												const CCVector3* Vcj = croppedVertices->getPoint(vertIndex);

												//intepolation weights
												CCVector3d w;
												mesh->computeInterpolationWeights(origTriIndex, *Vcj, w);

												if (	(tx1 || w.u[0] < ZERO_TOLERANCE)
													&&	(tx2 || w.u[1] < ZERO_TOLERANCE)
													&&	(tx3 || w.u[2] < ZERO_TOLERANCE) )
												{
													TexCoords2D t(	static_cast<float>((tx1 ? tx1->tx*w.u[0] : 0.0) + (tx2 ? tx2->tx*w.u[1] : 0.0) + (tx3 ? tx3->tx*w.u[2] : 0.0)),
																	static_cast<float>((tx1 ? tx1->ty*w.u[0] : 0.0) + (tx2 ? tx2->ty*w.u[1] : 0.0) + (tx3 ? tx3->ty*w.u[2] : 0.0)) );

													texCoords->addElement(t);
													texIndexes[j] = static_cast<int>(texCoords->currentSize()) - 1;
												}
											}
											
											croppedMesh->addTriangleTexCoordIndexes(texIndexes[0], texIndexes[1], texIndexes[2]);
										}
										croppedMesh->setTexCoordinatesTable(texCoords);
									}
									else
									{
										ccLog::Warning("[Crop] Failed to transfer texture coordinates on the output mesh (not enough memory)");
										delete texCoords;
										texCoords = 0;
									}
								}
							}
						}
						catch (const std::bad_alloc&)
						{
							ccLog::Warning("[Crop] Failed to transfer per-vertex features (color, SF values, etc.) on the output mesh (not enough memory)");
							croppedVertices->unallocateColors();
							croppedVertices->deleteAllScalarFields();
						}
					}
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

		if (croppedMesh)
		{
			croppedMesh->setDisplay_recursive(entity->getDisplay());
		}
		return croppedMesh;
	}

	//unhandled entity
	ccLog::Warning("[Crop] Unhandled entity type");
	return 0;
}
