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

#include "ccCropTool.h"

//qCC_db
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccLog.h>
#include <ccScalarField.h>

//CCLib
#include <ManualSegmentationTools.h>
#include <SimpleMesh.h>

ccHObject* ccCropTool::Crop(ccHObject* entity, const ccBBox& box, bool inside/*=true*/)
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
		params.trackOrigIndexes = mesh->hasColors() || mesh->hasScalarFields() || mesh->hasMaterials();

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
			ccPointCloud* croppedVertices = ccPointCloud::From(tempMesh->vertices());
			if (croppedVertices)
			{
				croppedMesh = new ccMesh(tempMesh, croppedVertices);
				croppedMesh->addChild(croppedVertices);
				croppedVertices->setEnabled(false);
				if (croppedMesh->size() == 0)
				{
					//no points fall inside selection!
					ccLog::Warning(QString("[Crop] No trinagle of the mesh '%1' falls %2side the input box!").arg(mesh->getName()).arg(inside ? "in" : "out"));
					delete croppedMesh;
					croppedMesh = 0;
				}
				else
				{
					ccGenericPointCloud* origVertices = mesh->getAssociatedCloud();
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
								std::vector<bool> vertProcessed(croppedVertices->size(),false);

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
									for (unsigned i=0; i<sfCount; ++i)
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
									for (unsigned i=0; i<croppedMesh->size(); ++i)
									{
										//get the origin triangle
										unsigned origTriIndex = origTriIndexes[i];
										const CCLib::TriangleSummitsIndexes* tsio = mesh->getTriangleIndexes(origTriIndex);
										const CCVector3* Vo[3] = {	origVertices->getPoint(tsio->i1),
																	origVertices->getPoint(tsio->i2),
																	origVertices->getPoint(tsio->i3) };

										//get the new triangle
										const CCLib::TriangleSummitsIndexes* tsic = croppedMesh->getTriangleIndexes(i);

										//we now have to test the 3 vertices of the new triangle
										for (unsigned j=0; j<3; ++j)
										{
											unsigned vertIndex = tsic->i[j];
											
											if (vertProcessed[vertIndex])
											{
												//vertex has already been process
												continue;
											}

											const CCVector3* Vcj = croppedVertices->getPoint(vertIndex);
											bool matchOriginVertex = false;
											for (unsigned k=0; k<3; ++k)
											{
												const CCVector3* Vok = Vo[k];
												if ((*Vcj-*Vok).norm2d() < params.epsilon*params.epsilon)
												{
													unsigned origVertIndex = tsio->i[k];
													//simply import the RGB color
													if (importColors)
													{
														croppedVertices->setPointColor(vertIndex,origVertices->getPointColor(origVertIndex));
													}
													//...and SF value(s)
													if (importSFs)
													{
														if (origVertices_pc)
														{
															//import multiple SF
															for (unsigned s=0; s<static_cast<unsigned>(importedSFs.size()); ++s)
															{
																const CCLib::ScalarField* sf = origVertices_pc->getScalarField(s);
																importedSFs[s]->setValue(vertIndex,sf->getValue(origVertIndex));
															}
														}
														else
														{
															assert(importedSFs.size() == 1);
															importedSFs.front()->setValue(vertIndex,origVertices->getPointScalarValue(origVertIndex));
														}
													}

													matchOriginVertex = true;
													break;
												}
											}

											if (!matchOriginVertex)
											{
												//if we couldn't find a 'match' then it means
												//that the vertex has been created inside the triangle
												//we'll deduce its color and SFs values by triangulation
												if (importColors)
												{
													ccColor::Rgb col;
													if (mesh->interpolateColors(origTriIndex, *Vcj, col))
													{
														croppedVertices->setPointColor(vertIndex,col.rgb);
													}
												}
												
												if (importSFs)
												{
													//intepolation weights
													double d1 = sqrt(((*Vcj-*Vo[1]).cross(*Vo[2]-*Vo[1])).norm2d())/*/2.0*/;
													double d2 = sqrt(((*Vcj-*Vo[2]).cross(*Vo[0]-*Vo[2])).norm2d())/*/2.0*/;
													double d3 = sqrt(((*Vcj-*Vo[0]).cross(*Vo[1]-*Vo[1])).norm2d())/*/2.0*/;
													//we must normalize weights
													double dsum = d1+d2+d3;
													d1 /= dsum;
													d2 /= dsum;
													d3 /= dsum;
													if (origVertices_pc)
													{
														//import multiple SF
														for (unsigned s=0; s<static_cast<unsigned>(importedSFs.size()); ++s)
														{
															const CCLib::ScalarField* sf = origVertices_pc->getScalarField(s);
															ScalarType s1 = sf->getValue(tsio->i1);
															ScalarType s2 = sf->getValue(tsio->i2);
															ScalarType s3 = sf->getValue(tsio->i3);
															
															ScalarType sVal = static_cast<ScalarType>(s1*d1 + s2*d2 + s3*d3);
															importedSFs[s]->setValue(vertIndex,sVal);
														}
													}
													else
													{
														assert(importedSFs.size() == 1);
														ScalarType s1 = origVertices->getPointScalarValue(tsio->i1);
														ScalarType s2 = origVertices->getPointScalarValue(tsio->i2);
														ScalarType s3 = origVertices->getPointScalarValue(tsio->i3);

														ScalarType sVal = static_cast<ScalarType>(s1*d1 + s2*d2 + s3*d3);
														importedSFs.front()->setValue(vertIndex,sVal);
													}
												}
											}

											//update 'processed' flag
											vertProcessed[vertIndex] = true;
										}

									}

									for (size_t s=0; s<importedSFs.size(); ++s)
									{
										importedSFs[s]->computeMinAndMax();
									}
								
									croppedVertices->showColors(importColors && origVertices->colorsShown());
									croppedVertices->showSF(importSFs && origVertices->sfShown());
									croppedMesh->showColors(importColors && mesh->colorsShown());
									croppedMesh->showSF(importSFs && mesh->sfShown());
								}
							}
						}
						catch(std::bad_alloc)
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

		return croppedMesh;
	}

	//unhandled entity
	ccLog::Warning("[Crop] Unhandled entity type");
	return 0;
}
