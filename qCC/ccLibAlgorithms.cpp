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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QElapsedTimer>
#include <QInputDialog>
#include <QMessageBox>

#include "ScalarField.h"
#include "ScalarFieldTools.h"

#include "ccCommon.h"
#include "ccConsole.h"

#include "ccOctree.h"
#include "ccPointCloud.h"

#include "ccCurvatureDlg.h"
#include "ccDensityDlg.h"
#include "ccProgressDialog.h"
#include "ccRegistrationTools.h"

#include "ccLibAlgorithms.h"
#include "ccUtils.h"

// This is included only for temporarily removing an object from the tree.
//	TODO figure out a cleaner way to do this without having to include all of mainwindow.h
#include "mainwindow.h"


namespace ccLibAlgorithms
{
	static QString GetDensitySFName(CCLib::GeometricalAnalysisTools::Density densityType, bool approx, double densityKernelSize = 0.0)
	{
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
		
		if (densityType != CCLib::GeometricalAnalysisTools::DENSITY_KNN)
			sfName += QString(" (r=%2)").arg(densityKernelSize);
		
		if (approx)
			sfName += " [approx]";
		
		return sfName;
	}
	
	
	PointCoordinateType GetDefaultCloudKernelSize(ccGenericPointCloud* cloud, unsigned knn/*=12*/)
	{
		assert(cloud);
		if (cloud && cloud->size() != 0)
		{
			//we get 1% of the cloud bounding box
			//and we divide by the number of points / 10e6 (so that the kernel for a 20 M. points cloud is half the one of a 10 M. cloud)
			ccBBox box = cloud->getOwnBB();
			
			//old way
			//PointCoordinateType radius = box.getDiagNorm() * static_cast<PointCoordinateType>(0.01/std::max(1.0,1.0e-7*static_cast<double>(cloud->size())));
			
			//new way
			CCVector3 d = box.getDiagVec();
			PointCoordinateType volume = d[0] * d[1] * d[2];
			PointCoordinateType surface = pow(volume, static_cast<PointCoordinateType>(2.0/3.0));
			PointCoordinateType surfacePerPoint = surface / cloud->size();
			return sqrt(surfacePerPoint * knn);
		}
		
		return -PC_ONE;
	}
	
	PointCoordinateType GetDefaultCloudKernelSize(const ccHObject::Container& entities, unsigned knn/*=12*/)
	{
		PointCoordinateType sigma = -PC_ONE;
		
		size_t selNum = entities.size();
		//computation of a first sigma guess
		for (size_t i=0; i<selNum; ++i)
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entities[i]);
			PointCoordinateType sigmaCloud = GetDefaultCloudKernelSize(pc);
			
			//we keep the smallest value
			if (sigma < 0 || sigmaCloud < sigma)
				sigma = sigmaCloud;
		}
		
		return sigma;
	}
	
	bool ApplyCCLibAlgorithm(CC_LIB_ALGORITHM algo, ccHObject::Container& entities, QWidget* parent/*=0*/, void** additionalParameters/*=0*/)
	{
		size_t selNum = entities.size();
		if (selNum < 1)
			return false;
		
		//generic parameters
		QString sfName;
		
		//computeDensity parameters
		PointCoordinateType densityKernelSize = PC_ONE;
		CCLib::GeometricalAnalysisTools::Density densityType = CCLib::GeometricalAnalysisTools::DENSITY_3D;
		
		//curvature parameters
		PointCoordinateType curvKernelSize = -PC_ONE;
		CCLib::Neighbourhood::CC_CURVATURE_TYPE curvType = CCLib::Neighbourhood::GAUSSIAN_CURV;
		
		//computeScalarFieldGradient parameters
		bool euclidean = false;
		
		//computeRoughness parameters
		PointCoordinateType roughnessKernelSize = PC_ONE;
		
		switch (algo)
		{
			case CCLIB_ALGO_APPROX_DENSITY:
			{
				//parameters already provided?
				if (additionalParameters)
				{
					densityType = *static_cast<CCLib::GeometricalAnalysisTools::Density*>(additionalParameters[0]);
				}
				else
				{
					//shouldn't happen
					assert(false);
				}
				sfName = GetDensitySFName(densityType,true);
			}
			break;
				
			case CCLIB_ALGO_ACCURATE_DENSITY:
			{
				//parameters already provided?
				if (additionalParameters)
				{
					densityKernelSize = *static_cast<PointCoordinateType*>(additionalParameters[0]);
					densityType = *static_cast<CCLib::GeometricalAnalysisTools::Density*>(additionalParameters[1]);
				}
				else //ask the user!
				{
					densityKernelSize = GetDefaultCloudKernelSize(entities);
					if (densityKernelSize < 0)
					{
						ccConsole::Error("Invalid kernel size!");
						return false;
					}
					
					ccDensityDlg dDlg(parent);
					dDlg.setRadius(densityKernelSize);
					
					//process cancelled by user
					if (!dDlg.exec())
						return false;
					
					if (!dDlg.isPrecise())
					{
						//Switch to approx density if necessary
						algo = CCLIB_ALGO_APPROX_DENSITY;
					}
					else
					{
						//otherwise read kernel size
						densityKernelSize = static_cast<PointCoordinateType>(dDlg.getRadius());
					}
					//and in both cases we need the density type
					densityType = dDlg.getDensityType();
				}
				
				sfName = GetDensitySFName(densityType,algo == CCLIB_ALGO_APPROX_DENSITY,densityKernelSize);
			}
			break;
				
			case CCLIB_ALGO_CURVATURE:
			{
				//parameters already provided?
				if (additionalParameters)
				{
					curvType = *static_cast<CCLib::Neighbourhood::CC_CURVATURE_TYPE*>(additionalParameters[0]);
					curvKernelSize = *static_cast<PointCoordinateType*>(additionalParameters[1]);
				}
				else //ask the user!
				{
					curvKernelSize = GetDefaultCloudKernelSize(entities);
					if (curvKernelSize < 0)
					{
						ccConsole::Error("Invalid kernel size!");
						return false;
					}
					ccCurvatureDlg curvDlg(0);
					if (selNum == 1)
						curvDlg.setKernelSize(curvKernelSize);
					if (!curvDlg.exec())
						return false;
					
					curvType = curvDlg.getCurvatureType();
					curvKernelSize = static_cast<PointCoordinateType>(curvDlg.getKernelSize());
				}
				
				QString fieldName;
				switch (curvType)
				{
					case CCLib::Neighbourhood::GAUSSIAN_CURV:
						fieldName = CC_CURVATURE_GAUSSIAN_FIELD_NAME;
						break;
					case CCLib::Neighbourhood::MEAN_CURV:
						fieldName = CC_CURVATURE_MEAN_FIELD_NAME;
						break;
					case CCLib::Neighbourhood::NORMAL_CHANGE_RATE:
						fieldName = CC_CURVATURE_NORM_CHANGE_RATE_FIELD_NAME;
						break;
					default:
						assert(false);
						break;
				}
				sfName = QString("%1 (%2)").arg(fieldName).arg(curvKernelSize);
			}
			break;
				
			case CCLIB_ALGO_SF_GRADIENT:
			{
				sfName = CC_GRADIENT_NORMS_FIELD_NAME;
				//parameters already provided?
				if (additionalParameters)
				{
					euclidean = *static_cast<bool*>(additionalParameters[0]);
				}
				else //ask the user!
				{
					euclidean = (	QMessageBox::question(parent,
																	 "Gradient",
																	 "Is the scalar field composed of (euclidean) distances?",
																	 QMessageBox::Yes | QMessageBox::No,
																	 QMessageBox::No ) == QMessageBox::Yes );
				}
			}
			break;
				
			case CCLIB_ALGO_ROUGHNESS:
			case CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST: //for tests: we'll use the roughness kernel for SNE
			{
				//parameters already provided?
				if (additionalParameters)
				{
					roughnessKernelSize = *static_cast<PointCoordinateType*>(additionalParameters[0]);
				}
				else //ask the user!
				{
					roughnessKernelSize = GetDefaultCloudKernelSize(entities);
					if (roughnessKernelSize < 0)
					{
						ccConsole::Error("Invalid kernel size!");
						return false;
					}
					bool ok;
					double val = QInputDialog::getDouble(parent, "Roughness", "Kernel size:", static_cast<double>(roughnessKernelSize), DBL_MIN, 1.0e9, 8, &ok);
					if (!ok)
						return false;
					roughnessKernelSize = static_cast<PointCoordinateType>(val);
				}
				
				sfName = QString(CC_ROUGHNESS_FIELD_NAME) + QString("(%1)").arg(roughnessKernelSize);
			}
			break;
				
			default:
				assert(false);
				return false;
		}
		
		for (size_t i = 0; i < selNum; ++i)
		{
			//is the ith selected data is eligible for processing?
			ccGenericPointCloud* cloud = 0;
			switch(algo)
			{
				case CCLIB_ALGO_SF_GRADIENT:
					//for scalar field gradient, we can apply it directly on meshes
					bool lockedVertices;
					cloud = ccHObjectCaster::ToGenericPointCloud(entities[i], &lockedVertices);
					if (lockedVertices)
					{
						ccUtils::DisplayLockedVerticesWarning(entities[i]->getName(), selNum == 1);
						cloud = 0;
					}
					if (cloud)
					{
						//but we need an already displayed SF!
						if (cloud->isA(CC_TYPES::POINT_CLOUD))
						{
							ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
							int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
							if (outSfIdx < 0)
							{
								cloud = 0;
							}
							else
							{
								//we set as 'output' SF the currently displayed scalar field
								pc->setCurrentOutScalarField(outSfIdx);
								sfName = QString("%1(%2)").arg(CC_GRADIENT_NORMS_FIELD_NAME).arg(pc->getScalarFieldName(outSfIdx));
							}
						}
						else //if (!cloud->hasDisplayedScalarField()) //TODO: displayed but not necessarily set as OUTPUT!
						{
							cloud = 0;
						}
					}
					break;
					
					//by default, we apply processings on clouds only
				default:
					if (entities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
						cloud = ccHObjectCaster::ToGenericPointCloud(entities[i]);
					break;
			}
			
			if (cloud)
			{
				ccPointCloud* pc = 0;
				int sfIdx = -1;
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					pc = static_cast<ccPointCloud*>(cloud);
					
					sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
					if (sfIdx < 0)
						sfIdx = pc->addScalarField(qPrintable(sfName));
					if (sfIdx >= 0)
						pc->setCurrentInScalarField(sfIdx);
					else
					{
						ccConsole::Error(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(pc->getName()));
						continue;
					}
				}
				
				ccProgressDialog pDlg(true, parent);
				
				ccOctree::Shared octree = cloud->getOctree();
				if (!octree)
				{
					pDlg.show();
					octree = cloud->computeOctree(&pDlg);
					if (!octree)
					{
						ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
						break;
					}
				}
				
				int result = 0;
				QElapsedTimer eTimer;
				eTimer.start();
				switch(algo)
				{
					case CCLIB_ALGO_APPROX_DENSITY:
						result = CCLib::GeometricalAnalysisTools::computeLocalDensityApprox(cloud,
																							densityType,
																							&pDlg,
																							octree.data());
						break;
						
					case CCLIB_ALGO_ACCURATE_DENSITY:
						result = CCLib::GeometricalAnalysisTools::computeLocalDensity(	cloud,
																						densityType,
																						densityKernelSize,
																						&pDlg,
																						octree.data());
						break;
						
					case CCLIB_ALGO_CURVATURE:
						result = CCLib::GeometricalAnalysisTools::computeCurvature(	cloud,
																					curvType,
																					curvKernelSize,
																					&pDlg,
																					octree.data());
						break;
						
					case CCLIB_ALGO_SF_GRADIENT:
						result = CCLib::ScalarFieldTools::computeScalarFieldGradient(	cloud,
																						0, //auto --> FIXME: should be properly set by the user!
																						euclidean,
																						false,
																						&pDlg,
																						octree.data());
						break;
						
					case CCLIB_ALGO_ROUGHNESS:
						result = CCLib::GeometricalAnalysisTools::computeRoughness(	cloud,
																					roughnessKernelSize,
																					&pDlg,
																					octree.data());
						break;
						
						//TEST
					case CCLIB_SPHERICAL_NEIGHBOURHOOD_EXTRACTION_TEST:
					{
						unsigned count = cloud->size();
						cloud->enableScalarField();
						{
							for (unsigned j=0; j<count; ++j)
								cloud->setPointScalarValue(j, NAN_VALUE);
						}

						QElapsedTimer subTimer;
						subTimer.start();
						unsigned long long extractedPoints = 0;
						unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(roughnessKernelSize);;
						const unsigned samples = 1000;
						std::random_device rd;   // non-deterministic generator
						std::mt19937 gen(rd());  // to seed mersenne twister.
						std::uniform_int_distribution<unsigned> dist(0, count - 1);

						for (unsigned j = 0; j < samples; ++j)
						{
							unsigned randIndex = dist(gen);
							CCLib::DgmOctree::NeighboursSet neighbours;
							octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex), roughnessKernelSize, neighbours, level);
							size_t neihgboursCount = neighbours.size();
							extractedPoints += static_cast<unsigned long long>(neihgboursCount);
							for (size_t k = 0; k < neihgboursCount; ++k)
								cloud->setPointScalarValue(neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(neighbours[k].squareDistd)));
						}
						ccConsole::Print("[SNE_TEST] Mean extraction time = %i ms (radius = %f, mean(neighbours) = %3.1f)", subTimer.elapsed(), roughnessKernelSize, static_cast<double>(extractedPoints) / static_cast<double>(samples));

						result = 0;
					}
						break;
						
					default:
						//missed something?
						assert(false);
				}
				qint64 elapsedTime_ms = eTimer.elapsed();
				
				if (result == 0)
				{
					if (pc && sfIdx >= 0)
					{
						pc->setCurrentDisplayedScalarField(sfIdx);
						pc->showSF(sfIdx >= 0);
						pc->getCurrentInScalarField()->computeMinAndMax();
					}
					cloud->prepareDisplayForRefresh();
					ccConsole::Print("[Algortihm] Timing: %3.2f s.", static_cast<double>(elapsedTime_ms) / 1000.0);
				}
				else
				{
					ccConsole::Warning(QString("Failed to apply processing to cloud '%1'").arg(cloud->getName()));
					if (pc && sfIdx >= 0)
					{
						pc->deleteScalarField(sfIdx);
						sfIdx = -1;
					}
				}
			}
		}
		
		return true;
	}
	
	
	bool ApplyScaleMatchingAlgorithm(	ScaleMatchingAlgorithm algo,
										ccHObject::Container& entities,
										double icpRmsDiff,
										int icpFinalOverlap,
										unsigned refEntityIndex/*=0*/,
										QWidget* parent/*=0*/)
	{
		if (	entities.size() < 2
			||	refEntityIndex >= entities.size())
		{
			ccLog::Error("[ApplyScaleMatchingAlgorithm] Invalid input parameter(s)");
			return false;
		}
		
		std::vector<double> scales;
		try
		{
			scales.resize(entities.size(), -1.0);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory!");
			return false;
		}
		
		//check the reference entity
		ccHObject* refEntity = entities[refEntityIndex];
		if (	!refEntity->isKindOf(CC_TYPES::POINT_CLOUD)
			&&	!refEntity->isKindOf(CC_TYPES::MESH))
		{
			ccLog::Warning("[Scale Matching] The reference entity must be a cloud or a mesh!");
			return false;
		}
		
		unsigned count = static_cast<unsigned>(entities.size());
		
		//now compute the scales
		ccProgressDialog pDlg(true,parent);
		pDlg.setMethodTitle(QObject::tr("Computing entities scales"));
		pDlg.setInfo(QObject::tr("Entities: %1").arg(count));
		CCLib::NormalizedProgress nProgress(&pDlg, 2 * count - 1);
		pDlg.start();
		QApplication::processEvents();
		
		for (unsigned i=0; i<count; ++i)
		{
			ccHObject* ent = entities[i];
			//try to get the underlying cloud (or the vertices set for a mesh)
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (cloud && !lockedVertices)
			{
				switch (algo)
				{
					case BB_MAX_DIM:
					case BB_VOLUME:
					{
						ccBBox box = ent->getOwnBB();
						if (box.isValid())
							scales[i] = algo == BB_MAX_DIM ? box.getMaxBoxDim() : box.computeVolume();
						else
							ccLog::Warning(QString("[Scale Matching] Entity '%1' has an invalid bounding-box!").arg(ent->getName()));
					}
						break;
						
					case PCA_MAX_DIM:
					{
						CCLib::Neighbourhood Yk(cloud);
						if (!Yk.getLSPlane())
						{
							ccLog::Warning(QString("[Scale Matching] Failed to perform PCA on entity '%1'!").arg(ent->getName()));
							break;
						}
						//deduce the scale
						{
							const CCVector3* X = Yk.getLSPlaneX();
							const CCVector3* O = Yk.getGravityCenter();
							double minX = 0,maxX = 0;
							for (unsigned j=0; j<cloud->size(); ++j)
							{
								double x = (*cloud->getPoint(j) - *O).dot(*X);
								if (j != 0)
								{
									minX = std::min(x,minX);
									maxX = std::max(x,maxX);
								}
								else
								{
									minX = maxX = x;
								}
							}
							scales[i] = maxX-minX;
						}
					}
					break;
						
					case ICP_SCALE:
					{
						ccGLMatrix transMat;
						double finalError = 0.0;
						double finalScale = 1.0;
						unsigned finalPointCount = 0;
						int transformationFilters = 0; //CCLib::RegistrationTools::SKIP_ROTATION;
						
						if (ccRegistrationTools::ICP(
								 ent,
								 refEntity,
								 transMat,
								 finalScale,
								 finalError,
								 finalPointCount,
								 icpRmsDiff,
								 0,
								 50000,
								 false,
								 CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE,
								 true,
								 icpFinalOverlap / 100.0,
								 false,
								 false,
								 transformationFilters,
								 0,
								 parent))
						{
							scales[i] = finalScale;
						}
						else
						{
							ccLog::Warning(QString("[Scale Matching] Failed to register entity '%1'!").arg(ent->getName()));
						}
						
					}
					break;
						
					default:
						assert(false);
						break;
				}
			}
			else if (cloud && lockedVertices)
			{
				//locked entities
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),false);
			}
			else
			{
				//we need a cloud or a mesh
				ccLog::Warning(QString("[Scale Matching] Entity '%1' can't be rescaled this way!").arg(ent->getName()));
			}
			
			//if the reference entity is invalid!
			if (scales[i] <= 0 && i == refEntityIndex)
			{
				ccLog::Error("Reference entity has an invalid scale! Can't proceed.");
				return false;
			}
			
			if (!nProgress.oneStep())
			{
				//process cancelled by user
				return false;
			}
		}
		
		ccLog::Print(QString("[Scale Matching] Reference entity scale: %1").arg(scales[refEntityIndex]));
		
		//now we can rescale
		pDlg.setMethodTitle(QObject::tr("Rescaling entities"));
		{
			for (unsigned i=0; i<count; ++i)
			{
				if (i == refEntityIndex)
					continue;
				if (scales[i] < 0)
					continue;
				
				ccLog::Print(QString("[Scale Matching] Entity '%1' scale: %2").arg(entities[i]->getName()).arg(scales[i]));
				if (scales[i] <= ZERO_TOLERANCE)
				{
					ccLog::Warning("[Scale Matching] Entity scale is too small!");
					continue;
				}
				
				ccHObject* ent = entities[i];
				
				bool lockedVertices;
				ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
				if (!cloud || lockedVertices)
					continue;
				
				double scaled = 1.0;
				if (algo == ICP_SCALE)
					scaled = scales[i];
				else
					scaled = scales[refEntityIndex] / scales[i];
				
				PointCoordinateType scale_pc = static_cast<PointCoordinateType>(scaled);
				
				//we temporarily detach entity, as it may undergo
				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::scale
				MainWindow* instance = dynamic_cast<MainWindow*>(parent);
				MainWindow::ccHObjectContext objContext;
				if (instance)
				{
					objContext = instance->removeObjectTemporarilyFromDBTree(cloud);
				}
				
				CCVector3 C = cloud->getOwnBB().getCenter();
				
				cloud->scale(	scale_pc,
								scale_pc,
								scale_pc,
								C );
				
				if (instance)
					instance->putObjectBackIntoDBTree(cloud,objContext);
				cloud->prepareDisplayForRefresh_recursive();
				
				//don't forget the 'global shift'!
				const CCVector3d& shift = cloud->getGlobalShift();
				cloud->setGlobalShift(shift*scaled);
				//DGM: nope! Not the global scale!
			}
			
			if (!nProgress.oneStep())
			{
				//process cancelled by user
				return false;
			}
		}
		
		return true;
	}
}
