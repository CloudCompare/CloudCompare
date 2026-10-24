// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include "ccLibAlgorithms.h"

// Local
#include "ccCommon.h"
#include "ccConsole.h"
#include "ccRegistrationTools.h"
#include "ccUtils.h"
// This is included only for temporarily removing an object from the tree.
//	TODO figure out a cleaner way to do this without having to include all of mainwindow.h
#include "mainwindow.h"

// CCCoreLib
#include <ScalarFieldTools.h>

// qCC_db
#include <ccBackgroundTask.h>
#include <ccOctree.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

// Qt
#include <QApplication>
#include <QElapsedTimer>
#include <QInputDialog>
#include <QMessageBox>

namespace ccLibAlgorithms
{
	static QString GetDensitySFName(CCCoreLib::GeometricalAnalysisTools::Density densityType, bool approx, double densityKernelSize = 0.0)
	{
		QString sfName;

		// update the name with the density type
		switch (densityType)
		{
		case CCCoreLib::GeometricalAnalysisTools::DENSITY_KNN:
			// in approximate mode only the nearest neighbor is extracted, so this is
			// the inverse of the distance to it and not a number of neighbors
			sfName = approx ? CC_LOCAL_NN_DISTANCE_FIELD_NAME : CC_LOCAL_KNN_DENSITY_FIELD_NAME;
			break;
		case CCCoreLib::GeometricalAnalysisTools::DENSITY_2D:
			sfName = CC_LOCAL_SURF_DENSITY_FIELD_NAME;
			break;
		case CCCoreLib::GeometricalAnalysisTools::DENSITY_3D:
			sfName = CC_LOCAL_VOL_DENSITY_FIELD_NAME;
			break;
		default:
			assert(false);
			break;
		}

		if (approx)
		{
			// the approximate density is computed from the nearest neighbor distance,
			// so there is no radius to report
			sfName += " [approx]";
		}
		else
		{
			sfName += QString(" (r=%2)").arg(densityKernelSize);
		}

		return sfName;
	}

	double GetDefaultCloudKernelSize(ccGenericPointCloud* cloud, unsigned knn /*=12*/)
	{
		assert(cloud);
		if (cloud && cloud->size() != 0)
		{
			// we get 1% of the cloud bounding box
			// and we divide by the number of points / 10e6 (so that the kernel for a 20 M. points cloud is half the one of a 10 M. cloud)
			ccBBox box = cloud->getOwnBB();

			// old way
			// PointCoordinateType radius = box.getDiagNorm() * static_cast<PointCoordinateType>(0.01/std::max(1.0,1.0e-7*static_cast<double>(cloud->size())));

			// new way
			CCVector3 d               = box.getDiagVec();
			double    volume          = (static_cast<double>(d[0]) * d[1]) * d[2];
			double    surface         = pow(volume, 2.0 / 3.0);
			double    surfacePerPoint = surface / cloud->size();
			return sqrt(surfacePerPoint * knn);
		}

		return -CCCoreLib::PC_ONE;
	}

	double GetDefaultCloudKernelSize(const ccHObject::Container& entities, unsigned knn /*=12*/)
	{
		double sigma = std::numeric_limits<double>::max();

		for (ccHObject* entity : entities)
		{
			ccPointCloud* pc         = ccHObjectCaster::ToPointCloud(entity);
			double        sigmaCloud = GetDefaultCloudKernelSize(pc);

			// we keep the smallest value
			sigma = std::min(sigmaCloud, sigma);
		}

		return sigma;
	}

	static QString GetCharacteristicSFName(CCCoreLib::GeometricalAnalysisTools::GeomCharacteristic algo,
	                                       int                                                     subOption,
	                                       PointCoordinateType                                     radius)
	{
		// generate the right SF name
		QString sfName;

		switch (algo)
		{
		case CCCoreLib::GeometricalAnalysisTools::Feature:
		{
			switch (subOption)
			{
			case CCCoreLib::Neighbourhood::EigenValuesSum:
				sfName = "Eigenvalues sum";
				break;
			case CCCoreLib::Neighbourhood::Omnivariance:
				sfName = "Omnivariance";
				break;
			case CCCoreLib::Neighbourhood::EigenEntropy:
				sfName = "Eigenentropy";
				break;
			case CCCoreLib::Neighbourhood::Anisotropy:
				sfName = "Anisotropy";
				break;
			case CCCoreLib::Neighbourhood::Planarity:
				sfName = "Planarity";
				break;
			case CCCoreLib::Neighbourhood::Linearity:
				sfName = "Linearity";
				break;
			case CCCoreLib::Neighbourhood::PCA1:
				sfName = "PCA1";
				break;
			case CCCoreLib::Neighbourhood::PCA2:
				sfName = "PCA2";
				break;
			case CCCoreLib::Neighbourhood::SurfaceVariation:
				sfName = "Surface variation";
				break;
			case CCCoreLib::Neighbourhood::Sphericity:
				sfName = "Sphericity";
				break;
			case CCCoreLib::Neighbourhood::Verticality:
				sfName = "Verticality";
				break;
			case CCCoreLib::Neighbourhood::EigenValue1:
				sfName = "1st eigenvalue";
				break;
			case CCCoreLib::Neighbourhood::EigenValue2:
				sfName = "2nd eigenvalue";
				break;
			case CCCoreLib::Neighbourhood::EigenValue3:
				sfName = "3rd eigenvalue";
				break;
			case CCCoreLib::Neighbourhood::DegreeOfPlanarity:
				sfName = "Degree of planarity (M)";
				break;
			case CCCoreLib::Neighbourhood::DegreeOfLinearity:
				sfName = "Degree of linearity (K)";
				break;
			default:
				assert(false);
				ccLog::Error("Internal error: invalid sub-option for Feature computation");
				return {};
			}
			sfName += QString(" (%1)").arg(radius);
		}
		break;

		case CCCoreLib::GeometricalAnalysisTools::Curvature:
		{
			switch (subOption)
			{
			case CCCoreLib::Neighbourhood::GAUSSIAN_CURV:
				sfName = CC_CURVATURE_GAUSSIAN_FIELD_NAME;
				break;
			case CCCoreLib::Neighbourhood::MEAN_CURV:
				sfName = CC_CURVATURE_MEAN_FIELD_NAME;
				break;
			case CCCoreLib::Neighbourhood::NORMAL_CHANGE_RATE:
				sfName = CC_CURVATURE_NORM_CHANGE_RATE_FIELD_NAME;
				break;
			default:
				assert(false);
				ccLog::Error("Internal error: invalid sub-option for Curvature computation");
				return {};
			}
			sfName += QString(" (%1)").arg(radius);
		}
		break;

		case CCCoreLib::GeometricalAnalysisTools::LocalDensity:
			sfName = GetDensitySFName(static_cast<CCCoreLib::GeometricalAnalysisTools::Density>(subOption), false, radius);
			break;

		case CCCoreLib::GeometricalAnalysisTools::ApproxLocalDensity:
			sfName = GetDensitySFName(static_cast<CCCoreLib::GeometricalAnalysisTools::Density>(subOption), true);
			break;

		case CCCoreLib::GeometricalAnalysisTools::Roughness:
			sfName = CC_ROUGHNESS_FIELD_NAME + QString(" (%1)").arg(radius);
			break;

		case CCCoreLib::GeometricalAnalysisTools::MomentOrder1:
			sfName = CC_MOMENT_ORDER1_FIELD_NAME + QString(" (%1)").arg(radius);
			break;

		default:
			assert(false);
			return {};
		}

		return sfName;
	}

	static QString ToErrorMessage(CCCoreLib::GeometricalAnalysisTools::ErrorCode result)
	{
		switch (result)
		{
		case CCCoreLib::GeometricalAnalysisTools::InvalidInput:
			return "Internal error (invalid input)";
		case CCCoreLib::GeometricalAnalysisTools::NotEnoughPoints:
			return "Not enough points";
		case CCCoreLib::GeometricalAnalysisTools::OctreeComputationFailed:
			return "Failed to compute octree (not enough memory?)";
		case CCCoreLib::GeometricalAnalysisTools::ProcessFailed:
			return "Process failed";
		case CCCoreLib::GeometricalAnalysisTools::UnhandledCharacteristic:
			return "Internal error (unhandled characteristic)";
		case CCCoreLib::GeometricalAnalysisTools::NotEnoughMemory:
			return "Not enough memory";
		case CCCoreLib::GeometricalAnalysisTools::ProcessCancelledByUser:
			return "Process cancelled by user";
		default:
			assert(false);
			break;
		}
		return "Unknown error";
	}

	bool ComputeGeomCharacteristics(const GeomCharacteristicWithSubOptionSet& characteristics,
	                                PointCoordinateType                       radius,
	                                const ccHObject::Container&               entities,
	                                const CCVector3*                          roughnessUpDir /*=nullptr*/,
	                                QWidget*                                  parent /*=nullptr*/)
	{
		size_t selNum = entities.size();
		if (selNum == 0)
		{
			return false;
		}

		// no feature case
		if (characteristics.empty())
		{
			// nothing to do
			assert(false);
			return true;
		}

		std::unique_ptr<ccProgressDialog> pDlg;
		if (!pDlg && parent)
		{
			pDlg = std::make_unique<ccProgressDialog>(true, parent);
			pDlg->setAutoClose(false);
		}

		for (size_t i = 0; i < selNum; ++i)
		{
			// is the ith selected entity eligible for processing?
			if (!entities[i]->isA(CC_TYPES::POINT_CLOUD))
			{
				continue;
			}
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entities[i]);
			assert(nullptr != pc);

			ccOctree::Shared octree = pc->getOctree();
			if (!octree)
			{
				if (pDlg)
				{
					pDlg->show();
				}
				octree = pc->computeOctree(pDlg.get());
				if (!octree)
				{
					ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
					return false;
				}
			}

			// build the set of 'requests' for the multi-characteristic computation
			CCCoreLib::GeometricalAnalysisTools::MultiGeomCharacteristicRequestSet requests;
			bool                                                                   success = true;
			for (const GeomCharacteristicWithSubOption& g : characteristics)
			{
				QString sfName = GetCharacteristicSFName(g.charac, g.subOption, radius);
				if (sfName.isEmpty())
				{
					ccConsole::Warning("Internal error");
					success = false;
					break;
				}

				int sfIdx = pc->getScalarFieldIndexByName(sfName.toStdString());
				if (sfIdx < 0)
				{
					sfIdx = pc->addScalarField(sfName.toStdString());
				}
				if (sfIdx < 0)
				{
					ccConsole::Error(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(pc->getName()));
					success = false;
					break;
				}

				// ApproxLocalDensity is not supported in multi geometric feature requests computation.
				// It's a special case that should be treated separately (see GeometricalAnalysisTools::ComputeCharacteristics)
				if (g.charac == CCCoreLib::GeometricalAnalysisTools::ApproxLocalDensity)
				{
					// ComputeCharactersitic writes in the active/current 'out' SF.
					// we must set it up before calling the method.
					pc->setCurrentScalarField(sfIdx);

					// only the computation itself runs in a worker thread
					CCCoreLib::GeometricalAnalysisTools::ErrorCode appoxDensResult = ccBackgroundTask::Run(
					    [&]()
					    {
						    return CCCoreLib::GeometricalAnalysisTools::ComputeCharactersitic(g.charac,
						                                                                      g.subOption,
						                                                                      pc,
						                                                                      radius,
						                                                                      roughnessUpDir,
						                                                                      pDlg.get(),
						                                                                      octree.data());
					    });

					if (appoxDensResult != CCCoreLib::GeometricalAnalysisTools::NoError)
					{
						QString errorMessage = ToErrorMessage(appoxDensResult);

						ccConsole::Warning(QString("Failed to apply processing to cloud '%1'").arg(pc->getName()));
						ccConsole::Warning(errorMessage);

						if (pc && sfIdx >= 0)
						{
							pc->deleteScalarField(sfIdx);
						}

						success = false;
						break;
					}

					// the approx density SF is not part of the requests and must be finalized here
					auto approxDensitySF = pc->getCCScalarField(sfIdx);
					if (approxDensitySF)
					{
						approxDensitySF->computeMinAndMax();
					}
					pc->setCurrentDisplayedScalarField(sfIdx);
					pc->showSF(true);
					continue;
				}

				// populate the request structure (except for ApproxLocalDensity which was handled separately)
				CCCoreLib::GeometricalAnalysisTools::MultiGeomCharacteristicRequest request;
				{
					request.c         = g.charac;
					request.subOption = g.subOption;
					request.sf        = (pc && sfIdx >= 0 ? pc->getCCScalarField(sfIdx).get() : nullptr);
					request.sfIdx     = sfIdx;
				}

				requests.push_back(request);
			}

			if (success)
			{
				// only the computation itself runs in a worker thread
				CCCoreLib::GeometricalAnalysisTools::ErrorCode result = ccBackgroundTask::Run(
				    [&]()
				    {
					    return CCCoreLib::GeometricalAnalysisTools::ComputeCharacteristics(requests,
					                                                                       pc,
					                                                                       radius,
					                                                                       roughnessUpDir,
					                                                                       pDlg.get(),
					                                                                       octree.data());
				    });

				if (result == CCCoreLib::GeometricalAnalysisTools::NoError)
				{
					// update all scalar fields
					int lastValidSFIdnex = -1;
					for (const auto& request : requests)
					{
						if (pc && request.sfIdx >= 0)
						{
							// the SF was written directly (not through the 'in/out' SF of the cloud)
							// so we retrieve it by its index
							auto sf = pc->getCCScalarField(request.sfIdx);
							if (sf)
							{
								lastValidSFIdnex = request.sfIdx;
								sf->computeMinAndMax();
								if (request.c == CCCoreLib::GeometricalAnalysisTools::Roughness && roughnessUpDir != nullptr)
								{
									// signed roughness should be displayed with a symmetrical color scale
									sf->setSymmetricalScale(true);
								}
							}
							else
							{
								assert(false);
							}
						}
					}

					// display the last valid SF (if any)
					if (lastValidSFIdnex >= 0)
					{
						pc->setCurrentDisplayedScalarField(lastValidSFIdnex);
						pc->showSF(true);
					}
					pc->prepareDisplayForRefresh();
				}
				else
				{
					ccConsole::Warning(QString("Failed to apply processing to cloud '%1'").arg(pc->getName()));
					ccConsole::Warning(ToErrorMessage(result));
					success = false;
				}
			}

			if (!success)
			{
				// early stop: we remove all the SFs that were created for this cloud
				for (const auto& request : requests)
				{
					if (pc && request.sfIdx >= 0)
					{
						pc->deleteScalarField(request.sfIdx);
					}
				}
				return false;
			}
		}

		return true;
	}

	bool ApplyCCLibAlgorithm(CC_LIB_ALGORITHM algo, ccHObject::Container& entities, QWidget* parent /*=nullptr*/, void** additionalParameters /*=nullptr*/)
	{
		size_t selNum = entities.size();
		if (selNum < 1)
			return false;

		// generic parameters
		QString sfName;

		// computeScalarFieldGradient parameters
		bool euclidean = false;

		switch (algo)
		{
		case CCLIB_ALGO_SF_GRADIENT:
		{
			sfName = CC_GRADIENT_NORMS_FIELD_NAME;
			// parameters already provided?
			if (additionalParameters)
			{
				euclidean = *static_cast<bool*>(additionalParameters[0]);
			}
			else // ask the user!
			{
				euclidean = (QMessageBox::question(parent,
				                                   "Gradient",
				                                   "Is the scalar field composed of (euclidean) distances?",
				                                   QMessageBox::Yes | QMessageBox::No,
				                                   QMessageBox::No)
				             == QMessageBox::Yes);
			}
		}
		break;

		default:
			assert(false);
			return false;
		}

		for (size_t i = 0; i < selNum; ++i)
		{
			// is the ith selected data is eligible for processing?
			ccGenericPointCloud* cloud = nullptr;
			switch (algo)
			{
			case CCLIB_ALGO_SF_GRADIENT:
			{
				// for scalar field gradient, we can apply it directly on meshes
				bool lockedVertices = false;
				cloud               = ccHObjectCaster::ToGenericPointCloud(entities[i], &lockedVertices);
				if (lockedVertices)
				{
					ccUtils::DisplayLockedVerticesWarning(entities[i]->getName(), selNum == 1);
					cloud = nullptr;
				}
				if (cloud)
				{
					// but we need an already displayed SF!
					if (cloud->isA(CC_TYPES::POINT_CLOUD))
					{
						ccPointCloud* pc       = static_cast<ccPointCloud*>(cloud);
						int           outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
						if (outSfIdx < 0)
						{
							cloud = nullptr;
						}
						else
						{
							// we set as 'output' SF the currently displayed scalar field
							pc->setCurrentOutScalarField(outSfIdx);
							sfName = QString("%1(%2)").arg(CC_GRADIENT_NORMS_FIELD_NAME, QString::fromStdString(pc->getScalarFieldName(outSfIdx)));
						}
					}
					else // if (!cloud->hasDisplayedScalarField()) //TODO: displayed but not necessarily set as OUTPUT!
					{
						cloud = nullptr;
					}
				}
			}
			break;

			// by default, we apply processings on clouds only
			default:
				if (entities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
				{
					cloud = ccHObjectCaster::ToGenericPointCloud(entities[i]);
				}
				break;
			}

			if (cloud)
			{
				ccPointCloud* pc    = nullptr;
				int           sfIdx = -1;
				if (cloud->isA(CC_TYPES::POINT_CLOUD))
				{
					pc = static_cast<ccPointCloud*>(cloud);

					sfIdx = pc->getScalarFieldIndexByName(sfName.toStdString());
					if (sfIdx < 0)
						sfIdx = pc->addScalarField(sfName.toStdString());
					if (sfIdx >= 0)
						pc->setCurrentInScalarField(sfIdx);
					else
					{
						ccConsole::Error(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(pc->getName()));
						continue;
					}
				}

				std::unique_ptr<ccProgressDialog> pDlg;
				if (parent)
				{
					pDlg = std::make_unique<ccProgressDialog>(true, parent);
				}

				ccOctree::Shared octree = cloud->getOctree();
				if (!octree)
				{
					if (pDlg)
					{
						pDlg->show();
					}
					octree = cloud->computeOctree(pDlg.get());
					if (!octree)
					{
						ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
						break;
					}
				}

				int           result = 0;
				QElapsedTimer eTimer;
				eTimer.start();
				switch (algo)
				{
				case CCLIB_ALGO_SF_GRADIENT:
					// only the computation itself runs in a worker thread
					result = ccBackgroundTask::Run(
					    [&]()
					    {
						    return CCCoreLib::ScalarFieldTools::computeScalarFieldGradient(cloud,
						                                                                   0, // auto --> FIXME: should be properly set by the user!
						                                                                   euclidean,
						                                                                   false,
						                                                                   pDlg.get(),
						                                                                   octree.data());
					    });
					break;

				default:
					// missed something?
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
					ccConsole::Printf("[Algorithm] Timing: %3.2f s.", elapsedTime_ms / 1000.0);
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

	bool ApplyScaleMatchingAlgorithm(ScaleMatchingAlgorithm algo,
	                                 ccHObject::Container&  entities,
	                                 double                 icpRmsDiff,
	                                 int                    icpFinalOverlap,
	                                 unsigned               refEntityIndex /*=0*/,
	                                 QWidget*               parent /*=nullptr*/,
	                                 double                 minScale /*=std::numeric_limits<double>::quiet_NaN()*/,
	                                 double                 maxScale /*=std::numeric_limits<double>::quiet_NaN()*/)
	{
		if (entities.size() < 2
		    || refEntityIndex >= entities.size())
		{
			ccLog::Error("[ApplyScaleMatchingAlgorithm] Invalid input parameter(s)");
			return false;
		}

		if (std::isfinite(minScale) && minScale < 0.0)
		{
			ccLog::Warning("Minimum scale should not be negative");
			minScale = 0.0;
		}
		if (std::isfinite(maxScale) && maxScale < 0.0)
		{
			ccLog::Warning("Maximum scale should not be negative");
			maxScale = 0.0;
		}

		if (std::isfinite(minScale) && std::isfinite(maxScale) && minScale > maxScale)
		{
			ccLog::Error("Maximum scale should not be smaller than minimum scale");
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

		// check the reference entity
		ccHObject* refEntity = entities[refEntityIndex];
		if (!refEntity->isKindOf(CC_TYPES::POINT_CLOUD)
		    && !refEntity->isKindOf(CC_TYPES::MESH))
		{
			ccLog::Warning("[Scale Matching] The reference entity must be a cloud or a mesh!");
			return false;
		}

		unsigned count = static_cast<unsigned>(entities.size());

		// now compute the scales
		std::unique_ptr<ccProgressDialog> pDlg(nullptr);
		if (parent)
		{
			pDlg = std::make_unique<ccProgressDialog>(true, parent);
			pDlg->setMethodTitle(QObject::tr("Computing entities scales"));
			pDlg->setInfo(QObject::tr("Entities: %1").arg(count));
		}
		CCCoreLib::NormalizedProgress nProgress(pDlg.get(), 2 * count - 1);
		if (pDlg)
		{
			pDlg->start();
			QApplication::processEvents();
		}

		for (unsigned i = 0; i < count; ++i)
		{
			ccHObject* ent = entities[i];
			// try to get the underlying cloud (or the vertices set for a mesh)
			bool                 lockedVertices = false;
			ccGenericPointCloud* cloud          = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);

			if (nullptr == cloud)
			{
				// we need a cloud or a mesh
				ccLog::Warning(QString("[Scale Matching] Entity '%1' can't be rescaled this way!").arg(ent->getName()));
			}
			else if (lockedVertices)
			{
				// locked entities
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), false);
			}
			else
			{
				switch (algo)
				{
				case BB_MAX_DIM:
				case BB_VOLUME:
				{
					ccBBox box = ent->getOwnBB();
					if (box.isValid())
					{
						scales[i] = algo == BB_MAX_DIM ? box.getMaxBoxDim() : box.computeVolume();
					}
					else
					{
						ccLog::Warning(QString("[Scale Matching] Entity '%1' has an invalid bounding-box!").arg(ent->getName()));
					}
				}
				break;

				case PCA_MAX_DIM:
				{
					CCCoreLib::Neighbourhood Yk(cloud);
					if (!Yk.getLSPlane())
					{
						ccLog::Warning(QString("[Scale Matching] Failed to perform PCA on entity '%1'!").arg(ent->getName()));
						break;
					}
					// deduce the scale
					{
						const CCVector3* X    = Yk.getLSPlaneX();
						const CCVector3* O    = Yk.getGravityCenter();
						double           minX = 0;
						double           maxX = 0;
						for (unsigned j = 0; j < cloud->size(); ++j)
						{
							double x = (*cloud->getPoint(j) - *O).dot(*X);
							if (j != 0)
							{
								minX = std::min(x, minX);
								maxX = std::max(x, maxX);
							}
							else
							{
								minX = maxX = x;
							}
						}
						scales[i] = maxX - minX;
					}
				}
				break;

				case ICP_SCALE:
				{
					ccGLMatrix transMat;
					double     finalError      = 0.0;
					double     finalScale      = 1.0;
					unsigned   finalPointCount = 0;

					CCCoreLib::ICPRegistrationTools::Parameters parameters;
					{
						parameters.convType                 = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
						parameters.minRMSDecrease           = icpRmsDiff;
						parameters.nbMaxIterations          = 0;
						parameters.adjustScale              = true;
						parameters.filterOutFarthestPoints  = false;
						parameters.samplingLimit            = 50000;
						parameters.finalOverlapRatio        = icpFinalOverlap / 100.0;
						parameters.transformationFilters    = CCCoreLib::RegistrationTools::SKIP_NONE;
						parameters.maxThreadCount           = 0;
						parameters.useC2MSignedDistances    = false;
						parameters.robustC2MSignedDistances = true;
						parameters.normalsMatching          = CCCoreLib::ICPRegistrationTools::NO_NORMAL;
						parameters.minScale                 = minScale;
						parameters.maxScale                 = maxScale;
					}

					if (ccRegistrationTools::ICP(
					        ent,
					        refEntity,
					        transMat,
					        finalScale,
					        finalError,
					        finalPointCount,
					        parameters,
					        false,
					        false,
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

			// if the reference entity is invalid!
			if (scales[i] <= 0 && i == refEntityIndex)
			{
				ccLog::Error("Reference entity has an invalid scale! Can't proceed.");
				return false;
			}

			if (!nProgress.oneStep())
			{
				// process cancelled by user
				return false;
			}
		}

		ccLog::Print(QString("[Scale Matching] Reference entity scale: %1").arg(scales[refEntityIndex]));

		// now we can rescale
		if (pDlg)
		{
			pDlg->setMethodTitle(QObject::tr("Rescaling entities"));
		}

		for (unsigned i = 0; i < count; ++i)
		{
			if (i == refEntityIndex)
			{
				continue;
			}
			if (scales[i] < 0)
			{
				continue;
			}

			ccLog::Print(QString("[Scale Matching] Entity '%1' scale: %2").arg(entities[i]->getName()).arg(scales[i]));
			if (scales[i] <= CCCoreLib::ZERO_TOLERANCE_D)
			{
				ccLog::Warning("[Scale Matching] Entity scale is too small!");
				continue;
			}

			ccHObject* ent = entities[i];

			bool                 lockedVertices = false;
			ccGenericPointCloud* cloud          = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			if (nullptr == cloud || lockedVertices)
			{
				continue;
			}

			double scaled = 1.0;
			if (algo == ICP_SCALE)
			{
				scaled = scales[i]; // already clamped normally
				assert(!std::isfinite(minScale) || scaled >= minScale);
				assert(!std::isfinite(maxScale) || scaled <= maxScale);
			}
			else
			{
				scaled = scales[refEntityIndex] / scales[i];

				// the caller can restrict the scale factor that may be applied
				if (std::isfinite(minScale) && scaled < minScale)
				{
					ccLog::Warning(QString("[Scale Matching] Entity '%1' scale factor (%2) was clamped to the minimum allowed value (%3)").arg(entities[i]->getName()).arg(scaled).arg(minScale));
					scaled = minScale;
				}
				else if (std::isfinite(maxScale) && scaled > maxScale)
				{
					ccLog::Warning(QString("[Scale Matching] Entity '%1' scale factor (%2) was clamped to the maximum allowed value (%3)").arg(entities[i]->getName()).arg(scaled).arg(maxScale));
					scaled = maxScale;
				}
			}

			PointCoordinateType scale_pc = static_cast<PointCoordinateType>(scaled);

			// we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::scale
			MainWindow*                  instance = dynamic_cast<MainWindow*>(parent);
			MainWindow::ccHObjectContext objContext;
			if (instance)
			{
				objContext = instance->removeObjectTemporarilyFromDBTree(cloud);
			}

			CCVector3 C = cloud->getOwnBB().getCenter();

			cloud->scale(scale_pc,
			             scale_pc,
			             scale_pc,
			             C);

			if (instance)
			{
				instance->putObjectBackIntoDBTree(cloud, objContext);
			}
			cloud->prepareDisplayForRefresh_recursive();

			// don't forget the 'global shift'!
			const CCVector3d& shift = cloud->getGlobalShift();
			cloud->setGlobalShift(shift * scaled);
			// DGM: nope! Not the global scale!
		}

		return nProgress.oneStep();
	}
} // namespace ccLibAlgorithms
