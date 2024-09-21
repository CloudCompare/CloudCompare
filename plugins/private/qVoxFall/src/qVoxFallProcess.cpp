//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#include "qVoxFallProcess.h"

//system
#include <unordered_set>

//local
#include "qVoxFallDialog.h"
#include "qVoxFallTools.h"

//CCCoreLib
#include <CloudSamplingTools.h>
#include "Grid3D.h"

//qCC_plugins
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccGenericPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccHObjectCaster.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QElapsedTimer>
#include <QtConcurrentMap>
#include <QMessageBox>

#if defined(_OPENMP)
//OpenMP
#include <omp.h>
#endif
using namespace CCCoreLib;


//! Default name for VoxFall scalar fields
static const char OCCUPANCY_SF_NAME[] = "Occupancy";
static const char CLUSTER_SF_NAME[] = "Cluster ID";
static const char CHANGE_TYPE_SF_NAME[] = "Loss/gain";
static const char VOLUME_SF_NAME[] = "Volume (m3)";
static const char UNCERTAINTY_SF_NAME[] = "Uncertainty (%)";


// Structure for parallel call
struct VoxFallParams
{
	//main options
	float voxelSize = 0;
	int clusterLabel = 0;
	int currentLabel;
	int changeType;
	bool exportBlocksAsMeshes = false;
	bool exportLossGain = false;
	CCVector3 minBound, maxBound, extent, steps;

	//helpers
	std::vector<std::vector<int>> nbs;
	std::vector<bool> isEmpty;
	std::vector<bool> isEmptyBefore;
	std::vector<bool> nonEmptyVoxelsVisited;
	std::vector<int> clusters;
	int emptyVoxelCount = 0;
	CCVector3 centroid;
	CCVector3 bbDims;
	std::vector<float> volumes;
	std::vector<unsigned int> clusterIndices;
	int clusterOutterVoxelCount;

	//export
	ccPointCloud* voxfall = nullptr;

	//scalar fields
	ccScalarField* clusterSF = nullptr;			//cluster ID
	ccScalarField* changeTypeSF = nullptr;		//loss or gain
	ccScalarField* volumeSF = nullptr;			//block volume
	ccScalarField* uncertaintySF = nullptr;		//volume uncertainty

	//progress notification
	CCCoreLib::NormalizedProgress* nProgress = nullptr;
	bool processCanceled = false;
	bool processFailed = false;
};
static VoxFallParams s_VoxFallParams;


bool InitializeOutputCloud(int voxelCount, GenericProgressCallback* progressCb = nullptr)
{
	//progress notification
	NormalizedProgress nProgress(progressCb, voxelCount);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo("Initialization");
			progressCb->setMethodTitle("VoxFall Detection");
		}
		progressCb->update(0);
		progressCb->start();
	}

	float voxelSize = s_VoxFallParams.voxelSize;
	CCVector3 minBound = s_VoxFallParams.minBound;

	for (int index = 0; index < voxelCount; ++index)
	{
		Tuple3i V = qVoxFallTools::Index2Grid(index, s_VoxFallParams.steps);
		CCVector3 P(static_cast<PointCoordinateType>(V.x * voxelSize + minBound.x),
					static_cast<PointCoordinateType>(V.y * voxelSize + minBound.y),
					static_cast<PointCoordinateType>(V.z * voxelSize + minBound.z));
		s_VoxFallParams.voxfall->addPoint(P);

		//progress bar
		if (progressCb && !nProgress.oneStep())
		{
			return false;
		}
	}
		
	return true;
}


void GetVoxelOccupancy(const Tuple3i& cellPos, unsigned n)
{
	int index = qVoxFallTools::Grid2Index(cellPos, s_VoxFallParams.steps);
	s_VoxFallParams.isEmpty[index] = false;
}


void GetVoxelOccupancyBefore(const Tuple3i& cellPos, unsigned n)
{
	int index = qVoxFallTools::Grid2Index(cellPos, s_VoxFallParams.steps);
	s_VoxFallParams.isEmptyBefore[index] = false;
}


bool ClusterEmptySpace(int maxThreads, int voxelCount, GenericProgressCallback* progressCb = nullptr)
{
	//progress notification
	NormalizedProgress nProgress(progressCb, voxelCount);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			char buffer[64];
			snprintf(buffer, 64, "Clustering empty space \n Voxels: %u", voxelCount);
			progressCb->setInfo(buffer);
			progressCb->setMethodTitle("VoxFall Detection");
		}
		progressCb->update(0);
		progressCb->start();
	}

	auto steps = s_VoxFallParams.steps;
	s_VoxFallParams.nbs.resize(voxelCount);
#if defined(_OPENMP)
#pragma omp parallel for schedule(static) \
        num_threads(maxThreads)
#endif
	for (int index = 0; index < voxelCount; ++index) {
		auto V = qVoxFallTools::Index2Grid(index, steps);
		auto NN = qVoxFallTools::FindAdjacents(V, steps, false);
		for (auto const& n : NN)
		{
			int nIdx = qVoxFallTools::Grid2Index(n, steps);
			s_VoxFallParams.nbs[index].push_back(nIdx);
		}
#if defined(_OPENMP)
#pragma omp critical(ClusterEmptySpace)
		{ nProgress.oneStep(); }
#endif
	}
	for (int index = 0; index < voxelCount; ++index)
	{
		// Check if voxel is empty.
		if (!s_VoxFallParams.isEmpty[index])
			continue;

		// Label is not undefined.
		if (s_VoxFallParams.clusterSF->getValue(index) != -1)
		{
			continue;
		}

		// Check density.
		int nCount = 0;
		for (auto const& n : s_VoxFallParams.nbs[index])
		{
			if (s_VoxFallParams.isEmpty[n])
			{
				nCount++;
			}
		}

		std::unordered_set<unsigned int> nbs_next(s_VoxFallParams.nbs[index].begin(), s_VoxFallParams.nbs[index].end());
		std::unordered_set<unsigned int> visited;
		visited.insert(index);

		s_VoxFallParams.clusterSF->setValue(index, static_cast<ScalarType>(s_VoxFallParams.clusterLabel));
		if (s_VoxFallParams.clusterLabel > 0)	// keep track of the total voxels included in volumes
		{
			s_VoxFallParams.emptyVoxelCount++;
		}
		if (progressCb && !nProgress.oneStep())		//progress bar
		{
			return false;
		}
		while (!nbs_next.empty())
		{
			unsigned nb = *nbs_next.begin();
			nbs_next.erase(nbs_next.begin());
			// Check empty neighbor.
			if (!s_VoxFallParams.isEmpty[nb])
			{
				continue;
			}
			visited.insert(nb);

			// Not undefined label.
			if (s_VoxFallParams.clusterSF->getValue(nb) != -1)
			{
				continue;
			}
			s_VoxFallParams.clusterSF->setValue(nb, static_cast<ScalarType>(s_VoxFallParams.clusterLabel));
			if (s_VoxFallParams.clusterLabel > 0)	// keep track of the total voxels included in volumes
			{
				s_VoxFallParams.emptyVoxelCount++;
			}
			if (progressCb && !nProgress.oneStep())		//progress bar
			{
				return false;
			}

			// Get neighbor's density.
			int nCount = 0;
			for (auto const& n : s_VoxFallParams.nbs[nb])
			{
				if (s_VoxFallParams.isEmpty[n])
				{
					nCount++;
				}
			}
			if (nCount >= 1)
			{
				for (int qnb : s_VoxFallParams.nbs[nb])
				{
					if (s_VoxFallParams.isEmpty[qnb])
					{
						if (visited.count(qnb) == 0)
						{
							nbs_next.insert(qnb);
						}
					}
				}
			}
		}
		s_VoxFallParams.clusterLabel++;
	}
	return true;
}


bool ComputeClusterVolume(int maxThreads, int clusterCount, ccHObject* clusterGroup = nullptr)
{

	bool error = false;
	CCVector3 minBound = s_VoxFallParams.maxBound;
	CCVector3 maxBound = s_VoxFallParams.minBound;
	int count = 0;

	if (s_VoxFallParams.processCanceled)
		return error;

#if defined(_OPENMP)
#pragma omp parallel for schedule(static) \
        num_threads(maxThreads)
#endif
	for (int i = 0; i < clusterCount; i++)
	{
		int index = s_VoxFallParams.clusterIndices[i];

		std::unordered_set<unsigned int> nbs_next(s_VoxFallParams.nbs[index].begin(), s_VoxFallParams.nbs[index].end());
		while (!nbs_next.empty())
		{
			unsigned nb = *nbs_next.begin();
			nbs_next.erase(nbs_next.begin());

			// Check non empty neighbor.
			if (s_VoxFallParams.isEmpty[nb])
			{
				continue;
			}
			if (s_VoxFallParams.nonEmptyVoxelsVisited[nb] == false)
			{
				s_VoxFallParams.clusterOutterVoxelCount++;
				s_VoxFallParams.nonEmptyVoxelsVisited[nb] = true;

				if (s_VoxFallParams.exportLossGain)
				{
					Tuple3i V = qVoxFallTools::Index2Grid(nb, s_VoxFallParams.steps);
					CCVector3 voxel(static_cast<PointCoordinateType>(V.x * s_VoxFallParams.voxelSize + s_VoxFallParams.minBound.x),
						static_cast<PointCoordinateType>(V.y * s_VoxFallParams.voxelSize + s_VoxFallParams.minBound.y),
						static_cast<PointCoordinateType>(V.z * s_VoxFallParams.voxelSize + s_VoxFallParams.minBound.z));

					if (voxel.x > maxBound.x) maxBound.x = static_cast<PointCoordinateType>(voxel.x);
					if (voxel.y > maxBound.y) maxBound.y = static_cast<PointCoordinateType>(voxel.y);
					if (voxel.z > maxBound.z) maxBound.z = static_cast<PointCoordinateType>(voxel.z);

					if (voxel.x < minBound.x) minBound.x = static_cast<PointCoordinateType>(voxel.x);
					if (voxel.y < minBound.y) minBound.y = static_cast<PointCoordinateType>(voxel.y);
					if (voxel.z < minBound.z) minBound.z = static_cast<PointCoordinateType>(voxel.z);
				}

			}
			if (s_VoxFallParams.exportBlocksAsMeshes)
			{
				s_VoxFallParams.clusters[nb] = s_VoxFallParams.currentLabel;
			}
		}

		//progress bar
		if (!s_VoxFallParams.nProgress->oneStep())
		{
			error = true;
			break;
		}
		if (error) break;
	}

	if (s_VoxFallParams.exportLossGain)
	{
		float ymin = minBound.y;
		float ymax = maxBound.y;
		CCVector3 extent = maxBound - minBound;
		CCVector3 center = minBound + extent / 2;
		minBound += extent / 2 * 0.9;
		maxBound -= extent / 2 * 0.9;
		maxBound.y = ymax + (ymax - ymin) / 2.0;

		s_VoxFallParams.centroid = minBound + (maxBound - minBound) / 1.5;
		s_VoxFallParams.bbDims = (maxBound - minBound) / 2;
	}

	if (error) return !error;
	return !error;
}


bool qVoxFallProcess::Compute(const qVoxFallDialog& dlg, QString& errorMessage, bool allowDialogs, QWidget* parentWidget/*=nullptr*/, ccMainAppInterface* app/*=nullptr*/)
{
	errorMessage.clear();

	//get the input meshes in the right order
	ccMesh* mesh1 = dlg.getMesh1();
	ccMesh* mesh2 = dlg.getMesh2();

	if (!mesh1 || !mesh2)
	{
		assert(false);
		return false;
	}

	//get parameters from dialog
	double azimuth = dlg.getAzimuth();

	//max thread count
	int maxThreadCount = dlg.getMaxThreadCount();

	if (app)
		app->dispToConsole(	QString("[VoxFall] Will use %1 threads").arg(maxThreadCount == 0 ? "the max number of" : QString::number(maxThreadCount)),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);

	//progress dialog
	ccProgressDialog pDlg(parentWidget);

	//Duration: initialization
	QElapsedTimer initTimer;
	initTimer.start();

	auto mesh = mesh1->cloneMesh();
	mesh->merge(mesh2, false);

	auto transform = qVoxFallTransform(azimuth);
	mesh->applyGLTransformation_recursive(transform.matrix);
	mesh1->applyGLTransformation_recursive(transform.matrix);

	mesh1->setEnabled(false);

	//parameters are stored in 's_VoxFallParams' for parallel call
	s_VoxFallParams = VoxFallParams();
	s_VoxFallParams.voxelSize = dlg.getVoxelSize();
	s_VoxFallParams.minBound = mesh->getOwnBB().minCorner();
	s_VoxFallParams.maxBound = mesh->getOwnBB().maxCorner();
	s_VoxFallParams.extent = s_VoxFallParams.maxBound - s_VoxFallParams.minBound;
	s_VoxFallParams.steps = (s_VoxFallParams.extent / s_VoxFallParams.voxelSize) + Vector3Tpl<float>(1, 1, 1);
	s_VoxFallParams.exportBlocksAsMeshes = dlg.getExportMeshesActivation();
	s_VoxFallParams.exportLossGain = dlg.getLossGainActivation();
	s_VoxFallParams.voxfall = new ccPointCloud(mesh1->getName() + "_to_" + mesh2->getName() + QString(" [VoxFall grid] (voxel %1 m)").arg(s_VoxFallParams.voxelSize));

	//Initialize voxel grid
	auto voxelGrid = CCCoreLib::Grid3D<int>();
	if (!voxelGrid.init(	int(s_VoxFallParams.steps.x),
							int(s_VoxFallParams.steps.y),
							int(s_VoxFallParams.steps.z),
							0	))  //margin
	{
		errorMessage = "Failed to initialize voxel grid!";
		return false;
	}

	// Initialize heplpers
	s_VoxFallParams.voxfall->reserve(voxelGrid.innerCellCount());
	s_VoxFallParams.nbs.resize(voxelGrid.innerCellCount());
	s_VoxFallParams.isEmpty.resize(voxelGrid.innerCellCount(), true);
	s_VoxFallParams.isEmptyBefore.resize(voxelGrid.innerCellCount(), true);
	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		s_VoxFallParams.clusters.resize(voxelGrid.innerCellCount(), NULL);
	}

	//allocate cluster ID SF
	s_VoxFallParams.clusterSF = new ccScalarField(CLUSTER_SF_NAME);
	s_VoxFallParams.clusterSF->link();
	if (!s_VoxFallParams.clusterSF->resizeSafe(voxelGrid.innerCellCount(), true, static_cast<ScalarType>(-1.0)))
	{
		errorMessage = "Failed to allocate memory for cluster ID values!";
		return false;
	}

	if (s_VoxFallParams.exportLossGain)
	{
		//allocate change type SF
		s_VoxFallParams.changeTypeSF = new ccScalarField(CHANGE_TYPE_SF_NAME);
		s_VoxFallParams.changeTypeSF->link();
		if (!s_VoxFallParams.changeTypeSF->resizeSafe(voxelGrid.innerCellCount(), true, CCCoreLib::NAN_VALUE))
		{
			errorMessage = "Failed to allocate memory for change type values!";
			return false;
		}
	}
	//allocate volume SF
	s_VoxFallParams.volumeSF = new ccScalarField(VOLUME_SF_NAME);
	s_VoxFallParams.volumeSF->link();
	if (!s_VoxFallParams.volumeSF->resizeSafe(voxelGrid.innerCellCount(), true, CCCoreLib::NAN_VALUE))
	{
		errorMessage = "Failed to allocate memory for volume values!";
		return false;
	}
	//allocate volume uncertainty SF
	s_VoxFallParams.uncertaintySF = new ccScalarField(UNCERTAINTY_SF_NAME);
	s_VoxFallParams.uncertaintySF->link();
	if (!s_VoxFallParams.uncertaintySF->resizeSafe(voxelGrid.innerCellCount(), true, CCCoreLib::NAN_VALUE))
	{
		errorMessage = "Failed to allocate memory for volume uncertainty values!";
		return false;
	}

	// Initialize output cloud
	if (!InitializeOutputCloud(voxelGrid.innerCellCount(), &pDlg))
	{
		errorMessage = "Failed to initialize output data!";
		return false;
	}

	qint64 initTime_ms = initTimer.elapsed();
	//we display init. timing only if no error occurred!
	if (app)
		app->dispToConsole(	QString("[VoxFall] Initialization: %1 s").arg(initTime_ms / 1000.0, 0, 'f', 3),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);


// 	   BLOCK DETECTION
//=======================================================================================================================

	//Duration: Detection
	QElapsedTimer detectTimer;
	detectTimer.start();

	if (!voxelGrid.intersectWith(	mesh,
									s_VoxFallParams.voxelSize,
									s_VoxFallParams.minBound,
									GetVoxelOccupancy,
									&pDlg	))
	{
		errorMessage = "Failed to compute  grid occupancy!";
		return false;
	}


	if (s_VoxFallParams.exportLossGain)
	{
		if (!voxelGrid.intersectWith(mesh1,
			s_VoxFallParams.voxelSize,
			s_VoxFallParams.minBound,
			GetVoxelOccupancyBefore,
			&pDlg))
		{
			errorMessage = "Failed to compute  grid occupancy!";
			return false;
		}
	}

	//cluster DBSCAN
	if (!ClusterEmptySpace(	maxThreadCount,
							voxelGrid.innerCellCount(),
							&pDlg	))
	{
		errorMessage = "Failed to compute grid occupancy!";
		return false;
	}

	qint64 detectTime_ms = detectTimer.elapsed();
	//we display block extraction timing only if no error occurred!
	if (app)
		app->dispToConsole(QString("[VoxFall] Block detection: %1 s").arg(detectTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
		app->dispToConsole(	QString("[VoxFall] Blocks found: %1").arg(s_VoxFallParams.clusterLabel - 1),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);

// 	   COMPUTE VOLUMES
//=======================================================================================================================

	//Duration: volume computation
	QElapsedTimer volumeTimer;
	volumeTimer.start();

	//progress notification
	pDlg.reset();
	NormalizedProgress nProgress(&pDlg, s_VoxFallParams.emptyVoxelCount);
	char buffer[64];
	snprintf(buffer, 64, "VoxFall clusters: %u \n Empty voxels: %u", s_VoxFallParams.clusterLabel - 1, s_VoxFallParams.emptyVoxelCount);
	pDlg.setInfo(buffer);
	pDlg.setMethodTitle(QObject::tr("Compute Volumes"));
	pDlg.update(0);
	pDlg.start();
	s_VoxFallParams.nProgress = &nProgress;


	s_VoxFallParams.volumes.reserve(s_VoxFallParams.clusterLabel);
	s_VoxFallParams.nonEmptyVoxelsVisited.resize(voxelGrid.innerCellCount(), false);
	for (unsigned label = 1; label < s_VoxFallParams.clusterLabel; ++label)
	{
		auto it = std::find(s_VoxFallParams.clusterSF->begin(), s_VoxFallParams.clusterSF->end(), label);
		while (it != s_VoxFallParams.clusterSF->end())
		{
			s_VoxFallParams.clusterIndices.push_back(it - s_VoxFallParams.clusterSF->begin());
			it = std::find(it + 1, s_VoxFallParams.clusterSF->end(), label);
		}

		s_VoxFallParams.currentLabel = label;
		s_VoxFallParams.clusterOutterVoxelCount = 0;


		if (!ComputeClusterVolume(	maxThreadCount, s_VoxFallParams.clusterIndices.size() ))
		{
			errorMessage = "Failed to compute cluster volume!";
			return false;
		}

		if (s_VoxFallParams.exportLossGain)
		{
			int count = 0;
			mesh1->placeIteratorAtBeginning();
			for (unsigned n = 0; n < mesh1->size(); n++)
			{
				//get the positions (in the grid) of each vertex
				const GenericTriangle* T = mesh1->_getNextTriangle();

				//current triangle vertices
				const CCVector3* triPoints[3]{ T->_getA(), T->_getB(), T->_getC() };

				if (CCMiscTools::TriBoxOverlap(s_VoxFallParams.centroid, s_VoxFallParams.bbDims, triPoints))
				{
					count++;
				}
			}
			if (count > 0)
			{
				s_VoxFallParams.changeType = -1;
			}
			else
			{
				s_VoxFallParams.changeType = 1;
			}
		}
		ScalarType changeType = static_cast<ScalarType>(s_VoxFallParams.changeType);
		ScalarType uncertainty = static_cast<ScalarType>(pow(s_VoxFallParams.voxelSize, 3) * s_VoxFallParams.clusterOutterVoxelCount / 2);
		ScalarType volume = static_cast<ScalarType>(pow(s_VoxFallParams.voxelSize, 3) * s_VoxFallParams.clusterIndices.size() + uncertainty);
		s_VoxFallParams.volumes[label - 1] = static_cast<float>(volume);

		for (unsigned i = 0; i < s_VoxFallParams.clusterIndices.size(); i++)
		{
			if (s_VoxFallParams.exportLossGain)
			{
				s_VoxFallParams.changeTypeSF->setValue(s_VoxFallParams.clusterIndices[i], changeType);
			}
			s_VoxFallParams.volumeSF->setValue(s_VoxFallParams.clusterIndices[i], volume);
			s_VoxFallParams.uncertaintySF->setValue(s_VoxFallParams.clusterIndices[i], volume/uncertainty/100);
		}
		s_VoxFallParams.clusterIndices.clear();
	}

	qint64 volumeTime_ms = volumeTimer.elapsed();
	//we display block volume computation timing only if no error occurred!
	if (app)
		app->dispToConsole(QString("[VoxFall] Volume computation: %1 s").arg(volumeTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);


// 	   EXPORT BLOCKS AS VOXEL MESH MODELS (IF SELECTED)
//=======================================================================================================================

	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		//Duration: block meshing
		QElapsedTimer meshTimer;
		meshTimer.start();

		//progress notification
		pDlg.reset();
		NormalizedProgress nProgress(&pDlg, s_VoxFallParams.emptyVoxelCount);
		char buffer[64];
		snprintf(buffer, 64, "Blocks: %u", s_VoxFallParams.clusterLabel - 1);
		pDlg.setInfo(buffer);
		pDlg.setMethodTitle(QObject::tr("Exporting blocks as meshes"));
		pDlg.update(0);
		pDlg.start();

		//we create a new group to store all output meshes as 'VoxFall clusters'
		ccHObject* ccGroup = new ccHObject(mesh1->getName() + "_to_" + mesh2->getName() + QString(" [VoxFall clusters] (voxel %1 m)").arg(s_VoxFallParams.voxelSize));

		for (unsigned label = 1; label < s_VoxFallParams.clusterLabel; ++label)
		{
			std::vector<unsigned int> indices;
			auto it = std::find(s_VoxFallParams.clusters.begin(), s_VoxFallParams.clusters.end(), label);
			while (it != s_VoxFallParams.clusters.end())
			{
				indices.push_back(it - s_VoxFallParams.clusters.begin());
				it = std::find(it + 1, s_VoxFallParams.clusters.end(), label);
			}

			ccHObject* clusterGroup = new ccHObject(QString("Cluster#%1 - (v: %2 m3)").arg(label).arg(s_VoxFallParams.volumes[label - 1]));
			clusterGroup->setVisible(true);
			ccGroup->addChild(clusterGroup);
			for (int i = 0; i < indices.size(); i++)
			{
				CCVector3 V;
				s_VoxFallParams.voxfall->getPoint(indices[i], V);
				auto voxel = qVoxFallTransform::CreateVoxelMesh(V, s_VoxFallParams.voxelSize, indices[i]);
				clusterGroup->addChild(voxel);

				//progress bar
				if (!nProgress.oneStep())
				{
					return false;
				}
			}
			indices.clear();
		}
		ccGroup->applyGLTransformation_recursive(transform.inverse);
		ccGroup->setVisible(true);
		app->addToDB(ccGroup);

		qint64 meshTime_ms = meshTimer.elapsed();
		//we display block as mesh export timing only if no error occurred!
		if (app)
			app->dispToConsole(QString("[VoxFall] Block as mesh export: %1 s").arg(meshTime_ms / 1000.0, 0, 'f', 3),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}


// 	   OUTPUT FORMATION
//=======================================================================================================================
		
	//associate cluster ID scalar fields to the voxel grid
	int sfIdx = -1;
	if (s_VoxFallParams.clusterSF)
	{
		//add cluster ID SF to voxel grid
		s_VoxFallParams.clusterSF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.clusterSF);
	}
	if (s_VoxFallParams.exportLossGain)
	{
		//associate change type scalar fields to the voxel grid
		if (s_VoxFallParams.changeTypeSF)
		{
			//add cluster ID SF to voxel grid
			s_VoxFallParams.changeTypeSF->computeMinAndMax();
			sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.changeTypeSF);
		}
	}
	//associate volume scalar field to the voxel grid
	if (s_VoxFallParams.volumeSF)
	{
		//add volume SF to voxel grid
		s_VoxFallParams.volumeSF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.volumeSF);
	}
	//associate volume uncertainty scalar field to the voxel grid
	if (s_VoxFallParams.uncertaintySF)
	{
		//add volume uncertainty SF to voxel grid
		s_VoxFallParams.uncertaintySF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.uncertaintySF);
	}

	//prepare export cloud
	mesh1->applyGLTransformation_recursive(transform.inverse);
	s_VoxFallParams.voxfall->applyGLTransformation_recursive(transform.inverse);
	sfIdx = s_VoxFallParams.voxfall->getScalarFieldIndexByName(CLUSTER_SF_NAME);
	s_VoxFallParams.voxfall->setCurrentDisplayedScalarField(sfIdx);;
	s_VoxFallParams.voxfall->showSF(true);
	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		s_VoxFallParams.voxfall->setEnabled(false);
	}
	app->addToDB(s_VoxFallParams.voxfall);

	if (app)
		app->refreshAll();

}
