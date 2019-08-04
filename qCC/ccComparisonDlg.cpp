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

#include "ccComparisonDlg.h"

//Qt
#include <QHeaderView>
#include <QMessageBox>

//CCLib
#include <DistanceComputationTools.h>
#include <MeshSamplingTools.h>
#include <ScalarField.h>
#include <DgmOctree.h>
#include <ScalarFieldTools.h>

//qCC_db
#include <ccLog.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccOctree.h>
#include <ccProgressDialog.h>
#include <ccGBLSensor.h>

//Local
#include "mainwindow.h"
#include "ccCommon.h"
#include "ccHistogramWindow.h"

//Qt
#include <QElapsedTimer>
#include <QThreadPool>

//System
#include <assert.h>

const unsigned char DEFAULT_OCTREE_LEVEL = 7;

ccComparisonDlg::ccComparisonDlg(	ccHObject* compEntity,
									ccHObject* refEntity,
									CC_COMPARISON_TYPE cpType,
									QWidget* parent/*=0*/,
									bool noDisplay/*=false*/)
	: QDialog(parent, Qt::Tool)
	, Ui::ComparisonDialog()
	, m_compEnt(compEntity)
	, m_compCloud(0)
	, m_compOctree(0)
	, m_compOctreeIsPartial(false)
	, m_compSFVisibility(false)
	, m_refEnt(refEntity)
	, m_refCloud(0)
	, m_refMesh(0)
	, m_refOctree(0)
	, m_refOctreeIsPartial(false)
	, m_refVisibility(false)
	, m_compType(cpType)
	, m_noDisplay(noDisplay)
	, m_bestOctreeLevel(0)
{
	setupUi(this);

	int maxThreadCount = QThread::idealThreadCount();
	maxThreadCountSpinBox->setRange(1, maxThreadCount);
	maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(maxThreadCount));
	maxThreadCountSpinBox->setValue(QThreadPool::globalInstance()->maxThreadCount());

	//populate the combo-boxes
	{
		//octree level
		octreeLevelComboBox->addItem("AUTO");
		for (int i=1; i<=CCLib::DgmOctree::MAX_OCTREE_LEVEL; ++i)
			octreeLevelComboBox->addItem(QString::number(i));

		//local model
		localModelComboBox->addItem("NONE");
		localModelComboBox->addItem("Least Square Plane");
		localModelComboBox->addItem("2D1/2 Triangulation");
		localModelComboBox->addItem("Quadric");
		localModelComboBox->setCurrentIndex(0);
	}

	signedDistCheckBox->setChecked(false);
	split3DCheckBox->setEnabled(false);
	okButton->setEnabled(false);

	compName->setText(m_compEnt->getName());
	refName->setText(m_refEnt->getName());
	preciseResultsTabWidget->setCurrentIndex(0);

	m_refVisibility = (m_refEnt ? m_refEnt->isVisible() : false);
	m_compSFVisibility = (m_compEnt ? m_compEnt->sfShown() : false);

	if (!prepareEntitiesForComparison())
		return;

	assert(compEntity);
	ccBBox compEntBBox = compEntity->getOwnBB();
	maxSearchDistSpinBox->setValue(compEntBBox.getDiagNorm());

	if (m_refMesh)
	{
		localModelingTab->setEnabled(false);
		signedDistCheckBox->setEnabled(true);
		signedDistCheckBox->setChecked(true);
		filterVisibilityCheckBox->setEnabled(false);
		filterVisibilityCheckBox->setVisible(false);
	}
	else
	{
		signedDistCheckBox->setEnabled(false);
		split3DCheckBox->setEnabled(true);
		lmRadiusDoubleSpinBox->setValue(compEntBBox.getDiagNorm() / 200.0);
		filterVisibilityCheckBox->setEnabled(m_refCloud && m_refCloud->isA(CC_TYPES::POINT_CLOUD) && static_cast<ccPointCloud*>(m_refCloud)->hasSensor());
	}

	connect(cancelButton,			SIGNAL(clicked()),					this,	SLOT(cancelAndExit()));
	connect(okButton,				SIGNAL(clicked()),					this,	SLOT(applyAndExit()));
	connect(computeButton,			SIGNAL(clicked()),					this,	SLOT(computeDistances()));
	connect(histoButton,			SIGNAL(clicked()),					this,	SLOT(showHisto()));
	connect(localModelComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(locaModelChanged(int)));
	connect(maxDistCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(maxDistUpdated()));
	connect(maxSearchDistSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(maxDistUpdated()));

	//be sure to show the dialog before computing the approx distances
	//(otherwise the progress bars appear anywhere!)
	if (!m_noDisplay)
	{
		show();
		QCoreApplication::processEvents();
	}

	//compute approximate results and unlock GUI
	computeApproxDistances();
}

ccComparisonDlg::~ccComparisonDlg()
{
	releaseOctrees();
}

bool ccComparisonDlg::prepareEntitiesForComparison()
{
	if (!m_compEnt || !m_refEnt)
		return false;

	//compared entity
	if (!m_compEnt->isA(CC_TYPES::POINT_CLOUD)) //TODO --> pas possible avec des GenericPointCloud ? :(
	{
		if (m_compType == CLOUDCLOUD_DIST || (m_compType == CLOUDMESH_DIST && !m_compEnt->isKindOf(CC_TYPES::MESH)))
		{
			ccLog::Error("Dialog initialization error! (bad entity type)");
			return false;
		}
		ccGenericMesh* compMesh = ccHObjectCaster::ToGenericMesh(m_compEnt);
		if (!compMesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD)) //TODO
		{
			ccLog::Error("Dialog initialization error! (bad entity type - works only with real point clouds [todo])");
			return false;
		}
		m_compCloud = static_cast<ccPointCloud*>(compMesh->getAssociatedCloud());
	}
	else
	{
		m_compCloud = static_cast<ccPointCloud*>(m_compEnt);
	}

	//whatever the case, we always need the compared cloud's octree
	m_compOctree = m_compCloud->getOctree();
	if (!m_compOctree)
	{
		m_compOctree = ccOctree::Shared(new ccOctree(m_compCloud));
	}
	m_compOctreeIsPartial = false;

	//backup currently displayed SF (on compared cloud)
	int oldSfIdx = m_compCloud->getCurrentDisplayedScalarFieldIndex();
	if (oldSfIdx >= 0)
		m_oldSfName = QString(m_compCloud->getScalarFieldName(oldSfIdx));

	//reference entity
	if (	(m_compType == CLOUDMESH_DIST && !m_refEnt->isKindOf(CC_TYPES::MESH))
		||	(m_compType == CLOUDCLOUD_DIST && !m_refEnt->isA(CC_TYPES::POINT_CLOUD)) )
	{
		ccLog::Error("Dialog initialization error! (bad entity type)");
		return false;
	}

	if (m_compType == CLOUDMESH_DIST)
	{
		m_refMesh = ccHObjectCaster::ToGenericMesh(m_refEnt);
		m_refCloud = m_refMesh->getAssociatedCloud();
		m_refOctree.clear();
	}
	else /*if (m_compType == CLOUDCLOUD_DIST)*/
	{
		m_refCloud = ccHObjectCaster::ToGenericPointCloud(m_refEnt);

		//for computing cloud/cloud distances we need also the reference cloud's octree
		m_refOctree = m_refCloud->getOctree();
		if (!m_refOctree)
		{
			m_refOctree = ccOctree::Shared(new ccOctree(m_refCloud));
		}
	}
	m_refOctreeIsPartial = false;

	return true;
}

void ccComparisonDlg::maxDistUpdated()
{
	//the current 'best octree level' is depreacted
	m_bestOctreeLevel = 0;
}

int ccComparisonDlg::getBestOctreeLevel()
{
	if (m_bestOctreeLevel == 0)
	{
		double maxDistance = (maxDistCheckBox->isChecked() ? maxSearchDistSpinBox->value() : 0);
		
		int bestOctreeLevel = determineBestOctreeLevel(maxDistance);
		if (bestOctreeLevel <= 0)
		{
			ccLog::Error("Can't evaluate best octree level! Try to set it manually ...");
			return -1;
		}

		m_bestOctreeLevel = bestOctreeLevel;
	}
	
	return m_bestOctreeLevel;
}

void ccComparisonDlg::locaModelChanged(int index)
{
	localModelParamsFrame->setEnabled(index != 0);

	if (index != 0)
	{
		unsigned minKNN = CC_LOCAL_MODEL_MIN_SIZE[index];
		lmKNNSpinBox->setMinimum(minKNN);
	}
}

void ccComparisonDlg::releaseOctrees()
{
	if (m_compOctree && m_compCloud)
	{
		m_compOctree.clear();
		m_compOctreeIsPartial = false;
	}

	if (m_refOctree && m_refCloud)
	{
		m_refOctree.clear();
		m_refOctreeIsPartial = false;
	}
}

void ccComparisonDlg::updateDisplay(bool showSF, bool showRef)
{
	if (m_noDisplay)
		return;

	if (m_compEnt)
	{
		m_compEnt->setVisible(true);
		m_compEnt->setEnabled(true);
		m_compEnt->showSF(showSF);
		m_compEnt->prepareDisplayForRefresh_recursive();
	}

	if (m_refEnt)
	{
		m_refEnt->setVisible(showRef);
		m_refEnt->prepareDisplayForRefresh_recursive();
	}

	MainWindow::UpdateUI();
	MainWindow::RefreshAllGLWindow(false);
}

bool ccComparisonDlg::isValid()
{
	if (	!m_compCloud
		||	!m_compOctree
		||	(!m_refMesh && !m_refCloud)
		||	(!m_refMesh && !m_refOctree))
	{
		ccLog::Error("Dialog initialization error! (void entity)");
		return false;
	}

	return true;
}

bool ccComparisonDlg::computeApproxDistances()
{
	histoButton->setEnabled(false);
	preciseResultsTabWidget->widget(2)->setEnabled(false);

	if (!isValid())
		return false;

	//create the approximate dist. SF if necessary
	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx < 0)
	{
		sfIdx = m_compCloud->addScalarField(CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx < 0)
		{
			ccLog::Error("Failed to allocate a new scalar field for computing approx. distances! Try to free some memory ...");
			return false;
		}
	}

	m_compCloud->setCurrentScalarField(sfIdx);
	CCLib::ScalarField* sf = m_compCloud->getCurrentInScalarField();
	assert(sf);

	//prepare the octree structures
	QScopedPointer<ccProgressDialog> progressDlg;
	if (parentWidget())
	{
		progressDlg.reset(new ccProgressDialog(true, this));
		progressDlg->show();
	}

	int approxResult = -1;
	QElapsedTimer eTimer;
	eTimer.start();
	switch (m_compType)
	{
	case CLOUDCLOUD_DIST: //cloud-cloud
		{
			approxResult = CCLib::DistanceComputationTools::computeApproxCloud2CloudDistance(	m_compCloud,
																								m_refCloud,
																								DEFAULT_OCTREE_LEVEL,
																								0,
																								progressDlg.data(),
																								m_compOctree.data(),
																								m_refOctree.data());
		}
		break;
	
	case CLOUDMESH_DIST: //cloud-mesh
		{
			CCLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams c2mParams;
			{
				c2mParams.octreeLevel = DEFAULT_OCTREE_LEVEL;
				c2mParams.maxSearchDist = 0;
				c2mParams.useDistanceMap = true;
				c2mParams.signedDistances = false;
				c2mParams.flipNormals = false;
				c2mParams.multiThread = false;
			}
			approxResult = CCLib::DistanceComputationTools::computeCloud2MeshDistance(	m_compCloud,
																						m_refMesh,
																						c2mParams,
																						progressDlg.data(),
																						m_compOctree.data());
		}
		break;

	default:
		assert(false);
		break;
	}
	qint64 elapsedTime_ms = eTimer.elapsed();

	if (progressDlg)
	{
		progressDlg->stop();
	}

	//if the approximate distances comptation failed...
	if (approxResult < 0)
	{
		ccLog::Warning("[computeApproxDistances] Computation failed (error code %i)", approxResult);
		m_compCloud->deleteScalarField(sfIdx);
		sfIdx = -1;
	}
	else
	{
		ccLog::Print("[computeApproxDistances] Time: %3.2f s.", elapsedTime_ms / 1.0e3);

		//display approx. dist. statistics
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean,&variance);

		approxStats->setColumnCount(2);
		approxStats->setRowCount(5);
		approxStats->setColumnWidth(1,200);
		approxStats->horizontalHeader()->hide();
		int curRow = 0;

		//min dist
		approxStats->setItem(curRow, 0, new QTableWidgetItem("Min dist."));
		approxStats->setItem(curRow++, 1, new QTableWidgetItem(QString("%1").arg(sf->getMin())));

		//max dist
		approxStats->setItem(curRow, 0, new QTableWidgetItem("Max dist."));
		approxStats->setItem(curRow++, 1, new QTableWidgetItem(QString("%1").arg(sf->getMax())));

		//mean dist
		approxStats->setItem(curRow, 0, new QTableWidgetItem("Avg dist."));
		approxStats->setItem(curRow++, 1, new QTableWidgetItem(QString("%1").arg(mean)));

		//sigma
		approxStats->setItem(curRow, 0, new QTableWidgetItem("Sigma"));
		approxStats->setItem(curRow++, 1, new QTableWidgetItem(QString("%1").arg(variance >= 0.0 ? sqrt(variance) : variance)));

		//Max relative error
		PointCoordinateType cs = m_compOctree->getCellSize(DEFAULT_OCTREE_LEVEL);
		double e = cs / 2.0;
		approxStats->setItem(curRow, 0, new QTableWidgetItem("Max error"));
		approxStats->setItem(curRow++, 1, new QTableWidgetItem(QString("%1").arg(e)));

		for (int i = 0; i < curRow; ++i)
		{
			approxStats->setRowHeight(i, 20);
		}

		approxStats->setEditTriggers(QAbstractItemView::NoEditTriggers);

		//enable the corresponding UI items
		preciseResultsTabWidget->widget(2)->setEnabled(true);
		histoButton->setEnabled(true);

		//init the max search distance
		maxSearchDistSpinBox->setValue(sf->getMax());

		//update display
		m_compCloud->setCurrentDisplayedScalarField(sfIdx);
		m_compCloud->showSF(sfIdx >= 0);
	}

	computeButton->setEnabled(true);
	preciseResultsTabWidget->setEnabled(true);
	//we don't let the user leave with approximate distances!!!
	okButton->setEnabled(false);

	updateDisplay(sfIdx >= 0, false);

	return true;
}

int ccComparisonDlg::determineBestOctreeLevel(double maxSearchDist)
{
	if (!isValid())
	{
		return -1;
	}

	//make sure a the temporary dist. SF is activated
	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx < 0)
	{
		//we must compute approx. results again
		if (!computeApproxDistances())
		{
			//failed to compute approx distances?!
			return -1;
		}
		sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME);
	}

	const CCLib::ScalarField* approxDistances = m_compCloud->getScalarField(sfIdx);
	if (!approxDistances)
	{
		assert(sfIdx >= 0);
		return -1;
	}

	//evalutate the theoretical time for each octree level
	const int MAX_OCTREE_LEVEL = m_refMesh ? 9 : CCLib::DgmOctree::MAX_OCTREE_LEVEL; //DGM: can't go higher than level 9 with a mesh as the grid is 'plain' and would take too much memory!
	std::vector<double> timings;
	try
	{
		timings.resize(MAX_OCTREE_LEVEL, 0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Can't determine best octree level: not enough memory!");
		return -1;
	}

	//if the reference is a mesh
	double meanTriangleSurface = 1.0;
	CCLib::GenericIndexedMesh* mesh = 0;
	if (!m_refOctree)
	{
		if (!m_refMesh)
		{
			ccLog::Error("Internal error: reference entity should be a mesh!");
			return -1;
		}
		mesh = static_cast<CCLib::GenericIndexedMesh*>(m_refMesh);
		if (!mesh || mesh->size() == 0)
		{
			ccLog::Warning("Can't determine best octree level: mesh is empty!");
			return -1;
		}
		//total mesh surface
		double meshSurface = CCLib::MeshSamplingTools::computeMeshArea(mesh);
		//average triangle surface
		if (meshSurface > 0)
		{
			meanTriangleSurface = meshSurface / mesh->size();
		}
	}

	//we skip the lowest subdivision levels (useless + incompatible with below formulas ;)
	static const int s_minOctreeLevel = 6;
	int theBestOctreeLevel = s_minOctreeLevel;

	//we don't test the very first and very last level
	QScopedPointer<ccProgressDialog> progressDlg;
	if (parentWidget())
	{
		progressDlg.reset(new ccProgressDialog(false, this));
		progressDlg->setMethodTitle(tr("Determining optimal octree level"));
		progressDlg->setInfo(tr("Testing %1 levels...").arg(MAX_OCTREE_LEVEL)); //we lie here ;)
		progressDlg->start();
	}
	CCLib::NormalizedProgress nProgress(progressDlg.data(), MAX_OCTREE_LEVEL - 2);
	QApplication::processEvents();

	bool maxDistanceDefined = maxDistCheckBox->isChecked();
	PointCoordinateType maxDistance = static_cast<PointCoordinateType>(maxDistanceDefined ? maxSearchDistSpinBox->value() : 0);

	//for each level
	for (int level = s_minOctreeLevel; level < MAX_OCTREE_LEVEL; ++level)
	{
		const unsigned char bitDec = CCLib::DgmOctree::GET_BIT_SHIFT(level);
		unsigned numberOfPointsInCell = 0;
		unsigned index = 0;
		double cellDist = -1;
		//unsigned skippedCells = 0;

		//we compute a 'correction factor' that converts an approximate distance into an
		//approximate size of the neighborhood (in terms of cells)
		PointCoordinateType cellSize = m_compOctree->getCellSize(static_cast<unsigned char>(level));

		//we also use the reference cloud density (points/cell) if we have the info
		double refListDensity = 1.0;
		if (m_refOctree)
		{
			refListDensity = m_refOctree->computeMeanOctreeDensity(static_cast<unsigned char>(level));
		}

		CCLib::DgmOctree::CellCode tempCode = 0xFFFFFFFF;

		//scan the octree structure
		const CCLib::DgmOctree::cellsContainer& compCodes = m_compOctree->pointsAndTheirCellCodes();
		for (CCLib::DgmOctree::cellsContainer::const_iterator c=compCodes.begin(); c!=compCodes.end(); ++c)
		{
			CCLib::DgmOctree::CellCode truncatedCode = (c->theCode >> bitDec);

			//new cell?
			if (truncatedCode != tempCode)
			{
				//if it's a real cell
				if (numberOfPointsInCell != 0)
				{
					//if 'maxSearchDist' has been defined by the user, we must take it into account!
					//(in this case we skip the cell if its approx. distance is superior)
					if (maxSearchDist <= 0 || cellDist <= maxSearchDist)
					{
						//approx. neighborhood radius
						cellDist /= cellSize;

						//approx. neighborhood width (in terms of cells)
						double neighbourSize = 2.0*cellDist + 1.0;

						//if the reference is a mesh
						if (mesh)
						{
							//(integer) approximation of the neighborhood size (in terms of cells)
							int nCell = static_cast<int>(ceil(cellDist));

							//Probable mesh surface in this neighborhood
							double crossingMeshSurface = (2.0*nCell+1.0) * cellSize;
							//squared surface!
							crossingMeshSurface *= crossingMeshSurface;

							//neighborhood "volume" (in terms of cells)
							double neighbourSize3 = neighbourSize*neighbourSize*neighbourSize;

							//TIME = NEIGHBORS SEARCH + proportional factor * POINTS/TRIANGLES COMPARISONS
							timings[level] += neighbourSize3 + 0.5 * numberOfPointsInCell * crossingMeshSurface/meanTriangleSurface;
						}
						else
						{
							//we ignore the "central" cell
							neighbourSize -= 1.0;
							//neighborhood "volume" (in terms of cells)
							double neighbourSize3 = neighbourSize*neighbourSize*neighbourSize;
							//volume of the last "slice" (in terms of cells)
							//=V(n)-V(n-1) = (2*n+1)^3 - (2*n-1)^3 = 24 * n^2 + 2 (si n>0)
							double lastSliceCellNumber = (cellDist > 0 ? cellDist*cellDist * 24.0 + 2.0 : 1.0);
							//TIME = NEIGHBORS SEARCH + proportional factor * POINTS/TRIANGLES COMPARISONS
							//(we admit that the filled cells roughly correspond to the sqrt of the total number of cells)
							timings[level] += neighbourSize3 + 0.1 * numberOfPointsInCell * sqrt(lastSliceCellNumber) * refListDensity;
						}
					}
					//else
					//{
					//	++skippedCells;
					//}
				}

				numberOfPointsInCell = 0;
				cellDist = 0;
				tempCode = truncatedCode;
			}

			ScalarType pointDist = approxDistances->getValue(index);
			if (maxDistanceDefined && pointDist > maxDistance)
			{
				pointDist = maxDistance;
			}

			//cellDist += pointDist;
			cellDist = std::max<double>(cellDist, pointDist);
			++index;
			++numberOfPointsInCell;
		}

		////very high levels are unlikely (levelModifier ~ 0.85 @ level 20)
		//{
		//	double levelModifier = level < 12 ? 1.0 : exp(-pow(level-12,2)/(20*20));
		//	timings[level] /= levelModifier;

		//	ccLog::PrintDebug(QString("[Distances] Level %1 - timing = %2 (modifier = %3)").arg(level).arg(timings[level]).arg(levelModifier));
		//}

		//ccLog::Print("[Timing] Level %i --> %f",level,timings[level]);
		//timings[level] += (static_cast<qreal>(skippedCells)/1000)*skippedCells; //empirical correction for skipped cells (not taken into account while they actually require some processing time!)
		if (timings[level] < timings[theBestOctreeLevel])
		{
			theBestOctreeLevel = level;
		}

		nProgress.oneStep();
	}

	ccLog::PrintDebug("[Distances] Best level: %i (maxSearchDist = %f)", theBestOctreeLevel, maxSearchDist);

	return theBestOctreeLevel;
}

bool ccComparisonDlg::computeDistances()
{
	if (!isValid())
		return false;

	int octreeLevel = octreeLevelComboBox->currentIndex();
	assert(octreeLevel <= CCLib::DgmOctree::MAX_OCTREE_LEVEL);

	if (octreeLevel == 0)
	{
		//we'll try to guess the best octree level
		octreeLevel = getBestOctreeLevel();
		if (octreeLevel <= 0)
		{
			//best octree level computation failed?!
			return false;
		}
		ccLog::Print(QString("[Distances] Octree level (auto): %1").arg(octreeLevel));
	}

	//options
	bool signedDistances = signedDistCheckBox->isEnabled() && signedDistCheckBox->isChecked();
	bool flipNormals = (signedDistances ? flipNormalsCheckBox->isChecked() : false);
	bool split3D = split3DCheckBox->isEnabled() && split3DCheckBox->isChecked();

	//does the cloud has already a temporary scalar field that we can use?
	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx < 0)
	{
		//we need to create a new scalar field
		sfIdx = m_compCloud->addScalarField(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx < 0)
		{
			ccLog::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
			return false;
		}
	}

	m_compCloud->setCurrentScalarField(sfIdx);
	CCLib::ScalarField* sf = m_compCloud->getCurrentInScalarField();
	assert(sf);

	//max search distance
	ScalarType maxSearchDist = static_cast<ScalarType>(maxDistCheckBox->isChecked() ? maxSearchDistSpinBox->value() : 0);
	//multi-thread
	bool multiThread = multiThreadedCheckBox->isChecked();

	CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams c2cParams;
	CCLib::DistanceComputationTools::Cloud2MeshDistanceComputationParams  c2mParams;
	c2cParams.maxThreadCount = c2mParams.maxThreadCount = maxThreadCountSpinBox->value();

	int result = -1;
	QScopedPointer<ccProgressDialog> progressDlg;
	if (parentWidget())
	{
		progressDlg.reset(new ccProgressDialog(true, this));
	}

	QElapsedTimer eTimer;
	eTimer.start();
	switch(m_compType)
	{
	case CLOUDCLOUD_DIST: //cloud-cloud

		if (split3D)
		{
			//we create 3 new scalar fields, one for each dimension
			unsigned count = m_compCloud->size();

			bool success = true;
			for (unsigned j = 0; j < 3; ++j)
			{
				ccScalarField* sfDim = new ccScalarField();
				if (sfDim->resizeSafe(count))
				{
					sfDim->link();
					c2cParams.splitDistances[j] = sfDim;
				}
				else
				{
					success = false;
					break;
				}
			}

			if (!success)
			{
				ccLog::Error("[ComputeDistances] Not enough memory to generate 3D split fields!");

				for (unsigned j = 0; j < 3; ++j)
				{
					if (c2cParams.splitDistances[j])
					{
						c2cParams.splitDistances[j]->release();
						c2cParams.splitDistances[j] = nullptr;
					}
				}
			}
		}
		
		if (m_refCloud->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* pc = static_cast<ccPointCloud*>(m_refCloud);

			//we enable the visibility checking if the user asked for it
			bool filterVisibility = filterVisibilityCheckBox->isEnabled() && filterVisibilityCheckBox->isChecked();
			if (filterVisibility)
			{
				size_t validDB = 0;
				//we also make sure that the sensors have valid depth buffer!
				for (unsigned i = 0; i < pc->getChildrenNumber(); ++i)
				{
					ccHObject* child = pc->getChild(i);
					if (child && child->isA(CC_TYPES::GBL_SENSOR))
					{
						ccGBLSensor* sensor = static_cast<ccGBLSensor*>(child);
						if (sensor->getDepthBuffer().zBuff.empty())
						{
							int errorCode;
							if (!sensor->computeDepthBuffer(pc, errorCode))
							{
								ccLog::Warning(QString("[ComputeDistances] ") + ccGBLSensor::GetErrorString(errorCode));
							}
							else
							{
								++validDB;
							}
						}
						else
						{
							++validDB;
						}
					}
				}

				if (validDB == 0)
				{
					filterVisibilityCheckBox->setChecked(false);
					if (QMessageBox::warning(	this,
												"Error",
												"Failed to find/init the depth buffer(s) on the associated sensor! Do you want to continue?",
												QMessageBox::Yes,
												QMessageBox::No) == QMessageBox::No)
					{
						break;
					}
					filterVisibility = false;
				}
			}
			pc->enableVisibilityCheck(filterVisibility);
		}

		//setup parameters
		{
			c2cParams.octreeLevel = static_cast<unsigned char>(octreeLevel);
			if (localModelingTab->isEnabled())
			{
				c2cParams.localModel = (CC_LOCAL_MODEL_TYPES)localModelComboBox->currentIndex();
				if (c2cParams.localModel != NO_MODEL)
				{
					c2cParams.useSphericalSearchForLocalModel = lmRadiusRadioButton->isChecked();
					c2cParams.kNNForLocalModel = static_cast<unsigned>(std::max(0,lmKNNSpinBox->value()));
					c2cParams.radiusForLocalModel = static_cast<ScalarType>(lmRadiusDoubleSpinBox->value());
					c2cParams.reuseExistingLocalModels = lmOptimizeCheckBox->isChecked();
				}
			}
			c2cParams.maxSearchDist = maxSearchDist;
			c2cParams.multiThread = multiThread;
			c2cParams.CPSet = 0;
		}
		
		result = CCLib::DistanceComputationTools::computeCloud2CloudDistance(	m_compCloud,
																				m_refCloud,
																				c2cParams,
																				progressDlg.data(),
																				m_compOctree.data(),
																				m_refOctree.data());
		break;

	case CLOUDMESH_DIST: //cloud-mesh

		if (multiThread && maxDistCheckBox->isChecked())
		{
			ccLog::Warning("[Cloud/Mesh comparison] Max search distance is not supported in multi-thread mode! Switching to single thread mode...");
		}

		//setup parameters
		{
			c2mParams.octreeLevel = static_cast<unsigned char>(octreeLevel);
			c2mParams.maxSearchDist = maxSearchDist;
			c2mParams.useDistanceMap = false;
			c2mParams.signedDistances = signedDistances;
			c2mParams.flipNormals = flipNormals;
			c2mParams.multiThread = multiThread;
		}
		
		result = CCLib::DistanceComputationTools::computeCloud2MeshDistance(	m_compCloud,
																				m_refMesh,
																				c2mParams,
																				progressDlg.data(),
																				m_compOctree.data());
		break;
	}
	qint64 elapsedTime_ms = eTimer.elapsed();

	if (progressDlg)
	{
		progressDlg->stop();
	}

	if (result >= 0)
	{
		ccLog::Print("[ComputeDistances] Time: %3.2f s.",static_cast<double>(elapsedTime_ms)/1.0e3);

		//display some statics about the computed distances
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean, &variance);
		ccLog::Print("[ComputeDistances] Mean distance = %f / std deviation = %f",mean,sqrt(variance));

		m_compCloud->setCurrentDisplayedScalarField(sfIdx);
		m_compCloud->showSF(sfIdx >= 0);

		//restore UI items
		okButton->setEnabled(true);

		m_sfName.clear();
		switch(m_compType)
		{
		case CLOUDCLOUD_DIST: //hausdorff
			m_sfName = QString(CC_CLOUD2CLOUD_DISTANCES_DEFAULT_SF_NAME);
			break;
		case CLOUDMESH_DIST: //cloud-mesh
			m_sfName = QString(signedDistances ? CC_CLOUD2MESH_SIGNED_DISTANCES_DEFAULT_SF_NAME : CC_CLOUD2MESH_DISTANCES_DEFAULT_SF_NAME);
			break;
		}

		if (c2cParams.localModel != NO_MODEL)
		{
			m_sfName += QString("[%1]").arg(localModelComboBox->currentText());
			if (c2cParams.useSphericalSearchForLocalModel)
				m_sfName += QString("[r=%1]").arg(c2cParams.radiusForLocalModel);
			else
				m_sfName += QString("[k=%1]").arg(c2cParams.kNNForLocalModel);
			if (c2cParams.reuseExistingLocalModels)
				m_sfName += QString("[fast]");
		}

		if (flipNormals)
		{
			m_sfName += QString("[-]");
		}

		if (maxSearchDist > 0)
		{
			m_sfName += QString("[<%1]").arg(maxSearchDist);
		}

		if (split3D)
		{
			//we add the corresponding scalar fields (one for each dimension)
			static const QChar charDim[3] = { 'X', 'Y', 'Z' };
			for (unsigned j = 0; j < 3; ++j)
			{
				CCLib::ScalarField* sf = c2cParams.splitDistances[j];
				if (sf)
				{
					sf->setName(qPrintable(m_sfName + QString(" (%1)").arg(charDim[j])));
					sf->computeMinAndMax();
					//check that SF doesn't already exist
					int sfExit = m_compCloud->getScalarFieldIndexByName(sf->getName());
					if (sfExit >= 0)
						m_compCloud->deleteScalarField(sfExit);
					int sfEnter = m_compCloud->addScalarField(static_cast<ccScalarField*>(sf));
					assert(sfEnter >= 0);
				}
			}
			ccLog::Warning("[ComputeDistances] Result has been split along each dimension (check the 3 other scalar fields with '_X', '_Y' and '_Z' suffix!)");
		}
	}
	else
	{
		ccLog::Error("[ComputeDistances] Error (%i)",result);
		
		m_compCloud->deleteScalarField(sfIdx);
		m_compCloud->showSF(false);
		sfIdx = -1;
	}

	for (unsigned j = 0; j < 3; ++j)
	{
		CCLib::ScalarField* &sf = c2cParams.splitDistances[j];
		if (sf)
		{
			sf->release();
			sf = nullptr;
		}
	}
			
	updateDisplay(sfIdx >= 0, false);

	return result >= 0;
}

void ccComparisonDlg::showHisto()
{
	if (!m_compCloud)
		return;

	ccScalarField* sf = m_compCloud->getCurrentDisplayedScalarField();
	if (!sf)
		return;

	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setWindowTitle(QString("Histogram [%1]").arg(m_compCloud->getName()));
	{
		ccHistogramWindow* histogram = hDlg->window();
		histogram->setTitle(QString("Approximate distances (%1 values)").arg(m_compCloud->size()));
		histogram->fromSF(sf, 8, false);
		histogram->setAxisLabels("Approximate distances", "Count");
	}
	hDlg->resize(400, 300);
	hDlg->show();
}

void ccComparisonDlg::applyAndExit()
{
	if (m_compCloud)
	{
		//m_compCloud->setCurrentDisplayedScalarField(-1);
		//m_compCloud->showSF(false);

		//remove the approximate dist. SF
		int tmpSfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME);
		if (tmpSfIdx >= 0)
		{
			m_compCloud->deleteScalarField(tmpSfIdx);
			tmpSfIdx = -1;
		}

		//now, if we have a temp distance scalar field (the 'real' distances computed by the user)
		//we should rename it properly
		int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx >= 0)
		{
			if (m_sfName.isEmpty()) //hum,hum
			{
				ccLog::Warning("Something went wrong!");
				m_compCloud->deleteScalarField(sfIdx);
				m_compCloud->setCurrentDisplayedScalarField(-1);
				m_compCloud->showSF(false);
			}
			else
			{
				//we delete any existing scalar field with the exact same name
				int _sfIdx = m_compCloud->getScalarFieldIndexByName(qPrintable(m_sfName));
				if (_sfIdx >= 0)
				{
					m_compCloud->deleteScalarField(_sfIdx);
					//we update sfIdx because indexes are all messed up after deletion
					sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
				}

				m_compCloud->renameScalarField(sfIdx,qPrintable(m_sfName));
				m_compCloud->setCurrentDisplayedScalarField(sfIdx);
				m_compCloud->showSF(sfIdx >= 0);
			}
		}

		//m_compCloud->setCurrentDisplayedScalarField(-1);
		//m_compCloud->showSF(false);

	}

	updateDisplay(true, m_refVisibility);

	releaseOctrees();

	accept();
}

void ccComparisonDlg::cancelAndExit()
{
	if (m_compCloud)
	{
		m_compCloud->setCurrentDisplayedScalarField(-1);
		m_compCloud->showSF(false);

		//remove the approximate dist. SF
		int tmpSfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_APPROX_DISTANCES_DEFAULT_SF_NAME);
		if (tmpSfIdx >= 0)
		{
			m_compCloud->deleteScalarField(tmpSfIdx);
			tmpSfIdx = -1;
		}

		int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx >= 0)
		{
			m_compCloud->deleteScalarField(sfIdx);
			sfIdx = -1;
		}

		if (!m_oldSfName.isEmpty())
		{
			int oldSfIdx = m_compCloud->getScalarFieldIndexByName(qPrintable(m_oldSfName));
			if (oldSfIdx)
			{
				m_compCloud->setCurrentDisplayedScalarField(oldSfIdx);
				m_compCloud->showSF(oldSfIdx >= 0);
			}
		}
	}

	updateDisplay(m_compSFVisibility, m_refVisibility);

	releaseOctrees();

	reject();
}
