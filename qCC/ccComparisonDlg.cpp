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

#include "ccComparisonDlg.h"

//Qt
#include <QHeaderView>

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

//Local
#include "ccDisplayOptionsDlg.h"
#include "mainwindow.h"
#include "ccCommon.h"
#include "ccHistogramWindow.h"

//Qt
#include <QElapsedTimer>

//System
#include <assert.h>

const uchar DEFAULT_OCTREE_LEVEL = 7;

ccComparisonDlg::ccComparisonDlg(	ccHObject* compEntity,
									ccHObject* refEntity,
									CC_COMPARISON_TYPE cpType,
									QWidget* parent/*=0*/,
									bool noDisplay/*=false*/)
	: QDialog(parent)
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
	, m_currentSFIsDistance(false)
	, m_noDisplay(noDisplay)
	, m_needToRecomputeBestLevel(true)
{
	setupUi(this);
	setWindowFlags(Qt::Tool);

	localModelComboBox->addItem("NONE");
	localModelComboBox->addItem("Least Square Plane");
	localModelComboBox->addItem("2D1/2 Triangulation");
	localModelComboBox->addItem("Height Function");
	localModelComboBox->setCurrentIndex(0);

	signedDistCheckBox->setChecked(false);
	split3DCheckBox->setEnabled(false);
	signedDistFrame->setEnabled(false);
	okButton->setEnabled(false);

	connect(cancelButton,			SIGNAL(clicked()),					this,	SLOT(cancelAndExit()));
	connect(okButton,				SIGNAL(clicked()),					this,	SLOT(applyAndExit()));
	connect(computeButton,			SIGNAL(clicked()),					this,	SLOT(compute()));
	connect(histoButton,			SIGNAL(clicked()),					this,	SLOT(showHisto()));
	connect(localModelComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(locaModelChanged(int)));
	connect(octreeLevelCheckBox,	SIGNAL(toggled(bool)),				this,	SLOT(octreeLevelCheckBoxToggled(bool)));
	connect(maxDistCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(maxDistUpdated()));
	connect(maxSearchDistSpinBox,	SIGNAL(editingFinished()),			this,	SLOT(maxDistUpdated()));
	connect(split3DCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(split3DCheckboxToggled(bool)));

	octreeLevelSpinBox->setRange(1,CCLib::DgmOctree::MAX_OCTREE_LEVEL);
	compName->setText(m_compEnt->getName());
	refName->setText(m_refEnt->getName());
	preciseResultsTabWidget->setCurrentIndex(0);

	m_refVisibility = (m_refEnt ? m_refEnt->isVisible() : false);
	m_compSFVisibility = (m_compEnt ? m_compEnt->sfShown() : false);

	if (!prepareEntitiesForComparison())
		return;

	assert(compEntity);
	ccBBox compEntBBox = compEntity->getOwnBB();
	maxSearchDistSpinBox->setValue(static_cast<double>(compEntBBox.getDiagNorm()));

	if (m_refMesh)
	{
		localModelingTab->setEnabled(false);
		signedDistFrame->setEnabled(true);
		signedDistCheckBox->setChecked(true);
	}
	else
	{
		split3DCheckBox->setEnabled(true);
		lmRadiusDoubleSpinBox->setValue(static_cast<double>(compEntBBox.getDiagNorm())/200);
	}

	//compute approximate results and unlock GUI
	computeApproxResults();
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
	m_compOctree = static_cast<CCLib::DgmOctree*>(m_compCloud->getOctree());
	if (!m_compOctree)
		m_compOctree = new CCLib::DgmOctree(m_compCloud);
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
		m_refOctree = 0;
	}
	else /*if (m_compType == CLOUDCLOUD_DIST)*/
	{
		m_refCloud = ccHObjectCaster::ToGenericPointCloud(m_refEnt);

		//for computing cloud/cloud distances we need also the reference cloud's octree
		m_refOctree = static_cast<CCLib::DgmOctree*>(m_refCloud->getOctree());
		if (!m_refOctree)
			m_refOctree = new CCLib::DgmOctree(m_refCloud);
	}
	m_refOctreeIsPartial = false;

	return true;
}

void ccComparisonDlg::octreeLevelCheckBoxToggled(bool state)
{
	if (!state) //automatic mode
	{
		//force best octree level computation
		m_needToRecomputeBestLevel = true;

		updateOctreeLevel();
	}
}

void ccComparisonDlg::maxDistUpdated()
{
	//force best octree level computation
	m_needToRecomputeBestLevel = true;
	computeApproxResults();

	//change the focus to another entity!
	computeButton->setFocus();

	updateOctreeLevel();
}

void ccComparisonDlg::updateOctreeLevel()
{
	if (!m_needToRecomputeBestLevel)
		return;

	//we only compute best octree level if "auto" mode is on or the user has set the level to "0"
	if (!octreeLevelSpinBox->isEnabled() || octreeLevelSpinBox->value() == 0)
	{
		double maxDistance = (maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
		
		int guessedBestOctreeLevel = determineBestOctreeLevel(static_cast<ScalarType>(maxDistance));
		if (guessedBestOctreeLevel > 0)
		{
			octreeLevelSpinBox->setValue(guessedBestOctreeLevel);
		}
		else
		{
			ccLog::Error("Can't evaluate best computation level! Try to set it manually ...");
			octreeLevelCheckBox->setCheckState(Qt::Checked);
			octreeLevelSpinBox->setEnabled(true);
		}

		m_needToRecomputeBestLevel = false;
	}
}

void ccComparisonDlg::split3DCheckboxToggled(bool state)
{
	if (m_compType == CLOUDMESH_DIST)
	{
		signedDistFrame->setEnabled(state);
		if (state && !signedDistCheckBox->isChecked())
		{
			signedDistCheckBox->setChecked(true);
			flipNormalsCheckBox->setEnabled(false);
		}
	}
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
		if (m_compCloud->getOctree() != m_compOctree)
			delete m_compOctree;
		m_compOctree = 0;
		m_compOctreeIsPartial = false;
	}

	if (m_refOctree && m_refCloud)
	{
		if (m_refCloud->getOctree() != m_refOctree)
			delete m_refOctree;
		m_refOctree = 0;
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

int ccComparisonDlg::computeApproxResults()
{
	int approxResult = -1;

	histoButton->setEnabled(false);
	preciseResultsTabWidget->widget(2)->setEnabled(false);

	if (!isValid())
		return approxResult;

	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx < 0)
		sfIdx = m_compCloud->addScalarField(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx < 0)
	{
		ccLog::Error("Failed to allocate a new scalar field for computing distances! Try to free some memory ...");
		return approxResult;
	}

	m_compCloud->setCurrentScalarField(sfIdx);
	CCLib::ScalarField* sf = m_compCloud->getCurrentInScalarField();
	assert(sf);

	//prepare the octree structures
	ccProgressDialog progressDlg(true,this);

	QElapsedTimer eTimer;
	eTimer.start();
	switch(m_compType)
	{
	case CLOUDCLOUD_DIST: //hausdroff
		{
			//Approximate distance can (and must) now take max search distance into account!
			PointCoordinateType maxDistance = static_cast<PointCoordinateType>(maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
			approxResult = CCLib::DistanceComputationTools::computeApproxCloud2CloudDistance(CHAMFER_345,m_compCloud,m_refCloud,DEFAULT_OCTREE_LEVEL,maxDistance,&progressDlg,m_compOctree,m_refOctree);
		}
		break;
	case CLOUDMESH_DIST: //cloud-mesh
		{
			approxResult = CCLib::DistanceComputationTools::computeCloud2MeshDistance(m_compCloud,m_refMesh,DEFAULT_OCTREE_LEVEL,-1.0,true,false,false,false,&progressDlg,m_compOctree);
		}
		break;
	}
	qint64 elapsedTime_ms = eTimer.elapsed();

	progressDlg.stop();

	int guessedBestOctreeLevel = -1;

	//if the approximate distances comptation failed...
	if (approxResult < 0)
	{
		ccLog::Warning("[computeApproxResults] Computation failed (error code %i)",approxResult);
		m_compCloud->deleteScalarField(sfIdx);
		sfIdx = -1;
		m_currentSFIsDistance = false;
	}
	else
	{
		ccLog::Print("[computeApproxResults] Time: %3.2f s.",static_cast<double>(elapsedTime_ms)/1.0e3);

		//display approx. dist. statistics
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean,&variance);

		approxStats->setColumnCount(2);
		approxStats->setRowCount(5);
		approxStats->setColumnWidth(1,200);
		approxStats->horizontalHeader()->hide();
		QTableWidgetItem *item = 0;
		int curRow=0;

		//min dist
		item = new QTableWidgetItem("Min dist.");
		approxStats->setItem(curRow, 0, item);
		item = new QTableWidgetItem(QString("%1").arg(sf->getMin()));
		approxStats->setItem(curRow++, 1, item);

		//max dist
		item = new QTableWidgetItem("Max dist.");
		approxStats->setItem(curRow, 0, item);
		item = new QTableWidgetItem(QString("%1").arg(sf->getMax()));
		approxStats->setItem(curRow++, 1, item);

		//mean dist
		item = new QTableWidgetItem("Avg dist.");
		approxStats->setItem(curRow, 0, item);
		item = new QTableWidgetItem(QString("%1").arg(mean));
		approxStats->setItem(curRow++, 1, item);

		//sigma
		item = new QTableWidgetItem("Sigma");
		approxStats->setItem(curRow, 0, item);
		item = new QTableWidgetItem(QString("%1").arg(variance >= 0.0 ? sqrt(variance) : variance));
		approxStats->setItem(curRow++, 1, item);

		//Max relative error
		PointCoordinateType cs = m_compOctree->getCellSize(DEFAULT_OCTREE_LEVEL);
		double e1 = 100.0*(1.0-sqrt(25.0/27.0));
		double e2 = 100.0*static_cast<double>(cs);
		item = new QTableWidgetItem("Max relative error");
		approxStats->setItem(curRow, 0, item);
		item = new QTableWidgetItem(QString("%1 + %2/d % (d>%3)").arg(e1).arg(e2).arg(cs));
		approxStats->setItem(curRow++, 1, item);

		for (int i=0;i<curRow;++i)
			approxStats->setRowHeight(i,20);

		//enable the corresponding UI items
		preciseResultsTabWidget->widget(2)->setEnabled(true);
		histoButton->setEnabled(true);

		//update display
		m_compCloud->setCurrentDisplayedScalarField(sfIdx);
		m_compCloud->showSF(sfIdx >= 0);

		m_currentSFIsDistance = true;

		//now find the best octree level for real dist. computation
		guessedBestOctreeLevel = determineBestOctreeLevel(-1.0);
	}

	if (guessedBestOctreeLevel < 0)
	{
		ccLog::Warning("[computeApproxResults] Can't evaluate best computation level! Try to set it manually ...");
		octreeLevelCheckBox->setCheckState(Qt::Checked);
		octreeLevelSpinBox->setEnabled(true);
		guessedBestOctreeLevel = static_cast<int>(DEFAULT_OCTREE_LEVEL);
	}
	octreeLevelSpinBox->setValue(guessedBestOctreeLevel);
	m_needToRecomputeBestLevel = false;

	computeButton->setEnabled(true);
	preciseGroupBox->setEnabled(true);
	//we don't let the user leave with approximate distances!!!
	okButton->setEnabled(false);

	updateDisplay(sfIdx >= 0, false);

	return guessedBestOctreeLevel;
}

int ccComparisonDlg::determineBestOctreeLevel(double maxSearchDist)
{
	if (!isValid())
		return -1;

	//make sure a valid SF is activated
	if (!m_currentSFIsDistance || m_compCloud->getCurrentOutScalarFieldIndex() < 0)
	{
		//we must compute approx. results again
		//(this method will be called again later)
		return computeApproxResults();
	}

	//evalutate the theoretical time for each octree level
	const int MaxLevel = m_refMesh ? 9 : CCLib::DgmOctree::MAX_OCTREE_LEVEL; //DGM: can't go higher than level 9 with a mesh as the grid is 'plain' and would take too much memory!
	std::vector<double> timings;
	try
	{
		timings.resize(MaxLevel,0);
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
			meanTriangleSurface = meshSurface / mesh->size();
	}

	//we skip the lowest subdivision levels (useless + incompatible with below formulas ;)
	int theBestOctreeLevel = 2;

	//we don't test the very first and very last level
	ccProgressDialog progressCb(false,this);
	progressCb.setMethodTitle("Determining optimal octree level");
	progressCb.setInfo(qPrintable(QString("Testing %1 levels...").arg(MaxLevel))); //we lie here ;)
	CCLib::NormalizedProgress nProgress(&progressCb,MaxLevel-2);
	progressCb.start();
	QApplication::processEvents();

	//for each level
	for (int level=2; level<MaxLevel; ++level)
	{
		const int bitDec = GET_BIT_SHIFT(level);
		unsigned numberOfPointsInCell = 0;
		unsigned index = 0;
		double cellDist = -1;
		//unsigned skippedCells = 0;

		//we compute a 'correction factor' that converts an approximate distance into an
		//approximate size of the neighborhood (in terms of cells)
		PointCoordinateType cellSize = m_compOctree->getCellSize(static_cast<uchar>(level));

		//we also use the reference cloud density (points/cell) if we have the info
		double refListDensity = 1.0;
		if (m_refOctree)
			refListDensity = m_refOctree->computeMeanOctreeDensity(static_cast<uchar>(level));

		CCLib::DgmOctree::OctreeCellCodeType tempCode = 0xFFFFFFFF;

		//scan the octree structure
		const CCLib::DgmOctree::cellsContainer& compCodes = m_compOctree->pointsAndTheirCellCodes();
		for (CCLib::DgmOctree::cellsContainer::const_iterator c=compCodes.begin(); c!=compCodes.end(); ++c)
		{
			CCLib::DgmOctree::OctreeCellCodeType truncatedCode = (c->theCode >> bitDec);

			//new cell?
			if (truncatedCode != tempCode)
			{
				//if it's a real cell
				if (numberOfPointsInCell != 0)
				{
					//if 'maxSearchDist' has been defined by the user, we must take it into account!
					//(in this case we skip the cell if its approx. distance is superior)
					if (maxSearchDist < 0 || cellDist <= maxSearchDist)
					{
						//approx. neighborhood radius
						cellDist /= cellSize;

						//approx. neighborhood width (in terms of cells)
						double neighbourSize = 2.0*cellDist+1.0;

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

			double pointDist = m_compCloud->getPointScalarValue(index);
			//cellDist += pointDist;
			cellDist = std::max(cellDist,pointDist);
			++index;
			++numberOfPointsInCell;
		}

		////very high levels are unlikely (levelModifier ~ 0.85 @ level 20)
		//{
		//	double levelModifier = level < 12 ? 1.0 : exp(-pow(level-12,2)/(20*20));
		//	timings[level] /= levelModifier;

		//	ccLog::PrintDebug(QString("[ccComparisonDlg] Level %1 - timing = %2 (modifier = %3)").arg(level).arg(timings[level]).arg(levelModifier));
		//}

		//ccLog::Print("[Timing] Level %i --> %f",level,timings[level]);
		//timings[level] += (static_cast<qreal>(skippedCells)/1000)*skippedCells; //empirical correction for skipped cells (not taken into account while they actually require some processing time!)
		if (timings[level] < timings[theBestOctreeLevel])
			theBestOctreeLevel = level;

		nProgress.oneStep();
	}

	ccLog::PrintDebug("[ccComparisonDlg] Best level: %i (maxSearchDist = %f)",theBestOctreeLevel,maxSearchDist);

	return theBestOctreeLevel;
}

bool ccComparisonDlg::compute()
{
	if (!isValid())
		return false;

	//updates best octree level guess if necessary
	updateOctreeLevel();
	int bestOctreeLevel = octreeLevelSpinBox->value();

	bool signedDistances = signedDistFrame->isEnabled() && signedDistCheckBox->isChecked();
	bool flipNormals = (signedDistances ? flipNormalsCheckBox->isChecked() : false);

	bool split3D = split3DCheckBox->isEnabled() && split3DCheckBox->isChecked();
	//for cloud-cloud distance, 'signedDistances' is only used for 'split3D' mode
	bool split3DSigned = signedDistances;

	//does the cloud has already a temporary scalar field that we can use?
	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx < 0)
	{
		//or maybe a real one?
		sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	}
	if (sfIdx >= 0)
	{
		CCLib::ScalarField* sf = m_compCloud->getScalarField(sfIdx);
		assert(sf);
		//we recycle an existing scalar field, we must rename it
		sf->setName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	}
	else
	{
		assert(!m_currentSFIsDistance);
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
	ScalarType maxSearchDist = static_cast<ScalarType>(maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
	//multi-thread
	bool multiThread = multiThreadedCheckBox->isChecked();

	int result = -1;
	ccProgressDialog progressDlg(true,this);

	//for 3D splitting (cloud-cloud dist. only)
	CCLib::ReferenceCloud* CPSet = 0;
	if (split3D)
	{
		assert(m_refCloud);
		CPSet = new CCLib::ReferenceCloud(m_refCloud);
	}

	CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
	params.octreeLevel = static_cast<uchar>(bestOctreeLevel);
	if (localModelingTab->isEnabled())
	{
		params.localModel = (CC_LOCAL_MODEL_TYPES)localModelComboBox->currentIndex();
		if (params.localModel != NO_MODEL)
		{
			params.useSphericalSearchForLocalModel = lmRadiusRadioButton->isChecked();
			params.kNNForLocalModel = static_cast<unsigned>(std::max(0,lmKNNSpinBox->value()));
			params.radiusForLocalModel = static_cast<ScalarType>(lmRadiusDoubleSpinBox->value());
			params.reuseExistingLocalModels = lmOptimizeCheckBox->isChecked();
		}
	}
	params.maxSearchDist = maxSearchDist;
	params.multiThread = multiThread;
	params.CPSet = CPSet;

	QElapsedTimer eTimer;
	eTimer.start();
	switch(m_compType)
	{
	case CLOUDCLOUD_DIST: //hausdorff

		result = CCLib::DistanceComputationTools::computeCloud2CloudDistance(	m_compCloud,
																				m_refCloud,
																				params,
																				&progressDlg,
																				m_compOctree,
																				m_refOctree);
		break;

	case CLOUDMESH_DIST: //cloud-mesh

		if (multiThread && maxSearchDistSpinBox->isEnabled())
			ccLog::Warning("[Cloud/Mesh comparison] Max search distance is not supported in multi-thread mode! Switching to single thread mode...");
		
		result = CCLib::DistanceComputationTools::computeCloud2MeshDistance(	m_compCloud,
																				m_refMesh,
																				static_cast<uchar>(bestOctreeLevel),
																				maxSearchDist,
																				false,
																				signedDistances,
																				flipNormals,
																				multiThread,
																				&progressDlg,
																				m_compOctree);
		break;
	}
	qint64 elapsedTime_ms = eTimer.elapsed();

	progressDlg.stop();

	if (result >= 0)
	{
		ccLog::Print("[ComputeDistances] Time: %3.2f s.",static_cast<double>(elapsedTime_ms)/1.0e3);

		//display some statics about the computed distances
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean,&variance);
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

		if (params.localModel != NO_MODEL)
		{
			m_sfName += QString("[%1]").arg(localModelComboBox->currentText());
			if (params.useSphericalSearchForLocalModel)
				m_sfName += QString("[r=%1]").arg(params.radiusForLocalModel);
			else
				m_sfName += QString("[k=%1]").arg(params.kNNForLocalModel);
			if (params.reuseExistingLocalModels)
				m_sfName += QString("[fast]");
		}

		if (flipNormals)
			m_sfName += QString("[-]");

		m_currentSFIsDistance = true;

		if (maxSearchDist >= 0)
		{
			m_sfName += QString("[<%1]").arg(maxSearchDist);
			m_currentSFIsDistance = false;
		}

		if (split3D)
		{
			//we create 3 new scalar fields, one for each dimension
			assert(CPSet && CPSet->size() == m_compCloud->size());
			unsigned count = CPSet->size();
			ccScalarField* sfDims[3]= {	new ccScalarField(qPrintable(m_sfName+QString(" (X)"))),
										new ccScalarField(qPrintable(m_sfName+QString(" (Y)"))),
										new ccScalarField(qPrintable(m_sfName+QString(" (Z)")))
			};

			sfDims[0]->link();
			sfDims[1]->link();
			sfDims[2]->link();

			if (sfDims[0]->resize(count) &&
				sfDims[1]->resize(count) &&
				sfDims[2]->resize(count))
			{
				for (unsigned i=0; i<count; ++i)
				{
					const CCVector3* P = CPSet->getPoint(i);
					const CCVector3* Q = m_compCloud->getPoint(i);
					CCVector3 D = *Q-*P;

					sfDims[0]->setValue(i,static_cast<ScalarType>(split3DSigned ? D.x : fabs(D.x)));
					sfDims[1]->setValue(i,static_cast<ScalarType>(split3DSigned ? D.y : fabs(D.y)));
					sfDims[2]->setValue(i,static_cast<ScalarType>(split3DSigned ? D.z : fabs(D.z)));
				}

				for (unsigned j=0; j<3; ++j)
				{
					sfDims[j]->computeMinAndMax();
					//check that SF doesn't already exist
					int sfExit = m_compCloud->getScalarFieldIndexByName(sfDims[j]->getName());
					if (sfExit >= 0)
						m_compCloud->deleteScalarField(sfExit);
					sfExit = m_compCloud->addScalarField(sfDims[j]);
					assert(sfExit >= 0);
				}
				ccLog::Warning("[ComputeDistances] Result has been split along each dimension (check the 3 other scalar fields with '_X', '_Y' and '_Z' suffix!)");
			}
			else
			{
				ccLog::Error("[ComputeDistances] Not enough memory to generate 3D split fields!");
			}

			sfDims[0]->release();
			sfDims[1]->release();
			sfDims[2]->release();
		}
	}
	else
	{
		ccLog::Error("[ComputeDistances] Error (%i)",result);
		
		m_compCloud->deleteScalarField(sfIdx);
		m_compCloud->showSF(false);
		sfIdx = -1;
		m_currentSFIsDistance = false;
	}

	updateDisplay(sfIdx >= 0, false);

	if (CPSet)
	{
		delete CPSet;
		CPSet = 0;
	}

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
		histogram->fromSF(sf,8,false);
		histogram->setAxisLabels("Approximate distances","Count");
	}
	hDlg->resize(400,300);
	hDlg->show();
}

void ccComparisonDlg::applyAndExit()
{
	if (m_compCloud)
	{
		//m_compCloud->setCurrentDisplayedScalarField(-1);
		//m_compCloud->showSF(false);

		//this SF shouldn't be here, but in any case, we get rid of it!
		int tmpSfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
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

		//we get rid of any temporary scalar field
		int tmpSfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
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
