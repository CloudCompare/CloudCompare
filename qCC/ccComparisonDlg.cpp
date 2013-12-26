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

ccComparisonDlg::ccComparisonDlg(ccHObject* compEntity, ccHObject* refEntity, CC_COMPARISON_TYPE cpType, QWidget* parent/* = 0*/, bool noDisplay/*=false*/)
	: QDialog(parent)
	, Ui::ComparisonDialog()
	, m_compEnt(compEntity)
	, m_refEnt(refEntity)
	, m_compOctree(0)
	, m_refOctree(0)
	, m_compCloud(0)
	, m_refCloud(0)
	, m_refMesh(0)
	, m_compType(cpType)
	, m_currentSFIsDistance(false)
	, m_noDisplay(noDisplay)
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
	ccBBox compEntBBox = compEntity->getBB();
	maxSearchDistSpinBox->setValue((double)compEntBBox.getDiagNorm());

	if (m_refMesh)
	{
		localModelingTab->setEnabled(false);
		signedDistFrame->setEnabled(true);
		signedDistCheckBox->setChecked(true);
	}
	else
	{
		split3DCheckBox->setEnabled(true);
		lmRadiusDoubleSpinBox->setValue((double)compEntBBox.getDiagNorm()/200.0);
	}

	//compute approximate results and unlock GUI
	computeApproxResults();
}

ccComparisonDlg::~ccComparisonDlg()
{
	clean();
}

bool ccComparisonDlg::prepareEntitiesForComparison()
{
	if (!m_compEnt || !m_refEnt)
		return false;

	//compared entity
	if (!m_compEnt->isA(CC_POINT_CLOUD)) //TODO --> pas possible avec des GenericPointCloud ? :(
	{
		if (m_compType == CLOUDCLOUD_DIST || (m_compType == CLOUDMESH_DIST && !m_compEnt->isKindOf(CC_MESH)))
		{
			ccLog::Error("Dialog initialization error! (bad entity type)");
			return false;
		}
		ccGenericMesh* compMesh = ccHObjectCaster::ToGenericMesh(m_compEnt);
		if (!compMesh->getAssociatedCloud()->isA(CC_POINT_CLOUD)) //TODO
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
	m_compOctree = static_cast<CCLib::DgmOctree*>(m_compCloud->getOctree());
	int oldSfIdx = m_compCloud->getCurrentDisplayedScalarFieldIndex();
	if (oldSfIdx>=0)
		m_oldSfName = QString(m_compCloud->getScalarFieldName(oldSfIdx));

	//on a toujours besoin du premier octree
	if (!m_compOctree)
		m_compOctree = new CCLib::DgmOctree(m_compCloud);

	//reference entity
	if ((m_compType == CLOUDMESH_DIST && !m_refEnt->isKindOf(CC_MESH))
		|| (m_compType == CLOUDCLOUD_DIST && !m_refEnt->isA(CC_POINT_CLOUD)))
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
		m_refOctree = static_cast<CCLib::DgmOctree*>(m_refCloud->getOctree());
		//on a besoin du deuxieme octree uniquement dans le cas de la distance nuage/nuage
		if (!m_refOctree)
			m_refOctree = new CCLib::DgmOctree(m_refCloud);
	}

	return true;
}

void ccComparisonDlg::updateOctreeLevel(double maxDistance)
{
	//we only compute best octree level if "auto" mode is on or the user has set the level to "0"
	if (!octreeLevelSpinBox->isEnabled() || octreeLevelSpinBox->value()==0)
	{
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

void ccComparisonDlg::clean()
{
	if (m_compOctree && m_compCloud)
	{
		if (!m_compCloud->getOctree())
			delete m_compOctree;
		m_compOctree = 0;
	}

	if (m_refOctree && m_refCloud)
	{
		if (!m_refCloud->getOctree())
			delete m_refOctree;
		m_refOctree = 0;
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
	MainWindow::RefreshAllGLWindow();
}

bool ccComparisonDlg::isValid()
{
	if (!m_compCloud || !m_compOctree || (!m_refMesh && !m_refCloud) || (!m_refMesh && !m_refOctree))
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
	approxGroupBox->setEnabled(false);

	if (!isValid())
		return approxResult;

	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx<0)
		sfIdx=m_compCloud->addScalarField(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx<0)
	{
		ccLog::Error("Failed to allocate a new scalar field for computing distances! Try to free some memory ...");
		return approxResult;
	}

	m_compCloud->setCurrentScalarField(sfIdx);
	CCLib::ScalarField* sf = m_compCloud->getCurrentInScalarField();
	assert(sf);

	//Preparation des octrees
	ccProgressDialog progressCb(true,this);

	QElapsedTimer eTimer;
	eTimer.start();
	switch(m_compType)
	{
	case CLOUDCLOUD_DIST: //hausdroff
		approxResult = CCLib::DistanceComputationTools::computeChamferDistanceBetweenTwoClouds(CHAMFER_345,m_compCloud,m_refCloud,DEFAULT_OCTREE_LEVEL,&progressCb,m_compOctree,m_refOctree);
		break;
	case CLOUDMESH_DIST: //cloud-mesh
		approxResult = CCLib::DistanceComputationTools::computePointCloud2MeshDistance(m_compCloud,m_refMesh,DEFAULT_OCTREE_LEVEL,-1.0,true,false,false,false,&progressCb,m_compOctree);
		break;
	}
	qint64 elapsedTime_ms = eTimer.elapsed();

	progressCb.stop();

	int guessedBestOctreeLevel = -1;

	//si le calcul de distances approx a echoue ...
	if (approxResult<0)
	{
		ccLog::Warning("Approx. results computation failed (error code %i)",approxResult);
		m_compCloud->deleteScalarField(sfIdx);
		sfIdx = -1;
		m_currentSFIsDistance = false;
	}
	else
	{
		ccLog::Print("[ComputeApproxDistances] Time: %3.2f s.",static_cast<double>(elapsedTime_ms)/1.0e3);

		//affichage des infos sur le champ scalaire approx.
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
		item = new QTableWidgetItem("Mean dist.");
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

		//on active les elements d'interface correspondant
		approxGroupBox->setEnabled(true);
		histoButton->setEnabled(true);

		//m.a.j. affichage
		m_compCloud->setCurrentDisplayedScalarField(sfIdx);
		m_compCloud->showSF(sfIdx>=0);

		m_currentSFIsDistance = true;

		//il faut determiner ici le niveau d'octree optimal pour le calcul precis
		guessedBestOctreeLevel = determineBestOctreeLevel(-1.0);
	}

	if (guessedBestOctreeLevel<0)
	{
		ccLog::Warning("Can't evaluate best computation level! Try to set it manually ...");
		octreeLevelCheckBox->setCheckState(Qt::Checked);
		octreeLevelSpinBox->setEnabled(true);
		guessedBestOctreeLevel = (int)DEFAULT_OCTREE_LEVEL;
	}
	octreeLevelSpinBox->setValue(guessedBestOctreeLevel);

	computeButton->setEnabled(true);

	//We set the button color to red, and the text to white
	QColor qRed(255,0,0);
	ccDisplayOptionsDlg::SetButtonColor(computeButton,qRed);
	QColor qWhite(255,255,255);
	ccDisplayOptionsDlg::SetButtonTextColor(computeButton,qWhite);

	preciseGroupBox->setEnabled(true);

	updateDisplay(sfIdx>=0, false);

	return guessedBestOctreeLevel;
}

int ccComparisonDlg::determineBestOctreeLevel(double maxSearchDist)
{
	if (!isValid())
		return -1;

	//make sure a valid SF is activated
	if (!m_currentSFIsDistance || m_compCloud->getCurrentOutScalarFieldIndex()<0)
	{
		//we must compute approx. results again
		//(this method will be called again)
		return computeApproxResults();
	}

	//Evaluation du "temps" de calcul pour chaque niveau d'octree
	double timings[CCLib::DgmOctree::MAX_OCTREE_LEVEL];
	memset(timings,0,sizeof(double)*CCLib::DgmOctree::MAX_OCTREE_LEVEL);

	//Pour le cas o� la reference est un maillage
	double meanTriangleSurface = 1.0;
	CCLib::GenericIndexedMesh* mesh = 0;
	if (!m_refOctree)
	{
		if (!m_refMesh)
		{
			ccLog::Error("Error: reference entity should be a mesh!");
			return -1;
		}
		mesh = static_cast<CCLib::GenericIndexedMesh*>(m_refMesh);
		if (!mesh || mesh->size()==0)
		{
			ccLog::Warning("Mesh is empty! Can't go further...");
			return -1;
		}
		//Surface totale du maillage
		double meshSurface = CCLib::MeshSamplingTools::computeMeshArea(mesh);
		//Surface moyenne d'un triangle
		if (meshSurface>0.0)
			meanTriangleSurface = meshSurface/double(mesh->size());
	}

	//On saute les niveaux les plus faibles, car il sont "inutiles" et font foirer les calculs suivants ;)
	int theBestOctreeLevel = 2;

	//Pour chaque niveau ...
	for (int level=2; level<CCLib::DgmOctree::MAX_OCTREE_LEVEL; ++level)
	{
		//Structures utiles pour le parcours de la structure octree
		const int bitDec = GET_BIT_SHIFT(level);
		unsigned numberOfPointsInCell = 0;
		unsigned index = 0;
		double cellDist = -1;

		//on calcule un facteur de correction qui va nous donner a partir de la distance
		//approximative une approximation (reelle) de la taille du voisinage necessaire
		//a inspecter au niveau courant
		PointCoordinateType cellSize = m_compOctree->getCellSize(static_cast<uchar>(level));

		//densite du nuage de reference (en points/cellule) s'il existe
		double refListDensity = 1.0;
		if (m_refOctree)
			refListDensity = m_refOctree->computeMeanOctreeDensity(static_cast<uchar>(level));

		CCLib::DgmOctree::OctreeCellCodeType tempCode = 0xFFFFFFFF;

		//On parcours la structure octree
		const CCLib::DgmOctree::cellsContainer& compCodes = m_compOctree->pointsAndTheirCellCodes();
		for (CCLib::DgmOctree::cellsContainer::const_iterator c=compCodes.begin();c!=compCodes.end();++c)
		{
			CCLib::DgmOctree::OctreeCellCodeType truncatedCode = (c->theCode >> bitDec);

			//nouvelle cellule
			if (truncatedCode != tempCode)
			{
				//si on a affaire a une vraie cellule
				if (numberOfPointsInCell != 0)
				{
					//si maxSearchDist est defini par l'utilisateur, il faut le prendre en compte !
					//(dans ce cas, on saute la cellule si sa distance approx. est superieure)
					if (maxSearchDist < 0 || cellDist <= maxSearchDist)
					{
						//approximation (flotante) de la taille du voisinage
						cellDist /= static_cast<double>(cellSize);

						//largeur (flotante) du voisinage (en nombre de cellules)
						double neighbourSize = 2.0*cellDist + 1.0;

						//si la reference est un maillage
						if (mesh)
						{
							//approximation (entiere) de la taille du voisinage
							//en nombre de cellules au niveau courant
							int nCell = static_cast<int>(ceil(cellDist));

							//surface probable de maillage rencontree dans ce
							//voisinage : largeur du voisinage au carre
							double crossingMeshSurface = (2.0*nCell+1.0) * static_cast<double>(cellSize);
							crossingMeshSurface*=crossingMeshSurface;

							//"volume" du voisinage (en nombre de cellules)
							double neighbourSize3 = neighbourSize*neighbourSize*neighbourSize;

							//TEMPS = RECHERCHE DES VOISINS + facteur_proportion * COMPARAISONS POINTS/TRIANGLES
							timings[level] += neighbourSize3 + 0.5 * static_cast<double>(numberOfPointsInCell) * crossingMeshSurface/meanTriangleSurface;
						}
						else
						{
							//on ignore la cellule "centrale"
							neighbourSize -= 1.0;
							//volume du voisinage (en nombre de cellules)
							double neighbourSize3 = neighbourSize*neighbourSize*neighbourSize;
							//volume de la derniere tranche (en nombre de cellules)
							//=V(n)-V(n-1) = (2*n+1)^3 - (2*n-1)^3 = 24 * n^2 + 2 (si n>0)
							double lastSliceCellNumber = (cellDist > 0 ? cellDist*cellDist*24.0+2.0 : 1.0);
							//TEMPS = RECHERCHE DES VOISINS + facteur_proportion * COMPARAISONS POINTS/TRIANGLES
							//(on estime que les cellules remplies representent la racine du total)
							timings[level] += neighbourSize3 + 0.1 * static_cast<double>(numberOfPointsInCell)*sqrt(lastSliceCellNumber)*refListDensity;
						}
					}
				}

				numberOfPointsInCell = 0;
				cellDist = m_compCloud->getPointScalarValue(index);
				tempCode = truncatedCode;
			}

			++index;
			++numberOfPointsInCell;
		}

		//ccLog::Print("[Timing] Level %i --> %f",level,timings[level]);

		if (timings[level]<timings[theBestOctreeLevel])
			theBestOctreeLevel = level;
	}

	ccLog::PrintDebug("[ccComparisonDlg] Best level: %i (maxSearchDist=%f)",theBestOctreeLevel,maxSearchDist);

	return theBestOctreeLevel;
}

bool ccComparisonDlg::compute()
{
	if (!isValid())
		return false;

	//updates best octree level guess if necessary
	updateOctreeLevel(maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
	int bestOctreeLevel = octreeLevelSpinBox->value();

	bool signedDistances = signedDistFrame->isEnabled() && signedDistCheckBox->isChecked();
	bool flipNormals = (signedDistances ? flipNormalsCheckBox->isChecked() : false);

	bool split3D = split3DCheckBox->isEnabled() && split3DCheckBox->isChecked();
	//for cloud-cloud distance, 'signedDistances' is only used for 'split3D' mode
	bool split3DSigned = signedDistances;

	//does the cloud has already a temporary scalar field that we can use?
	int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx<0)
		sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx>=0)
	{
		CCLib::ScalarField* sf = m_compCloud->getScalarField(sfIdx);
		assert(sf);

		//we recycle an existing scalar field, we must rename it
		sf->setName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	}

	//do we need to create a new scalar field?
	if (sfIdx<0)
	{
		sfIdx = m_compCloud->addScalarField(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx<0)
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
	ccProgressDialog progressCb(true,this);

	//for 3D splitting (cloud-cloud dist. only)
	CCLib::ReferenceCloud* CPSet=0;
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

		result = CCLib::DistanceComputationTools::computeHausdorffDistance(m_compCloud,
			m_refCloud,
			params,
			&progressCb,
			m_compOctree,
			m_refOctree);
		break;

	case CLOUDMESH_DIST: //cloud-mesh

		if (multiThread && maxSearchDistSpinBox->isEnabled())
			ccLog::Warning("[Cloud/Mesh comparison] Max search distance is not supported in multi-thread mode! Switching to single thread mode...");
		
		result = CCLib::DistanceComputationTools::computePointCloud2MeshDistance(	m_compCloud,
																					m_refMesh,
																					static_cast<uchar>(bestOctreeLevel),
																					maxSearchDist,
																					false,
																					signedDistances,
																					flipNormals,
																					multiThread,
																					&progressCb,
																					m_compOctree);
		break;
	}
	qint64 elapsedTime_ms = eTimer.elapsed();

	progressCb.stop();

	if (result >= 0)
	{
		ccLog::Print("[ComputeDistances] Time: %3.2f s.",static_cast<double>(elapsedTime_ms)/1.0e3);

		//affichage des infos sur le champ scalaire
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean,&variance);
		ccLog::Print("[ComputeDistances] Mean distance = %f / std deviation = %f",mean,sqrt(variance));

		m_compCloud->setCurrentDisplayedScalarField(sfIdx);
		m_compCloud->showSF(sfIdx>=0);

		//on retablit l'apparence du bouton
		okButton->setEnabled(true);

		m_sfName = QString();
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
					m_compCloud->addScalarField(sfDims[j]);
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
	}

	//m.a.j. affichage
	updateDisplay(sfIdx >= 0, false);

	if (CPSet)
		delete CPSet;
	CPSet = 0;

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
		histogram->setInfoStr(QString("Approximate distances (%1 values)").arg(m_compCloud->size()));
		histogram->fromSF(sf,8);
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
		if (tmpSfIdx>=0)
		{
			m_compCloud->deleteScalarField(tmpSfIdx);
			tmpSfIdx=-1;
		}

		//now, if we have a temp distance scalar field (the 'real' distances computed by the user)
		//we should rename it properly
		int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx>=0)
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
				if (_sfIdx>=0)
				{
					m_compCloud->deleteScalarField(_sfIdx);
					//we update sfIdx because indexes are all messed up after deletion
					sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
				}

				m_compCloud->renameScalarField(sfIdx,qPrintable(m_sfName));
				m_compCloud->setCurrentDisplayedScalarField(sfIdx);
				m_compCloud->showSF(sfIdx>=0);
			}
		}

		//m_compCloud->setCurrentDisplayedScalarField(-1);
		//m_compCloud->showSF(false);

	}

	updateDisplay(true, m_refVisibility);

	clean();

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
		if (tmpSfIdx>=0)
		{
			m_compCloud->deleteScalarField(tmpSfIdx);
			tmpSfIdx=-1;
		}

		int sfIdx = m_compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx>=0)
		{
			m_compCloud->deleteScalarField(sfIdx);
			sfIdx=-1;
		}

		if (!m_oldSfName.isEmpty())
		{
			int oldSfIdx = m_compCloud->getScalarFieldIndexByName(qPrintable(m_oldSfName));
			if (oldSfIdx)
			{
				m_compCloud->setCurrentDisplayedScalarField(oldSfIdx);
				m_compCloud->showSF(oldSfIdx>=0);
			}
		}
	}

	updateDisplay(m_compSFVisibility, m_refVisibility);

	clean();

	reject();
}
