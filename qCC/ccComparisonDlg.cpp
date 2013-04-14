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
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccOctree.h>
#include <ccProgressDialog.h>

//Local
#include "ccDisplayOptionsDlg.h"
#include "mainwindow.h"
#include "ccConsole.h"
#include "ccCommon.h"
#include "ccHistogramWindow.h"

//Qt
#include <QElapsedTimer>

//System
#include <assert.h>

const uchar DEFAULT_OCTREE_LEVEL = 7;

ccComparisonDlg::ccComparisonDlg(ccHObject* compEntity, ccHObject* refEntity, CC_COMPARISON_TYPE cpType, QWidget* parent/* = 0*/)
	: QDialog(parent)
	, Ui::ComparisonDialog()
	, compEnt(compEntity)
	, refEnt(refEntity)
	, compOctree(0)
	, refOctree(0)
	, compCloud(0)
	, refCloud(0)
	, refMesh(0)
	, compType(cpType)
	, sfCanBeUsedToEstimateBestOctreeLevel(false)
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
	//connect(octreeLevelCheckBox,	SIGNAL(stateChanged(int)),			this,	SLOT(guessBestOctreeLevel(int)));
	//connect(maxSearchDistSpinBox,	SIGNAL(valueChanged(double)),		this,	SLOT(updateOctreeLevel(double)));
	//connect(maxDistCheckBox,		SIGNAL(stateChanged(int)),			this,	SLOT(guessBestOctreeLevel(int)));
	connect(localModelComboBox,		SIGNAL(currentIndexChanged(int)),	this,	SLOT(locaModelChanged(int)));
	connect(split3DCheckBox,		SIGNAL(toggled(bool)),				this,	SLOT(split3DCheckboxToggled(bool)));

	octreeLevelSpinBox->setRange(1,CCLib::DgmOctree::MAX_OCTREE_LEVEL);
	compName->setText(compEnt->getName());
	refName->setText(refEnt->getName());
	//predMaxSearchDistValue = maxSearchDistSpinBox->value();
	preciseResultsTabWidget->setCurrentIndex(0);

	refVisibility = (refEnt ? refEnt->isVisible() : false);
	compSFVisibility = (compEnt ? compEnt->sfShown() : false);

	if (!prepareEntitiesForComparison())
		return;

	assert(compEntity);
	ccBBox compEntBBox = compEntity->getBB();
	maxSearchDistSpinBox->setValue((double)compEntBBox.getDiagNorm());

	if (refMesh)
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
	if (!compEnt || !refEnt)
		return false;

	//compared entity
	if (!compEnt->isA(CC_POINT_CLOUD)) //TODO --> pas possible avec des GenericPointCloud ? :(
	{
		if (compType == CLOUDCLOUD_DIST || (compType == CLOUDMESH_DIST && !compEnt->isKindOf(CC_MESH)))
		{
			ccConsole::Error("Dialog initialization error! (bad entity type)");
			return false;
		}
		ccGenericMesh* compMesh = static_cast<ccGenericMesh*>(compEnt);
		if (!compMesh->getAssociatedCloud()->isA(CC_POINT_CLOUD)) //TODO
		{
			ccConsole::Error("Dialog initialization error! (bad entity type - works only with real point clouds [todo])");
			return false;
		}
		compCloud = static_cast<ccPointCloud*>(compMesh->getAssociatedCloud());
	}
	else
	{
		compCloud = static_cast<ccPointCloud*>(compEnt);
	}
	compOctree = static_cast<CCLib::DgmOctree*>(compCloud->getOctree());
	int oldSfIdx = compCloud->getCurrentDisplayedScalarFieldIndex();
	if (oldSfIdx>=0)
		oldSfName = QString(compCloud->getScalarFieldName(oldSfIdx));

	//on a toujours besoin du premier octree
	if (!compOctree)
		compOctree = new CCLib::DgmOctree(compCloud);

	//reference entity
	if ((compType == CLOUDMESH_DIST && !refEnt->isKindOf(CC_MESH))
		|| (compType == CLOUDCLOUD_DIST && !refEnt->isA(CC_POINT_CLOUD)))
	{
		ccConsole::Error("Dialog initialization error! (bad entity type)");
		return false;
	}

	if (compType == CLOUDMESH_DIST)
	{
		refMesh = static_cast<ccGenericMesh*>(refEnt);
		refCloud = refMesh->getAssociatedCloud();
		refOctree = 0;
	}
	else /*if (compType == CLOUDCLOUD_DIST)*/
	{
		refCloud = static_cast<ccGenericPointCloud*>(refEnt);
		refOctree = static_cast<CCLib::DgmOctree*>(refCloud->getOctree());
		//on a besoin du deuxieme octree uniquement dans le cas de la distance nuage/nuage
		if (!refOctree)
			refOctree = new CCLib::DgmOctree(refCloud);
	}

	return true;
}

void ccComparisonDlg::updateOctreeLevel(double value)
{
	//we only compute best octree level if "auto" mode is on or the user has set the level to "0"
	if (!octreeLevelSpinBox->isEnabled() || octreeLevelSpinBox->value()==0)
	{
		//predMaxSearchDistValue = maxSearchDistSpinBox->value();
		int guessedBestOctreeLevel = determineBestOctreeLevelForDistanceComputation(value);
		if (guessedBestOctreeLevel>0)
		{
			octreeLevelSpinBox->setValue(guessedBestOctreeLevel);
		}
		else
		{
			ccConsole::Error("Can't evaluate best computation level! Try to set it manually ...");
			octreeLevelCheckBox->setCheckState(Qt::Checked);
			octreeLevelSpinBox->setEnabled(true);
		}
	}
}

void ccComparisonDlg::split3DCheckboxToggled(bool state)
{
	if (compType == CLOUDMESH_DIST)
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

void ccComparisonDlg::guessBestOctreeLevel(int state)
{
	if (state == Qt::Unchecked || QObject::sender() == maxDistCheckBox)
		updateOctreeLevel(maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
}

void ccComparisonDlg::clean()
{
	if (compOctree && compCloud)
	{
		if (!compCloud->getOctree())
			delete compOctree;
		compOctree = 0;
	}

	if (refOctree && refCloud)
	{
		if (!refCloud->getOctree())
			delete refOctree;
		refOctree = 0;
	}
}

void ccComparisonDlg::updateDisplay(bool showSF, bool showRef)
{
	if (compEnt)
	{
		compEnt->setVisible(true);
		compEnt->setEnabled(true);
		compEnt->showSF(showSF);
		compEnt->prepareDisplayForRefresh_recursive();
	}

	if (refEnt)
	{
		refEnt->setVisible(showRef);
		refEnt->prepareDisplayForRefresh_recursive();
	}

	MainWindow::UpdateUI();
	MainWindow::RefreshAllGLWindow();
}

bool ccComparisonDlg::isValid()
{
	if (!compCloud || !compOctree || (!refMesh && !refCloud) || (!refMesh && !refOctree))
	{
		ccConsole::Error("Dialog initialization error! (void entity)");
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

	int sfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx<0)
		sfIdx=compCloud->addScalarField(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx<0)
	{
		ccConsole::Error("Failed to allocate a new scalar field for computing distances! Try to free some memory ...");
		return approxResult;
	}

	compCloud->setCurrentScalarField(sfIdx);
	CCLib::ScalarField* sf = compCloud->getCurrentInScalarField();
	assert(sf);

	//Preparation des octrees
	ccProgressDialog progressCb(true,this);

	QElapsedTimer eTimer;
	eTimer.start();
	switch(compType)
	{
	case CLOUDCLOUD_DIST: //hausdroff
		approxResult = CCLib::DistanceComputationTools::computeChamferDistanceBetweenTwoClouds(CHAMFER_345,compCloud,refCloud,DEFAULT_OCTREE_LEVEL,&progressCb,compOctree,refOctree);
		break;
	case CLOUDMESH_DIST: //cloud-mesh
		approxResult = CCLib::DistanceComputationTools::computePointCloud2MeshDistance(compCloud,refMesh,DEFAULT_OCTREE_LEVEL,-1.0,true,false,false,false,&progressCb,compOctree);
		break;
	}
	int elapsedTime_ms = eTimer.elapsed();

	progressCb.stop();

	int guessedBestOctreeLevel = -1;

	//si le calcul de distances approx a echoue ...
	if (approxResult<0)
	{
		ccConsole::Warning("Approx. results computation failed (error code %i)",approxResult);
		compCloud->deleteScalarField(sfIdx);
		sfIdx = -1;
		sfCanBeUsedToEstimateBestOctreeLevel = false;
	}
	else
	{
		ccConsole::Print("[ComputeApproxDistances] Time: %3.2f s.",elapsedTime_ms/1.0e3);

		//affichage des infos sur le champ scalaire approx.
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean,&variance);
		//precision
		PointCoordinateType cs = compOctree->getCellSize(DEFAULT_OCTREE_LEVEL);
		float e1 = 100.0*(1.0-sqrt(25.0/27.0));
		float e2 = 100.0*cs;

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
		compCloud->setCurrentDisplayedScalarField(sfIdx);
		compCloud->showSF(sfIdx>=0);

		sfCanBeUsedToEstimateBestOctreeLevel = true;

		//il faut determiner ici le niveau d'octree optimal pour le calcul precis
		guessedBestOctreeLevel = determineBestOctreeLevelForDistanceComputation(-1.0);
	}

	if (guessedBestOctreeLevel<0)
	{
		ccConsole::Warning("Can't evaluate best computation level! Try to set it manually ...");
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

int ccComparisonDlg::determineBestOctreeLevelForDistanceComputation(ScalarType maxSearchDist)
{
	if (!isValid())
		return -1;

	//make sure a valid SF is activated
	if (!sfCanBeUsedToEstimateBestOctreeLevel || compCloud->getCurrentOutScalarFieldIndex()<0)
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
	if (!refOctree)
	{
		if (!refMesh)
		{
			ccConsole::Error("Error: reference entity should be a mesh!");
			return -1;
		}
		mesh = static_cast<CCLib::GenericIndexedMesh*>(refMesh);
		if (!mesh || mesh->size()==0)
		{
			ccConsole::Warning("Mesh is empty! Can't go further...");
			return -1;
		}
		//Surface totale du maillage
		double meshSurface = CCLib::MeshSamplingTools::computeMeshArea(mesh);
		//Surface moyenne d'un triangle
		if (meshSurface>0.0)
			meanTriangleSurface = meshSurface/double(mesh->size());
	}

	//On saute les niveaux les plus faibles, car il sont "inutiles" et font foirer les calculs suivants ;)
	int theBestOctreeLevel=2;

	//Pour chaque niveau ...
	for (unsigned char level=2; level<CCLib::DgmOctree::MAX_OCTREE_LEVEL; ++level)
	{
		//Structures utiles pour le parcours de la structure octree
		const uchar bitDec = GET_BIT_SHIFT(level);
		unsigned numberOfPointsInCell=0;
		unsigned index=0;
		ScalarType cellDist=-1.0;

		//on calcule un facteur de correction qui va nous donner a partir de la distance
		//approximative une approximation (reelle) de la taille du voisinage necessaire
		//a inspecter au niveau courant
		ScalarType cellSize = compOctree->getCellSize(level);

		//densite du nuage de reference (en points/cellule) s'il existe
		double refListDensity=1.0;
		if (refOctree)
			refListDensity = refOctree->computeMeanOctreeDensity(level);

		CCLib::DgmOctree::OctreeCellCodeType tempCode = 0xFFFFFFFF;

		//On parcours la structure octree
		const CCLib::DgmOctree::cellsContainer& compCodes = compOctree->pointsAndTheirCellCodes();
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
						cellDist /= cellSize;

						//largeur (flotante) du voisinage (en nombre de cellules)
						double neighbourSize = (double)(2.0*cellDist+1.0);

						//si la reference est un maillage
						if (mesh)
						{
							//approximation (entiere) de la taille du voisinage
							//en nombre de cellules au niveau courant
							int nCell = (int)ceil(cellDist);

							//surface probable de maillage rencontree dans ce
							//voisinage : largeur du voisinage au carre
							double crossingMeshSurface = (2.0*nCell+1.0)*cellSize;
							crossingMeshSurface*=crossingMeshSurface;

							//"volume" du voisinage (en nombre de cellules)
							double neighbourSize3 = neighbourSize*neighbourSize*neighbourSize;

							//TEMPS = RECHERCHE DES VOISINS + facteur_proportion * COMPARAISONS POINTS/TRIANGLES
							timings[level] += neighbourSize3 + 0.5 * double(numberOfPointsInCell)*crossingMeshSurface/meanTriangleSurface;
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
							timings[level] += neighbourSize3 + 0.1 * double(numberOfPointsInCell)*sqrt(lastSliceCellNumber)*refListDensity;
						}
					}
				}

				numberOfPointsInCell = 0;
				cellDist = compCloud->getPointScalarValue(index);
				tempCode = truncatedCode;
			}

			++index;
			++numberOfPointsInCell;
		}

		//ccConsole::Print("[Timing] Level %i --> %f",level,timings[level]);

		if (timings[level]<timings[theBestOctreeLevel])
			theBestOctreeLevel = level;
	}

	ccConsole::PrintDebug("[ccComparisonDlg] Best level: %i (maxSearchDist=%f)",theBestOctreeLevel,maxSearchDist);

	return theBestOctreeLevel;
}

void ccComparisonDlg::compute()
{
	if (!isValid())
		return;

	//best octree level should be up-to-date
	updateOctreeLevel(maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
	int bestOctreeLevel = octreeLevelSpinBox->value();

	bool signedDistances = signedDistFrame->isEnabled() && signedDistCheckBox->isChecked();
	bool flipNormals = (signedDistances ? flipNormalsCheckBox->isChecked() : false);

	bool split3D = split3DCheckBox->isEnabled() && split3DCheckBox->isChecked();
	//for cloud-cloud distance, 'signedDistances' is only used for 'split3D' mode
	bool split3DSigned = signedDistances;

	//does the cloud has already a temporary scalar field that we can use?
	int sfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx<0)
		sfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	if (sfIdx>=0)
	{
		CCLib::ScalarField* sf = compCloud->getScalarField(sfIdx);
		assert(sf);

		//we recycle an existing scalar field, we must rename it
		sf->setName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
	}

	//do we need to create a new scalar field?
	if (sfIdx<0)
	{
		sfIdx = compCloud->addScalarField(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx<0)
		{
			ccConsole::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
			return;
		}
	}

	compCloud->setCurrentScalarField(sfIdx);
	CCLib::ScalarField* sf = compCloud->getCurrentInScalarField();
	assert(sf);

	//max search distance
	ScalarType maxSearchDist = (maxSearchDistSpinBox->isEnabled() ? maxSearchDistSpinBox->value() : -1.0);
	//multi-thread
	bool enableMT = multiThreadedCheckBox->isChecked();

	int result=-1;
	ccProgressDialog progressCb(true,this);

	//for 3D splitting (cloud-cloud dist. only)
	CCLib::ReferenceCloud* CPSet=0;
	if (split3D)
	{
		assert(refCloud);
		CPSet = new CCLib::ReferenceCloud(refCloud);
	}

	CCLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams params;
	params.octreeLevel = bestOctreeLevel;
	if (localModelingTab->isEnabled())
	{
		params.localModel = (CC_LOCAL_MODEL_TYPES)localModelComboBox->currentIndex();
		if (params.localModel != NO_MODEL)
		{
			params.useSphericalSearchForLocalModel = lmRadiusRadioButton->isChecked();
			params.kNNForLocalModel = lmKNNSpinBox->value();
			params.radiusForLocalModel = lmRadiusDoubleSpinBox->value();
			params.reuseExistingLocalModels = lmOptimizeCheckBox->isChecked();
		}
	}
	params.maxSearchDist = maxSearchDist;
	params.multiThread = enableMT;
	params.CPSet = CPSet;

	QElapsedTimer eTimer;
	eTimer.start();
	switch(compType)
	{
	case CLOUDCLOUD_DIST: //hausdorff
		result = CCLib::DistanceComputationTools::computeHausdorffDistance(compCloud,
			refCloud,
			params,
			&progressCb,
			compOctree,
			refOctree);
		break;
	case CLOUDMESH_DIST: //cloud-mesh
		if (enableMT && maxSearchDistSpinBox->isEnabled())
			ccConsole::Warning("[Cloud/Mesh comparison] Max search distance is not supported in multi-thread mode! Switching to single thread mode...");
		result = CCLib::DistanceComputationTools::computePointCloud2MeshDistance(compCloud,
			refMesh,
			bestOctreeLevel,
			maxSearchDist,
			false,
			signedDistances,
			flipNormals,
			enableMT,
			&progressCb,
			compOctree);
		break;
	}
	int elapsedTime_ms = eTimer.elapsed();

	progressCb.stop();

	if (result>=0)
	{
		ccConsole::Print("[ComputeDistances] Time: %3.2f s.",elapsedTime_ms/1.0e3);

		//affichage des infos sur le champ scalaire
		ScalarType mean,variance;
		sf->computeMinAndMax();
		sf->computeMeanAndVariance(mean,&variance);
		ccConsole::Print("[ComputeDistances] Mean distance = %f / std deviation = %f",mean,sqrt(variance));

		compCloud->setCurrentDisplayedScalarField(sfIdx);
		compCloud->showSF(sfIdx>=0);

		//on retablit l'apparence du bouton
		okButton->setEnabled(true);

		sfName = QString();
		switch(compType)
		{
		case CLOUDCLOUD_DIST: //hausdorff
			sfName = QString(CC_CLOUD2CLOUD_DISTANCES_DEFAULT_SF_NAME);
			break;
		case CLOUDMESH_DIST: //cloud-mesh
			sfName = QString(signedDistances ? CC_CLOUD2MESH_SIGNED_DISTANCES_DEFAULT_SF_NAME : CC_CLOUD2MESH_DISTANCES_DEFAULT_SF_NAME);
			break;
		}

		if (params.localModel != NO_MODEL)
		{
			sfName += QString("[%1]").arg(localModelComboBox->currentText());
			if (params.useSphericalSearchForLocalModel)
				sfName += QString("[r=%1]").arg(params.radiusForLocalModel);
			else
				sfName += QString("[k=%1]").arg(params.kNNForLocalModel);
			if (params.reuseExistingLocalModels)
				sfName += QString("[fast]");
		}

		if (flipNormals)
			sfName += QString("[-]");

		sfCanBeUsedToEstimateBestOctreeLevel = true;

		if (maxSearchDist >= 0)
		{
			sfName += QString("[<%1]").arg(maxSearchDist);
			sfCanBeUsedToEstimateBestOctreeLevel = false;
		}

		if (split3D)
		{
			//we create 3 new scalar fields, one for each dimension
			assert(CPSet && CPSet->size() == compCloud->size());
			unsigned i,count=CPSet->size();
			ccScalarField* sfDims[3]= {	new ccScalarField(qPrintable(sfName+QString(" (X)"))),
										new ccScalarField(qPrintable(sfName+QString(" (Y)"))),
										new ccScalarField(qPrintable(sfName+QString(" (Z)")))
			};

			for (i=0;i<3;++i)
			{
				sfDims[i]->link();
			}

			if (sfDims[0]->reserve(count) &&
				sfDims[1]->reserve(count) &&
				sfDims[2]->reserve(count))
			{
				for (i=0;i<count;++i)
				{
					const CCVector3* P = CPSet->getPoint(i);
					const CCVector3* Q = compCloud->getPoint(i);
					CCVector3 D = *Q-*P;

					sfDims[0]->setValue(i,split3DSigned ? D.x : fabs(D.x));
					sfDims[1]->setValue(i,split3DSigned ? D.y : fabs(D.y));
					sfDims[2]->setValue(i,split3DSigned ? D.z : fabs(D.z));
				}

				for (i=0;i<3;++i)
				{
					sfDims[i]->computeMinAndMax();
					//check that SF doesn't already exist
					int sfExit = compCloud->getScalarFieldIndexByName(sfDims[i]->getName());
					if (sfExit>=0)
						compCloud->deleteScalarField(sfExit);
					compCloud->addScalarField(sfDims[i]);
				}
				ccConsole::Warning("[ComputeDistances] Result has been splitted along each dimension (check the 3 other scalar fields with '_X', '_Y' and '_Z' suffix!)");
			}
			else
			{
				ccConsole::Error("[ComputeDistances] Not enough memory to generate 3D splitted fields!");
			}

			for (i=0;i<3;++i)
			{
				sfDims[i]->release();
			}
		}
	}
	else
	{
		ccConsole::Error("[ComputeDistances] Error (%i)",result);
		compCloud->deleteScalarField(sfIdx);
		compCloud->showSF(false);
		sfIdx=-1;
	}

	//m.a.j. affichage
	updateDisplay(sfIdx>=0, false);

	if (CPSet)
		delete CPSet;
	CPSet=0;
}

void ccComparisonDlg::showHisto()
{
	if (!compCloud)
		return;

	ccScalarField* sf = compCloud->getCurrentDisplayedScalarField();
	if (!sf)
		return;

	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setWindowTitle(QString("Histogram [%1]").arg(compCloud->getName()));
	{
		ccHistogramWindow* histogram = hDlg->window();
		histogram->setInfoStr(QString("Approximate distances (%1 values)").arg(compCloud->size()));
		histogram->fromSF(sf,8);
	}
	hDlg->resize(400,300);
	hDlg->show();
}

void ccComparisonDlg::applyAndExit()
{
	if (compCloud)
	{
		//compCloud->setCurrentDisplayedScalarField(-1);
		//compCloud->showSF(false);

		//this shouldn't be here, but in any case, we get rid of it!
		int tmpSfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
		if (tmpSfIdx>=0)
		{
			compCloud->deleteScalarField(tmpSfIdx);
			tmpSfIdx=-1;
		}

		//now, if we have a temp distance scalar field (the 'real' distances computed by the user)
		//we should rename it properly
		int sfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx>=0)
		{
			if (sfName.isEmpty()) //hum,hum
			{
				ccConsole::Warning("Something went wrong!");
				compCloud->deleteScalarField(sfIdx);
				compCloud->setCurrentDisplayedScalarField(-1);
				compCloud->showSF(false);
			}
			else
			{
				//we delete any existing scalar field with the exact same name
				int _sfIdx = compCloud->getScalarFieldIndexByName(qPrintable(sfName));
				if (_sfIdx>=0)
				{
					compCloud->deleteScalarField(_sfIdx);
					//we update sfIdx because indexes are all messed up after deletion
					sfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
				}

				compCloud->renameScalarField(sfIdx,qPrintable(sfName));
				compCloud->setCurrentDisplayedScalarField(sfIdx);
				compCloud->showSF(sfIdx>=0);
			}
		}

		//compCloud->setCurrentDisplayedScalarField(-1);
		//compCloud->showSF(false);

	}

	updateDisplay(true, refVisibility);

	clean();

	accept();
}

void ccComparisonDlg::cancelAndExit()
{
	if (compCloud)
	{
		compCloud->setCurrentDisplayedScalarField(-1);
		compCloud->showSF(false);

		//we get rid of any temporary scalar field
		int tmpSfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_CHAMFER_DISTANCES_DEFAULT_SF_NAME);
		if (tmpSfIdx>=0)
		{
			compCloud->deleteScalarField(tmpSfIdx);
			tmpSfIdx=-1;
		}

		int sfIdx = compCloud->getScalarFieldIndexByName(CC_TEMP_DISTANCES_DEFAULT_SF_NAME);
		if (sfIdx>=0)
		{
			compCloud->deleteScalarField(sfIdx);
			sfIdx=-1;
		}

		if (!oldSfName.isEmpty())
		{
			int oldSfIdx = compCloud->getScalarFieldIndexByName(qPrintable(oldSfName));
			if (oldSfIdx)
			{
				compCloud->setCurrentDisplayedScalarField(oldSfIdx);
				compCloud->showSF(oldSfIdx>=0);
			}
		}
	}

	updateDisplay(compSFVisibility, refVisibility);

	clean();

	reject();
}
