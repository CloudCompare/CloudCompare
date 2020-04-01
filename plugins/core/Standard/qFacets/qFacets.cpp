//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFacets                       #
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
//#                      COPYRIGHT: Thomas Dewez, BRGM                     #
//#                                                                        #
//##########################################################################

#include "qFacets.h"

//Local
#include "facetsClassifier.h"
#include "classificationParamsDlg.h"
#include "facetsExportDlg.h"
#include "stereogramDlg.h"
#include "kdTreeForFacetExtraction.h"
#include "fastMarchingForFacetExtraction.h"
#include "disclaimerDialog.h"

//Qt
#include <QtGui>
#include <QInputDialog>
#include <QElapsedTimer>
#include <QSettings>
#include <QFileInfo>
#include <QMessageBox>

//qCC_db
#include <ccFileUtils.h>
#include <ccHObjectCaster.h>
#include <ccMesh.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccOctree.h> //for ComputeAverageNorm
#include <ShpDBFFields.h>

//qCC_io
#include <ShpFilter.h>


//semi-persistent dialog values
static unsigned s_octreeLevel = 8;
static bool		s_fmUseRetroProjectionError = false;

static unsigned	s_minPointsPerFacet = 10;
static double	s_errorMaxPerFacet = 0.2;
static int		s_errorMeasureType = 3; //max dist @ 99 %
static double	s_maxEdgeLength = 1.0;

static double	s_kdTreeFusionMaxAngle_deg = 20.0;
static double	s_kdTreeFusionMaxRelativeDistance = 1.0;

static double	s_classifAngleStep = 30.0;
static double	s_classifMaxDist = 1.0;

static double	s_stereogramAngleStep = 30.0;
static double	s_stereogramResolution_deg = 5.0;
static ccPointCloud* s_lastCloud = nullptr;


qFacets::qFacets(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qFacets/info.json" )
	, m_doFuseKdTreeCells(nullptr)
	, m_fastMarchingExtraction(nullptr)
	, m_doExportFacets(nullptr)
	, m_doExportFacetsInfo(nullptr)
	, m_doClassifyFacetsByAngle(nullptr)
	, m_doShowStereogram(nullptr)
	, m_stereogramDialog(nullptr)
{
}

QList<QAction *> qFacets::getActions()
{	
	//actions
	if (!m_doFuseKdTreeCells)
	{
		m_doFuseKdTreeCells = new QAction("Extract facets (Kd-tree)", this);
		m_doFuseKdTreeCells->setToolTip("Detect planar facets by fusing Kd-tree cells");
		m_doFuseKdTreeCells->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qFacets/images/extractKD.png")));
		//connect signal
		connect(m_doFuseKdTreeCells, &QAction::triggered, this, &qFacets::fuseKdTreeCells);
	}

	if (!m_fastMarchingExtraction)
	{
		m_fastMarchingExtraction = new QAction("Extract facets (Fast Marching)", this);
		m_fastMarchingExtraction->setToolTip("Detect planar facets with Fast Marching");
		m_fastMarchingExtraction->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qFacets/images/extractFM.png")));
		//connect signal
		connect(m_fastMarchingExtraction, &QAction::triggered, this, &qFacets::extractFacetsWithFM);
	}

	if (!m_doExportFacets)
	{
		m_doExportFacets = new QAction("Export facets (SHP)", this);
		m_doExportFacets->setToolTip("Exports one or several facets to a shapefile");
		m_doExportFacets->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qFacets/images/shpFile.png")));
		//connect signal
		connect(m_doExportFacets, &QAction::triggered, this, &qFacets::exportFacets);
	}

	if (!m_doExportFacetsInfo)
	{
		m_doExportFacetsInfo = new QAction("Export facets info (CSV)", this);
		m_doExportFacetsInfo->setToolTip("Exports various information on a set of facets (ASCII CSV file)");
		m_doExportFacetsInfo->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qFacets/images/csvFile.png")));
		//connect signal
		connect(m_doExportFacetsInfo, &QAction::triggered, this, &qFacets::exportFacetsInfo);
	}

	if (!m_doClassifyFacetsByAngle)
	{
		m_doClassifyFacetsByAngle = new QAction("Classify facets by orientation", this);
		m_doClassifyFacetsByAngle->setToolTip("Classifies facets based on their orienation (dip & dip direction)");
		m_doClassifyFacetsByAngle->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qFacets/images/classifIcon.png")));
		//connect signal
		connect(m_doClassifyFacetsByAngle, &QAction::triggered, this, [=] () {
			classifyFacetsByAngle();
		} );
	}

	if (!m_doShowStereogram)
	{
		m_doShowStereogram = new QAction("Show stereogram", this);
		m_doShowStereogram->setToolTip("Computes and displays a stereogram (+ interactive filtering)");
		m_doShowStereogram->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qFacets/images/stereogram.png")));
		//connect signal
		connect(m_doShowStereogram, &QAction::triggered, this, &qFacets::showStereogram);
	}

	return QList<QAction *>{
				m_doFuseKdTreeCells,
				m_fastMarchingExtraction,
				m_doExportFacets,
				m_doExportFacetsInfo,
				m_doClassifyFacetsByAngle,
				m_doShowStereogram,
	};
}

void qFacets::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_doFuseKdTreeCells)
		m_doFuseKdTreeCells->setEnabled(selectedEntities.size() == 1 && selectedEntities.back()->isA(CC_TYPES::POINT_CLOUD));
	if (m_fastMarchingExtraction)
		m_fastMarchingExtraction->setEnabled(selectedEntities.size() == 1 && selectedEntities.back()->isA(CC_TYPES::POINT_CLOUD));
	if (m_doExportFacets)
		m_doExportFacets->setEnabled(selectedEntities.size() != 0);
	if (m_doExportFacetsInfo)
		m_doExportFacetsInfo->setEnabled(selectedEntities.size() != 0);
	if (m_doClassifyFacetsByAngle)
		m_doClassifyFacetsByAngle->setEnabled(selectedEntities.size() == 1 && selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT));
	if (m_doShowStereogram)
		m_doShowStereogram->setEnabled(selectedEntities.size() == 1 && (selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT) || selectedEntities.back()->isA(CC_TYPES::POINT_CLOUD)));
}

void qFacets::extractFacetsWithFM()
{
	extractFacets(CellsFusionDlg::ALGO_FAST_MARCHING);
}

void qFacets::fuseKdTreeCells()
{
	extractFacets(CellsFusionDlg::ALGO_KD_TREE);
}

void qFacets::extractFacets(CellsFusionDlg::Algorithm algo)
{
	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	assert(m_app);
	if (!m_app)
		return;

	//we expect a unique cloud as input
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	ccPointCloud* pc = (m_app->haveOneSelection() ? ccHObjectCaster::ToPointCloud(selectedEntities.back()) : nullptr);
	if (!pc)
	{
		m_app->dispToConsole("Select one and only one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (algo != CellsFusionDlg::ALGO_FAST_MARCHING && algo != CellsFusionDlg::ALGO_KD_TREE)
	{
		m_app->dispToConsole("Internal error: invalid algorithm type!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//first time: we compute the max edge length automatically
	if (s_lastCloud != pc)
	{
		s_maxEdgeLength = static_cast<double>(pc->getOwnBB().getMinBoxDim()) / 50;
		s_minPointsPerFacet = std::max<unsigned>(pc->size() / 100000, 10);
		s_lastCloud = pc;
	}

	CellsFusionDlg fusionDlg(algo, m_app->getMainWindow());
	if (algo == CellsFusionDlg::ALGO_FAST_MARCHING)
		fusionDlg.octreeLevelSpinBox->setCloud(pc);

	fusionDlg.octreeLevelSpinBox->setValue(s_octreeLevel);
	fusionDlg.useRetroProjectionCheckBox->setChecked(s_fmUseRetroProjectionError);
	fusionDlg.minPointsPerFacetSpinBox->setValue(s_minPointsPerFacet);
	fusionDlg.errorMeasureComboBox->setCurrentIndex(s_errorMeasureType);
	fusionDlg.maxRMSDoubleSpinBox->setValue(s_errorMaxPerFacet);
	fusionDlg.maxAngleDoubleSpinBox->setValue(s_kdTreeFusionMaxAngle_deg);
	fusionDlg.maxRelativeDistDoubleSpinBox->setValue(s_kdTreeFusionMaxRelativeDistance);
	fusionDlg.maxEdgeLengthDoubleSpinBox->setValue(s_maxEdgeLength);
	//"no normal" warning
	fusionDlg.noNormalWarningLabel->setVisible(!pc->hasNormals());

	if (!fusionDlg.exec())
		return;

	s_octreeLevel = fusionDlg.octreeLevelSpinBox->value();
	s_fmUseRetroProjectionError = fusionDlg.useRetroProjectionCheckBox->isChecked();
	s_minPointsPerFacet = fusionDlg.minPointsPerFacetSpinBox->value();
	s_errorMeasureType = fusionDlg.errorMeasureComboBox->currentIndex();
	s_errorMaxPerFacet = fusionDlg.maxRMSDoubleSpinBox->value();
	s_kdTreeFusionMaxAngle_deg = fusionDlg.maxAngleDoubleSpinBox->value();
	s_kdTreeFusionMaxRelativeDistance = fusionDlg.maxRelativeDistDoubleSpinBox->value();
	s_maxEdgeLength = fusionDlg.maxEdgeLengthDoubleSpinBox->value();

	//convert 'errorMeasureComboBox' index to enum
	CCLib::DistanceComputationTools::ERROR_MEASURES errorMeasure = CCLib::DistanceComputationTools::RMS;
	switch (s_errorMeasureType)
	{
	case 0:
		errorMeasure = CCLib::DistanceComputationTools::RMS;
		break;
	case 1:
		errorMeasure = CCLib::DistanceComputationTools::MAX_DIST_68_PERCENT;
		break;
	case 2:
		errorMeasure = CCLib::DistanceComputationTools::MAX_DIST_95_PERCENT;
		break;
	case 3:
		errorMeasure = CCLib::DistanceComputationTools::MAX_DIST_99_PERCENT;
		break;
	case 4:
		errorMeasure = CCLib::DistanceComputationTools::MAX_DIST;
		break;
	default:
		assert(false);
		break;
	}

	//create scalar field to host the fusion result
	const char c_defaultSFName[] = "facet indexes";
	int sfIdx = pc->getScalarFieldIndexByName(c_defaultSFName);
	if (sfIdx < 0)
		sfIdx = pc->addScalarField(c_defaultSFName);
	if (sfIdx < 0)
	{
		m_app->dispToConsole("Couldn't allocate a new scalar field for computing fusion labels! Try to free some memory ...", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	pc->setCurrentScalarField(sfIdx);

	//computation
	QElapsedTimer eTimer;
	eTimer.start();
	ccProgressDialog pDlg(true, m_app->getMainWindow());

	bool success = true;
	if (algo == CellsFusionDlg::ALGO_KD_TREE)
	{
		//we need a kd-tree
		QElapsedTimer eTimer;
		eTimer.start();
		ccKdTree kdtree(pc);

		if (kdtree.build(s_errorMaxPerFacet / 2, errorMeasure, s_minPointsPerFacet, 1000, &pDlg))
		{
			qint64 elapsedTime_ms = eTimer.elapsed();
			m_app->dispToConsole(QString("[qFacets] Kd-tree construction timing: %1 s").arg(static_cast<double>(elapsedTime_ms) / 1.0e3, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);

			success = ccKdTreeForFacetExtraction::FuseCells(
				&kdtree,
				s_errorMaxPerFacet,
				errorMeasure,
				s_kdTreeFusionMaxAngle_deg,
				static_cast<PointCoordinateType>(s_kdTreeFusionMaxRelativeDistance),
				true,
				&pDlg);
		}
		else
		{
			m_app->dispToConsole("Failed to build Kd-tree! (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			success = false;
		}
	}
	else if (algo == CellsFusionDlg::ALGO_FAST_MARCHING)
	{
		int result = FastMarchingForFacetExtraction::ExtractPlanarFacets(
			pc,
			static_cast<unsigned char>(s_octreeLevel),
			static_cast<ScalarType>(s_errorMaxPerFacet),
			errorMeasure,
			s_fmUseRetroProjectionError,
			&pDlg,
			pc->getOctree().data());

		success = (result >= 0);
	}

	if (success)
	{
		pc->setCurrentScalarField(sfIdx); //for AutoSegmentationTools::extractConnectedComponents

		CCLib::ReferenceCloudContainer components;
		if (!CCLib::AutoSegmentationTools::extractConnectedComponents(pc, components))
		{
			m_app->dispToConsole("Failed to extract fused components! (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		else
		{
			//we remove the temporary scalar field (otherwise it will be copied to the sub-clouds!)
			ccScalarField* indexSF = static_cast<ccScalarField*>(pc->getScalarField(sfIdx));
			indexSF->link(); //to prevent deletion below
			pc->deleteScalarField(sfIdx);
			sfIdx = -1;

			bool error = false;
			ccHObject* group = createFacets(pc, components, s_minPointsPerFacet, s_maxEdgeLength, false, error);

			if (group)
			{
				switch (algo)
				{
				case CellsFusionDlg::ALGO_KD_TREE:
					group->setName(group->getName() + QString(" [Kd-tree][error < %1][angle < %2 deg.]").arg(s_errorMaxPerFacet).arg(s_kdTreeFusionMaxAngle_deg));
					break;
				case CellsFusionDlg::ALGO_FAST_MARCHING:
					group->setName(group->getName() + QString(" [FM][level %2][error < %1]").arg(s_octreeLevel).arg(s_errorMaxPerFacet));
					break;
				default:
					break;
				}

				unsigned count = group->getChildrenNumber();
				m_app->dispToConsole(QString("[qFacets] %1 facet(s) where created from cloud '%2'").arg(count).arg(pc->getName()));

				if (error)
				{
					m_app->dispToConsole("Error(s) occurred during the generation of facets! Result may be incomplete", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				}
				else
				{
					//we but back the scalar field
					if (indexSF)
						sfIdx = pc->addScalarField(indexSF);
				}

				//pc->setEnabled(false);
				m_app->addToDB(group);
				group->prepareDisplayForRefresh();
			}
			else if (error)
			{
				m_app->dispToConsole("An error occurred during the generation of facets!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
			else
			{
				m_app->dispToConsole("No facet remains! Check the parameters (min size, etc.)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}
		}
	}
	else
	{
		m_app->dispToConsole("An error occurred during the fusion process!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	if (sfIdx >= 0)
	{
		pc->getScalarField(sfIdx)->computeMinAndMax();
#ifdef _DEBUG
		pc->setCurrentDisplayedScalarField(sfIdx);
		pc->showSF(true);
#endif
	}

	//currently selected entities appearance may have changed!
	m_app->redrawAll();
}

ccHObject* qFacets::createFacets(ccPointCloud* cloud,
	CCLib::ReferenceCloudContainer& components,
	unsigned minPointsPerComponent,
	double maxEdgeLength,
	bool randomColors,
	bool& error)
{
	if (!cloud)
	{
		return nullptr;
	}

	//we create a new group to store all input CCs as 'facets'
	ccHObject* ccGroup = new ccHObject(cloud->getName() + QString(" [facets]"));
	ccGroup->setDisplay(cloud->getDisplay());
	ccGroup->setVisible(true);

	bool cloudHasNormal = cloud->hasNormals();

	//number of input components
	size_t componentCount = components.size();

	//progress notification
	ccProgressDialog pDlg(true, m_app->getMainWindow());
	pDlg.setMethodTitle(QObject::tr("Facets creation"));
	pDlg.setInfo(QObject::tr("Components: %1").arg(componentCount));
	pDlg.setMaximum(static_cast<int>(componentCount));
	pDlg.show();
	QApplication::processEvents();

	//for each component
	error = false;
	while (!components.empty())
	{
		CCLib::ReferenceCloud* compIndexes = components.back();
		components.pop_back();

		//if it has enough points
		if (compIndexes && compIndexes->size() >= minPointsPerComponent)
		{
			ccPointCloud* facetCloud = cloud->partialClone(compIndexes);
			if (!facetCloud)
			{
				//not enough  memory!
				error = true;
				delete facetCloud;
				facetCloud = nullptr;
			}
			else
			{
				ccFacet* facet = ccFacet::Create(facetCloud, static_cast<PointCoordinateType>(maxEdgeLength), true);
				if (facet)
				{
					QString facetName = QString("facet %1 (rms=%2)").arg(ccGroup->getChildrenNumber()).arg(facet->getRMS());
					facet->setName(facetName);
					if (facet->getPolygon())
					{
						facet->getPolygon()->enableStippling(false);
						facet->getPolygon()->showNormals(false);
					}
					if (facet->getContour())
					{
						facet->getContour()->setGlobalScale(facetCloud->getGlobalScale());
						facet->getContour()->setGlobalShift(facetCloud->getGlobalShift());
					}

					//check the facet normal sign
					if (cloudHasNormal)
					{
						CCVector3 N = ccOctree::ComputeAverageNorm(compIndexes, cloud);

						if (N.dot(facet->getNormal()) < 0)
							facet->invertNormal();
					}

#ifdef _DEBUG
					facet->showNormalVector(true);
#endif

					//shall we colorize it with a random color?
					ccColor::Rgb col;
					ccColor::Rgb darkCol;
					if (randomColors)
					{
						col = ccColor::Generator::Random();
						assert(c_darkColorRatio <= 1.0);
						darkCol.r = static_cast<ColorCompType>(static_cast<double>(col.r) * c_darkColorRatio);
						darkCol.g = static_cast<ColorCompType>(static_cast<double>(col.g) * c_darkColorRatio);
						darkCol.b = static_cast<ColorCompType>(static_cast<double>(col.b) * c_darkColorRatio);
					}
					else
					{
						//use normal-based HSV coloring
						CCVector3 N = facet->getNormal();
						PointCoordinateType dip = 0;
						PointCoordinateType dipDir = 0;
						ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
						FacetsClassifier::GenerateSubfamilyColor(col, dip, dipDir, 0, 1, &darkCol);
					}
					facet->setColor(col);
					if (facet->getContour())
					{
						facet->getContour()->setColor(darkCol);
						facet->getContour()->setWidth(2);
					}
					ccGroup->addChild(facet);
				}
			}

			delete compIndexes;
			compIndexes = nullptr;
		}

		pDlg.setValue(static_cast<int>(componentCount - components.size()));
		//QApplication::processEvents();
	}

	if (ccGroup->getChildrenNumber() == 0)
	{
		delete ccGroup;
		ccGroup = nullptr;
	}

	return ccGroup;
}

void qFacets::getFacetsInCurrentSelection(FacetSet& facets) const
{
	facets.clear();

	//look for potential facets
	for ( ccHObject *entity : m_app->getSelectedEntities() )
	{
		if (entity->isA(CC_TYPES::FACET))
		{
			ccFacet* facet = static_cast<ccFacet*>(entity);
			if (facet->getContour()) //if no contour, we won't be able to save it?!
				facets.insert(facet);
		}
		else //if (entity->isA(CC_TYPES::HIERARCHY_OBJECT)) //recursively tests group's children
		{
			ccHObject::Container childFacets;
			entity->filterChildren(childFacets, true, CC_TYPES::FACET);
			
			for ( ccHObject *childFacet : childFacets )
			{
				ccFacet* facet = static_cast<ccFacet*>(childFacet);
				if (facet->getContour()) //if no contour, we won't be able to save it?!
				{
					facets.insert(facet);
				}
			}
		}
	}
}

//standard meta-data for the qFacets plugin
struct FacetMetaData
{
	int facetIndex;
	CCVector3 center;
	CCVector3 normal;
	double surface;
	int dip_deg;
	int dipDir_deg;
	double rms;
	int familyIndex;
	int subfamilyIndex;

	//! Default constructor
	FacetMetaData()
		: facetIndex(-1)
		, center(0, 0, 0)
		, normal(0, 0, 1)
		, surface(0.0)
		, dip_deg(0)
		, dipDir_deg(0)
		, rms(0.0)
		, familyIndex(0)
		, subfamilyIndex(0)
	{}
};

//helper: extract all meta-data information form a facet
void GetFacetMetaData(ccFacet* facet, FacetMetaData& data)
{
	//try to get the facet index from the facet name!
	{
		QStringList tokens = facet->getName().split(" ", QString::SkipEmptyParts);
		if (tokens.size() > 1 && tokens[0] == QString("facet"))
		{
			bool ok = true;
			data.facetIndex = tokens[1].toInt(&ok);
			if (!ok)
				data.facetIndex = -1;
		}
	}

	data.center = facet->getCenter();
	data.normal = facet->getNormal();
	data.surface = facet->getSurface();
	data.rms = facet->getRMS();

	//family and subfamily indexes
	QVariant fi = facet->getMetaData(s_OriFamilyKey);
	if (fi.isValid())
		data.familyIndex = fi.toInt();
	QVariant sfi = facet->getMetaData(s_OriSubFamilyKey);
	if (sfi.isValid())
		data.subfamilyIndex = sfi.toInt();

	//compute dip direction & dip
	{
		PointCoordinateType dipDir = 0;
		PointCoordinateType dip = 0;
		ccNormalVectors::ConvertNormalToDipAndDipDir(data.normal, dip, dipDir);
		data.dipDir_deg = static_cast<int>(dipDir);
		data.dip_deg = static_cast<int>(dip);
	}
}

//helper: computes a facet horizontal and vertical extensions
void ComputeFacetExtensions(CCVector3& N, ccPolyline* facetContour, double& horizExt, double& vertExt)
{
	//horizontal and vertical extensions
	horizExt = vertExt = 0;

	CCLib::GenericIndexedCloudPersist* vertCloud = facetContour->getAssociatedCloud();
	if (vertCloud)
	{
		//oriRotMat.applyRotation(N); //DGM: oriRotMat is only for display!
		//we assume that at this point the "up" direction is always (0,0,1)
		CCVector3 Xf(1, 0, 0);
		CCVector3 Yf(0, 1, 0);
		//we get the horizontal vector on the plane
		CCVector3 D = CCVector3(0, 0, 1).cross(N);
		if (D.norm2() > ZERO_TOLERANCE) //otherwise the facet is horizontal!
		{
			Yf = D;
			Yf.normalize();
			Xf = N.cross(Yf);
		}

		const CCVector3* G = CCLib::Neighbourhood(vertCloud).getGravityCenter();

		ccBBox box;
		for (unsigned i = 0; i < vertCloud->size(); ++i)
		{
			const CCVector3 P = *(vertCloud->getPoint(i)) - *G;
			CCVector3 p(P.dot(Xf), P.dot(Yf), 0);
			box.add(p);
		}

		vertExt = box.getDiagVec().x;
		horizExt = box.getDiagVec().y;
	}
}

void qFacets::exportFacets()
{
	assert(m_app);
	if (!m_app)
		return;

	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	//Retrive selected facets
	FacetSet facets;
	getFacetsInCurrentSelection(facets);

	if (facets.empty())
	{
		m_app->dispToConsole("Couldn't find any facet in the current selection!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	assert(!facets.empty());

	FacetsExportDlg fDlg(FacetsExportDlg::SHAPE_FILE_IO, m_app->getMainWindow());

	//persistent settings (default export path)
	QSettings settings;
	settings.beginGroup("qFacets");
	QString facetsSavePath = settings.value("exportPath", ccFileUtils::defaultDocPath()).toString();
	fDlg.destinationPathLineEdit->setText(facetsSavePath + QString("/facets.shp"));

	if (!fDlg.exec())
		return;

	QString filename = fDlg.destinationPathLineEdit->text();

	//save current export path to persistent settings
	settings.setValue("exportPath", QFileInfo(filename).absolutePath());

	if (QFile(filename).exists())
	{
		//if the file already exists, ask for confirmation!
		if (QMessageBox::warning(m_app->getMainWindow(), "File already exists!", "File already exists! Are you sure you want to overwrite it?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return;
	}

	//fields (shapefile) - WARNING names must not have more than 10 chars!
	IntegerDBFField  facetIndex("index");
	DoubleDBFField   facetSurface("surface");
	DoubleDBFField   facetRMS("rms");
	IntegerDBFField  facetDipDir("dip_dir");
	IntegerDBFField  facetDip("dip");
	IntegerDBFField  familyIndex("family_ind");
	IntegerDBFField  subfamilyIndex("subfam_ind");
	DoubleDBFField3D facetNormal("normal");
	DoubleDBFField3D facetBarycenter("center");
	DoubleDBFField   horizExtension("horiz_ext");
	DoubleDBFField   vertExtension("vert_ext");
	DoubleDBFField   surfaceExtension("surf_ext");

	size_t facetCount = facets.size();
	assert(facetCount != 0);
	try
	{
		facetIndex.values.reserve(facetCount);
		facetSurface.values.reserve(facetCount);
		facetRMS.values.reserve(facetCount);
		facetDipDir.values.reserve(facetCount);
		facetDip.values.reserve(facetCount);
		familyIndex.values.reserve(facetCount);
		subfamilyIndex.values.reserve(facetCount);
		facetNormal.values.reserve(facetCount);
		facetBarycenter.values.reserve(facetCount);
		horizExtension.values.reserve(facetCount);
		vertExtension.values.reserve(facetCount);
		surfaceExtension.values.reserve(facetCount);
	}
	catch (const std::bad_alloc&)
	{
		m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject toSave("facets");

	//depending on the 'main orientation', the job is more or less easy ;)
	bool useNativeOrientation = fDlg.nativeOriRadioButton->isChecked();
	bool useGlobalOrientation = fDlg.verticalOriRadioButton->isChecked();
	bool useCustomOrientation = fDlg.customOriRadioButton->isChecked();

	//Default base
	CCVector3 X(1, 0, 0);
	CCVector3 Y(0, 1, 0);
	CCVector3 Z(0, 0, 1);

	//'vertical' orientation (potentially specified by the user)
	if (!useNativeOrientation)
	{
		if (useCustomOrientation)
		{
			Z = CCVector3(	static_cast<PointCoordinateType>(fDlg.nXLineEdit->text().toDouble()),
							static_cast<PointCoordinateType>(fDlg.nYLineEdit->text().toDouble()),
							static_cast<PointCoordinateType>(fDlg.nZLineEdit->text().toDouble()));
			Z.normalize();
		}
		else if (useGlobalOrientation)
		{
			//we compute the mean orientation (weighted by each facet's surface)
			CCVector3d Nsum(0, 0, 0);
			for (FacetSet::iterator it = facets.begin(); it != facets.end(); ++it)
			{
				double surf = (*it)->getSurface();
				CCVector3 N = (*it)->getNormal();
				Nsum.x += static_cast<double>(N.x) * surf;
				Nsum.y += static_cast<double>(N.y) * surf;
				Nsum.z += static_cast<double>(N.z) * surf;
			}
			Nsum.normalize();

			Z = CCVector3(static_cast<PointCoordinateType>(Nsum.x),
				static_cast<PointCoordinateType>(Nsum.y),
				static_cast<PointCoordinateType>(Nsum.z));
		}

		//update X & Y
		CCVector3 D = Z.cross(CCVector3(0, 0, 1));
		if (D.norm2() > ZERO_TOLERANCE) //otherwise the vertical dir hasn't changed!
		{
			X = -D;
			X.normalize();
			Y = Z.cross(X);
		}
	}

	//we compute the mean center (weighted by each facet's surface)
	CCVector3 C(0, 0, 0);
	{
		double weightSum = 0;
		for (FacetSet::iterator it = facets.begin(); it != facets.end(); ++it)
		{
			double surf = (*it)->getSurface();
			CCVector3 Ci = (*it)->getCenter();
			C += Ci * static_cast<PointCoordinateType>(surf);
			weightSum += surf;
		}
		if (weightSum)
			C /= static_cast<PointCoordinateType>(weightSum);
	}

	//determine the 'global' orientation matrix
	ccGLMatrix oriRotMat;
	oriRotMat.toIdentity();
	if (!useNativeOrientation)
	{
		oriRotMat.getColumn(0)[0] = static_cast<float>(X.x);
		oriRotMat.getColumn(0)[1] = static_cast<float>(X.y);
		oriRotMat.getColumn(0)[2] = static_cast<float>(X.z);
		oriRotMat.getColumn(1)[0] = static_cast<float>(Y.x);
		oriRotMat.getColumn(1)[1] = static_cast<float>(Y.y);
		oriRotMat.getColumn(1)[2] = static_cast<float>(Y.z);
		oriRotMat.getColumn(2)[0] = static_cast<float>(Z.x);
		oriRotMat.getColumn(2)[1] = static_cast<float>(Z.y);
		oriRotMat.getColumn(2)[2] = static_cast<float>(Z.z);
		oriRotMat.invert();

		ccGLMatrix transMat;
		transMat.setTranslation(-C);
		oriRotMat = oriRotMat * transMat;
		oriRotMat.setTranslation(oriRotMat.getTranslationAsVec3D() + C);
	}

	//for each facet
	for (FacetSet::iterator it = facets.begin(); it != facets.end(); ++it)
	{
		ccFacet* facet = *it;
		ccPolyline* poly = facet->getContour();

		//if necessary, we create a (temporary) new facet
		if (!useNativeOrientation)
		{
			CCLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
			if (!vertices || vertices->size() < 3)
				continue;

			//create (temporary) new polyline
			ccPolyline* newPoly = new ccPolyline(*poly);
			ccPointCloud* pc = (newPoly ? dynamic_cast<ccPointCloud*>(newPoly->getAssociatedCloud()) : nullptr);
			if (pc)
			{
				pc->applyGLTransformation_recursive(&oriRotMat);
			}
			else
			{
				m_app->dispToConsole(QString("Failed to change the orientation of polyline '%1'! (not enough memory)").arg(poly->getName()), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				continue;
			}

			newPoly->set2DMode(true);
			poly = newPoly;
		}

		toSave.addChild(poly, useNativeOrientation ? ccHObject::DP_NONE : ccHObject::DP_PARENT_OF_OTHER);

		//save associated meta-data as 'shapefile' fields
		{
			//main parameters
			FacetMetaData data;
			GetFacetMetaData(facet, data);

			//horizontal and vertical extensions
			double horizExt = 0;
			double vertExt = 0;
			ComputeFacetExtensions(data.normal, poly, horizExt, vertExt);

			facetIndex.values.push_back(data.facetIndex);
			facetSurface.values.push_back(data.surface);
			facetRMS.values.push_back(data.rms);
			facetDipDir.values.push_back(data.dipDir_deg);
			facetDip.values.push_back(data.dip_deg);
			familyIndex.values.push_back(data.familyIndex);
			subfamilyIndex.values.push_back(data.subfamilyIndex);
			facetNormal.values.push_back(CCVector3d(data.normal.x, data.normal.y, data.normal.z));
			facetBarycenter.values.push_back(CCVector3d(data.center.x, data.center.y, data.center.z));
			vertExtension.values.push_back(vertExt);
			horizExtension.values.push_back(horizExt);
			surfaceExtension.values.push_back(horizExt*vertExt);
		}
	}

	//save entities
	if (toSave.getChildrenNumber())
	{
		std::vector<GenericDBFField*> fields;
		fields.push_back(&facetIndex);
		fields.push_back(&facetBarycenter);
		fields.push_back(&facetNormal);
		fields.push_back(&facetRMS);
		fields.push_back(&horizExtension);
		fields.push_back(&vertExtension);
		fields.push_back(&surfaceExtension);
		fields.push_back(&facetSurface);
		fields.push_back(&facetDipDir);
		fields.push_back(&facetDip);
		fields.push_back(&familyIndex);
		fields.push_back(&subfamilyIndex);
		ShpFilter filter;
		filter.treatClosedPolylinesAsPolygons(true);
		ShpFilter::SaveParameters params;
		params.alwaysDisplaySaveDialog = false;
		if (filter.saveToFile(&toSave, fields, filename, params) == CC_FERR_NO_ERROR)
		{
			m_app->dispToConsole(QString("[qFacets] File '%1' successfully saved").arg(filename), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
		else
		{
			m_app->dispToConsole(QString("[qFacets] Failed to save file '%1'!").arg(filename), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}
}

void qFacets::showStereogram()
{
	assert(m_app);
	if (!m_app)
		return;

	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	//we expect a facet group or a cloud
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	if (!m_app->haveOneSelection()
		|| (!selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT)
		&& !selectedEntities.back()->isA(CC_TYPES::POINT_CLOUD))
		)
	{
		m_app->dispToConsole("Select a group of facets or a point cloud!");
		return;
	}

	StereogramParamsDlg stereogramParamsDlg(m_app->getMainWindow());
	stereogramParamsDlg.angleStepDoubleSpinBox->setValue(s_stereogramAngleStep);
	stereogramParamsDlg.resolutionDoubleSpinBox->setValue(s_stereogramResolution_deg);
	if (!stereogramParamsDlg.exec())
		return;

	s_stereogramAngleStep = stereogramParamsDlg.angleStepDoubleSpinBox->value();
	s_stereogramResolution_deg = stereogramParamsDlg.resolutionDoubleSpinBox->value();

	if ( m_stereogramDialog == nullptr )
	{
		m_stereogramDialog = new StereogramDialog( m_app );
	}
	
	if (m_stereogramDialog->init(s_stereogramAngleStep, selectedEntities.back(), s_stereogramResolution_deg))
	{
		m_stereogramDialog->show();
		m_stereogramDialog->raise();
	}
}

void qFacets::classifyFacetsByAngle()
{
	assert(m_app);
	if (!m_app)
		return;

	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	//we expect a facet group
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	if (!m_app->haveOneSelection() || !selectedEntities.back()->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		m_app->dispToConsole("Select a group of facets!");
		return;
	}

	ClassificationParamsDlg classifParamsDlg(m_app->getMainWindow());
	classifParamsDlg.angleStepDoubleSpinBox->setValue(s_classifAngleStep);
	classifParamsDlg.maxDistDoubleSpinBox->setValue(s_classifMaxDist);
	if (!classifParamsDlg.exec())
		return;

	s_classifAngleStep = classifParamsDlg.angleStepDoubleSpinBox->value();
	s_stereogramAngleStep = s_classifAngleStep; //we automatically copy it to the stereogram's equivalent parameter
	s_classifMaxDist = classifParamsDlg.maxDistDoubleSpinBox->value();

	ccHObject* group = selectedEntities.back();
	classifyFacetsByAngle(group, s_classifAngleStep, s_classifMaxDist);
}

void qFacets::classifyFacetsByAngle(ccHObject* group,
	double angleStep_deg,
	double maxDist)
{
	assert(m_app);
	if (!m_app)
		return;

	assert(group);

	if (group->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		if (group->getParent())
		{
			m_app->removeFromDB(group, false);
		}

		bool success = FacetsClassifier::ByOrientation(group, angleStep_deg, maxDist);
		m_app->addToDB(group);

		if (!success)
		{
			m_app->dispToConsole("An error occurred while classifying the facets! (not enough memory?)",
				ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}

	m_app->redrawAll();
}

void qFacets::exportFacetsInfo()
{
	assert(m_app);
	if (!m_app)
		return;

	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	//Retrive selected facets
	FacetSet facets;
	getFacetsInCurrentSelection(facets);

	if (facets.empty())
	{
		m_app->dispToConsole("Couldn't find any facet in the current selection!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	assert(!facets.empty());

	FacetsExportDlg fDlg(FacetsExportDlg::ASCII_FILE_IO, m_app->getMainWindow());
	fDlg.orientationGroupBox->setEnabled(false);

	//persistent settings (default export path)
	QSettings settings;
	settings.beginGroup("qFacets");
	QString facetsSavePath = settings.value("exportPath", ccFileUtils::defaultDocPath()).toString();
	fDlg.destinationPathLineEdit->setText(facetsSavePath + QString("/facets.csv"));

	if (!fDlg.exec())
		return;

	QString filename = fDlg.destinationPathLineEdit->text();

	//save current export path to persistent settings
	settings.setValue("exportPath", QFileInfo(filename).absolutePath());

	QFile outFile(filename);
	if (outFile.exists())
	{
		//if the file already exists, ask for confirmation!
		if (QMessageBox::warning(m_app->getMainWindow(),
			"Overwrite",
			"File already exists! Are you sure you want to overwrite it?",
			QMessageBox::Yes,
			QMessageBox::No) == QMessageBox::No)
			return;
	}

	//open CSV file
	if (!outFile.open(QFile::WriteOnly | QFile::Text))
	{
		m_app->dispToConsole(QString("Failed to open file for writing! Check available space and access rights"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//write header
	QTextStream outStream(&outFile);
	outStream << "Index;";
	outStream << " CenterX;";
	outStream << " CenterY;";
	outStream << " CenterZ;";
	outStream << " NormalX;";
	outStream << " NormalY;";
	outStream << " NormalZ;";
	outStream << " RMS;";
	outStream << " Horiz_ext;";
	outStream << " Vert_ext;";
	outStream << " Surf_ext;";
	outStream << " Surface;";
	outStream << " Dip dir.;";
	outStream << " Dip;";
	outStream << " Family ind.;";
	outStream << " Subfamily ind.;";
	outStream << " \n";

	//write data (one line per facet)
	for (FacetSet::iterator it = facets.begin(); it != facets.end(); ++it)
	{
		ccFacet* facet = *it;
		FacetMetaData data;
		GetFacetMetaData(facet, data);
		//horizontal and vertical extensions
		double horizExt = 0;
		double vertExt = 0;
		ComputeFacetExtensions(data.normal, facet->getContour(), horizExt, vertExt);

		outStream << data.facetIndex << ";";
		outStream << data.center.x << ";" << data.center.y << ";" << data.center.z << ";";
		outStream << data.normal.x << ";" << data.normal.y << ";" << data.normal.z << ";";
		outStream << data.rms << ";";
		outStream << horizExt << ";";
		outStream << vertExt << ";";
		outStream << horizExt*vertExt << ";";
		outStream << data.surface << ";";
		outStream << data.dipDir_deg << ";";
		outStream << data.dip_deg << ";";
		outStream << data.familyIndex << ";";
		outStream << data.subfamilyIndex << ";";
		outStream << "\n";
	}

	outFile.close();

	m_app->dispToConsole(QString("[qFacets] File '%1' successfully saved").arg(filename), ccMainAppInterface::STD_CONSOLE_MESSAGE);
}
