//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qM3C2.h"

//local
#include "qM3C2Tools.h"
#include "qM3C2Dialog.h"
#include "qM3C2DisclaimerDialog.h"

//CCLib
#include <CloudSamplingTools.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccHObjectCaster.h>
#include <ccProgressDialog.h>
#include <ccNormalVectors.h>
#include <ccScalarField.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QElapsedTimer>
#include <QtConcurrentMap>

//! Default name for M3C2 scalar fields
static const char M3C2_DIST_SF_NAME[]			= "M3C2 distance";
static const char DIST_UNCERTAINTY_SF_NAME[]	= "distance uncertainty";
static const char SIG_CHANGE_SF_NAME[]			= "significant change";
static const char STD_DEV_CLOUD1_SF_NAME[]		= "%1_cloud1";
static const char STD_DEV_CLOUD2_SF_NAME[]		= "%1_cloud2";
static const char DENSITY_CLOUD1_SF_NAME[]		= "Npoints_cloud1";
static const char DENSITY_CLOUD2_SF_NAME[]		= "Npoints_cloud2";
static const char NORMAL_SCALE_SF_NAME[]		= "normal scale";

qM3C2Plugin::qM3C2Plugin(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

static void RemoveScalarField(ccPointCloud* cloud, const char sfName[])
{
	int sfIdx = cloud ? cloud->getScalarFieldIndexByName(sfName) : -1;
	if (sfIdx >= 0)
	{
		cloud->deleteScalarField(sfIdx);
	}
}

void qM3C2Plugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		m_action->setEnabled(	selectedEntities.size() == 2
							&&	selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
							&&	selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD) );
	}

	m_selectedEntities = selectedEntities;
}

void qM3C2Plugin::getActions(QActionGroup& group)
{
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

static ScalarType SCALAR_ZERO = 0;
static ScalarType SCALAR_ONE = 1;

// Structure for parallel call to ComputeM3C2DistForPoint
static struct
{
	ccPointCloud* outputCloud;
	ccPointCloud* corePoints;
	NormsIndexesTableType* coreNormals;

	PointCoordinateType projectionRadius;
	PointCoordinateType projectionDepth;
	bool updateNormal;
	bool exportNormal;
	bool useMedian;
	bool computeConfidence;
	bool progressiveSearch;
	bool onlyPositiveSearch;
	unsigned minPoints4Stats;
	double registrationRms;

	//export
	qM3C2Dialog::ExportOptions exportOption;
	bool keepOriginalCloud;

	ccOctree::Shared cloud1Octree;
	unsigned char level1;
	ccOctree::Shared cloud2Octree;
	unsigned char level2;

	ccScalarField* m3c2DistSF;
	ccScalarField* distUncertaintySF;
	ccScalarField* sigChangeSF;
	ccScalarField* stdDevCloud1SF;
	ccScalarField* stdDevCloud2SF;
	ccScalarField* densityCloud1SF;
	ccScalarField* densityCloud2SF;

	CCLib::NormalizedProgress* nProgress;
	bool processCanceled;

} s_M3C2Params;

void ComputeM3C2DistForPoint(unsigned index)
{
	if (s_M3C2Params.processCanceled)
		return;

	ScalarType dist = NAN_VALUE;

	//get core point #i
	CCVector3 P;
	s_M3C2Params.corePoints->getPoint(index, P);

	//get core point's normal #i
	CCVector3 N(0, 0, 1);
	if (s_M3C2Params.updateNormal) //i.e. all cases but the VERTICAL mode
	{
		N = ccNormalVectors::GetNormal(s_M3C2Params.coreNormals->getValue(index));
	}

	//output point
	CCVector3 outputP = P;

	//compute M3C2 distance
	{
		double mean1 = 0, stdDev1 = 0;
		bool validStats1 = false;

		//extract cloud #1's neighbourhood
		CCLib::DgmOctree::ProgressiveCylindricalNeighbourhood cn1;
		cn1.center = P;
		cn1.dir = N;
		cn1.level = s_M3C2Params.level1;
		cn1.maxHalfLength = s_M3C2Params.projectionDepth;
		cn1.radius = s_M3C2Params.projectionRadius;
		cn1.onlyPositiveDir = s_M3C2Params.onlyPositiveSearch;

		if (s_M3C2Params.progressiveSearch)
		{
			//progressive search
			size_t previousNeighbourCount = 0;
			while (cn1.currentHalfLength < cn1.maxHalfLength)
			{
				size_t neighbourCount = s_M3C2Params.cloud1Octree->getPointsInCylindricalNeighbourhoodProgressive(cn1);
				if (neighbourCount != previousNeighbourCount)
				{
					//do we have enough points for computing stats?
					if (neighbourCount >= s_M3C2Params.minPoints4Stats)
					{
						qM3C2Tools::ComputeStatistics(cn1.neighbours,s_M3C2Params.useMedian,mean1,stdDev1);
						validStats1 = true;
						//do we have a sharp enough 'mean' to stop?
						if (fabs(mean1) + 2*stdDev1 < static_cast<double>(cn1.currentHalfLength))
							break;
					}
					previousNeighbourCount = neighbourCount;
				}
			}
		}
		else
		{
			s_M3C2Params.cloud1Octree->getPointsInCylindricalNeighbourhood(cn1);
		}
		
		size_t n1 = cn1.neighbours.size();
		if (n1 != 0)
		{
			//compute stat. dispersion on cloud #1 neighbours (if necessary)
			if (!validStats1)
			{
				qM3C2Tools::ComputeStatistics(cn1.neighbours, s_M3C2Params.useMedian, mean1, stdDev1);
			}

			if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD1)
			{
				//shift output point on the 1st cloud
				outputP += static_cast<PointCoordinateType>(mean1) * N;
			}

			//save cloud #1's std. dev.
			if (s_M3C2Params.stdDevCloud1SF)
			{
				ScalarType val = static_cast<ScalarType>(stdDev1);
				s_M3C2Params.stdDevCloud1SF->setValue(index, val);
			}
		}

		//save cloud #1's density
		if (s_M3C2Params.densityCloud1SF)
		{
			ScalarType val = static_cast<ScalarType>(n1);
			s_M3C2Params.densityCloud1SF->setValue(index, val);
		}

		//now we can process cloud #2
		if (	n1 != 0
			||	s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2
			||	s_M3C2Params.stdDevCloud2SF
			||	s_M3C2Params.densityCloud2SF
			)
		{
			double mean2 = 0, stdDev2 = 0;
			bool validStats2 = false;
			
			//extract cloud #2's neighbourhood
			CCLib::DgmOctree::ProgressiveCylindricalNeighbourhood cn2;
			cn2.center = P;
			cn2.dir = N;
			cn2.level = s_M3C2Params.level2;
			cn2.maxHalfLength = s_M3C2Params.projectionDepth;
			cn2.radius = s_M3C2Params.projectionRadius;
			cn2.onlyPositiveDir = s_M3C2Params.onlyPositiveSearch;

			if (s_M3C2Params.progressiveSearch)
			{
				//progressive search
				size_t previousNeighbourCount = 0;
				while (cn2.currentHalfLength < cn2.maxHalfLength)
				{
					size_t neighbourCount = s_M3C2Params.cloud2Octree->getPointsInCylindricalNeighbourhoodProgressive(cn2);
					if (neighbourCount != previousNeighbourCount)
					{
						//do we have enough points for computing stats?
						if (neighbourCount >= s_M3C2Params.minPoints4Stats)
						{
							qM3C2Tools::ComputeStatistics(cn2.neighbours,s_M3C2Params.useMedian,mean2,stdDev2);
							validStats2 = true;
							//do we have a sharp enough 'mean' to stop?
							if (fabs(mean2) + 2*stdDev2 < static_cast<double>(cn2.currentHalfLength))
								break;
						}
						previousNeighbourCount = neighbourCount;
					}
				}
			}
			else
			{
				s_M3C2Params.cloud2Octree->getPointsInCylindricalNeighbourhood(cn2);
			}

			size_t n2 = cn2.neighbours.size();
			if (n2 != 0)
			{
				//compute stat. dispersion on cloud #2 neighbours (if necessary)
				if (!validStats2)
				{
					qM3C2Tools::ComputeStatistics(cn2.neighbours, s_M3C2Params.useMedian, mean2, stdDev2);
				}
				assert(stdDev2 != stdDev2 || stdDev2 >= 0); //first inequality fails if stdDev2 is NaN ;)

				if (s_M3C2Params.exportOption == qM3C2Dialog::PROJECT_ON_CLOUD2)
				{
					//shift output point on the 2nd cloud
					outputP += static_cast<PointCoordinateType>(mean2) * N;
				}

				if (n1 != 0)
				{
					//m3c2 dist = distance between i1 and i2 (i.e. either the mean or the median of both neighborhoods)
					dist = static_cast<ScalarType>(mean2 - mean1);
					s_M3C2Params.m3c2DistSF->setValue(index,dist);

					//confidence interval
					if (s_M3C2Params.computeConfidence)
					{
						//have we enough points for computing the confidence interval?
						if (n1 >= s_M3C2Params.minPoints4Stats && n2 >= s_M3C2Params.minPoints4Stats)
						{
							//distance uncertainty (see eq. (1) in M3C2 article)
							ScalarType LOD = static_cast<ScalarType>( 1.96 * (	sqrt(	stdDev1*stdDev1/static_cast<double>(n1)  +
																						stdDev2*stdDev2/static_cast<double>(n2)
																					)
																				+ s_M3C2Params.registrationRms
																			)
																	);

							if (s_M3C2Params.distUncertaintySF)
							{
								s_M3C2Params.distUncertaintySF->setValue(index, LOD);
							}

							if (s_M3C2Params.sigChangeSF)
							{
								bool significant = (dist < -LOD || dist > LOD);
								if (significant)
									s_M3C2Params.sigChangeSF->setValue(index, SCALAR_ONE); //already equal to SCALAR_ZERO otherwise
							}
						}
						//else //DGM: scalar fields have already been initialized with the right 'default' values
						//{
						//	if (distUncertaintySF)
						//		distUncertaintySF->setValue(index,NAN_VALUE);
						//	if (sigChangeSF)
						//		sigChangeSF->setValue(index,SCALAR_ZERO);
						//}
					}
				}

				//save cloud #2's std. dev.
				if (s_M3C2Params.stdDevCloud2SF)
				{
					ScalarType val = static_cast<ScalarType>(stdDev2);
					s_M3C2Params.stdDevCloud2SF->setValue(index, val);
				}
			}

			//save cloud #2's density
			if (s_M3C2Params.densityCloud2SF)
			{
				ScalarType val = static_cast<ScalarType>(n2);
				s_M3C2Params.densityCloud2SF->setValue(index, val);
			}
		}
	}

	//output point
	if (s_M3C2Params.outputCloud != s_M3C2Params.corePoints)
	{
		*const_cast<CCVector3*>(s_M3C2Params.outputCloud->getPoint(index)) = outputP;
	}
	if (s_M3C2Params.exportNormal)
	{
		s_M3C2Params.outputCloud->setPointNormal(index, N);
	}

	//progress notification
	if (s_M3C2Params.nProgress && !s_M3C2Params.nProgress->oneStep())
	{
		s_M3C2Params.processCanceled = true;
	}
}

void qM3C2Plugin::doAction()
{
	//disclaimer accepted?
	if (!ShowDisclaimer(m_app))
		return;

	//m_app should have already been initialized by CC when plugin is loaded!
	assert(m_app);
	if (!m_app)
		return;

	if (	m_selectedEntities.size() != 2
		||	!m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
		||	!m_selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select two point clouds!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud1 = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
	ccPointCloud* cloud2 = ccHObjectCaster::ToPointCloud(m_selectedEntities[1]);

	//display dialog
	qM3C2Dialog dlg(cloud1, cloud2, m_app);

	if (!dlg.exec())
		return;
	dlg.saveParamsToPersistentSettings();

	//get the clouds in the right order
	cloud1 = dlg.getCloud1();
	cloud2 = dlg.getCloud2();

	//normals computation parameters
	double normalScale = dlg.normalScaleDoubleSpinBox->value();
	bool useCloud1Normals = dlg.useCloud1NormalsCheckBox->isChecked() && dlg.useCloud1NormalsCheckBox->isEnabled();
	double projectionScale = dlg.cylDiameterDoubleSpinBox->value();
	qM3C2Normals::ComputationMode normMode = dlg.getNormalsComputationMode();
	double samplingDist = dlg.cpSubsamplingDoubleSpinBox->value();
	ccScalarField* normalScaleSF = 0; //normal scale (multi-scale mode only)

	//other parameters are stored in 's_M3C2Params' for parallel call
	s_M3C2Params.projectionRadius = static_cast<PointCoordinateType>(projectionScale/2); //we want the radius in fact ;)
	s_M3C2Params.projectionDepth = static_cast<PointCoordinateType>(dlg.cylHalfHeightDoubleSpinBox->value());
	s_M3C2Params.corePoints = dlg.getCorePointsCloud();
	s_M3C2Params.registrationRms = dlg.rmsCheckBox->isChecked() ? dlg.rmsDoubleSpinBox->value() : 0.0;
	s_M3C2Params.exportOption = dlg.getExportOption();
	s_M3C2Params.keepOriginalCloud = dlg.keepOriginalCloud();
	s_M3C2Params.useMedian = dlg.useMedianCheckBox->isChecked();
	s_M3C2Params.minPoints4Stats = dlg.getMinPointsForStats();
	s_M3C2Params.progressiveSearch = !dlg.useSinglePass4DepthCheckBox->isChecked();
	s_M3C2Params.onlyPositiveSearch = dlg.positiveSearchOnlyCheckBox->isChecked();
	//scalar fields
	s_M3C2Params.m3c2DistSF = 0;			//M3C2 distance
	s_M3C2Params.distUncertaintySF = 0;		//distance uncertainty
	s_M3C2Params.sigChangeSF = 0;			//significant change
	s_M3C2Params.stdDevCloud1SF = 0;		//standard deviation information for cloud #1
	s_M3C2Params.stdDevCloud2SF = 0;		//standard deviation information for cloud #2
	s_M3C2Params.densityCloud1SF = 0;		//export point density at projection scale for cloud #1
	s_M3C2Params.densityCloud2SF = 0;		//export point density at projection scale for cloud #2
	//octrees
	s_M3C2Params.cloud1Octree.clear();
	s_M3C2Params.level1 = 0;
	s_M3C2Params.cloud2Octree.clear();
	s_M3C2Params.level2 = 0;
	//other
	s_M3C2Params.outputCloud = 0;
	s_M3C2Params.coreNormals = 0;
	s_M3C2Params.updateNormal = false;
	s_M3C2Params.exportNormal = false;
	s_M3C2Params.computeConfidence = false;
	s_M3C2Params.nProgress = 0;
	s_M3C2Params.processCanceled = false;

	//max thread count
	int maxThreadCount = dlg.getMaxThreadCount();

	//progress dialog
	ccProgressDialog pDlg(true, m_app->getMainWindow());

	//Duration: initialization & normals computation
	QElapsedTimer initTimer;
	initTimer.start();

	//compute octree(s) if necessary
	s_M3C2Params.cloud1Octree = cloud1->getOctree();
	if (!s_M3C2Params.cloud1Octree)
	{
		s_M3C2Params.cloud1Octree = cloud1->computeOctree(&pDlg);
		if (s_M3C2Params.cloud1Octree && cloud1->getParent())
		{
			m_app->addToDB(cloud1->getOctreeProxy());
		}
	}
	if (!s_M3C2Params.cloud1Octree)
	{
		m_app->dispToConsole("Failed to compute cloud #1's octree!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	
	s_M3C2Params.cloud2Octree = cloud2->getOctree();
	if (!s_M3C2Params.cloud2Octree)
	{
		s_M3C2Params.cloud2Octree = cloud2->computeOctree(&pDlg);
		if (s_M3C2Params.cloud2Octree && cloud2->getParent())
		{
			m_app->addToDB(cloud2->getOctreeProxy());
		}
	}
	if (!s_M3C2Params.cloud2Octree)
	{
		m_app->dispToConsole("Failed to compute cloud #2's octree!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//start the job
	bool error = false;

	//should we generate the core points?
	bool corePointsHaveBeenSubsampled = false;
	if (!s_M3C2Params.corePoints && samplingDist > 0)
	{
		CCLib::CloudSamplingTools::SFModulationParams modParams(false);
		CCLib::ReferenceCloud* subsampled = CCLib::CloudSamplingTools::resampleCloudSpatially(	cloud1, 
																								static_cast<PointCoordinateType>(samplingDist),
																								modParams,
																								s_M3C2Params.cloud1Octree.data(),
																								&pDlg);

		if (subsampled)
		{
			s_M3C2Params.corePoints = static_cast<ccPointCloud*>(cloud1)->partialClone(subsampled);
			
			//don't need those references anymore
			delete subsampled;
			subsampled = 0;
		}

		if (s_M3C2Params.corePoints)
		{
			m_app->dispToConsole(QString("[M3C2] Sub-sampled cloud has been saved ('%1')").arg(s_M3C2Params.corePoints->getName()),ccMainAppInterface::STD_CONSOLE_MESSAGE);
			s_M3C2Params.corePoints->setName(QString("%1.subsampled [min dist. = %2]").arg(cloud1->getName()).arg(samplingDist));
			s_M3C2Params.corePoints->setVisible(true);
			s_M3C2Params.corePoints->setDisplay(cloud1->getDisplay());
			m_app->addToDB(s_M3C2Params.corePoints);
			corePointsHaveBeenSubsampled = true;
		}
		else
		{
			m_app->dispToConsole("Failed to compute sub-sampled core points!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			error = true;
		}
	}

	//output
	QString outputName("M3C2 output");
	
	if (!error)
	{
		//whatever the case, at this point we should have core points
		assert(s_M3C2Params.corePoints);
		m_app->dispToConsole(QString("[M3C2] Core points: %1").arg(s_M3C2Params.corePoints->size()), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		if (s_M3C2Params.keepOriginalCloud)
		{
			s_M3C2Params.outputCloud = s_M3C2Params.corePoints;
		}
		else
		{
			s_M3C2Params.outputCloud = new ccPointCloud(/*outputName*/); //setName will be called at the end
			if (!s_M3C2Params.outputCloud->resize(s_M3C2Params.corePoints->size())) //resize as we will 'set' the new points positions in 'ComputeM3C2DistForPoint'
			{
				m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				error = true;
			}
			s_M3C2Params.corePoints->setEnabled(false); //we can hide the core points
		}
	}

	//compute normals
	if (!error)
	{
		bool normalsAreOk = false;
		bool useCorePointsOnly = dlg.normUseCorePointsCheckBox->isChecked();

		switch(normMode)
		{
		case qM3C2Normals::HORIZ_MODE:
				outputName += QString(" [HORIZONTAL]");
		case qM3C2Normals::DEFAULT_MODE:
		case qM3C2Normals::MULTI_SCALE_MODE:
			{
				s_M3C2Params.coreNormals = new NormsIndexesTableType();
				s_M3C2Params.coreNormals->link(); //will be released anyway at the end of the process
				
				std::vector<PointCoordinateType> radii;
				if (normMode == qM3C2Normals::MULTI_SCALE_MODE)
				{
					//get multi-scale parameters
					double startScale = dlg.minScaleDoubleSpinBox->value();
					double step = dlg.stepScaleDoubleSpinBox->value();
					double stopScale = dlg.maxScaleDoubleSpinBox->value();
					stopScale = std::max(startScale,stopScale); //just to be sure
					//generate all corresponding 'scales'
					for (double scale = startScale ; scale <= stopScale; scale += step)
					{
						radii.push_back(static_cast<PointCoordinateType>(scale / 2));
					}

					outputName += QString(" scale=[%1:%2:%3]").arg(startScale).arg(step).arg(stopScale);

					normalScaleSF = new ccScalarField(NORMAL_SCALE_SF_NAME);
					normalScaleSF->link(); //will be released anyway at the end of the process
				}
				else
				{
					outputName += QString(" scale=%1").arg(normalScale);
					//otherwise, we use a unique scale by default
					radii.push_back(static_cast<PointCoordinateType>(normalScale/2)); //we want the radius in fact ;)
				}

				bool invalidNormals = false;
				ccPointCloud* baseCloud = (useCorePointsOnly ? s_M3C2Params.corePoints : cloud1);
				ccOctree* baseOctree = (baseCloud == cloud1 ? s_M3C2Params.cloud1Octree.data() : 0);
				
				//dedicated core points method
				normalsAreOk = qM3C2Normals::ComputeCorePointsNormals(	s_M3C2Params.corePoints,
																		s_M3C2Params.coreNormals,
																		baseCloud,
																		radii,
																		invalidNormals,
																		maxThreadCount,
																		normalScaleSF,
																		&pDlg,
																		baseOctree);

				//now fix the orientation
				if (normalsAreOk)
				{
					//some invalid normals?
					if (invalidNormals)
					{
						m_app->dispToConsole("[M3C2] Some normals are invalid! You may have to increase the scale.",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
					}

					//make normals horizontal if necessary
					if (normMode == qM3C2Normals::HORIZ_MODE)
					{
						qM3C2Normals::MakeNormalsHorizontal(*s_M3C2Params.coreNormals);
					}

					//then either use a simple heuristic
					bool usePreferredOrientation = dlg.normOriPreferredRadioButton->isChecked();
					if (usePreferredOrientation)
					{
						int preferredOrientation = dlg.normOriPreferredComboBox->currentIndex();
						assert(preferredOrientation >= ccNormalVectors::MINUS_X && preferredOrientation <= ccNormalVectors::PLUS_ZERO);
						if (!ccNormalVectors::UpdateNormalOrientations(	s_M3C2Params.corePoints,
																		*s_M3C2Params.coreNormals,
																		static_cast<ccNormalVectors::Orientation>(preferredOrientation)))
						{
							m_app->dispToConsole("[M3C2] Failed to re-orient the normals (invalid parameter?)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
							error = true;
						}
					}
					else //or use external points
					{
						ccPointCloud* orientationCloud = dlg.getNormalsOrientationCloud();
						assert(orientationCloud);

						if (!qM3C2Normals::UpdateNormalOrientationsWithCloud(	s_M3C2Params.corePoints,
																				*s_M3C2Params.coreNormals,
																				orientationCloud,
																				maxThreadCount,
																				&pDlg ))
						{
							m_app->dispToConsole("[M3C2] Failed to re-orient the normals with input point cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
							error = true;
						}
					}

					if (!error && s_M3C2Params.coreNormals)
					{
						s_M3C2Params.outputCloud->setNormsTable(s_M3C2Params.coreNormals);
						s_M3C2Params.outputCloud->showNormals(true);
					}
				}
			}
			break;
		
		case qM3C2Normals::USE_CLOUD1_NORMALS:
			{
				outputName += QString(" scale=%1").arg(normalScale);
				ccPointCloud* sourceCloud = (corePointsHaveBeenSubsampled ? s_M3C2Params.corePoints : cloud1);
				s_M3C2Params.coreNormals = sourceCloud->normals();
				normalsAreOk = (s_M3C2Params.coreNormals && s_M3C2Params.coreNormals->currentSize() == sourceCloud->size());
				s_M3C2Params.coreNormals->link(); //will be released anyway at the end of the process

				//DGM TODO: should we export the normals to the output cloud?
			}
			break;
		
		case qM3C2Normals::VERT_MODE:
			{
				outputName += QString(" scale=%1").arg(normalScale);
				outputName += QString(" [VERTICAL]");

				//nothing to do
				normalsAreOk = true;
			}
			break;
		}

		if (!normalsAreOk)
		{
			m_app->dispToConsole("Failed to compute normals!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			error = true;
		}
	}

	if (!error && s_M3C2Params.coreNormals && corePointsHaveBeenSubsampled)
	{
		if (s_M3C2Params.corePoints->hasNormals() || s_M3C2Params.corePoints->resizeTheNormsTable())
		{
			for (unsigned i = 0; i < s_M3C2Params.coreNormals->currentSize(); ++i)
				s_M3C2Params.corePoints->setPointNormalIndex(i, s_M3C2Params.coreNormals->getValue(i));
			s_M3C2Params.corePoints->showNormals(true);
		}
		else
		{
			m_app->dispToConsole("Failed to allocate memory for core points normals!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
	}

	qint64 initTime_ms = initTimer.elapsed();

	while (!error) //fake loop for easy break
	{
		//we display init. timing only if no error occurred!
		m_app->dispToConsole(QString("[M3C2] Initialization & normal computation: %1 s.").arg(static_cast<double>(initTime_ms) / 1000.0, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		QElapsedTimer distCompTimer;
		distCompTimer.start();

		//we are either in vertical mode or we have as many normals as core points
		unsigned corePointCount = s_M3C2Params.corePoints->size();
		assert(normMode == qM3C2Normals::VERT_MODE || (s_M3C2Params.coreNormals && corePointCount == s_M3C2Params.coreNormals->currentSize()));

		pDlg.reset();
		CCLib::NormalizedProgress nProgress(&pDlg,corePointCount);
		pDlg.setMethodTitle(QObject::tr("M3C2 Distances Computation"));
		pDlg.setInfo(QObject::tr("Core points: %1").arg(corePointCount));
		pDlg.start();
		s_M3C2Params.nProgress = &nProgress;

		//allocate distances SF
		s_M3C2Params.m3c2DistSF = new ccScalarField(M3C2_DIST_SF_NAME);
		s_M3C2Params.m3c2DistSF->link();
		if (!s_M3C2Params.m3c2DistSF->resize(corePointCount, true, NAN_VALUE))
		{
			m_app->dispToConsole("Failed to allocate memory for distance values!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			error = true;
			break;
		}
		//allocate dist. uncertainty SF
		s_M3C2Params.distUncertaintySF = new ccScalarField(DIST_UNCERTAINTY_SF_NAME);
		s_M3C2Params.distUncertaintySF->link();
		if (!s_M3C2Params.distUncertaintySF->resize(corePointCount, true, NAN_VALUE))
		{
			m_app->dispToConsole("Failed to allocate memory for dist. uncertainty values!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			error = true;
			break;
		}
		//allocate change significance SF
		s_M3C2Params.sigChangeSF = new ccScalarField(SIG_CHANGE_SF_NAME);
		s_M3C2Params.sigChangeSF->link();
		if (!s_M3C2Params.sigChangeSF->resize(corePointCount, true, SCALAR_ZERO))
		{
			m_app->dispToConsole("Failed to allocate memory for change significance values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			s_M3C2Params.sigChangeSF->release();
			s_M3C2Params.sigChangeSF = 0;
			//no need to stop just for this SF!
			//error = true;
			//break;
		}

		if (dlg.exportStdDevInfoCheckBox->isChecked())
		{
			//allocate cloud #1 std. dev. SF
			s_M3C2Params.stdDevCloud1SF = new ccScalarField(qPrintable(QString(STD_DEV_CLOUD1_SF_NAME).arg(s_M3C2Params.useMedian ? "IQR" : "STD")));
			s_M3C2Params.stdDevCloud1SF->link();
			if (!s_M3C2Params.stdDevCloud1SF->resize(corePointCount, true, NAN_VALUE))
			{
				m_app->dispToConsole("Failed to allocate memory for cloud #1 std. dev. values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.stdDevCloud1SF->release();
				s_M3C2Params.stdDevCloud1SF = 0;
			}
			//allocate cloud #2 std. dev. SF
			s_M3C2Params.stdDevCloud2SF = new ccScalarField(qPrintable(QString(STD_DEV_CLOUD2_SF_NAME).arg(s_M3C2Params.useMedian ? "IQR" : "STD")));
			s_M3C2Params.stdDevCloud2SF->link();
			if (!s_M3C2Params.stdDevCloud2SF->resize(corePointCount, true, NAN_VALUE))
			{
				m_app->dispToConsole("Failed to allocate memory for cloud #2 std. dev. values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.stdDevCloud2SF->release();
				s_M3C2Params.stdDevCloud2SF = 0;
			}
		}
		if (dlg.exportDensityAtProjScaleCheckBox->isChecked())
		{
			//allocate cloud #1 density SF
			s_M3C2Params.densityCloud1SF = new ccScalarField(DENSITY_CLOUD1_SF_NAME);
			s_M3C2Params.densityCloud1SF->link();
			if (!s_M3C2Params.densityCloud1SF->resize(corePointCount, true, NAN_VALUE))
			{
				m_app->dispToConsole("Failed to allocate memory for cloud #1 density values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.densityCloud1SF->release();
				s_M3C2Params.densityCloud1SF = 0;
			}
			//allocate cloud #2 density SF
			s_M3C2Params.densityCloud2SF = new ccScalarField(DENSITY_CLOUD2_SF_NAME);
			s_M3C2Params.densityCloud2SF->link();
			if (!s_M3C2Params.densityCloud2SF->resize(corePointCount, true, NAN_VALUE))
			{
				m_app->dispToConsole("Failed to allocate memory for cloud #2 density values!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				s_M3C2Params.densityCloud2SF->release();
				s_M3C2Params.densityCloud2SF = 0;
			}
		}

		//get best levels for neighbourhood extraction on both octrees
		assert(s_M3C2Params.cloud1Octree && s_M3C2Params.cloud2Octree);

		s_M3C2Params.level1 = s_M3C2Params.cloud1Octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(2.5)*s_M3C2Params.projectionRadius); //2.5 = empirical!
		m_app->dispToConsole(QString("[M3C2] Working subdivision level (cloud #1): %1").arg(s_M3C2Params.level1), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		s_M3C2Params.level2 = s_M3C2Params.cloud2Octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(2.5)*s_M3C2Params.projectionRadius); //2.5 = empirical!
		m_app->dispToConsole(QString("[M3C2] Working subdivision level (cloud #2): %1").arg(s_M3C2Params.level2), ccMainAppInterface::STD_CONSOLE_MESSAGE);

		//other options
		s_M3C2Params.updateNormal = (normMode != qM3C2Normals::VERT_MODE);
		s_M3C2Params.exportNormal = s_M3C2Params.updateNormal && !s_M3C2Params.outputCloud->hasNormals();
		if (s_M3C2Params.exportNormal && !s_M3C2Params.outputCloud->resizeTheNormsTable()) //resize because we will 'set' the normal in ComputeM3C2DistForPoint
		{
			m_app->dispToConsole("Failed to allocate memory for exporting normals!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			s_M3C2Params.exportNormal = false;
		}
		s_M3C2Params.computeConfidence = (s_M3C2Params.distUncertaintySF || s_M3C2Params.sigChangeSF);

		//compute distances
		{
			std::vector<unsigned> pointIndexes;
			bool useParallelStrategy = true;
#ifdef _DEBUG
			useParallelStrategy = false;
#endif
			if (useParallelStrategy)
			{
				try
				{
					pointIndexes.resize(corePointCount);
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					useParallelStrategy = false;
				}
			}

			if (useParallelStrategy)
			{
				for (unsigned i = 0; i < corePointCount; ++i)
				{
					pointIndexes[i] = i;
				}

				if (maxThreadCount == 0)
				{
					maxThreadCount = QThread::idealThreadCount();
				}
				assert(maxThreadCount > 0 && maxThreadCount <= QThread::idealThreadCount());
				QThreadPool::globalInstance()->setMaxThreadCount(maxThreadCount);
				QtConcurrent::blockingMap(pointIndexes, ComputeM3C2DistForPoint);
			}
			else
			{
				//manually call the static per-point method!
				for (unsigned i = 0; i < corePointCount; ++i)
				{
					ComputeM3C2DistForPoint(i);
				}
			}
		}

		if (s_M3C2Params.processCanceled)
		{
			m_app->dispToConsole("Process canceled by user!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			error = true;
		}
		else
		{
			qint64 distTime_ms = distCompTimer.elapsed();
			//we display init. timing only if no error occurred!
			m_app->dispToConsole(QString("[M3C2] Distances computation: %1 s.").arg(static_cast<double>(distTime_ms) / 1000.0, 0, 'f', 3), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}

		s_M3C2Params.nProgress = 0;

		break; //to break from fake loop
	}

	//associate scalar fields to the output cloud
	//(use reverse order so as to get the index of
	//the most important one at the end)
	if (!error)
	{
		assert(s_M3C2Params.outputCloud && s_M3C2Params.corePoints);
		int sfIdx = -1;

		//normal scales
		if (normalScaleSF)
		{
			normalScaleSF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, normalScaleSF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(normalScaleSF);
		}

		//add clouds' density SFs to output cloud
		if (s_M3C2Params.densityCloud1SF)
		{
			s_M3C2Params.densityCloud1SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.densityCloud1SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.densityCloud1SF);
		}
		if (s_M3C2Params.densityCloud2SF)
		{
			s_M3C2Params.densityCloud2SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.densityCloud2SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.densityCloud2SF);
		}

		//add clouds' std. dev. SFs to output cloud
		if (s_M3C2Params.stdDevCloud1SF)
		{
			s_M3C2Params.stdDevCloud1SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.stdDevCloud1SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.stdDevCloud1SF);
		}
		if (s_M3C2Params.stdDevCloud2SF)
		{
			//add cloud #2 std. dev. SF to output cloud
			s_M3C2Params.stdDevCloud2SF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.stdDevCloud2SF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.stdDevCloud2SF);
		}

		if (s_M3C2Params.sigChangeSF)
		{
			//add significance SF to output cloud
			s_M3C2Params.sigChangeSF->computeMinAndMax();
			s_M3C2Params.sigChangeSF->setMinDisplayed(SCALAR_ONE);
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.sigChangeSF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.sigChangeSF);
		}

		if (s_M3C2Params.distUncertaintySF)
		{
			//add dist. uncertainty SF to output cloud
			s_M3C2Params.distUncertaintySF->computeMinAndMax();
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.distUncertaintySF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.distUncertaintySF);
		}

		if (s_M3C2Params.m3c2DistSF)
		{
			//add M3C2 distances SF to output cloud
			s_M3C2Params.m3c2DistSF->computeMinAndMax();
			s_M3C2Params.m3c2DistSF->setSymmetricalScale(true);
			//in case the output cloud is the original cloud, we must remove the former SF
			RemoveScalarField(s_M3C2Params.outputCloud, s_M3C2Params.m3c2DistSF->getName());
			sfIdx = s_M3C2Params.outputCloud->addScalarField(s_M3C2Params.m3c2DistSF);
		}

		s_M3C2Params.outputCloud->invalidateBoundingBox(); //see 'const_cast<...>' in ComputeM3C2DistForPoint ;)
		s_M3C2Params.outputCloud->setCurrentDisplayedScalarField(sfIdx);
		s_M3C2Params.outputCloud->showSF(true);
		s_M3C2Params.outputCloud->showNormals(true);
		s_M3C2Params.outputCloud->setVisible(true);

		if (s_M3C2Params.outputCloud != s_M3C2Params.corePoints)
		{
			s_M3C2Params.outputCloud->setName(outputName);
			s_M3C2Params.outputCloud->setDisplay(s_M3C2Params.corePoints->getDisplay());
			s_M3C2Params.outputCloud->importParametersFrom(s_M3C2Params.corePoints);
			m_app->addToDB(s_M3C2Params.outputCloud);
		}
	}
	else if (s_M3C2Params.outputCloud)
	{
		if (s_M3C2Params.outputCloud != s_M3C2Params.corePoints)
		{
			delete s_M3C2Params.outputCloud;
		}
		s_M3C2Params.outputCloud = 0;
	}

	m_app->refreshAll();

	//release structures
	if (normalScaleSF)
		normalScaleSF->release();
	if (s_M3C2Params.coreNormals)
		s_M3C2Params.coreNormals->release();
	if (s_M3C2Params.m3c2DistSF)
		s_M3C2Params.m3c2DistSF->release();
	if (s_M3C2Params.sigChangeSF)
		s_M3C2Params.sigChangeSF->release();
	if (s_M3C2Params.distUncertaintySF)
		s_M3C2Params.distUncertaintySF->release();
	if (s_M3C2Params.stdDevCloud1SF)
		s_M3C2Params.stdDevCloud1SF->release();
	if (s_M3C2Params.stdDevCloud2SF)
		s_M3C2Params.stdDevCloud2SF->release();
	if (s_M3C2Params.densityCloud1SF)
		s_M3C2Params.densityCloud1SF->release();
	if (s_M3C2Params.densityCloud2SF)
		s_M3C2Params.densityCloud2SF->release();
}

QIcon qM3C2Plugin::getIcon() const
{
	return QIcon(":/CC/plugin/qM3C2Plugin/iconM3C2.png");
}
