//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#include "qCanupo.h"

//local
#include "qCanupoClassifDialog.h"
#include "qCanupoTrainingDialog.h"
#include "qCanupo2DViewDialog.h"
#include "qCanupoTools.h"
#include "trainer.h"
#include "qCanupoDisclaimerDialog.h"
#include "qCanupoCommands.h"

//CCLib
#include <CloudSamplingTools.h>
#include <ReferenceCloud.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccPolyline.h>
#include <ccSphere.h>
#include <ccScalarField.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QMessageBox>
#include <QStringList>

qCanupoPlugin::qCanupoPlugin(QObject* parent/*=0*/)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qCanupoPlugin/info.json" )
	, m_classifyAction(nullptr)
	, m_trainAction(nullptr)
{
}

void qCanupoPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_classifyAction)
	{
		//classification: only one point cloud
		m_classifyAction->setEnabled(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
	}

	if (m_trainAction)
	{
		m_trainAction->setEnabled(m_app && m_app->dbRootObject() && m_app->dbRootObject()->getChildrenNumber() != 0); //need some loaded entities to train the classifier!
	}

	m_selectedEntities = selectedEntities;
}

QList<QAction*> qCanupoPlugin::getActions()
{
	QList<QAction*> group;

	if (!m_trainAction)
	{
		m_trainAction = new QAction("Train classifier", this);
		m_trainAction->setToolTip("Train classifier");
		m_trainAction->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qCanupoPlugin/iconCreate.png")));
		connect(m_trainAction, SIGNAL(triggered()), this, SLOT(doTrainAction()));
	}
	group.push_back(m_trainAction);

	if (!m_classifyAction)
	{
		m_classifyAction = new QAction("Classify", this);
		m_classifyAction->setToolTip("Classify cloud");
		m_classifyAction->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qCanupoPlugin/iconClassify.png")));
		connect(m_classifyAction, SIGNAL(triggered()), this, SLOT(doClassifyAction()));
	}
	group.push_back(m_classifyAction);

	return group;
}

void qCanupoPlugin::doClassifyAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	//disclaimer accepted?
	if (!ShowClassifyDisclaimer(m_app))
	{
		return;
	}

	if (m_selectedEntities.empty() || !m_selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one and only one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(m_selectedEntities.front());

	//display dialog
	qCanupoClassifDialog cDlg(cloud, m_app);
	if (!cDlg.exec())
	{
		//process cancelled by the user
		return;
	}
	cDlg.saveParamsToPersistentSettings();

	//store parameters
	qCanupoProcess::ClassifyParams params;
	{
		params.confidenceThreshold = cDlg.getConfidenceTrehshold();
		params.generateAdditionalSF = cDlg.generateAdditionalSFsCheckBox->isChecked();
		params.generateRoughnessSF = cDlg.generateRoughnessSFsCheckBox->isChecked();
		params.maxThreadCount = cDlg.getMaxThreadCount();
		params.useActiveSFForConfidence = cDlg.useSF();
		params.samplingDist = 0; //only set if we subsample the input cloud!

	}
	QString classifierFilename = cDlg.classifFileLineEdit->text();

	//how should we generate the core points?
	qCanupoClassifDialog::CORE_CLOUD_SOURCES coreSource = cDlg.getCorePointsCloudSource();
	PointCoordinateType samplingDist = static_cast<PointCoordinateType>(cDlg.cpSubsamplingDoubleSpinBox->value());

	CorePointDescSet corePointsDescriptors; //core point descriptors
	ccPointCloud* realCorePoints = nullptr; //the core point cloud (as a real point cloud, if available)
	CCLib::GenericIndexedCloudPersist* corePoints = nullptr; //the core points, potentially as references!

	switch (coreSource)
	{
	case qCanupoClassifDialog::ORIGINAL:
	case qCanupoClassifDialog::OTHER:
	{
		realCorePoints = cDlg.getCorePointsCloud();
		if (!realCorePoints)
		{
			assert(false);
			m_app->dispToConsole("Internal error: failed to access core pointss?!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
		corePoints = realCorePoints;
	}
	break;

	case qCanupoClassifDialog::SUBSAMPLED:
	{
		//progress dialog
		ccProgressDialog pDlg(true, m_app->getMainWindow());

		assert(samplingDist > 0);
		CCLib::CloudSamplingTools::SFModulationParams modParams(false);
		CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(cloud,
																							samplingDist,
																							modParams,
																							nullptr,
																							&pDlg);
		if (!refCloud)
		{
			m_app->dispToConsole("Failed to compute sub-sampled core points!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		params.samplingDist = samplingDist;

		//try to convert the references into a real point cloud (not mandatory though!)
		realCorePoints = cloud->partialClone(refCloud);
		if (realCorePoints)
		{
			realCorePoints->setName(cloud->getName() + QString(".core points (subsampled @ %1)").arg(samplingDist));
			cloud->addChild(realCorePoints);
			m_app->addToDB(realCorePoints);
			corePoints = realCorePoints;
		}
		else
		{
			m_app->dispToConsole("Can't save subsampled cloud (not enough memory)!");
		}

		delete refCloud;
		refCloud = nullptr;
	}
	break;

	case qCanupoClassifDialog::MSC_FILE:
	{
		realCorePoints = new ccPointCloud("MSC core points");
		QString filenmae = cDlg.getMscFilename();
		QString error;
		if (!corePointsDescriptors.loadFromMSC(filenmae, error, realCorePoints))
		{
			//failed to read the input MSC file
			if (realCorePoints)
			{
				delete realCorePoints;
				realCorePoints = nullptr;
			}
			m_app->dispToConsole(error, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
		else if (!error.isNull())
		{
			//then it's just a warning
			m_app->dispToConsole(QString("[qCanupo] ") + error, ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}

		assert(realCorePoints);
		cloud->addChild(realCorePoints);
		m_app->addToDB(realCorePoints);
		corePoints = realCorePoints;
	}
	break;

	default:
	{
		assert(false);
		m_app->dispToConsole("Internal error: no core point source specified?!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	}

	assert(corePoints);

	if (qCanupoProcess::Classify(classifierFilename, params, cloud, corePoints, corePointsDescriptors, realCorePoints, m_app, m_app->getMainWindow()))
	{
		cloud->prepareDisplayForRefresh();
		m_app->refreshAll();
		m_app->updateUI();
	}

	//dispose of the 'virtual' core points (if any)
	if (corePoints != realCorePoints)
	{
		delete corePoints;
		corePoints = nullptr;
	}
}

void qCanupoPlugin::doTrainAction()
{
	//disclaimer accepted?
	if (!ShowTrainDisclaimer(m_app))
		return;

	//if (m_selectedEntities.size() != 2
	//	|| !m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)
	//	|| !m_selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
	//{
	//	m_app->dispToConsole("Select two point clouds!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	//	return;
	//}
	//
	//ccPointCloud* cloud1 = static_cast<ccPointCloud*>(m_selectedEntities[0]);
	//ccPointCloud* cloud2 = static_cast<ccPointCloud*>(m_selectedEntities[1]);

	//display dialog
	qCanupoTrainingDialog ctDlg(/*cloud1, cloud2, */m_app);
	if (!ctDlg.exec())
		return;
	ctDlg.saveParamsToPersistentSettings();

	//get scales
	std::vector<float> scales;
	{
		if (!ctDlg.getScales(scales))
		{
			m_app->dispToConsole("Invalid scale parameters!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
		//make sure values are in descending order!
		std::sort(scales.begin(), scales.end(), std::greater<float>());
	}

	ccPointCloud* originCloud = ctDlg.getOriginPointCloud();
	ccPointCloud* cloud1 = ctDlg.getClass1Cloud();
	ccPointCloud* cloud2 = ctDlg.getClass2Cloud();
	if (!cloud1 || !cloud2)
	{
		if (m_app)
			m_app->dispToConsole("At least one cloud (class #1 or #2) was not defined!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	assert(cloud1 != cloud2);
	ccPointCloud* evaluationCloud = ctDlg.getEvaluationCloud();

	//Descriptor ID
	unsigned descriptorID = ctDlg.getDescriptorID();
	//check that the selected descriptor (computer) is valid
	{
		assert(descriptorID != 0);
		ScaleParamsComputer* computer = ScaleParamsComputer::GetByID(descriptorID);
		if (!computer)
		{
			if (m_app)
				m_app->dispToConsole(QString("Internal error: unhandled descriptor ID (%1)!").arg(descriptorID), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
		if (computer->needSF()
			&& (cloud1->getCurrentDisplayedScalarField() == nullptr
			|| cloud2->getCurrentDisplayedScalarField() == nullptr
			|| (evaluationCloud && evaluationCloud->getCurrentDisplayedScalarField() == nullptr)
			)
			)
		{
			if (m_app)
				m_app->dispToConsole(QString("To compute this type of descriptor, all clouds must have an active scalar field!"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}

	//sub-sampled clouds
	CCLib::GenericIndexedCloudPersist* corePoints1 = nullptr;
	CCLib::GenericIndexedCloudPersist* corePoints2 = nullptr;
	CCLib::GenericIndexedCloudPersist* evaluationPoints = nullptr;

	//progress dialog
	ccProgressDialog pDlg(true, m_app->getMainWindow());

	while (true)
	{
		//sub-sample clouds (if necessary)
		{
			assert(ctDlg.maxPointsSpinBox->value() > 0);
			unsigned maxCorePoints = static_cast<unsigned>(ctDlg.maxPointsSpinBox->value());

			//if the user has specified a third cloud for behavior representation
			if (evaluationCloud)
			{
				if (evaluationCloud->size() > maxCorePoints)
					evaluationPoints = CCLib::CloudSamplingTools::subsampleCloudRandomly(evaluationCloud, maxCorePoints, &pDlg);
				else
					evaluationPoints = evaluationCloud;

				if (!evaluationPoints)
				{
					m_app->dispToConsole("Failed to compute sub-sampled version of evaluation points!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					break;
				}
			}

			//for class clouds, we take the smallest count (if inferior to the specified limit)
			if (maxCorePoints > cloud1->size())
				maxCorePoints = cloud1->size();
			if (maxCorePoints > cloud2->size())
				maxCorePoints = cloud2->size();

			if (cloud1->size() > maxCorePoints)
				corePoints1 = CCLib::CloudSamplingTools::subsampleCloudRandomly(cloud1, maxCorePoints, &pDlg);
			else
				corePoints1 = cloud1;

			if (!corePoints1)
			{
				m_app->dispToConsole("Failed to compute sub-sampled version of cloud #1!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				break;
			}

			if (cloud2->size() > maxCorePoints)
				corePoints2 = CCLib::CloudSamplingTools::subsampleCloudRandomly(cloud2, maxCorePoints, &pDlg);
			else
				corePoints2 = cloud2;

			if (!corePoints2)
			{
				m_app->dispToConsole("Failed to compute sub-sampled version of cloud #1!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				break;
			}
		}

		//compute MSC data for cloud #1
		CorePointDescSet descriptors1;
		{
			bool invalidDescriptors = false;
			QString errorStr;
			if (!qCanupoTools::ComputeCorePointsDescriptors(corePoints1,
				descriptors1,
				//if the origin cloud was specified, then we'll use it as base cloud for descriptors
				originCloud ? originCloud : cloud1,
				scales,
				invalidDescriptors,
				errorStr,
				descriptorID,
				ctDlg.getMaxThreadCount(),
				&pDlg/*,
				octree*/))
			{
				m_app->dispToConsole(QString("Failed to compute core points descriptors: %1").arg(errorStr), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				break;
			}
			else if (invalidDescriptors)
			{
				m_app->dispToConsole("[qCanupo] Some descriptors couldn't be computed on cloud#1 (min scale may be too small)!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}
		}

		//compute MSC data for cloud #2
		CorePointDescSet descriptors2;
		{
			bool invalidDescriptors = false;
			QString errorStr;
			if (!qCanupoTools::ComputeCorePointsDescriptors(corePoints2,
				descriptors2,
				//if the origin cloud was specified, then we'll use it as base cloud for descriptors
				originCloud ? originCloud : cloud2,
				scales,
				invalidDescriptors,
				errorStr,
				descriptorID,
				ctDlg.getMaxThreadCount(),
				&pDlg/*,
				octree*/))
			{
				m_app->dispToConsole(QString("Failed to compute core points descriptors: %1").arg(errorStr), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				break;
			}
			else if (invalidDescriptors)
			{
				m_app->dispToConsole("[qCanupo] Some descriptors couldn't be computed on cloud#2 (min scale may be too small)!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}
		}

		//if the user has specified a third cloud for behavior representation
		//we must compute its descriptors now
		CorePointDescSet evaluationDescriptors;
		if (evaluationPoints)
		{
			//computes the 'descriptors'
			bool invalidDescriptors = false;
			QString errorStr;
			if (!qCanupoTools::ComputeCorePointsDescriptors(evaluationPoints,
				evaluationDescriptors,
				evaluationCloud,
				scales,
				invalidDescriptors,
				errorStr,
				descriptorID,
				ctDlg.getMaxThreadCount(),
				&pDlg))
			{
				m_app->dispToConsole(QString("Failed to compute core points descriptors: %1").arg(errorStr), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				break;
			}

			if (invalidDescriptors)
			{
				m_app->dispToConsole("[qCanupo] Some descriptors couldn't be computed on evaluation cloud (min scale may be too small)!", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			}
		}

		//now for the Classifier training!
		{
			qCanupo2DViewDialog c2DDlg(&descriptors1,
				&descriptors2,
				cloud1->getName(),
				cloud2->getName(),
				ctDlg.cloud1ClassSpinBox->value(),
				ctDlg.cloud2ClassSpinBox->value(),
				evaluationPoints ? &evaluationDescriptors : nullptr,
				m_app);

			//we need the 3D view to be visible before updating the zoom!
			c2DDlg.show();
			c2DDlg.setWindowModality(Qt::ApplicationModal);
			QApplication::processEvents();

			if (!c2DDlg.trainClassifier())
			{
				//a real error message should have already been issued
				m_app->dispToConsole("[qCanupo] Classifier training failed...", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				break;
			}

			c2DDlg.exec();
		}

		//end of the story!
		break;
	}

	if (corePoints1 != cloud1)
		delete corePoints1;
	if (corePoints2 != cloud2)
		delete corePoints2;
	if (evaluationPoints != evaluationCloud)
		delete evaluationPoints;
}

void qCanupoPlugin::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandCanupoClassif));
}
