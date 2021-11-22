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

#include "FastGlobalRegistrationDlg.h"

//common
#include <ccQtHelpers.h>

//qCC_db
#include <ccOctree.h>
#include <ccPointCloud.h>

//system
#include <assert.h>

static double s_featureRadius = 0;

FastGlobalRegistrationDialog::FastGlobalRegistrationDialog(	const std::vector<ccPointCloud*>& allClouds,
															QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::FastGlobalRegistrationDialog()
	, clouds(allClouds)
	, referencesCloudUinqueID(ccUniqueIDGenerator::InvalidUniqueID)
{
	setupUi(this);

	ccQtHelpers::SetButtonColor(dataColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(modelColorButton, Qt::yellow);

	for (ccPointCloud* cloud : clouds)
	{
		referenceComboBox->addItem(cloud->getName(), cloud->getUniqueID());
	}

	if (!clouds.empty())
	{
		referencesCloudUinqueID = clouds.front()->getUniqueID();
	}

	updateGUI();

	//restore semi-persistent settings
	{
		//semi-persistent options
		double previousRadius = s_featureRadius;
		if (previousRadius == 0)
		{
			for (ccPointCloud* cloud : clouds)
			{
				double radius = ccOctree::GuessNaiveRadius(cloud);
				previousRadius = std::max(radius, previousRadius);
			}
		}

		featureRadiusDoubleSpinBox->setValue(previousRadius);
	}

	connect(autoRadiusToolButton, &QToolButton::clicked, this, &FastGlobalRegistrationDialog::autoEstimateRadius);
	connect(referenceComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &FastGlobalRegistrationDialog::referenceEntityChanged);
}

FastGlobalRegistrationDialog::~FastGlobalRegistrationDialog()
{
	for (ccPointCloud* cloud : clouds)
	{
		cloud->enableTempColor(false);
		cloud->prepareDisplayForRefresh_recursive();
	}

	for (ccPointCloud* cloud : clouds)
	{
		cloud->refreshDisplay();
	}
}

void FastGlobalRegistrationDialog::saveParameters() const
{
	s_featureRadius = getFeatureRadius();
}

ccPointCloud* FastGlobalRegistrationDialog::getReferenceCloud()
{
	for (ccPointCloud* cloud : clouds)
	{
		if (cloud->getUniqueID() == referencesCloudUinqueID)
			return cloud;
	}

	return nullptr;
}

double FastGlobalRegistrationDialog::getFeatureRadius() const
{
	return featureRadiusDoubleSpinBox->value();
}

void FastGlobalRegistrationDialog::updateGUI()
{
	if (clouds.size() < 2)
		return;
	
	ccPointCloud* referenceCloud = getReferenceCloud();
	if (!referenceCloud)
	{
		assert(false);
		return;
	}

	// aligned cloud(s)
	ccPointCloud* alignedCloud = nullptr; //only one of them
	int referenceCloudIndex = -1;
	for (size_t i = 0; i< clouds.size(); ++i)
	{
		ccPointCloud* cloud = clouds[i];
		if (cloud->getUniqueID() != referencesCloudUinqueID)
		{
			alignedCloud = cloud;
			alignedCloud->setVisible(true);
			alignedCloud->setTempColor(ccColor::red);
			alignedCloud->prepareDisplayForRefresh_recursive();
		}
		else
		{
			referenceCloudIndex = static_cast<int>(i);
		}
	}
	alignedLineEdit->setText(clouds.size() == 2 ? alignedCloud->getName() : tr("%1 other clouds").arg(clouds.size() - 1));

	// reference cloud
	referenceCloud->setVisible(true);
	referenceCloud->setTempColor(ccColor::yellow);
	referenceCloud->prepareDisplayForRefresh_recursive();

	referenceComboBox->blockSignals(true);
	referenceComboBox->setCurrentIndex(referenceCloudIndex);
	referenceComboBox->blockSignals(false);

	// refersh display(s)
	for (ccPointCloud* cloud : clouds)
	{
		cloud->refreshDisplay();
	}
}

void FastGlobalRegistrationDialog::referenceEntityChanged(int index)
{
	referencesCloudUinqueID = referenceComboBox->itemData(index).toUInt();

	updateGUI();
}

void FastGlobalRegistrationDialog::autoEstimateRadius()
{
	ccOctree::BestRadiusParams params;
	{
		params.aimedPopulationPerCell = 64;
		params.aimedPopulationRange = 16;
		params.minCellPopulation = 48;
		params.minAboveMinRatio = 0.97;
	}

	PointCoordinateType largestRadius = 0.0;
	for (ccPointCloud* cloud : clouds)
	{
		PointCoordinateType radius = ccOctree::GuessBestRadiusAutoComputeOctree(cloud, params, this);
		if (radius < 0)
		{
			ccLog::Error(tr("Failed to estimate the radius for cloud %1").arg(cloud->getName()));
			return;
		}
		largestRadius = std::max(largestRadius, radius);
	}

	featureRadiusDoubleSpinBox->setValue(largestRadius);
}
