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

#include "ccNormalComputationDlg.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccProgressDialog.h>

//Qt
#include <QComboBox>

//system
#include <assert.h>

ccNormalComputationDlg::ccNormalComputationDlg(bool withScanGrid, bool withSensor, QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::NormalComputationDlg()
	, m_cloud(nullptr)
{
	setupUi(this);

	//by default, the 'auto' button is hidden (as long as setCloud is not called)
	autoRadiusToolButton->setVisible(false);

	connect(localModelComboBox,			SIGNAL(currentIndexChanged(int)), this, SLOT(localModelChanged(int)));
	connect(autoRadiusToolButton,		SIGNAL(clicked()),                this, SLOT(autoEstimateRadius()));

	if (withScanGrid)
	{
		useScanGridCheckBox->setChecked(true);
		scanGridsOrientCheckBox->setChecked(true);
	}
	else
	{
		//disable 'scan grid' options
		useScanGridCheckBox->setChecked(false);
		useScanGridCheckBox->setEnabled(false);
		gridAngleFrame->setEnabled(false);

		scanGridsOrientCheckBox->setChecked(false);
		scanGridsOrientCheckBox->setEnabled(false);
	}

	if (withSensor)
	{
		sensorOrientCheckBox->setChecked(true);
	}
	else
	{
		//disable 'sensor' options
		sensorOrientCheckBox->setChecked(false);
		sensorOrientCheckBox->setEnabled(false);
	}
}

void ccNormalComputationDlg::setLocalModel(CC_LOCAL_MODEL_TYPES  model)
{
	int index = -1;
	switch (model)
	{
	case LS:
		index = 0;
		break;
	case QUADRIC:
		index = 1;
		break;
	case TRI:
		index = 2;
		break;
	default:
		assert(false);
		break;
	}
	localModelComboBox->setCurrentIndex(index);
}

CC_LOCAL_MODEL_TYPES ccNormalComputationDlg::getLocalModel() const
{
	switch (localModelComboBox->currentIndex())
	{
	case 0:
		return LS;
	case 1:
		return QUADRIC;
	case 2:
		return TRI;
	}

	assert(false);
	return LS;
}

void ccNormalComputationDlg::localModelChanged(int index)
{
	//DGM: we don't disable the parent frame anymore as it is used by the octree/grid toggling
	radiusDoubleSpinBox->setEnabled(index != 2);
	autoRadiusToolButton->setEnabled(index != 2);
}

void ccNormalComputationDlg::setRadius(PointCoordinateType radius)
{
	radiusDoubleSpinBox->setValue(static_cast<double>(radius));
}

void ccNormalComputationDlg::setCloud(ccPointCloud* cloud)
{
	m_cloud = cloud;

	autoRadiusToolButton->setVisible(m_cloud != 0);
}

PointCoordinateType ccNormalComputationDlg::getRadius() const
{
	return static_cast<PointCoordinateType>(radiusDoubleSpinBox->value());
}

void ccNormalComputationDlg::setPreferredOrientation(ccNormalVectors::Orientation orientation)
{
	if (orientation == ccNormalVectors::UNDEFINED)
	{
		preferredOrientRadioButton->setChecked(false);
	}
	else
	{
		preferredOrientRadioButton->setChecked(true);
		preferredOrientationComboBox->setCurrentIndex(orientation);
	}
}

bool ccNormalComputationDlg::useScanGridsForComputation() const
{
	return useScanGridCheckBox->isChecked();
}

double ccNormalComputationDlg::getMinGridAngle_deg() const
{
	return gridAngleDoubleSpinBox->value();
}

void ccNormalComputationDlg::setMinGridAngle_deg(double value)
{
	gridAngleDoubleSpinBox->setValue(value);
}

bool ccNormalComputationDlg::orientNormals() const
{
	return normalsOrientGroupBox->isChecked();
}

bool ccNormalComputationDlg::useScanGridsForOrientation() const
{
	return scanGridsOrientCheckBox->isChecked();
}

bool ccNormalComputationDlg::useSensorsForOrientation() const
{
	return sensorOrientCheckBox->isChecked();
}

bool ccNormalComputationDlg::usePreferredOrientation() const
{
	return preferredOrientRadioButton->isChecked();
}

bool ccNormalComputationDlg::useMSTOrientation() const
{
	return mstOrientRadioButton->isChecked();
}

void ccNormalComputationDlg::setMSTNeighborCount(int n)
{
	mstNeighborsSpinBox->setValue(n);
}

int ccNormalComputationDlg::getMSTNeighborCount() const
{
	return mstNeighborsSpinBox->value();
}

ccNormalVectors::Orientation ccNormalComputationDlg::getPreferredOrientation() const
{
	int index = preferredOrientRadioButton->isChecked() ? preferredOrientationComboBox->currentIndex() : -1;

	return (index >= 0 && index <= ccNormalVectors::PREVIOUS ? static_cast<ccNormalVectors::Orientation>(index) : ccNormalVectors::UNDEFINED);
}

void ccNormalComputationDlg::autoEstimateRadius()
{
	if (!m_cloud)
	{
		assert(false);
		return;
	}

	if (!m_cloud->getOctree())
	{
		ccProgressDialog pDlg(true, this);
		if (!m_cloud->computeOctree(&pDlg))
		{
			ccLog::Error(QString("Could not compute octree for cloud '%1'").arg(m_cloud->getName()));
			autoRadiusToolButton->setVisible(false);
			return;
		}
	}

	PointCoordinateType radius = ccNormalVectors::GuessBestRadius(m_cloud, m_cloud->getOctree().data());
	if (radius > 0)
	{
		radiusDoubleSpinBox->setValue(radius);
	}
}
