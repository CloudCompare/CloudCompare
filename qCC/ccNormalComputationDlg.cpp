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

#include "ccNormalComputationDlg.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccProgressDialog.h>

//Qt
#include <QComboBox>

//system
#include <assert.h>

ccNormalComputationDlg::ccNormalComputationDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, Ui::NormalComputationDlg()
	, m_cloud(0)
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	//by default, the 'auto' button is hidden (as long as setCloud is not called)
	autoRadiusToolButton->setVisible(false);

	connect(localModelComboBox,   SIGNAL(currentIndexChanged(int)), this, SLOT(localModelChanged(int)));
	connect(autoRadiusToolButton, SIGNAL(clicked()),                this, SLOT(autoEstimateRadius()));
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
	localRadiusFrame->setEnabled(index != 2);
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
		preferredOrientationGroupBox->setChecked(false);
	}
	else
	{
		preferredOrientationGroupBox->setChecked(true);
		preferredOrientationComboBox->setCurrentIndex(orientation);
	}
}

ccNormalVectors::Orientation ccNormalComputationDlg::getPreferredOrientation() const
{
	int index = preferredOrientationGroupBox->isChecked() ? preferredOrientationComboBox->currentIndex() : -1;

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
		ccProgressDialog pDlg(true,this);
		if (!m_cloud->computeOctree(&pDlg))
		{
			ccLog::Error(QString("Could not compute octree for cloud '%1'").arg(m_cloud->getName()));
			autoRadiusToolButton->setVisible(false);
			return;
		}
	}

	PointCoordinateType radius = ccNormalVectors::GuessBestRadius(m_cloud,m_cloud->getOctree());
	if (radius > 0)
	{
		radiusDoubleSpinBox->setValue(radius);
	}
}