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


#include "ccSensorComputeDistancesDlg.h"
#include "ui_sensorComputeDistancesDlg.h"


ccSensorComputeDistancesDlg::ccSensorComputeDistancesDlg(QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::sensorComputeDistancesDlg )
{
	m_ui->setupUi(this);
}

ccSensorComputeDistancesDlg::~ccSensorComputeDistancesDlg()
{
	delete m_ui;
}

bool ccSensorComputeDistancesDlg::computeSquaredDistances() const
{
	return m_ui->checkSquaredDistance->checkState() == Qt::Checked;
}
