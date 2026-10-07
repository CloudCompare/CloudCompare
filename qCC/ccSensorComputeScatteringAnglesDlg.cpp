// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccSensorComputeScatteringAnglesDlg.h"

// Ui
#include <ui_sensorComputeScatteringAnglesDlg.h>

ccSensorComputeScatteringAnglesDlg::ccSensorComputeScatteringAnglesDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , m_ui(std::make_unique<Ui::sensorComputeScatteringAnglesDlg>())
{
	m_ui->setupUi(this);
}

ccSensorComputeScatteringAnglesDlg::~ccSensorComputeScatteringAnglesDlg() = default;

bool ccSensorComputeScatteringAnglesDlg::anglesInDegrees() const
{
	return m_ui->anglesToDegCheckbox->checkState() == Qt::Checked;
}
