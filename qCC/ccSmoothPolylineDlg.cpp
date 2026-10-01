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
// #                    COPYRIGHT: CloudCompare project                     #
// #                                                                        #
// ##########################################################################

#include "ccSmoothPolylineDlg.h"

// ui
#include <ui_smoothPolylineDlg.h>

// Qt
#include <QDialog>

ccSmoothPolylineDialog::ccSmoothPolylineDialog(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , m_ui(std::make_unique<Ui::SmoothPolylineDialog>())
{
	m_ui->setupUi(this);
}

ccSmoothPolylineDialog::~ccSmoothPolylineDialog() = default;

void ccSmoothPolylineDialog::setIerationCount(int count)
{
	m_ui->iterationSpinBox->setValue(count);
}

void ccSmoothPolylineDialog::setRatio(double ratio)
{
	m_ui->ratioDoubleSpinBox->setValue(ratio);
}

int ccSmoothPolylineDialog::getIerationCount() const
{
	return m_ui->iterationSpinBox->value();
}

double ccSmoothPolylineDialog::getRatio() const
{
	return m_ui->ratioDoubleSpinBox->value();
}
