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

#include "ccSORFilterDlg.h"

#include "ui_sorFilterDlg.h"

#include <QThread>

ccSORFilterDlg::ccSORFilterDlg(QWidget* parent /*=nullptr*/)
    : QDialog(parent, Qt::Tool)
    , m_ui(new Ui::SorFilterDialog)
{
	m_ui->setupUi(this);

	static const int MaxThreadCount = QThread::idealThreadCount();
	m_ui->maxThreadCountSpinBox->setRange(1, MaxThreadCount);
	m_ui->maxThreadCountSpinBox->setSuffix(QString(" / %1").arg(MaxThreadCount));
}

ccSORFilterDlg::~ccSORFilterDlg()
{
	delete m_ui;
}

int ccSORFilterDlg::KNN() const
{
	return m_ui->knnSpinBox->value();
}

void ccSORFilterDlg::setKNN(int knn)
{
	m_ui->knnSpinBox->setValue(knn);
}

double ccSORFilterDlg::nSigma() const
{
	return m_ui->nSigmaDoubleSpinBox->value();
}

void ccSORFilterDlg::setNSigma(double nSigma)
{
	m_ui->nSigmaDoubleSpinBox->setValue(nSigma);
}

void ccSORFilterDlg::setMaxThreadCount(int count)
{
	m_ui->maxThreadCountSpinBox->setValue(count);
}

int ccSORFilterDlg::maxThreadCount() const
{
	return m_ui->maxThreadCountSpinBox->value();
}
