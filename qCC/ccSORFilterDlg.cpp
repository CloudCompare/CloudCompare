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

#include "ccSORFilterDlg.h"
#include "ui_sorFilterDlg.h"


ccSORFilterDlg::ccSORFilterDlg(QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::SorFilterDialog )
{
	m_ui->setupUi(this);
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
	m_ui->knnSpinBox->setValue( knn );
}

double ccSORFilterDlg::nSigma() const
{
	return m_ui->nSigmaDoubleSpinBox->value();
}

void ccSORFilterDlg::setNSigma(double nSigma)
{
	m_ui->nSigmaDoubleSpinBox->setValue( nSigma );
}
