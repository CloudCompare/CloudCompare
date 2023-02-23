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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "ccKrigingParamsDialog.h"

//ui
#include <ui_krigingParamsDialog.h>

//system
#include <assert.h>

ccKrigingParamsDialog::ccKrigingParamsDialog(QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, m_ui(new Ui_KrigingParamsDialog)
{
	m_ui->setupUi(this);
}

ccKrigingParamsDialog::~ccKrigingParamsDialog()
{
	if (m_ui)
	{
		delete m_ui;
		m_ui = nullptr;
	}
}

void ccKrigingParamsDialog::setParameters(const ccRasterGrid::KrigingParams& krigingParams)
{
	// altitudes
	m_ui->krigeParamsGroupBox->setChecked(!krigingParams.autoGuess);
	{
		m_ui->nuggetDoubleSpinBox->setValue(krigingParams.params.nugget);
		m_ui->sillDoubleSpinBox->setValue(krigingParams.params.sill);
		m_ui->rangeDoubleSpinBox->setValue(krigingParams.params.range);
	}

	// common
	m_ui->modelComboBox->setCurrentIndex(static_cast<int>(krigingParams.params.model != Kriging::Invalid ? krigingParams.params.model : Kriging::Spherical));
	m_ui->knnSpinBox->setValue(krigingParams.kNN);
}

void ccKrigingParamsDialog::getParameters(ccRasterGrid::KrigingParams& krigingParams)
{
	krigingParams.autoGuess = !m_ui->krigeParamsGroupBox->isChecked();
	{
		krigingParams.params.model = static_cast<Kriging::Model>(m_ui->modelComboBox->currentIndex());
		krigingParams.params.nugget = m_ui->nuggetDoubleSpinBox->value();
		krigingParams.params.sill = m_ui->sillDoubleSpinBox->value();
		krigingParams.params.range = m_ui->rangeDoubleSpinBox->value();
	}

	krigingParams.kNN = m_ui->knnSpinBox->value();
}
