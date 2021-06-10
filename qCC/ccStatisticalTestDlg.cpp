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

#include "ccStatisticalTestDlg.h"
#include "ui_statisticalTestDlg.h"


ccStatisticalTestDlg::ccStatisticalTestDlg(	QString p1Label,
											QString p2Label,
											QString p3Label/*=QString()*/,
											QString windowTitle/*=QString()*/,
											QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, m_ui( new Ui::StatisticalTestDialog )
{
	m_ui->setupUi(this);

	m_ui->param1Label->setText(p1Label);
	m_ui->param2Label->setText(p2Label);
	if (!p3Label.isNull())
	{
		m_ui->param3Label->setText(p3Label);
	}
	else
	{
		m_ui->param3Label->setVisible(false);
		m_ui->param3SpinBox->setVisible(false);
	}

	m_ui->neighborsSpinBox->setValue(16);

	if (!windowTitle.isNull())
	{
		setWindowTitle(windowTitle);
	}
}

ccStatisticalTestDlg::~ccStatisticalTestDlg()
{
	delete m_ui;
}

double ccStatisticalTestDlg::getParam1() const
{
	return m_ui->param1SpinBox->value();
}

double ccStatisticalTestDlg::getParam2() const
{
	return m_ui->param2SpinBox->value();
}

double ccStatisticalTestDlg::getParam3() const
{
	return m_ui->param3SpinBox->value();
}

int ccStatisticalTestDlg::getNeighborsNumber() const
{
	return m_ui->neighborsSpinBox->value();
}

double ccStatisticalTestDlg::getProbability() const
{
	return m_ui->probaSpinBox->value();
}
