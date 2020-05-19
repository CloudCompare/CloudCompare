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

#include "ccPickOneElementDlg.h"

//UI file
#include <ui_pickOneElementDlg.h>

ccPickOneElementDlg::ccPickOneElementDlg(	const QString &label,
											const QString &windowTitle/*=QString()*/,
											QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, m_ui(new Ui_PickOneElementDialog)
{
	m_ui->setupUi(this);

	if (!windowTitle.isNull())
	{
		setWindowTitle(windowTitle);
	}

	m_ui->comboLabel->setText(label);
}

ccPickOneElementDlg::~ccPickOneElementDlg()
{
	delete m_ui;
	m_ui = nullptr;
}

void ccPickOneElementDlg::addElement(const QString &elementName)
{
	m_ui->comboBox->addItem(elementName);
}

void ccPickOneElementDlg::setDefaultIndex(int index)
{
	m_ui->comboBox->setCurrentIndex(index);
}

int ccPickOneElementDlg::getSelectedIndex()
{
	return m_ui->comboBox->currentIndex();
}
