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

#include "ccOrderChoiceDlg.h"

// common
#include <ccQtHelpers.h>

// qCC_plugins
#include <ccMainAppInterface.h>

// qCC_db
#include <ccHObject.h>

// Qt
#include <QMainWindow>

// ui template
#include <ui_roleChoiceDlg.h>

ccOrderChoiceDlg::ccOrderChoiceDlg(ccHObject*          firstEntity,
                                   QString             firstRole,
                                   ccHObject*          secondEntity,
                                   QString             secondRole,
                                   ccMainAppInterface* app /*=nullptr*/)
    : QDialog(app ? app->getMainWindow() : nullptr, Qt::Tool)
    , m_ui(std::make_unique<Ui::RoleChoiceDialog>())
    , m_app(app)
    , m_firstEnt(firstEntity)
    , m_secondEnt(secondEntity)
    , m_useInputOrder(true)
{
	m_ui->setupUi(this);

	connect(m_ui->swapButton, &QAbstractButton::clicked, this, &ccOrderChoiceDlg::swap);

	m_ui->firstlabel->setText(firstRole);
	m_ui->secondlabel->setText(secondRole);

	ccQtHelpers::SetButtonColor(m_ui->firstColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(m_ui->secondColorButton, Qt::yellow);

	setColorsAndLabels();
}

ccOrderChoiceDlg::~ccOrderChoiceDlg()
{
	if (m_firstEnt)
	{
		m_firstEnt->enableTempColor(false);
		m_firstEnt->prepareDisplayForRefresh_recursive();
	}
	if (m_secondEnt)
	{
		m_secondEnt->enableTempColor(false);
		m_secondEnt->prepareDisplayForRefresh_recursive();
	}

	if (m_app)
	{
		m_app->refreshAll();
	}
}

ccHObject* ccOrderChoiceDlg::getFirstEntity()
{
	return m_useInputOrder ? m_firstEnt : m_secondEnt;
}

ccHObject* ccOrderChoiceDlg::getSecondEntity()
{
	return m_useInputOrder ? m_secondEnt : m_firstEnt;
}

void ccOrderChoiceDlg::setColorsAndLabels()
{
	ccHObject* o1 = getFirstEntity();
	if (o1)
	{
		m_ui->firstLineEdit->setText(o1->getName());
		o1->setEnabled(true);
		o1->setVisible(true);
		o1->setTempColor(ccColor::red);
		o1->prepareDisplayForRefresh_recursive();
	}
	else
	{
		m_ui->firstLineEdit->setText("No entity!");
	}

	ccHObject* o2 = getSecondEntity();
	if (o2)
	{
		m_ui->secondLineEdit->setText(o2->getName());
		o2->setEnabled(true);
		o2->setVisible(true);
		o2->setTempColor(ccColor::yellow);
		o2->prepareDisplayForRefresh_recursive();
	}
	else
	{
		m_ui->secondLineEdit->setText("No entity!");
	}

	if (m_app)
		m_app->refreshAll();
}

void ccOrderChoiceDlg::swap()
{
	m_useInputOrder = !m_useInputOrder;
	setColorsAndLabels();
}
