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

#include "ccOrderChoiceDlg.h"

//common
#include <ccQtHelpers.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QMainWindow>

//ui template
#include <ui_roleChoiceDlg.h>

ccOrderChoiceDlg::ccOrderChoiceDlg(	ccHObject* firstEntity,
									QString firstRole,
									ccHObject* secondEntity,
									QString secondRole,
									ccMainAppInterface* app/*=0*/)
	: QDialog(app ? app->getMainWindow() : nullptr, Qt::Tool)
	, m_gui(new Ui_RoleChoiceDialog)
	, m_app(app)
	, m_firstEnt(firstEntity)
	, m_secondEnt(secondEntity)
	, m_useInputOrder(true)
{
	m_gui->setupUi(this);

	connect(m_gui->swapButton, &QAbstractButton::clicked, this, &ccOrderChoiceDlg::swap);

	m_gui->firstlabel->setText(firstRole);
	m_gui->secondlabel->setText(secondRole);

	ccQtHelpers::SetButtonColor(m_gui->firstColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(m_gui->secondColorButton, Qt::yellow);

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
	
	if (m_gui)
	{
		delete m_gui;
		m_gui = nullptr;
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
		m_gui->firstLineEdit->setText(o1->getName());
		o1->setEnabled(true);
		o1->setVisible(true);
		o1->setTempColor(ccColor::red);
		o1->prepareDisplayForRefresh_recursive();
	}
	else
	{
		m_gui->firstLineEdit->setText("No entity!");
	}

	ccHObject* o2 = getSecondEntity();
	if (o2)
	{
		m_gui->secondLineEdit->setText(o2->getName());
		o2->setEnabled(true);
		o2->setVisible(true);
		o2->setTempColor(ccColor::yellow);
		o2->prepareDisplayForRefresh_recursive();
	}
	else
	{
		m_gui->secondLineEdit->setText("No entity!");
	}

	if (m_app)
		m_app->refreshAll();
}

void ccOrderChoiceDlg::swap()
{
	m_useInputOrder = !m_useInputOrder;
	setColorsAndLabels();
}
