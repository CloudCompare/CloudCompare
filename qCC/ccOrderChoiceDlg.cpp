//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccOrderChoiceDlg.h"

//local
#include "ccDisplayOptionsDlg.h"

//qCC_plugins
#include <ccMainAppInterface.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QMainWindow>

ccOrderChoiceDlg::ccOrderChoiceDlg(	ccHObject* firstEntity,
									const char* firstRole,
									ccHObject* secondEntity,
									const char* secondRole,
									ccMainAppInterface* app/*=0*/)
	: QDialog(app ? app->getMainWindow() : 0)
	, Ui::RoleChoiceDialog()
	, m_app(app)
	, m_firstEnt(firstEntity)
	, m_secondEnt(secondEntity)
	, m_originalOrder(true)
{
	setupUi(this);
	setWindowFlags(Qt::Tool);

	connect(swapButton, SIGNAL(clicked()), this, SLOT(swap()));

	firstlabel->setText(firstRole);
	secondlabel->setText(secondRole);

	QColor qRed(255,0,0);
	QColor qYellow(255,255,0);
	ccDisplayOptionsDlg::SetButtonColor(firstColorButton,qRed);
	ccDisplayOptionsDlg::SetButtonColor(secondColorButton,qYellow);

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
		m_app->refreshAll();
}

ccHObject* ccOrderChoiceDlg::getFirstEntity()
{
	return m_originalOrder ? m_firstEnt : m_secondEnt;
}

ccHObject* ccOrderChoiceDlg::getSecondEntity()
{
	return m_originalOrder ? m_secondEnt : m_firstEnt;
}

void ccOrderChoiceDlg::setColorsAndLabels()
{
	ccHObject* o1 = getFirstEntity();
	if (o1)
	{
		firstLineEdit->setText(o1->getName());
		o1->setEnabled(true);
		o1->setVisible(true);
		o1->setTempColor(ccColor::red);
		o1->prepareDisplayForRefresh_recursive();
	}
	else
	{
		firstLineEdit->setText("No entity!");
	}

	ccHObject* o2 = getSecondEntity();
	if (o2)
	{
		secondLineEdit->setText(o2->getName());
		o2->setEnabled(true);
		o2->setVisible(true);
		o2->setTempColor(ccColor::yellow);
		o2->prepareDisplayForRefresh_recursive();
	}
	else
	{
		secondLineEdit->setText("No entity!");
	}

	if (m_app)
		m_app->refreshAll();
}

void ccOrderChoiceDlg::swap()
{
	m_originalOrder = !m_originalOrder;
	setColorsAndLabels();
}
