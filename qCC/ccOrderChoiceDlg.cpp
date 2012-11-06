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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#include "ccOrderChoiceDlg.h"

#include <ccHObject.h>
#include "ccDisplayOptionsDlg.h"
#include "mainwindow.h"

ccOrderChoiceDlg::ccOrderChoiceDlg(ccHObject* firstEntity, const char* firstRole, ccHObject* secondEntity, const char* secondRole, QWidget* parent/* = 0*/)
    : QDialog(parent), Ui::RoleChoiceDialog()
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

    firstEnt=firstEntity;
    secondEnt=secondEntity;
    originalOrder=true;

    setColorsAndLabels();
}

ccOrderChoiceDlg::~ccOrderChoiceDlg()
{
    firstEnt->enableTempColor(false);
    secondEnt->enableTempColor(false);
    firstEnt->prepareDisplayForRefresh_recursive();
    secondEnt->prepareDisplayForRefresh_recursive();
    MainWindow::RefreshAllGLWindow();
}

ccHObject* ccOrderChoiceDlg::getFirstEntity()
{
    return originalOrder ? firstEnt : secondEnt;
}

ccHObject* ccOrderChoiceDlg::getSecondEntity()
{
    return originalOrder ? secondEnt : firstEnt;
}

void ccOrderChoiceDlg::setColorsAndLabels()
{
    ccHObject* o1 = getFirstEntity();
    ccHObject* o2 = getSecondEntity();

    firstLineEdit->setText(o1->getName());
    secondLineEdit->setText(o2->getName());
	o1->setEnabled(true);
    o1->setVisible(true);
	o2->setEnabled(true);
    o2->setVisible(true);
    o1->setTempColor(ccColor::red);
    o2->setTempColor(ccColor::yellow);
    o1->prepareDisplayForRefresh_recursive();
    o2->prepareDisplayForRefresh_recursive();
    MainWindow::RefreshAllGLWindow();
}

void ccOrderChoiceDlg::swap()
{
    originalOrder = !originalOrder;
    setColorsAndLabels();
}
