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

#include "ccTwoColorsDlg.h"

//Local
#include "ccDisplayOptionsDlg.h"

//Qt
#include <QColorDialog>

QColor ccTwoColorsDlg::s_firstColor(Qt::black);
QColor ccTwoColorsDlg::s_secondColor(Qt::white);

ccTwoColorsDlg::ccTwoColorsDlg(QWidget* parent) : QDialog(parent), Ui::TwoColorChoiceDialog()
{
    setupUi(this);

    directionComboBox->addItem("X");
    directionComboBox->addItem("Y");
    directionComboBox->addItem("Z");
    directionComboBox->setCurrentIndex(2);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

    connect(firstColorButton, SIGNAL(clicked()), this, SLOT(changeFirstColor()));
    connect(secondColorButton, SIGNAL(clicked()), this, SLOT(changeSecondColor()));

    ccDisplayOptionsDlg::SetButtonColor(secondColorButton,s_secondColor);
    ccDisplayOptionsDlg::SetButtonColor(firstColorButton,s_firstColor);
}

void ccTwoColorsDlg::changeFirstColor()
{
    QColor newCol = QColorDialog::getColor(s_firstColor, this);
    if (newCol.isValid())
    {
        s_firstColor = newCol;
        ccDisplayOptionsDlg::SetButtonColor(firstColorButton,s_firstColor);
    }
}

void ccTwoColorsDlg::changeSecondColor()
{
    QColor newCol = QColorDialog::getColor(s_secondColor, this);
    if (newCol.isValid())
    {
        s_secondColor = newCol;
        ccDisplayOptionsDlg::SetButtonColor(secondColorButton,s_secondColor);
    }
}
