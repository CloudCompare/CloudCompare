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
//$Rev:: 1933                                                              $
//$LastChangedDate:: 2011-11-20 23:42:07 +0100 (dim., 20 nov. 2011)        $
//**************************************************************************
//

#include "ccScalarFieldArithmeticDlg.h"

//Qt
#include <QPushButton>

//qCC_db
#include <ccPointCloud.h>

#include <assert.h>
#ifdef _MSC_VER
#include <windows.h>
#endif

ccScalarFieldArithmeticDlg::ccScalarFieldArithmeticDlg(ccPointCloud* cloud,
                                                        QWidget* parent/*=0*/)
    : QDialog(parent), Ui::SFComparisonDlg()
{
    assert(cloud);

    setupUi(this);
    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

    unsigned sfCount = cloud->getNumberOfScalarFields();
    if (sfCount<1)
    {
        sf1ComboBox->setEnabled(false);
        sf2ComboBox->setEnabled(false);
        buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
    }
    else
    {
        QStringList sfLabels;
        for (unsigned i=0;i<sfCount;++i)
            sfLabels << QString(cloud->getScalarFieldName(i));

        sf1ComboBox->addItems(sfLabels);
        sf1ComboBox->setCurrentIndex(0);
        sf2ComboBox->addItems(sfLabels);
        sf2ComboBox->setCurrentIndex(std::min((unsigned)1,sfCount-1));
    }
}

int ccScalarFieldArithmeticDlg::getSF1Index()
{
    return sf1ComboBox->currentIndex();
}

int ccScalarFieldArithmeticDlg::getSF2Index()
{
    return sf2ComboBox->currentIndex();
}

ccScalarFieldArithmeticDlg::Operation ccScalarFieldArithmeticDlg::getOperation()
{
    switch (operationComboBox->currentIndex())
    {
        case 0:
            return PLUS;
            break;
        case 1:
            return MINUS;
            break;
        case 2:
            return MULTIPLY;
            break;
        case 3:
            return DIVIDE;
            break;
    }

    assert(false);
    return PLUS;
}
