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
//$Rev:: 1691                                                              $
//$LastChangedDate:: 2010-10-22 16:52:55 +0200 (ven., 22 oct. 2010)        $
//**************************************************************************
//

#ifndef CC_STATISTICAL_TEST_DLG_HEADER
#define CC_STATISTICAL_TEST_DLG_HEADER

#include <ui_statisticalTestDlg.h>

class ccStatisticalTestDlg : public QDialog, public Ui::StatisticalTestDialog
{
public:
    ccStatisticalTestDlg(const char* param1Label,
                            const char* param2Label,
                                const char* param3Label=0,
                                    const char* windowTitle=0,
                                        QWidget* parent=0);

    double getParam1();
    double getParam2();
    double getParam3();
    int getNeighborsNumber();
    double getProba();
};

#endif
