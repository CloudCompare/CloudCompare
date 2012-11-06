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

#ifndef CC_ASK_ONE_INT_VALUE_DLG_HEADER
#define CC_ASK_ONE_INT_VALUE_DLG_HEADER

#include <ui_askOneIntValueDlg.h>

class ccAskOneIntValueDlg : public QDialog, public Ui::AskOneIntValueDialog
{
public:
    ccAskOneIntValueDlg(const char* valueName,
                            int minVal,
                                int maxVal,
                                    int defaultVal,
                                        const char* windowTitle=0,
                                            QWidget* parent=0);

    int getValue();
};

#endif
