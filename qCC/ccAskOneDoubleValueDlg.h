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

#ifndef CC_ASK_ONE_DOUBLE_VALUE_DLG_HEADER
#define CC_ASK_ONE_DOUBLE_VALUE_DLG_HEADER

#include <ui_askOneDoubleValueDlg.h>

class ccAskOneDoubleValueDlg : public QDialog, public Ui::AskOneDoubleValueDialog
{
public:
    ccAskOneDoubleValueDlg(const char* valueName,
                            double minVal,
                                double maxVal,
                                    double defaultVal,
                                        int precision=6,
                                            const char* windowTitle=0,
                                                QWidget* parent=0);
};

#endif
