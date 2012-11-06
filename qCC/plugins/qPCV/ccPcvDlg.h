//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCV                        #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2038                                                              $
//$LastChangedDate:: 2012-03-25 13:50:08 +0200 (dim., 25 mars 2012)        $
//**************************************************************************
//

#ifndef CC_PCV_DLG_HEADER
#define CC_PCV_DLG_HEADER

#include <ui_pcvDlg.h>

class ccPcvDlg : public QDialog, public Ui::PCVDialog
{
public:
    ccPcvDlg(QWidget* parent=0);
};

#endif
