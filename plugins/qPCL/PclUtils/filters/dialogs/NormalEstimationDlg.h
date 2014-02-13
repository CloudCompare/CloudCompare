//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#               COPYRIGHT: Luca Penasa                                   #
//#                                                                        #
//##########################################################################
//
#ifndef Q_PCL_PLUGIN_NORMALESTIMATIONDIALOG_HEADER
#define Q_PCL_PLUGIN_NORMALESTIMATIONDIALOG_HEADER

#include <ui_NormalEstimationDlg.h>

class NormalEstimationDialog : public QDialog, public Ui::NormalEstimationDialog
{
    Q_OBJECT
public:
    NormalEstimationDialog(QWidget* parent=0);

public:

};

#endif // Q_PCL_PLUGIN_NORMALESTIMATIONDIALOG_HEADER
