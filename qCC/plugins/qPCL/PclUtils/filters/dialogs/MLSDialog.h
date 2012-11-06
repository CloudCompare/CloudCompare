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
#ifndef MLSDIALOG_H
#define MLSDIALOG_H

#include <ui_MLSDialog.h>

//Qt
#include <QDialog>

//PCL
#include <pcl/surface/mls.h>

class MLSDialog : public QDialog, public Ui::MLSDialog
{
    Q_OBJECT


public:
   MLSDialog(QWidget *parent = 0);

private:


    void updateCombo();
    void deactivateAllMethods();


private slots:
    void activateMenu(QString name);
    void toggleMethods(bool status);
    void updateSquaredGaussian(double radius);

private:



};

#endif // MLSDIALOG_H
