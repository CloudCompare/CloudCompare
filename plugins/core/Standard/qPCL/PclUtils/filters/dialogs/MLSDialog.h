//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#ifndef MLSDIALOG_H
#define MLSDIALOG_H

#include <ui_MLSDialog.h>

//Qt
#include <QDialog>

class MLSDialog : public QDialog, public Ui::MLSDialog
{
	Q_OBJECT

public:
	explicit MLSDialog(QWidget *parent = 0);

protected slots:
	void activateMenu(QString name);
	void toggleMethods(bool status);
	void updateSquaredGaussian(double radius);

protected:
	void updateCombo();
	void deactivateAllMethods();

};

#endif // MLSDIALOG_H
