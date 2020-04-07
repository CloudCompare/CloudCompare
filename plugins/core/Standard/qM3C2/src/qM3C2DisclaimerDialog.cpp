//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
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
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#include "qM3C2DisclaimerDialog.h"
#include "ui_disclaimerDlg.h"

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QMainWindow>

bool DisclaimerDialog::s_disclaimerAccepted = false;


DisclaimerDialog::DisclaimerDialog(QWidget *parent)
    : QDialog(parent)
    , m_ui( new Ui::DisclaimerDialog )
{
    m_ui->setupUi( this );
}

DisclaimerDialog::~DisclaimerDialog()
{
    delete m_ui;
}

bool DisclaimerDialog::show(ccMainAppInterface *app)
{
	if ( !s_disclaimerAccepted )
	{
		//if the user "cancels" it, then he refuses the disclaimer
		s_disclaimerAccepted = DisclaimerDialog(app ? app->getMainWindow() : 0).exec();
	}

	return s_disclaimerAccepted;
}
