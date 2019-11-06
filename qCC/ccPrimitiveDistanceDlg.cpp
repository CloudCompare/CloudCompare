//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccPrimitiveDistanceDlg.h"

//Qt
#include <QHeaderView>
#include <QMessageBox>


//System
#include <assert.h>

ccPrimitiveDistanceDlg::ccPrimitiveDistanceDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::primitiveDistanceDlg()
{
	setupUi(this);

	

	signedDistCheckBox->setChecked(true);
	flipNormalsCheckBox->setEnabled(true);
	
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancelAndExit()));
	connect(okButton, SIGNAL(clicked()), this, SLOT(applyAndExit()));
	connect(signedDistCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleSigned(bool)));
	show();
	QCoreApplication::processEvents();
}

ccPrimitiveDistanceDlg::~ccPrimitiveDistanceDlg()
{
}

void ccPrimitiveDistanceDlg::applyAndExit()
{
	accept();
}

void ccPrimitiveDistanceDlg::cancelAndExit()
{
	reject();
}

void ccPrimitiveDistanceDlg::toggleSigned(bool state)
{
	flipNormalsCheckBox->setEnabled(state);
}

