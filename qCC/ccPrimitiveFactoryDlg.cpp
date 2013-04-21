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

#include "ccPrimitiveFactoryDlg.h"

//qCC
#include <mainwindow.h>

//qCC_db
#include <ccGenericPrimitive.h>
#include <ccPlane.h>
#include <ccBox.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <ccTorus.h>
#include <ccDish.h>

//system
#include <assert.h>

ccPrimitiveFactoryDlg::ccPrimitiveFactoryDlg(MainWindow* win)
	: QDialog(win)
	, Ui::PrimitiveFactoryDlg()
	, m_win(win)
{
	assert(m_win);

	setupUi(this);	

	connect(createPushButton, SIGNAL(clicked()), this, SLOT(createPrimitive()));
	connect(closePushButton, SIGNAL(clicked()), this, SLOT(accept()));
}

void ccPrimitiveFactoryDlg::createPrimitive()
{
	if (!m_win)
		return;

	ccGenericPrimitive* primitive = 0;
	switch(tabWidget->currentIndex())
	{
		//Plane
		case 0:
			{
				primitive = new ccPlane(planeWidthDoubleSpinBox->value(),
										planeHeightDoubleSpinBox->value());
			}
			break;
		//Box
		case 1:
			{
				CCVector3 dims(boxDxDoubleSpinBox->value(),
								boxDyDoubleSpinBox->value(),
								boxDzDoubleSpinBox->value());
				primitive = new ccBox(dims);
			}
			break;
		//Sphere
		case 2:
			{
				primitive = new ccSphere(sphereRadiusDoubleSpinBox->value());
			}
			break;
		//Cylinder
		case 3:
			{
				primitive = new ccCylinder( cylRadiusDoubleSpinBox->value(),
											cylHeightDoubleSpinBox->value());
			}
			break;
		//Cone
		case 4:
			{
				primitive = new ccCone( coneBottomRadiusDoubleSpinBox->value(),
										coneTopRadiusDoubleSpinBox->value(),
										coneHeightDoubleSpinBox->value(),
										snoutGroupBox->isChecked() ? coneXOffsetDoubleSpinBox->value() : 0,
										snoutGroupBox->isChecked() ? coneYOffsetDoubleSpinBox->value() : 0);
			}
			break;
		//Torus
		case 5:
			{
				primitive = new ccTorus( torusInsideRadiusDoubleSpinBox->value(),
										torusOutsideRadiusDoubleSpinBox->value(),
										torusAngleDoubleSpinBox->value()*CC_DEG_TO_RAD,
										torusRectGroupBox->isChecked(),
										torusRectGroupBox->isChecked() ? torusRectSectionHeightDoubleSpinBox->value() : 0);
			}
			break;
		//Dish
		case 6:
			{
				primitive = new ccDish( dishRadiusDoubleSpinBox->value(),
										dishHeightDoubleSpinBox->value(),
										dishEllipsoidGroupBox->isChecked() ? dishRadius2DoubleSpinBox->value() : 0);
			}
			break;
	}

	if (primitive)
		m_win->addToDB(primitive);
}
