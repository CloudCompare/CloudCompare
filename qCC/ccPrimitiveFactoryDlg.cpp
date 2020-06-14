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

#include "ccPrimitiveFactoryDlg.h"

//Qt
#include <QClipboard>

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

	connect(createPushButton, &QAbstractButton::clicked, this, &ccPrimitiveFactoryDlg::createPrimitive);
	connect(closePushButton, &QAbstractButton::clicked, this, &QDialog::accept);
	connect(spherePosFromClipboardButton, &QPushButton::clicked, this, &ccPrimitiveFactoryDlg::setSpherePositionFromClipboard);
	connect(spherePosToOriginButton, &QPushButton::clicked, this, &ccPrimitiveFactoryDlg::setSpherePositionToOrigin);
}

void ccPrimitiveFactoryDlg::createPrimitive()
{
	if (!m_win)
		return;

	ccGenericPrimitive* primitive = nullptr;
	switch(tabWidget->currentIndex())
	{
		//Plane
		case 0:
			{
				primitive = new ccPlane(static_cast<PointCoordinateType>(planeWidthDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(planeHeightDoubleSpinBox->value()));
			}
			break;
		//Box
		case 1:
			{
				CCVector3 dims(	static_cast<PointCoordinateType>(boxDxDoubleSpinBox->value()),
								static_cast<PointCoordinateType>(boxDyDoubleSpinBox->value()),
								static_cast<PointCoordinateType>(boxDzDoubleSpinBox->value()));
				primitive = new ccBox(dims);
			}
			break;
		//Sphere
		case 2:
			{
				ccGLMatrix transMat;
				transMat.setTranslation(CCVector3f(spherePosXDoubleSpinBox->value(), spherePosYDoubleSpinBox->value(), spherePosZDoubleSpinBox->value()));
				primitive = new ccSphere(static_cast<PointCoordinateType>(sphereRadiusDoubleSpinBox->value()), &transMat);
			}
			break;
		//Cylinder
		case 3:
			{
				primitive = new ccCylinder( static_cast<PointCoordinateType>(cylRadiusDoubleSpinBox->value()),
											static_cast<PointCoordinateType>(cylHeightDoubleSpinBox->value()));
			}
			break;
		//Cone
		case 4:
			{
				primitive = new ccCone( static_cast<PointCoordinateType>(coneBottomRadiusDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(coneTopRadiusDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(coneHeightDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(snoutGroupBox->isChecked() ? coneXOffsetDoubleSpinBox->value() : 0),
										static_cast<PointCoordinateType>(snoutGroupBox->isChecked() ? coneYOffsetDoubleSpinBox->value() : 0));
			}
			break;
		//Torus
		case 5:
			{
				primitive = new ccTorus(static_cast<PointCoordinateType>(torusInsideRadiusDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(torusOutsideRadiusDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(torusAngleDoubleSpinBox->value()*CC_DEG_TO_RAD),
										torusRectGroupBox->isChecked(),
										static_cast<PointCoordinateType>(torusRectGroupBox->isChecked() ? torusRectSectionHeightDoubleSpinBox->value() : 0));
			}
			break;
		//Dish
		case 6:
			{
				primitive = new ccDish( static_cast<PointCoordinateType>(dishRadiusDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(dishHeightDoubleSpinBox->value()),
										static_cast<PointCoordinateType>(dishEllipsoidGroupBox->isChecked() ? dishRadius2DoubleSpinBox->value() : 0));
			}
			break;
	}

	if (primitive)
	{
		m_win->addToDB(primitive, true, true, true);
	}
}

void ccPrimitiveFactoryDlg::setSpherePositionFromClipboard()
{
	QClipboard *clipboard = QApplication::clipboard();
	if (clipboard != nullptr)
	{
		QStringList valuesStr = clipboard->text().simplified().split(QChar(' '), QString::SkipEmptyParts);
		if (valuesStr.size() == 3)
		{
			CCVector3d vec;
			bool success;
			for (unsigned i = 0; i < 3; ++i)
			{
				vec[i] = valuesStr[i].toDouble(&success);
				if (!success)
					break;
			}
			if (success)
			{
				spherePosXDoubleSpinBox->setValue(vec.x);
				spherePosYDoubleSpinBox->setValue(vec.y);
				spherePosZDoubleSpinBox->setValue(vec.z);
			}
		}
	}
}

void ccPrimitiveFactoryDlg::setSpherePositionToOrigin()
{
	spherePosXDoubleSpinBox->setValue(0);
	spherePosYDoubleSpinBox->setValue(0);
	spherePosZDoubleSpinBox->setValue(0);
}