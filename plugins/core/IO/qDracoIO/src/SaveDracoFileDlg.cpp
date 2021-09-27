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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include <SaveDracoFileDlg.h>

//Qt
#include <QDialogButtonBox>
#include <QPushButton>
#include <QSettings>

static const int DefaultCoordsQuant = 11;
static const int DefaultNormQuant = 8;
static const int DefaultSFQuant = 8;

SaveDracoFileDlg::SaveDracoFileDlg(QWidget* parent/*=nullptr*/)
	: QDialog(parent)
{
	setupUi(this);

	initFromPersistentSettings();

	connect(buttonBox, &QDialogButtonBox::accepted, this, &SaveDracoFileDlg::saveToPersistentSettings);
	connect(buttonBox->button(QDialogButtonBox::Reset), &QPushButton::clicked, this, &SaveDracoFileDlg::reset);
}

void SaveDracoFileDlg::initFromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("DracoSaveDialog");

	//read parameters
	int coordQuantization = settings.value("coordQuantization", DefaultCoordsQuant).toInt();
	int normQuantization = settings.value("normalQuantization", DefaultNormQuant).toInt();
	int sfQuantization = settings.value("sfQuantization", DefaultSFQuant).toInt();

	//apply parameters
	coordsQuantSpinBox->setValue(coordQuantization);
	normQuantSpinBox->setValue(normQuantization);
	sfQuantSpinBox->setValue(sfQuantization);

	settings.endGroup();
}

void SaveDracoFileDlg::saveToPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("DracoSaveDialog");

	//write parameters
	settings.setValue("coordQuantization", coordsQuantSpinBox->value());
	settings.setValue("normalQuantization", normQuantSpinBox->value());
	settings.setValue("sfQuantization", sfQuantSpinBox->value());

	settings.endGroup();

	accept();
}

void SaveDracoFileDlg::reset()
{
	coordsQuantSpinBox->setValue(DefaultCoordsQuant);
	normQuantSpinBox->setValue(DefaultNormQuant);
	sfQuantSpinBox->setValue(DefaultSFQuant);
}
