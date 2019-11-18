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

#include "ccRenderToFileDlg.h"

//qCC_db
#include <ccLog.h>

//Qt
#include <QFileDialog>
#include <QDoubleSpinBox>
#include <QImageWriter>
#include <QList>
#include <QSettings>
#include <QStandardPaths>

//we keep track of the zoom for the session only!
static double s_renderZoom = 1.0;

ccRenderToFileDlg::ccRenderToFileDlg(unsigned baseWidth, unsigned baseHeight, QWidget* parent/*=0*/)
	: QDialog(parent)
	, w(baseWidth)
	, h(baseHeight)
{
	setupUi(this);

	//we grab the list of supported image file formats (output)
	QList<QByteArray> list = QImageWriter::supportedImageFormats();
	if (list.size() < 1)
	{
		ccLog::Error("No supported image format on this platform?!");
		reject();
		return;
	}

	//we convert this list into a proper "filters" string
	QString firstExtension(list[0].data());
	QString firstFilter;
	for (int i=0; i<list.size(); ++i)
	{
		filters.append(QString("%1 image (*.%2)\n").arg(QString(list[i].data()).toUpper()).arg(list[i].data()));
		if (i == 0 || QString(list[i].data()) == "jpg")
		{
			firstFilter = filters;
		}
	}

	QSettings settings;
	settings.beginGroup("RenderToFile");
	selectedFilter				= settings.value("selectedFilter", firstFilter).toString();
	QString currentPath         = settings.value("currentPath", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
	QString selectedExtension	= settings.value("selectedExtension", firstExtension).toString();
	QString baseFilename		= settings.value("baseFilename", "capture").toString();
	bool dontScale				= settings.value("dontScaleFeatures", dontScalePoints()).toBool();
	bool doRenderOverlayItems	= settings.value("renderOverlayItems", renderOverlayItems()).toBool();
	settings.endGroup();

	dontScaleFeaturesCheckBox->setChecked(dontScale);
	renderOverlayItemsCheckBox->setChecked(doRenderOverlayItems);
	filenameLineEdit->setText(currentPath + QString("/") + baseFilename + QString(".") + selectedExtension);

	zoomDoubleSpinBox->setValue(s_renderZoom);

	connect(chooseFileButton,	&QToolButton::clicked,			this, &ccRenderToFileDlg::chooseFile);
	connect(buttonBox,	 		&QDialogButtonBox::accepted,	this, &ccRenderToFileDlg::saveSettings);
	connect(zoomDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccRenderToFileDlg::updateInfo);

	updateInfo();
}

void ccRenderToFileDlg::saveSettings()
{
	//we update current file path
	QFileInfo fi(filenameLineEdit->text());
	QString currentPath = fi.absolutePath();
	QString selectedExtension = fi.suffix();
	QString baseFilename = fi.completeBaseName();

	QSettings settings;
	settings.beginGroup("RenderToFile");
	settings.setValue("currentPath",currentPath);
	settings.setValue("selectedExtension",selectedExtension);
	settings.setValue("selectedFilter",selectedFilter);
	settings.setValue("baseFilename",baseFilename);
	settings.setValue("dontScaleFeatures",dontScalePoints());
	settings.setValue("renderOverlayItems",renderOverlayItems());
	settings.endGroup();
}

void ccRenderToFileDlg::chooseFile()
{
	QString selectedFileName = QFileDialog::getSaveFileName(this,
															tr("Save Image"),
															filenameLineEdit->text(),
															filters,
															&selectedFilter);

	//if operation is canceled, selectedFileName is empty
	if (selectedFileName.size() < 1)
		return;

	filenameLineEdit->setText(selectedFileName);
}

float ccRenderToFileDlg::getZoom() const
{
	return static_cast<float>(zoomDoubleSpinBox->value());
}

QString ccRenderToFileDlg::getFilename() const
{
	return filenameLineEdit->text();
}

bool ccRenderToFileDlg::dontScalePoints() const
{
	return dontScaleFeaturesCheckBox->isChecked();
}

bool ccRenderToFileDlg::renderOverlayItems() const
{
	return renderOverlayItemsCheckBox->isChecked();
}

void ccRenderToFileDlg::updateInfo()
{
	s_renderZoom = getZoom();

	unsigned w2 = static_cast<unsigned>(w*s_renderZoom);
	unsigned h2 = static_cast<unsigned>(h*s_renderZoom);

	finalSizeLabel->setText(QString("(%1 x %2)").arg(w2).arg(h2));
}
