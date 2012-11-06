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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2011                                                              $
//$LastChangedDate:: 2012-02-01 00:15:21 +0100 (mer., 01 févr. 2012)      $
//**************************************************************************
//

#include "ccRenderToFileDlg.h"
#include "ccConsole.h"

#include <QFileDialog>
#include <QDoubleSpinBox>
#include <QImageWriter>
#include <QList>
#include <QSettings>

//we keep track of the zoom for the session only!
static double s_zoom = 1.0;

ccRenderToFileDlg::ccRenderToFileDlg(unsigned baseWidth, unsigned baseHeight, QWidget* parent/*=0*/)
	: QDialog(parent)
	, w(baseWidth)
	, h(baseHeight)
{
    setupUi(this);

    //we grab the list of supported image file formats (writing)
    QList<QByteArray> list = QImageWriter::supportedImageFormats();
    if (list.size()<1)
    {
        ccConsole::Error("No supported image format on this platform?!");
        reject();
        return;
    }

    //we convert this list into a proper "filters" string
    QString firstExtension(list[0].data());
	QString firstFilter;
    for (int i=0;i<list.size();++i)
	{
        filters.append(QString("%1 image (*.%2)\n").arg(QString(list[i].data()).toUpper()).arg(list[i].data()));
		if (i==0)
			firstFilter = filters;
	}

	QSettings settings;
    settings.beginGroup("RenderToFile");
    selectedFilter				= settings.value("selectedFilter",firstFilter).toString();
    QString currentPath			= settings.value("currentPath",QApplication::applicationDirPath()).toString();
    QString selectedExtension	= settings.value("selectedExtension",firstExtension).toString();
	QString baseFilename		= settings.value("baseFilename","capture").toString();
    bool dontScale				= settings.value("dontScaleFeatures",dontScalePoints()).toBool();
    settings.endGroup();

	dontScaleFeaturesCheckBox->setChecked(dontScale);
    filenameLineEdit->setText(currentPath+QString("/")+baseFilename+QString(".")+selectedExtension);

	zoomDoubleSpinBox->setValue(s_zoom);

    connect(chooseFileButton,           SIGNAL(clicked()),              this, SLOT(chooseFile()));
    connect(zoomDoubleSpinBox,          SIGNAL(valueChanged(double)),   this, SLOT(updateInfo()));
    connect(buttonBox,                  SIGNAL(accepted()),             this, SLOT(saveSettings()));

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
    if (selectedFileName.size()<1)
        return;

	filenameLineEdit->setText(selectedFileName);
}

double ccRenderToFileDlg::getZoom() const
{
    return zoomDoubleSpinBox->value();
}

QString ccRenderToFileDlg::getFilename() const
{
    return filenameLineEdit->text();
}

bool ccRenderToFileDlg::dontScalePoints() const
{
	return dontScaleFeaturesCheckBox->isChecked();
}

void ccRenderToFileDlg::updateInfo()
{
    s_zoom = getZoom();

    unsigned w2 = (unsigned)(double(w)*s_zoom);
    unsigned h2 = (unsigned)(double(h)*s_zoom);

    finalSizeLabel->setText(QString("(%1 x %2)").arg(w2).arg(h2));
}
