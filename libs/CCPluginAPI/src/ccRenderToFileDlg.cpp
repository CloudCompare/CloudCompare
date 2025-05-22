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
#include "ui_renderToFileDialog.h"

//Local
#include <ccInfoDlg.h>

//qCC_glWindow
#include <ccGLWindowInterface.h>

//qCC_db
#include <ccLog.h>

//Qt
#include <QFileDialog>
#include <QImageWriter>
#include <QSettings>
#include <QStandardPaths>

namespace
{
	//we keep track of the zoom for the session only!
	double s_renderZoom = 1.0;
}

ccRenderToFileDlg::ccRenderToFileDlg(ccGLWindowInterface* win, QWidget* parent/*=nullptr*/)
	: QDialog(parent)
	, m_associatedWindow(win)
	, m_ui( new Ui::RenderToFileDialog )
{
	m_ui->setupUi(this);

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
	for (int i = 0; i < list.size(); ++i)
	{
		m_filters.append(QString("%1 image (*.%2)\n").arg(QString(list[i].data()).toUpper()).arg(list[i].data()));
		if (i == 0 || QString(list[i].data()) == "jpg")
		{
			firstFilter = m_filters;
		}
	}

	QSettings settings;
	settings.beginGroup("RenderToFile");
	m_selectedFilter			= settings.value("selectedFilter", firstFilter).toString();
	QString currentPath			= settings.value("currentPath", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
	QString selectedExtension	= settings.value("selectedExtension", firstExtension).toString();
	QString baseFilename		= settings.value("baseFilename", "capture").toString();
	bool dontScale				= settings.value("dontScaleFeatures", dontScalePoints()).toBool();
	bool doRenderOverlayItems	= settings.value("renderOverlayItems", renderOverlayItems()).toBool();
	settings.endGroup();

	m_ui->dontScaleFeaturesCheckBox->setChecked(dontScale);
	m_ui->renderOverlayItemsCheckBox->setChecked(doRenderOverlayItems);
	m_ui->filenameLineEdit->setText(currentPath + QString("/") + baseFilename + QString(".") + selectedExtension);

	m_ui->zoomDoubleSpinBox->setValue(s_renderZoom);

	connect(m_ui->chooseFileButton,		&QToolButton::clicked,			this, &ccRenderToFileDlg::chooseFile);
	connect(m_ui->infoToolButton,		&QToolButton::clicked,			this, &ccRenderToFileDlg::showOutputInfo);
	connect(m_ui->buttonBox,	 		&QDialogButtonBox::accepted,	this, &ccRenderToFileDlg::saveSettings);
	connect(m_ui->zoomDoubleSpinBox,	qOverload<double>(&QDoubleSpinBox::valueChanged), this, &ccRenderToFileDlg::updateInfo);

	updateInfo();
}

ccRenderToFileDlg::~ccRenderToFileDlg()
{
	delete m_ui;
}

void ccRenderToFileDlg::hideOptions()
{
	m_ui->dontScaleFeaturesCheckBox->setChecked( false );
	m_ui->dontScaleFeaturesCheckBox->setVisible( false );
	m_ui->renderOverlayItemsCheckBox->setChecked( false );
	m_ui->renderOverlayItemsCheckBox->setVisible( false );
}

void ccRenderToFileDlg::saveSettings()
{
	//we update current file path
	QFileInfo fi(m_ui->filenameLineEdit->text());
	QString currentPath = fi.absolutePath();
	QString selectedExtension = fi.suffix();
	QString baseFilename = fi.completeBaseName();

	QSettings settings;
	settings.beginGroup("RenderToFile");
	settings.setValue("currentPath", currentPath);
	settings.setValue("selectedExtension", selectedExtension);
	settings.setValue("selectedFilter", m_selectedFilter);
	settings.setValue("baseFilename", baseFilename);
	settings.setValue("dontScaleFeatures", dontScalePoints());
	settings.setValue("renderOverlayItems", renderOverlayItems());
	settings.endGroup();
}

void ccRenderToFileDlg::chooseFile()
{
	QString selectedFileName = QFileDialog::getSaveFileName(this,
															tr("Save Image"),
															m_ui->filenameLineEdit->text(),
															m_filters,
															&m_selectedFilter);

	//if operation is canceled, selectedFileName is empty
	if (selectedFileName.size() < 1)
		return;

	m_ui->filenameLineEdit->setText(selectedFileName);
}

float ccRenderToFileDlg::getZoom() const
{
	return static_cast<float>(m_ui->zoomDoubleSpinBox->value());
}

QString ccRenderToFileDlg::getFilename() const
{
	return m_ui->filenameLineEdit->text();
}

bool ccRenderToFileDlg::dontScalePoints() const
{
	return m_ui->dontScaleFeaturesCheckBox->isChecked();
}

bool ccRenderToFileDlg::renderOverlayItems() const
{
	return m_ui->renderOverlayItemsCheckBox->isChecked();
}

void ccRenderToFileDlg::updateInfo()
{
	s_renderZoom = getZoom();

	unsigned w = 0;
	unsigned h = 0;
	if (m_associatedWindow)
	{
		w = m_associatedWindow->glWidth();
		h = m_associatedWindow->glHeight();
	}
	unsigned w2 = static_cast<unsigned>(w*s_renderZoom);
	unsigned h2 = static_cast<unsigned>(h*s_renderZoom);

	m_ui->finalSizeLabel->setText(QString("(%1 x %2)").arg(w2).arg(h2));
}

void ccRenderToFileDlg::showOutputInfo()
{
	if (!m_associatedWindow)
	{
		assert(false);
		return;
	}

	s_renderZoom = getZoom();

	QStringList info = m_associatedWindow->getWindowInfo(s_renderZoom);

	ccInfoDlg infoDlg(this);

	infoDlg.showText(info.join('\n'));

	infoDlg.exec();
}
