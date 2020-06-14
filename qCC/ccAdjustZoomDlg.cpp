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

#include "ccAdjustZoomDlg.h"

//local
#include "ccGLWindow.h"

ccAdjustZoomDlg::ccAdjustZoomDlg(ccGLWindow* win, QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::AdjustZoomDialog()
	, m_basePixelSize(1.0)
{
	setupUi(this);

	if (win)
	{
		windowLabel->setText(QString("%1 [%2 x %3]").arg(win->windowTitle()).arg(win->glWidth()).arg(win->glHeight()));
		
		const ccViewportParameters& params = win->getViewportParameters();
		assert(!params.perspectiveView);

		m_basePixelSize = params.pixelSize;
		zoomDoubleSpinBox->setValue(params.zoom);
		pixelCountSpinBox->setValue(1);
		pixelSizeDoubleSpinBox->setValue(params.pixelSize / params.zoom);
	}
	else
	{
		windowLabel->setText("Error");
	}

	connect(zoomDoubleSpinBox,		static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this, &ccAdjustZoomDlg::onZoomChanged);
	connect(pixelSizeDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),	this, &ccAdjustZoomDlg::onPixelSizeChanged);
	connect(pixelCountSpinBox,		static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),					this, &ccAdjustZoomDlg::onPixelCountChanged);
}

double ccAdjustZoomDlg::getZoom() const
{
	return zoomDoubleSpinBox->value();
}

void ccAdjustZoomDlg::onZoomChanged(double zoom)
{
	pixelSizeDoubleSpinBox->blockSignals(true);
	pixelSizeDoubleSpinBox->setValue( m_basePixelSize * static_cast<double>(pixelCountSpinBox->value()) / zoom );
	pixelSizeDoubleSpinBox->blockSignals(false);
}

void ccAdjustZoomDlg::onPixelSizeChanged(double ps)
{
	zoomDoubleSpinBox->blockSignals(true);
	zoomDoubleSpinBox->setValue( m_basePixelSize * static_cast<double>(pixelCountSpinBox->value()) / ps );
	zoomDoubleSpinBox->blockSignals(false);
}

void ccAdjustZoomDlg::onPixelCountChanged(int count)
{
	pixelSizeDoubleSpinBox->blockSignals(true);
	pixelSizeDoubleSpinBox->setValue( m_basePixelSize * static_cast<double>(count) / zoomDoubleSpinBox->value() );
	pixelSizeDoubleSpinBox->blockSignals(false);
}
