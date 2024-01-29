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
#include "ccGLWindowInterface.h"

ccAdjustZoomDlg::ccAdjustZoomDlg(ccGLWindowInterface* win, QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::AdjustZoomDialog()
	, m_windowWidth_pix(0)
	, m_distanceToWidthRatio(0.0)
{
	setupUi(this);

	if (win)
	{
		windowLabel->setText(QString("%1 [%2 x %3]").arg(win->getWindowTitle()).arg(win->glWidth()).arg(win->glHeight()));
		
		const ccViewportParameters& params = win->getViewportParameters();
		assert(!params.perspectiveView);

		m_windowWidth_pix = win->glWidth();
		m_distanceToWidthRatio = params.computeDistanceToWidthRatio();
		double focalDist = params.getFocalDistance();

		if (m_windowWidth_pix < 1)
		{
			assert(false);
			m_windowWidth_pix = 1;
		}
		if (m_distanceToWidthRatio <= 0.0)
		{
			assert(false);
			m_distanceToWidthRatio = 1.0;
		}

		focalDoubleSpinBox->setValue(focalDist);
		pixelCountSpinBox->setValue(1);
		pixelSizeDoubleSpinBox->setValue(focalDist * m_distanceToWidthRatio / m_windowWidth_pix);
	}
	else
	{
		windowLabel->setText("Error");
	}

	connect(focalDoubleSpinBox,		qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccAdjustZoomDlg::onFocalChanged);
	connect(pixelSizeDoubleSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),	this, &ccAdjustZoomDlg::onPixelSizeChanged);
	connect(pixelCountSpinBox,		qOverload<int>(&QSpinBox::valueChanged),			this, &ccAdjustZoomDlg::onPixelCountChanged);
}

double ccAdjustZoomDlg::getFocalDistance() const
{
	return focalDoubleSpinBox->value();
}

void ccAdjustZoomDlg::onFocalChanged(double focalDist)
{
	assert(pixelCountSpinBox->value() > 0);

	pixelSizeDoubleSpinBox->blockSignals(true);
	double pixelSizeNPixels = (focalDist * m_distanceToWidthRatio * pixelCountSpinBox->value()) / m_windowWidth_pix;
	pixelSizeDoubleSpinBox->setValue(pixelSizeNPixels);
	pixelSizeDoubleSpinBox->blockSignals(false);
}

void ccAdjustZoomDlg::onPixelSizeChanged(double pixelSizeNPixels)
{
	assert(pixelCountSpinBox->value() > 0);

	focalDoubleSpinBox->blockSignals(true);
	double focalDist = (pixelSizeNPixels * m_windowWidth_pix) / (pixelCountSpinBox->value() * m_distanceToWidthRatio);
	focalDoubleSpinBox->setValue(focalDist);
	focalDoubleSpinBox->blockSignals(false);
}

void ccAdjustZoomDlg::onPixelCountChanged(int pixelCount)
{
	assert(pixelCount > 0);

	pixelSizeDoubleSpinBox->blockSignals(true);
	double pixelSizeNPixels = (focalDoubleSpinBox->value() * m_distanceToWidthRatio * pixelCount) / m_windowWidth_pix;
	pixelSizeDoubleSpinBox->setValue( pixelSizeNPixels );
	pixelSizeDoubleSpinBox->blockSignals(false);
}
