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
//#                       COPYRIGHT: CNRS / OSUR                           #
//#                                                                        #
//##########################################################################

#include "ccWaveformDialog.h"

//Local
#include "ccQCustomPlot.h"
#include "ccPersistentSettings.h"
#include "ccGuiParameters.h"

//qCC_db
#include <ccPointCloud.h>

//Qt
#include <QCloseEvent>
#include <QSettings>

//System
#include <assert.h>

//Gui
#include "ui_waveDlg.h"

ccWaveWidget::ccWaveWidget(QWidget* parent/*=0*/)
	: QCustomPlot(parent)
	, m_titlePlot(0)
	, m_curve(0)
	, m_dt(0)
	, m_minA(0)
	, m_maxA(0)
	, m_vertBar(0)
	, m_drawVerticalIndicator(false)
	, m_verticalIndicatorPositionPercent(0)
	, m_lastMouseClick(0,0)
{
	setWindowTitle("Waveform");
	setFocusPolicy(Qt::StrongFocus);

	setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

	setAutoAddPlottableToLegend(false);

	//default font for text rendering
	m_renderingFont.setFamily(QString::fromUtf8("Arial"));
	m_renderingFont.setBold(false);
	//m_renderingFont.setWeight(75);

	// make ticks on bottom axis go outward
	assert(xAxis && yAxis);
	xAxis->setTickLength(0, 5);
	xAxis->setSubTickLength(0, 3);
	yAxis->setTickLength(0, 5);
	yAxis->setSubTickLength(0, 3);
}

ccWaveWidget::~ccWaveWidget()
{
	clearInternal();
}

void ccWaveWidget::clear()
{
	clearInternal();
	refresh();
}

void ccWaveWidget::clearInternal()
{
	m_curveValues.clear();
	m_dt = 0;
	m_minA = m_maxA = 0;
}

void ccWaveWidget::setTitle(const QString& str)
{
	m_titleStr = str;
}

void ccWaveWidget::setAxisLabels(const QString& xLabel, const QString& yLabel)
{
	if (xLabel.isNull())
	{
		xAxis->setVisible(false);
	}
	else
	{
		// set labels
		xAxis->setLabel(xLabel);
		xAxis->setVisible(true);
	}

	if (xLabel.isNull())
	{
		yAxis->setVisible(false);
	}
	else
	{
		// set labels
		yAxis->setLabel(yLabel);
		yAxis->setVisible(true);
	}
}

void ccWaveWidget::init(ccPointCloud* cloud, unsigned pointIndex, bool logScale)
{
	clearInternal();

	if (!cloud || !cloud->hasFWF())
	{
		return;
	}

	const ccWaveform& w = cloud->fwfData()[pointIndex];
	uint8_t descriptorID = w.descriptorID();
	if (descriptorID == 0 || !cloud->fwfDescriptors().contains(descriptorID))
	{
		//no valid descriptor
		return;
	}

	const WaveformDescriptor& d = cloud->fwfDescriptors()[descriptorID];
	if (d.numberOfSamples == 0)
	{
		return;
	}

	try
	{
		m_curveValues.resize(d.numberOfSamples, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return;
	}

	for (uint32_t i = 0; i < d.numberOfSamples; ++i)
	{
		double c = w.getSample(i, d);
		if (logScale)
		{
			if (c >= 0)
			{
				c = log(1.0 + c);
			}
			else // c < 0
			{
				c = -log(1.0 - c);
			}
		}
		m_curveValues[i] = c;

		if (i)
		{
			m_maxA = std::max(m_maxA, c);
			m_minA = std::min(m_minA, c);
		}
		else
		{
			m_minA = m_maxA = c;
		}
	}

	m_dt = d.samplingRate_ps;

	refresh();
};

void ccWaveWidget::refresh()
{
	// set ranges appropriate to show data
	xAxis->setRange(0, m_curveValues.size() * m_dt);
	yAxis->setRange(m_minA, m_maxA);

	if (!m_titleStr.isEmpty())
	{
		// add title layout element
		if (!m_titlePlot)
		{
			//add a row for the title
			plotLayout()->insertRow(0);
		}
		else
		{
			//remove previous title
			plotLayout()->remove(m_titlePlot);
			m_titlePlot = 0;
		}
		m_titlePlot = new QCPPlotTitle(this, m_titleStr);
		
		//title font
		m_renderingFont.setPointSize(ccGui::Parameters().defaultFontSize);
		m_titlePlot->setFont(m_renderingFont);
		plotLayout()->addElement(0, 0, m_titlePlot);
	}

	//clear previous display
	m_vertBar	= 0;
	m_curve		= 0;
	this->clearGraphs();
	this->clearPlottables();

	if (m_curveValues.empty())
	{
		return;
	}

	//wave curve
	int curveSize = static_cast<int>(m_curveValues.size());
	{
		QVector<double> x(curveSize), y(curveSize);
		
		for (int i = 0; i < curveSize; ++i)
		{
			x[i] = i * m_dt;
			y[i] = m_curveValues[i];
		}

		// create graph and assign data to it:
		m_curve = addGraph();
		m_curve->setData(x, y);
		m_curve->setName("WaveCurve");

		//set pen color
		QPen pen(Qt::blue);
		m_curve->setPen(pen);

		//set width
		updateCurveWidth(rect().width(), rect().height());
	}
	
	if (m_drawVerticalIndicator) //vertical hint
	{
		m_vertBar = new QCPBarsWithText(xAxis, yAxis);
		addPlottable(m_vertBar);
		
		// now we can modify properties of vertBar
		m_vertBar->setName("VertLine");
		m_vertBar->setWidth(0);
		m_vertBar->setBrush(QBrush(Qt::red));
		m_vertBar->setPen(QPen(Qt::red));
		m_vertBar->setAntialiasedFill(false);
		QVector<double> keyData(1);
		QVector<double> valueData(1);

		//horizontal position
		int curvePos = static_cast<int>(curveSize * m_verticalIndicatorPositionPercent);
		keyData[0] = curvePos * m_dt;
		valueData[0] = m_maxA;

		m_vertBar->setData(keyData, valueData);

		//precision
		int precision = static_cast<int>(ccGui::Parameters().displayedNumPrecision);
		
		QString valueStr = QString("Sample %0").arg(curvePos);
		m_vertBar->setText(valueStr);
		valueStr = QString("= %0").arg(curvePos < curveSize ? m_curveValues[curvePos] : 0);
		m_vertBar->appendText(valueStr);
		m_vertBar->setTextAlignment(m_verticalIndicatorPositionPercent > 0.5);
	}

	//rescaleAxes();

	// redraw
	replot();
}

void ccWaveWidget::updateCurveWidth(int w, int h)
{
	if (m_curve)
	{
		int penWidth = std::max(w, h) / 200;
		if (m_curve->pen().width() != penWidth)
		{
			QPen pen = m_curve->pen();
			pen.setWidth(penWidth);
			m_curve->setPen(pen);
		}
	}
}

void ccWaveWidget::resizeEvent(QResizeEvent * event)
{
	QCustomPlot::resizeEvent(event);

	updateCurveWidth(event->size().width(), event->size().height());
	
	refresh();
}

void ccWaveWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastMouseClick = event->pos();

	mouseMoveEvent(event);
}

void ccWaveWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (event->buttons() & Qt::LeftButton)
	{
		if (m_curve && !m_curveValues.empty())
		{
			QRect roi = /*m_curve->*/rect();
			if (roi.contains(event->pos(), false))
			{
				m_drawVerticalIndicator = true;
				m_verticalIndicatorPositionPercent = static_cast<double>(event->x() - roi.x()) / roi.width();
				refresh();
			}
		}
	}
	else
	{
		event->ignore();
	}
}

//void ccWaveWidget::wheelEvent(QWheelEvent* e)
//{
//	e->ignore();
//}

ccWaveDialog::ccWaveDialog(ccPointCloud* cloud, QWidget* parent/*=0*/)
	: QDialog(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_cloud(cloud)
	, m_widget(new ccWaveWidget(this))
	, m_gui(new Ui_WaveDialog)
{
	m_gui->setupUi(this);
	
	QHBoxLayout* hboxLayout = new QHBoxLayout(m_gui->waveFrame);
	hboxLayout->addWidget(m_widget);
	hboxLayout->setContentsMargins(0,0,0,0);
	m_gui->waveFrame->setLayout(hboxLayout);

	if (cloud && cloud->size())
	{
		m_gui->pointIndexSpinBox->setMaximum(static_cast<int>(cloud->size()));
		m_gui->pointIndexSpinBox->setSuffix(QString(" / %1").arg(cloud->size()-1));
	}

	connect(m_gui->pointIndexSpinBox, SIGNAL(valueChanged(int)), this, SLOT(onPointIndexChanged(int)));
	connect(m_gui->logScaleCheckBox, SIGNAL(toggled(bool)), this, SLOT(onLogScaleToggled(bool)));

	//force update
	onPointIndexChanged(0);
}

ccWaveDialog::~ccWaveDialog()
{
	if (m_gui)
	{
		delete m_gui;
	}
}

void ccWaveDialog::onPointIndexChanged(int index)
{
	if (index < 0 || !m_cloud)
	{
		assert(false);
		return;
	}

	m_widget->init(m_cloud, static_cast<unsigned>(index), m_gui->logScaleCheckBox->isChecked());
}

void ccWaveDialog::onLogScaleToggled(bool)
{
	onPointIndexChanged(m_gui->pointIndexSpinBox->value());
}
