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
#include "ccFileUtils.h"
#include "ccPersistentSettings.h"
#include "ccQCustomPlot.h"

//common
#include <ccPickingHub.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//Qt
#include <QCloseEvent>
#include <QSettings>

//System
#include <cassert>
#include <cmath>

//Gui
#include "ui_waveDlg.h"

ccWaveWidget::ccWaveWidget(QWidget* parent/*=0*/)
	: QCustomPlot(parent)
	, m_titlePlot(nullptr)
	, m_curve(nullptr)
	, m_dt(0.0)
	, m_minA(0.0)
	, m_maxA(0.0)
	, m_echoPos(-1.0)
	, m_vertBar(nullptr)
	, m_drawVerticalIndicator(false)
	, m_verticalIndicatorPositionPercent(0.0)
	, m_peakBar(nullptr)
	, m_lastMouseClick(0, 0)
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
	m_curveValues.resize(0);
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

static double AbsLog(double c) { return (c >= 0 ? log1p(c) : -log1p(-c)); }

void ccWaveWidget::init(ccPointCloud* cloud, unsigned pointIndex, bool logScale, double maxValue/*=0.0*/)
{
	clearInternal();

	if (!cloud || !cloud->hasFWF())
	{
		return;
	}

	const ccWaveformProxy& w = cloud->waveformProxy(pointIndex);
	if (!w.isValid())
	{
		//no valid descriptor
		return;
	}

	if (w.numberOfSamples() == 0)
	{
		return;
	}

	try
	{
		m_curveValues.resize(w.numberOfSamples(), 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Error("Not enough memory");
		return;
	}

	for (uint32_t i = 0; i < w.numberOfSamples(); ++i)
	{
		double c = w.getSample(i);
		if (logScale)
		{
			c = AbsLog(c);
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

	if (maxValue != 0)
	{
		m_maxA = logScale ? AbsLog(maxValue) : maxValue;
	}

	m_dt = w.descriptor().samplingRate_ps;
	m_echoPos = w.echoTime_ps();
}

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
			m_titlePlot = nullptr;
		}
		m_titlePlot = new QCPPlotTitle(this, m_titleStr);
		
		//title font
		m_renderingFont.setPointSize(ccGui::Parameters().defaultFontSize);
		m_titlePlot->setFont(m_renderingFont);
		plotLayout()->addElement(0, 0, m_titlePlot);
	}

	//clear previous display
	m_vertBar = nullptr;
	m_curve = nullptr;
	m_peakBar = nullptr;
	this->clearGraphs();
	this->clearPlottables();

	//wave curve
	int curveSize = static_cast<int>(m_curveValues.size());
	if (curveSize != 0)
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
		QString valueStr = QString("Sample %0").arg(curvePos);
		m_vertBar->setText(valueStr);
		valueStr = QString("= %0").arg(curvePos < curveSize ? m_curveValues[curvePos] : 0);
		m_vertBar->appendText(valueStr);
		m_vertBar->setTextAlignment(m_verticalIndicatorPositionPercent > 0.5);
	}

	if (m_echoPos >= 0)
	{
		m_peakBar = new QCPBarsWithText(xAxis, yAxis);
		addPlottable(m_peakBar);

		// now we can modify properties of vertBar
		m_peakBar->setName("PeakLine");
		m_peakBar->setWidth(0);
		m_peakBar->setBrush(QBrush(Qt::blue));
		m_peakBar->setPen(QPen(Qt::blue));
		m_peakBar->setAntialiasedFill(false);
		QVector<double> keyData(1);
		QVector<double> valueData(1);

		//horizontal position
		keyData[0] = m_echoPos;
		valueData[0] = m_maxA;

		m_peakBar->setData(keyData, valueData);

		//precision
		m_peakBar->setText("Peak");
		m_peakBar->setTextAlignment(m_echoPos > 0.5 * curveSize * m_dt);
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

ccWaveDialog::ccWaveDialog(	ccPointCloud* cloud,
							ccPickingHub* pickingHub,
							QWidget* parent/*=0*/)
	: QDialog(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_cloud(cloud)
	, m_widget(new ccWaveWidget(this))
	, m_pickingHub(pickingHub)
	, m_gui(new Ui_WaveDialog)
	, m_waveMax(0)
{
	m_gui->setupUi(this);
	
	QHBoxLayout* hboxLayout = new QHBoxLayout(m_gui->waveFrame);
	hboxLayout->addWidget(m_widget);
	hboxLayout->setContentsMargins(0, 0, 0, 0);
	m_gui->waveFrame->setLayout(hboxLayout);

	if (cloud && cloud->size())
	{
		m_gui->pointIndexSpinBox->setMaximum(static_cast<int>(cloud->size()));
		m_gui->pointIndexSpinBox->setSuffix(QString(" / %1").arg(cloud->size() - 1));

		//init m_waveMax
		double waveMin = 0;
		ccProgressDialog pDlg(parent);
		if (cloud->computeFWFAmplitude(waveMin, m_waveMax, &pDlg))
		{
			ccLog::Print(QString("[ccWaveDialog] Cloud '%1': max FWF amplitude = %2").arg(cloud->getName()).arg(m_waveMax));
		}
		else
		{
			ccLog::Warning("[ccWaveDialog] Input cloud has no valid FWF data");
		}
	}

	connect(m_gui->pointIndexSpinBox, SIGNAL(valueChanged(int)), this, SLOT(onPointIndexChanged(int)));
	connect(m_gui->logScaleCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateCurrentWaveform()));
	connect(m_gui->fixedAmplitudeCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateCurrentWaveform()));
	connect(m_gui->pointPickingToolButton, SIGNAL(toggled(bool)), SLOT(onPointPickingButtonToggled(bool)));
	connect(this, &QDialog::finished, [&]() { m_gui->pointPickingToolButton->setChecked(false); }); //auto disable picking mode when the dialog is closed
	connect(m_gui->saveWaveToolButton, SIGNAL(clicked()), SLOT(onExportWaveAsCSV()));


	//force update
	onPointIndexChanged(0);
}

ccWaveDialog::~ccWaveDialog()
{

	delete m_gui;
}

void ccWaveDialog::onPointIndexChanged(int index)
{
	if (!m_widget)
	{
		assert(false);
		return;
	}
	if (index < 0 || !m_cloud)
	{
		assert(false);
		return;
	}

	m_widget->init(m_cloud, static_cast<unsigned>(index), m_gui->logScaleCheckBox->isChecked(), m_gui->fixedAmplitudeCheckBox->isChecked() ? m_waveMax : 0.0);
	m_widget->refresh();
}

void ccWaveDialog::onItemPicked(const PickedItem& pi)
{
	if (pi.entity == m_cloud)
	{
		m_gui->pointIndexSpinBox->setValue(static_cast<int>(pi.itemIndex));
	}
}

void ccWaveDialog::updateCurrentWaveform()
{
	onPointIndexChanged(m_gui->pointIndexSpinBox->value());
}

void ccWaveDialog::onPointPickingButtonToggled(bool state)
{
	if (!m_pickingHub)
	{
		assert(false);
		return;
	}

	if (state)
	{
		if (!m_pickingHub->addListener(this))
		{
			ccLog::Error("Another tool is currently using the point picking mechanism.\nYou'll have to close it first.");
			m_gui->pointPickingToolButton->blockSignals(true);
			m_gui->pointPickingToolButton->setChecked(false);
			m_gui->pointPickingToolButton->blockSignals(false);
			return;
		}
	}
	else
	{
		m_pickingHub->removeListener(this);
	}
}

void ccWaveDialog::onExportWaveAsCSV()
{
	if (!m_cloud)
	{
		assert(false);
		return;
	}

	int pointIndex = m_gui->pointIndexSpinBox->value();
	if (pointIndex >= static_cast<int>(m_cloud->waveforms().size()))
	{
		assert(false);
		return;
	}
	
	const ccWaveformProxy& w = m_cloud->waveformProxy(pointIndex);
	if (!w.isValid())
	{
		//no valid descriptor
		return;
	}
	if (w.numberOfSamples() == 0)
	{
		//nothing to do
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	currentPath += QString("/") + QString("waveform_%1.csv").arg(pointIndex);

	//ask for a filename
	QString filename = QFileDialog::getSaveFileName(this, "Select output file", currentPath, "*.csv");
	if (filename.isEmpty())
	{
		//process cancelled by user
		return;
	}

	//save last saving location
	settings.setValue(ccPS::CurrentPath(), QFileInfo(filename).absolutePath());
	settings.endGroup();

	//save file
	w.toASCII(filename);
}
