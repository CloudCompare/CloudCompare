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

#include "ccHistogramWindow.h"
#include "ccGuiParameters.h"

//Local
#include "ccQCustomPlot.h"
#include "ccPersistentSettings.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccFileUtils.h>

//qCC_io
#include <ImageFileFilter.h>

//Qt
#include <QCloseEvent>
#include <QFile>
#include <QTextStream>
#include <QSettings>
#include <QFileDialog>

//System
#include <assert.h>
#include <cmath>

//Gui
#include "ui_histogramDlg.h"

ccHistogramWindow::ccHistogramWindow(QWidget* parent/*=0*/)
	: QCustomPlot(parent)
	, m_titlePlot(nullptr)
	, m_colorScheme(USE_SOLID_COLOR)
	, m_solidColor(Qt::blue)
	, m_colorScale(ccColorScalesManager::GetDefaultScale())
	, m_associatedSF(nullptr)
	, m_numberOfClassesCanBeChanged(false)
	, m_histogram(nullptr)
	, m_minVal(0)
	, m_maxVal(0)
	, m_maxHistoVal(0)
	, m_overlayCurve(nullptr)
	, m_vertBar(nullptr)
	, m_drawVerticalIndicator(false)
	, m_verticalIndicatorPositionPercent(0)
	, m_sfInteractionModes(SFInteractionMode::None)
	, m_axisDisplayOptions(AxisDisplayOption::All)
	, m_selectedItem(NONE)
	, m_areaLeft(nullptr)
	, m_areaLeftlastValue(std::numeric_limits<double>::quiet_NaN())
	, m_areaRight(nullptr)
	, m_areaRightlastValue(std::numeric_limits<double>::quiet_NaN())
	, m_arrowLeft(nullptr)
	, m_arrowLeftlastValue(std::numeric_limits<double>::quiet_NaN())
	, m_arrowRight(nullptr)
	, m_arrowRightlastValue(std::numeric_limits<double>::quiet_NaN())
	, m_lastMouseClick(0, 0)
	, m_refreshAfterResize(true)
{
	setWindowTitle("Histogram");
	setFocusPolicy(Qt::StrongFocus);

	setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

	setAutoAddPlottableToLegend(false);
	setAntialiasedElements(QCP::AntialiasedElement::aeAll);

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

ccHistogramWindow::~ccHistogramWindow()
{
	clearInternal();
}

void ccHistogramWindow::clear()
{
	clearInternal();
	refresh();
}

void ccHistogramWindow::clearInternal()
{
	if (m_associatedSF)
	{
		m_associatedSF->release();
		m_associatedSF = nullptr;
	}

	m_histoValues.resize(0);
	m_maxHistoVal = 0;

	m_curveValues.resize(0);

	m_selectedItem = NONE;
}

void ccHistogramWindow::setTitle(const QString& str)
{
	m_titleStr = str;
}

void ccHistogramWindow::setAxisLabels(const QString& xLabel, const QString& yLabel)
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

void ccHistogramWindow::fromSF(	ccScalarField* sf,
								unsigned initialNumberOfClasses/*=0*/,
								bool numberOfClassesCanBeChanged/*=true*/,
								bool showNaNValuesInGrey/*=true*/)
{
	if (sf && m_associatedSF != sf)
	{
		if (m_associatedSF)
			m_associatedSF->release();
		m_associatedSF = sf;
		if (m_associatedSF)
			m_associatedSF->link();
	}

	if (m_associatedSF)
	{
		m_minVal = showNaNValuesInGrey ? m_associatedSF->getMin() : m_associatedSF->displayRange().start();
		m_maxVal = showNaNValuesInGrey ? m_associatedSF->getMax() : m_associatedSF->displayRange().stop();
		m_numberOfClassesCanBeChanged = numberOfClassesCanBeChanged;
	}
	else
	{
		assert(false);
		m_minVal = m_maxVal = 0;
		m_numberOfClassesCanBeChanged = false;
	}

	setColorScheme(USE_SF_SCALE);
	setNumberOfClasses(initialNumberOfClasses);
};

void ccHistogramWindow::fromBinArray(	const std::vector<unsigned>& histoValues,
										double minVal,
										double maxVal)
{
	try
	{
		m_histoValues = histoValues;
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccHistogramWindow::fromBinArray] Not enough memory!");
		return;
	}
	m_minVal = minVal;
	m_maxVal = maxVal;
	m_numberOfClassesCanBeChanged = false;

	//update max histogram value
	m_maxHistoVal = getMaxHistoVal();
}

void ccHistogramWindow::setCurveValues(const std::vector<double>& curveValues)
{
	try
	{
		m_curveValues = curveValues;
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccHistogramWindow::setCurveValues] Not enough memory!");
	}
}

bool ccHistogramWindow::computeBinArrayFromSF(size_t binCount)
{
	//clear any existing histogram
	m_histoValues.resize(0);

	if (!m_associatedSF)
	{
		assert(false);
		ccLog::Error("[ccHistogramWindow::computeBinArrayFromSF] Need an associated SF!");
		return false;
	}

	if (binCount == 0)
	{
		assert(false);
		ccLog::Error("[ccHistogramWindow::computeBinArrayFromSF] Invalid number of classes!");
		return false;
	}

	//shortcut: same number of classes than the SF own histogram!
	if (binCount == m_associatedSF->getHistogram().size())
	{
		try
		{
			m_histoValues = m_associatedSF->getHistogram();
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("[ccHistogramWindow::computeBinArrayFromSF] Not enough memory!");
			return false;
		}
		return true;
	}

	//(try to) create new array
	try
	{
		m_histoValues.resize(binCount, 0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccHistogramWindow::computeBinArrayFromSF] Not enough memory!");
		return false;
	}

	double range = m_maxVal - m_minVal;
	if (range > 0.0)
	{
		unsigned count = m_associatedSF->currentSize();
		double step = range / static_cast<double>(binCount);
		for (unsigned i = 0; i < count; ++i)
		{
			double val = static_cast<double>(m_associatedSF->getValue(i));

			//we ignore values outside of [m_minVal,m_maxVal] (works fro NaN values as well)
			if (/*ccScalarField::ValidValue(val) &&*/val >= m_minVal && val <= m_maxVal)
			{
				size_t bin = static_cast<size_t>(floor((val - m_minVal) / step));
				++m_histoValues[std::min(bin, binCount - 1)];
			}
		}
	}
	else
	{
		m_histoValues[0] = m_associatedSF->currentSize();
	}

	return true;
}

unsigned ccHistogramWindow::getMaxHistoVal()
{
	unsigned m_maxHistoVal = 0;

	for (size_t i = 0; i < m_histoValues.size(); ++i)
	{
		m_maxHistoVal = std::max(m_maxHistoVal, m_histoValues[i]);
	}

	return m_maxHistoVal;
}

void ccHistogramWindow::setNumberOfClasses(size_t n)
{
	if (n == 0)
	{
		//invalid parameter
		assert(false);
		return;
	}

	if (n == m_histoValues.size())
	{
		//nothing to do
		return;
	}

	if (m_associatedSF)
	{
		//dynamically recompute histogram values
		computeBinArrayFromSF(n);
	}

	//update max histogram value
	m_maxHistoVal = getMaxHistoVal();
}

void ccHistogramWindow::refreshBars()
{
	if (	m_histogram
		&&	m_colorScheme == USE_SF_SCALE
		&&	m_associatedSF
		&&	m_associatedSF->getColorScale())
	{
		int histoSize = static_cast<int>(m_histoValues.size());

		//DGM: the bars will be redrawn only if we delete and recreate the graph?!
		m_histogram->clearData();

		QVector<double> keyData(histoSize);
		QVector<double> valueData(histoSize);
		QVector<QColor> colors(histoSize);

		for (int i = 0; i < histoSize; ++i)
		{
			//we take the 'normalized' value at the middle of the class
			double normVal = (static_cast<double>(i)+0.5) / histoSize;

			keyData[i] = m_minVal + normVal * (m_maxVal - m_minVal);
			valueData[i] = m_histoValues[i];

			const ccColor::Rgb* col = m_associatedSF->getColor(static_cast<ScalarType>(keyData[i]));
			if (!col) //hidden values may have no associated color!
				col = &ccColor::lightGreyRGB;
			colors[i] = QColor(col->r, col->g, col->b);
		}

		m_histogram->setData(keyData, valueData, colors);

		//rescaleAxes();
	}

	replot(QCustomPlot::rpImmediate);
}

void ccHistogramWindow::setSFInteractionMode(SFInteractionModes modes)
{
   m_sfInteractionModes = modes;
}

void ccHistogramWindow::setAxisDisplayOption(AxisDisplayOptions axisOptions)
{
	m_axisDisplayOptions = axisOptions;
}

void ccHistogramWindow::setRefreshAfterResize(bool refreshAfterResize)
{
	m_refreshAfterResize = refreshAfterResize;
}

void ccHistogramWindow::refresh()
{
	// set ranges appropriate to show data
	double minVal = m_minVal;
	double maxVal = m_maxVal;
	if (m_sfInteractionModes && m_associatedSF)
	{
		double minSat = m_associatedSF->saturationRange().min();
		double maxSat = m_associatedSF->saturationRange().max();
		minVal = std::min(minVal, minSat);
		maxVal = std::max(maxVal, maxSat);
	}
	xAxis->setRange(minVal, std::max(minVal + std::numeric_limits<ScalarType>::epsilon(), maxVal));
	yAxis->setRange(0, m_maxHistoVal);

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
		m_titlePlot = new QCPPlotTitle(this, QString("%0 [%1 classes]").arg(m_titleStr).arg(m_histoValues.size()));
		//title font
		m_renderingFont.setPointSize(ccGui::Parameters().defaultFontSize);
		m_titlePlot->setFont(m_renderingFont);
		plotLayout()->addElement(0, 0, m_titlePlot);
	}

	//clear previous display
	m_histogram = nullptr;
	m_vertBar = nullptr;
	m_overlayCurve = nullptr;
	m_areaLeft = nullptr;
	m_areaRight = nullptr;
	m_arrowLeft = nullptr;
	m_arrowRight = nullptr;
	this->clearGraphs();
	this->clearPlottables();

	if (m_histoValues.empty())
		return;

	//default color scale to be used for display
	ccColorScale::Shared colorScale = (m_colorScale ? m_colorScale : ccColorScalesManager::GetDefaultScale());

	//histogram
	int histoSize = static_cast<int>(m_histoValues.size());
	double totalSum = 0;
	double partialSum = 0;
	if (histoSize > 0)
	{
		m_histogram = new QCPColoredBars(xAxis, yAxis);

		m_histogram->setWidth((m_maxVal - m_minVal) / histoSize);
		m_histogram->setAntialiased(false);
		m_histogram->setAntialiasedFill(false);
		
		addPlottable(m_histogram);
		
		QVector<double> keyData(histoSize);
		QVector<double> valueData(histoSize);

		HISTOGRAM_COLOR_SCHEME colorScheme = m_colorScheme;
		switch (colorScheme)
		{
		case USE_SOLID_COLOR:
			m_histogram->setBrush(QBrush(m_solidColor, Qt::SolidPattern));
			m_histogram->setPen(QPen(m_solidColor));
			break;
		case USE_CUSTOM_COLOR_SCALE:
			//nothing to do
			break;
		case USE_SF_SCALE:
			if (m_associatedSF && m_associatedSF->getColorScale())
			{
				//we use the SF's color scale
				colorScale = m_associatedSF->getColorScale();
			}
			else
			{
				//we'll use the default one...
				assert(false);
				colorScheme = USE_CUSTOM_COLOR_SCALE;
			}
			break;
		default:
			assert(false);
			colorScheme = USE_CUSTOM_COLOR_SCALE;
			break;
		}

		QVector<QColor> colors;
		if (colorScheme != USE_SOLID_COLOR)
		{
			colors.resize(histoSize);
		}

		for (int i = 0; i < histoSize; ++i)
		{
			//we take the 'normalized' value at the middle of the class
			double normVal = (static_cast<double>(i)+0.5) / histoSize;

			totalSum += m_histoValues[i];
			if (normVal < m_verticalIndicatorPositionPercent)
				partialSum += m_histoValues[i];

			keyData[i] = m_minVal + normVal * (m_maxVal - m_minVal);
			valueData[i] = m_histoValues[i];

			//import color for the current bin
			if (colorScheme != USE_SOLID_COLOR)
			{
				const ccColor::Rgb* col = nullptr;
				if (colorScheme == USE_SF_SCALE)
				{
					//equivalent SF value
					assert(m_associatedSF);
					col = m_associatedSF->getColor(static_cast<ScalarType>(keyData[i]));
				}
				else if (colorScheme == USE_CUSTOM_COLOR_SCALE)
				{
					//use default gradient
					assert(colorScale);
					col = colorScale->getColorByRelativePos(normVal);
				}
				if (!col) //hidden values may have no associated color!
				{
					col = &ccColor::lightGreyRGB;
				}
				colors[i] = QColor(col->r, col->g, col->b);
			}
		}

		if (!colors.isEmpty())
		{
			m_histogram->setData(keyData, valueData, colors);
		}
		else
		{
			m_histogram->setData(keyData, valueData);
		}
	}

	//overlay curve?
	int curveSize = static_cast<int>(m_curveValues.size());
	if (curveSize > 1)
	{
		QVector<double> x(curveSize);
		QVector<double> y(curveSize);

		double step = (m_maxVal - m_minVal) / (curveSize - 1);
		for (int i = 0; i < curveSize; ++i)
		{
			x[i] = m_minVal + (static_cast<double>(i)/*+0.5*/) * step;
			y[i] = m_curveValues[i];
		}

		// create graph and assign data to it:
		m_overlayCurve = addGraph();
		m_overlayCurve->setData(x, y);
		m_overlayCurve->setName("OverlayCurve");

		//set pen color
		const ccColor::Rgb& col = ccColor::darkGrey;
		QPen pen(QColor(col.r, col.g, col.b));
		m_overlayCurve->setPen(pen);

		//set width
		updateOverlayCurveWidth(rect().width(), rect().height());
	}

	//sf interaction mode
	if (m_sfInteractionModes && m_associatedSF)
	{
		if ( m_sfInteractionModes.testFlag(SFInteractionMode::DisplayRange) )
		{
			const ccScalarField::Range& dispRange = m_associatedSF->displayRange();
	
			m_areaLeft = new QCPHiddenArea(true, xAxis, yAxis);
			m_areaLeft->setRange(dispRange.min(), dispRange.max());
			m_areaLeft->setCurrentVal(!std::isnan(m_areaLeftlastValue) ? m_areaLeftlastValue : dispRange.start());
			addPlottable(m_areaLeft);
	
			m_areaRight = new QCPHiddenArea(false, xAxis, yAxis);
			m_areaRight->setRange(dispRange.min(), dispRange.max());
			m_areaRight->setCurrentVal(!std::isnan(m_areaRightlastValue) ? m_areaRightlastValue : dispRange.stop());
			addPlottable(m_areaRight);
		}

		if ( m_sfInteractionModes.testFlag(SFInteractionMode::SaturationRange) )
		{
			const ccScalarField::Range& satRange = m_associatedSF->saturationRange();
			
			m_arrowLeft = new QCPArrow(xAxis, yAxis);
			m_arrowLeft->setRange(satRange.min(), satRange.max());
			m_arrowLeft->setCurrentVal(!std::isnan(m_arrowLeftlastValue) ? m_arrowLeftlastValue : satRange.start());
			if (colorScale)
			{
				const ccColor::Rgb* col = colorScale->getColorByRelativePos(m_associatedSF->symmetricalScale() ? 0.5 : 0, m_associatedSF->getColorRampSteps());
				if (col)
				{
					m_arrowLeft->setColor(col->r, col->g, col->b);
				}
			}
			addPlottable(m_arrowLeft);
			
			m_arrowRight = new QCPArrow(xAxis, yAxis);
			m_arrowRight->setRange(satRange.min(), satRange.max());
			m_arrowRight->setCurrentVal(!std::isnan(m_arrowRightlastValue) ? m_arrowRightlastValue : satRange.stop());
			if (colorScale)
			{
				const ccColor::Rgb* col = colorScale->getColorByRelativePos(1.0, m_associatedSF->getColorRampSteps());
				if (col)
				{
					m_arrowRight->setColor(col->r, col->g, col->b);
				}
			}
			addPlottable(m_arrowRight);
		}
	}
	else if (m_drawVerticalIndicator) //vertical hint
	{
		m_vertBar = new QCPBarsWithText(xAxis, yAxis);
		addPlottable(m_vertBar);

		// now we can modify properties of vertBar
		m_vertBar->setName("VertLine");
		m_vertBar->setWidth(0/*(m_maxVal - m_minVal) / histoSize*/);
		m_vertBar->setBrush(QBrush(Qt::red));
		m_vertBar->setPen(QPen(Qt::red));
		m_vertBar->setAntialiasedFill(false);
		QVector<double> keyData(1);
		QVector<double> valueData(1);

		//horizontal position
		keyData[0] = m_minVal + (m_maxVal - m_minVal) * m_verticalIndicatorPositionPercent;
		valueData[0] = m_maxHistoVal;

		m_vertBar->setData(keyData, valueData);

		//precision (same as color scale)
		int precision = static_cast<int>(ccGui::Parameters().displayedNumPrecision);
		unsigned bin = static_cast<unsigned>(m_verticalIndicatorPositionPercent * m_histoValues.size());
		QString valueStr = QString("bin %0").arg(bin);
		m_vertBar->setText(valueStr);
		valueStr = QString("< %0 %").arg(100.0*static_cast<double>(partialSum) / static_cast<double>(totalSum), 0, 'f', 3);
		m_vertBar->appendText(valueStr);
		valueStr = QString("val = %0").arg(m_minVal + (m_maxVal - m_minVal)*m_verticalIndicatorPositionPercent, 0, 'f', precision);
		m_vertBar->appendText(valueStr);
		m_vertBar->setTextAlignment(m_verticalIndicatorPositionPercent > 0.5);
	}

	//rescaleAxes();

	// redraw
	replot();
}

void ccHistogramWindow::setMinDispValue(double val)
{
	m_areaLeftlastValue = val;
	if (m_areaLeft && m_areaLeft->currentVal() != val)
	{
		m_areaLeft->setCurrentVal(val);

		if (m_associatedSF)
		{
			//auto-update
			m_associatedSF->setMinDisplayed(static_cast<ScalarType>(val));
			refreshBars();
		}
		else
		{
			replot();
		}

		emit sfMinDispValChanged(val);
	}
}

void ccHistogramWindow::setMaxDispValue(double val)
{
	m_areaRightlastValue = val;
	if (m_areaRight && m_areaRight->currentVal() != val)
	{
		m_areaRight->setCurrentVal(val);

		if (m_associatedSF)
		{
			//auto-update
			m_associatedSF->setMaxDisplayed(static_cast<ScalarType>(val));
			refreshBars();
		}
		else
		{
			replot();
		}

		emit sfMaxDispValChanged(val);
	}
}

void ccHistogramWindow::setMinSatValue(double val)
{
	m_arrowLeftlastValue = val;
	if (m_arrowLeft && m_arrowLeft->currentVal() != val)
	{
		m_arrowLeft->setCurrentVal(val);

		if (m_associatedSF)
		{
			//auto-update
			m_associatedSF->setSaturationStart(static_cast<ScalarType>(val));
			refreshBars();
		}
		else
		{
			replot();
		}

		emit sfMinSatValChanged(val);
	}
}

void ccHistogramWindow::setMaxSatValue(double val)
{
	m_arrowRightlastValue = val;
	if (m_arrowRight && m_arrowRight->currentVal() != val)
	{
		m_arrowRight->setCurrentVal(val);

		if (m_associatedSF)
		{
			//auto-update
			m_associatedSF->setSaturationStop(static_cast<ScalarType>(val));
			refreshBars();
		}
		replot();

		emit sfMaxSatValChanged(val);
	}
}

void ccHistogramWindow::updateOverlayCurveWidth(int w, int h)
{
	if (m_overlayCurve)
	{
		int penWidth = std::max(w, h) / 200;
		if (m_overlayCurve->pen().width() != penWidth)
		{
			QPen pen = m_overlayCurve->pen();
			pen.setWidth(penWidth);
			m_overlayCurve->setPen(pen);
		}
	}
}

void ccHistogramWindow::resizeEvent(QResizeEvent * event)
{
	QCustomPlot::resizeEvent(event);

	updateOverlayCurveWidth(event->size().width(), event->size().height());

	if (m_refreshAfterResize)
	{
		refresh();
	}
}

void ccHistogramWindow::mousePressEvent(QMouseEvent *event)
{
	m_lastMouseClick = event->pos();

	if (m_sfInteractionModes)
	{
		m_selectedItem = NONE;
		//check greyed areas (circles)
		if ( m_sfInteractionModes.testFlag(SFInteractionMode::DisplayRange) )
		{
			if (m_areaLeft && m_areaLeft->isSelectable(m_lastMouseClick))
				m_selectedItem = LEFT_AREA;
			if (m_areaRight && m_areaRight->isSelectable(m_lastMouseClick))
			{
				if (m_selectedItem == NONE)
					m_selectedItem = RIGHT_AREA;
				else
					m_selectedItem = BOTH_AREAS;
			}
		}

		//check yellow triangles
		if ( m_sfInteractionModes.testFlag(SFInteractionMode::SaturationRange)
			 && (m_selectedItem == NONE) )
		{
			if (m_arrowLeft && m_arrowLeft->isSelectable(m_lastMouseClick))
				m_selectedItem = LEFT_ARROW;
			if (m_arrowRight && m_arrowRight->isSelectable(m_lastMouseClick))
			{
				if (m_selectedItem == NONE)
					m_selectedItem = RIGHT_ARROW;
				else
					m_selectedItem = BOTH_ARROWS;
			}
		}
	}
	else
	{
		mouseMoveEvent(event);
	}
}

void ccHistogramWindow::mouseMoveEvent(QMouseEvent *event)
{
	if (event->buttons() & Qt::LeftButton)
	{
		if (m_sfInteractionModes)
		{
			QPoint mousePos = event->pos();
			if (m_histogram)
			{
				QRect rect = m_histogram->rect();
				mousePos.setX(std::min(rect.x() + rect.width(), std::max(rect.x(), mousePos.x())));
			}

			switch (m_selectedItem)
			{
			case NONE:
				//nothing to do
				break;
			case LEFT_AREA:
				if (m_areaLeft)
				{
					double newValue = m_areaLeft->pixelToKey(mousePos.x());
					if (m_areaRight)
						newValue = std::min(newValue, m_areaRight->currentVal());
					setMinDispValue(newValue);
				}
				break;
			case RIGHT_AREA:
				if (m_areaRight)
				{
					double newValue = m_areaRight->pixelToKey(mousePos.x());
					if (m_areaLeft)
						newValue = std::max(newValue, m_areaLeft->currentVal());
					setMaxDispValue(newValue);
				}
				break;
			case BOTH_AREAS:
			{
				int dx = m_lastMouseClick.x() - mousePos.x();
				if (dx < -2)
				{
					//going to the right
					m_selectedItem = RIGHT_AREA;
					//call the same method again
					mouseMoveEvent(event);
					return;
				}
				else if (dx > 2)
				{
					//going to the left
					m_selectedItem = LEFT_AREA;
					//call the same method again
					mouseMoveEvent(event);
					return;
				}
				//else: nothing we can do right now!
			}
			break;
			case LEFT_ARROW:
				if (m_arrowLeft)
				{
					double newValue = m_arrowLeft->pixelToKey(mousePos.x());
					if (m_arrowRight)
						newValue = std::min(newValue, m_arrowRight->currentVal());
					setMinSatValue(newValue);
				}
				break;
			case RIGHT_ARROW:
				if (m_arrowRight)
				{
					double newValue = m_arrowRight->pixelToKey(mousePos.x());
					if (m_arrowLeft)
						newValue = std::max(newValue, m_arrowLeft->currentVal());
					setMaxSatValue(newValue);
				}
				break;
			case BOTH_ARROWS:
			{
				int dx = m_lastMouseClick.x() - mousePos.x();
				if (dx < -2)
				{
					//going to the right
					m_selectedItem = RIGHT_ARROW;
					//call the same method again
					mouseMoveEvent(event);
					return;
				}
				else if (dx > 2)
				{
					//going to the left
					m_selectedItem = LEFT_ARROW;
					//call the same method again
					mouseMoveEvent(event);
					return;
				}
				//else: nothing we can do right now!
			}
			break;
			default:
				assert(false);
				break;
			}
		}
		else
		{
			if (m_histogram && !m_histoValues.empty())
			{
				QRect roi = m_histogram->rect();
				if (roi.contains(event->pos(), false))
				{
					m_drawVerticalIndicator = true;

					int verticalIndicatorPosition = (static_cast<int>(m_histoValues.size()) * (event->x() - roi.x())) / roi.width();
					m_verticalIndicatorPositionPercent = static_cast<double>(verticalIndicatorPosition) / m_histoValues.size();

					refresh();
				}
			}
		}
	}
	else
	{
		event->ignore();
	}
}

void ccHistogramWindow::wheelEvent(QWheelEvent* e)
{
	if (!m_numberOfClassesCanBeChanged)
	{
		e->ignore();
		return;
	}

	if (e->delta() < 0)
	{
		if (m_histoValues.size() > 4)
		{
			setNumberOfClasses(std::max<size_t>(4, m_histoValues.size() - 4));
			refresh();
		}
	}
	else //if (e->delta() > 0)
	{
		setNumberOfClasses(m_histoValues.size() + 4);
		refresh();
	}

	e->accept();
}

ccHistogramWindowDlg::ccHistogramWindowDlg(QWidget* parent/*=0*/)
	: QDialog(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, m_win(new ccHistogramWindow(this))
	, m_gui(new Ui_HistogramDialog)
{
	m_gui->setupUi(this);
	
	auto hboxLayout = new QHBoxLayout;
	
	hboxLayout->setContentsMargins(0, 0, 0, 0);
	hboxLayout->addWidget(m_win);

	m_gui->histoFrame->setLayout(hboxLayout);

	connect(m_gui->exportCSVToolButton, &QAbstractButton::clicked, this, &ccHistogramWindowDlg::onExportToCSV);
	connect(m_gui->exportImageToolButton, &QAbstractButton::clicked, this, &ccHistogramWindowDlg::onExportToImage);
}

ccHistogramWindowDlg::~ccHistogramWindowDlg()
{
	delete m_gui;
}

//CSV file default separator
static const QChar s_csvSep(';');

bool ccHistogramWindowDlg::exportToCSV(QString filename) const
{
	if (!m_win || m_win->histoValues().empty())
	{
		ccLog::Warning("[Histogram] Histogram has no associated values (can't save file)");
		return false;
	}

	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		ccLog::Warning(QString("[Histogram] Failed to save histogram to file '%1'").arg(filename));
		return false;
	}

	QTextStream stream(&file);
	stream.setRealNumberPrecision(12);
	stream.setRealNumberNotation(QTextStream::FixedNotation);

	//header
	stream << "Class; Value; Class start; Class end;" << endl;

	//data
	{
		const std::vector<unsigned>& histoValues = m_win->histoValues();
		int histoSize = static_cast<int>(histoValues.size());
		double step = (m_win->maxVal() - m_win->minVal()) / histoSize;
		for (int i = 0; i < histoSize; ++i)
		{
			double minVal = m_win->minVal() + i*step;
			stream << i + 1;				//class index
			stream << s_csvSep;
			stream << histoValues[i];	//class value
			stream << s_csvSep;
			stream << minVal;			//min value
			stream << s_csvSep;
			stream << minVal + step;	//max value
			stream << s_csvSep;
			stream << endl;
		}
	}

	file.close();

	ccLog::Print(QString("[Histogram] File '%1' saved").arg(filename));

	return true;
}

void ccHistogramWindowDlg::onExportToCSV()
{
	if (!m_win)
	{
		assert(false);
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	currentPath += QString("/") + m_win->windowTitle() + ".csv";

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
	exportToCSV(filename);
}

void ccHistogramWindowDlg::onExportToImage()
{
	if (!m_win)
	{
		assert(false);
		return;
	}

	//persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::SaveFile());
	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();

	QString outputFilename = ImageFileFilter::GetSaveFilename("Select output file",
		m_win->windowTitle(),
		currentPath,
		this);

	if (outputFilename.isEmpty())
	{
		//process cancelled by user (or error)
		return;
	}

	//save current export path to persistent settings
	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
	settings.endGroup();

	//save the widget as an image file
	QPixmap image = m_win->grab();
	if (image.save(outputFilename))
	{
		ccLog::Print(QString("[Histogram] Image '%1' successfully saved").arg(outputFilename));
	}
	else
	{
		ccLog::Error(QString("Failed to save file '%1'").arg(outputFilename));
	}
}
