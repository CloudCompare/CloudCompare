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

#include "ccHistogramWindow.h"
#include "ccGuiParameters.h"

//qCC_db
#include <ccColorScalesManager.h>

//Qt
#include <QCloseEvent>

//System
#include <assert.h>

/*********************************/
/*** CUSTOM QCustomPlot wigets ***/
/*********************************/

//! Vertical bar with text along side
class QCPBarsWithText : public QCPBars
{
public:

	QCPBarsWithText(QCPAxis* keyAxis, QCPAxis* valueAxis) : QCPBars(keyAxis,valueAxis), m_textOnTheLeft(false) {}

	void setText(QString text) { m_text = QStringList(text); }
	void appendText(QString text) { m_text.append(text); }
	void setTextAlignment(bool left) { m_textOnTheLeft = left; }

protected:
	
	QStringList m_text;
	bool m_textOnTheLeft;
	
	// reimplemented virtual draw method
	virtual void draw(QCPPainter *painter)
	{
		if (!mKeyAxis || !mValueAxis) { qDebug() << Q_FUNC_INFO << "invalid key or value axis"; return; }

		//switch to standard display
		QCPBars::draw(painter);

		int fontHeight = painter->fontMetrics().height();

		if (!mData->isEmpty())
		{
			double& key = mData->begin()->key;
			double& value = mData->begin()->value;
			QPointF P = coordsToPixels(key, value);
			//apply a small shift
			int margin = 5; //in pixels
			if (m_textOnTheLeft)
				margin = -margin;
			P.setX(P.x() + margin);
			//we draw at the 'base' line
			P.setY(P.y() + fontHeight);

			for (int i=0; i<m_text.size(); ++i)
			{
				QPointF Pstart = P;
				if (m_textOnTheLeft)
					Pstart.setX(P.x() - painter->fontMetrics().width(m_text[i]));
				painter->drawText(Pstart,m_text[i]);
				P.setY(P.y() + fontHeight);
			}
		}
	}

};

//! Colored histogram
class QCPColoredBars : public QCPBars
{
public:

	class QCPColoredBarData : public QCPBarData
	{
	public:
		QCPColoredBarData()
			: QCPBarData()
			, color(Qt::blue)
		{}

		QColor color;
	};
	typedef QMap<double, QCPColoredBarData> QCPColoredBarDataMap;

	QCPColoredBars(QCPAxis *keyAxis, QCPAxis *valueAxis) : QCPBars(keyAxis,valueAxis) {}
	
	void setData(const QVector<double> &key, const QVector<double> &value)
	{
		//no colors? we switch to the standard QCPBars object
		m_coloredData.clear();
		QCPBars::setData(key,value);
	}

	void setData(const QVector<double> &key, const QVector<double> &value, const QVector<QColor>& colors)
	{
		Q_ASSERT(colors.size() == key.size());

		mData->clear(); //we duplicate the structures so that other stuff in QCPBarData works!

		int n = qMin(key.size(), value.size());

		for (int i=0; i<n; ++i)
		{
			QCPColoredBarData newData;
			newData.key = key[i];
			newData.value = value[i];
			if (colors.size() > i)
				newData.color = colors[i];
			m_coloredData.insertMulti(newData.key, newData);
			mData->insertMulti(newData.key, newData);
		}
	}

	inline QRect rect() const { return clipRect(); }

protected:

	// reimplemented virtual draw method
	virtual void draw(QCPPainter *painter)
	{
		//no colors?
		if (m_coloredData.empty())
		{
			//switch to standard display
			QCPBars::draw(painter);
		}

		if (!mKeyAxis || !mValueAxis) { qDebug() << Q_FUNC_INFO << "invalid key or value axis"; return; }

		QCPColoredBarDataMap::const_iterator it;
		for (it = m_coloredData.constBegin(); it != m_coloredData.constEnd(); ++it)
		{
			// skip bar if not visible in key axis range:
			if (it.key()+mWidth*0.5 < mKeyAxis.data()->range().lower || it.key()-mWidth*0.5 > mKeyAxis.data()->range().upper)
				continue;

			QPolygonF barPolygon = getBarPolygon(it.key(), it.value().value);
			// draw bar fill:
			if (mainBrush().style() != Qt::NoBrush && mainBrush().color().alpha() != 0)
			{
				QBrush brush = mainBrush();
				brush.setColor(it.value().color);
				
				applyFillAntialiasingHint(painter);
				painter->setPen(Qt::NoPen);
				painter->setBrush(brush);
				painter->drawPolygon(barPolygon);
			}
			// draw bar line:
			if (mainPen().style() != Qt::NoPen && mainPen().color().alpha() != 0)
			{
				QPen pen = mainPen();
				pen.setColor(it.value().color);

				applyDefaultAntialiasingHint(painter);
				painter->setPen(pen);
				painter->setBrush(Qt::NoBrush);
				painter->drawPolyline(barPolygon);
			}
		}
	}

	QCPColoredBarDataMap m_coloredData;
};

/*********************************/
/*** ccHistogramWindow stuffs  ***/
/*********************************/

ccHistogramWindow::ccHistogramWindow(QWidget* parent/*=0*/)
	: QCustomPlot(parent)
	, m_titlePlot(0)
	, m_colorScheme(USE_SOLID_COLOR)
	, m_solidColor(Qt::blue)
	, m_colorScale(ccColorScalesManager::GetDefaultScale())
	, m_associatedSF(0)
	, m_numberOfClassesCanBeChanged(false)
	, m_histogram(0)
	, m_minVal(0)
	, m_maxVal(0)
	, m_maxHistoVal(0)
	, m_vertBar(0)
	, m_drawVerticalIndicator(false)
	, m_verticalIndicatorPositionPercent(0)
{
	setWindowTitle("Histogram");
	setFocusPolicy(Qt::StrongFocus);

	setMinimumSize(300,200);

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
		m_associatedSF = 0;
	}

	m_histoValues.clear();
	m_maxHistoVal = 0;

	m_curveValues.clear();
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
								bool numberOfClassesCanBeChanged/*=true*/)
{
	if (m_associatedSF != sf)
	{
		if (m_associatedSF)
			m_associatedSF->release();
		m_associatedSF = sf;
	}

	if (m_associatedSF)
	{
		m_minVal = m_associatedSF->getMin();
		m_maxVal = m_associatedSF->getMax();
		m_numberOfClassesCanBeChanged = numberOfClassesCanBeChanged;
		m_associatedSF->link();
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
	catch(std::bad_alloc)
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
	catch(std::bad_alloc)
	{
		ccLog::Warning("[ccHistogramWindow::setCurveValues] Not enough memory!");
	}
}

bool ccHistogramWindow::computeBinArrayFromSF(size_t binCount)
{
	//clear any existing histogram
	m_histoValues.clear();

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

	//(try to) create new array
	try
	{
		m_histoValues.resize(binCount,0);
	}
	catch(std::bad_alloc)
	{
		ccLog::Warning("[ccHistogramWindow::computeBinArrayFromSF] Not enough memory!");
		return false;
	}

	double range = m_maxVal - m_minVal;
	if (range > 0.0)
	{
		unsigned count = m_associatedSF->currentSize();
		double step = range/static_cast<double>(binCount);
		for (unsigned i=0; i<count; ++i)
		{
			double val = static_cast<double>(m_associatedSF->getValue(i));

			//we ignore values outside of [m_minVal,m_maxVal] (works fro NaN values as well)
			if (/*ccScalarField::ValidValue(val) &&*/val >= m_minVal && val <= m_maxVal)
			{
				size_t bin = static_cast<size_t>( floor((val-m_minVal) / step) );
				++m_histoValues[std::min(bin,binCount-1)];
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
	
	for (size_t i=0; i<m_histoValues.size(); ++i)
		m_maxHistoVal = std::max(m_maxHistoVal,m_histoValues[i]);

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

	if (!m_numberOfClassesCanBeChanged)
	{
		assert(false);
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

void ccHistogramWindow::refresh()
{
	// set ranges appropriate to show data
	xAxis->setRange(m_minVal, m_maxVal);
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
			m_titlePlot = 0;
		}
		m_titlePlot = new QCPPlotTitle(this, QString("%0 [%1 classes]").arg(m_titleStr).arg(m_histoValues.size()));
		//title font
		m_renderingFont.setPointSize(ccGui::Parameters().defaultFontSize);
		m_titlePlot->setFont(m_renderingFont);
		plotLayout()->addElement(0, 0, m_titlePlot);
	}

	//clear previous display
	m_histogram = 0;
	m_vertBar = 0;
	this->clearGraphs();
	this->clearPlottables();

	if (m_histoValues.empty())
		return;

	//histogram
	int histoSize = static_cast<int>(m_histoValues.size());
	double totalSum = 0;
	double partialSum = 0;
	if (histoSize > 0)
	{
		m_histogram = new QCPColoredBars(xAxis, yAxis);
		addPlottable(m_histogram);
		// now we can modify properties of myBars:
		m_histogram->setName("Histogram");
		m_histogram->setWidth((m_maxVal - m_minVal) / histoSize);
		m_histogram->setAntialiasedFill(false);
		QVector<double> keyData(histoSize);
		QVector<double> valueData(histoSize);

		HISTOGRAM_COLOR_SCHEME colorScheme = m_colorScheme;
		ccColorScale::Shared colorScale = (m_colorScale ? m_colorScale : ccColorScalesManager::GetDefaultScale());
		switch(colorScheme)
		{
		case USE_SOLID_COLOR:
			m_histogram->setBrush(QBrush(m_solidColor,Qt::SolidPattern));
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

		for (int i=0; i<histoSize; ++i)
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
				const colorType* col = 0;
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
					col = ccColor::lightGrey;
				colors[i] = QColor(col[0],col[1],col[2]);
			}
		}

		if (!colors.isEmpty())
			m_histogram->setData(keyData, valueData, colors);
		else
			m_histogram->setData(keyData, valueData);
	}

	//overlay curve?
	int curveSize = static_cast<int>(m_curveValues.size());
	if (curveSize > 1)
	{
		QVector<double> x(curveSize), y(curveSize);
		
		double step = (m_maxVal - m_minVal) / (curveSize-1);
		for (int i=0; i<curveSize; ++i)
		{
			x[i] = m_minVal + (static_cast<double>(i)/*+0.5*/) * step;
			y[i] = m_curveValues[i];
		}

		// create graph and assign data to it:
		addGraph();
		graph(0)->setData(x, y);

		//default color
		const unsigned char* col = ccColor::black;
		QPen pen(QColor(col[0],col[1],col[2]));
		pen.setWidth(std::max(1,rect().width()/200));
		graph(0)->setPen(pen);
	}

	//vertical hint
	if (m_drawVerticalIndicator)
	{
		m_vertBar = new QCPBarsWithText(xAxis, yAxis);
		addPlottable(m_vertBar);
		
		// now we can modify properties of vertBar
		m_vertBar->setName("Vertical line");
		m_vertBar->setWidth(0/*(m_maxVal - m_minVal) / histoSize*/);
		m_vertBar->setBrush(QBrush(Qt::red));
		m_vertBar->setPen(QPen(Qt::red));
		m_vertBar->setAntialiasedFill(false);
		QVector<double> keyData(1);
		QVector<double> valueData(1);

		//horizontal position
		keyData[0] = m_minVal + (m_maxVal-m_minVal) * m_verticalIndicatorPositionPercent;
		valueData[0] = m_maxHistoVal;

		m_vertBar->setData(keyData,valueData);

		//precision (same as color scale)
		int precision = static_cast<int>(ccGui::Parameters().displayedNumPrecision);
		unsigned bin = static_cast<unsigned>(m_verticalIndicatorPositionPercent * m_histoValues.size());
		QString valueStr = QString("bin %0").arg(bin);
		m_vertBar->setText(valueStr);
		valueStr = QString("< %0 %").arg(100.0*static_cast<double>(partialSum)/static_cast<double>(totalSum),0,'f',3);
		m_vertBar->appendText(valueStr);
		valueStr = QString("val = %0").arg(m_minVal+(m_maxVal-m_minVal)*m_verticalIndicatorPositionPercent,0,'f',precision);
		m_vertBar->appendText(valueStr);
		m_vertBar->setTextAlignment(m_verticalIndicatorPositionPercent > 0.5);
	}

	rescaleAxes();

	// redraw
	replot();
}

void ccHistogramWindow::resizeEvent(QResizeEvent * event)
{
	QCustomPlot::resizeEvent(event);
	
	if (graph(0))
	{
		QPen pen = graph(0)->pen();
		pen.setWidth(std::max(1,event->size().width()/300));
		graph(0)->setPen(pen);
	}

	refresh();
}

void ccHistogramWindow::mousePressEvent(QMouseEvent *event)
{
	mouseMoveEvent(event);
}

void ccHistogramWindow::mouseMoveEvent(QMouseEvent *event)
{
	if (event->buttons() & Qt::LeftButton)
	{
		if (m_histogram && !m_histoValues.empty())
		{
			QRect roi = m_histogram->rect();
			if (roi.contains(event->pos(),false))
			{
				m_drawVerticalIndicator = true;

				int verticalIndicatorPosition = (static_cast<int>(m_histoValues.size()) * (event->x() - roi.x())) / roi.width();
				m_verticalIndicatorPositionPercent = static_cast<double>(verticalIndicatorPosition) / m_histoValues.size();

				refresh();
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
			setNumberOfClasses(std::max<size_t>(4,m_histoValues.size()-4));
			refresh();
		}
	}
	else //if (e->delta() > 0)
	{
		setNumberOfClasses(m_histoValues.size()+4);
		refresh();
	}

	e->accept();
}

ccHistogramWindowDlg::ccHistogramWindowDlg(QWidget* parent/*=0*/)
	: QDialog(parent)
	, m_win(new ccHistogramWindow(this))
{
	QHBoxLayout* hboxLayout = new QHBoxLayout(this);
	hboxLayout->addWidget(m_win);
	hboxLayout->setContentsMargins(0,0,0,0);

	resize(400,275);
}
