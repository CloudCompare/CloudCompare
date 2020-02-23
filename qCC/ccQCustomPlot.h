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

#ifndef CC_QCUSTOMPLOT_HEADER
#define CC_QCUSTOMPLOT_HEADER

//QCustomPlot
#include <qcustomplot.h>

//System
#include <assert.h>

/*********************************/
/*** Custom QCustomPlot wigets ***/
/*********************************/

//! QCustomPlot: vertical bar with text along side
class QCPBarsWithText : public QCPBars
{
	Q_OBJECT

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

//! QCustomPlot: colored histogram
class QCPColoredBars : public QCPBars
{
	Q_OBJECT

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

	// reimplemented virtual methods:
	virtual void clearData() { QCPBars::clearData(); m_coloredData.clear(); }

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

//! QCustomPlot: selectable cursor interface
class QCPSelectableCursor : public QCPAbstractPlottable
{
	Q_OBJECT

public:
	
	//! Default constructor
	explicit QCPSelectableCursor(QCPAxis *keyAxis, QCPAxis *valueAxis)
		: QCPAbstractPlottable(keyAxis, valueAxis)
		, mCurrentVal(0)
		, mMinVal(0)
		, mMaxVal(0)
		, mLastPos(-1,-1)
		, mLastRadius(0)
	{}

	//! Returns whether the item is "selectable" when the mouse is clicked at a given position
	inline virtual bool isSelectable(QPoint click) const
	{
		if (mLastPos.x() < 0 || mLastPos.y() < 0)
			return false;
		QPoint d = mLastPos - click;
		return (d.x()*d.x() + d.y()*d.y() <= mLastRadius*mLastRadius);
	}

	// getters
	inline double currentVal() const { return mCurrentVal; }
	inline double minVal() const { return mMinVal; }
	inline double maxVal() const { return mMaxVal; }
	inline void range(double& minVal, double& maxVal) const { minVal = mMinVal; maxVal = mMaxVal; }

	// setters
	inline void setCurrentVal(double val) { mCurrentVal = std::max(std::min(val,mMaxVal),mMinVal); }
	inline void setRange(double minVal, double maxVal) { mMinVal = minVal; mMaxVal = maxVal; }

	//! Converts a pixel value (X) to the equivalent key
	inline double pixelToKey(int pixX) const { return keyAxis() ? keyAxis()->pixelToCoord(pixX) : 0; }
	//! Converts a pixel value (Y) to the equivalent value
	inline double pixelToValue(int pixY) const { return valueAxis() ? valueAxis()->pixelToCoord(pixY) : 0; }


	// reimplemented virtual methods:
	virtual void clearData() {}
	virtual double selectTest(const QPointF &pos, bool onlySelectable, QVariant *details=0) const { return -1; } //we don't use the QCP internal selection mechanism!

protected:
	// reimplemented virtual methods:
	virtual void drawLegendIcon(QCPPainter *painter, const QRectF &rect) const {}
	virtual QCPRange getKeyRange(bool &foundRange, SignDomain inSignDomain=sdBoth) const { foundRange = false; return QCPRange(); }
	virtual QCPRange getValueRange(bool &foundRange, SignDomain inSignDomain=sdBoth) const { foundRange = false; return QCPRange(); }

	// property members:
	double mCurrentVal;
	double mMinVal,mMaxVal;
	QPoint mLastPos;
	int mLastRadius;
};

//! QCustomPlot: greyed areas
class QCPHiddenArea : public QCPSelectableCursor
{
	Q_OBJECT

public:
	explicit QCPHiddenArea(bool leftSide, QCPAxis *keyAxis, QCPAxis *valueAxis)
		: QCPSelectableCursor(keyAxis, valueAxis)
		, mLeftSide(leftSide)
	{
		mPen = QPen(QColor(80, 80, 80),Qt::SolidLine); // dark grey
		mPen.setWidth(2);
		mSelectedPen = mPen;
		
		mBrush = QBrush(Qt::white,Qt::SolidPattern); // white
		mSelectedBrush = mBrush;
	}

protected:

	// reimplemented virtual methods:
	virtual void draw(QCPPainter *painter)
	{
		if (!keyAxis())
			return;

		QRect rect = clipRect();

		double currentPosd = keyAxis()->coordToPixel(mCurrentVal);
		if (mLeftSide)
		{
			int x2 = static_cast<int>(ceil(currentPosd));
			//assert(x2 >= rect.x());
			if (x2 < rect.x())
				return;
			rect.setWidth(x2 - rect.x());
		}
		else
		{
			int x1 = static_cast<int>(floor(currentPosd));
			//assert(x1 >= rect.x());
			if (x1 < rect.x())
				return;
			int newWidth = rect.width() - (x1 - rect.x());
			rect.setX(x1);
			rect.setWidth(newWidth);
		}

		// draw greyed rect
		if (	(mLeftSide && mCurrentVal > mMinVal)
			||	(!mLeftSide && mCurrentVal < mMaxVal) )
		{
			applyFillAntialiasingHint(painter);
			painter->setPen(Qt::NoPen);
			painter->setBrush(QBrush(QColor(128, 128, 128, 128),Qt::SolidPattern)); // semi-transparent grey
			painter->drawPolygon(rect);
		}

		//draw circle (handle)
		if (mainPen().style() != Qt::NoPen && mainPen().color().alpha() != 0)
		{
			//circle
			QPoint C(mLeftSide ? rect.x()+rect.width() : rect.x(), rect.y()+rect.height()/2);
			int r = rect.height() / 10;

			painter->setPen(mainPen());
			painter->setBrush(mainBrush());
			painter->drawEllipse(C,r,r);

			painter->setPen(QPen(QColor(128, 128, 128, 128),Qt::SolidLine)); // semi-transparent grey
			painter->drawLine(C+QPoint(0,r),C-QPoint(0,r));

			//save last circle position
			mLastPos = C;
			mLastRadius = r;
		}
		else
		{
			//no circle
			mLastPos = QPoint(-1,-1);
			mLastRadius = 0;
		}
	}

	//! Whether the cursor is displayed on the left side or not
	bool mLeftSide;
};

//! QCustomPlot: small arrows at the bottom
class QCPArrow : public QCPSelectableCursor
{
	Q_OBJECT

public:
	explicit QCPArrow(QCPAxis *keyAxis, QCPAxis *valueAxis)
		: QCPSelectableCursor(keyAxis, valueAxis)
	{
		mPen.setColor(QColor(128, 128, 0)); // dark yellow
		mPen.setStyle(Qt::SolidLine);
		mPen.setWidth(2);
		mSelectedPen = mPen;
		
		mBrush.setColor(QColor(255, 255, 0, 196)); // semi-transparent yellow
		mBrush.setStyle(Qt::SolidPattern);
		mSelectedBrush = mBrush;
	}

	//! Sets triangle 'inside' color
	void setColor(int r, int g, int b)
	{
		mBrush.setColor(QColor(r, g, b, 196)); // semi-transparent color
		mSelectedBrush = mBrush;
	}
	
protected:

	// reimplemented virtual methods:
	virtual void draw(QCPPainter *painter)
	{
		if (!keyAxis())
			return;

		QRect rect = clipRect();

		int currentPos = static_cast<int>(keyAxis()->coordToPixel(mCurrentVal));

		int r = rect.height() / 10;

		// draw dashed line
		{
			QPen pen(QColor(128, 128, 128, 128),Qt::DashLine);
			pen.setWidth(1);
			painter->setPen(pen); // semi-transparent grey
			painter->drawLine(QPoint(currentPos,rect.y()+2*r), QPoint(currentPos,rect.y()+rect.height()));
		}

		//draw triangle(handle)
		if (mainPen().style() != Qt::NoPen && mainPen().color().alpha() != 0)
		{
			//QPoint O(currentPos,rect.y() + rect.height() - r);
			//QPoint T[3] = { O - QPoint(0,r), O + QPoint(r,r), O + QPoint(-r,r) };

			QPoint O(currentPos,rect.y() + r);
			QPoint T[3] = { O + QPoint(0,r), O - QPoint(r,r), O - QPoint(-r,r) };

			painter->setPen(mainPen());
			painter->setBrush(mainBrush());
			painter->drawPolygon(T,3);

			//save last circle position
			mLastPos = O;
			mLastRadius = r;
		}
		else
		{
			//no circle
			mLastPos = QPoint(-1,-1);
			mLastRadius = 0;
		}
	}
};


#endif //CC_QCUSTOMPLOT_HEADER
