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

#ifndef CC_HISTOGRAM_WINDOW_HEADER
#define CC_HISTOGRAM_WINDOW_HEADER

//Qt
#include <QGLWidget>
#include <QDialog>
#include <QHBoxLayout>
#include <QFont>
#include <QString>

//qCC_db
#include <ccScalarField.h>

//! Histogram widget
class ccHistogramWindow : public QGLWidget
{
	Q_OBJECT

public:

	//! Default constructor
	ccHistogramWindow(QWidget *parent = 0);

	//! Destructor
	virtual ~ccHistogramWindow();

	//! Sets first line
	void setInfoStr(const QString& str);

	//! Computes histogram from a scalar field
	/** Number of classes can be freely modified afterwards (if enabled).
		\param associated scalar field
		\param initialNumberOfClasses initial number of classes
		\param numberOfClassesCanBeChanged whether to allow the user to modify the number of classes
		\return success
	**/
	void fromSF(ccScalarField* sf,
				unsigned initialNumberOfClasses = 0,
				bool numberOfClassesCanBeChanged = true);

	//! Creates histogram from a bin array (each bin = number of elements per class)
	/** Number of classes can't be modified.
		\param histoValues array of bins (number of points per class)
		\param numberOfClasses corresponding number of classes
		\param minVal minimum value
		\param maxVal maximum value
		\param giveArrayOwnership whether array ownership is passed to the dialog or not
		\return success
	**/
	void fromBinArray(unsigned* histoValues,
						unsigned numberOfClasses,
						double minVal,
						double maxVal,
						bool giveArrayOwnership = true);

	//! Sets overlay curve values
	/** Curve will only appear if the number of points matches the current number of classes)
		\param curveValues curve points 'Y' coordinates (points will be regularly spread over histogram span)
		\param numberOfCurvePoints number of points
		\param giveArrayOwnership whether array ownership is passed to the dialog or not
	**/
	void setCurveValues(double* curveValues,
						unsigned numberOfCurvePoints,
						bool giveArrayOwnership = true);

protected:

	//! Changes the current number of classes
	/** Warning: n should be a multiple of 4.
	**/
	void setNumberOfClasses(unsigned  n);

	//mouse events handling
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent* event);

	void closeEvent(QCloseEvent *event);

	//inherited from QGLWidget
	//void initializeGL();
	//void resizeGL(int w, int h);
	void paintGL();

	//! Returns current maximum bin size
	unsigned getMaxHistoVal();

	//! Clears internal structures
	void clear();

	//! Dynamically computes histogram bins from scalar field
	bool computeBinArrayFromSF();

	//! 1st line
	QString m_infoStr;
	bool m_viewInitialized;
	bool m_numberOfClassesCanBeChanged;

	//table and type
	ccScalarField* m_associatedSF;

	//histogram variables
	unsigned m_numberOfClasses;
	unsigned* m_histoValues;
	bool m_ownHistoValues;
	double m_minVal;
	double m_maxVal;
	unsigned m_maxHistoVal;

	//overlay curve
	double* m_curveValues;
	double m_maxCurveValue;
	unsigned m_numberOfCurvePoints;
	bool m_ownCurveValues;

	//histogram display area
	int m_roi[4];
	//classes number modification buttons ("+" and "-")
	int m_xMinusButton,m_yMinusButton,m_xPlusButton,m_yPlusButton;
	int m_buttonSize;

	//vertical indicator
	bool m_drawVerticalIndicator;
	double m_verticalIndicatorPositionPercent;

	//rendering font
	QFont m_renderingFont;
};

//! Encapsulating dialog for ccHistogramWindow
class ccHistogramWindowDlg : public QDialog
{
public:
	//! Default constructor
	ccHistogramWindowDlg(QWidget* parent = 0)
		: QDialog(parent)
		, m_win(new ccHistogramWindow(this))
	{
		QHBoxLayout* hboxLayout = new QHBoxLayout(this);
		hboxLayout->addWidget(m_win);
		hboxLayout->setContentsMargins(0,0,0,0);
	}

	//! Returns encapsulated ccHistogramWindow
	ccHistogramWindow* window() { return m_win; };

protected:

	//Associated histogram window
	ccHistogramWindow* m_win;
};

#endif
