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

#ifndef CC_HISTOGRAM_WINDOW_HEADER
#define CC_HISTOGRAM_WINDOW_HEADER

//Always first
#include <ccIncludeGL.h>

//Qt
#include <QDialog>
#include <QFont>

//qCC_db
#include <ccScalarField.h>

//QCustomPlot
#include <qcustomplot.h>

class QCPColoredBars;
class QCPBarsWithText;
class QCPHiddenArea;
class QCPArrow;
class Ui_HistogramDialog;

//! Histogram widget
class ccHistogramWindow : public QCustomPlot
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccHistogramWindow(QWidget *parent = 0);

	//! Destructor
	virtual ~ccHistogramWindow();

	//! Sets title
	void setTitle(const QString& str);
	//! Sets axis labels
	void setAxisLabels(const QString& xLabel, const QString& yLabel);

	//! Computes histogram from a scalar field
	/** Number of classes can be freely modified afterwards (if enabled).
		\param sf associated scalar field
		\param initialNumberOfClasses initial number of classes
		\param numberOfClassesCanBeChanged whether to allow the user to modify the number of classes
		\param showNaNValuesInGrey show NaN values (in gray)
		\return success
	**/
	void fromSF(ccScalarField* sf,
				unsigned initialNumberOfClasses = 0,
				bool numberOfClassesCanBeChanged = true,
				bool showNaNValuesInGrey = true);

	//! Creates histogram from a bin array (each bin = number of elements per class)
	/** Number of classes can't be modified.
		\param histoValues array of bins
		\param minVal minimum value
		\param maxVal maximum value
		\return success
	**/
	void fromBinArray(	const std::vector<unsigned>& histoValues,
						double minVal,
						double maxVal);

	//! Sets overlay curve values
	/** The curve will only appear over an histogram
		\param curveValues curve points 'Y' coordinates only (regularly sampled between the min and max histogram values)
	**/
	void setCurveValues(const std::vector<double>& curveValues);

	enum HISTOGRAM_COLOR_SCHEME { USE_SOLID_COLOR, USE_CUSTOM_COLOR_SCALE, USE_SF_SCALE };
	//! Sets how the gradient bars should be colored
	void setColorScheme(HISTOGRAM_COLOR_SCHEME scheme) { m_colorScheme = scheme; }

	//! Sets solid color
	/** Only used if color scheme is set to USE_SOLID_COLOR. **/
	void setSolidColor(QColor color) { m_solidColor = color; }
	//! Sets gradient color scale
	/** Only used if color scheme is set to USE_CUSTOM_COLOR_SCALE. **/
	void setColorScale(ccColorScale::Shared scale) { m_colorScale = scale; }
	
	//! Clears the display
	void clear();
	//! Updates the display
	void refresh();
	//! Updates the histogram bars only
	/** Only works if a SF is associated and color scheme is USE_SF_SCALE.
	**/
	void refreshBars();

	//! Returns the current histogram bins
	inline const std::vector<unsigned>& histoValues() const { return m_histoValues; }
	//! Returns the current histogram min value
	inline double minVal() const { return m_minVal; }
	//! Returns the current histogram max value
	inline double maxVal() const { return m_maxVal; }

public: //SF interactor mode

	//! Enables SF interaction mode
	void enableSFInteractionMode(bool state) { m_sfInteractionMode = state; }

	void setMinDispValue(double);
	void setMaxDispValue(double);
	void setMinSatValue(double);
	void setMaxSatValue(double);

signals:

	void sfMinDispValChanged(double);
	void sfMaxDispValChanged(double);
	void sfMinSatValChanged(double);
	void sfMaxSatValChanged(double);

protected: //methods

	//! Changes the current number of classes
	/** Warning: n should be a multiple of 4.
	**/
	void setNumberOfClasses(size_t n);

	//mouse events handling
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent* event);
	void resizeEvent(QResizeEvent * event);
	
	//! Returns current maximum bin size
	unsigned getMaxHistoVal();

	//! Clears internal structures
	void clearInternal();

	//! Dynamically computes histogram bins from scalar field
	bool computeBinArrayFromSF(size_t binCount);

	//! Updates overlay curve width depending on the widget display size
	void updateOverlayCurveWidth(int w, int h);

protected: //attributes

	//Title
	QString m_titleStr;
	QCPPlotTitle* m_titlePlot;

	//! Color scheme
	HISTOGRAM_COLOR_SCHEME m_colorScheme;
	//! Solid color
	QColor m_solidColor;
	//! Gradient color scale
	ccColorScale::Shared m_colorScale;

	//! Associated scalar field
	ccScalarField* m_associatedSF;
	//Whether the number of classes can be changed or not
	/** Only possible with an associated scalar field.
	**/
	bool m_numberOfClassesCanBeChanged;

	//histogram data
	QCPColoredBars* m_histogram;
	std::vector<unsigned> m_histoValues;
	double m_minVal;
	double m_maxVal;
	unsigned m_maxHistoVal;

	//! Overlay curve
	QCPGraph* m_overlayCurve;
	std::vector<double> m_curveValues;

	//vertical indicator
	QCPBarsWithText* m_vertBar;
	bool m_drawVerticalIndicator;
	double m_verticalIndicatorPositionPercent;

	//! Rendering font
	QFont m_renderingFont;

protected: //SF interactor mode

	//! Whether SF interaction mode is enabled or not
	bool m_sfInteractionMode;

	//! Selectable items in "SF interaction" mode
	enum SELECTABLE_ITEMS { NONE, LEFT_AREA, RIGHT_AREA, BOTH_AREAS, LEFT_ARROW, RIGHT_ARROW, BOTH_ARROWS };
	//! Currently selected item
	SELECTABLE_ITEMS m_selectedItem;

	//! Left greyed area
	QCPHiddenArea* m_areaLeft;
	//! Right greyed area
	QCPHiddenArea* m_areaRight;

	//! Left arrow
	QCPArrow* m_arrowLeft;
	//! Right arrow
	QCPArrow* m_arrowRight;

	//! Last mouse click
	QPoint m_lastMouseClick;
};

//! Encapsulating dialog for ccHistogramWindow
class ccHistogramWindowDlg : public QDialog
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccHistogramWindowDlg(QWidget* parent = 0);
	//! Destructor
	virtual ~ccHistogramWindowDlg();

	//! Returns encapsulated ccHistogramWindow
	inline ccHistogramWindow* window() { return m_win; }

	//! Exports histogram to a CSV file
	bool exportToCSV(QString filename) const;

protected slots:

	//! When the export to CSV file button is pressed
	void onExportToCSV();

	//! When the export to Image file button is pressed
	void onExportToImage();

protected:

	//Associated histogram window
	ccHistogramWindow* m_win;

	//! Associated widgets
	Ui_HistogramDialog* m_gui;
};

#endif
