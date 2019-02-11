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

#ifndef CC_WAVEFORM_DIALOG_HEADER
#define CC_WAVEFORM_DIALOG_HEADER

//Local
#include "ccPickingListener.h"

//Qt
#include <QDialog>
#include <QFont>

//QCustomPlot
#include <qcustomplot.h>

class QCPBarsWithText;
class QCPHiddenArea;
class QCPGraph;
class QCPArrow;
class Ui_WaveDialog;
class ccPointCloud;
class ccPickingHub;

//! Waveform widget
class ccWaveWidget : public QCustomPlot
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccWaveWidget(QWidget *parent = nullptr);

	//! Destructor
	~ccWaveWidget() override;

	//! Sets title
	void setTitle(const QString& str);
	//! Sets axis labels
	void setAxisLabels(const QString& xLabel, const QString& yLabel);

	//! Computes the wave (curve) from a given point waveform
	void init(ccPointCloud* cloud, unsigned pointIndex, bool logScale, double maxValue = 0.0);

	//! Clears the display
	void clear();
	//! Updates the display
	void refresh();

protected: //methods

	//mouse events handling
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void resizeEvent(QResizeEvent * event) override;
	
	//! Clears internal structures
	void clearInternal();

	//! Updates overlay curve width depending on the widget display size
	void updateCurveWidth(int w, int h);

protected: //attributes

	//Title
	QString m_titleStr;
	QCPPlotTitle* m_titlePlot;

	//! Wave curve
	QCPGraph* m_curve;
	std::vector<double> m_curveValues;
	double m_dt;
	double m_minA, m_maxA;
	double m_echoPos;

	//vertical indicator
	QCPBarsWithText* m_vertBar;
	bool m_drawVerticalIndicator;
	double m_verticalIndicatorPositionPercent;

	//Peak marker
	QCPBarsWithText* m_peakBar;

	//! Rendering font
	QFont m_renderingFont;

	//! Last mouse click
	QPoint m_lastMouseClick;
};

//! Waveform dialog
class ccWaveDialog : public QDialog, public ccPickingListener
{
	Q_OBJECT

public:
	//! Default constructor
	explicit ccWaveDialog(ccPointCloud* cloud, ccPickingHub* pickingHub, QWidget* parent = nullptr);
	//! Destructor
	~ccWaveDialog() override;

	//! Returns the encapsulated widget
	inline ccWaveWidget* waveWidget() { return m_widget; }

	//inherited from ccPickingListener
	virtual void onItemPicked(const PickedItem& pi) override;

protected slots:

	void onPointIndexChanged(int);
	void updateCurrentWaveform();
	void onPointPickingButtonToggled(bool);
	void onExportWaveAsCSV();

protected: //members

	//! Associated point cloud
	ccPointCloud* m_cloud;

	//! Wave widget
	ccWaveWidget* m_widget;

	//! Picking hub
	ccPickingHub* m_pickingHub;

	//! GUI
	Ui_WaveDialog* m_gui;

	//! Maximum wave amplitude (for all points)
	double m_waveMax;
};

#endif //CC_WAVEFORM_DIALOG_HEADER
