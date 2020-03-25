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

#ifndef CC_FROM_SCALAR_DLG_HEADER
#define CC_FROM_SCALAR_DLG_HEADER

//Qt
#include <QColor>

#include <ui_colorFromScalarDlg.h>
#include <ccColorScale.h>
#include <ccScalarField.h>

class ccHistogramWindow;
class ccPointCloud;

//! Dialog to change the color levels
class ccColorFromScalarDlg : public QDialog, public Ui::ColorFromScalarDialog
{
	Q_OBJECT

public:

	//! Default constructor
	ccColorFromScalarDlg(QWidget* parent, ccPointCloud* pointCloud);

	//update and redraw histograms
	void updateHistogram(int);
protected slots:
	
	//events to set scalar fields
	void onChannelChangedR(int);
	void onChannelChangedG(int);
	void onChannelChangedB(int);
	void updateChannel(int);

	//mapping ranges changed
	void minChanged(int n, double val)
	{
		m_minSat[n] = val;
		m_boxes_min[n]->setValue(val);
	}

	void maxChanged(int n, double val)
	{
		m_maxSat[n] = val;
		m_boxes_max[n]->setValue(val);
	}
	void minChangedR(double val) { minChanged(0,val); }
	void maxChangedR(double val) { maxChanged(0,val); }
	void minChangedG(double val) { minChanged(1, val); }
	void maxChangedG(double val) { maxChanged(1, val); }
	void minChangedB(double val) { minChanged(2, val); }
	void maxChangedB(double val) { maxChanged(2, val); }

	//done!
	void onApply();

protected:

	//! Associated histogram view
	ccHistogramWindow *m_histograms[3];  //0 - red, 1 - green, 2 - blue

	 //scalar fields
	ccScalarField *m_scalars[3]; //0 - red, 1 - green, 2 - blue

	//gui elements
	QComboBox* m_combos[3];
	QDoubleSpinBox* m_boxes_min[3];
	QDoubleSpinBox* m_boxes_max[3];

	//saturation values
	double m_minSat[3];
	double m_maxSat[3];

	//! Associated colour scales
	ccColorScale::Shared m_colors[3];
	ccColorScale::Shared m_red;
	ccColorScale::Shared m_green;
	ccColorScale::Shared m_blue;
	ccColorScale::Shared m_hue;
	ccColorScale::Shared m_sat;
	ccColorScale::Shared m_val;

	//! Associated point cloud (color source)
	ccPointCloud* m_cloud;


};

#endif //CC_COLOR_LEVELS_DLG_HEADER
