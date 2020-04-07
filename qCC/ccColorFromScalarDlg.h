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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#ifndef CC_FROM_SCALAR_DLG_HEADER
#define CC_FROM_SCALAR_DLG_HEADER

//Qt
#include <QColor>

//CC
#include <ccColorScale.h>
#include <ccHistogramWindow.h>
#include <ccScalarField.h>

class ccPointCloud;

namespace Ui {
	class ColorFromScalarDialog;
}

//! Dialog to change the color levels
class ccColorFromScalarDlg : public QDialog
{
	Q_OBJECT

public:
	//! Default constructor
	ccColorFromScalarDlg(QWidget* parent, ccPointCloud* pointCloud);
	
	~ccColorFromScalarDlg();

	//update and redraw histograms
	void updateHistogram(int);

protected:
	//events to set scalar fields
	void onChannelChangedR(int) { updateChannel(0); }
	void onChannelChangedG(int) { updateChannel(1); }
	void onChannelChangedB(int) { updateChannel(2); }
	void onChannelChangedA(int) { updateChannel(3); }
	void updateChannel(int);
	void updateColormaps();
	//mapping ranges changed
	void minChangedR(double val) { minChanged(0, val, true); }
	void maxChangedR(double val) { maxChanged(0, val, true); }
	void minChangedG(double val) { minChanged(1, val, true); }
	void maxChangedG(double val) { maxChanged(1, val, true); }
	void minChangedB(double val) { minChanged(2, val, true); }
	void maxChangedB(double val) { maxChanged(2, val, true); }
	void minChangedA(double val) { minChanged(3, val, true); }
	void maxChangedA(double val) { maxChanged(3, val, true); }
	void minSpinChangedR(double val) { minChanged(0, val, false); }
	void maxSpinChangedR(double val) { maxChanged(0, val, false); }
	void minSpinChangedG(double val) { minChanged(1, val, false); }
	void maxSpinChangedG(double val) { maxChanged(1, val, false); }
	void minSpinChangedB(double val) { minChanged(2, val, false); }
	void maxSpinChangedB(double val) { maxChanged(2, val, false); }
	void minSpinChangedA(double val) { minChanged(3, val, false); }
	void maxSpinChangedA(double val) { maxChanged(3, val, false); }
	void toggleFixedR() { updateChannel(0);}
	void toggleFixedG() { updateChannel(1);}
	void toggleFixedB() { updateChannel(2);}
	void toggleFixedA() { updateChannel(3);}
	void toggleColors(int val);
	void toggleColorMode(bool state);
	void minChanged(int n, double val, bool slider);
	void maxChanged(int n, double val, bool slider);
	//done!
	void onApply();

protected:
	//! Associated histogram view
	ccHistogramWindow *m_histograms[4];  //0 - red, 1 - green, 2 - blue, 3 - alpha
	//scalar fields
	ccScalarField *m_scalars[4]; //0 - red, 1 - green, 2 - blue, 3 - alpha
	//gui elements
	QComboBox* m_combos[4];
	QDoubleSpinBox* m_boxes_min[4];
	QDoubleSpinBox* m_boxes_max[4];
	QLabel* m_labels_min[4];
	QLabel* m_labels_max[4];
	QCheckBox* m_reverse[4];
	//saturation values
	double m_minSat[4];
	double m_maxSat[4];
	//! Associated colour scales
	ccColorScale::Shared m_colors[4];
	//! Associated point cloud (color source)
	ccPointCloud* m_cloud;

private:
	Ui::ColorFromScalarDialog *m_ui;
};

#endif //CC_COLOR_LEVELS_DLG_HEADER
