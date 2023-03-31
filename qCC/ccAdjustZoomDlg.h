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

#ifndef CC_ADJUST_ZOOM_DIALOG_HEADER
#define CC_ADJUST_ZOOM_DIALOG_HEADER

#include <QDialog>

#include <ui_adjustZoomDlg.h>

class ccGLWindowInterface;

//! Dialog to set the current focal of a 3D view (or equivalently the pixel size)
/** Orthographic mode only.
**/
class ccAdjustZoomDlg: public QDialog, public Ui::AdjustZoomDialog
{
	Q_OBJECT

public:

	ccAdjustZoomDlg(ccGLWindowInterface* win, QWidget* parent = nullptr);
	virtual ~ccAdjustZoomDlg() = default;

	//! Returns requested focal distance
	double getFocalDistance() const;

protected Q_SLOTS:
	void onFocalChanged(double);
	void onPixelSizeChanged(double);
	void onPixelCountChanged(int);

protected:

	int m_windowWidth_pix;
	double m_distanceToWidthRatio;
};

#endif // CC_ADJUST_ZOOM_DIALOG_HEADER
