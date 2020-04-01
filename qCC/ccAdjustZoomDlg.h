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

class ccGLWindow;

//! Dialog to set the current zoom of a 3D view (or equivalently the pixel size)
/** Orthographic mode only.
**/
class ccAdjustZoomDlg: public QDialog, public Ui::AdjustZoomDialog
{
	Q_OBJECT

public:

	ccAdjustZoomDlg(ccGLWindow* win, QWidget* parent = 0);
	virtual ~ccAdjustZoomDlg() = default;

	//! Returns requested zoom
	double getZoom() const;

protected slots:
	void onZoomChanged(double);
	void onPixelSizeChanged(double);
	void onPixelCountChanged(int);

protected:

	double m_basePixelSize;
};

#endif // CC_ADJUST_ZOOM_DIALOG_HEADER
