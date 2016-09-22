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

#ifndef CC_MATRIX_DISPLAY_DIALOG_HEADER
#define CC_MATRIX_DISPLAY_DIALOG_HEADER

//Qt
#include <QWidget>

//qCC_db
#include <ccGLMatrix.h>

#include <ui_matrixDisplayDlg.h>

//! Simple widget to display a 4x4 matrix in various formats
class MatrixDisplayDlg : public QWidget, public Ui::MatrixDisplayDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit MatrixDisplayDlg(QWidget* parent = 0);

	//! Clears widget
	void clear();

	//! Updates dialog with a given (float) matrix
	void fillDialogWith(const ccGLMatrix& mat);
	//! Updates dialog with a given (double) matrix
	void fillDialogWith(const ccGLMatrixd& mat);

public slots:

	//! Exports current matrix to an ASCII file
	/** Will display a file selection dialog!
	**/
	void exportToASCII();
	//! Exports current matrix to the clipboard
	void exportToClipboard();

protected:

	//! Fills the second part of the dialog
	void fillDialogWith(const CCVector3d& axis, double angle_rad, const CCVector3d& T, int precision);

	//! Matrix
	ccGLMatrixd m_mat;

};

#endif //CC_MATRIX_DISPLAY_DIALOG_HEADER
