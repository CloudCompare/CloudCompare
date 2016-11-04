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

#ifndef CC_APPLY_TRANSFORMATION_DLG_HEADER
#define CC_APPLY_TRANSFORMATION_DLG_HEADER

#include <ui_applyTransformationDlg.h>

//qCC_db
#include <ccGLMatrix.h>

//! Dialog to input a 4x4 matrix
class ccApplyTransformationDlg : public QDialog, public Ui::ApplyTransformationDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccApplyTransformationDlg(QWidget* parent = 0);

	//! Returns input matrix
	ccGLMatrixd getTransformation() const;

protected slots:

	//! Checks matrix validity and 'accept' dialog if ok
	void checkMatrixValidityAndAccept();

	//! Automatically removes anything between square brackets, and update the other forms
	void onMatrixTextChange();

	//! Updates dialog when a component of the rotation axis/angle form changes
	void onRotAngleValueChanged(double);
	//! Updates dialog when a component of the euleur form changes
	void onEulerValueChanged(double);

	//! Loads matrix from ASCII file
	void loadFromASCIIFile();
	//! Loads matrix from clipboard ("paste")
	void loadFromClipboard();
	//! Inits matrix from dip / dip direction values
	void initFromDipAndDipDir();

	//! Signal called when a button is clicked
	void buttonClicked(QAbstractButton*);

protected:

	//! Updates all forms with a given matrix
	void updateAll(const ccGLMatrix& mat, bool textForm = true, bool axisAngleForm = true, bool eulerForm = true);
};

#endif //CC_APPLY_TRANSFORMATION_DLG_HEADER
