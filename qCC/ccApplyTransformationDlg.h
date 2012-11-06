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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

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
	ccApplyTransformationDlg(QWidget* parent=0);

	//! Returns input matrix
	ccGLMatrix getTransformation() const;

protected slots:

	//! Check matrix validity and 'accept' dialog if ok
	void checkMatrixValidityAndAccept();

protected:

	//! Check matrix validity
	/** \return true if input matrix is valid (false otherwise)
	**/
	bool checkMatrixValidity() const;

};

#endif
