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
//#                    COPYRIGHT: Daniel Girardeau-Montaut                 #
//#                                                                        #
//##########################################################################

#ifndef CC_INTERPOLATION_DLG_HEADER
#define CC_INTERPOLATION_DLG_HEADER

#include <ui_interpolationDlg.h>

//qCC_db
#include <ccPointCloudInterpolator.h>

//! Dialog for generic interpolation algorithms
class ccInterpolationDlg : public QDialog, public Ui::InterpolationDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccInterpolationDlg(QWidget* parent = 0);

	ccPointCloudInterpolator::Parameters::Method getInterpolationMethod() const;
	void setInterpolationMethod(ccPointCloudInterpolator::Parameters::Method method);

	ccPointCloudInterpolator::Parameters::Algo getInterpolationAlgorithm() const;
	void setInterpolationAlgorithm(ccPointCloudInterpolator::Parameters::Algo algo);

protected slots:

	void onRadiusUpdated(double);
};

#endif //CC_INTERPOLATION_DLG_HEADER
