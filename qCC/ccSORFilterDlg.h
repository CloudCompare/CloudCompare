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

#ifndef CC_SOR_FILTER_DLG_HEADER
#define CC_SOR_FILTER_DLG_HEADER

#include <QDialog>

namespace Ui {
	class SorFilterDialog;
}

//! Dialog to choose which dimension(s) (X, Y or Z) should be exported as SF(s)
class ccSORFilterDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccSORFilterDlg(QWidget* parent = nullptr);
	
	~ccSORFilterDlg();
	
	int KNN() const;
	void setKNN( int knn );
	
	double nSigma() const;
	void setNSigma( double nSigma );
	
private:
	Ui::SorFilterDialog* m_ui;
};

#endif //CC_SOR_FILTER_DLG_HEADER
