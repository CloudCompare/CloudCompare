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

#ifndef CC_SF_EDIT_DIALOG_HEADER
#define CC_SF_EDIT_DIALOG_HEADER

#include <QWidget>

#include <ui_sfEditDlg.h>

class ccScalarField;

//! GUI scalar field interactor for properties list dialog
class sfEditDlg : public QWidget, public Ui::SFEditDlg
{
    Q_OBJECT

public:

	//! Default constructor
    sfEditDlg(QWidget* parent=0);

    void fillDialogWith(ccScalarField* sf);

public slots:

    void minValSBChanged(double val);
    void maxValSBChanged(double val);
    void minSatSBChanged(double val);
    void maxSatSBChanged(double val);

	void dispValSLDChanged(int,int);
	void satValSLDChanged(int,int);

	void absSatChanged(bool);
	void logScaleChanged(bool);
	void boundariesLockChanged(bool);
	void nanInGrayChanged(bool);

signals:

    void entitySFHasChanged();

protected:

    double spin2slider_1(double val);
    double spin2slider_2(double val);
    double slider2spin_1(int val);
    double slider2spin_2(int val);

	ccScalarField* m_associatedSF;

    double m_step1,m_step2;
    double m_coef1,m_coef2;
    double m_lowBound,m_upBound;
    double m_satSpan;
};

#endif //CC_SF_EDIT_DIALOG_HEADER
