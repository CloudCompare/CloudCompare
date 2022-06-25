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

#ifndef CC_SF_EDIT_DIALOG_HEADER
#define CC_SF_EDIT_DIALOG_HEADER

//Qt
#include <QWidget>


class ccScalarField;
class ccHistogramWindow;

namespace Ui {
	class SFEditDlg;
}

//! GUI scalar field interactor for properties list dialog
class sfEditDlg : public QWidget
{
	Q_OBJECT

public:

	//! Default constructor
	explicit sfEditDlg(QWidget* parent = nullptr);

	~sfEditDlg();
	
	//! Updates dialog with a given scalar field
	void fillDialogWith(ccScalarField* sf);

public:

	void minValSBChanged(double);
	void maxValSBChanged(double);
	void minSatSBChanged(double);
	void maxSatSBChanged(double);

	void minValHistoChanged(double);
	void maxValHistoChanged(double);
	void minSatHistoChanged(double);
	void maxSatHistoChanged(double);

	void nanInGrayChanged(bool);
	void alwaysShow0Changed(bool);
	void symmetricalScaleChanged(bool);
	void logScaleChanged(bool);

Q_SIGNALS:

	//! Signal emitted when the SF display parameters have changed
	void entitySFHasChanged();

protected:

	//conversionb between sliders (integer) and checkbox (double) values
	double dispSpin2slider(double val) const;
	double satSpin2slider(double val) const;
	double dispSlider2spin(int pos) const;
	double satSlider2spin(int pos) const;

	//! Associated scalar field
	ccScalarField* m_associatedSF;
	//! Associated scalar field histogram
	ccHistogramWindow* m_associatedSFHisto;
	
	Ui::SFEditDlg* m_ui;
};

#endif //CC_SF_EDIT_DIALOG_HEADER
