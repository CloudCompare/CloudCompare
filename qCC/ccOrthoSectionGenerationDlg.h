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

#ifndef CC_ORTHO_SECTION_GENERATION_DIALOG_HEADER
#define CC_ORTHO_SECTION_GENERATION_DIALOG_HEADER

//Qt
#include <QDialog>

#include <ui_orthoSectionGenerationDlg.h>

//! Dialog for generating orthogonal sections along a path (Section Extraction Tool)
class ccOrthoSectionGenerationDlg : public QDialog, public Ui::OrthoSectionGenerationDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccOrthoSectionGenerationDlg(QWidget* parent = 0);

	//! Sets the path legnth
	void setPathLength(double l);

	//! Sets whether the generatrix should be automatically saved and removed
	void setAutoSaveAndRemove(bool state);
	//! Returns whether the generatrix should be automatically saved and removed
	bool autoSaveAndRemove() const;

	//! Sets the generation step
	void setGenerationStep(double s);
	//! Sets he sections width
	void setSectionsWidth(double w);

	//! Returns the generation step
	double getGenerationStep() const;
	//! Returns the sections width
	double getSectionsWidth() const;

protected slots:
	void onStepChanged(double);

protected:

	//! Path length
	double m_pathLength;

};

#endif // CC_ORTHO_SECTION_GENERATION_DIALOG_HEADER
