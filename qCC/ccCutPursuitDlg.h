#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include <ui_cutPursuitDlg.h>

#include <QStringList>

//! Dialog to define Cut-Pursuit parameters
class ccCutPursuitDlg : public QDialog
    , public Ui::CutPursuitDialog
{
	Q_OBJECT

  public:
	//! Default constructor
	explicit ccCutPursuitDlg(QWidget* parent = nullptr);

	//! Populates the scalar fields list with checkable entries
	/** \param sfNames names of the available scalar fields (the Cut Pursuit label field should already be excluded)
	    \param includeRGB whether an "RGB" entry should be added at the top of the list (only when all selected clouds have colors)
	**/
	void setScalarFields(const QStringList& sfNames, bool includeRGB = false);

	//! Returns the list of scalar field names that are checked (to be included in the Y matrix)
	/** The special "RGB" entry (if present) is excluded from this list; use useRGB() to check it.
	**/
	QStringList getSelectedScalarFields() const;

	//! Returns knn parameter
	int getKNN();

	//! Returns search radius parameter
	double getKNNRadius();
	
	//! Returns regularization parameter
	double getRegularization();

	//! Returns spatial weight factor
	double getSpatialWeight();

	//! Returns cutoff parameter
	int getCutoff();

	//! Returns whether the "RGB" feature entry is checked
	bool useRGB();

	//! Returns average colors parameter
	bool averageColors();

  protected:
	//! Saves the current dialog parameters to the persistent (application-wide) settings
	void saveToPersistentSettings() const;

	//! Restores the dialog parameters from the persistent (application-wide) settings
	void loadFromPersistentSettings();

	//! Overridden to save the parameters when the dialog is accepted
	void accept() override;
};
