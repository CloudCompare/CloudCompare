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

#include <ui_saveIndividualFilesDlg.h>

//! Dialog to save several entities as individual files (one file per entity)
class ccSaveIndividualFilesDlg : public QDialog
    , public Ui::SaveIndividualFilesDlg
{
	Q_OBJECT

  public:
	//! How the DB tree hierarchy is reproduced as sub-folders
	enum class FolderStructure
	{
		Flat          = 0, //!< all files are saved in the output folder
		FromSelection = 1, //!< sub-folders start at the selected entities
		FullTree      = 2, //!< sub-folders start at the DB tree root
	};

	//! Default constructor
	/** \param cloudCount number of clouds to save
	    \param meshCount number of meshes to save
	    \param polylineCount number of polylines to save
	    \param parent parent widget
	**/
	ccSaveIndividualFilesDlg(unsigned cloudCount, unsigned meshCount, unsigned polylineCount, QWidget* parent = nullptr);

	//! Returns the output folder
	QString outputFolder() const;

	//! Returns the selected 'file filter' for clouds
	QString cloudFilter() const;
	//! Returns the selected 'file filter' for meshes
	QString meshFilter() const;
	//! Returns the selected 'file filter' for polylines
	QString polylineFilter() const;

	//! Returns how the DB tree hierarchy should be reproduced as sub-folders
	FolderStructure folderStructure() const;

	//! Returns whether existing files should be overwritten
	bool overwriteExisting() const;

	//! Returns whether the I/O filters dialogs should be displayed for each file
	bool showFilterDialogs() const;

	//! Saves the current parameters in the persistent settings
	void saveSettings() const;

  protected:
	//! Asks the user for the output folder
	void browseOutputFolder();

	//! Enables the 'Ok' button only if the parameters are valid
	void updateOkButton();
};
