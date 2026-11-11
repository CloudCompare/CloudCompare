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

// Ui
#include <ui_openPlyFileDlg.h>

// Qt
#include <QStringList>

// System
#include <array>
#include <set>
#include <vector>

class QComboBox;
struct PlyLoadingContext;

//! Dialog for configuration of PLY files opening sequence
class PlyOpenDlg : public QDialog
    , public Ui::PlyOpenDlg
{
	Q_OBJECT

  public:
	//! Default constructor
	explicit PlyOpenDlg(QWidget* parent = nullptr);

	//! Sets the items of the standard comboboxes (coordinates, color components, etc.)
	void setDefaultComboItems(const QStringList& stdPropsText);
	//! Sets the items of the list-related comboboxes (faces, etc.)
	void setListComboItems(const QStringList& listPropsText);
	//! Sets the items of the single-related comboboxes (texture index, etc.)
	void setSingleComboItems(const QStringList& singlePropsText);

	//! Returns whether the current configuration is valid or not
	bool isValid(bool displayErrors = true) const;

	//! Returns whether the user chose to apply the current configuration to all subsequent files
	bool shouldApplyAll() const;

	//! Restores the previously saved configuration (if any)
	/** \param restoredProps list of restored properties (if any)
	    \param restoreSFs whether the SFs should be restored or not
	    \return whether the previous context can be restored or not
	**/
	bool restorePreviousContext(std::set<QString>& restoredProps, bool restoreSFs);

	//! Resets the "apply all" flag (if set)
	static void ResetApplyAll();

	//! Returns whether the dialog can be 'skipped'
	/** i.e. 'Apply all' button has been previously clicked
	    and the configuration is valid.
	**/
	bool canBeSkipped() const;

	void addSFComboBox(int selectedIndex = 0);
	void getSFPropIndexes(std::vector<int>& sfPropIndexes) const;

  protected:
	void apply();
	void applyAll();
	void addAllStdPropsAsSF();

  Q_SIGNALS:
	void fullyAccepted();

  protected:
	//! Saves current configuration (for internal use)
	void saveContext(PlyLoadingContext* context);

  protected:
	//! Standard comboboxes
	std::array<QComboBox*, 10> m_standardCombos;
	//! List-related comboboxes (faces, etc.)
	std::array<QComboBox*, 2> m_listCombos;
	//! Single-related comboboxes (texture index, etc.)
	std::array<QComboBox*, 1> m_singleCombos;
	//! SF comboboxes
	std::vector<QComboBox*> m_sfCombos;

	//! Standard comboboxes elements
	QStringList m_stdPropsText;
	//! List-related comboboxes elements
	QStringList m_listPropsText;
	//! Single-related comboboxes elements
	QStringList m_singlePropsText;
};
