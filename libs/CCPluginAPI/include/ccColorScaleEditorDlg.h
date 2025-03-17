#pragma once
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

#include "CCPluginAPI.h"

//qCC_db
#include <ccColorScale.h>

//Qt
#include <QDialog>

class ccScalarField;
class ccColorScaleEditorWidget;
class ccColorScalesManager;
class ccMainAppInterface;

namespace Ui
{
	class ColorScaleEditorDlg;	
}

/**
 * \brief Dialog for creating, editing, and managing color scales
 * 
 * \details Provides a comprehensive interface for manipulating color scales used 
 * in data visualization and scalar field representation
 * 
 * Key features:
 * - Create and modify color scales interactively
 * - Support for relative and absolute color scaling modes
 * - Manage color steps, labels, and scale properties
 * - Integration with CloudCompare's color scales manager
 * 
 * Typical use cases:
 * - Customizing color mappings for scalar fields
 * - Creating gradient color schemes
 * - Exporting and importing color scales
 * 
 * \note Inherits from QDialog and provides advanced color scale editing capabilities
 * 
 * \example
 * \code
 * // Creating and using the color scale editor dialog
 * ccColorScalesManager* scaleManager = ccColorScalesManager::GetUniqueInstance();
 * ccMainAppInterface* mainApp = // get main application interface
 * 
 * // Open color scale editor
 * ccColorScaleEditorDialog* editor = new ccColorScaleEditorDialog(
 *     scaleManager,      // Color scales manager
 *     mainApp,           // Main application interface
 *     nullptr,           // Optional initial color scale
 *     parentWidget       // Parent widget
 * );
 * 
 * // Optional: Set associated scalar field for context
 * ccScalarField* sf = // get scalar field
 * editor->setAssociatedScalarField(sf);
 * 
 * // Show the dialog
 * if (editor->exec() == QDialog::Accepted)
 * {
 *     // Get the modified or newly created color scale
 *     ccColorScale::Shared modifiedScale = editor->getActiveScale();
 *     
 *     // Apply the color scale to a point cloud or other visualization
 *     if (modifiedScale)
 *     {
 *         pointCloud->setColorScale(modifiedScale);
 *     }
 * }
 * \endcode
 */
class CCPLUGIN_LIB_API ccColorScaleEditorDialog : public QDialog
{
	Q_OBJECT

public:
	/**
	 * \brief Constructor for color scale editor dialog
	 * 
	 * \param[in] manager Pointer to the color scales manager
	 * \param[in] mainApp Pointer to the main application interface
	 * \param[in] currentScale Optional initial color scale to edit
	 * \param[in] parent Optional parent widget
	 */
	ccColorScaleEditorDialog(	ccColorScalesManager* manager,
								ccMainAppInterface* mainApp,
								ccColorScale::Shared currentScale = ccColorScale::Shared(nullptr),
								QWidget* parent = nullptr);

	/**
	 * \brief Destructor
	 * 
	 * Ensures proper cleanup of dialog resources
	 */
	~ccColorScaleEditorDialog() override;

	/**
	 * \brief Set an associated scalar field for context
	 * 
	 * \param[in] sf Pointer to the scalar field to associate with the color scale
	 * 
	 * \note Optional method to provide additional context for color scale editing
	 */
	void setAssociatedScalarField(ccScalarField* sf);

	/**
	 * \brief Set the active color scale for editing
	 * 
	 * \param[in] currentScale Shared pointer to the color scale to be edited
	 */
	void setActiveScale(ccColorScale::Shared currentScale);

	/**
	 * \brief Get the currently active color scale
	 * 
	 * \return Shared pointer to the current color scale
	 */
	ccColorScale::Shared getActiveScale() { return m_colorScale; }

protected:
	// Internal UI event handlers (protected to prevent direct external calls)

	/**
	 * \brief Handle color scale selection change
	 * 
	 * \param[in] index Index of the selected color scale
	 */
	void colorScaleChanged(int);

	/**
	 * \brief Handle relative/absolute mode change
	 * 
	 * \param[in] index Mode selection index
	 */
	void relativeModeChanged(int);

	/**
	 * \brief Handle color scale step selection
	 * 
	 * \param[in] index Selected step index
	 */
	void onStepSelected(int);

	/**
	 * \brief Handle color scale step modification
	 * 
	 * \param[in] index Modified step index
	 */
	void onStepModified(int);

	/**
	 * \brief Delete the currently selected color scale step
	 */
	void deletecSelectedStep();

	/**
	 * \brief Change color of the selected color scale step
	 */
	void changeSelectedStepColor();

	/**
	 * \brief Change value of the selected color scale step
	 * 
	 * \param[in] value New step value
	 */
	void changeSelectedStepValue(double);

	/**
	 * \brief Handle changes in custom labels list
	 */
	void onCustomLabelsListChanged();

	/**
	 * \brief Toggle visibility of custom labels list
	 * 
	 * \param[in] enabled Flag to show/hide custom labels
	 */
	void toggleCustomLabelsList(bool);

	/**
	 * \brief Copy the current color scale
	 */
	void copyCurrentScale();

	/**
	 * \brief Save the current color scale
	 * 
	 * \return True if save was successful, false otherwise
	 */
	bool saveCurrentScale();

	/**
	 * \brief Delete the current color scale
	 */
	void deleteCurrentScale();

	/**
	 * \brief Rename the current color scale
	 */
	void renameCurrentScale();

	/**
	 * \brief Export the current color scale
	 */
	void exportCurrentScale();

	/**
	 * \brief Import a color scale
	 */
	void importScale();

	/**
	 * \brief Create a new color scale
	 */
	void createNewScale();

	/**
	 * \brief Apply current changes
	 */
	void onApply();

	/**
	 * \brief Close the dialog
	 */
	void onClose();

protected:
	// Internal utility methods

	/**
	 * \brief Update the main combo box with color scales from the manager
	 */
	void updateMainComboBox();

	/**
	 * \brief Set the modification flag state
	 * 
	 * \param[in] state Flag indicating whether the color scale has been modified
	 */
	void setModified(bool state);

	/**
	 * \brief Check if current scale can be changed
	 * 
	 * \return True if scale can be changed, false if user cancels
	 * 
	 * \note Prompts user to save modifications if needed
	 */
	bool canChangeCurrentScale();

	/**
	 * \brief Check if current scale is in relative mode
	 * 
	 * \return True if in relative mode, false if in absolute mode
	 * 
	 * \warning May not reflect the current scale's state if modifications are unsaved
	 */
	bool isRelativeMode() const;

	/**
	 * \brief Set the current scale mode to relative or absolute
	 * 
	 * \param[in] isRelative Flag to set relative (true) or absolute (false) mode
	 * 
	 * \warning May not reflect the current scale's state if modifications are unsaved
	 */
	void setScaleModeToRelative(bool isRelative);

	/**
	 * \brief Validate the custom labels list
	 * 
	 * \param[in] showWarnings Flag to display warning messages
	 * 
	 * \return True if custom labels are valid, false otherwise
	 */
	bool checkCustomLabelsList(bool showWarnings);

	/**
	 * \brief Export the custom labels list
	 * 
	 * \param[out] labels Reference to store exported labels
	 * 
	 * \return Error description string (empty if no errors)
	 */
	QString exportCustomLabelsList(ccColorScale::LabelSet& labels) const;

protected:
	// Member variables

	/**
	 * \brief Pointer to the color scales manager
	 */
	ccColorScalesManager* m_manager;

	/**
	 * \brief Currently active color scale
	 */
	ccColorScale::Shared m_colorScale;

	/**
	 * \brief Color scale editor widget
	 */
	ccColorScaleEditorWidget* m_scaleWidget;

	/**
	 * \brief Associated scalar field (optional)
	 */
	ccScalarField* m_associatedSF;

	/**
	 * \brief Flag indicating if the color scale has been modified
	 */
	bool m_modified;

	/**
	 * \brief Minimum boundary for absolute color scales
	 */
	double m_minAbsoluteVal;

	/**
	 * \brief Maximum boundary for absolute color scales
	 */
	double m_maxAbsoluteVal;

	/**
	 * \brief Pointer to the main application interface
	 */
	ccMainAppInterface* m_mainApp;
	
	/**
	 * \brief Pointer to the UI components
	 */
	Ui::ColorScaleEditorDlg* m_ui;
};
