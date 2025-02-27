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

//Qt
#include <QFrame>

//qCC_db
#include <ccColorScale.h>

class QComboBox;
class QToolButton;
class ccColorScalesManager;

/**
 * \brief Advanced color scale selection widget
 * 
 * \details Provides an interactive widget for selecting and managing color scales
 * with a combo box and a dedicated editor button
 * 
 * Key features:
 * - Combo box for selecting predefined color scales
 * - Quick access to color scale editor
 * - Integration with CloudCompare's color scales manager
 * 
 * Typical use cases:
 * - Selecting color scales for data visualization
 * - Customizing color mappings
 * - Providing user-friendly color scale management
 * 
 * \note Inherits from QFrame and provides a comprehensive color scale selection interface
 * 
 * \example
 * \code
 * // Creating a color scale selector
 * ccColorScalesManager* scaleManager = ccColorScalesManager::GetUniqueInstance();
 * 
 * ccColorScaleSelector* scaleSelector = new ccColorScaleSelector(
 *     scaleManager,        // Color scales manager
 *     parentWidget,        // Parent widget
 *     ":/path/to/icon.png" // Optional button icon path
 * );
 * 
 * // Initialize the selector
 * scaleSelector->init();
 * 
 * // Connect to selection and editor signals
 * connect(scaleSelector, &ccColorScaleSelector::colorScaleSelected,
 *         [](int index) {
 *             qDebug() << "Selected color scale index:" << index;
 *         });
 * 
 * connect(scaleSelector, &ccColorScaleSelector::colorScaleEditorSummoned,
 *         []() {
 *             // Open color scale editor
 *         });
 * 
 * // Get the currently selected color scale
 * ccColorScale::Shared selectedScale = scaleSelector->getSelectedScale();
 * 
 * // Set a specific color scale by UUID
 * scaleSelector->setSelectedScale("unique-color-scale-uuid");
 * \endcode
 */
class CCPLUGIN_LIB_API ccColorScaleSelector : public QFrame
{
	Q_OBJECT

public:
	/**
	 * \brief Constructor for color scale selector
	 * 
	 * \param[in] manager Pointer to the color scales manager
	 * \param[in] parent Optional parent widget
	 * \param[in] defaultButtonIconPath Optional path to a custom button icon
	 */
	ccColorScaleSelector(ccColorScalesManager* manager, QWidget* parent, QString defaultButtonIconPath = QString());

	/**
	 * \brief Initialize the selector with the Color Scales Manager
	 * 
	 * \note Must be called after construction to set up the widget
	 */
	void init();

	/**
	 * \brief Set the selected color scale by its unique identifier
	 * 
	 * \param[in] uuid Unique identifier of the color scale to select
	 */
	void setSelectedScale(QString uuid);

	/**
	 * \brief Get the currently selected color scale
	 * 
	 * \return Shared pointer to the selected color scale
	 */
	ccColorScale::Shared getSelectedScale() const;

	/**
	 * \brief Get a color scale by its index in the combo box
	 * 
	 * \param[in] index Index of the color scale
	 * 
	 * \return Shared pointer to the color scale at the specified index
	 */
	ccColorScale::Shared getScale(int index) const;

Q_SIGNALS:
	/**
	 * \brief Signal emitted when a color scale is selected
	 * 
	 * \param[out] index Index of the selected color scale
	 */
	void colorScaleSelected(int);

	/**
	 * \brief Signal emitted when the user requests to open the color scale editor
	 */
	void colorScaleEditorSummoned();

protected:
	/**
	 * \brief Pointer to the color scales manager
	 */
	ccColorScalesManager* m_manager;

	/**
	 * \brief Combo box for selecting color scales
	 */
	QComboBox* m_comboBox;

	/**
	 * \brief Button to spawn the color scale editor
	 */
	QToolButton* m_button;
};
