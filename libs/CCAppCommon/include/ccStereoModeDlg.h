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

#include "CCAppCommon.h"

//qCC_gl
#include <ccGLWindowInterface.h>

//Qt
#include <QDialog>

namespace Ui
{
	class StereoModeDialog;
}

/**
 * \brief Dialog for configuring stereo mode parameters in 3D views.
 * 
 * \details This dialog allows users to customize stereo rendering settings
 * for enhanced 3D visualization. It provides options to:
 * - Select different types of stereo glasses
 * - Adjust stereo rendering parameters
 * - Toggle field of view (FOV) updates
 * 
 * \note Inherits from QDialog to provide a standard dialog interface
 * 
 * \example
 * \code
 * // Create a stereo mode dialog
 * ccStereoModeDlg stereoDialog(parentWidget);
 * 
 * // Set initial stereo parameters
 * ccGLWindowInterface::StereoParams initialParams;
 * initialParams.glassType = ccGLWindowInterface::STEREO_GLASSES_NVIDIA;
 * stereoDialog.setParameters(initialParams);
 * 
 * // Show the dialog and check if user accepted
 * if (stereoDialog.exec() == QDialog::Accepted)
 * {
 *     // Retrieve the updated stereo parameters
 *     ccGLWindowInterface::StereoParams updatedParams = stereoDialog.getParameters();
 *     
 *     // Check if FOV needs to be updated
 *     if (stereoDialog.updateFOV())
 *     {
 *         // Apply FOV update logic
 *     }
 * }
 * \endcode
 */
class CCAPPCOMMON_LIB_API ccStereoModeDlg : public QDialog
{
	Q_OBJECT

public:
	/**
	 * \brief Constructs a new stereo mode configuration dialog
	 * 
	 * \param[in] parent Optional parent widget
	 */
	explicit ccStereoModeDlg(QWidget* parent);

	/**
	 * \brief Destructor for the stereo mode dialog
	 * 
	 * Cleans up resources associated with the dialog
	 */
	~ccStereoModeDlg() override;

	/**
	 * \brief Retrieve the current stereo mode parameters
	 * 
	 * \return Current stereo rendering parameters
	 */
	ccGLWindowInterface::StereoParams getParameters() const;

	/**
	 * \brief Set the stereo mode parameters
	 * 
	 * \param[in] params Stereo rendering parameters to configure
	 */
	void setParameters(const ccGLWindowInterface::StereoParams& params);

	/**
	 * \brief Check if the field of view (FOV) should be updated
	 * 
	 * \return True if FOV needs to be updated, false otherwise
	 */
	bool updateFOV() const;

protected:
	/**
	 * \brief Slot called when the stereo glasses type is modified
	 * 
	 * Updates the dialog's UI and internal state based on the selected 
	 * stereo glasses type
	 * 
	 * \param[in] index Index of the selected glasses type
	 */
	void glassTypeChanged(int);
	
private:
	/**
	 * \brief Pointer to the UI components of the dialog
	 * 
	 * Manages the user interface elements of the stereo mode configuration dialog
	 */
	Ui::StereoModeDialog* m_ui;
};
