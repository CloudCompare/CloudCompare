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

//Local
#include "ccOptions.h"

//qCC_gl
#include <ccGuiParameters.h>

//Qt
#include <QDialog>

namespace Ui
{
	class DisplaySettingsDlg;
}

/**
 * \class ccDisplaySettingsDlg
 * \brief Dialog for configuring and customizing display settings in CloudCompare
 * 
 * This dialog allows users to modify various visual and interaction parameters
 * for the application, including colors, sizes, rendering options, and other 
 * display-related preferences.
 * 
 * \note Inherits from QDialog to provide a standard dialog interface
 */
class CCAPPCOMMON_LIB_API ccDisplaySettingsDlg : public QDialog
{
	Q_OBJECT

public:
	/**
	 * \brief Constructs a new display settings dialog
	 * 
	 * \param[in] parent Pointer to the parent widget (default is nullptr)
	 */
	explicit ccDisplaySettingsDlg(QWidget* parent);

	/**
	 * \brief Destructor for the display settings dialog
	 * 
	 * Cleans up resources associated with the dialog
	 */
	~ccDisplaySettingsDlg() override;

Q_SIGNALS:
	/**
	 * \brief Signal emitted when display aspect has changed
	 * 
	 * Notifies listeners that display settings have been modified
	 */
	void aspectHasChanged();

protected:
	// Color modification methods
	/**
	 * \brief Changes the diffuse color of the light source
	 */
	void changeLightDiffuseColor();

	/**
	 * \brief Changes the ambient color of the light source
	 */
	void changeLightAmbientColor();

	/**
	 * \brief Changes the specular color of the light source
	 */
	void changeLightSpecularColor();

	/**
	 * \brief Changes the front-facing diffuse color of meshes
	 */
	void changeMeshFrontDiffuseColor();

	/**
	 * \brief Changes the back-facing diffuse color of meshes
	 */
	void changeMeshBackDiffuseColor();

	/**
	 * \brief Changes the specular color of meshes
	 */
	void changeMeshSpecularColor();

	/**
	 * \brief Changes the default color of point clouds
	 */
	void changePointsColor();

	/**
	 * \brief Changes the color of bounding boxes
	 */
	void changeBBColor();

	/**
	 * \brief Changes the default text color
	 */
	void changeTextColor();

	/**
	 * \brief Changes the background color of the display
	 */
	void changeBackgroundColor();

	/**
	 * \brief Changes the background color of labels
	 */
	void changeLabelBackgroundColor();

	/**
	 * \brief Changes the marker color of labels
	 */
	void changeLabelMarkerColor();

	// Size and rendering modification methods
	/**
	 * \brief Changes the maximum size of meshes
	 * 
	 * \param[in] size New maximum mesh size
	 */
	void changeMaxMeshSize(double);

	/**
	 * \brief Changes the maximum size of point clouds
	 * 
	 * \param[in] size New maximum point cloud size
	 */
	void changeMaxCloudSize(double);

	/**
	 * \brief Toggles Vertex Buffer Object (VBO) usage
	 */
	void changeVBOUsage();

	/**
	 * \brief Changes the width of the color scale ramp
	 * 
	 * \param[in] width New width of the color scale ramp
	 */
	void changeColorScaleRampWidth(int);

	/**
	 * \brief Changes the picking cursor style
	 * 
	 * \param[in] style Index of the new picking cursor style
	 */
	void changePickingCursor(int);

	/**
	 * \brief Changes the logging verbosity level
	 * 
	 * \param[in] verbosity New verbosity level
	 */
	void changeLogVerbosity(int);

	// Font and display text methods
	/**
	 * \brief Changes the default font size
	 * 
	 * \param[in] size New default font size
	 */
	void changeDefaultFontSize(int);

	/**
	 * \brief Changes the font size of labels
	 * 
	 * \param[in] size New label font size
	 */
	void changeLabelFontSize(int);

	/**
	 * \brief Changes the number precision for display
	 * 
	 * \param[in] precision New number precision
	 */
	void changeNumberPrecision(int);

	/**
	 * \brief Changes the opacity of labels
	 * 
	 * \param[in] opacity New label opacity level
	 */
	void changeLabelOpacity(int);

	/**
	 * \brief Changes the size of label markers
	 * 
	 * \param[in] size New label marker size
	 */
	void changeLabelMarkerSize(int);

	/**
	 * \brief Changes the zoom speed
	 * 
	 * \param[in] speed New zoom speed
	 */
	void changeZoomSpeed(double);

	/**
	 * \brief Changes the auto-compute octree option
	 * 
	 * \param[in] option New auto-compute octree setting
	 */
	void changeAutoComputeOctreeOption(int);

	// Dialog action methods
	/**
	 * \brief Accepts and applies the current settings
	 */
	void doAccept();

	/**
	 * \brief Rejects the current settings and closes the dialog
	 */
	void doReject();

	/**
	 * \brief Applies the current settings without closing the dialog
	 */
	void apply();

	/**
	 * \brief Resets settings to their default values
	 */
	void reset();

protected:
	/**
	 * \brief Refreshes the dialog to reflect current parameter values
	 * 
	 * Updates the UI elements to match the current configuration
	 */
	void refresh();

	// Color member variables
	QColor m_lightDiffuseColor;   ///< Diffuse color of the light source
	QColor m_lightAmbientColor;   ///< Ambient color of the light source
	QColor m_lightSpecularColor;  ///< Specular color of the light source
	QColor m_meshFrontDiff;       ///< Front-facing diffuse color of meshes
	QColor m_meshBackDiff;        ///< Back-facing diffuse color of meshes
	QColor m_meshSpecularColor;   ///< Specular color of meshes
	QColor m_pointsDefaultCol;    ///< Default color of point clouds
	QColor m_textDefaultCol;      ///< Default text color
	QColor m_backgroundCol;       ///< Background color
	QColor m_labelBackgroundCol;  ///< Background color of labels
	QColor m_labelMarkerCol;      ///< Marker color of labels
	QColor m_bbDefaultCol;        ///< Default bounding box color

	/**
	 * \brief Current GUI parameters
	 * 
	 * Stores the active configuration for the GUI
	 */
	ccGui::ParamStruct m_parameters;

	/**
	 * \brief Current application options
	 * 
	 * Stores additional configuration options for the application
	 */
	ccOptions m_options;

	/**
	 * \brief Previous GUI parameters (for restoration)
	 * 
	 * Keeps a backup of the original parameters to allow reverting changes
	 */
	ccGui::ParamStruct m_oldParameters;

	/**
	 * \brief Previous application options (for restoration)
	 * 
	 * Keeps a backup of the original options to allow reverting changes
	 */
	ccOptions m_oldOptions;

	/**
	 * \brief Index of the default application style
	 * 
	 * Stores the style that was active when the dialog was first opened
	 */
	int m_defaultAppStyleIndex;
	
private:
	/**
	 * \brief Pointer to the UI components of the dialog
	 * 
	 * Manages the user interface elements of the display settings dialog
	 */
	Ui::DisplaySettingsDlg* m_ui;
};
