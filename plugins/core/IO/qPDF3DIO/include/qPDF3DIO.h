//##########################################################################
//#                                                                        #
//#          CLOUDCOMPARE PLUGIN: qPDF3DIO                                 #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#pragma once

// Standard includes
#include <QObject>
#include <QString>
#include <QStringList>

// CloudCompare plugin interface
#include "ccIOPluginInterface.h"

// Forward declarations
class ccHObject;
class ccPointCloud;
class ccMesh;
class ccGenericPointCloud;

/**
 * @brief qPDF3DIO Plugin - 3D PDF Export Functionality
 * 
 * This plugin provides functionality to export point clouds and meshes
 * as interactive 3D PDF files. 3D PDF files allow users to share 3D data
 * in a universally accessible format that can be viewed in standard PDF readers
 * such as Adobe Acrobat Reader, without requiring specialized 3D software.
 * 
 * Features:
 * ---------
 * - Export point clouds to 3D PDF format
 * - Export meshes to 3D PDF format
 * - Support for color information (RGB)
 * - Support for scalar fields as color mapping
 * - Interactive 3D viewing in PDF readers
 * - Configurable export options (quality, compression, etc.)
 * - Command-line interface support
 * 
 * Technical Details:
 * ------------------
 * 3D PDF format (PDF 1.7 with 3D annotations) uses:
 * - U3D (Universal 3D) or PRC (Product Representation Compact) formats
 *   embedded in PDF files
 * - JavaScript for interactive controls
 * - 3D annotations for embedding 3D content
 * 
 * Implementation Approach:
 * ------------------------
 * 1. Convert point clouds/meshes to U3D or PRC format
 *    - U3D: More widely supported, simpler format
 *    - PRC: Better compression, more features (Adobe proprietary)
 * 2. Embed 3D content in PDF using PDF libraries
 * 3. Add JavaScript for interactive controls
 * 4. Set up 3D annotations and viewports
 * 
 * Dependencies:
 * ------------
 * - PDF generation library (e.g., PoDoFo, PDFium, or similar)
 * - 3D format conversion library (U3D/PRC)
 * - Qt framework for GUI components
 * 
 * File Format:
 * -----------
 * - Output: .pdf (PDF 1.7 with 3D annotations)
 * - Embedded: U3D or PRC 3D data
 * - Viewable in: Adobe Acrobat Reader, Foxit Reader, and other PDF viewers
 * 
 * Use Cases:
 * ---------
 * - Sharing 3D models with non-technical users
 * - Documentation and reports with embedded 3D content
 * - Presentation of point cloud data
 * - Archiving 3D data in a standard format
 * - Integration with document workflows
 * 
 * Limitations:
 * ------------
 * - Large point clouds may need simplification for PDF embedding
 * - 3D PDF support varies between PDF readers
 * - File size can be large for complex models
 * - Some advanced features may not be supported in all viewers
 * 
 * Future Enhancements:
 * -------------------
 * - Support for multiple 3D objects in one PDF
 * - Custom lighting and material settings
 * - Animation support
 * - Measurement tools in PDF
 * - Layer/visibility controls
 * - Texture mapping support
 * 
 * References:
 * -----------
 * - PDF 1.7 specification (ISO 32000-1)
 * - U3D specification
 * - PRC format documentation
 * - CloudCompare plugin development guide
 */

//! 3D PDF Export Plugin for CloudCompare
class qPDF3DIO : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccIOPluginInterface )

	// Plugin metadata - links to info.json file
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.PDF3D" FILE "../info.json" )

public:
	/**
	 * @brief Constructor
	 * @param parent Parent QObject (typically the plugin manager)
	 * 
	 * Initializes the plugin and loads metadata from info.json
	 */
	explicit qPDF3DIO( QObject *parent = nullptr );

	/**
	 * @brief Destructor
	 * 
	 * Cleanup is handled automatically by Qt's parent-child mechanism
	 */
	~qPDF3DIO() override = default;

	/**
	 * @brief Register command-line commands
	 * @param cmd Command-line interface instance
	 * 
	 * Registers command-line options for 3D PDF export.
	 * This allows users to export 3D PDFs from the command line.
	 * 
	 * Example command:
	 *   CloudCompare -O cloud.bin -PDF3D_EXPORT output.pdf
	 */
	void registerCommands( ccCommandLineInterface *cmd ) override;

	/**
	 * @brief Get list of file filters provided by this plugin
	 * @return List of file I/O filters
	 * 
	 * Returns the list of file filters that this plugin provides.
	 * Currently provides:
	 * - PDF3DFilter: Export point clouds and meshes to 3D PDF format
	 * 
	 * The filters are registered with CloudCompare's file I/O system
	 * and will appear in the file save dialog.
	 */
	ccIOPluginInterface::FilterList getFilters() override;

private:
	// Private helper methods could be added here if needed
	// For example:
	// - validateExportOptions()
	// - convertToU3D()
	// - createPDFDocument()
	// etc.
};

