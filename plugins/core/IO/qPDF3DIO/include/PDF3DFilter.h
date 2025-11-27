//##########################################################################
//#                                                                        #
//#          CLOUDCOMPARE PLUGIN: PDF3DFilter                              #
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
#include <QString>
#include <QStringList>
#include <QFileInfo>

// CloudCompare includes
#include "FileIOFilter.h"

// Forward declarations
class ccHObject;
class ccPointCloud;
class ccMesh;
class ccGenericPointCloud;

/**
 * @brief PDF3DFilter - File I/O Filter for 3D PDF Export
 * 
 * This filter handles the export of point clouds and meshes to 3D PDF format.
 * It implements the FileIOFilter interface to integrate with CloudCompare's
 * file I/O system.
 * 
 * The filter supports:
 * - Exporting point clouds to 3D PDF
 * - Exporting meshes to 3D PDF
 * - Color information preservation
 * - Scalar field visualization
 * - Configurable export quality
 * - Compression options
 * 
 * Implementation Details:
 * -----------------------
 * The filter converts CloudCompare entities (point clouds, meshes) to
 * a format suitable for embedding in PDF files. This typically involves:
 * 
 * 1. Data preparation:
 *    - Simplification of large point clouds (if needed)
 *    - Normal computation (if needed for lighting)
 *    - Color/normalization
 * 
 * 2. Format conversion:
 *    - Convert to U3D or PRC format
 *    - Apply compression if requested
 *    - Optimize for PDF embedding
 * 
 * 3. PDF generation:
 *    - Create PDF document structure
 *    - Embed 3D content
 *    - Add JavaScript for interactivity
 *    - Set up 3D annotations
 *    - Configure viewports and views
 * 
 * Export Options:
 * --------------
 * - Quality level (affects point count/triangle count)
 * - Compression level
 * - Color mode (RGB, scalar field, grayscale)
 * - Point size (for point clouds)
 * - Background color
 * - Lighting settings
 * - Initial view/camera position
 * 
 * File Format Support:
 * -------------------
 * - Input: ccPointCloud, ccMesh entities
 * - Output: PDF 1.7 with embedded U3D/PRC
 * - Extension: .pdf
 * 
 * Error Handling:
 * --------------
 * The filter should handle various error conditions:
 * - Invalid entities (empty clouds, invalid meshes)
 * - File write errors
 * - Memory allocation failures
 * - Format conversion errors
 * - PDF generation errors
 * 
 * Progress Reporting:
 * -----------------
 * For large exports, the filter should report progress:
 * - Data preparation progress
 * - Format conversion progress
 * - PDF generation progress
 * 
 * This allows users to track long-running export operations.
 */

//! File I/O filter for 3D PDF export
class PDF3DFilter : public FileIOFilter
{
public:
	/**
	 * @brief Constructor
	 * 
	 * Initializes the filter with default settings and file format information.
	 */
	PDF3DFilter();

	/**
	 * @brief Destructor
	 * 
	 * Cleanup is handled automatically.
	 */
	~PDF3DFilter() override = default;

	/**
	 * @brief Get import capabilities
	 * @return Import capabilities (currently none - export only)
	 * 
	 * This filter currently only supports export, not import.
	 * Future versions may add import support for 3D PDF files.
	 */
	CC_FILE_ERROR loadFile( const QString &filename, ccHObject &container, LoadParameters &parameters ) override;

	/**
	 * @brief Check if this filter can save the given entity
	 * @param entity Entity to check
	 * @return true if the entity can be saved, false otherwise
	 * 
	 * This filter can save:
	 * - Point clouds (ccPointCloud)
	 * - Meshes (ccMesh)
	 * - Groups containing point clouds or meshes
	 */
	bool canSave( CC_CLASS_ENUM type, bool &multiple, bool &exclusive ) const override;

	/**
	 * @brief Save entity to 3D PDF file
	 * @param entities List of entities to save
	 * @param baseFilename Base filename for output
	 * @param parameters Save parameters
	 * @return CC_FILE_ERROR code indicating success or failure
	 * 
	 * Main export function. Converts CloudCompare entities to 3D PDF format.
	 * 
	 * Process:
	 * 1. Validate input entities
	 * 2. Show export options dialog (if GUI mode)
	 * 3. Prepare data (simplify if needed, compute normals, etc.)
	 * 4. Convert to U3D/PRC format
	 * 5. Generate PDF with embedded 3D content
	 * 6. Write to file
	 * 
	 * Error codes:
	 * - CC_FERR_NO_ERROR: Success
	 * - CC_FERR_BAD_ARGUMENT: Invalid input
	 * - CC_FERR_WRITING: File write error
	 * - CC_FERR_NOT_ENOUGH_MEMORY: Memory allocation failure
	 * - CC_FERR_CANCELED_BY_USER: User canceled operation
	 */
	CC_FILE_ERROR saveToFile( ccHObject *entity, const QString &filename, const SaveParameters &parameters ) override;

	/**
	 * @brief Get file filters for file dialog
	 * @return File filter string for Qt file dialogs
	 * 
	 * Returns the file filter string used in file save dialogs.
	 * Example: "3D PDF Files (*.pdf)"
	 */
	QStringList getFileFilters( bool onImport ) const override;

	/**
	 * @brief Get default file extension
	 * @return Default file extension (without dot)
	 * 
	 * Returns "pdf" as the default extension for 3D PDF files.
	 */
	QString getDefaultExtension() const override;

	/**
	 * @brief Check if this filter can load the given file
	 * @param filename File to check
	 * @param header Optional file header data
	 * @return true if file can be loaded, false otherwise
	 * 
	 * Currently returns false as import is not yet implemented.
	 */
	bool canLoadExtension( const QString &upperCaseExt ) const override;

	/**
	 * @brief Check if this filter can import files
	 * @return false (export only for now)
	 */
	bool canLoadFile( const QString &filename ) const override;

private:
	/**
	 * @brief Convert point cloud to 3D PDF format
	 * @param cloud Point cloud to convert
	 * @param filename Output PDF filename
	 * @param parameters Save parameters
	 * @return CC_FILE_ERROR code
	 * 
	 * Internal method to handle point cloud export.
	 * This is where the actual conversion happens.
	 */
	CC_FILE_ERROR exportPointCloud( ccPointCloud *cloud, const QString &filename, const SaveParameters &parameters );

	/**
	 * @brief Convert mesh to 3D PDF format
	 * @param mesh Mesh to convert
	 * @param filename Output PDF filename
	 * @param parameters Save parameters
	 * @return CC_FILE_ERROR code
	 * 
	 * Internal method to handle mesh export.
	 * This is where the actual conversion happens.
	 */
	CC_FILE_ERROR exportMesh( ccMesh *mesh, const QString &filename, const SaveParameters &parameters );

	/**
	 * @brief Simplify point cloud if needed
	 * @param cloud Point cloud to potentially simplify
	 * @param maxPoints Maximum number of points
	 * @return Simplified cloud (new instance) or original if no simplification needed
	 * 
	 * Large point clouds may need simplification before embedding in PDF.
	 * This method handles the simplification process.
	 */
	ccPointCloud* simplifyPointCloud( ccPointCloud *cloud, unsigned maxPoints );

	/**
	 * @brief Simplify mesh if needed
	 * @param mesh Mesh to potentially simplify
	 * @param maxTriangles Maximum number of triangles
	 * @return Simplified mesh (new instance) or original if no simplification needed
	 * 
	 * Large meshes may need simplification before embedding in PDF.
	 * This method handles the simplification process.
	 */
	ccMesh* simplifyMesh( ccMesh *mesh, unsigned maxTriangles );

	// Additional private helper methods can be added here as needed
	// For example:
	// - computeNormalsIfNeeded()
	// - convertToU3D()
	// - createPDFDocument()
	// - add3DAnnotation()
	// - addJavaScript()
	// etc.
};

