//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_FILE_IO_FILTER_HEADER
#define CC_FILE_IO_FILTER_HEADER

//Qt
#include <QString>

//qCC_db
#include <ccHObject.h>
#include <ccHObjectCaster.h>

//! Max number of characters per line in an ASCII file
const int MAX_ASCII_FILE_LINE_LENGTH	=	4096;

//! File types handled by CloudCompare (loading and/or saving)
enum CC_FILE_TYPES {UNKNOWN_FILE = 0	,		/**< unknown type */
					SOI					,		/**< SOI (Mensi Soisic) */
					ASCII				,		/**< ASC,NEU, XYZ, TXT, PTS, etc. */
					BIN					,		/**< CloudCompare binary */
					PN					,		/**< Point-Normal (binary) */
					PV					,		/**< Point-Value (binary) */
					PLY					,		/**< Stanford mesh file */
					OBJ					,		/**< Wavefront mesh file */
					POV					,		/**< Multiple Point-Of-View cloud meta-file (ascii) */
					MA					,		/**< Maya mesh (ascii) */
					ICM					,		/**< Calibrated Images meta-file */
					DM_ASCII			,		/**< Depth Map (ascii) */
					BUNDLER				,		/**< Bundler output (ascii) */
					VTK					,		/**< VTK mesh/cloud file */
					STL					,		/**< STL mesh file (ascii) */
					PCD					,		/**< Point Cloud Library file */
					OFF					,		/**< OFF mesh file (ascii) */
					PTX					,		/**< PTX cloud file (ascii) */
#ifdef CC_X3D_SUPPORT
					X3D					,		/**< X3D mesh file */
#endif
#ifdef CC_LAS_SUPPORT
					LAS					,		/**< LAS lidar point cloud (binary) */
#endif
#ifdef CC_E57_SUPPORT
					E57					,		/**< ASTM E2807-11 E57 file */
#endif
#ifdef CC_PDMS_SUPPORT
					PDMS				,		/**< PDMS (.pdmsmac) */
#endif
#ifdef CC_DXF_SUPPORT
					DXF					,		/**< DXF (Autocad) */
#endif
#ifdef CC_GDAL_SUPPORT
					RASTER				,		/**< GIS 2D1/2 raster (supported by GDAL) */
#endif
#ifdef CC_FBX_SUPPORT
					FBX					,		/**< Autodesk FBX format */
#endif
					FILE_TYPES_COUNT	,		/**< Fake file type (for automatic counting) */
};

const CC_FILE_TYPES CC_FILE_TYPES_ENUMS[] = {UNKNOWN_FILE, SOI, ASCII, BIN,
												PN, PV, PLY, OBJ, POV,
												MA, ICM, DM_ASCII, BUNDLER,
												VTK, STL, PCD, OFF, PTX
#ifdef CC_X3D_SUPPORT
												,X3D
#endif
#ifdef CC_LAS_SUPPORT
												,LAS
#endif
#ifdef CC_E57_SUPPORT
												,E57
#endif
#ifdef CC_PDMS_SUPPORT
												,PDMS
#endif
#ifdef CC_DXF_SUPPORT
												,DXF
#endif
#ifdef CC_GDAL_SUPPORT
												,RASTER
#endif
#ifdef CC_FBX_SUPPORT
												,FBX
#endif
};

const char CC_FILE_TYPE_FILTERS[][64] = {
			"All (*.*)",
			"SOI Soisic Mensi (*.soi)",
			"ASCII files (*.txt *.asc *.neu *.xyz *.pts *.csv)",
			"BIN CloudCompare binaries (*.bin)",
			"PN Point-Normal [binary] (*.pn)",
			"PV Point-Value [binary] (*.pv)",
			"PLY Stanford mesh file (*.ply)",
			"OBJ Wavefront mesh file (*.obj)",
			"POV Multiple Point-Of-View cloud meta-file [ascii] (*.pov)",
			"MA Maya ASCII file (*.ma)",
			"ICM Calibrated Images meta-file (*.icm)",
			"ASCII Depth Map (*.txt *.asc)",
			"Snavely's Bundler output (*.out)",
			"VTK cloud or mesh (*.vtk)",
			"STL mesh (*.stl)",
			"PCD Point Cloud Library cloud (*.pcd)",
			"OFF mesh (*.off)",
			"PTX cloud (*.ptx)" 
#ifdef CC_X3D_SUPPORT
			, "X3D mesh file (*.x3d)"
#endif
#ifdef CC_LAS_SUPPORT
			, "LAS lidar point cloud (*.las *.laz)"
#endif
#ifdef CC_E57_SUPPORT
			, "E57 ASTM E2807-11 files (*.e57)"
#endif
#ifdef CC_PDMS_SUPPORT
			, "PDMS (*.pdms *.pdmsmac *.mac)"
#endif
#ifdef CC_DXF_SUPPORT
			, "DXF (*.dxf)"
#endif
#ifdef CC_GDAL_SUPPORT
			, "RASTER grid (*.*)"
#endif
#ifdef CC_FBX_SUPPORT
			, "FBX Autodesk mesh (*.fbx)"
#endif
};

const char CC_FILE_TYPE_DEFAULT_EXTENSION[][8] = {
				"",
				"soi",
				"asc",
				"bin",
				"pn",
				"pv",
				"ply",
				"obj",
				"pov",
				"ma",
				"icm",
				"txt",
				"out",
				"vtk",
				"stl",
				"pcd",
				"off",
				"ptx"
#ifdef CC_X3D_SUPPORT
				, "x3d"
#endif
#ifdef CC_LAS_SUPPORT
				, "las"
#endif
#ifdef CC_E57_SUPPORT
				, "e57"
#endif
#ifdef CC_PDMS_SUPPORT
				, "pdms"
#endif
#ifdef CC_DXF_SUPPORT
				, "dxf"
#endif
#ifdef CC_GDAL_SUPPORT
				, "tif"
#endif
#ifdef CC_FBX_SUPPORT
				, "fbx"
#endif
};

//! Typical I/O filter errors
enum CC_FILE_ERROR {CC_FERR_NO_ERROR,
					CC_FERR_BAD_ARGUMENT,
					CC_FERR_UNKNOWN_FILE,
					CC_FERR_WRONG_FILE_TYPE,
					CC_FERR_WRITING,
					CC_FERR_READING,
					CC_FERR_NO_SAVE,
					CC_FERR_NO_LOAD,
					CC_FERR_BAD_ENTITY_TYPE,
					CC_FERR_CANCELED_BY_USER,
					CC_FERR_NOT_ENOUGH_MEMORY,
					CC_FERR_MALFORMED_FILE,
					CC_FERR_CONSOLE_ERROR,
					CC_FERR_BROKEN_DEPENDENCY_ERROR,
					CC_FERR_FILE_WAS_WRITTEN_BY_PLUGIN
};

//! Generic file I/O filter
/** Gives static access to file loader.
	Must be implemented by any specific I/O filter.
**/
class FileIOFilter
{
public:
   virtual ~FileIOFilter() {}

	//! Loads one or more entities from a file with known type
	/** \param filename filename
		\param fType file type (if left to UNKNOWN_FILE, file type will be guessed from extension)
		\param alwaysDisplayLoadDialog always display (eventual) display dialog, even if automatic guess is possible
		\param coordinatesShiftEnabled whether shift on load has been applied after loading
		\param coordinatesShift if applicable, applied shift on load (3D translation)
		\return loaded entities (or 0 if an error occurred)
	**/
	static ccHObject* LoadFromFile(const QString& filename,
									CC_FILE_TYPES fType = UNKNOWN_FILE,
									bool alwaysDisplayLoadDialog = true,
									bool* coordinatesShiftEnabled = 0,
									CCVector3d* coordinatesShift = 0);

	//! Saves an entity (or a group of) to a specific file (with name and type)
	static CC_FILE_ERROR SaveToFile(ccHObject* entities,
									const QString& filename,
									CC_FILE_TYPES fType);

	//! Displays (to console) the message corresponding to a given error code
	/** \param err error code
		\param action "saving", "reading", etc.
		\param filename corresponding file
	**/
	static void DisplayErrorMessage(CC_FILE_ERROR err,
									const QString& action,
									const QString& filename);

	//! Loads one or more entities from a file
	/** This method must be implemented by children classes.
		\param filename file to load
		\param container container to store loaded entities
		\param alwaysDisplayLoadDialog always display (eventual) display dialog, even if automatic guess is possible
		\param coordinatesShiftEnabled whether shift on load has already been defined or not (may be modified by this method)
		\param coordinatesShift already applied (input) or newly applied (output) shift on load (3D translation)
		\return error
	**/
	virtual CC_FILE_ERROR loadFile(	QString filename,
									ccHObject& container,
									bool alwaysDisplayLoadDialog = true,
									bool* coordinatesShiftEnabled = 0,
									CCVector3d* coordinatesShift = 0) = 0;

	//! Saves an entity (or a group of) to a file
	/** This method must be implemented by children classes.
		\param entity entity (or group of) to save
		\param filename filename
		\return error
	**/
	virtual CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename) = 0;

	//! Detecs file type from file extension
	static CC_FILE_TYPES GuessFileFormatFromExtension(QString ext);

	//! Factory: returns a filter given it's type
	static FileIOFilter* CreateFilter(CC_FILE_TYPES fType);

};

#endif
