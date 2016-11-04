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

#ifndef CC_FILE_IO_FILTER_HEADER
#define CC_FILE_IO_FILTER_HEADER

//qCC_db
#include <ccHObject.h>
#include <ccHObjectCaster.h>

//Local
#include "ccGlobalShiftManager.h"

class QWidget;

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
					CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN,
					CC_FERR_THIRD_PARTY_LIB_FAILURE,
					CC_FERR_THIRD_PARTY_LIB_EXCEPTION,
					CC_FERR_NOT_IMPLEMENTED,
};

//! Generic file I/O filter
/** Gives static access to file loader.
	Must be implemented by any specific I/O filter.
**/
class FileIOFilter
{
public: //initialization

	//! Destructor
	virtual ~FileIOFilter() {}

	//! Generic loading parameters
	struct LoadParameters
	{
		//! Default constructor
		LoadParameters()
			: shiftHandlingMode(ccGlobalShiftManager::DIALOG_IF_NECESSARY)
			, alwaysDisplayLoadDialog(true)
			, coordinatesShiftEnabled(0)
			, coordinatesShift(0)
			, autoComputeNormals(false)
			, parentWidget(0)
		{}

		//! How to handle big coordinates
		ccGlobalShiftManager::Mode shiftHandlingMode;
		//! Wether to always display a dialog (if any), even if automatic guess is possible
		bool alwaysDisplayLoadDialog;
		//! Whether shift on load has been applied after loading (optional)
		bool* coordinatesShiftEnabled;
		//! If applicable, applied shift on load (optional)
		CCVector3d* coordinatesShift;
		//! Whether normals should be computed at loading time (if possible - e.g. for gridded clouds) or not
		bool autoComputeNormals;
		//! Parent widget (if any)
		QWidget* parentWidget;
	};

	//! Generic saving parameters
	struct SaveParameters
	{
		//! Default constructor
		SaveParameters()
			: alwaysDisplaySaveDialog(true)
			, parentWidget(0)
		{}

		//! Wether to always display a dialog (if any), even if automatic guess is possible
		bool alwaysDisplaySaveDialog;
		//! Parent widget (if any)
		QWidget* parentWidget;
	};

	//! Shared type
	typedef QSharedPointer<FileIOFilter> Shared;

public: //public interface (to be reimplemented by each I/O filter)

	//! Returns whether this I/O filter can import files
	virtual bool importSupported() const { return false; }

	//! Loads one or more entities from a file
	/** This method must be implemented by children classes.
		\param filename file to load
		\param container container to store loaded entities
		\param parameters generic loading parameters
		\return error
	**/
	virtual CC_FILE_ERROR loadFile(	QString filename,
									ccHObject& container,
									LoadParameters& parameters)
	{
		 return CC_FERR_NOT_IMPLEMENTED;
	}

	//! Returns whether this I/O filter can export files
	virtual bool exportSupported() const { return false; }

	//! Saves an entity (or a group of) to a file
	/** This method must be implemented by children classes.
		\param entity entity (or group of) to save
		\param filename filename
		\param parameters generic saving parameters
		\return error
	**/
	virtual CC_FILE_ERROR saveToFile(	ccHObject* entity,
										QString filename,
										SaveParameters& parameters)
	{
		 return CC_FERR_NOT_IMPLEMENTED;
	}

	//! Returns the file filter(s) for this I/O filter
	/** E.g. 'ASCII file (*.asc)'
		\param onImport whether the requested filters are for import or export
		\return list of filters
	**/
	virtual QStringList getFileFilters(bool onImport) const = 0;

	//! Returns the default file extension
	virtual QString getDefaultExtension() const = 0;

	//! Returns whether a specific file can be loaded by this filter
	/** Used when a file is dragged over the application window for instance.
		Should remain simple (guess based on the file extension, etc.)
	**/
	//virtual bool canLoad(QString filename) const = 0;

	//! Returns whether a specific extension can be loaded by this filter
	/** \param upperCaseExt upper case extension
		\return whether the extension is (theoretically) handled by this filter
	**/
	virtual bool canLoadExtension(QString upperCaseExt) const = 0;

	//! Returns whether this I/O filter can save the specified type of entity
	/** \param type entity type
		\param multiple whether the filter can save multiple instances of this entity at once
		\param exclusive whether the filter can only save this type of entity if selected or if it can be mixed with other types
		\return whether the entity type can be saved
	**/
	virtual bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const = 0;

public: //static methods

	//! Loads one or more entities from a file with a known filter
	/** Shortcut to FileIOFilter::loadFile
		\param filename filename
		\param parameters generic loading parameters
		\param filter input filter
		\return loaded entities (or 0 if an error occurred)
	**/
	QCC_IO_LIB_API static ccHObject* LoadFromFile(	const QString& filename,
													LoadParameters& parameters,
													Shared filter,
													CC_FILE_ERROR& result);

	//! Loads one or more entities from a file with known type
	/** Shortcut to the other version of FileIOFilter::LoadFromFile
		\param filename filename
		\param parameters generic loading parameters
		\param fileFilter input filter 'file filter' (if empty, the best I/O filter will be guessed from the file extension)
		\return loaded entities (or 0 if an error occurred)
	**/
	QCC_IO_LIB_API static ccHObject* LoadFromFile(	const QString& filename,
													LoadParameters& parameters,
													CC_FILE_ERROR& result,
													QString fileFilter = QString());

	//! Saves an entity (or a group of) to a specific file thanks to a given filter
	/** Shortcut to FileIOFilter::saveFile
		\param entities entity to save (can be a group of other entities)
		\param filename filename
		\param parameters saving parameters
		\param filter output filter
		\return error type (if any)
	**/
	QCC_IO_LIB_API static CC_FILE_ERROR SaveToFile(	ccHObject* entities,
													const QString& filename,
													SaveParameters& parameters,
													Shared filter);

	//! Saves an entity (or a group of) to a specific file thanks to a given filter
	/** Shortcut to the other version of FileIOFilter::SaveToFile
		\param entities entity to save (can be a group of other entities)
		\param filename filename
		\param parameters saving parameters
		\param fileFilter output filter 'file filter'
		\return error type (if any)
	**/
	QCC_IO_LIB_API static CC_FILE_ERROR SaveToFile(	ccHObject* entities,
													const QString& filename,
													SaveParameters& parameters,
													QString fileFilter);

	//! Shortcut to the ccGlobalShiftManager mechanism specific for files
	/** \param[in] P sample point (typically the first loaded)
		\param[out] Pshift global shift
		\param[in] loadParameters loading parameters
		\param[in] useInputCoordinatesShiftIfPossible whether to use the input 'PShift' vector if possible
		\return whether global shift has been defined/enabled
	**/
	QCC_IO_LIB_API static bool HandleGlobalShift(	const CCVector3d& P,
													CCVector3d& Pshift,
													LoadParameters& loadParameters,
													bool useInputCoordinatesShiftIfPossible = false);

	//! Displays (to console) the message corresponding to a given error code
	/** \param err error code
		\param action "saving", "reading", etc.
		\param filename corresponding file
	**/
	QCC_IO_LIB_API static void DisplayErrorMessage(CC_FILE_ERROR err,
									const QString& action,
									const QString& filename);

	//! Returns whether special characters are present in the input string
	QCC_IO_LIB_API static bool CheckForSpecialChars(QString filename);

public: //global filters registration mechanism

	//! Init internal filters (should be called once)
	QCC_IO_LIB_API static void InitInternalFilters();

	//! Registers a new filter
	QCC_IO_LIB_API static void Register(Shared filter);

	//! Returns the filter corresponding to the given 'file filter'
	QCC_IO_LIB_API static Shared GetFilter(QString fileFilter, bool onImport);

	//! Returns the best filter (presumably) to open a given file extension
	QCC_IO_LIB_API static Shared FindBestFilterForExtension(QString ext);

	//! Type of a I/O filters container
	typedef std::vector< FileIOFilter::Shared > FilterContainer;

	//! Returns the set of all registered filters
	QCC_IO_LIB_API static const FilterContainer& GetFilters();

	//! Unregisters all filters
	/** Should be called at the end of the application
	**/
	QCC_IO_LIB_API static void UnregisterAll();

	//! Called when the filter is unregistered
	/** Does nothing by default **/
	virtual void unregister() {}

};

#endif
