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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#ifndef CC_COMMAND_LINE_INTERFACE_HEADER
#define CC_COMMAND_LINE_INTERFACE_HEADER

//qCC_db
#include <ccPointCloud.h>

//qCC_io
#include <FileIOFilter.h>

//Qt
#include <QSharedPointer>
#include <QString>

//System
#include <vector>

class ccGenericMesh;
class ccProgressDialog;

class QDialog;
class QStringList;

enum class CL_ENTITY_TYPE {
	GROUP,
	CLOUD,
	MESH
};

//! Loaded entity description
struct CLEntityDesc
{
	QString basename;
	QString path;
	int indexInFile;

	CLEntityDesc( const QString &name );
	CLEntityDesc( const QString &filename, int _indexInFile );
	CLEntityDesc( const QString &_basename, const QString &_path, int _indexInFile = -1 );
	
	virtual ~CLEntityDesc() = default;
	
	virtual ccHObject* getEntity() = 0;
	virtual const ccHObject* getEntity() const = 0;
	virtual CL_ENTITY_TYPE getCLEntityType() const = 0;
};

//! Loaded group description
struct CLGroupDesc : CLEntityDesc
{
	ccHObject* groupEntity;

	CLGroupDesc(ccHObject* group,
				const QString& basename,
				const QString& path = QString());

	~CLGroupDesc() override = default;
	
	ccHObject* getEntity() override;
	const ccHObject* getEntity() const override;
	CL_ENTITY_TYPE getCLEntityType() const override;
};

//! Loaded cloud description
struct CLCloudDesc : CLEntityDesc
{
	ccPointCloud* pc;

	CLCloudDesc();

	CLCloudDesc(ccPointCloud* cloud,
				const QString& filename = QString(),
				int index = -1);

	CLCloudDesc(ccPointCloud* cloud,
				const QString& basename,
				const QString& path,
				int index = -1);

	~CLCloudDesc() override = default;

	ccHObject* getEntity() override;
	const ccHObject* getEntity() const override;
	CL_ENTITY_TYPE getCLEntityType() const override;
};

//! Loaded mesh description
struct CLMeshDesc : CLEntityDesc
{
	ccGenericMesh* mesh;

	CLMeshDesc();

	CLMeshDesc(	ccGenericMesh* _mesh,
				const QString& filename = QString(),
				int index = -1 );

	CLMeshDesc(	ccGenericMesh* _mesh,
				const QString& basename,
				const QString& path,
				int index = -1 );

	~CLMeshDesc() override = default;
	
	ccHObject* getEntity() override;
	const ccHObject* getEntity() const override;
	CL_ENTITY_TYPE getCLEntityType() const override;
};

//! Command line interface
class ccCommandLineInterface
{
public: //constructor

	//! Default constructor
	ccCommandLineInterface();
	
	virtual ~ccCommandLineInterface() = default;

	enum class ExportOption {
		NoOptions = 0x0,
		ForceCloud = 0x1,
		ForceMesh = 0x2,
		ForceHierarchy = 0x4,
		ForceNoTimestamp = 0x8
	};
	Q_DECLARE_FLAGS(ExportOptions, ExportOption)

public: //commands

	//! Generic command interface
	struct Command
	{
		//! Shared type
		using Shared = QSharedPointer<Command>;

		//! Default constructor
		Command(const QString& name, const QString& keyword);

		virtual ~Command() = default;
		
		//! Main process
		virtual bool process(ccCommandLineInterface& cmd) = 0;

		//! Command name
		QString m_name;
		//! Command keyword
		QString m_keyword;
	};

	//! Test whether a command line token is a valid command keyword or not
	static bool IsCommand(const QString& token, const char* command);

public: //virtual methods

	//! Registers a new command
	/** \return success
	**/
	virtual bool registerCommand(Command::Shared command) = 0;

	//! Returns the name of a to-be-exported entity
	virtual QString getExportFilename(	const CLEntityDesc& entityDesc,
										QString extension = QString(),
										QString suffix = QString(),
										QString* baseOutputFilename = nullptr,
										bool forceNoTimestamp = false) const = 0;

	//! Exports a cloud or a mesh
	/** \return error string (if any)
	**/
	virtual QString exportEntity(CLEntityDesc& entityDesc,
		const QString &suffix = QString(),
		QString* outputFilename = nullptr,
		ccCommandLineInterface::ExportOptions options = ExportOption::NoOptions) = 0;

	//! Saves all clouds
	/** \param suffix optional suffix
		\param allAtOnce whether to save all clouds in the same file or one cloud per file
		\return success
	**/
	virtual bool saveClouds(QString suffix = QString(), bool allAtOnce = false, const QString* allAtOnceFileName = nullptr) = 0;

	//! Saves all meshes
	/** \param suffix optional suffix
		\param allAtOnce whether to save all meshes in the same file or one mesh per file
		\return success
	**/
	virtual bool saveMeshes(QString suffix = QString(), bool allAtOnce = false, const QString* allAtOnceFileName = nullptr) = 0;

	//! Removes all clouds (or only the last one ;)
	virtual void removeClouds(bool onlyLast = false) = 0;

	//! Removes all meshes (or only the last one ;)
	virtual void removeMeshes(bool onlyLast = false) = 0;

	//! Returns the list of arguments
	virtual QStringList& arguments() = 0;
	//! Returns the list of arguments (const version)
	virtual const QStringList& arguments() const = 0;

	//! Returns a (shared) progress dialog (if any is available)
	virtual ccProgressDialog* progressDialog();
	//! Returns a (widget) parent (if any is available)
	virtual QDialog* widgetParent();

public: //file I/O

	//Extended file loading parameters
	struct CLLoadParameters : public FileIOFilter::LoadParameters
	{
		CLLoadParameters();

		bool m_coordinatesShiftEnabled;
		CCVector3d m_coordinatesShift;
	};

	//! File loading parameters
	virtual CLLoadParameters& fileLoadingParams();

	//! Loads a file with a specific filter
	/** Automatically dispatches the entities between the clouds and meshes sets.
	**/
	virtual bool importFile(QString filename, FileIOFilter::Shared filter = FileIOFilter::Shared(nullptr)) = 0;

	//! Returns the current cloud(s) export format
	virtual QString cloudExportFormat() const = 0;
	//! Returns the current cloud(s) export extension (warning: can be anything)
	virtual QString cloudExportExt() const = 0;
	
	//! Returns the current mesh(es) export format
	virtual QString meshExportFormat() const = 0;
	//! Returns the current mesh(es) export extension (warning: can be anything)
	virtual QString meshExportExt() const = 0;
	
	//! Returns the current hierarchy(ies) export format
	virtual QString hierarchyExportFormat() const = 0;
	//! Returns the current hierarchy(ies) export extension (warning: can be anything)
	virtual QString hierarchyExportExt() const = 0;

	//! Sets the current cloud(s) export format and extension
	virtual void setCloudExportFormat(QString format, QString ext) = 0;
	//! Sets the current mesh(es) export format and extension
	virtual void setMeshExportFormat(QString format, QString ext) = 0;
	//! Sets the current hierarchy(ies) export format and extension
	virtual void setHierarchyExportFormat(QString format, QString ext) = 0;

public: //logging

	//logging
	virtual void print(const QString& message) const = 0;
	virtual void warning(const QString& message) const = 0;
	virtual bool error(const QString& message) const = 0; //must always return false!

public: //access to data

	//! Currently opened point clouds and their filename
	virtual std::vector< CLCloudDesc >& clouds();
	//! Currently opened point clouds and their filename (const version)
	virtual const std::vector< CLCloudDesc >& clouds() const;

	//! Currently opened meshes and their filename
	virtual std::vector< CLMeshDesc >& meshes();
	//! Currently opened meshes and their filename (const version)
	virtual const std::vector< CLMeshDesc >& meshes() const;

	//! Toggles silent mode
	/** Must be called BEFORE calling start. **/
	void toggleSilentMode(bool state);
	//! Returns the silent mode
	bool silentMode() const;

	//! Sets whether files should be automatically saved (after each process) or not
	void toggleAutoSaveMode(bool state);
	//! Returns whether files should be automatically saved (after each process) or not
	bool autoSaveMode() const;

	//! Sets whether a timestamp should be automatically added to output files or not
	void toggleAddTimestamp(bool state);
	//! Returns whether a timestamp should be automatically added to output files or not
	bool addTimestamp() const;

	//! Sets the numerical precision
	void setNumericalPrecision(int p);
	//! Returns the numerical precision
	int numericalPrecision() const;

public: //Global shift management

	//! Returns whether Global (coordinate) shift has already been defined
	bool coordinatesShiftWasEnabled() const;
	//! Returns the Global (coordinate) shift (if already defined)
	const CCVector3d& formerCoordinatesShift() const;
	//! Sets whether Global (coordinate) shift is defined or not
	void storeCoordinatesShiftParams();

	bool nextCommandIsGlobalShift() const;

	//! Check the current command line argument stack against the 'COMMAND_OPEN_SHIFT_ON_LOAD' keyword and process the following commands if necessary
	/** \warning This method assumes the 'COMMAND_OPEN_SHIFT_ON_LOAD' argument has already been removed from the argument stack
	**/
	bool processGlobalShiftCommand();

protected: //members

	//! Currently opened point clouds and their filename
	std::vector< CLCloudDesc > m_clouds;

	//! Currently opened meshes and their filename
	std::vector< CLMeshDesc > m_meshes;

	//! Silent mode
	bool m_silentMode;

	//! Whether files should be automatically saved (after each process) or not
	bool m_autoSaveMode;

	//! Whether a timestamp should be automatically added to output files or not
	bool m_addTimestamp;

	//! Default numerical precision for ASCII output
	int m_precision;

	//! File loading parameters
	CLLoadParameters m_loadingParameters;

	//! Whether Global (coordinate) shift has already been defined
	bool m_coordinatesShiftWasEnabled;
	//! Global (coordinate) shift (if already defined)
	CCVector3d m_formerCoordinatesShift;
};

#endif //CC_COMMAND_LINE_INTERFACE_HEADER
