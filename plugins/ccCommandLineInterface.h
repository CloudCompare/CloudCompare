#ifndef CC_COMMAND_LINE_INTERFACE_HEADER
#define CC_COMMAND_LINE_INTERFACE_HEADER

//qCC_db
#include <ccGenericMesh.h>
#include <ccPointCloud.h>

//qCC_io
#include <FileIOFilter.h>

//Qt
#include <QDir>
#include <QSharedPointer>
#include <QString>
#include <QStringList>

//System
#include <vector>

class ccProgressDialog;
class QDialog;

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

	CLEntityDesc( const QString &name )
		: basename( name )
		, path( QDir::currentPath() )
		, indexInFile( -1 )
	{	
	}
	
	CLEntityDesc(const QString &filename, int _indexInFile)
		: indexInFile(_indexInFile)
	{
		if (filename.isNull())
		{
			basename = "unknown";
			path = QDir::currentPath();
		}
		else
		{
			QFileInfo fi(filename);
			basename = fi.completeBaseName();
			path = fi.path();
		}
	}
	
	CLEntityDesc(const QString &_basename, const QString &_path, int _indexInFile = -1)
		: basename(_basename)
		, path(_path)
		, indexInFile(_indexInFile)
	{
	}
	
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
				const QString& path = QString())
		: CLEntityDesc(basename, path)
		, groupEntity(group)
	{}

	~CLGroupDesc() override = default;
	
	ccHObject* getEntity() override { return groupEntity; }
	const ccHObject* getEntity() const override { return groupEntity; }
	CL_ENTITY_TYPE getCLEntityType() const override { return CL_ENTITY_TYPE::GROUP; }
};

//! Loaded cloud description
struct CLCloudDesc : CLEntityDesc
{
	ccPointCloud* pc;

	CLCloudDesc()
		: CLEntityDesc("Unnamed cloud")
		, pc( nullptr )
	{}

	CLCloudDesc(ccPointCloud* cloud,
				const QString& filename = QString(),
				int index = -1)
		: CLEntityDesc(filename, index)
		, pc(cloud)
	{}

	CLCloudDesc(ccPointCloud* cloud,
				const QString& basename,
				const QString& path,
				int index = -1)
		: CLEntityDesc(basename, path, index)
		, pc(cloud)
	{}

	~CLCloudDesc() override = default;

	ccHObject* getEntity() override { return static_cast<ccHObject*>(pc); }
	const ccHObject* getEntity() const override { return static_cast<ccHObject*>(pc); }
	CL_ENTITY_TYPE getCLEntityType() const override { return CL_ENTITY_TYPE::CLOUD; }
};

//! Loaded mesh description
struct CLMeshDesc : CLEntityDesc
{
	ccGenericMesh* mesh;

	CLMeshDesc()
		: CLEntityDesc("Unnamed mesh")
		, mesh( nullptr )
	{}

	CLMeshDesc(	ccGenericMesh* _mesh,
				const QString& filename = QString(),
				int index = -1)
		: CLEntityDesc(filename, index)
		, mesh(_mesh)
	{}

	CLMeshDesc(	ccGenericMesh* _mesh,
				const QString& basename,
				const QString& path,
				int index = -1)
		: CLEntityDesc(basename, path, index)
		, mesh(_mesh)
	{}

	~CLMeshDesc() override = default;
	
	ccHObject* getEntity() override { return static_cast<ccHObject*>(mesh); }
	const ccHObject* getEntity() const override { return static_cast<ccHObject*>(mesh); }
	CL_ENTITY_TYPE getCLEntityType() const override { return CL_ENTITY_TYPE::MESH; }
};

//! Command line interface
class ccCommandLineInterface
{
public: //constructor

	//! Default constructor
	ccCommandLineInterface()
		: m_silentMode(false)
		, m_autoSaveMode(true)
		, m_addTimestamp(true)
		, m_precision(12)
		, m_coordinatesShiftWasEnabled(false)
	{}
	
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
		Command(const QString& name, const QString& keyword)
			: m_name(name)
			, m_keyword(keyword)
		{}

		virtual ~Command() = default;
		
		//! Main process
		virtual bool process(ccCommandLineInterface& cmd) = 0;

		//! Command name
		QString m_name;
		//! Command keyword
		QString m_keyword;
	};

	//! Test whether a command line token is a valid command keyword or not
	static bool IsCommand(const QString& token, const char* command)
	{
		return token.startsWith("-") && token.mid(1).toUpper() == QString(command);
	}

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
	virtual ccProgressDialog* progressDialog() { return nullptr; }
	//! Returns a (widget) parent (if any is available)
	virtual QDialog* widgetParent() { return nullptr; }

public: //file I/O

	//Extended file loading parameters
	struct CLLoadParameters : public FileIOFilter::LoadParameters
	{
		CLLoadParameters()
			: FileIOFilter::LoadParameters()
			, m_coordinatesShiftEnabled(false)
			, m_coordinatesShift(0, 0, 0)
		{
			shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
			alwaysDisplayLoadDialog = false;
			autoComputeNormals = false;
			coordinatesShiftEnabled = &m_coordinatesShiftEnabled;
			coordinatesShift = &m_coordinatesShift;
		}

		bool m_coordinatesShiftEnabled;
		CCVector3d m_coordinatesShift;
	};

	//! File loading parameters
	virtual CLLoadParameters& fileLoadingParams() { return m_loadingParameters; }

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
	virtual std::vector< CLCloudDesc >& clouds() { return m_clouds; }
	//! Currently opened point clouds and their filename (const version)
	virtual const std::vector< CLCloudDesc >& clouds() const { return m_clouds; }

	//! Currently opened meshes and their filename
	virtual std::vector< CLMeshDesc >& meshes() { return m_meshes; }
	//! Currently opened meshes and their filename (const version)
	virtual const std::vector< CLMeshDesc >& meshes() const { return m_meshes; }

	//! Toggles silent mode
	/** Must be called BEFORE calling start.
	**/
	void toggleSilentMode(bool state) { m_silentMode = state; }
	//! Returns the silent mode
	bool silentMode() const { return m_silentMode; }

	//! Sets whether files should be automatically saved (after each process) or not
	void toggleAutoSaveMode(bool state) { m_autoSaveMode = state; }
	//! Returns whether files should be automatically saved (after each process) or not
	bool autoSaveMode() const { return m_autoSaveMode; }

	//! Sets whether a timestamp should be automatically added to output files or not
	void toggleAddTimestamp(bool state) { m_addTimestamp = state; }
	//! Returns whether a timestamp should be automatically added to output files or not
	bool addTimestamp() const { return m_addTimestamp; }

	//! Sets the numerical precision
	void setNumericalPrecision(int p) { m_precision = p; }
	//! Returns the numerical precision
	int numericalPrecision() const { return m_precision; }

public: //Global shift management

	//! Returns whether Global (coordinate) shift has already been defined
	bool coordinatesShiftWasEnabled() const { return m_coordinatesShiftWasEnabled; }
	//! Returns the Global (coordinate) shift (if already defined)
	const CCVector3d& formerCoordinatesShift() const { return m_formerCoordinatesShift; }
	//! Sets whether Global (coordinate) shift is defined or not
	void storeCoordinatesShiftParams() { m_coordinatesShiftWasEnabled = m_loadingParameters.m_coordinatesShiftEnabled; m_formerCoordinatesShift = m_loadingParameters.m_coordinatesShift; }

	static const char* COMMAND_OPEN_SHIFT_ON_LOAD()			{ return "GLOBAL_SHIFT"; }	//!< Global shift
	static const char* COMMAND_OPEN_SHIFT_ON_LOAD_AUTO()	{ return "AUTO"; }			//!< "AUTO" keyword
	static const char* COMMAND_OPEN_SHIFT_ON_LOAD_FIRST()	{ return "FIRST"; }			//!< "FIRST" keyword

	bool nextCommandIsGlobalShift() const { return !arguments().empty() && IsCommand(arguments().front(), COMMAND_OPEN_SHIFT_ON_LOAD()); }

	//! Check the current command line argument stack against the 'COMMAND_OPEN_SHIFT_ON_LOAD' keyword and process the following commands if necessary
	/** \warning This method assumes the 'COMMAND_OPEN_SHIFT_ON_LOAD' argument has already been removed from the argument stack
	**/
	bool processGlobalShiftCommand()
	{
		if (arguments().empty())
		{
			return error(QObject::tr("Missing parameter: global shift vector or %1 or %2 after '%3'").arg(COMMAND_OPEN_SHIFT_ON_LOAD_AUTO(), COMMAND_OPEN_SHIFT_ON_LOAD_FIRST(), COMMAND_OPEN_SHIFT_ON_LOAD()));
		}

		QString firstParam = arguments().takeFirst();

		m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG;
		m_loadingParameters.m_coordinatesShiftEnabled = false;
		m_loadingParameters.m_coordinatesShift = CCVector3d(0, 0, 0);

		if (firstParam.toUpper() == COMMAND_OPEN_SHIFT_ON_LOAD_AUTO())
		{
			//let CC handle the global shift automatically
			m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
		}
		else if (firstParam.toUpper() == COMMAND_OPEN_SHIFT_ON_LOAD_FIRST())
		{
			//use the first encountered global shift value (if any)
			m_loadingParameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
			m_loadingParameters.m_coordinatesShiftEnabled = m_coordinatesShiftWasEnabled;
			m_loadingParameters.m_coordinatesShift = m_formerCoordinatesShift;
		}
		else if (arguments().size() < 2)
		{
			return error(QObject::tr("Missing parameter: global shift vector after '%1' (3 values expected)").arg(COMMAND_OPEN_SHIFT_ON_LOAD()));
		}
		else
		{
			bool ok = true;
			CCVector3d shiftOnLoadVec;
			shiftOnLoadVec.x = firstParam.toDouble(&ok);
			if (!ok)
				return error(QObject::tr("Invalid parameter: X coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SHIFT_ON_LOAD()));
			shiftOnLoadVec.y = arguments().takeFirst().toDouble(&ok);
			if (!ok)
				return error(QObject::tr("Invalid parameter: Y coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SHIFT_ON_LOAD()));
			shiftOnLoadVec.z = arguments().takeFirst().toDouble(&ok);
			if (!ok)
				return error(QObject::tr("Invalid parameter: Z coordinate of the global shift vector after '%1'").arg(COMMAND_OPEN_SHIFT_ON_LOAD()));

			//set the user defined shift vector as default shift information
			m_loadingParameters.m_coordinatesShiftEnabled = true;
			m_loadingParameters.m_coordinatesShift = shiftOnLoadVec;
		}

		return true;
	}

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
