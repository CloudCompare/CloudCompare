#ifndef CC_COMMAND_LINE_PARSER_HEADER
#define CC_COMMAND_LINE_PARSER_HEADER

//qCC_io
#include <FileIOFilter.h>

//qCC_db
#include <ccGenericMesh.h>
#include <ccPointCloud.h>

//Qt
#include <QString>

class ccProgressDialog;
class QDialog;

//! Command line parser
class ccCommandLineParser
{
public:
	static int Parse(int nargs, char** args);

protected:

	bool commandLoad						(QStringList& arguments);
	bool commandSubsample					(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool commandCurvature					(QStringList& arguments, QDialog* parent = 0);
	bool commandDensity						(QStringList& arguments, QDialog* parent = 0);
	bool commandApproxDensity				(QStringList& arguments, QDialog* parent = 0);
	bool commandSFGradient					(QStringList& arguments, QDialog* parent = 0);
	bool commandRoughness					(QStringList& arguments, QDialog* parent = 0);
	bool commandSampleMesh					(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool commandBundler						(QStringList& arguments);
	bool commandDist						(QStringList& arguments, bool cloud2meshDist, QDialog* parent = 0);
	bool commandFilterSFByValue				(QStringList& arguments);
	bool commandMergeClouds					(QStringList& arguments);
	bool commandStatTest					(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool commandBestFitPlane				(QStringList& arguments);
	bool commandCrop						(QStringList& arguments);
	bool commandCrop2D						(QStringList& arguments);
	bool commandCrossSection				(QStringList& arguments, QDialog* parent = 0);
	bool commandColorBanding				(QStringList& arguments);
	bool matchBBCenters						(QStringList& arguments);
	bool commandSfArithmetic				(QStringList& arguments);
	bool commandSfOp						(QStringList& arguments);
	bool commandICP							(QStringList& arguments, QDialog* parent = 0);
	bool commandDelaunay					(QStringList& arguments, QDialog* parent = 0);
	bool commandChangeCloudOutputFormat		(QStringList& arguments);
	bool commandChangeMeshOutputFormat		(QStringList& arguments);
	bool commandChangePLYExportFormat		(QStringList& arguments);
	bool commandChangeFBXOutputFormat		(QStringList& arguments);
	bool commandForceNormalsComputation		(QStringList& arguments);
	bool commandSaveClouds					(QStringList& arguments);
	bool commandSaveMeshes					(QStringList& arguments);
	bool commandAutoSave					(QStringList& arguments);
	bool setActiveSF						(QStringList& arguments);
	bool removeAllSFs						(QStringList& arguments);
	bool commandApplyTransformation			(QStringList& arguments);
	bool commandLogFile						(QStringList& arguments);
	bool commandSORFilter					(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool commandOrientNormalsMST			(QStringList& arguments, ccProgressDialog* pDlg = 0);
	bool commandDropGlobalShift				(QStringList& arguments);

protected:

	//! Default constructor
	/** Shouldn't be called by user.
	**/
	ccCommandLineParser();

	//! Destructor
	/** Shouldn't be called by user.
	**/
	~ccCommandLineParser();

	//! Parses command line
	int parse(QStringList& arguments, QDialog* parent = 0);

	//! Loaded entity description
	struct EntityDesc
	{
		QString basename;
		QString path;
		int indexInFile;

		EntityDesc(QString filename, int _indexInFile =-1);
		EntityDesc(QString baseName, QString path, int _indexInFile =-1);
		virtual ccHObject* getEntity() = 0;
	};

	//! Loaded group description
	struct GroupDesc : EntityDesc
	{
		ccHObject* groupEntity;

		GroupDesc(	ccHObject* group,
					QString basename,
					QString path = QString())
			: EntityDesc(basename, path)
			, groupEntity(group)
		{}

		virtual ccHObject* getEntity() { return groupEntity; }
	};

	//! Loaded cloud description
	struct CloudDesc : EntityDesc
	{
		ccPointCloud* pc;

		CloudDesc()
			: EntityDesc(QString())
			, pc(0)
		{}

		CloudDesc(	ccPointCloud* cloud,
					QString filename,
					int index = -1)
			: EntityDesc(filename,index)
			, pc(cloud)
		{}

		CloudDesc(	ccPointCloud* cloud,
					QString basename,
					QString path,
					int index = -1)
			: EntityDesc(basename,path,index)
			, pc(cloud)
		{}
		
		virtual ccHObject* getEntity() { return static_cast<ccHObject*>(pc); }
	};

	//! Loaded mesh description
	struct MeshDesc : EntityDesc
	{
		ccGenericMesh* mesh;

		MeshDesc()
			: EntityDesc(QString())
			, mesh(0)
		{}

		MeshDesc(	ccGenericMesh* _mesh,
					QString filename,
					int index = -1)
			: EntityDesc(filename,index)
			, mesh(_mesh)
		{}

		MeshDesc(	ccGenericMesh* _mesh,
					QString basename,
					QString path,
					int index = -1)
			: EntityDesc(basename,path,index)
			, mesh(_mesh)
		{}

		virtual ccHObject* getEntity() { return static_cast<ccHObject*>(mesh); }
	};

	//! Exports a cloud or a mesh
	/** \return error string (if any)
	**/
	static QString Export(EntityDesc& cloudDesc, QString suffix = QString(), QString* outputFilename = 0, bool forceIsCloud = false);

	//! Reads out file format
	static QString GetFileFormatFilter(QStringList& arguments, QString& defaultExt);

	//! Saves all clouds
	/** \param suffix optional suffix
		\param allAtOnce whether to save all clouds in the same file or one cloud per file
		\return success
	**/
	bool saveClouds(QString suffix = QString(), bool allAtOnce = false);

	//! Saves all meshes
	/** \param suffix optional suffix
		\param allAtOnce whether to save all meshes in the same file or one mesh per file
		\return success
	**/
	bool saveMeshes(QString suffix = QString(), bool allAtOnce = false);

	//! Removes all clouds (or only the last one ;)
	void removeClouds(bool onlyLast = false);

	//! Removes all meshes (or only the last one ;)
	void removeMeshes(bool onlyLast = false);

	//! Currently opened point clouds and their filename
	std::vector< CloudDesc > m_clouds;

	//! Currently opened meshes and their filename
	std::vector< MeshDesc > m_meshes;

	//! Oprhan entities
	ccHObject m_orphans;

	//! Mesh filename
	QString m_meshFilename;
};

#endif
