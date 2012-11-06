#ifndef CC_COMMAND_LINE_PARSER_HEADER
#define CC_COMMAND_LINE_PARSER_HEADER

//Qt
#include <QString>

//STL
#include <vector>

class ccPointCloud;
class ccGenericMesh;
class QDialog;

//! Command line parser
class ccCommandLineParser
{
public:
	static int Parse(int nargs, char** args);

protected:

	//! Loaded cloud description
	struct CloudDesc
	{
		ccPointCloud* pc;
		QString filename;
		int indexInFile;

		CloudDesc(ccPointCloud* cloud = 0,
					const QString& file = QString(),
					int index = -1)
			: pc(cloud)
			, filename(file)
			, indexInFile(index)
		{
		}
	};

	//! Default constructor
	/** Shouldn't be called by user.
	**/
	ccCommandLineParser();

	//! Destructor
	/** Shouldn't be called by user.
	**/
	~ccCommandLineParser();

	//! Parses command line
	int parse(int nargs, char** args, bool silent, QDialog* dialog=0);

	//! Outputs error
	static int Error(const QString& message);

	//! Outputs message
	static void Print(const QString& message);

	//! Exports a cloud
	/** \return error string (if any)
	**/
	static QString Export2BIN(CloudDesc& cloudDesc, QString suffix);

    //! Removes all clouds
	void removeClouds();

    //! Removes all meshes
	void removeMeshes();

	//! Currently opened point clouds and their filename
	std::vector< CloudDesc > m_clouds;

	//! Currently opened meshes and their filename
	std::vector< std::pair<ccGenericMesh*,QString> > m_meshes;

	//! Mesh filename
	QString m_meshFilename;
};

#endif
