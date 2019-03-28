#ifndef CC_COMMAND_LINE_PARSER_HEADER
#define CC_COMMAND_LINE_PARSER_HEADER

//interface
#include "../plugins/ccCommandLineInterface.h"

//Local
#include "ccPluginManager.h"

class ccProgressDialog;
class QDialog;

//! Command line parser
class ccCommandLineParser : public ccCommandLineInterface
{
public:

	//! Parses the input command
	static int Parse(int nargs, char** args, ccPluginInterfaceList& plugins);

	//! Destructor
	~ccCommandLineParser() override;

	//inherited from ccCommandLineInterface
	QString getExportFilename(	const CLEntityDesc& entityDesc,
								QString extension = QString(),
								QString suffix = QString(),
								QString* baseOutputFilename = nullptr,
								bool forceNoTimestamp = false) const override;
	QString exportEntity(	CLEntityDesc& entityDesc,
							QString suffix = QString(),
							QString* baseOutputFilename = nullptr,
							bool forceIsCloud = false,
							bool forceNoTimestamp = false) override;
	void removeClouds(bool onlyLast = false) override;
	void removeMeshes(bool onlyLast = false) override;
	QStringList& arguments() override { return m_arguments; }
	const QStringList& arguments() const override { return m_arguments; }
	bool registerCommand(Command::Shared command) override;
	QDialog* widgetParent() override { return m_parentWidget; }
	void print(const QString& message) const override;
	void warning(const QString& message) const override;
	bool error(const QString& message) const override; //must always return false!
	bool saveClouds(QString suffix = QString(), bool allAtOnce = false, const QString* allAtOnceFileName = nullptr) override;
	bool saveMeshes(QString suffix = QString(), bool allAtOnce = false, const QString* allAtOnceFileName = nullptr) override;
	bool importFile(QString filename, FileIOFilter::Shared filter = FileIOFilter::Shared(nullptr)) override;
	QString cloudExportFormat() const override { return m_cloudExportFormat; }
	QString cloudExportExt() const override { return m_cloudExportExt; }
	QString meshExportFormat() const override { return m_meshExportFormat; }
	QString meshExportExt() const override { return m_meshExportExt; }
	void setCloudExportFormat(QString format, QString ext) override { m_cloudExportFormat = format; m_cloudExportExt = ext; }
	void setMeshExportFormat(QString format, QString ext) override { m_meshExportFormat = format; m_meshExportExt = ext; }

protected: //other methods

	//! Default constructor
	/** Shouldn't be called by user.
	**/
	ccCommandLineParser();
   
   void  registerBuiltInCommands();
   
   void  cleanup();

	//! Parses the command line
	int start(QDialog* parent = nullptr);

private: //members

	//! Current cloud(s) export format (can be modified with the 'COMMAND_CLOUD_EXPORT_FORMAT' option)
	QString m_cloudExportFormat;
	//! Current cloud(s) export extension (warning: can be anything)
	QString m_cloudExportExt;
	//! Current mesh(es) export format (can be modified with the 'COMMAND_MESH_EXPORT_FORMAT' option)
	QString m_meshExportFormat;
	//! Current mesh(es) export extension (warning: can be anything)
	QString m_meshExportExt;

	//! Mesh filename
	QString m_meshFilename;

	//! Arguments
	QStringList m_arguments;

	//! Registered commands
	QMap< QString, Command::Shared > m_commands;

	//! Oprhan entities
	ccHObject m_orphans;

	//! Shared progress dialog
	ccProgressDialog* m_progressDialog;

	//! Widget parent
	QDialog* m_parentWidget;
};

#endif
