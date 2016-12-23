#ifndef CC_COMMAND_LINE_PARSER_HEADER
#define CC_COMMAND_LINE_PARSER_HEADER

//interface
#include "../plugins/ccCommandLineInterface.h"

//Local
#include "ccPluginInfo.h"

class ccProgressDialog;
class QDialog;

//! Command line parser
class ccCommandLineParser : public ccCommandLineInterface
{
public:

	//! Parses the input command
	static int Parse(int nargs, char** args, tPluginInfoList* plugins = 0);

	//! Destructor
	virtual ~ccCommandLineParser();

	//inherited from ccCommandLineInterface
	virtual QString exportEntity(	CLEntityDesc& entityDesc,
									QString suffix = QString(),
									QString* outputFilename = 0,
									bool forceIsCloud = false,
									bool forceNoTimestamp = false) override;
	virtual void removeClouds(bool onlyLast = false) override;
	virtual void removeMeshes(bool onlyLast = false) override;
	virtual QStringList& arguments() override { return m_arguments; }
	virtual const QStringList& arguments() const override { return m_arguments; }
	virtual bool registerCommand(Command::Shared command) override;
	virtual ccProgressDialog* progressDialog() override { return m_progressDialog; }
	virtual QDialog* widgetParent() override { return m_parentWidget; }
	virtual void print(const QString& message) const override;
	virtual void warning(const QString& message) const override;
	virtual bool error(const QString& message) const override; //must always return false!
	virtual bool saveClouds(QString suffix = QString(), bool allAtOnce = false) override;
	virtual bool saveMeshes(QString suffix = QString(), bool allAtOnce = false) override;
	virtual bool importFile(QString filename, FileIOFilter::Shared filter = FileIOFilter::Shared(0)) override;

protected: //other methods

	//! Default constructor
	/** Shouldn't be called by user.
	**/
	ccCommandLineParser();

	//! Parses the command line
	int start(QDialog* parent = 0);

public: //members (too annoying to make them private ;)

	//! Current cloud(s) export format (can be modified with the 'COMMAND_CLOUD_EXPORT_FORMAT' option)
	QString m_cloudExportFormat;
	//! Current cloud(s) export extension (warning: can be anything)
	QString m_cloudExportExt;
	//! Current mesh(es) export format (can be modified with the 'COMMAND_MESH_EXPORT_FORMAT' option)
	QString m_meshExportFormat;
	//! Current mesh(es) export extension (warning: can be anything)
	QString m_meshExportExt;

private: //members

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
