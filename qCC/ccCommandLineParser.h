#pragma once

// ##########################################################################
// #                                                                        #
// #                            CLOUDCOMPARE                                #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 of the License.               #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: CloudCompare project                      #
// #                                                                        #
// ##########################################################################

// interface
#include "ccCommandLineInterface.h"

// Local
#include "ccPluginManager.h"

class ccProgressDialog;
class QDialog;

//! Command line parser
class ccCommandLineParser : public ccCommandLineInterface
{
  public:
	//! Parses the input command
	static int Parse(const QStringList& arguments, ccPluginInterfaceList& plugins);

	//! Destructor
	~ccCommandLineParser() override;

	// inherited from ccCommandLineInterface
	QString      getExportFilename(const CLEntityDesc& entityDesc,
	                               QString             extension          = QString(),
	                               QString             suffix             = QString(),
	                               QString*            baseOutputFilename = nullptr,
	                               bool                forceNoTimestamp   = false) const override;
	QString      exportEntity(CLEntityDesc&                         entityDesc,
	                          const QString&                        suffix             = QString(),
	                          QString*                              baseOutputFilename = nullptr,
	                          ccCommandLineInterface::ExportOptions options            = ExportOption::NoOptions) override;
	void         removeClouds(bool onlyLast = false) override;
	void         removeMeshes(bool onlyLast = false) override;
	bool         selectClouds(const SelectEntitiesOptions& options) override;
	bool         selectMeshes(const SelectEntitiesOptions& options) override;
	QStringList& arguments() override
	{
		return m_arguments;
	}
	const QStringList& arguments() const override
	{
		return m_arguments;
	}
	bool     registerCommand(Command::Shared command) override;
	QDialog* widgetParent() override
	{
		return m_parentWidget;
	}
	void    printVerbose(const QString& message) const override;
	void    print(const QString& message) const override;
	void    printHigh(const QString& message) const override;
	void    printDebug(const QString& message) const override;
	void    warning(const QString& message) const override;
	void    warningDebug(const QString& message) const override;
	bool    error(const QString& message) const override;      // must always return false!
	bool    errorDebug(const QString& message) const override; // must always return false!
	bool    saveClouds(QString suffix = QString(), bool allAtOnce = false, const QString* allAtOnceFileName = nullptr) override;
	bool    saveMeshes(QString suffix = QString(), bool allAtOnce = false, const QString* allAtOnceFileName = nullptr) override;
	bool    importFile(QString filename, const GlobalShiftOptions& globalShiftOptions, FileIOFilter::Shared filter = FileIOFilter::Shared(nullptr)) override;
	void    setGlobalShiftOptions(const GlobalShiftOptions& globalShiftOptions) override;
	void    updateInteralGlobalShift(const GlobalShiftOptions& globalShiftOptions) override;
	QString cloudExportFormat() const override
	{
		return m_cloudExportFormat;
	}
	QString cloudExportExt() const override
	{
		return m_cloudExportExt;
	}
	QString meshExportFormat() const override
	{
		return m_meshExportFormat;
	}
	QString meshExportExt() const override
	{
		return m_meshExportExt;
	}
	QString hierarchyExportFormat() const override
	{
		return m_hierarchyExportFormat;
	}
	QString hierarchyExportExt() const override
	{
		return m_hierarchyExportExt;
	}
	void setCloudExportFormat(QString format, QString ext) override
	{
		m_cloudExportFormat = format;
		m_cloudExportExt    = ext;
	}
	void setMeshExportFormat(QString format, QString ext) override
	{
		m_meshExportFormat = format;
		m_meshExportExt    = ext;
	}
	void setHierarchyExportFormat(QString format, QString ext) override
	{
		m_hierarchyExportFormat = format;
		m_hierarchyExportExt    = ext;
	}

  protected: // other methods
	//! Default constructor
	/** Shouldn't be called by user.
	 **/
	ccCommandLineParser();

	void registerBuiltInCommands();

	void cleanup();

	//! Parses the command line
	int start(QDialog* parent = nullptr);

  private: // members
	//! Current cloud(s) export format (can be modified with the 'COMMAND_CLOUD_EXPORT_FORMAT' option)
	QString m_cloudExportFormat;
	//! Current cloud(s) export extension (warning: can be anything)
	QString m_cloudExportExt;
	//! Current mesh(es) export format (can be modified with the 'COMMAND_MESH_EXPORT_FORMAT' option)
	QString m_meshExportFormat;
	//! Current mesh(es) export extension (warning: can be anything)
	QString m_meshExportExt;
	//! Current hierarchy(ies) export format (can be modified with the 'COMMAND_HIERARCHY_EXPORT_FORMAT' option)
	QString m_hierarchyExportFormat;
	//! Current hierarchy(ies) export extension (warning: can be anything)
	QString m_hierarchyExportExt;

	//! Mesh filename
	QString m_meshFilename;

	//! Arguments
	QStringList m_arguments;

	//! Registered commands
	QMap<QString, Command::Shared> m_commands;

	//! Oprhan entities
	ccHObject m_orphans;

	//! Shared progress dialog
	ccProgressDialog* m_progressDialog;

	//! Widget parent
	QDialog* m_parentWidget;
};
