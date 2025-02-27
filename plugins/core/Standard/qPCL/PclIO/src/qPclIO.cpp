//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qPclIO                       #
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
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#include "qPclIO.h"

//Local
#include "PcdFilter.h"

//CC
#include <ccCommandLineInterface.h>

//Qt
#include <QtPlugin>

qPclIO::qPclIO(QObject *parent)
    : QObject(parent)
    , ccIOPluginInterface(":/CC/plugin/qPclIO/info.json")
{
}

// Command line command to set the PCD output file format
ccIOPluginInterface::FilterList qPclIO::getFilters()
{
    return { FileIOFilter::Shared( new PcdFilter ) };
}

constexpr char COMMAND_PCD_OUTPUT_FORMAT[] = "PCD_OUTPUT_FORMAT";
constexpr char OPTION_PCD_OUTPUT_FORMAT_CBINARY[] = "COMPRESSED_BINARY";
constexpr char OPTION_PCD_OUTPUT_FORMAT_BINARY[] = "BINARY";
constexpr char OPTION_PCD_OUTPUT_FORMAT_ASCII[] = "ASCII";

class PCDCommand : public ccCommandLineInterface::Command
{
public:
	PCDCommand(): ccCommandLineInterface::Command("Set PCD output format", COMMAND_PCD_OUTPUT_FORMAT)	{}

	~PCDCommand() override = default;

	bool process(ccCommandLineInterface& cmd) override
	{
		if (cmd.arguments().size() == 0)
		{
			return cmd.error(QObject::tr("Missing argument after %1: output format type").arg(COMMAND_PCD_OUTPUT_FORMAT));
		}

		QString format = cmd.arguments().takeFirst().toUpper();

		if (format == OPTION_PCD_OUTPUT_FORMAT_CBINARY)
		{
			cmd.print(QObject::tr("PCD output format: compressed binary"));
			PcdFilter::SetOutputFileFormat(PcdFilter::COMPRESSED_BINARY);
		}
		else if (format == OPTION_PCD_OUTPUT_FORMAT_BINARY)
		{
			cmd.print(QObject::tr("PCD output format: binary"));
			PcdFilter::SetOutputFileFormat(PcdFilter::BINARY);
		}
		else if (format == OPTION_PCD_OUTPUT_FORMAT_ASCII)
		{
			cmd.print(QObject::tr("PCD output format: ASCII/text"));
			PcdFilter::SetOutputFileFormat(PcdFilter::ASCII);
		}
		else
		{
			return cmd.error(QObject::tr("Unknown PCD format: ") + format);
		}

		return true;
	}
};

void qPclIO::registerCommands(ccCommandLineInterface* cmd)
{
	if (cmd)
	{
		cmd->registerCommand(ccCommandLineInterface::Command::Shared(new PCDCommand));
	}
}
