//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qLasFWFIO                      #
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
//#                         COPYRIGHT: CNRS / OSUR                         #
//#                                                                        #
//##########################################################################

#ifndef LAS_FWF_IO_PLUGIN_COMMANDS_HEADER
#define LAS_FWF_IO_PLUGIN_COMMANDS_HEADER

#include "ccCommandLineInterface.h"

//Local
#include "Filter/LASFWFFilter.h"

static const char COMMAND_LOAD_FWF[]		= "FWF_O";
static const char COMMAND_SAVE_CLOUDS_FWF[]	= "FWF_SAVE_CLOUDS";
static const char OPTION_ALL_AT_ONCE[]		= "ALL_AT_ONCE";
static const char OPTION_COMPRESSED[]		= "COMPRESSED";

struct CommandLoadLASFWF : public ccCommandLineInterface::Command
{
	CommandLoadLASFWF() : ccCommandLineInterface::Command("Load FWF", COMMAND_LOAD_FWF) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[LOADING LAS]");
		if (cmd.arguments().empty())
		{
			return cmd.error(QString("Missing parameter: filename after \"-%1\"").arg(COMMAND_LOAD_FWF));
		}

		bool coordinatesShiftWasEnabled = cmd.coordinatesShiftWasEnabled();
		if (cmd.nextCommandIsGlobalShift())
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (!cmd.processGlobalShiftCommand())
			{
				//error message already issued
				return false;
			}
		}

		//open specified file
		QString filename(cmd.arguments().takeFirst());
		cmd.print(QString("Opening file: '%1'").arg(filename));

		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		FileIOFilter::Shared filter = FileIOFilter::GetFilter(LASFWFFilter::GetFileFilter(), true);
		if (!filter)
		{
			return cmd.error("Internal error: failed to load the LAS FWF I/O filter");
		}

		if (!cmd.importFile(filename, filter))
		{
			return false;
		}

		if (!coordinatesShiftWasEnabled)
		{
			//store semi-persistent parameters
			cmd.storeCoordinatesShiftParams();
		}

		return true;
	}
};

struct CommandSaveLASFWF : public ccCommandLineInterface::Command
{
	CommandSaveLASFWF() : ccCommandLineInterface::Command("Save clouds with FWF", COMMAND_SAVE_CLOUDS_FWF) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		bool allAtOnce = false;
		QString ext = "las";

		//look for additional parameters
		while (!cmd.arguments().empty())
		{
			QString argument = cmd.arguments().front();

			if (argument.toUpper() == OPTION_ALL_AT_ONCE)
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();
				allAtOnce = true;
			}
			else if (argument.toUpper() == OPTION_COMPRESSED)
			{
				//local option confirmed, we can move on
				cmd.arguments().pop_front();
				ext = "laz";
			}
			else
			{
				break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
			}
		}

		//backup the existing cloud export format first
		QString previousCloudExportFormat = cmd.cloudExportFormat();
		QString previousCloudExportExt = cmd.cloudExportExt();

		//force the export format to LAS FWF
		cmd.setCloudExportFormat(LASFWFFilter::GetFileFilter(), ext);

		bool success = cmd.saveClouds(QString(), allAtOnce);

		//restore the previous file output format
		cmd.setCloudExportFormat(previousCloudExportFormat, previousCloudExportExt);

		return success;
	}
};

#endif //LAS_FWF_IO_PLUGIN_COMMANDS_HEADER
