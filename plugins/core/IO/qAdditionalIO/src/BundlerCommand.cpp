//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN                               #
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

#include "BundlerCommand.h"
#include "BundlerFilter.h"

static const char COMMAND_BUNDLER[]					= "BUNDLER_IMPORT"; //Import Bundler file + orthorectification
static const char COMMAND_BUNDLER_ALT_KEYPOINTS[]	= "ALT_KEYPOINTS";
static const char COMMAND_BUNDLER_SCALE_FACTOR[]	= "SCALE_FACTOR";
static const char COMMAND_BUNDLER_UNDISTORT[]		= "UNDISTORT";
static const char COMMAND_BUNDLER_COLOR_DTM[]		= "COLOR_DTM";


BundlerCommand::BundlerCommand() :
    ccCommandLineInterface::Command( "Bundler", COMMAND_BUNDLER )
{
}

bool BundlerCommand::process( ccCommandLineInterface &cmd )
{
	cmd.print("[BUNDLER]");
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: filename after \"-%1\"").arg(COMMAND_BUNDLER));

	//open specified file
	QString bundlerFilename(cmd.arguments().takeFirst());
	cmd.print(QObject::tr("Importing Bundler file: '%1'").arg(bundlerFilename));

	QString altKeypointsFilename;
	bool undistortImages = false;
	bool generateColoredDTM = false;
	unsigned coloredDTMVerticesCount = 0;
	float scaleFactor = 1.0f;

	//inner loop for Bundler import options
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_BUNDLER_ALT_KEYPOINTS))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: filename after \"-%1\"").arg(COMMAND_BUNDLER_ALT_KEYPOINTS));
			altKeypointsFilename = cmd.arguments().takeFirst();
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_BUNDLER_SCALE_FACTOR))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: value after \"-%1\"").arg(COMMAND_BUNDLER_SCALE_FACTOR));
			bool conversionOk = false;
			scaleFactor = cmd.arguments().takeFirst().toFloat(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_BUNDLER_SCALE_FACTOR));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_BUNDLER_UNDISTORT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			undistortImages = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_BUNDLER_COLOR_DTM))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: vertices count after \"-%1\"").arg(COMMAND_BUNDLER_COLOR_DTM));
			bool conversionOk = false;
			coloredDTMVerticesCount = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: vertices count after \"-%1\"").arg(COMMAND_BUNDLER_COLOR_DTM));
			generateColoredDTM = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}

	ccHObject tempContainer;
	FileIOFilter::LoadParameters parameters;
	parameters.alwaysDisplayLoadDialog = false;
	BundlerFilter().loadFileExtended(	qPrintable(bundlerFilename),
	                                    tempContainer,
	                                    parameters,
	                                    altKeypointsFilename,
	                                    undistortImages,
	                                    generateColoredDTM,
	                                    coloredDTMVerticesCount,
	                                    scaleFactor);
	return true;
}
