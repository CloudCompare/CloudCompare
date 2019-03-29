//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "FBXCommand.h"
#include "FBXFilter.h"

constexpr char COMMAND_FBX[] = "FBX";
constexpr char COMMAND_FBX_EXPORT_FORMAT[] = "EXPORT_FMT";


FBXCommand::FBXCommand() :
	Command( "FBX", COMMAND_FBX )
{
}

bool FBXCommand::process( ccCommandLineInterface &cmd )
{
	cmd.print( "[FBX]" );
	
	while ( !cmd.arguments().empty() )
	{
		const QString& arg = cmd.arguments().front();
		
		if ( ccCommandLineInterface::IsCommand( arg, COMMAND_FBX_EXPORT_FORMAT ) )
		{
			cmd.arguments().pop_front();

			QString format = cmd.arguments().takeFirst();
			
			if ( format.isNull() )
			{
				return cmd.error(QObject::tr("Missing parameter: FBX format (string) after '%1'").arg( COMMAND_FBX_EXPORT_FORMAT ));
			}
			
			cmd.print( QObject::tr( "FBX format: %1" ).arg( format ) );
			
			FBXFilter::SetDefaultOutputFormat( format );
		}
	}

	return true;	
}
