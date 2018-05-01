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

#include <QDebug>

#include "FileIO.h"


QString FileIO::s_writerInfo;

void FileIO::setWriterInfo( const QString &info )
{
    s_writerInfo = info;
}

QString FileIO::writerInfo()
{
	if ( s_writerInfo.isNull() )
	{
		qWarning() << "FileIO::setWriterInfo has not been called";
		
		return QStringLiteral( "(writer info not set)" );
	}
	
    return s_writerInfo;
}
