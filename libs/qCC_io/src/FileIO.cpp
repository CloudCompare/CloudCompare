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

#include <QDateTime>
#include <QDebug>

#include "FileIO.h"


QString FileIO::s_applicationName;
QString FileIO::s_version;
QString FileIO::s_writerInfo;

void FileIO::setWriterInfo( const QString &applicationName, const QString &version )
{
	s_applicationName = applicationName;
	s_version = version;
	s_writerInfo = QStringLiteral( "%1 v%2" ).arg( applicationName, version );
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

QString FileIO::applicationName()
{
	return s_applicationName;
}

QString FileIO::version()
{
	return s_version;
}

QString FileIO::createdBy()
{
	return QStringLiteral( "Created by %1" ).arg( FileIO::writerInfo() );
}

QString FileIO::createdDateTime()
{
	return QStringLiteral( "Created %1" ).arg( QDateTime::currentDateTime().toString( Qt::SystemLocaleShortDate ) );
}
