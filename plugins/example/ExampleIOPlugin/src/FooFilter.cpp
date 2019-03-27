//##########################################################################
//#                                                                        #
//#                           ExampleIOPlugin                              #
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

#include <QString>

#include "FooFilter.h"


FooFilter::FooFilter()
	: FileIOFilter( {
					"Foo Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "foo", "txt" },
					"foo",
					QStringList{ "Foo file (*.foo)", "Text file (*.txt)" },
					QStringList(),
					Import
					} )
{
}

CC_FILE_ERROR FooFilter::loadFile( const QString &fileName, ccHObject &container, LoadParameters &parameters )
{	
	Q_UNUSED( container );
	Q_UNUSED( parameters );
	
	QFile file( fileName );
	
	if ( !file.open( QIODevice::ReadOnly ) )
	{
		return CC_FERR_READING;
	}
	
	QTextStream stream( &file );
	
	// ...do some stuff with the contents of the file
	
	// In this example, we treat the file as text and count the occurenecs of the string 'foo'
	QString line;
	int count = 0;
	
	while ( stream.readLineInto( &line ) )
	{
		if ( line.contains( QStringLiteral( "foo" ), Qt::CaseInsensitive ) )
		{
			++count;
		}
	}
	
	ccLog::Print( QStringLiteral( "[foo] The file %1 has %2 lines containing 'foo'" ).arg( file.fileName(), QString::number( count ) ) );
	
	return CC_FERR_NO_ERROR;
}

bool FooFilter::canSave( CC_CLASS_ENUM type, bool &multiple, bool &exclusive ) const
{
	Q_UNUSED( type );
	Q_UNUSED( multiple );
	Q_UNUSED( exclusive );

	// ... can we save this?
	return false;
}
