#ifndef FILEIO_H
#define FILEIO_H

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

#include <QString>

class FileIO
{
    public:
        static void setWriterInfo( const QString &applicationName, const QString &version );
        static QString writerInfo();

		static QString applicationName();
		static QString version();
		
		static QString createdBy();
		static QString createdDateTime();
		
    private:
        FileIO() = delete;

		static QString s_applicationName;
		static QString s_version;
		static QString s_writerInfo;
};
#endif
