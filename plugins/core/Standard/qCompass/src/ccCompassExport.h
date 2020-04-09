//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: ccCompass                      #
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
//#                     COPYRIGHT: Sam Thiele  2017                        #
//#                                                                        #
//##########################################################################

#ifndef CCCOMPASSEXPORT_H
#define CCCOMPASSEXPORT_H

class QString;

class ccMainAppInterface;


namespace ccCompassExport
{
	void saveCSV( ccMainAppInterface *app, const QString &filename );
	void saveSVG( ccMainAppInterface *app, const QString &filename, float zoom );
	void saveXML( ccMainAppInterface *app, const QString &filename );
};

#endif // CCCOMPASSEXPORT_H
