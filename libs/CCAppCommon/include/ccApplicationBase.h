#pragma once
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

#include "CCAppCommon.h"

//Qt
#include <QApplication>

//! Mimic Qt's qApp for easy access to the application instance
#define ccApp (static_cast<ccApplicationBase *>( QCoreApplication::instance() ))
//! ccApplicationBase
/*!
 * ccApplicationBase provides a QApplication class with additional Constructor param isCommandLine and version
 * The class also initialises openGL before instantiating the application class
 * \see QApplication
 */
class CCAPPCOMMON_LIB_API ccApplicationBase : public QApplication
{
public:
	//! This must be called before instantiating the application class so it
	//! can setup OpenGL first.
	static void	initOpenGL();

    //! ccApplicationBase Constructor
	/*!
	 * @param argc argc must be greater than zero, this is set by QT
	 * @param argv this contains the command line values, see QApplication, again this is set by QT
	 * @param isCommandLine boolean to define the application as either a Commandline or GUI. If true its a commandline application
	 * @param version string of the application version set by CloudCompare viewer or app.
	 */
	ccApplicationBase( int &argc, char **argv, bool isCommandLine, const QString &version );

    //! isCommandLine
    /*!
     * Method that confirms if it is running via commandline or not
     * @return bool
     */
	bool isCommandLine() const { return c_CommandLine; }
	
	QString versionStr() const;
	QString versionLongStr( bool includeOS ) const;
	
	const QString &translationPath() const;
	
private:
	void setupPaths();
		
	const QString c_VersionStr;
	
	QString	m_ShaderPath;
	QString	m_TranslationPath;
	QStringList m_PluginPaths;
	
	const bool c_CommandLine;
};
