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

class CCAPPCOMMON_LIB_API ccApplicationBase : public QApplication
{
public:
	//! This must be called before instantiating the application class so it
	//! can setup OpenGL first.
	static void	InitOpenGL();
	
	ccApplicationBase( int &argc, char **argv, bool isCommandLine, const QString &version );
	
	inline bool isCommandLine() const { return m_isCommandLine; }
	inline const QString& translationPath() const { return m_translationPath; }
	inline const QString& versionStr() const { return m_versionStr; }
	QString versionLongStr(bool includeOS) const;
	
	//! Set the application style (based on a QStyleFactory key)
	bool setAppStyle(QString styleKey);

private:
	void setupPaths();
		
	const QString m_versionStr;
	
	QString	m_shaderPath;
	QString	m_translationPath;
	QStringList m_pluginPaths;
	
	const bool m_isCommandLine;
};
