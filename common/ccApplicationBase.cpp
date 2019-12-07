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

#include <clocale>

//Qt
#include <QDir>
#include <QStandardPaths>
#include <QString>
#include <QSurfaceFormat>
#include <QTranslator>
#include <QtGlobal>

// CCLib
#include "CCPlatform.h"

// qCC_db
#include "ccMaterial.h"

// qCC_glWindow
#include "ccGLWindow.h"

//Common
#include "ccApplicationBase.h"
#include "ccPluginManager.h"
#include "ccTranslationManager.h"


#if (QT_VERSION < QT_VERSION_CHECK(5, 5, 0))
#error CloudCompare does not support versions of Qt prior to 5.5
#endif


void ccApplicationBase::initOpenGL()
{
	//See http://doc.qt.io/qt-5/qopenglwidget.html#opengl-function-calls-headers-and-qopenglfunctions
	/** Calling QSurfaceFormat::setDefaultFormat() before constructing the QApplication instance is mandatory
		on some platforms (for example, OS X) when an OpenGL core profile context is requested. This is to
		ensure that resource sharing between contexts stays functional as all internal contexts are created
		using the correct version and profile.
	**/
	{
		QSurfaceFormat format = QSurfaceFormat::defaultFormat();

		format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
		format.setStencilBufferSize(0);

#ifdef CC_GL_WINDOW_USE_QWINDOW
		format.setStereo(true);
#endif

#ifdef Q_OS_MAC
		format.setVersion(2, 1);	// must be 2.1 - see ccGLWindow::functions()
		format.setProfile(QSurfaceFormat::CoreProfile);
#endif

#ifdef QT_DEBUG
		format.setOption(QSurfaceFormat::DebugContext, true);
#endif

		QSurfaceFormat::setDefaultFormat(format);
	}

	// The 'AA_ShareOpenGLContexts' attribute must be defined BEFORE the creation of the Q(Gui)Application
	// DGM: this is mandatory to enable exclusive full screen for ccGLWidget (at least on Windows)
	QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
}

ccApplicationBase::ccApplicationBase(int &argc, char **argv, bool isCommandLine, const QString &version)
    : QApplication( argc, argv )
    , c_VersionStr( version )
	, c_CommandLine( isCommandLine )
{
	setOrganizationName( "CCCorp" );

	setupPaths();

#ifdef Q_OS_WIN
	//enables automatic scaling based on the monitor's pixel density
	setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
		
#ifdef Q_OS_MAC
	// Mac OS X apps don't show icons in menus
	setAttribute( Qt::AA_DontShowIconsInMenus );
#endif

	// Force 'english' locale so as to get a consistent behavior everywhere
	QLocale::setDefault( QLocale::English );

#ifdef Q_OS_UNIX
	// We reset the numeric locale for POSIX functions
	// See https://doc.qt.io/qt-5/qcoreapplication.html#locale-settings
	setlocale( LC_NUMERIC, "C" );
#endif
	
	ccGLWindow::setShaderPath( m_ShaderPath );
	ccPluginManager::get().setPaths( m_PluginPaths );
	
	ccTranslationManager::get().registerTranslatorFile( QStringLiteral( "qt" ), m_TranslationPath );
	ccTranslationManager::get().registerTranslatorFile( QStringLiteral( "CloudCompare" ), m_TranslationPath );
	ccTranslationManager::get().loadTranslations();
	
	connect( this, &ccApplicationBase::aboutToQuit, [=](){ ccMaterial::ReleaseTextures(); } );
}

QString ccApplicationBase::versionStr() const
{
	return c_VersionStr;
}

QString ccApplicationBase::versionLongStr( bool includeOS ) const
{
	QString verStr = c_VersionStr;

#ifdef CC_GL_WINDOW_USE_QWINDOW
	verStr += QStringLiteral( " Stereo" );
#endif

#if defined(CC_ENV_64)
	const QString arch( "64-bit" );
#elif defined(CC_ENV_32)
	const QString arch( "32-bit" );
#else
	const QString arch( "\?\?-bit" );
#endif

	if ( includeOS )
	{
#if defined(CC_WINDOWS)
		const QString platform( "Windows" );
#elif defined(CC_MAC_OS)
		const QString platform( "macOS" );
#elif defined(CC_LINUX)
		const QString platform( "Linux" );
#else
		const QString platform( "Unknown OS" );
#endif
		verStr += QStringLiteral( " [%1 %2]" ).arg( platform, arch );
	}
	else
	{
		verStr += QStringLiteral( " [%1]" ).arg( arch );
	}

#ifdef QT_DEBUG
	verStr += QStringLiteral( " [DEBUG]" );
#endif

	return verStr;
}

const QString &ccApplicationBase::translationPath() const
{
	return m_TranslationPath;
}

void ccApplicationBase::setupPaths()
{
	QDir  appDir = QCoreApplication::applicationDirPath();

	// Set up our shader and plugin paths
#if defined(Q_OS_MAC)
	QDir  bundleDir = appDir;

	if ( bundleDir.dirName() == "MacOS" )
	{
		bundleDir.cdUp();
	}

	m_PluginPaths << (bundleDir.absolutePath() + "/PlugIns/ccPlugins");

#if defined(CC_MAC_DEV_PATHS)
	// Used for development only - this is the path where the plugins are built
	// and the shaders are located.
	// This avoids having to install into the application bundle when developing.
	bundleDir.cdUp();
	bundleDir.cdUp();
	bundleDir.cdUp();

	m_PluginPaths << (bundleDir.absolutePath() + "/ccPlugins");
	m_ShaderPath = (bundleDir.absolutePath() + "/shaders");
	m_TranslationPath = (bundleDir.absolutePath() + "/qCC/translations");
#else
	m_ShaderPath = (bundleDir.absolutePath() + "/Shaders");
	m_TranslationPath = (bundleDir.absolutePath() + "/translations");
#endif
#elif defined(Q_OS_WIN)
	m_PluginPaths << (appDir.absolutePath() + "/plugins");
	m_ShaderPath = (appDir.absolutePath() + "/shaders");
	m_TranslationPath = (appDir.absolutePath() + "/translations");
#elif defined(Q_OS_LINUX)
	// Shaders & plugins are relative to the bin directory where the executable is found
	QDir  theDir = appDir;

	if ( theDir.dirName() == "bin" )
	{
		theDir.cdUp();

		m_PluginPaths << (theDir.absolutePath() + "/lib/cloudcompare/plugins");
		m_ShaderPath = (theDir.absolutePath() + "/share/cloudcompare/shaders");
		m_TranslationPath = (theDir.absolutePath() + "/share/cloudcompare/translations");
	}
	else
	{
		// Choose a reasonable default to look in
		m_PluginPaths << "/usr/lib/cloudcompare/plugins";
		m_ShaderPath = "/usr/share/cloudcompare/shaders";
		m_TranslationPath = "/usr/share/cloudcompare/translations";
	}
#else
#warning Need to specify the shader path for this OS.
#endif

	// Add any app data paths to plugin paths
	// Plugins in these directories take precendence over the included ones
	// This allows users to put plugins outside of the install directories.
	const QStringList appDataPaths = QStandardPaths::standardLocations( QStandardPaths::AppDataLocation );

	for ( const QString &appDataPath : appDataPaths )
	{
		QString path = appDataPath + "/plugins";

		if (!m_PluginPaths.contains(path)) //avoid duplicate entries (can happen, at least on Windows)
		{
			m_PluginPaths << path;
		}
	}
}
