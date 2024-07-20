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
#include <QProcessEnvironment>
#include <QSettings>
#include <QStandardPaths>
#include <QString>
#include <QStyleFactory>
#include <QSurfaceFormat>
#include <QTranslator>
#include <QtGlobal>
#include <QTextCodec>

// CCCoreLib
#include "CCPlatform.h"

// qCC_db
#include "ccMaterial.h"
#include <ccPointCloud.h>

// qCC_glWindow
#include "ccGLWindowInterface.h"

// Common
#include "ccApplicationBase.h"
#include "ccPluginManager.h"
#include "ccTranslationManager.h"

// ccPluginAPI
#include <ccPersistentSettings.h>

// Qt
#include <QOpenGLWidget>

#if (QT_VERSION < QT_VERSION_CHECK(5, 5, 0))
#error CloudCompare does not support versions of Qt prior to 5.5
#endif

void ccApplicationBase::InitOpenGL()
{
	//See http://doc.qt.io/qt-5/qopenglwidget.html#opengl-function-calls-headers-and-qopenglfunctions
	/** Calling QSurfaceFormat::setDefaultFormat() before constructing the QApplication instance is mandatory
		on some platforms (for example, OS X) when an OpenGL core profile context is requested. This is to
		ensure that resource sharing between contexts stays functional as all internal contexts are created
		using the correct version and profile.
	**/
	{
		QSurfaceFormat format = QSurfaceFormat::defaultFormat();
		format.setStencilBufferSize(0);
#ifndef CC_LINUX // seems to cause some big issues on Linux if Quad-buffering is not supported
				// we would need to find a way to check whether it's supported or not in advance...
		format.setStereo(true); //we request stereo support by default, but this may not be supported!
#endif
		format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);

#ifdef Q_OS_MAC
		format.setVersion(2, 1);	// must be 2.1 - see ccGLWindowInterface::functions()
		format.setProfile(QSurfaceFormat::CoreProfile);
#endif

#ifdef QT_DEBUG
		format.setOption(QSurfaceFormat::DebugContext, true);
#endif

		QSurfaceFormat::setDefaultFormat(format);
	}

	// The 'AA_ShareOpenGLContexts' attribute must be defined BEFORE the creation of the Q(Gui)Application
	// DGM: this is mandatory to enable exclusive full screen for ccGLWindow (at least on Windows)
	QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
}

ccApplicationBase::ccApplicationBase(int& argc, char** argv, bool isCommandLine, const QString& version)
	: QApplication(argc, argv)
	, m_versionStr(version)
	, m_isCommandLine(isCommandLine)
{
	setOrganizationName("CCCorp");

	setupPaths();

#ifdef Q_OS_MAC
	// Mac OS X apps don't show icons in menus
	setAttribute(Qt::AA_DontShowIconsInMenus);
#endif

	// Force 'english' locale so as to get a consistent behavior everywhere
	QLocale::setDefault(QLocale::English);
	QTextCodec* utf8Codec = QTextCodec::codecForName("UTF-8");
	if (utf8Codec)
	{
		QTextCodec::setCodecForLocale(utf8Codec);
	}
	else
	{
		ccLog::Warning("Failed to set the UTF-8 codec as default (codec not found)");
	}

#ifdef Q_OS_UNIX
	// We reset the numeric locale for POSIX functions
	// See https://doc.qt.io/qt-5/qcoreapplication.html#locale-settings
	setlocale(LC_NUMERIC, "C");
#endif

	// restore the style from persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::AppStyle());
	{
		QString styleKey = settings.value("style", QString()).toString();
		if (!styleKey.isEmpty())
		{
			setAppStyle(styleKey);
		}
	}
	settings.endGroup();

	ccGLWindowInterface::SetShaderPath(m_shaderPath);
	ccPointCloud::SetShaderPath(m_shaderPath);
	ccPluginManager::Get().setPaths(m_pluginPaths);

	ccTranslationManager::Get().registerTranslatorFile(QStringLiteral("qt"), m_translationPath);
	ccTranslationManager::Get().registerTranslatorFile(QStringLiteral("CloudCompare"), m_translationPath);
	ccTranslationManager::Get().loadTranslations();

	connect(this, &ccApplicationBase::aboutToQuit, [=]() { ccMaterial::ReleaseTextures(); });
}

QString ccApplicationBase::versionLongStr(bool includeOS) const
{
	QString verStr = m_versionStr;

#if defined(CC_ENV_64)
	const QString arch("64-bit");
#elif defined(CC_ENV_32)
	const QString arch("32-bit");
#else
	const QString arch("\?\?-bit");
#endif

	if (includeOS)
	{
#if defined(CC_WINDOWS)
		const QString platform("Windows");
#elif defined(CC_MAC_OS)
		const QString platform("macOS");
#elif defined(CC_LINUX)
		const QString platform("Linux");
#else
		const QString platform("Unknown OS");
#endif
		verStr += QStringLiteral(" [%1 %2]").arg(platform, arch);
	}
	else
	{
		verStr += QStringLiteral(" [%1]").arg(arch);
	}

#ifdef QT_DEBUG
	verStr += QStringLiteral(" [DEBUG]");
#endif

	return verStr;
}

void ccApplicationBase::setupPaths()
{
	QDir appDir = QCoreApplication::applicationDirPath();

	// Set up our shader and plugin paths
#if defined(Q_OS_MAC)
	QDir bundleDir = appDir;
	if (bundleDir.dirName() == "MacOS")
	{
		bundleDir.cdUp();
	}

	m_pluginPaths << (bundleDir.absolutePath() + "/PlugIns/ccPlugins");

#if defined(CC_MAC_DEV_PATHS)
	// Used for development only - this is the path where the plugins are built
	// and the shaders are located.
	// This avoids having to install into the application bundle when developing.
	bundleDir.cdUp();
	bundleDir.cdUp();
	bundleDir.cdUp();

	m_pluginPaths << (bundleDir.absolutePath() + "/ccPlugins");
	m_shaderPath = (bundleDir.absolutePath() + "/shaders");
	m_translationPath = (bundleDir.absolutePath() + "/qCC/translations");
#else
	m_shaderPath = (bundleDir.absolutePath() + "/Shaders");
	m_translationPath = (bundleDir.absolutePath() + "/translations");
#endif
#elif defined(Q_OS_WIN)
	m_pluginPaths << (appDir.absolutePath() + "/plugins");
	m_shaderPath = (appDir.absolutePath() + "/shaders");
	m_translationPath = (appDir.absolutePath() + "/translations");
#elif defined(Q_OS_LINUX)
	// Shaders & plugins are relative to the bin directory where the executable is found
	QDir  theDir = appDir;

	if (theDir.dirName() == "bin")
	{
		theDir.cdUp();

		m_pluginPaths << (theDir.absolutePath() + "/lib/cloudcompare/plugins");
		m_shaderPath = (theDir.absolutePath() + "/share/cloudcompare/shaders");
		m_translationPath = (theDir.absolutePath() + "/share/cloudcompare/translations");
	}
	else
	{
		// Choose a reasonable default to look in
		m_pluginPaths << "/usr/lib/cloudcompare/plugins";
		m_shaderPath = "/usr/share/cloudcompare/shaders";
		m_translationPath = "/usr/share/cloudcompare/translations";
	}
#else
	#warning Need to specify the shader path for this OS.
#endif

	// If the environment variables are specified, overwrite the shader and translation paths.
	QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

	if (env.contains("CC_SHADER_PATH"))
	{
		m_shaderPath = env.value("CC_SHADER_PATH");
	}

	if (env.contains("CC_TRANSLATION_PATH"))
	{
		m_translationPath = env.value("CC_TRANSLATION_PATH");
	}

	// Add any app data paths to plugin paths
	// Plugins in these directories take precendence over the included ones
	// This allows users to put plugins outside of the install directories.
	const QStringList appDataPaths = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);

	for (const QString &appDataPath : appDataPaths)
	{
		QString path = appDataPath + "/plugins";
		if (!m_pluginPaths.contains(path)) //avoid duplicate entries (can happen, at least on Windows)
		{
			m_pluginPaths << path;
		}
	}

	// If the environment variable is specified, the path takes precedence over
	// included and appdata ones.
	if (env.contains("CC_PLUGIN_PATH"))
	{
		m_pluginPaths << env.value("CC_PLUGIN_PATH").split(QDir::listSeparator());
	}
}

bool ccApplicationBase::setAppStyle(QString styleKey)
{
	const auto loadStyleSheet = [this](const QString& resourcePath)
	{
		QFile f(resourcePath);
		if (!f.exists())
		{
			f.close();
			return false;
		}
		else
		{
			f.open(QFile::ReadOnly | QFile::Text);
			QTextStream ts(&f);
			setStyleSheet(ts.readAll());
			f.close();
			return true;
		}
	};

	if (styleKey == "QDarkStyleSheet::Dark")
	{
		if (!loadStyleSheet(":qdarkstyle/dark/darkstyle.qss"))
		{
			return false;
		}
	}
	else if (styleKey == "QDarkStyleSheet::Light")
	{
		if (!loadStyleSheet(":qdarkstyle/light/lightstyle.qss"))
		{
			return false;
		}
	}
	else
	{
		QStyle* style = QStyleFactory::create(styleKey);
		if (!style)
		{
			ccLog::Warning("Invalid style key or style couldn't be created: " + styleKey);
			return false;
		}

		setStyleSheet({});
		ccLog::Print("Applying application style: " + styleKey);
		setStyle(style);
	}

	// remember the style in persistent settings
	QSettings settings;
	settings.beginGroup(ccPS::AppStyle());
	{
		settings.setValue("style", styleKey);
	}
	settings.endGroup();

	return true;
}

