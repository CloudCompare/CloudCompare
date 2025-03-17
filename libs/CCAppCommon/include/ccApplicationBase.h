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

/**
 * \def ccApp
 * \brief Global macro to access the application instance
 *
 * Provides a convenient way to access the ccApplicationBase singleton
 * similar to Qt's qApp macro.
 *
 * \note Requires a valid QCoreApplication instance to be created first
 * \warning Must be used after application initialization
 */
#define ccApp (static_cast<ccApplicationBase *>( QCoreApplication::instance() ))

/**
 * \class ccApplicationBase
 * \brief Base application class for CloudCompare
 *
 * Extends QApplication to provide CloudCompare-specific 
 * initialization, configuration, and utility methods.
 *
 * \note Manages application paths, version, and style settings
 * \warning Must call InitOpenGL() before creating an instance
 */
class CCAPPCOMMON_LIB_API ccApplicationBase : public QApplication
{
public:
    /**
     * \brief Initialize OpenGL for the application
     *
     * Must be called before creating the application instance
     * to ensure proper OpenGL setup and configuration.
     *
     * \note Static method that sets up OpenGL context
     * \warning Mandatory pre-initialization step
     */
    static void InitOpenGL();
    
    /**
     * \brief Constructs the CloudCompare application
     *
     * \param argc Reference to argument count
     * \param argv Pointer to argument vector
     * \param isCommandLine Flag indicating command-line mode
     * \param version Application version string
     *
     * \note Sets up application paths and configuration
     */
    ccApplicationBase( int &argc, char **argv, bool isCommandLine, const QString &version );
    
    /**
     * \brief Checks if application is running in command-line mode
     *
     * \return bool True if running in command-line mode, false otherwise
     */
    inline bool isCommandLine() const { return m_isCommandLine; }
    
    /**
     * \brief Gets the path to translation files
     *
     * \return const QString& Path to translation directory
     */
    inline const QString& translationPath() const { return m_translationPath; }
    
    /**
     * \brief Gets the application version string
     *
     * \return const QString& Short version string
     */
    inline const QString& versionStr() const { return m_versionStr; }
    
    /**
     * \brief Generates a detailed version string
     *
     * \param includeOS If true, includes operating system information
     * \return QString Comprehensive version string
     */
    QString versionLongStr(bool includeOS) const;
    
    /**
     * \brief Sets the application style
     *
     * Changes the application's visual style using QStyleFactory
     *
     * \param styleKey Name of the style to apply (from QStyleFactory)
     * \return bool True if style was successfully set, false otherwise
     *
     * \note Supports styles like 'Windows', 'Fusion', 'macOS', etc.
     * \warning May not work identically across all platforms
     */
    bool setAppStyle(QString styleKey);

private:
    /**
     * \brief Sets up application-specific paths
     *
     * Initializes paths for shaders, translations, and plugins
     *
     * \note Called during application construction
     */
    void setupPaths();
        
    const QString m_versionStr;          ///< Short version string
    
    QString	m_shaderPath;               ///< Path to shader resources
    QString	m_translationPath;          ///< Path to translation files
    QStringList m_pluginPaths;            ///< List of plugin search paths
    
    const bool m_isCommandLine;           ///< Flag for command-line mode
};
