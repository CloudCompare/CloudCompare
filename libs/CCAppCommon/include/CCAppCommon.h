//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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

#pragma once

#include <QtCore/QtGlobal>

/**
 * \file CCAppCommon.h
 * \brief Defines library export macros for the CCAppCommon library
 *
 * This header provides platform-specific macros for dynamic library 
 * export and import, ensuring proper symbol visibility across different 
 * platforms and build configurations.
 *
 * \note Used by other CloudCompare libraries and components to manage 
 * library symbol visibility
 */

/**
 * \def CCAPPCOMMON_LIB_API
 * \brief Cross-platform library export/import macro
 *
 * Determines the appropriate symbol export/import directive based on 
 * the current build configuration (library build or library usage).
 *
 * - When building the library: Exports symbols
 * - When using the library: Imports symbols
 *
 * \note Uses Qt's Q_DECL_EXPORT and Q_DECL_IMPORT macros for compatibility
 * \warning Ensures correct symbol visibility in shared library scenarios
 */
#if defined( CCAPPCOMMON_LIBRARY_BUILD )
#  define CCAPPCOMMON_LIB_API Q_DECL_EXPORT
#else
#  define CCAPPCOMMON_LIB_API Q_DECL_IMPORT
#endif
