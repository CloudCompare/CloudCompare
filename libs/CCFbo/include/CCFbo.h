#pragma once
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

#include <QtCore/QtGlobal>

/**
 * \brief Defines the export/import macro for the CCFbo library.
 * 
 * \details This macro handles the platform-specific symbol visibility 
 * for the CCFbo library, ensuring proper dynamic library linking 
 * across different compilation modes.
 * 
 * Key functionalities:
 * - Exports symbols when building the library
 * - Imports symbols when using the library
 * - Compatible with Qt's cross-platform library mechanism
 * 
 * \note Uses Qt's Q_DECL_EXPORT and Q_DECL_IMPORT macros for 
 * platform-independent symbol visibility
 * 
 * \example
 * \code
 * // In library header file (CCFbo.h)
 * #if defined( CCFBO_LIBRARY_BUILD )
 * #  define CCFBO_LIB_API Q_DECL_EXPORT  // Exporting symbols
 * #else
 * #  define CCFBO_LIB_API Q_DECL_IMPORT  // Importing symbols
 * #endif
 * 
 * // Usage in library classes
 * class CCFBO_LIB_API MyLibraryClass {
 *     // Class definition with exported/imported symbols
 * };
 * \endcode
 * 
 * Compilation scenarios:
 * - When building the library: CCFBO_LIBRARY_BUILD is defined
 * - When using the library: CCFBO_LIBRARY_BUILD is not defined
 */
#if defined( CCFBO_LIBRARY_BUILD )
#  define CCFBO_LIB_API Q_DECL_EXPORT
#else
#  define CCFBO_LIB_API Q_DECL_IMPORT
#endif
