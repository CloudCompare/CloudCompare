//##########################################################################
//#                                                                        #
//#                               QCC_DB                                   #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_DB_HEADER
#define CC_DB_HEADER

#include <QtCore/QtGlobal>

#if defined( QCC_DB_LIBRARY_BUILD )
#  define QCC_DB_LIB_API Q_DECL_EXPORT
#else
#  define QCC_DB_LIB_API Q_DECL_IMPORT
#endif

#endif //CC_DB_HEADER