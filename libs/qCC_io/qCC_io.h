//##########################################################################
//#                                                                        #
//#                               QCC_IO                                   #
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

#ifndef CC_IO_HEADER
#define CC_IO_HEADER

#include <QtCore/QtGlobal>

#if defined( QCC_IO_LIBRARY_BUILD )
#  define QCC_IO_LIB_API Q_DECL_EXPORT
#else
#  define QCC_IO_LIB_API Q_DECL_IMPORT
#endif

#endif //CC_IO_HEADER