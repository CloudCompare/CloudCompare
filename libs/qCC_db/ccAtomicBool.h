//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef ATOMIC_BOOL_HEADER
#define ATOMIC_BOOL_HEADER

//qCC_db
#include "ccObject.h" //for CC_QT5

//Qt
#include <QAtomicInt>

//! Qt 4/5 compatible atomic boolean
class ccAtomicBool
{
public:
	ccAtomicBool() : value(0) {}
	ccAtomicBool(bool state) : value(state ? 1 : 0) {}

	//! Conversion to bool
#ifndef CC_QT5
	inline operator bool() const { return value != 0; }
#else
	inline operator bool() const { return value.load() != 0; }
#endif

	QAtomicInt value;
};

#endif //ATOMIC_BOOL_HEADER
