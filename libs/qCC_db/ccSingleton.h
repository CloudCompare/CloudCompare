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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_SINGLETON_HEADER
#define CC_SINGLETON_HEADER

//! Generic singleton encapsulation structure
template<class T> struct ccSingleton
{
	//! Default constructor
	ccSingleton() : instance(nullptr) {}
	//! Destructor
	~ccSingleton() { release(); }
	//! Releases the current instance
	inline void release() { if (instance) { delete instance; instance = nullptr; } }
	
	//! Current instance
	T* instance;
};

#endif //CC_SINGLETON_HEADER
