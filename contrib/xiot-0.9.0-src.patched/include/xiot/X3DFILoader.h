/*=========================================================================
     This file is part of the XIOT library.

     Copyright (C) 2008-2009 EDF R&D
     Author: Kristian Sons (xiot@actor3d.com)

     This library is free software; you can redistribute it and/or modify
     it under the terms of the GNU Lesser Public License as published by
     the Free Software Foundation; either version 2.1 of the License, or
     (at your option) any later version.

     The XIOT library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU Lesser Public License for more details.

     You should have received a copy of the GNU Lesser Public License
     along with XIOT; if not, write to the Free Software
     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
     MA 02110-1301  USA
=========================================================================*/
#ifndef X3D_X3DFILOADER_H
#define X3D_X3DFILOADER_H

#include <xiot/X3DLoader.h>
#include <xiot/X3DNodeHandler.h>
#include <xiot/X3DSwitch.h>
#include <xiot/X3DFIAttributes.h>
#include <xiot/FITypes.h>
#include <string>
#include <fstream>

namespace XIOT {

/**  
 *  Loader for FI (binary) encoded X3D files. 
 *
 *  The class uses the openFI parser implementation to generate the events for the X3DNodeHandler.
 *  Instead of using this class directly you can use the X3DLoader which will delgate to the
 *  right encoding implementation depending on the files suffix.
 *  
 *  @see X3DLoader
 */
class XIOT_EXPORT X3DFILoader : public X3DLoader
{
public:
  /// Constructor.
  X3DFILoader();
  /// Destructor.
  virtual ~X3DFILoader();
 
  /**
   * @see X3DLoader::load()
   */ 
  bool load(const char* fileStr, bool fileValidation = true); // C.2

  
protected:
	
private:
	std::ifstream file;
	X3DNodeHandler *handler;
	X3DSwitch	x3dswitch;
};

}

#endif
