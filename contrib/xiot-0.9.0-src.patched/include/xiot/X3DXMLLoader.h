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
#ifndef X3D_X3DXMLLOADER_H
#define X3D_X3DXMLLOADER_H

#include <xiot/X3DLoader.h>
#include <string>

namespace XIOT {

/**
 * @class XMLParserImpl
 * Wrapper to hide the XercesC Implementation from the interface.
 */
class XMLParserImpl;

/**  
 *  Loader for XML encoded X3D files. 
 *
 *  Wrapper to hide the XML parser implementation from the interface.
 *  Instead of using this class directly you can use the X3DLoader which will delgate to the
 *  right encoding implementation depending on the files suffix.
 *  
 *  @see X3DLoader
 *  @ingroup x3dloader
 */
class XIOT_EXPORT X3DXMLLoader : public X3DLoader
{
public:
  /// Constructor.
  X3DXMLLoader();
  /// Destructor.
  virtual ~X3DXMLLoader();
  
  /** Loads an X3D scene graph from the file. If fileValidation is true, then the file will be 
  * verified. 
  * @return True, if loading was successful.
  * @exception X3DParseException If a parsing error occures that cannot be handled.
  */
  bool load(const char* fileStr, bool fileValidation = true) const;
  
protected:
	XMLParserImpl *_impl;
};

}

#endif
