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
#ifndef X3D_X3DPARSEEXCEPTION_H
#define X3D_X3DPARSEEXCEPTION_H

#include <xiot/XIOTConfig.h>
#include <string>

namespace XIOT {

/** 
 * An X3D parse error or warning.
 *
 * <p>This exception will include information for locating the error
 * in the original X3D document.
 * @ingroup x3dloader
 */
class XIOT_EXPORT X3DParseException
{
public:
  /// Constructor.
	X3DParseException(const std::string &message, int lineNumber, int columnNumber);
  /// Constructor.
	X3DParseException(const std::string &message);

  
  /**
   *  Get the reason for the parse exception 
   */
  virtual const std::string getMessage() const { return _message; }
  
  /**
  * The line number of the end of the text where the exception occurred.
  *
  * @return An integer representing the line number, or 0
  *         if none is available.
  */
  virtual int getLineNumber() const { return _lineNumber; }

  /**
  * The column number of the end of the text where the exception occurred.
  *
  * <p>The first column in a line is position 1.</p>
  *
  * @return An integer representing the column number, or 0
  *         if none is available.
  */
  virtual int getColumnNumber() const { return _columnNumber; }

protected:
	int _columnNumber;
	int _lineNumber;
	std::string _message;
};

} // namespace X3D

#endif

