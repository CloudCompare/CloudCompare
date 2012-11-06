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
#ifndef X3DWRITER_H
#define X3DWRITER_H

#include <xiot/X3DTypes.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <map>

namespace XIOT {

enum X3DWriterType {X3DXML, X3DVRML, X3DFI};

class XIOT_EXPORT X3DWriter
{
public:
 
  // Description:
  // Opens the file specified with file
  // returns 1 if sucessfull otherwise 0
  virtual int openFile(const char* file) = 0;
  // Closes the file if open
  virtual void closeFile() = 0;
  // Flush can be called optionally after some operations to
  // flush the buffer to the filestream. A writer not necessarily 
  // implements this function
  virtual void flush() {};

  // Description:
  // Starts/Ends a document and sets all necessary informations, 
  // i.e. the header of the implemented encoding
  virtual void startDocument() = 0;
  virtual void endDocument() = 0;

  // Description:
  // Starts/ends a typical X3D document including the root X3D tag,
  // header information and the Scene tag. 
  virtual void startX3DDocument(X3DProfile profile = Immersive, X3DVersion version = VERSION_3_0, const std::multimap<std::string, std::string>* const meta = NULL, bool writeDefault = true);
  virtual void endX3DDocument();

  // Description:
  // Starts/ends a new X3D node specified via nodeID. The list of
  // nodeIds can be found in vtkX3DTypes.h. The EndNode
  // function closes the last open node. So there must be
  // corresponding Start/EndNode() calls for every node
  virtual void startNode(int nodeID) = 0;
  virtual void endNode() = 0;

  // Single Field
  virtual void setSFFloat(int attributeID, float) = 0;
  virtual void setSFInt32(int attributeID, int) = 0;
  virtual void setSFBool(int attributeID, bool) = 0;

  template <class C>
  inline void setSFVec3f(int attributeID, const C& c) { setSFVec3f(attributeID, c[0], c[1], c[2]); };
  virtual void setSFVec3f(int attributeID, float x, float y, float z) = 0;

  template <class C>
  inline void setSFVec2f(int attributeID, const C& c) { setSFVec2f(attributeID, c[0], c[1]); };
  virtual void setSFVec2f(int attributeID, float s, float t) = 0;

  template <class C>
  inline void setSFRotation(int attributeID, const C& c) { setSFRotation(attributeID, c[0], c[1], c[2], c[3]); };
  virtual void setSFRotation(int attributeID, float x, float y, float z, float angle) = 0;

  virtual void setSFString(int attributeID, const std::string &s) = 0;

  template <class C>
  inline void setSFColor(int attributeID, const C& c) { setSFColor(attributeID, c[0], c[1], c[2]); };
  virtual void setSFColor(int attributeID, float r, float g, float b) = 0;

  virtual void setSFImage(int attributeID, const std::vector<int>&) = 0; 

  // Multi Field
  virtual void setMFFloat(int attributeID, const std::vector<float>&) = 0;
  virtual void setMFInt32(int attributeID, const std::vector<int>&) = 0;

  virtual void setMFVec3f(int attributeID, const std::vector<float>&) = 0;
  virtual void setMFVec2f(int attributeID, const std::vector<float>&) = 0;
  virtual void setMFRotation(int attributeID, const std::vector<float>&) = 0;
  virtual void setMFString(int attributeID, const std::vector<std::string>&) = 0;
  virtual void setMFColor(int attributeID, const std::vector<float>&) = 0;

  /**
    * Set the value of any property in a X3DWriter.
    *
    * @param name The unique identifier (URI) of the property being set.
    * @param value The requested value for the property.  See documentation to learn
    *            exactly what type of property value each property expects for processing.
    *            Passing a void pointer that was initialized with a wrong type will lead
    *            to unexpected result.
	* @return	 True if the property is knwon and cpuld be set
    */
  virtual bool setProperty(const char* const name, void* value) = 0 ;
  
  /**
   * Query the current value of a property in a X3DWriter.
   *
   * The parser owns the returned pointer.  The memory allocated for
   * the returned pointer will be destroyed when the parser is deleted.
   *
   * @param name The unique identifier (URI) of the property being set.
   * @return     The current value of the property.  The pointer spans the same
   *             life-time as the parser.  A null pointer is returned if nothing
   *             was specified externally or if the property is not known.
   */
  virtual void* getProperty(const char* const name) const = 0 ;

protected:
  X3DWriterType type;  // stores which implementation is represented by the interface
	
};

} // namespace XIOT

#endif

