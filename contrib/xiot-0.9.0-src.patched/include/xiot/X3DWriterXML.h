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
#ifndef X3DWriterXML_H
#define X3DWriterXML_H

#include <xiot/X3DWriter.h>

namespace XIOT {

struct XMLInfo {

XMLInfo(int _elementId)
{
  this->elementId = _elementId;
  this->endTagWritten = false;
}
  int elementId;
  bool endTagWritten;
};

class XIOT_EXPORT X3DWriterXML : public X3DWriter
{

public:
    
  virtual void closeFile();
  virtual int openFile(const char* file);
  virtual void flush();


  void startDocument();
  void endDocument();

  // Elements
  void startNode(int elementID);
  void endNode();
  
  
  // Single Field
  virtual void setSFFloat(int attributeID, float);
  virtual void setSFInt32(int attributeID, int);
  virtual void setSFBool(int attributeID, bool);

  virtual void setSFVec3f(int attributeID, float x, float y, float z);
  virtual void setSFVec2f(int attributeID, float s, float t);
  virtual void setSFRotation(int attributeID, float x, float y, float z, float angle);
  virtual void setSFString(int attributeID, const std::string &s);
  virtual void setSFColor(int attributeID, float r, float g, float b);
  virtual void setSFImage(int attributeID, const std::vector<int>&); 

  // Multi Field
  virtual void setMFFloat(int attributeID, const std::vector<float>&);
  virtual void setMFInt32(int attributeID, const std::vector<int>&);

  virtual void setMFVec3f(int attributeID, const std::vector<float>&);
  virtual void setMFVec2f(int attributeID, const std::vector<float>&);
  virtual void setMFRotation(int attributeID, const std::vector<float>&);
  virtual void setMFString(int attributeID, const std::vector<std::string>&);
  virtual void setMFColor(int attributeID, const std::vector<float>&);
  
  virtual bool setProperty(const char* const name, void* value);
  virtual void* getProperty(const char* const name) const;

public:
  X3DWriterXML();
  ~X3DWriterXML();

private:
 
  const char* getNewline() { return "\n"; };
  void addDepth();
  void subDepth();
  void printAttributeString(int attributeID);

  std::string ActTab;
  int Depth;
  FILE *OutputStream;
  std::vector<XMLInfo> *InfoStack;

};

}

#endif
