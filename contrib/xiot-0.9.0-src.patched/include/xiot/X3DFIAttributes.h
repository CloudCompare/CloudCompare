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
#ifndef X3D_X3DFIATTRIBUTES_H
#define X3D_X3DFIATTRIBUTES_H

#include <xiot/X3DAttributes.h>
#include <xiot/FITypes.h>

namespace FI {
	class ParserVocabulary;
};

namespace XIOT {

class FIAttributeImpl;

/**
 * Stores the attributes of an Fi encoded XML element
 *
 * This class is the FI implementation of X3DAttributes and 
 * provides functions to retrieve the value of an attribute,
 * specified by its index. 
 *
 * It's also able to retrieve the index of an element
 * by its ID, as specified in:
 * @link(http://www.web3d.org/x3d/specifications/ISO-IEC-FCD-19776-3.2-X3DEncodings-CompressedBinary/Part03/tables.html)
 *
 * Resolving the binary ocetet attribute values to the X3D types is
 * done directly if possible, i.e. by using the encoding algorithms.
 * Otherwise it is delegated to the ParserVocabulary.
 *
 * @see X3DAttributes
 * @see X3DParserVocabulary
 * @ingroup x3dloader
 */
class XIOT_EXPORT X3DFIAttributes : public X3DAttributes
{
public:
  /// Constructor.
  X3DFIAttributes(const void *const attributes, const FI::ParserVocabulary* vocab);

  /// Destructor.
  virtual ~X3DFIAttributes();

  void addAttribute(FI::Attribute &attr);

  virtual int getAttributeIndex(int attributeID) const;
  virtual size_t getLength() const;
  virtual std::string getAttributeValue(int attributeID) const;
  virtual std::string getAttributeName(int attributeID) const;

// Single fields
  virtual bool  getSFBool(int index) const;
  virtual float getSFFloat(int index) const;
  virtual int   getSFInt32(int index) const;

  virtual void getSFVec3f(int index, SFVec3f &value) const;
  virtual void getSFVec2f(int index, SFVec2f &value) const;
  virtual void getSFRotation(int index, SFRotation &value) const;
  virtual void getSFString(int index, SFString &value) const;
  virtual void getSFColor(int index, SFColor &value) const;
  virtual void getSFColorRGBA(int index,SFColorRGBA &value) const;
  virtual void getSFImage(int index, SFImage &value) const; 

  // Multi Field
  virtual void getMFFloat(int index, MFFloat &value) const;
  virtual void getMFInt32(int index, MFInt32 &value) const;
  virtual void getMFVec3f(int index, MFVec3f &value) const;
  virtual void getMFVec2f(int index, MFVec2f &value) const;
  virtual void getMFRotation(int index, MFRotation &value) const;
  virtual void getMFString(int index, MFString &value) const;
  virtual void getMFColor(int index, MFColor &value) const;
  virtual void getMFColorRGBA(int index, MFColorRGBA &value) const;

protected:
  void getFloatArray(const FI::NonIdentifyingStringOrIndex &value, std::vector<float> &vec) const;
  void   getIntArray(const FI::NonIdentifyingStringOrIndex &value, std::vector<int> &vec) const;

  FIAttributeImpl* _impl;
};

}; // End namespace X3D;


#endif

