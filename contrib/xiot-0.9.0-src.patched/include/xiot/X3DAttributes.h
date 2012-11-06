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
#ifndef X3D_X3DATTRIBUTES_H
#define X3D_X3DATTRIBUTES_H

#include <xiot/XIOTConfig.h>
#include <xiot/X3DTypes.h>
#include <xiot/X3DDataTypeFactory.h>
#include <string>
#include <vector>

namespace XIOT {

/** 
 * <b>X3DAttributes</b> is an interface for retrieving information about attributes of a X3D element.
 * It provides function prototypes for obtaining the value of a specific attribute, specified by its index.
 * 
 * Additionally, <code>isDEF() / isUse()</code> can be used to check whether the node contains a DEF/USE attribute.
 * In case a DEF/USE attribute is available, <code>getDEF() / getUSE()</code> can be used to retrieve the DEF/USE - name
 * as a string.
 *
 * X3DXMLAttributes is the XML implementation of X3DAttributes.
 * @see X3DXMLAttributes
 * @ingroup x3dloader
 */
class XIOT_EXPORT X3DAttributes 
{
public:

  static const int ATTRIBUTE_NOT_FOUND = -1;

  /// Constructor.
  X3DAttributes() {};
  /// Destructor.
  virtual ~X3DAttributes() {};


  /**
   * Checks whether there's a DEF attribute.
   * @return <code>true</code>, if there is a DEF attribute. Otherwise <code>false</code>
   */
  virtual bool isDEF() const;
  /**
   * Obtains the string of the DEF attribute.
   * @return The value of the DEF attribute. If there's no DEF attribute, an empty string will be returned.
   */
  virtual std::string getDEF() const;
  /**
   * Checks whether there's a USE attribute.
   * @return <code>true</code>, if there is a USE attribute. Otherwise <code>false</code>
   */
  virtual bool isUSE() const;
  /**
   * Obtains the string of the USE attribute.
   * @return The value of the USE attribute. If there's no USE attribute, an empty string will be returned.
   */
  virtual std::string getUSE() const;
  
   /**
   * Returns the index of an attribute, specified by its ID.
   *
   * @param attributeID The id of the attribute as specified in 
   * @link{http://www.web3d.org/x3d/specifications/ISO-IEC-FCD-19776-3.2-X3DEncodings-CompressedBinary/Part03/tables.html}
   * @return Index of the specified attribute
   */
  virtual int getAttributeIndex(int attributeID) const = 0;
  /**
   * Returns the number of attributes in the node.
   *
   * @return Number of attributes in the node
   */
  virtual size_t getLength() const = 0;
   
  /**
   * Returns a string representation of all attributes in the node (without values)
   *
   * @return std::string containing all attributes in the node
   */
  virtual std::string getAttributeValue(int attributeID) const = 0;
  virtual std::string getAttributeName(int attributeID) const = 0;

  // Single fields
   /**
   * Returns the value of the specified attribute as a bool.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual bool getSFBool(int index) const = 0;
  /**
   * Returns the value of the specified attribute as a float.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual float getSFFloat(int index) const = 0;
   
   /**
   * Returns the value of the specified attribute as an integer.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual int getSFInt32(int index) const = 0;
/**
   * Returns the value of the specified attribute as a SFVec3f structure.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getSFVec3f(int index, SFVec3f &value) const = 0;
  
    /**
   * Returns the value of the specified attribute as a SFVec2f structure.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getSFVec2f(int index, SFVec2f &value) const = 0;
  /**
   * Returns the value of the specified attribute as a SFRotation structure.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getSFRotation(int index, SFRotation &value) const = 0;
   /**
   * Returns the value of the specified attribute as a std::string.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getSFString(int index, SFString &value) const = 0;
  /**
   * Returns the value of the specified attribute as a SFColor structure.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */ 
  virtual void getSFColor(int index, SFColor &value) const = 0;
  /**
   * Returns the value of the specified attribute as a SFColorRGBA structure.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */ 
  virtual void getSFColorRGBA(int index, SFColorRGBA &value) const = 0;
  
  /**
   * Returns the value of the specified attribute as a SFImage structure.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getSFImage(int index, SFImage &value) const = 0; 

  // Multi Field
   /**
   * Returns the values of the specified attribute as a std::vector<float>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFFloat(int index, MFFloat& value) const = 0;
   /**
   * Returns the values of the specified attribute as a std::vector<int>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFInt32(int index, MFInt32 &value) const = 0;

   /**
   * Returns the values of the specified attribute as a std::vector<SFVec3f>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFVec3f(int index, MFVec3f &value) const = 0;
   /**
   * Returns the values of the specified attribute as a std::vector<SFVec2f>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFVec2f(int index, MFVec2f &value) const = 0;
   /**
   * Returns the values of the specified attribute as a std::vector<SFRotation>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFRotation(int index, MFRotation &value) const = 0;
   /**
   * Returns the values of the specified attribute as a std::vector<std::string>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFString(int index, MFString &value) const = 0;
   /**
   * Returns the values of the specified attribute as a std::vector<SFColor>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFColor(int index, MFColor &value) const = 0;
   /**
   * Returns the values of the specified attribute as a std::vector<SFColorRGBA>.
   * @param index The index of the attribute. Can be obtained using getAttributeIndex(int attributeID)
   * @return Value of the specified attribute.
   */
  virtual void getMFColorRGBA(int index, MFColorRGBA &value) const = 0;


};

}

#endif

