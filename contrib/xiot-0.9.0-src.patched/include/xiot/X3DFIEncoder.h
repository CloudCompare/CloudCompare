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
#ifndef X3D_X3DFIENCODER_H
#define X3D_X3DFIENCODER_H

#include <xiot/XIOTConfig.h>
#include <xiot/FIEncoder.h>

namespace XIOT {

/**  
 *  Decoder for FI files. 
 */
class XIOT_EXPORT X3DFIEncoder : public FI::FIEncoder
{
public:
  X3DFIEncoder(void);
  virtual ~X3DFIEncoder(void);

  virtual void encodeAttributeIntegerArray(const int* values, size_t size);
  virtual void encodeAttributeFloatArray(const float* values, size_t size);

  void setFloatAlgorithm(int algorithmID);
  void setIntAlgorithm(int algorithmID);
  
  int getFloatAlgorithm() const;
  int getIntAlgorithm() const;


protected:
  int _floatAlgorithm;
  int _intAlgorithm;
};

} // namespace XIOT

#endif // X3D_X3DFIENCODER_H

