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
#ifndef X3D_X3DPARSERVOCABULARY_H
#define X3D_X3DPARSERVOCABULARY_H

#include <xiot/XIOTConfig.h>
#include <xiot/FIParserVocabulary.h>
#include <xiot/X3DFIEncodingAlgorithms.h>
#include <string>
#include <fstream>
#include <map>

namespace XIOT {


/**
 * <b>X3DParserVocabulary</b> implements the External Parser Vocabulary as defined
 * in the X3DB spec.
 *
 * This includes the tables for element and attribute names, for 
 * attribute values and two X3D specific encoding algorithms
 *
 * @link(http://www.web3d.org/x3d/specifications/ISO-IEC-FCD-19776-3.2-X3DEncodings-CompressedBinary/Part03/tables.html)
 *
 * @ingroup x3dloader
 */
class XIOT_EXPORT X3DParserVocabulary : public FI::DefaultParserVocabulary
{
public:
	static const int ATTRIBUT_VALUE_FALSE_INDEX = 1;
	static const int ATTRIBUT_VALUE_TRUE_INDEX = 2;

	X3DParserVocabulary();
	virtual ~X3DParserVocabulary() {};

	QuantizedzlibFloatArrayAlgorithm _quantizedzlibFloatArrayAlgorithm;
	DeltazlibIntArrayAlgorithm _deltazlibIntArrayAlgorithm;
private:
};
}

#endif

