//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef X3D_NODE_HANDLER_HEADER
#define X3D_NODE_HANDLER_HEADER

#include "FileIOFilter.h"

#ifdef CC_X3D_SUPPORT

//XIOT
//#include <xiot/X3DLoader.h>
#include <xiot/X3DDefaultNodeHandler.h>
//#include <xiot/X3DAttributes.h>
//#include <xiot/X3DParseException.h>

class ccHObject;

using namespace XIOT;

//! X3D/XOIT node handler
/** To be used with by X3DFilter
**/
class X3DXIOTNodeHandler : public X3DDefaultNodeHandler
{
public:

	//! Default constructor
	explicit X3DXIOTNodeHandler(ccHObject* root);

	//Destructor
	virtual ~X3DXIOTNodeHandler();

	//! Returns current root;
	ccHObject* root();

	//inherited from X3DDefaultNodeHandler
	virtual int startShape(const X3DAttributes& attr);
	virtual int endShape();
	
	virtual int startUnhandled(const char* nodeName, const X3DAttributes& attr);
	virtual int endUnhandled(const char* nodeName);

	virtual int startIndexedFaceSet(const X3DAttributes &attr);
	virtual int endIndexedFaceSet();

	virtual int startCoordinate(const X3DAttributes &attr);
	virtual int endCoordinate();

	virtual int startTransform(const X3DAttributes &attr);
	virtual int startNormal(const X3DAttributes &attr);
	virtual int startColor(const X3DAttributes &attr);

	//virtual int startSphere(const X3DAttributes &attr);
	//virtual int startBox(const X3DAttributes &attr);
	//virtual int startMaterial(const X3DAttributes &attr);
	//virtual int startIndexedLineSet(const X3DAttributes &attr);
	//virtual int endIndexedLineSet();

protected:

	//! Loaded scene graph root
	ccHObject* m_root;

	//! Current scene graph entity
	ccHObject* m_currentLeaf;
};

#endif //CC_X3D_SUPPORT

#endif
