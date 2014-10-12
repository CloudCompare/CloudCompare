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

#include "X3DFilter.h"

#ifdef CC_X3D_SUPPORT

//XIOT
#include <xiot/X3DLoader.h>
#include <xiot/X3DDefaultNodeHandler.h>
#include <xiot/X3DAttributes.h>
#include <xiot/X3DParseException.h>

#include "X3DXIOTNodeHandler.h"

//CCLib
#include <ScalarField.h>

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccMesh.h>

//Qt
#include <QFile>
#include <QTextStream>

bool X3DFilter::canLoadExtension(QString upperCaseExt) const
{
	return (	upperCaseExt == "X3D"
			||	upperCaseExt == "WRL" );
}

bool X3DFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	//not supported
	return false;
}

CC_FILE_ERROR X3DFilter::loadFile(QString filename, ccHObject& container, LoadParameters& parameters)
{
	XIOT::X3DLoader loader;
	X3DXIOTNodeHandler handler(&container);
	loader.setNodeHandler(&handler);
	try
	{
		loader.load(qPrintable(filename));
	}
	catch (XIOT::X3DParseException& e)
	{
		ccLog::Error("[X3DFilter] Error: '%s' (line: %i, col.: %i)\n",e.getMessage().c_str(),e.getLineNumber(),e.getColumnNumber());
		return CC_FERR_READING;
	}

	if (container.getChildrenNumber() == 0)
		return CC_FERR_NO_LOAD;

	return CC_FERR_NO_ERROR;
}

#endif
