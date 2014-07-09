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

#include "FileIOFilter.h"

//file wrappers
//CLOUDS
#include "BinFilter.h"
#include "IcmFilter.h"
#include "AsciiFilter.h"
#include "SoiFilter.h"
#include "PNFilter.h"
#include "PVFilter.h"
#include "PovFilter.h"
#include "BundlerFilter.h"
#include "VTKFilter.h"
#include "STLFilter.h"
#include "LASFilter.h"
#include "E57Filter.h"
#include "PCDFilter.h"
#include "PTXFilter.h"
//MESHES
#include "X3DFilter.h"
#include "ObjFilter.h"
#include "PlyFilter.h"
#include "MAFilter.h"
#include "FBXFilter.h"
#include "OFFFilter.h"
//CAD
#include "PDMS/PDMSFilter.h"
//OTHERS
#include "DepthMapFileFilter.h"
#include "RasterGridFilter.h"
//#include "PovFilter.h"
#include "DxfFilter.h"

//Qt
#include <QFileInfo>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

//system
#include <assert.h>

CC_FILE_TYPES FileIOFilter::GuessFileFormatFromExtension(QString ext)
{
	ext = ext.toUpper();

	if (ext == "ASC")
		return ASCII;
	else if (ext == "TXT")
		return ASCII;
	else if (ext == "XYZ")
		return ASCII;
	else if (ext == "NEU")
		return ASCII;
	else if (ext == "PTS")
		return ASCII;
	else if (ext == "CSV")
		return ASCII;
	else if (ext == "BIN")
		return BIN;
	else if (ext == "SOI")
		return SOI;
	else if (ext == "PN")
		return PN;
	else if (ext == "PV")
		return PV;
	else if (ext == "PLY")
		return PLY;
	else if (ext == "OBJ")
		return OBJ;
	else if (ext == "POV")
		return POV;
	else if (ext == "MA")
		return MA;
	else if (ext == "ICM")
		return ICM;
	else if (ext == "OUT")
		return BUNDLER;
	else if (ext == "VTK")
		return VTK;
	else if (ext == "STL")
		return STL;
	else if (ext == "PCD")
		return PCD;
	else if (ext == "OFF")
		return OFF;
	else if (ext == "PTX")
		return PTX;
#ifdef CC_X3D_SUPPORT
	else if (ext == "X3D")
		return X3D;
	else if (ext == "WRL")
		return X3D;
#endif
#ifdef CC_LAS_SUPPORT
	else if (ext == "LAS")
		return LAS;
	else if (ext == "LAZ")
		return LAS; //DGM: LAZ extension is handled by LASFilter
#endif
#ifdef CC_E57_SUPPORT
	else if (ext == "E57")
		return E57;
#endif
#ifdef CC_PDMS_SUPPORT
	else if (ext == "MAC" || ext == "PDMS" || ext == "PDMSMAC")
		return PDMS;
#endif
#ifdef CC_DXF_SUPPORT
	else if (ext == "DXF")
		return DXF;
#endif
#ifdef CC_GDAL_SUPPORT
	else if (ext == "TIF" || ext == "TIFF" || ext == "ADF")
		return RASTER;
#endif
#ifdef CC_FBX_SUPPORT
	else if (ext == "FBX")
		return FBX;
#endif
	return UNKNOWN_FILE;
}

FileIOFilter* FileIOFilter::CreateFilter(CC_FILE_TYPES fType)
{
	//return corresponding loader
	switch (fType)
	{
		//we keep the same order as the CC_FILE_TYPES enumerator!
	case UNKNOWN_FILE:
		assert(false);
		return 0;
	case SOI:
		return new SoiFilter();
	case ASCII:
		return new AsciiFilter();
	case BIN:
		return new BinFilter();
	case PN:
		return new PNFilter();
	case PV:
		return new PVFilter();
	case PLY:
		return new PlyFilter();
	case OBJ:
		return new ObjFilter();
	case POV:
		return new PovFilter();
	case MA:
		return new MAFilter();
	case ICM:
		return new IcmFilter();
	case DM_ASCII:
		return new DepthMapFileFilter();
	case BUNDLER:
		return new BundlerFilter();
	case VTK:
		return new VTKFilter();
	case STL:
		return new STLFilter();
	case PCD:
		return new PCDFilter();
	case OFF:
		return new OFFFilter();
	case PTX:
		return new PTXFilter();
#ifdef CC_X3D_SUPPORT
	case X3D:
		return new X3DFilter();
#endif
#ifdef CC_LAS_SUPPORT
	case LAS:
		return new LASFilter();
#endif
#ifdef CC_E57_SUPPORT
	case E57:
		return new E57Filter();
#endif
#ifdef CC_PDMS_SUPPORT
	case PDMS:
		return new PDMSFilter();
#endif
#ifdef CC_DXF_SUPPORT
	case DXF:
		return new DxfFilter();
#endif
#ifdef CC_GDAL_SUPPORT
	case RASTER:
		return new RasterGridFilter();
#endif
#ifdef CC_FBX_SUPPORT
	case FBX:
		return new FBXFilter();
#endif
	case FILE_TYPES_COUNT:
	default:
		assert(false);
		break;
	}

	return 0;
}

ccHObject* FileIOFilter::LoadFromFile(	const QString& filename,
										CC_FILE_TYPES fType,
										bool alwaysDisplayLoadDialog/*=true*/,
										bool* coordinatesShiftEnabled/*=0*/,
										CCVector3d* coordinatesShift/*=0*/)
{
	//check file existence
	QFileInfo fi(filename);
	if (!fi.exists())
	{
		ccLog::Error(QString("[Load] File '%1' doesn't exist!").arg(filename));
		return 0;
	}

	//do we need to guess file format?
	if (fType == UNKNOWN_FILE)
	{
		//look for file extension (we trust Qt on this task)
		QString extension = QFileInfo(filename).suffix();
		if (extension.isEmpty())
		{
			ccLog::Error("[Load] Can't guess file format: no file extension");
			return 0;
		}

		//convert extension to file format
		fType = GuessFileFormatFromExtension(extension);

		//unknown extension?
		if (fType == UNKNOWN_FILE)
		{
			ccLog::Error(QString("[Load] Can't guess file format: unknown file extension '%1'").arg(extension));
			return 0;
		}
	}

	//get corresponding loader
	FileIOFilter* fio = CreateFilter(fType);
	if (!fio)
		return 0;

	//load file
	ccHObject* container = new ccHObject();
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	try
	{
		result = fio->loadFile(	filename,
								*container,
								alwaysDisplayLoadDialog,
								coordinatesShiftEnabled,
								coordinatesShift);
	}
	catch(...)
	{
		ccLog::Warning("[I/O] Exception caught during file opening!");
		if (container)
			container->removeAllChildren();
		result = CC_FERR_CONSOLE_ERROR;
	}

	//we can release the loader instance
	delete fio;
	fio = 0;

	if (result != CC_FERR_NO_ERROR)
		DisplayErrorMessage(result,"loading",fi.baseName());
	else
		ccLog::Print(QString("[I/O] File '%1' loaded successfully").arg(fi.baseName()));

	unsigned childrenCount = container->getChildrenNumber();
	if (childrenCount != 0)
	{
		//we set the main container name as the full filename (with path)
		container->setName(QString("%1 (%2)").arg(fi.fileName()).arg(fi.absolutePath()));
		for (unsigned i=0; i<childrenCount; ++i)
		{
			ccHObject* child = container->getChild(i);
			QString newName = child->getName();
			if (newName.startsWith("unnamed"))
			{
				//we automatically replace occurences of 'unnamed' in entities names by the base filename (no path, no extension)
				newName.replace(QString("unnamed"),fi.baseName());
				child->setName(newName);
			}
		}
	}
	else
	{
		delete container;
		container = 0;
	}

	return container;
}

CC_FILE_ERROR FileIOFilter::SaveToFile(ccHObject* entities, const QString& filename, CC_FILE_TYPES fType)
{
	if (!entities || filename.isEmpty() || fType == UNKNOWN_FILE)
		return CC_FERR_BAD_ARGUMENT;

	FileIOFilter* fio = CreateFilter(fType);
	if (!fio)
		return CC_FERR_WRONG_FILE_TYPE;

	//if the file name has no extension, we had a default one!
	QString completeFileName(filename);
	if (QFileInfo(filename).suffix().isEmpty())
		completeFileName += QString(".%1").arg(CC_FILE_TYPE_DEFAULT_EXTENSION[fType]);

	CC_FILE_ERROR result = fio->saveToFile(entities, completeFileName);

	delete fio;
	fio = 0;

	return result;
}

void FileIOFilter::DisplayErrorMessage(CC_FILE_ERROR err, const QString& action, const QString& filename)
{
	QString errorStr;

	bool warning = false;
	switch(err)
	{
	case CC_FERR_NO_ERROR:
		return; //no message will be displayed!
	case CC_FERR_BAD_ARGUMENT:
		errorStr = "[internal] bad argument";
		break;
	case CC_FERR_UNKNOWN_FILE:
		errorStr = "Unknown file";
		break;
	case CC_FERR_WRONG_FILE_TYPE:
		errorStr = "Wrong file type (check header)";
		break;
	case CC_FERR_WRITING:
		errorStr = "Writing error (disk full/no access right?)";
		break;
	case CC_FERR_READING:
		errorStr = "Reading error (no access right?)";
		break;
	case CC_FERR_NO_SAVE:
		errorStr = "Nothing to save";
		break;
	case CC_FERR_NO_LOAD:
		errorStr = "Nothing to load";
		break;
	case CC_FERR_BAD_ENTITY_TYPE:
		errorStr = "Incompatible entity/file types";
		break;
	case CC_FERR_CANCELED_BY_USER:
		errorStr = "Process canceled by user";
		warning=true;
		break;
	case CC_FERR_NOT_ENOUGH_MEMORY:
		errorStr = "Not enough memory";
		break;
	case CC_FERR_MALFORMED_FILE:
		errorStr = "Malformed file";
		break;
	case CC_FERR_BROKEN_DEPENDENCY_ERROR:
		errorStr = "dependent entities missing (see Console)";
		break;
	case CC_FERR_CONSOLE_ERROR:
		//error already sent!
	default:
		return; //no message will be displayed!
	}

	QString outputString = QString("An error occurred while %1 '%2': ").arg(action).arg(filename) + errorStr;
	if (warning)
		ccLog::Warning(outputString);
	else
		ccLog::Error(outputString);
}
