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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2257                                                              $
//$LastChangedDate:: 2012-10-11 23:48:15 +0200 (jeu., 11 oct. 2012)        $
//**************************************************************************
//
#include "FileIOFilter.h"

//Qt
#include <QFileInfo>

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
#include "UltFilter.h"
//MESHES
#include "X3DFilter.h"
#include "ObjFilter.h"
#include "PlyFilter.h"
#include "MAFilter.h"
#ifdef CC_PDMS_SUPPORT
#include "PDMS/PDMSFilter.h"
#endif
//OTHERS
#include "DepthMapFileFilter.h"
//#include "PovFilter.h"

#include <assert.h>

CC_FILE_TYPES FileIOFilter::StringToFileFormat(const char* ext)
{
	CC_FILE_TYPES fType=UNKNOWN_FILE;

	if (strcmp(ext,"ASC")==0)
		fType=ASCII;
	else if (strcmp(ext,"TXT")==0)
		fType=ASCII;
	else if (strcmp(ext,"XYZ")==0)
		fType=ASCII;
	else if (strcmp(ext,"NEU")==0)
		fType=ASCII;
	else if (strcmp(ext,"PTS")==0)
		fType=ASCII;
	else if (strcmp(ext,"CSV")==0)
		fType=ASCII;
	else if (strcmp(ext,"BIN")==0)
		fType=BIN;
	else if (strcmp(ext,"SOI")==0)
		fType=SOI;
	else if (strcmp(ext,"PN")==0)
		fType=PN;
	else if (strcmp(ext,"PV")==0)
		fType=PV;
	else if (strcmp(ext,"PLY")==0)
		fType=PLY;
	else if (strcmp(ext,"OBJ")==0)
		fType=OBJ;
	else if (strcmp(ext,"POV")==0)
		fType=POV;
	else if (strcmp(ext,"MA")==0)
		fType=MA;
	else if (strcmp(ext,"ICM")==0)
		fType=ICM;
	else if (strcmp(ext,"OUT")==0)
		fType=BUNDLER;
	else if (strcmp(ext,"VTK")==0)
		fType=VTK;
	else if (strcmp(ext,"STL")==0)
		fType=STL;
#ifdef CC_X3D_SUPPORT
	else if (strcmp(ext,"X3D")==0)
		fType=X3D;
	else if (strcmp(ext,"WRL")==0)
		fType=X3D;
#endif
#ifdef CC_LAS_SUPPORT
	else if (strcmp(ext,"LAS")==0)
		fType=LAS;
	else if (strcmp(ext,"LAZ")==0)
		fType=LAS; //DGM: LAZ extension is handled by LASFilter
#endif
#ifdef CC_E57_SUPPORT
	else if (strcmp(ext,"E57")==0)
		fType=E57;
#endif
#ifdef CC_PDMS_SUPPORT
	else if (strcmp(ext,"PDMS")==0 || strcmp(ext,"PDMSMAC")==0)
		fType=PDMS;
#endif
#ifdef CC_ULT_SUPPORT
	else if (strcmp(ext,"ULT")==0)
		fType=ULT;
#endif
	return fType;
}

ccHObject* FileIOFilter::LoadFromFile(const QString& filename,
										CC_FILE_TYPES fType,
										bool alwaysDisplayLoadDialog/*=true*/,
										bool* coordinatesShiftEnabled/*=0*/,
										double* coordinatesShift/*=0*/)
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
		fType = StringToFileFormat(qPrintable(extension.toUpper()));

		//unknown extension?
		if (fType == UNKNOWN_FILE)
		{
			ccLog::Error(QString("[Load] Can't guess file format: unknown file extension '%1'").arg(extension));
			return 0;
		}
	}

	//get corresponding loader
	FileIOFilter* fio = NULL;
	switch (fType)
	{
	case BIN:
		fio = (FileIOFilter*)(new BinFilter());
		break;
	case OBJ:
		fio = (FileIOFilter*)(new ObjFilter());
		break;
	case PLY:
		fio = (FileIOFilter*)(new PlyFilter());
		break;
	case ICM:
		fio = (FileIOFilter*)(new IcmFilter());
		break;
	case ASCII:
		fio = (FileIOFilter*)(new AsciiFilter());
		break;
	case SOI:
		fio = (FileIOFilter*)(new SoiFilter());
		break;
	case PN:
		fio = (FileIOFilter*)(new PNFilter());
		break;
	case PV:
		fio = (FileIOFilter*)(new PVFilter());
		break;
    case DM_ASCII:
        fio = (FileIOFilter*)(new DepthMapFileFilter());
        break;
	case POV:
		fio = (FileIOFilter*)(new PovFilter());
		break;
	case BUNDLER:
		fio = (FileIOFilter*)(new BundlerFilter());
		break;
	case VTK:
		fio = (FileIOFilter*)(new VTKFilter());
		break;
	case STL:
		fio = (FileIOFilter*)(new STLFilter());
		break;
#ifdef CC_X3D_SUPPORT
    case X3D:
		fio = (FileIOFilter*)(new X3DFilter());
		break;
#endif
#ifdef CC_E57_SUPPORT
    case E57:
		fio = (FileIOFilter*)(new E57Filter());
		break;
#endif
#ifdef CC_PDMS_SUPPORT
    case PDMS:
		fio = (FileIOFilter*)(new PDMSFilter());
		break;
#endif
#ifdef CC_LAS_SUPPORT
	case LAS:
		fio = (FileIOFilter*)(new LASFilter());
		break;
#endif
#ifdef CC_ULT_SUPPORT
	case ULT:
		fio = (FileIOFilter*)(new UltFilter());
		break;
#endif
    case MA:
    case UNKNOWN_FILE:
        //nothing to do
		break;
	}

    if (!fio)
        return 0;

	//load file
    ccHObject* container = new ccHObject();
	CC_FILE_ERROR result = fio->loadFile(qPrintable(filename),
											*container,
											alwaysDisplayLoadDialog,
											coordinatesShiftEnabled,
											coordinatesShift);
	//we can release the loader instance
    delete fio;
    fio=0;

	if (result != CC_FERR_NO_ERROR)
        DisplayErrorMessage(result,"loading",fi.baseName());

    unsigned childrenCount = container->getChildrenNumber();
    if (childrenCount != 0)
    {
		//we set the main container name as the full filename (with path)
        container->setName(QString("%1 (%2)").arg(fi.fileName()).arg(fi.absolutePath()));
        for (unsigned i=0;i<childrenCount;++i)
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
        container=0;
    }

	return container;
}

CC_FILE_ERROR FileIOFilter::SaveToFile(ccHObject* entities, const char* filename, CC_FILE_TYPES fType)
{
    if (!entities || !filename || fType == UNKNOWN_FILE)
        return CC_FERR_BAD_ARGUMENT;

	FileIOFilter* fio = NULL;

	switch (fType)
	{
	case BIN:
		fio = (FileIOFilter*)(new BinFilter());
		break;
	case ASCII:
		fio = (FileIOFilter*)(new AsciiFilter());
		break;
	case OBJ:
		fio = (FileIOFilter*)(new ObjFilter());
		break;
	case PLY:
		fio = (FileIOFilter*)(new PlyFilter());
		break;
	case PN:
		fio = (FileIOFilter*)(new PNFilter());
		break;
	case PV:
		fio = (FileIOFilter*)(new PVFilter());
		break;
	case MA:
		fio = (FileIOFilter*)(new MAFilter());
		break;
    case DM_ASCII:
        fio = (FileIOFilter*)(new DepthMapFileFilter());
        break;
	case VTK:
		fio = (FileIOFilter*)(new VTKFilter());
		break;
	case STL:
		fio = (FileIOFilter*)(new STLFilter());
		break;
#ifdef CC_X3D_SUPPORT
    case X3D:
        fio = (FileIOFilter*)(new X3DFilter());
        break;
#endif
#ifdef CC_E57_SUPPORT
    case E57:
        fio = (FileIOFilter*)(new E57Filter());
        break;
#endif
#ifdef CC_PDMS_SUPPORT
    case PDMS:
		fio = (FileIOFilter*)(new PDMSFilter());
		break;
#endif
#ifdef CC_LAS_SUPPORT
    case LAS:
        fio = (FileIOFilter*)(new LASFilter());
        break;
#endif
#ifdef CC_ULT_SUPPORT
    case ULT:
        fio = (FileIOFilter*)(new UltFilter());
        break;
#endif
	case POV:
        //TODO
		//fio = (FileIOFilter*)(new PovFilter());
		break;
    case SOI:
    case ICM:
    case BUNDLER:
    case UNKNOWN_FILE:
        //nothing to do
		break;
	}

	if (!fio)
        return CC_FERR_WRONG_FILE_TYPE;

    //if the file name has no extension, we had a default one!
    char completeFileName[1024];
    strcpy(completeFileName,filename);

    int ppos = QString(filename).lastIndexOf(QChar('.'));
    if (ppos<0)
    {
        completeFileName[ppos]=0;
        sprintf(completeFileName,"%s.%s",completeFileName,CC_FILE_TYPE_DEFAULT_EXTENSION[fType]);
    }

    CC_FILE_ERROR result = fio->saveToFile(entities, completeFileName);

    delete fio;
    fio=0;

    return result;
}

void FileIOFilter::DisplayErrorMessage(CC_FILE_ERROR err, const QString& action, const QString& filename)
{
    QString errorStr;

    bool warning=false;
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

    QString outputString = QString("An error occured while %1 '%2': ").arg(action).arg(filename) + errorStr;
    if (warning)
        ccLog::Warning(outputString);
    else
        ccLog::Error(outputString);
}
