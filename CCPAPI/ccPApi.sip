//##########################################################################
//#                                                                        #
//#                               CCPAPI                                   #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          Copyright 2020 Paul RASCLE www.openfields.fr                  #
//#                                                                        #
//##########################################################################

// sip implementation of ccPApi

%ExportedHeaderCode
#include <ccPApi.h>
%End

%ModuleHeaderCode
#define _CCDEBUG_
#include <ccTrace.h>
#include <ccPApi.h>
%End

enum CC_SHIFT_MODE
{
    AUTO = 0, XYZ = 1
};

//! Typical I/O filter errors
enum CC_FILE_ERROR 
{
    CC_FERR_NO_ERROR,
    CC_FERR_BAD_ARGUMENT,
    CC_FERR_UNKNOWN_FILE,
    CC_FERR_WRONG_FILE_TYPE,
    CC_FERR_WRITING,
    CC_FERR_READING,
    CC_FERR_NO_SAVE,
    CC_FERR_NO_LOAD,
    CC_FERR_BAD_ENTITY_TYPE,
    CC_FERR_CANCELED_BY_USER,
    CC_FERR_NOT_ENOUGH_MEMORY,
    CC_FERR_MALFORMED_FILE,
    CC_FERR_CONSOLE_ERROR,
    CC_FERR_BROKEN_DEPENDENCY_ERROR,
    CC_FERR_FILE_WAS_WRITTEN_BY_UNKNOWN_PLUGIN,
    CC_FERR_THIRD_PARTY_LIB_FAILURE,
    CC_FERR_THIRD_PARTY_LIB_EXCEPTION,
    CC_FERR_NOT_IMPLEMENTED,
};

enum CurvatureType
{
    GAUSSIAN_CURV = 1,
    MEAN_CURV,
    NORMAL_CHANGE_RATE
};

ccPointCloud* loadPointCloud(const char* filename,
                             CC_SHIFT_MODE mode=AUTO,
                             int skip=0, 
                             double x=0,
                             double y=0, 
                             double z=0);
%Docstring
Load a 3D cloud from a file
params:
- filename
- optional parameters not used yet!
return a ccPointCloud object.
Usage: see ccPointCloud doc.
%End

CC_FILE_ERROR SavePointCloud(ccPointCloud* cloud,
                             const QString& filename);
%Docstring
Save a 3D cloud in a file
params:
- cloud
- filename
return 0 or I/O error.
%End

bool computeCurvature(CurvatureType option, double radius, QList<ccPointCloud*>);
%Docstring
Compute the curvature on a list of point clouds (create a scalarField)
params:
- CurvatureType from cloudCompare.GAUSSIAN_CURV, cloudCompare.MEAN_CURV, cloudCompare.NORMAL_CHANGE_RATE
- radius try 1.9
- list of clouds
%End

%Exception std::exception(SIP_Exception) /PyName=StdException, default/
{
%TypeHeaderCode
#include <exception>
%End
%RaiseCode
    const char *detail = sipExceptionRef.what();

    SIP_BLOCK_THREADS
    PyErr_SetString(sipException_std_exception, detail);
    SIP_UNBLOCK_THREADS
%End
};

