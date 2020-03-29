// sip implementation of ccPApi

%ExportedHeaderCode
#include <ccPApi.h>
%End

//! Typical I/O filter errors
enum CC_FILE_ERROR {CC_FERR_NO_ERROR,
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

class ccPApi {

    %TypeHeaderCode
    #define _CCDEBUG_
    #include <ccTrace.h>
    #include <ccPApi.h>
    %End

%Docstring
ccPApi is the actual context of the Python interface for CloudCompare:
cloud compare Python Api.
-_____-_______-______---.
You should create an instance of ccPApi to get access to the cloudCompare features.
For instance:
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
capi = cc.ccPApi()
cloud = capi.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6446_MNT_LAMB93_IGN69.xyz")
%End

public:

    enum CC_SHIFT_MODE
    {
        AUTO = 0, XYZ = 1
    };

    ccPApi();

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

private:
    ccPApi(const ccPApi &);
};


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
