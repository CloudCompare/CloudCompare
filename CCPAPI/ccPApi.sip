// sip implementation of ccPApi

%ExportedHeaderCode
#include <ccPApi.h>
%End

class ccPApi {

%TypeHeaderCode
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
