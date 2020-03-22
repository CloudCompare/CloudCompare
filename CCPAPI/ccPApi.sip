// sip implementation of ccPApi

%ExportedHeaderCode
#include <ccPApi.h>
%End

class ccPApi: public QMainWindow {

%TypeHeaderCode
#include <ccPApi.h>
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
