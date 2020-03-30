// sip implementation of ScalarField

%ExportedHeaderCode
#include <ScalarField.h>
%End

namespace CCLib
{
    class ScalarField
    {
    %TypeHeaderCode
    #define _CCDEBUG_
    #include <ccTrace.h>
    #define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
    #include <ScalarField.h>
    #include <numpy/ndarraytypes.h>
    #include <numpy/npy_common.h>
    #include <numpy/arrayobject.h>
    %End
    
    %Docstring
    A simple scalar field (to be associated to a point cloud)
    A monodimensional array of scalar values.
    Invalid values can be represented by NAN_VALUE.
    Dealing with numpy (method toNpArray) requires to initialize first numpy interface:
    cloudCompare.CCLib.ScalarField.initNumpyApi()
    %End
    
    public:
        explicit ScalarField(const char* name = nullptr);
        
        const char* getName() const;
    
        static void initNumpyApi();
        %MethodCode
        CCTRACE("initialization numpy C++ API");
        import_array();
        CCTRACE("initialization numpy C++ API done");
        %End

        PyObject* toNpArray();
        %MethodCode
        if( !sipCpp->empty() )
        {
            int type_num = NPY_FLOAT32;
            npy_intp nRows = sipCpp->size(); //size_t
            npy_intp dims[1] = {nRows};
            CCTRACE("nrows: " << nRows);
            float *p=sipCpp->data();
            CCTRACE("data address: " << p);
            CCTRACE("p[0] : " << p[0]);
            CCTRACE("p[1] : " << p[1]);
            PyObject* npArray = PyArray_SimpleNewFromData(1, dims, type_num, p ); //  /TransferThis/ memory management ownership C++
            CCTRACE("---");
            sipRes = npArray;
        }
        else
        {
            npy_intp dims[1] = {0};
            sipRes = PyArray_ZEROS(1, dims, NPY_FLOAT32, 0);
        }
        %End
        
    protected:
        ~ScalarField();
    };
    
};

















