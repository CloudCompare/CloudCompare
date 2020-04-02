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
            sipRes = PyArray_SimpleNewFromData(1, dims, type_num, p ); //  /TransferThis/ memory management ownership C++
            CCTRACE("---");
            //sipRes = npArray;
        }
        else
        {
            npy_intp dims[1] = {0};
            sipRes = PyArray_ZEROS(1, dims, NPY_FLOAT32, 0);
        }
        %End
        
        bool fromNpArray(SIP_PYOBJECT p);
        %MethodCode
        if (PyObject_TypeCheck(a0, &PyArray_Type))
        {
            CCTRACE("fromNpArray: ndarray");
            PyArrayObject* pya = (PyArrayObject*)a0;
            int nd = PyArray_NDIM(pya);
            CCTRACE("nDim: " << nd);
            npy_intp* dims = PyArray_DIMS(pya);
            npy_intp nRows = dims[0];
            CCTRACE("nRows: " << nRows);
            int atype = PyArray_TYPE(pya);
            CCTRACE("type: " << atype << " " << NPY_FLOAT32);
            if ( nd==1 &&  nRows==sipCpp->size() && atype==NPY_FLOAT32)
            {
                CCTRACE("OK");
                float* d = sipCpp->data();
                CCTRACE("dest address: " << d);
                float* s = (float*)PyArray_DATA(pya);
                CCTRACE("orig address: " << s);
                memcpy(d, s, nRows*sizeof(float));
                sipRes = true;
            }
            else
            {
                CCTRACE("NOK");
                sipRes = true;
            }
        }
        else
        {
            CCTRACE("fromNpArray: not a ndarray");
            sipRes = false;
        }
        %End
        
    protected:
        ~ScalarField();
    };
    
};

















