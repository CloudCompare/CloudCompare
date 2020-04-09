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
        
        SIP_PYTUPLE computeMeanAndVariance() [ void (ScalarType&, ScalarType*) ];
        %Docstring
        Return a tuple (mean, variance)
        %End
        %MethodCode
        Py_BEGIN_ALLOW_THREADS
        ScalarType mean;
        ScalarType variance;
        CCTRACE("computeMeanAndVariance");
        sipCpp->computeMeanAndVariance(mean, &variance);
        CCTRACE("mean: " << mean);
        CCTRACE("variance: " << variance);
        PyObject *t;
        t = Py_BuildValue("(dd)", mean, variance);
        sipRes = t;
        Py_END_ALLOW_THREADS
        %End
        
        void computeMinAndMax();
        %Docstring
        Should be called before getMin or getMax.
        No return!
        %End
        
        float getMin() [ ScalarType ()];
        %Docstring
        Return minimal value of the ScalarField.
        Call computeMinAndMax before!
        %End
        
        float getMax() [ ScalarType ()];
        %Docstring
        Return maximal value of the ScalarField.
        Call computeMinAndMax before!
        %End
    
        static void initNumpyApi();
        %Docstring
        Should be done once before transforming ScalarField in numpyArray or the other way.
        %End
        %MethodCode
        CCTRACE("initialization numpy C++ API");
        import_array();
        CCTRACE("initialization numpy C++ API done");
        %End

        PyObject* toNpArray();
        %Docstring
        Wrap the ScalarField data into a numpy Array, no copy.
        (Only float) Data ownership stays in CloudCompare C++ library.
        TODO: detect double option, check ownership...
        %End
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
        %Docstring
        Copy data from numpy array into an existing ScalarField.
        Check if the numpy array is of float, one dimension, same size as the ScalarField,
        before overwriting ScalarField data.
        %End
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
                CCTRACE("NOK (dim, size or type");
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

















