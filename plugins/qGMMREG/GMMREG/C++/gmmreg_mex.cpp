/* This mexFunction was written based on the Example: edit([matlabroot '/extern/examples/refbook/revord.c']); */

#include "gmmreg_api.h"
#include "mex.h"
#include "string.h"

/* #define MAXCHARS 80   /* max length of string contained in each field */

/*  the gateway routine.  */
void mexFunction(int nlhs,       mxArray *plhs[],
         int nrhs, const mxArray *prhs[])
{
    /* Declare variables */
    char *f_config;
    char *method;


    /* Check for proper number of input and output arguments */
    if (nrhs != 2) {
        mexErrMsgTxt("Two input arguments required.");
    }
    else if (nlhs > 0){
        mexErrMsgTxt("No output argument.");
    }

    /* Check data type of input argument */
    /* input must be a string */
    if ( mxIsChar(prhs[0]) != 1)
      mexErrMsgTxt("Input must be a string.");

    /* input must be a row vector */
    //if (mxGetM(prhs[0])!=2)
    //  mexErrMsgTxt("Input must be a row vector.");


    /* copy the string data from prhs[0] into a C string. */
    f_config = mxArrayToString(prhs[0]);

    if(f_config == NULL)
      mexErrMsgTxt("Could not convert config to string.");

    method = mxArrayToString(prhs[1]);

    if(method == NULL)
      mexErrMsgTxt("Could not convert method to string.");

    /* call the C subroutine */
    gmmreg_api(f_config, method);

    /* no output, clean up */
    mxFree(f_config);
    mxFree(method);
    return;
}
