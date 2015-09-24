#include <Python.h>
#include "gmmreg_api.h"

static PyObject *
py_gmmreg_api(PyObject *self, PyObject *args)
{
    char *f_config;
    char *method;
    /* send Python arrays to C */
    if (!PyArg_ParseTuple(args, "ss",  &f_config, &method))
    {
        return NULL;
    }

    /* call function */
    gmmreg_api(f_config,method);

    /* send the result back to Python */
    //Py_DECREF(f_config);
    return Py_BuildValue("s", f_config);
}

static PyMethodDef GMMRegMethods[] = {
    {"gmmreg_api",  py_gmmreg_api, METH_VARARGS,
     "Robust Point Set Registration Using Mixture of Gaussians."},
     {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initpygmmreg(void)
{
    (void) Py_InitModule("pygmmreg", GMMRegMethods);
    //import_array();
}
