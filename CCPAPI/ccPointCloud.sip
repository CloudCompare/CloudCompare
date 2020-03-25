// sip implementation of ccPointCloud

%ExportedHeaderCode
#include <ccPointCloud.h>
%End

class ccPointCloud
{
%TypeHeaderCode
#include <ccPointCloud.h>
%End

public:
    ccPointCloud(QString name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID) throw();

    QString getName() const;
    
    SIP_PYTUPLE computeGravityCenter() [CCVector3 () ];
    %MethodCode
    Py_BEGIN_ALLOW_THREADS
    CCVector3  g = sipCpp->computeGravityCenter();
    PyObject *t;
    t = Py_BuildValue("(ddd)", g.x, g.y, g.z);
    sipRes = t;
    Py_END_ALLOW_THREADS
    %End
    
    bool hasScalarFields() const;
    
    void scale(double fx, double fy, double fz, SIP_PYTUPLE center) [ void (PointCoordinateType, PointCoordinateType, PointCoordinateType, CCVector3) ];
    %MethodCode
    double v[3];
    if (PyArg_ParseTuple(a3, "ddd", &v[0], &v[1], &v[2]))
    {
        Py_BEGIN_ALLOW_THREADS
        sipCpp->scale(a0, a1, a2, CCVector3(v[0], v[1], v[2]));
        Py_END_ALLOW_THREADS
    }
    else
    {
        // PyArg_ParseTuple() will have raised the exception.
        sipIsErr = 1;
    }
    %End
};
    