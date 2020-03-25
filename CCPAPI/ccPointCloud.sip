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
};
    