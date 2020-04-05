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

// sip implementation of ccPointCloud

%ExportedHeaderCode
#include <ccPointCloud.h>
%End

class ccPointCloud
{
%TypeHeaderCode
#include <ccPointCloud.h>
//#include <ScalarField.h>
%End

%Docstring
A 3D cloud and its associated features (color, normals, scalar fields, etc.)
A point cloud can have multiple features:
- colors (RGB)
- normals (compressed)
- scalar fields
- an octree strucutre
- per-point visibility information (to hide/display subsets of points)
- other children objects (meshes, calibrated pictures, etc.)
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
    
    void scale(double fx, double fy, double fz, SIP_PYTUPLE center = Py_BuildValue("(ddd)", 0., 0., 0.)) [ void (PointCoordinateType, PointCoordinateType, PointCoordinateType, CCVector3) ];
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
    
    void translate(SIP_PYTUPLE t) [ void (const CCVector3&) ];
    %MethodCode
    double v[3];
    if (PyArg_ParseTuple(a0, "ddd", &v[0], &v[1], &v[2]))
    {
        Py_BEGIN_ALLOW_THREADS
        sipCpp->translate(CCVector3(v[0], v[1], v[2]));
        Py_END_ALLOW_THREADS
    }
    else
    {
        // PyArg_ParseTuple() will have raised the exception.
        sipIsErr = 1;
    }
    %End
    
    bool exportCoordToSF(SIP_PYTUPLE b) [ bool (bool*) ];
    %MethodCode
    int v[3];
    if (PyArg_ParseTuple(a0, "ppp", &v[0], &v[1], &v[2]))
    {
        bool b[3];
        b[0] = v[0]; b[1] = v[1]; b[2] = v[2];
        Py_BEGIN_ALLOW_THREADS
        sipRes = sipCpp->exportCoordToSF(b);
        Py_END_ALLOW_THREADS
    }
    else
    {
        sipRes = false;
        // PyArg_ParseTuple() will have raised the exception.
        sipIsErr = 1;
    }
    %End
    
    unsigned getNumberOfScalarFields() const;
    
    CCLib::ScalarField* getScalarField(int index) const;
    
    const char* getScalarFieldName(int index) const;
};
    