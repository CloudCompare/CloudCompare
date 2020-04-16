Cloud Compare Python API TODO list & questions
==============================================

Wrapping: PyQt SIP, or migration to Qt for Python (Pyside2 Shiboken2)?
----------------------------------------------------------------------

CloudCompare relies on Qt: PyQt - SIP is more suitable than SWIG, but a bit laborious ...
With recent versions of Qt (5.12 -), Qt offers Pyside2 and Shiboken2.
For a first test, on Ubuntu 18.4, (Qt 5.9.5), I stayed at PyQt -SIP, with the native system packages.
Should we switch to Pyside2 / Shiboken2?
See https://machinekoder.com/pyqt-vs-qt-for-python-pyside2-pyside/
If yes, migration to be planned SIP -> Shiboken2, later, when the required Qt version will be readily available on platforms (for Linux, native distribution package)
On the Python side, great compatibility of scripts between Pyside2 and PyQt ==> no significant impact for future users.

CMakefile for PYQT-SIP
----------------------
No ready-made detection: FindPYQTSIP.cmake inspired by SALOME works, and probably incomplete.
A CMake proposed by Qt_Python_Binding (https://github.com/ros-visualization/python_qt_binding.git) requires too many other prerequisites ...
Tests with FindPYQTSIP.cmake:
- Ubuntu 18.4 with native prerequisites
- Windows 10, Visual Studio 2017, prerequisites provided by Anaconda.
- tests to be continued on different configurations

Windows 10 generation and testing
-----------------------------------
To test, I launched Visual Studio 2017 in the Anaconda environment, to facilitate the detection of prerequisites.
Some paths must be given at the configuration step for Sip, PyQt5, Numpy ...

Wrapping extension
---------------------
Very limited interface, for first tests:

cloudCompare
- read / write point clouds
- curvature calculation
- scalar field filtering
ccPointCloud
- computeGravityCenter
- scale, translate
- getName, hasScalarFields, getNumberOfScalarFields, getScalarFieldName
- exportCoordToSF
- getScalarField
- ...
ScalarField
- getName
- initNumpyApi (static to call once)
- toNpArray
- fromNpArray

The idea is to complete by taking inspiration from the functions available in the command interface.
To do as a priority: export points (x, y, z) to a Numpy array.

Duplicate qCC code elements
---------------------------
The Cloud Compare Python API code is gathered in a CCPAPI directory (Cloud Compare Python API).
It is based on potentially all the CloudCompare libraries, except the application itself (qCC).
It does not require any modification of the called libraries.
By cons, need to duplicate code from qCC, without GUI graphics calls.
For example, for the calculation of curvature, retrieved from ccLibAlgorithms :: ComputeGeomCharacteristic, ccLibAlgorithms :: GetDensitySFName.
Do we need an intermediate layer of callable processing from the GUI and the Python interface?

ScalarField <--> Numpy Array: always floating type (double type not implemented)
--------------------------------------------------------------------------------
Implement the ScalarType == double option for ScalarField <--> Numpy Array conversions.

Numpy, ownership of ScalarFields
--------------------------------
It is possible to transform a ScalarField into Numpy Array without copying.
Normally, ownership remains on the C ++ side: the destruction of ScalarField can only be done via a CloudCompare method (explicit method to add to the Python interface).
In the other direction, we can overwrite the data of an existing ScalarField with that of a Numpy Array of the same type, same dimensions and size (vector of the same number of elements).
Here, the operation is done by copy (memcpy).
- Build tests to ensure that premature destruction or memory leaks are avoided.

Python references counter, Ownership
---------------------------------------
Python's garbage collector relies on a references counter on which we can act in the C ++ interface (Py_INCREF, PY_DECREF).
SIP makes it possible to say explicitly who has the ownership of the objects created at the interface.
Establish rules and tests to be sure of writing reliable code (Numpy Array <--> ScalarField is an important particular case).
NB: no problem detected to date on the reduced interface, with simple tests ...

Portable Python tests
----------------------
The test scripts are dependent on local paths, and on a local dataset.
TODO:
- dataset generated or easily accessible.
- Scripts configured to run in the end user environment.
- Use CTest (or another test tool) during construction, deployment?



