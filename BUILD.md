Compilation with CMake!
=======================

**WARNING**: if you already have a clone of the CloudCompare git repository (prior to July 2015), you may want to update/checkout the submodules with ```git submodule update --init --recursive```

Prerequisites
-------------

1.  Clone the main repository and its submodules from the main git(hub) server: <https://github.com/cloudcompare/trunk>
      
    `git clone --recursive https://github.com/cloudcompare/trunk.git`

2.  Install [CMake](http://www.cmake.org) (2.8 or newer)
3.  Install all necessary dependencies:
    - [*WINDOWS*] [Qt](http://qt-project.org/) (**version 5 WITH OPENGL** preferably but the version 4.8 is still supported)
    - [*LINUX/MAC OS X*] qt-sdk, opengl

*(refer to the [Appendix](#appendix) section if your version of Qt is older than 4.8 or if you need to compile with Qt 4.8 in 64 bits mode on Windows)*

Generating the project
----------------------

1. Launch CMake GUI (`cmake-qt-gui` on Linux)
  - *(for more convenience, you should check the "Grouped" check-box)*
  - make the `Where is the source code` field point to your local repository (for instance `C:\trunk_CC`).
  - make the `Where to build the binaries` field point to ... almost anywhere you want (**apart from the same folder as above!**). For instance: `C:\trunk_CC\build`.
  - click on the `Configure` button
  - select your generator: already tested: Visual 2008 (32/64 bits), Visual 2010 Express (32/64 bits - see appendix), Visual 2012 Express (64 bits/Qt 5), Code::Blocks (Linux & Windows 32 bits), gcc (Linux 32/64 bits, Mac OS X)
  - wait for CMake configuration/tests to finish...

2. Before clicking on the 'Generate' button, you may want to set some options,  if you expand the `OPTION` group, you'll be able to set some general options:
  - `OPTION_BUILD_CC_VIEWER`: whether to build or not the ccViewer side project (activated by default)
  - `OPTION_EXPORT_TARGETS`: not documented
  - `OPTION_MP_BUILD`: for MSVC only *(mutli-process build --> much faster, but takes almost all available CPU)*
  - `OPTION_SUPPORT_3D_CONNEXION_DEVICES`: for 3D mouses handling
  - `OPTION_SUPPORT_MAC_PDMS_FORMAT`: to activate support for PDMS .mac scripts (CAD)
  - `OPTION_USE_DXFLIB`: to activate support for DXF files in CloudCompare/ccViewer with **dxflib** - see [below](#optional-setup-for-dxflib-support)
  - `OPTION_USE_FBX_SDK`: to activate support for FBX files in CloudCompare/ccViewer with the official **FBX SDK** - see [below](#optional-setup-for-fbx-sdk-support)
  - `OPTION_USE_GDAL`: to activate support for a lot of raster files in CloudCompare/ccViewer with **GDAL** libray - see [below](#gdal_setup)
  - `OPTION_USE_LIBE57`: to activate support for E57 files in CloudCompare/ccViewer with **libE57** - see [below](#libE57_setup)
  - `OPTION_USE_LIBLAS`: to activate support for LAS files in CloudCompare/ccViewer with **libLAS** - see [below](#optional-setup-for-gdal-supportt)
  - `OPTION_USE_VISUAL_LEAK_DETECTOR`: to use the Visual Leak Detector library for MSVC (http://vld.codeplex.com/)
  - `OPTION_USE_XIOT`: to activate support for X3D files in qCC (and ccViewer) with **XIOT** - see [below](#optional-setup-for-x3dxiot-support)

3.  if you expand the `INSTALL` group, you'll be able to select which [plugin(s)|Plugins] you want to compile (by default, none are selected)
  - qBLUR *(warning: does not compile with Code::Blocks on Windows for the moment)*
  - qCork (see [below](#optional-setup-for-cork--mpir-support-for-qcork))
  - qDummy *(warning: does nothing, template for developers)*
  - qEDL
  - qHPR
  - qKinect (see [below](#optional-setup-for-libfreenect-support))
  - qPCL (requires PCL - see [below](#optional-setup-for-pcl-required-by-qpcl))
  - qPCV
  - qPoissonRecon
  - qRansacSD *(only tested on Windows for the moment)*
  - qSRA
  - qSSAO

5.  eventually, the `CMAKE` group contains a `CMAKE_INSTALL_PREFIX` which is where CloudCompare and ccViewer will be installed (when you compile the `INSTALL` project)
  - On Linux, default install dir is `/usr/local` (be sure to have administrative rights if you want to install CloudCompare there: once configured, you can call `# make install` from the sources directory)
  - On Windows Seven, you may not have the right to 'install' files in the default `Program Files` folder

### [Optional] Setup for LibLAS support

If you want to compile CloudCompare (and ccViewer) with LAS/LAZ files support, you'll need:

1.  [LibLAS](http://liblas.org) (*last tested version: 1.8 on Windows*)
2.  and optionally [laszip](http://www.laszip.org/) for LAZ files support (*last tested version: 2.2.0 on Windows*) --> prefer the static version (`BUILD_STATIC` in LASzip CMake configuration) and mind the `WITH_STATIC_LASZIP` option in libLAS CMake configuration! (*only appears in 'Advanced' mode*)
3. [Boost](http://www.boost.org/) multi-thread static libraries
  - make the `BOOST_ROOT` environment variable point to your Boost installation before launching CMake in order for the automatic `find_package` script to work properly
  - otherwise refer to LibLAS [documentation](http://liblas.org/compilation.html) for more directions

Then, the CloudCompare CMake project will request that you set the 3 following variables:

1. `LIBLAS_INCLUDE_DIR`: LibLAS include directory (pretty straightforward ;))
2. `LIBLAS_RELEASE_LIBRARY_FILE`: main LibLAS release library (the .lib or .a file itself!)
3. [Windows] `LIBLAS_SHARED_LIBRARY_FILE`: full path to the `liblas.dll` file

*For the moment, only the release version of CloudCompare supports LibLAS files*

### [Optional] Setup for LibE57 support

If you want to compile CloudCompare (and ccViewer) with LibE57 files support, you'll need:

1. [Boost](http://www.boost.org/) multi-thread static libraries (same as [libLAS](#liblas_setup))
2. [Xerces-C++](http://xerces.apache.org/xerces-c) multi-thread **static** libraries
    - On Visual C++ (Windows):
        1. select the `Static Debug` or `Static Release` configurations
        2. you'll have to manually modify the `XercesLib` project options so that the `C/C++ > Code Generation > Runtime Library` are of DLL type in both release and debug modes (i.e. `/MD` in release or `/MDd` in debug)
        3. for 64 bits version be sure to select the right platform (x64 instead of Win32). If you use Visual Studio Express 2010, be sure also that the `toolset` (in the project properties) is set to something like `Windows7.1SDK`
    - only the XercesLib project neet to be compiled
    - eventually, CMake will look for the resulting files in `/include` (instead of `/src`) and `/lib` (without the Release or Debug subfolders). By default the visual project will put them in `/Build/WinXX/VCXX/StaticXXX`. Therefore you should create a custom folder with the right organization and copy the files there.

3. [LibE57](http://libe57.org) (*last tested version: 1.1.312 on Windows*)
    - **WARNING**: with Visual Studio (at least), you'll need the libraries compiled with `/MD` (=DLL Multithreaded) in release mode and `/MDd` in debug mode. You may have to replace all `/MT` by `/MD` in the main libE57 root CMake file (or in `cmake/c_flag_overrides.cmake` and `cmake/cxx_flag_overrides.cmake` if there's no `/MT` in it)
    - If you found `set(Boost_USE_STATIC_RUNTIME ON)` in the CMake file, comment it
    - **the version 1.1.312 of libE57 has a small glitch that must be manually patched**:
        1.  open `E57FoundationImpl.cpp` and browse to the `CheckedFile::operator<<(float f)` method (line 4670)
        2.  set the output precision to 8 instead of 7! (otherwise the interal checks for precision loss may fail and libE57 will throw an exception)

The CloudCompare CMake project will only require that you set the path where libE57 has been installed (`LIBE57_INSTALL_DIR`)

### [Optional] Setup for libfreenect support

If you want to compile qKinect you'll need [OpenKinect / libfreenect](https://github.com/OpenKinect/libfreenect) (*last tested version: v0.5.0 Saturn, compiled on Windows 7 32 & 64 bits - you'll need [libusb](http://libusb.info/)* too)

Then, the CloudCompare CMake project will request that you set the 3 following variables:

1. `LIBFREENECT_INCLUDE_DIR`: libfreenect include directory (pretty straightforward ;)
2. `LIBFREENECT_LIBRARY_FILE`: main libfreenect library (the `freenect.lib` or `libfreenect.a` file itself!)
3. [Windows] ``LIBFREENECT_SHARED_LIBRARY_FILE`: full path to the `freenect.dll` file

### [Optional] Setup for X3D/XIOT support

Not ready yet.

### [Optional] Setup for PCL (required by qPCL)

If you want to compile qPCL you'll need [PCL](http://pointclouds.org/) (*last tested version: 1.7 on Windows and 1.6 on Linux*)

Follow the online instructions/tutorials. Basically, you'll need Boost, Qt, Flann and Eigen.

Once properly installed, the CloudCompare CMake script should automatically find PCL definitions. However, you'll have to set again the parameters related to Flann and Eigen.

### [Optional] Setup for dxflib support

If you want to compile CloudCompare (and ccViewer) with DXF files support, you'll need [Ribbonsoft's dxflib](http://www.ribbonsoft.com/en/dxflib-downloads) v3.3.4 or newer (last tested version: v3.3.4 on Windows)

The CMake project will only require that you set the path where dxflib sources are (`DXF_LIB_SRC_DIR`), i.e **the 'src' folder full path**

**Warning, if you use version 3.3.4:**
- edit the `dl_dxf.cpp` file
- browse to method `DL_Dxf::writeStyle`
- un-comment all commented lines (**but the `dw.dxfHex(330, 0)` call**) at the beginning and end of the method (i.e. lines 4130 to 4134, lines 4136 to 4139, and line 4181)

**Warning, version 3.7.5 syntax has changed so much that it can't be used to compile with CloudCompare anymore!**

### [Optional] Setup for FBX SDK support

If you want to compile CloudCompare (and ccViewer) with FBX files support, you'll need: The official [Autodesk's FBX SDK](http://usa.autodesk.com/adsk/servlet/pc/item?siteID=123112&id=10775847) (last tested version: 2014.2 Windows)

Then, the CloudCompare CMake project will request that you set the 3 following variables:

1. `FBX_SDK_INCLUDE_DIR`: FBX SDK include directory (pretty straightforward ;)
2. `FBX_SDK_LIBRARY_FILE`: main FBX SDK library (e.g. `libfbxsdk-md.lib`)
3. `FBX_SDK_LIBRARY_FILE_DEBUG`: main FBX SDK library for debug mode (if any)

### [Optional] Setup for GDAL support

If you want to compile CloudCompare (and ccViewer) with GDAL (raster) files support, you'll need a compiled version of the [GDAL library](http://www.gdal.org/) (last tested version: 1.10 on Windows)

Then, the CloudCompare CMake project will request that you set the 2 following variables:
1. `GDAL_INCLUDE_DIR`: GDAL include directory (pretty straightforward ;)
2. `GFAL_LIBRARY`: the static library (e.g. `gdal_i.lib`)

### [Optional] Setup for Cork + MPIR support (for qCork)

If you want to compile the qCork plugin (**on Windows only for now**), you'll need:

1. [MPIR 2.6.0](http://www.mpir.org/)
2. the forked version of the Cork library for CC: [<https://github.com/cloudcompare/cork>](https://github.com/cloudcompare/cork)
    - on Windows see the VS2010, VS2012 and VS2013 projects shipped with this fork
    - for VS2010 and VS2012 you'll have to edit the `wincork` project and update the include path for MPIR (for all configurations/platforms)
    - for VS2013 just edit the `mpir` property sheet (in the Properties manager) and update the MPIR macro (in the `User macros` tab)

Then, the CloudCompare CMake project will request that you set the following variables:

1. `CORK_INCLUDE_DIR` and `MPIR_INCLUDE_DIR`: both libraries include directories (pretty straightforward ;)
2. `CORK_RELEASE_LIBRARY_FILE` and `MPIR_RELEASE_LIBRARY_FILE`: both main library files
3. and optionally `CORK_DEBUG_LIBRARY_FILE` and `MPIR_DEBUG_LIBRARY_FILE`: both main library files (for debug mode)

### Generate the project files

Once all red items have disappeared (click multiple times on `Configure` if necessary), you can go ahead! Click on the `Generate button to create the corresponding project files.

Compiling the project
---------------------

Open the resulting project with the generator you have previously chosen (the file(s) should be where you told CMake to *build the binaries* - e.g. `C:\trunk_CC\build`).

You should (always?) found the two following configuration/sub-projects:

1. `build all` should do all the compilation work (in the right order) but the binaries and libraries will be generated (by default) among all the other compilation files, in a somewhat complicated folder tree structure.
2.  `install` should export all these files to the `CMAKE_INSTALL_PREFIX` folder, placing everything where it should be (and almost exactly as the official binary build)

### Working with Visual Studio on Windows

As all the files (executables, plugins and other DLLs) are copied in the `CMAKE_INSTALL_PREFIX` directory, the standard Visual Studio mechanism is broken and you won't be able to 'run' the CloudCompare or ccViewer projects as is. See [this post](http://www.danielgm.net/cc/forum/viewtopic.php?t=992) on the forum to setup Visual correctly.

Appendix
========

Common issues
-------------

On Linux, you may encounter issues with shared libraries (.so files) if the project is not installed in `/usr`. In this case:

1. either set the `LD_LIBRARY_PATH` variable so that it points to the qCC and ccViewer installation folders (`export LD_LIBRARY_PATH=...`).
2. or call  `# /sbin/ldconfig -v` once as suggested [here](http://www.danielgm.net/cc/forum/viewtopic.php?f=10&t=195&p=602#p600)

If you use a version of Qt older than 4.7, you'll get issues with `QElapsedTimer` (for instance with Ubuntu Lucid or Scientific Linux). In this case you'll have to replace the `#include <QElapsedTimer>` lines by:

```
//#include  <QElapsedTimer>
#include `<QTime>
typedef QTime QElapsedTimer;
```
- in `cloudcompare/libs/qCC_db/ccTimer.h`, after `//Qt`
- in `cloudcompare/libs/qCC/ccCommandLineParse.cpp`, after `#include <QDateTime>`
- in `cloudcompare/libs/qCC/ccComparisonDlg.cpp`, after `//Qt`
- in `cloudcompare/libs/qCC/ccGLWindow.cpp`, after `#include <QWheelEvent>`
- in `cloudcompare/libs/qCC/ccSubsamplingDlg.cpp`, after `//Qt`
- in `cloudcompare/libs/qCC/mainwindow.cpp`, after `#include <QMessageBox>`

Compiling the 64 bits version with Qt 4.8 on Windows
----------------------------------------------------

Here are several "hints" regarding the compilation on Windows 64 bits:

1.  if only Visual 2010 Express is installed, Cmake will require you to install Microsoft Windows SDK 7.1
2.  to compile with Visual 2010 you must then apply the following patch: [1](http://support.microsoft.com/kb/2280741) (this solves a bug from the MSVC 2010 compiler that prevents Qt from running correctly in release mode)
3.  you'll also have to compile Qt 4.8 yourself, as Nokia never provided the corresponding binaries:
    - download and decompress the Qt sources
    - start the Microsoft Windows SDK 7.1 command line mode with the dedicated shortcut (you'll have to add `/Release` at the end of the shortcut - edit the shortcut properties for that). Something like: `C:\Windows\System32\cmd.exe /E:ON /V:ON /T:0E /K "C:\Program Files\Microsoft SDKs\Windows\v7.1\Bin\SetEnv.cmd" /Release`
    - if you don't have perl installed, just delete the `bin/syncqt.bat` file
    - And follow this [guide](http://www.holoborodko.com/pavel/2011/02/01/how-to-compile-qt-4-7-with-visual-studio-2010/)

Otherwise use Visual 2012 with Qt 5 64 bits ;)
