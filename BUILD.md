# Compilation of CloudCompare 2.7+ (with CMake)

**WARNING**: if you already have a clone of the CloudCompare git repository (prior to July 2015), you may want to update/checkout the submodules with ```git submodule update --init --recursive```

## Prerequisites

1.  Clone the main repository and its submodules from the main git(hub) server: <https://github.com/cloudcompare/CloudCompare>

    `git clone --recursive https://github.com/cloudcompare/CloudCompare.git`

2.  Install [CMake](http://www.cmake.org) (3.0 or newer)
3.  Install Qt (http://www.qt.io/ - for *Linux/Mac OS X*: qt-sdk)
      * CloudCompare 2.7 requires **Qt version 5.5** or newer

4. Make sure you have a C++11 compliant compiler (gcc 4.7+ / clang / Visual 2013 and newer)

*To compile the project with older versions of Qt (from 4.8 to 5.4) or with a non C++11 compliant compiler, you'll have to stick with the https://github.com/cloudcompare/CloudCompare/releases/tag/v2.6.3.1 version*

## Generating the project

1. Launch CMake GUI (`cmake-qt-gui` on Linux, the CMake application on Mac OS X)
  - *(for more convenience, you should check the "Grouped" check-box)*
  - set the `Where is the source code` field to your local repository (for instance `C:\CloudCompare\CloudCompare`)
  - set the `Where to build the binaries` field to ... almost anywhere you want **apart from the same folder as above or the *Program Files* folder (on Windows)**. (for instance: `C:\CloudCompare\build`)
  - click the `Configure` button
  - select the generator for the project  
    The following generators have been tested:
      - Visual 2013, 2015, 2017 (64 bits)
      - gcc (Linux 64 bits)
      - Unix Makefiles (Mac OS X)
      - CodeBlocks - Unix Makefiles (Mac OS X)
  - wait for CMake configuration/tests to finish...
  - on the first run you may have to manually set the **QT5_ROOT_PATH** variable. Make it point to your installation of Qt (on Windows it's where the 'bin' folder lies - e.g. *Qt\5.6\msvc2013_64*)

2. Before clicking on the 'Generate' button, you may want to set some more options. If you expand the `OPTION` group, you'll be able to set some general options:
  - `OPTION_BUILD_CCVIEWER`: whether or not to build the ccViewer side project (ON by default)
  - `OPTION_USE_DXF_LIB`: to add support for DXF files in CloudCompare/ccViewer with **dxflib** - see [below](#optional-setup-for-dxflib-support)
  - `PLUGIN_IO_QFBX`: to add support for FBX files in CloudCompare/ccViewer with the official **FBX SDK** - see [below](#optional-setup-for-fbx-sdk-support)
  - `OPTION_USE_GDAL`: to add support for a lot of raster files in CloudCompare/ccViewer with **GDAL** library - see [below](#optional-setup-for-gdal-support)
  - `PLUGIN_IO_QE57`: to add support for E57 files in CloudCompare/ccViewer with **libE57** - see [below](#optional-setup-for-libe57-support)
  - `OPTION_USE_SHAPE_LIB`: to add support for SHP files in CloudCompare/ccViewer
  - `PLUGIN_IO_QPDAL`: to add support for LAS files in CloudCompare/ccViewer with **PDAL** - see [below](#optional-setup-for-las-using-pdal)

  The following are Windows-only options:
  - `OPTION_MP_BUILD`: for Visual Studio only *(multi-process build --> much faster but uses a lot of CPU power)*
  - `OPTION_SUPPORT_3D_CONNEXION_DEVICES`: for 3D mouses handling
  - `OPTION_USE_OCULUS_SDK`: to add support for the Oculus Rift SDK in CloudCompare/ccViewer (*work in progress*)
  - `OPTION_USE_VISUAL_LEAK_DETECTOR`: to use the Visual Leak Detector library for MSVC (http://vld.codeplex.com/)

3.  If you expand the `INSTALL` group, you'll be able to select the plugin(s) you want to compile.  By default, none are selected and **none are required** to work with CloudCompare. See http://www.cloudcompare.org/doc/wiki/index.php?title=Plugins.
  - qAnimation *(relies on ffmpeg - https://www.ffmpeg.org/ - to generate video files)*
  - qBlur
  - qCork (see [below](#optional-setup-for-cork--mpir-support-for-qcork))
  - qDummy *(does nothing, template for developers)*
  - qCSV_MATRIX_IO *(to load CSV matrix files)*
  - qEDL
  - qFacets
  - qGMMReg *(relies on VXL)*
  - qHPR
  - qKinect *(warning: discontinued & no longer supported)*
  - qPCL (requires PCL - see [below](#optional-setup-for-pcl-required-by-qpcl))
  - qPCV
  - qPoissonRecon *(note: be sure to update the PoissonRecon submodule - see above)*
  - qRansacSD *(mainly tested on Windows but works with flaws on Linux)*
  - qSRA
  - qSSAO

4. _(Mac OS X only)_

  If you are compiling and running locally, add `-DCC_MAC_DEV_PATHS` to the `CMAKE_CXX_FLAGS` in the `CMAKE` group.  This will look for the plugins in your build directory rather than the application bundle.  If you need the shaders as well, you will have to create a `shaders` folder in the build directory and copy the shaders you need into it.

5.  Last but not least, the `CMAKE` group contains a `CMAKE_INSTALL_PREFIX` variable which is where CloudCompare and ccViewer will be installed (when you compile the `INSTALL` project)
  - On Linux, default install dir is `/usr/local` (be sure to have administrative rights if you want to install CloudCompare there: once configured, you can call `# make install` from the sources directory)
  - On Windows 7/8/10 CMake doesn't have the rights to 'install' files in the `Program Files` folder (even though it's CMake's default installation destination!)

## Generate the project files

Once all CMake errors have been resolved (you may to click multiple times on `Configure` if necessary)  be sure to click on the 'Generate' at least once at the end. This will create the project files for the compiler/IDE you have selected at the beginning. **At this point the project still needs to be compiled**.

## Compiling the project

Eventually you can run the compiler on the generated cmake file or open the project (e.g. for Visual Studio). The resulting files should be where you told CMake to *build the binaries* (e.g. `C:\CloudCompare\build`).

*You should always find the two following configuration/sub-projects*:

1. `build all`: does all the compilation work (in the right order) but the binaries and libraries will be generated (by default) among all the other compilation files, in a somewhat complicated folder tree structure.
2.  `install`: copies all the necessary files (executable, resources, plugins, DLLs etc.) to the `CMAKE_INSTALL_PREFIX` folder. **This is mandatory to actually launch CloudCompare or ccViewer.**


The Mac OS X install/release process is still a bit less automated than the others. If you are putting together a complete install (using `make install`), you will need to change the `PATH_PREFIX` variable in the script `mac_scripts/delete_rpaths.sh`.  Please see the comment in that file and if you know how to solve it, please submit a patch.

### Working with Visual Studio on Windows

As all the files (executables, plugins and other DLLs) are copied in the `CMAKE_INSTALL_PREFIX` directory, the standard project launch/debug mechanism is broken. Therefore by default you won't be able to 'run' the CloudCompare or ccViewer projects as is (with F5 or Ctrl + F5 for instance). See [this post](http://www.danielgm.net/cc/forum/viewtopic.php?t=992) on the forum to setup Visual correctly.

### Debugging plugins

If you want to use or debug plugins in DEBUG mode while using a single configuration compiler/IDE (gcc, etc.) the you'll have to comment the automatic definition of the `QT_NO_DEBUG` macro in '/plugins/CMakePluginTpl.cmake' (see http://www.cloudcompare.org/forum/viewtopic.php?t=2070).

# Appendix

## Additional optional CMake setup steps

### [Optional] Setup for the qPoissonRecon plugin

1. The version of the Poisson Surface Reconstruction library (M. Kazhdan et al.) used by the  is https://github.com/cloudcompare/PoissonRecon. It is declared as a submodule of CC's repository. You have to explicitly synchronize it (see https://git-scm.com/docs/git-submodule).
2. Then simply check the INSTALL_QPOISSON_RECON_PLUGIN option in CMake

### [Optional] Setup for LAS using PDAL

If you want to compile CloudCompare (and ccViewer) with LAS/LAZ files support, you'll need:

1. [PDAL](https://pdal.io/)
2. Set `PLUGIN_IO_QPDAL=TRUE`

If your PDAL installation is not correctly picked up by CMake, 
set the `PDAL_DIR` to the path containing `PDALConfig.cmake`.

### [Optional] Setup for LibE57 support

If you want to compile CloudCompare (and ccViewer) with LibE57 files support, you'll need:

1. [Xerces-C++](http://xerces.apache.org/xerces-c) multi-thread **static** libraries
    - On Ubuntu install the package `libxerces-c-dev`
    - On Visual C++ (Windows):
        1. select the `Static Debug` or `Static Release` configurations
        2. you'll have to manually modify the `XercesLib` project options so that the `C/C++ > Code Generation > Runtime Library` are of DLL type in both release and debug modes (i.e. `/MD` in release or `/MDd` in debug)
        3. for 64 bits version be sure to select the right platform (x64 instead of Win32). If you use Visual Studio Express 2010, be sure also that the `toolset` (in the project properties) is set to something like `Windows7.1SDK`
    - only the XercesLib project neet to be compiled
    - eventually, CMake will look for the resulting files in `/include` (instead of `/src`) and `/lib` (without the Release or Debug subfolders). By default the visual project will put them in `/Build/WinXX/VCXX/StaticXXX`. Therefore you should create a custom folder with the right organization and copy the files there.

2. [LibE57](https://github.com/asmaloney/libE57Format) (*last tested version: 2.0.1 on Windows*)
    - Checkout the submodule in `plugins/core/IO/qE57IO/extern/libE57Format` or download and extract the latest [libE57Format](https://github.com/asmaloney/libE57Format) release

### [Optional] Setup for PCL (required by qPCL)

If you want to compile qPCL you'll need [PCL](http://pointclouds.org/) (*last tested version: 1.8 on Windows and 1.6 on Linux*)

Follow the online instructions/tutorials. Basically, you'll need Boost, Qt, Flann and Eigen.

Once properly installed, the CloudCompare CMake script should automatically find PCL definitions. However, you'll have to set again the parameters related to Flann and Eigen.

### [Optional] Setup for FBX SDK support

If you want to compile CloudCompare (and ccViewer) with FBX files support, you'll need: The official [Autodesk's FBX SDK](http://usa.autodesk.com/adsk/servlet/pc/item?siteID=123112&id=10775847) (last tested version: 2015.1 on Windows)

Then, the CloudCompare CMake project will request that you set the 3 following variables:

1. `FBX_SDK_INCLUDE_DIR`: FBX SDK include directory (pretty straightforward ;)
2. `FBX_SDK_LIBRARY_FILE`: main FBX SDK library (e.g. `libfbxsdk-md.lib`)
3. `FBX_SDK_LIBRARY_FILE_DEBUG`: main FBX SDK library for debug mode (if any)

### [Optional] Setup for GDAL support

If you want to compile CloudCompare (and ccViewer) with GDAL (raster) files support, you'll need a compiled version of the [GDAL library](http://www.gdal.org/) (last tested version: 1.10 on Windows, 2.0.2 on Mac OS X)

Then, the CloudCompare CMake project will request that you set the 2 following variables:
1. `GDAL_INCLUDE_DIR`: GDAL include directory (pretty straightforward ;)
2. `GDAL_LIBRARY`: the static library (e.g. `gdal_i.lib`)

### [Optional] Setup for Cork + MPIR support (for qCork)

If you want to compile the qCork plugin (**on Windows only for now**), you'll need:

1. [MPIR 2.6.0](http://www.mpir.org/)
2. the forked version of the Cork library for CC: [<https://github.com/cloudcompare/cork>](https://github.com/cloudcompare/cork)
    - on Windows see the Visual project shipped with this fork and corresponding to your version (if any ;)
    - for VS2013 just edit the `mpir` property sheet (in the Properties manager) and update the MPIR macro (in the `User macros` tab)

Then, the CloudCompare CMake project will request that you set the following variables:

1. `CORK_INCLUDE_DIR` and `MPIR_INCLUDE_DIR`: both libraries include directories (pretty straightforward ;)
2. `CORK_RELEASE_LIBRARY_FILE` and `MPIR_RELEASE_LIBRARY_FILE`: both main library files
3. and optionally `CORK_DEBUG_LIBRARY_FILE` and `MPIR_DEBUG_LIBRARY_FILE`: both main library files (for debug mode)
