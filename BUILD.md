# Compilation of CloudCompare 2.13+

## 1. Base dependencies

CloudCompare requires [CMake](http://www.cmake.org) to be built.

The main dependency of CloudCompare is Qt. CloudCompare 2.13+ requires 5.12 <= Qt < 6.0.

- On Windows it is recommended to use the installer from the [Qt website](https://www.qt.io/).
- On macOs you can also use the installer from Qt's website, or use homebrew.
- On Linux it is recommended to use your distribution's package manager:

Debian/ubuntu package names:
``` console
libqt5svg5-dev libqt5opengl5-dev qt5-default qttools5-dev qttools5-dev-tools libqt5websockets5-dev
```

## 2. Cloning

The command to clone CloudCompare is:

`git clone --recursive https://github.com/cloudcompare/CloudCompare.git`

The `--recursive` **is important** as CloudCompare uses Git's submodules system to 'vendor' external libraries
or plugins.

## 3. CMake Configuration

When doing a basic build of CloudCompare, the only CMake arguments that is required
is the `CMAKE_PREFIX_PATH` to tell CMake where to find your Qt installation.

This is only required on **Windows** and **macOs** as on Linux Qt is installed in a standard location
that CMake will find.

You can also use the `CMAKE_INSTALL_PREFIX` to customize the path where CloudCompare will be installed.

If you use CMake-GUI the options are the same, you just enter the values in the GUI.

Example:
```shell
# Windows
mkdir build & cd build
cmake -DCMAKE_PREFIX_PATH=C:\Qt\5.15.2\msvc2019_64 ..

# macOs
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=/usr/local/opt/qt@5 ..

# Linux
mkdir build && cd build
cmake ..
```

You can always take a look at how CloudCompare is being build on [GitHub's CI](.github/workflows/build.yml)

## 4. Build

```
# Still in the build folder
cmake --build .
```

## 5. Install

```console
# Still in the build folder
cmake --install .
```

## Optional features and plugins

CloudCompare has a bunch of optional features and plugins not built by default.
Below you will find a list of these options/plugins. To add an option to CMake use the `-DMY_OPTION=ON` syntax.
Example, to build the `qEDL` plugin, add `-DPLUGIN_GL_QEDL=ON` argument to CMake at the configuration step.

The optional features are:

|     CMake Option      | Default Value | Description
|-----------------------|---------------|-------------
| OPTION_BUILD_CCVIEWER |      ON       | Whether or not to build the ccViewer side project.
| OPTION_USE_SHAPE_LIB  |      ON       | Use the vendored shapelib to add support for SHP files.
| OPTION_USE_DXF_LIB    |      ON       | Use the vendored dxflib to add support for DXF files.
| OPTION_USE_GDAL       |      OFF      | Add support for a lot of raster files in CloudCompare/ccViewer with **GDAL** library.


The following options are **Windows-only**:

|     CMake Option                    | Default Value | Description
|-------------------------------------|---------------|-------------
| OPTION_MP_BUILD                     | OFF           | Visual Studio only *(multi-process build --> much faster but uses a lot of CPU power)*
| OPTION_SUPPORT_3D_CONNEXION_DEVICES | OFF           | 3D mouses handling
| OPTION_USE_OCULUS_SDK               | OFF           | Build with Oculus SDK (LibOVR) support.
| OPTION_USE_VISUAL_LEAK_DETECTOR     | OFF           | To use the Visual Leak Detector library for MSVC (http://vld.codeplex.com/)

The available plugins are

## GL Plugins

|       Plugin Name       |         CMake Option                     | Default Value | Description
|-------------------------|------------------------------------------|---------------|-------------
| qEDL                    | PLUGIN_GL_QEDL                           | OFF           |
| qSSAO                   | PLUGIN_GL_QSSAO                          | OFF           |

## IO Plugins

|       Plugin Name       |         CMake Option                     | Default Value | Description
|-------------------------|------------------------------------------|---------------|-------------
| qAdditionalIO           | PLUGIN_IO_QADDITIONAL                    | OFF           |
| qCoreIO                 | PLUGIN_IO_QCORE                          | ON            |
| qCSVMatrixIO            | PLUGIN_IO_QCSV_MATRIX                    | OFF           | Add support for CSV matrix files.
| qDraco                  | PLUGIN_IO_QDRACO                         | OFF           | Add support force draco files
| qE57IO                  | PLUGIN_IO_QE57                           | OFF           | Add support for e57 files using **libE57**.
| qFBXIO                  | PLUGIN_IO_QFBX                           | OFF           | Add support for AutoDesk FBX files using the official **FBX SDK**
| qLASFWIO                | PLUGIN_IO_QLAS_FWF                       | OFF           | Windows only. Support for LAS/LAZ with and without waveform using LIBlas (***deprecated, consider using qLASIO instead***).
| qLASIO                  | PLUGIN_IO_QLAS                           | OFF           | Support for LAS/LAZ with and without waveform (all platforms) using **LASZIP**.
| qPDALIO                 | PLUGIN_IO_QPDAL                          | OFF           | Add support for LAS/LAZ files using PDAL (***deprecated, consider using qLASIO instead***).
| qPhotoscanIO            | PLUGIN_IO_QPHOTOSCAN                     | OFF           |
| qRDBIO                  | PLUGIN_IO_QRDB                           | OFF           | Add support for RDB.
| qStepCADImport          | PLUGIN_IO_QSTEP                          | OFF           | Add support for STEP files.

## Standard Plugins

|       Plugin Name       |         CMake Option                     | Default Value | Description
|-------------------------|------------------------------------------|---------------|-------------
| q3DMASC                 | PLUGIN_STANDARD_3DMASC                   | OFF           | Automatic point cloud classification: https://lidar.univ-rennes.fr/en/3dmasc
| qAnimation              | PLUGIN_STANDARD_QANIMATION               | OFF           | Plugin to create videos: https://www.cloudcompare.org/doc/wiki/index.php/Animation_(plugin).
| qBroom                  | PLUGIN_STANDARD_QBROOM                   | OFF           | Interactive cloud cleaning tool: https://www.cloudcompare.org/doc/wiki/index.php/Virtual_broom_(plugin)
| qCanupo                 | PLUGIN_STANDARD_QCANUPO                  | OFF           | Automatic point cloud classification: https://www.cloudcompare.org/doc/wiki/index.php/CANUPO_(plugin)
| qCloudLayers            | PLUGIN_STANDARD_QCLOUDLAYERS             | OFF           | Manual point cloud classification/labelling: https://www.cloudcompare.org/doc/wiki/index.php/QCloudLayers_(plugin)
| qColorimetricSegmenter  | PLUGIN_STANDARD_QCOLORIMETRIC_SEGMENTER  | OFF           | Point cloud color-based segmentation: https://www.cloudcompare.org/doc/wiki/index.php/Colorimetric_Segmenter_(plugin)
| qCompass                | PLUGIN_STANDARD_QCOMPASS                 | OFF           | Digitization of geological structures and structural traces on point clouds: https://www.cloudcompare.org/doc/wiki/index.php/Compass_(plugin)
| qCork                   | PLUGIN_STANDARD_QCORK                    | OFF           | Mesh Boolean Operations: https://www.cloudcompare.org/doc/wiki/index.php/Cork_(plugin)
| qCSF                    | PLUGIN_STANDARD_QCSF                     | OFF           | Automatic ground/non-ground classification: https://www.cloudcompare.org/doc/wiki/index.php/CSF_(plugin)
| qFacets                 | PLUGIN_STANDARD_QFACETS                  | OFF           | Structural geology plugin: https://www.cloudcompare.org/doc/wiki/index.php/Facets_(plugin)
| qHoughNormals           | PLUGIN_STANDARD_QHOUGH_NORMALS           | OFF           | Normals computation: https://www.cloudcompare.org/doc/wiki/index.php/HoughNormals_(plugin)
| qHPR                    | PLUGIN_STANDARD_QHPR                     | OFF           | Hidden Point Removal: https://www.cloudcompare.org/doc/wiki/index.php/Hidden_Point_Removal_(plugin)
| qJSonRPCPlugin          | PLUGIN_STANDARD_QJSONRPC                 | OFF           | Json/RPC control plugin
| qM3C2                   | PLUGIN_STANDARD_QM3C2                    | OFF           | Robust point cloud distances computation: https://www.cloudcompare.org/doc/wiki/index.php/M3C2_(plugin)
| qMasonry                |                                          |               | Segmentation of masonry structures: https://www.cloudcompare.org/doc/wiki/index.php/Masonry_Segmentation_(plugin)
|  - qAutoSeg             | PLUGIN_STANDARD_MASONRY_QAUTO_SEG        | OFF           |
|  - qManualSeg           | PLUGIN_STANDARD_MASONRY_QMANUAL_SEG      | OFF           |
| qMeshBoolean            | PLUGIN_STANDARD_QMESH_BOOLEAN            | OFF           | Mesh Boolean Operations: https://www.cloudcompare.org/doc/wiki/index.php/Mesh_Boolean_(plugin)
| qMPlane                 | PLUGIN_STANDARD_QMPLANE                  | OFF           | Normal distance measurements against a defined plane: https://www.cloudcompare.org/doc/wiki/index.php/MPlane_(plugin)
| qPCL                    | PLUGIN_STANDARD_QPCL                     | OFF           | Interface to some algorithms of the PCL library: https://www.cloudcompare.org/doc/wiki/index.php/Point_Cloud_Library_Wrapper_(plugin)
| qPCV                    | PLUGIN_STANDARD_QPCV                     | OFF           | Ambient Occlusion for meshes or point clouds: https://www.cloudcompare.org/doc/wiki/index.php/ShadeVis_(plugin)
| qPoissonRecon           | PLUGIN_STANDARD_QPOISSON_RECON           | OFF           | Surface Mesh Reconstruction: https://www.cloudcompare.org/doc/wiki/index.php/Poisson_Surface_Reconstruction_(plugin)
| qRANSAC_SD              | PLUGIN_STANDARD_QRANSAC_SD               | OFF           | Automatic RANSAC shape detection: https://www.cloudcompare.org/doc/wiki/index.php/RANSAC_Shape_Detection_(plugin)
| qSRA                    | PLUGIN_STANDARD_QSRA                     | OFF           | Surface of Revolution Analysis: https://www.cloudcompare.org/doc/wiki/index.php/Surface_of_Revolution_Analysis_(plugin)
| qTreeIso                | PLUGIN_STANDARD_QTREEISO                 | OFF           | Individual Tree Isolation: https://www.cloudcompare.org/doc/wiki/index.php/Treeiso_(plugin)

## Additional Instructions

Some plugins do not have their dependencies vendored in CloudCompare's repo, thus they require additional steps
to install them. This section aims to document the steps for these plugins.

### qAnimation

The qAnimation plugin has an additional option `QANIMATION_WITH_FFMPEG_SUPPORT` to be able to create
movies with FFMPEG.

### qE57IO

If you want to compile CloudCompare (and ccViewer) with LibE57 files support, you'll need:

1. [Xerces-C++](http://xerces.apache.org/xerces-c) multi-thread **static** libraries
    - On Ubuntu install the package `libxerces-c-dev`
    - On Visual C++ (Windows):
        1. select the `Static Debug` or `Static Release` configurations
        2. you'll have to manually modify the `XercesLib` project options so that
           the `C/C++ > Code Generation > Runtime Library` are of DLL type in both release and debug modes (i.e. `/MD`
           in release or `/MDd` in debug)
        3. for 64 bits version be sure to select the right platform (x64 instead of Win32). If you use Visual Studio
           Express 2010, be sure also that the `toolset` (in the project properties) is set to something
           like `Windows7.1SDK`
    - only the XercesLib project neet to be compiled
    - eventually, CMake will look for the resulting files in `/include` (instead of `/src`) and `/lib` (without the
      Release or Debug subfolders). By default the visual project will put them in `/Build/WinXX/VCXX/StaticXXX`.
      Therefore you should create a custom folder with the right organization and copy the files there.

2. [LibE57](https://github.com/asmaloney/libE57Format) (*last tested version: 2.0.1 on Windows*)
    - Checkout the submodule in `plugins/core/IO/qE57IO/extern/libE57Format` or download and extract the
      latest [libE57Format](https://github.com/asmaloney/libE57Format) release

### qPCL

qPCL relies on the PCL library. Follow the instructions from their website [PCL](http://pointclouds.org/).

### qFBXIO

If you want to compile CloudCompare (and ccViewer) with FBX files support, you'll need: The
official [Autodesk's FBX SDK](http://usa.autodesk.com/adsk/servlet/pc/item?siteID=123112&id=10775847) (last tested
version: 2015.1 on Windows)

Then, the CloudCompare CMake project will request that you set the 3 following variables:

1. `FBX_SDK_INCLUDE_DIR`: FBX SDK include directory (pretty straightforward ;)
2. `FBX_SDK_LIBRARY_FILE`: main FBX SDK library (e.g. `libfbxsdk-md.lib`)
3. `FBX_SDK_LIBRARY_FILE_DEBUG`: main FBX SDK library for debug mode (if any)

### qLASIO

If you want to compile CloudCompare (and ccViewer) with LAS/LAZ files support,
you'll need [LASZip](https://laszip.org/).

### qPDALIO [deprecated]

If you want to compile the deprecated PDAL I/O plugin to load LAS/LAZ files,
you'll need [PDAL](https://pdal.io/).

### qLASFWIO [deprecated]

The deprecated LAS FWF I/O plugin can also be compiled on Windows with
[LASlib](https://github.com/CloudCompare/LAStools).

You'll have to set the following variables:
-   `LASLIB_INCLUDE_DIR`: LAStools/LASlib/inc
-   `LASZIP_INCLUDE_DIR`: LAStools/LASzip/src
-   `LASLIB_RELEASE_LIBRARY` or `LASLIB_DEBUG_LIBRARY`, depending on build type: the compiled library, e.g. `LAStools/LASlib/VC14/lib/x64/LASlibVC14.lib`

### GDAL support

If you want to compile CloudCompare (and ccViewer) with GDAL (raster) files support, you'll need a compiled version of
the [GDAL library](http://www.gdal.org/) (last tested version: 2.2.1 on Windows, 2.0.2 on Mac OS X)

Then, the CloudCompare CMake project will request that you set the following variables:
1. `GDAL_INCLUDE_DIR`: GDAL include directory (pretty straightforward ;)
2. `GDAL_LIBRARY`: the static library (e.g. `gdal_i.lib`)
3. `GDAL_BIN_DIR`: on Windows, the folder with DLL files

### qCork + MPIR support

If you want to compile the qCork plugin (**on Windows only for now**), you'll need:

1. [MPIR 2.6.0](http://www.mpir.org/)
2. the forked version of the Cork library for
   CC: [<https://github.com/cloudcompare/cork>](https://github.com/cloudcompare/cork)
    - on Windows see the Visual project shipped with this fork and corresponding to your version (if any ;)
    - for VS2013 just edit the `mpir` property sheet (in the Properties manager) and update the MPIR macro (in
      the `User macros` tab)

Then, the CloudCompare CMake project will request that you set the following variables:

1. `CORK_INCLUDE_DIR` and `MPIR_INCLUDE_DIR`: both libraries include directories (pretty straightforward ;)
2. `CORK_RELEASE_LIBRARY_FILE` and `MPIR_RELEASE_LIBRARY_FILE`: both main library files
3. and optionally `CORK_DEBUG_LIBRARY_FILE` and `MPIR_DEBUG_LIBRARY_FILE`: both main library files (for debug mode)

4. Make sure you have a C++11 compliant compiler (gcc 4.7+ / clang / Visual 2013 and newer)

*To compile the project with older versions of Qt (from 4.8 to 5.4) or with a non C++11 compliant compiler, you'll have to stick with the https://github.com/cloudcompare/CloudCompare/releases/tag/v2.6.3.1 version*


5.  Last but not least, the `CMAKE` group contains a `CMAKE_INSTALL_PREFIX` variable which is where CloudCompare and ccViewer will be installed (when you compile the `INSTALL` project)
  - On Linux, default install dir is `/usr/local` (be sure to have administrative rights if you want to install CloudCompare there: once configured, you can call `# make install` from the sources directory)
  - On Windows 7/8/10 CMake doesn't have the rights to 'install' files in the `Program Files` folder (even though it's CMake's default installation destination!)


# Other things

## linux

With versions of CMake prior to 3.13, the compiled application might not work for all users (due to some libraries being linked from the /usr/local directory). See https://github.com/CloudCompare/CloudCompare/issues/1680.

## macOs

If you are compiling and running locally, add `-DCC_MAC_DEV_PATHS` to the `CMAKE_CXX_FLAGS` in the `CMAKE` group. This
will look for the plugins in your build directory rather than the application bundle. If you need the shaders as well,
you will have to create a `shaders` folder in the build directory and copy the shaders you need into it.

For convenience, we provide a 100% self contained build system based on `pixi`. Simply install `pixi` from the [official website](www.pixi.sh) and in the root directory of CC code repository simply launch `pixi run build` and then `pixi run CloudCompare`.
You can also create a portable (relocatable) `.app` by using `pixi run bundle`. You could find the bundle in `.build/install/CloudCompare` directory.

## Working with Visual Studio on Windows

As all the files (executables, plugins and other DLLs) must be copied in the `CMAKE_INSTALL_PREFIX` directory prior to execution,
the standard project launch/debug mechanism is broken. Therefore, by default you won't be able to 'run' the CloudCompare or ccViewer
projects as is (with F5 or Ctrl + F5 for instance).

See [this post](http://www.danielgm.net/cc/forum/viewtopic.php?t=992) on the CloudCompare forum to setup Visual Studio correctly.

## Debugging plugins

If you want to use or debug plugins in DEBUG mode while using a single configuration compiler/IDE (gcc, etc.) the you'll have to comment the automatic definition of the `QT_NO_DEBUG` macro in '/plugins/CMakePluginTpl.cmake' (see http://www.cloudcompare.org/forum/viewtopic.php?t=2070).

If you wish to specify a custom directory to search for plugins, shaders, or translations, you may set the `CC_PLUGIN_PATH`, `CC_SHADER_PATH`, and `CC_TRANSLATION_PATH` environment variables. The paths may be relative to the current working directory or absolute.
