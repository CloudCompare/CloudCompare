# RDB IO Plugin

This plugin adds support to load Riegl DataBase 2 files.

To compile this plugin the RDB library is needed. To get the library go to the
[Riegl Members Area](http://www.riegl.com/members-area/) and create an account.
The account will be validated by an administrator. After validation download
the RDB library for your platform and extract it.


## Build the Plugin
To build and install the plugin enable the plugin with the `PLUGIN_IO_QRDB` flag
and pass the location of the RDB library with `rdb_DIR` (the path to the
`rdb-config.cmake` file located in `interface/cpp`)

### Build on Linux and Unix like
In the CloudCompare root folder execute the following commands to configure,
build and install

```
cmake -H. -B_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PWD}/_install" -DPLUGIN_IO_QRDB=ON -Drdb_DIR=/path/to/rdblib-2.2.1/interface/cpp
cmake --build _build -- -j
cmake --build _build --target install
```

### Build on Windows with MSVC
To build on Windows with Visual Studio 2017 additionally the generator needs to
be specified. The following command assumes installed build tools as well as Qt5
system wide installation

```
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\BuildTools\Common7\Tools\VsDevCmd.bat"
cmake -H. -B_build -G "Visual Studio 15 2017 Win64" -DCMAKE_INSTALL_PREFIX=%cd%/_install -DPLUGIN_IO_QRDB=ON -Drdb_DIR=C:\Users\username\Downloads\rdblib-2.2.1-x86_64-windows\interface\cpp 
cmake --build _build --config Release --target install
```
