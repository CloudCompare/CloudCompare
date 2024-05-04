# LAS-IO

---

Yet another CloudCompare plugin to read and write LAS/LAZ.

Features:

- Supports all formats including waveforms and extra bytes
- Allows to choose in which format the file should be saved.

# Installation

Install LASzip using one of: you distro's package manager, conda, vcpkg, homebrew or compile it yourself.

Fedora: `laszip-devel`

Ubuntu: `liblaszip-dev`

Clone in `plugins/private` folder of CloudComapare's sources.

Add `-DPLUGIN_IO_QLAS=ON` to cmake.

Adding `-DCMAKE_INCLUDE_PATH=` and `-DCMAKE_LIBRARY_PATH=` maybe be needed to help find laszip libraries.

