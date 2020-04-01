CloudCompare
============

Homepage: http://www.cloudcompare.org/

[![GitHub release](https://img.shields.io/github/release/cloudcompare/trunk.svg)](https://github.com/cloudcompare/trunk/releases)

- [![Build Status](https://travis-ci.org/CloudCompare/CloudCompare.svg?branch=master)](https://travis-ci.org/CloudCompare/CloudCompare) Linux
- [![Build](https://github.com/CloudCompare/CloudCompare/workflows/Build/badge.svg?branch=master)](https://github.com/CloudCompare/CloudCompare/actions?query=workflow%3ABuild+branch%3Amaster) Windows & MacOS
- [![Releases](https://coderelease.io/badge/CloudCompare/CloudCompare)](https://coderelease.io/github/repository/CloudCompare/CloudCompare)

Introduction
------------

CloudCompare is a 3D point cloud (and triangular mesh) processing software.
It was originally designed to perform comparison between two 3D points clouds
(such as the ones obtained with a laser scanner) or between a point cloud and a
triangular mesh. It relies on an octree structure that is highly optimized for
this particular use-case. It was also meant to deal with huge point
clouds (typically more than 10 millions points, and up to 120 millions with 2 Gb
of memory).

More on CloudCompare [here](http://en.wikipedia.org/wiki/CloudCompare)

Compilation
-----------

Supported OS: Windows, Linux, and Mac OS X

Refer to the [BUILD.md file](BUILD.md) for up-to-date information.

Basically, you have to:
- clone this repository
- install mandatory dependencies (OpenGL,  etc.) and optional ones if you really need them
(mainly to support particular file formats, or for some plugins)
- launch CMake (from the trunk root)
- enjoy!


Contributing to CloudCompare
----------------------------

If you want to help us improve CloudCompare or create a new plugin you can start by reading this [guide](CONTRIBUTING.md)

Supporting the project
----------------------

If you want to help us in another way, you can make donations via [donorbox](https://donorbox.org/support-cloudcompare)
Thanks!

<a href='https://donorbox.org/support-cloudcompare' target="_blank"><img src="https://d1iczxrky3cnb2.cloudfront.net/button-medium-blue.png"></a>
