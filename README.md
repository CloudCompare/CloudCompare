CloudCompare
============

Homepage: http://www.cloudcompare.org/

[![GitHub release](https://img.shields.io/github/release/cloudcompare/trunk.svg)](https://github.com/cloudcompare/trunk/releases)
[![Build Status](https://travis-ci.org/CloudCompare/CloudCompare.svg)](https://travis-ci.org/CloudCompare/CloudCompare) 

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

If you want to help us in another way, you can make donations via [pledgie](http://pledgie.com/campaigns/19052)  
Thanks!

<a href='https://pledgie.com/campaigns/19052'>
    <img alt='Click here to lend your support to: CloudCompare and make a donation at pledgie.com !' src='https://pledgie.com/campaigns/19052.png?skin_name=chrome' border='0'>
</a>
