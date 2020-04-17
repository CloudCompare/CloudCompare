#!/usr/bin/env python3

##########################################################################
#                                                                        #
#                               CCPAPI                                   #
#                                                                        #
#  This program is free software; you can redistribute it and/or modify  #
#  it under the terms of the GNU Library General Public License as       #
#  published by the Free Software Foundation; version 2 or later of the  #
#  License.                                                              #
#                                                                        #
#  This program is distributed in the hope that it will be useful,       #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
#  GNU General Public License for more details.                          #
#                                                                        #
#          Copyright 2020 Paul RASCLE www.openfields.fr                  #
#                                                                        #
##########################################################################

import os
import sys
import math
from gendata import getSampleCloud, dataDir, isCoordEqual

from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cc.CCLib.ScalarField.initNumpyApi() # to do once before dealing with numpy

cloud = cc.loadPointCloud(getSampleCloud(2.0))
namecloud = cloud.getName()
print("cloud name: %s"%namecloud)
if namecloud != "dataSample_2 - Cloud":
    raise RuntimeError

npts = cloud.size()
print("cloud.size %s"%npts)
if npts != 1000000:
    raise RuntimeError

res = cloud.hasScalarFields()
print("hasScalarField: %s"%res)
if res:
    raise RuntimeError

sf = cloud.getScalarField(0)
if sf is not None:
    raise RuntimeError

nsf = cloud.getNumberOfScalarFields()
if nsf != 0:
    raise RuntimeError

sfname = cloud.getScalarFieldName(0)
if sfname is not None:
    raise RuntimeError

sfname = cloud.getScalarFieldName(-1)
if sfname is not None:
    raise RuntimeError

sfname = cloud.getScalarFieldName(25)
if sfname is not None:
    raise RuntimeError

g = cloud.computeGravityCenter()
print("gravityCenter: (%14.7e, %14.7e, %14.7e)"%(g[0], g[1], g[2]))
if not isCoordEqual(g, (-4.9999999e-03, -4.9999999e-03,  9.6193114e-03)):
    raise RuntimeError

cloud.scale(1.0, 1.0, 2.0, (0., 0., 0.) )
g = cloud.computeGravityCenter()
print("gravityCenter: (%14.7e, %14.7e, %14.7e)"%(g[0], g[1], g[2]))
if not isCoordEqual(g, (-4.9999999e-03, -4.9999999e-03,  1.9238623e-02)):
    raise RuntimeError

cloud.scale(1, 1, 0.5 )
g = cloud.computeGravityCenter()
print("gravityCenter: (%14.7e, %14.7e, %14.7e)"%(g[0], g[1], g[2]))
if not isCoordEqual(g, (-4.9999999e-03, -4.9999999e-03,  9.6193114e-03)):
    raise RuntimeError

cloud.translate((-1, -2, -3))
g = cloud.computeGravityCenter()
print("gravityCenter: (%14.7e, %14.7e, %14.7e)"%(g[0], g[1], g[2]))
if not isCoordEqual(g, (-1.0050000e+00, -2.0050001e+00, -2.9903808e+00)):
    raise RuntimeError

cloud.translate((1, 2, 3))
g = cloud.computeGravityCenter()
print("gravityCenter: (%14.7e, %14.7e, %14.7e)"%(g[0], g[1], g[2]))
if not isCoordEqual(g, (-4.9999999e-03, -4.9999999e-03,  9.6193114e-03)):
    raise RuntimeError

res=cc.SavePointCloud(cloud, os.path.join(dataDir,"res1.bin"))
if res:
    raise RuntimeError
    
cloud = cc.loadPointCloud(os.path.join(dataDir,"res1.bin"))
namecloud = cloud.getName()
print("cloud name: %s"%namecloud)
if namecloud != "dataSample_2 - Cloud":
    raise RuntimeError

npts = cloud.size()
print("cloud.size %s"%npts)
if npts != 1000000:
    raise RuntimeError

g = cloud.computeGravityCenter()
print("gravityCenter: (%14.7e, %14.7e, %14.7e)"%(g[0], g[1], g[2]))
if not isCoordEqual(g, (-4.9999999e-03, -4.9999999e-03,  9.6193114e-03)):
    raise RuntimeError
