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
from gendata import getSampleCloud, dataDir
import numpy as np

from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cc.CCLib.ScalarField.initNumpyApi() # to do once before dealing with numpy

cloud1 = cc.loadPointCloud(getSampleCloud(2.0))

ok = cloud1.exportCoordToSF((False, False, True))
if not ok:
    raise RuntimeError

res = cloud1.hasScalarFields()
print("hasScalarField: %s"%res)
if not res:
    raise RuntimeError

n = cloud1.getNumberOfScalarFields()
print("number of saclar fields: %s"%n)
if n != 1:
    raise RuntimeError

csfname0 = cloud1.getScalarFieldName(0)
print("cloud scalar field name: %s"%csfname0)

sf1=cloud1.getScalarField(0)
sfname = sf1.getName()
print("scalar field name: %s"%sfname)
if sfname != "Coord. Z":
    raise RuntimeError

mean, var = sf1.computeMeanAndVariance()
print("mean: %14.7e, var: %14.7e"%(mean, var))
if not math.isclose(mean, 9.6193114e-03, rel_tol=1e-06):
    raise RuntimeError
if not math.isclose(var, 1.4433296e-01, rel_tol=1e-06):
    raise RuntimeError

sf1.computeMinAndMax()
sfmin = sf1.getMin()
sfmax = sf1.getMax()
print("min: %14.7e"%sfmin)
print("max: %14.7e"%sfmax)
if not math.isclose(sfmin, -1.0861681e+00, rel_tol=1e-06):
    raise RuntimeError
if not math.isclose(sfmax, 5.0000000e+00, rel_tol=1e-06):
    raise RuntimeError

asf1=sf1.toNpArray()
print(asf1.size)
if asf1.size != 1000000:
    raise RuntimeError

cloud2 = cc.loadPointCloud(getSampleCloud(1.9))
res = cloud2.exportCoordToSF((False, False, True))
sf2=cloud2.getScalarField(0)
asf2=sf2.toNpArray()

sf2.fromNpArray(asf1)

res=cc.SavePointCloud(cloud2,os.path.join(dataDir,"res2.xyz"))
cloud2 = cc.loadPointCloud(os.path.join(dataDir,"res2.xyz"))
sf2=cloud2.getScalarField(0)
sfname = sf2.getName()
print("scalar field name: %s"%sfname)
if sfname != "Scalar field":
    raise RuntimeError
asf2=sf2.toNpArray()
ok = np.allclose(asf1, asf2, rtol=1.e-6)
if not ok:
    raise RuntimeError
