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

from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cc.CCLib.ScalarField.initNumpyApi() # to do once before dealing with numpy

cloud = cc.loadPointCloud(getSampleCloud(5.0))
res = cloud.exportCoordToSF((False, True, True))
n = cloud.getNumberOfScalarFields()
cloud.setCurrentInScalarField(0)
cloud.setCurrentOutScalarField(1)

sfi=cloud.getCurrentInScalarField()
if sfi.getName() != cloud.getScalarFieldName(0):
    raise RuntimeError

sfo=cloud.getCurrentOutScalarField()
if sfo.getName() != cloud.getScalarFieldName(1):
    raise RuntimeError

res=cc.computeCurvature(cc.GAUSSIAN_CURV, 0.05, [cloud])
nsf=cloud.getNumberOfScalarFields()
sfc=cloud.getScalarField(nsf-1)
if sfc.getName() != "Gaussian curvature (0.05)":
    raise RuntimeError

cloud.setCurrentOutScalarField(nsf-1)
fcloud=cc.filterBySFValue(0.01, sfc.getMax(), cloud)
filteredSize = fcloud.size()
if not math.isclose(filteredSize, 780611, rel_tol=1e-03):
    raise RuntimeError

res=cc.SavePointCloud(fcloud, os.path.join(dataDir,"res3.xyz"))
