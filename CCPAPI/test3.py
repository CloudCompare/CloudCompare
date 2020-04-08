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

import sys
sys.path.append('/home/paul/projets/CloudCompare/install/lib/cloudcompare')
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cc.CCLib.ScalarField.initNumpyApi() # to do once before dealing with numpy
cloud = cc.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6446_MNT_LAMB93_IGN69.xyz")
namecloud = cloud.getName()
print(namecloud)
g = cloud.computeGravityCenter()
print(g)
print("has scalar fields: %s"%cloud.hasScalarFields() )
sf = cloud.getScalarField(0)
print(sf is None)
cloud.getScalarFieldName(0)
a=cloud.getScalarFieldName(0)
print(a is None)
a=cloud.getScalarFieldName(-1)
print(a is None)
a=cloud.getScalarFieldName(25)
print(a is None)
res = cloud.exportCoordToSF((False, True, True))
print("has scalar fields: %s"%cloud.hasScalarFields() )
n = cloud.getNumberOfScalarFields()
print("number of saclar fields: %s"%n)
cloud.setCurrentInScalarField(0)
cloud.setCurrentOutScalarField(1)
sfi=cloud.getCurrentInScalarField()
print(sfi)
sfo=cloud.getCurrentOutScalarField()
print(sfo)
res=cc.computeCurvature(cc.GAUSSIAN_CURV, 1.88, [cloud])
nsf=cloud.getNumberOfScalarFields()
sfc=cloud.getScalarField(nsf-1)
print(sfc.getName())
meanvar = sfc.computeMeanAndVariance()
print(meanvar)
sfc.computeMinAndMax()
print("min: %s"%sfc.getMin())
print("max: %s"%sfc.getMax())
cloud.setCurrentOutScalarField(nsf-1)
fcloud=cc.filterBySFValue(0.01, sfc.getMax(), cloud)
print(fcloud.size())
res=cc.SavePointCloud(fcloud, "/home/paul/projets/CloudCompare/data/res.xyz")















