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

from gendata import getSampleCloud, dataDir
import os, sys

from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cc.CCLib.ScalarField.initNumpyApi() # to do once before dealing with numpy
cloud1 = cc.loadPointCloud(getSampleCloud(5.0))
cloud2 = cc.loadPointCloud(getSampleCloud(4.9))
res = cloud1.exportCoordToSF((False, False, True))
res = cloud2.exportCoordToSF((False, False, True))
sf1=cloud1.getScalarField(0)
sf2=cloud2.getScalarField(0)
asf1=sf1.toNpArray()
asf2=sf2.toNpArray()
sf2.fromNpArray(asf1)
res=cc.SavePointCloud(cloud2,os.path.join(dataDir,"res2.xyz"))
