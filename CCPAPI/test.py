#!/usr/bin/env python3

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
cloud.scale(1.0, 1.0, 2.0, (0., 0., 0.) )
g = cloud.computeGravityCenter()
print(g)
cloud.scale(1, 1, 0.5 )
g = cloud.computeGravityCenter()
print(g)
cloud.translate((-845499.5, -6445500.5, -192.53309631347656))
g = cloud.computeGravityCenter()
print(g)
res = cloud.exportCoordToSF((False, True, True))
print("has scalar fields: %s"%cloud.hasScalarFields() )
n = cloud.getNumberOfScalarFields()
print("number of saclar fields: %s"%n)
csfname0 = cloud.getScalarFieldName(0)
print("cloud scalar field name: %s"%csfname0)

sf = cloud.getScalarField(0)
print("scalar field: %s"%sf)    
sfname = sf.getName()
print("scalar field name: %s"%sfname)
#sf.initNumpyApi()
asf = sf.toNpArray()

sf2 = cloud.getScalarField(1)
print("scalar field: %s"%sf2)    
sf2name = sf2.getName()
print("scalar field name: %s"%sf2name)
asf2 = sf2.toNpArray()

res=cc.SavePointCloud(cloud, "/home/paul/projets/CloudCompare/data/res.xyz")

















