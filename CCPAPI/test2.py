# coding: utf-8
import sys
sys.path.append('/home/paul/projets/CloudCompare/install/lib/cloudcompare')
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cc.CCLib.ScalarField.initNumpyApi() # to do once before dealing with numpy
cloud1 = cc.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6446_MNT_LAMB93_IGN69.xyz")
cloud2 = cc.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6447_MNT_LAMB93_IGN69.xyz")
res = cloud1.exportCoordToSF((False, False, True))
res = cloud2.exportCoordToSF((False, False, True))
sf1=cloud1.getScalarField(0)
sf2=cloud2.getScalarField(0)
asf1=sf1.toNpArray()
asf2=sf2.toNpArray()
sf2.fromNpArray(asf1)
res=cc.SavePointCloud(cloud2,"/home/paul/projets/CloudCompare/data/res.xyz")
