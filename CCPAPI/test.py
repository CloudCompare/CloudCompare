#!/usr/bin/env python3

import sys
sys.path.append('/home/paul/projets/CloudCompare/install/lib/cloudcompare')
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
capi = cc.ccPApi()
cloud = capi.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6446_MNT_LAMB93_IGN69.xyz")
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

# addressBook = AddressBook()
# addressBook.show()
# sys.exit(app.exec_())
