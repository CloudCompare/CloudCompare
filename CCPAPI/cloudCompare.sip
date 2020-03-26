// sip implementation of cloudCompare

%DefaultDocstringSignature(name="prepended")

%Import QtGui/QtGuimod.sip

%Module(name=cloudCompare)

%If (Qt_5_9_1 -)
%Include ccPApi.sip
%Include ccPointCloud.sip
%End


//%Docstring
//cloudCompare is the Python module interfacing cloudCompare library
//actual access to cloudCompare objects is done via ccPApi.
//For instance:
//from PyQt5.QtWidgets import QApplication
//app = QApplication(sys.argv)
//import cloudCompare as cc
//capi = cc.ccPApi()
//cloud = capi.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6446_MNT_LAMB93_IGN69.xyz")
//%End

