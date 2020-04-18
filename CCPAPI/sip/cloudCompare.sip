//##########################################################################
//#                                                                        #
//#                               CCPAPI                                   #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          Copyright 2020 Paul RASCLE www.openfields.fr                  #
//#                                                                        #
//##########################################################################

// sip implementation of cloudCompare

%DefaultDocstringSignature(name="prepended")

%Import QtGui/QtGuimod.sip

%Module(name=cloudCompare)
{
%Docstring
cloudCompare is the Python module interfacing cloudCompare library
Python3 access to cloudCompare objects is done like this:
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
import cloudCompare as cc
cloud = cc.loadPointCloud("/home/paul/projets/CloudCompare/data/altiXYZ/RGEALTI_FXX_0845_6446_MNT_LAMB93_IGN69.xyz")
%End
};

// Not tested with a Qt version below 5.9: %If (Qt_5_9_1 -) means version 5.9.1 and up.
%If (Qt_5_9_1 -)
%Include ccPApi.sip
%Include ccPointCloud.sip
namespace CCLib;
%Include ScalarField.sip
%End
