// sip implementation of cloudCompare

%Module cloudCompare

%Import QtGui/QtGuimod.sip

%If (Qt_5_9_1 -)

%Include ccPApi.sip
%Include ccPointCloud.sip

%End