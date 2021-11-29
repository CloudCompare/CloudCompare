@echo off
set SRC_FOLDER=CloudCompare
set DEST_FOLDER=CloudCompare_v2.12.beta_bin_x64
set SEVENZIP_FOLDER=C:\Program Files\7-Zip
set SIGNTOOL_FOLDER="C:\Program Files (x86)\Windows Kits\10\bin\10.0.18362.0\x64"
set CERTIFICATE_SUBJECT="Open Source Developer, Daniel Girardeau-Montaut"

%SIGNTOOL_FOLDER%\signtool.exe Sign /n %CERTIFICATE_SUBJECT% /fd sha256 /t http://timestamp.digicert.com %SRC_FOLDER%/CloudCompare.exe

rmdir /q /s %DEST_FOLDER%
xcopy /i /e /y /q %SRC_FOLDER% %DEST_FOLDER%
del %DEST_FOLDER%\*.manifest
del %DEST_FOLDER%\plugins\QFARO_IO_PLUGIN*.dll
del %DEST_FOLDER%.7z
"%SEVENZIP_FOLDER%\7z" a %DEST_FOLDER%.7z %DEST_FOLDER%