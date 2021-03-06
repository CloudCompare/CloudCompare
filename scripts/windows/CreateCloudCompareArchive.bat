@echo off
set SRC_FOLDER=CloudCompare
set DEST_FOLDER=CloudCompare_v2.12.alpha_bin_x64
set SEVENZIP_FOLDER=C:\Program Files\7-Zip

rmdir /q /s %DEST_FOLDER%
xcopy /i /e /y /q %SRC_FOLDER% %DEST_FOLDER%
del %DEST_FOLDER%\*.manifest
del %DEST_FOLDER%\plugins\QFARO_IO_PLUGIN*.dll
del %DEST_FOLDER%.7z
"%SEVENZIP_FOLDER%\7z" a %DEST_FOLDER%.7z %DEST_FOLDER%