set QT_DIR=E:\Qt\Qt5.9.9\5.9.9\msvc2017_64\bin

set SRC_DIRS=common qCC plugins/core libs/CCFbo libs/qCC_db libs/qCC_glWindow libs/qCC_io

echo "Updating translation files"

%call lupdate
cd ..\..
%QT_DIR%\lupdate.exe %SRC_DIRS% -no-obsolete -ts qCC\translations\CloudCompare_es_AR.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% -no-obsolete -ts qCC\translations\CloudCompare_fr.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% -no-obsolete -ts qCC\translations\CloudCompare_ja.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% -no-obsolete -ts qCC\translations\CloudCompare_pt.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% -no-obsolete -ts qCC\translations\CloudCompare_ru.ts
cd scripts\windows
