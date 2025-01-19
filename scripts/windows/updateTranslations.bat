set QT_DIR=E:\Qt\5.15.2\msvc2019_64\bin

set SRC_DIRS=qCC plugins/core libs/CCFbo libs/qCC_db libs/qCC_glWindow libs/qCC_io
set OPTIONS=-no-obsolete -extensions h,cpp

echo "Updating translation files"

%call lupdate
cd ..\..
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_chs.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_de.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_es_AR.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_fr.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_ja.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_ko.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_pt.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_ru.ts
%QT_DIR%\lupdate.exe %SRC_DIRS% %OPTIONS% -ts qCC\translations\CloudCompare_zh.ts
cd scripts\windows
