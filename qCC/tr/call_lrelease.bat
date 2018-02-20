set QT_DIR=C:\Qt\Qt5.5.0\5.5\msvc2013_64\bin

%call lrelease
%QT_DIR%\lrelease.exe translation_french.ts
%QT_DIR%\lrelease.exe translation_chinese.ts
%QT_DIR%\lrelease.exe translation_russian.ts
