set QT_DIR=C:\Qt\Qt5.5.0\5.5\msvc2013_64\bin

echo %QT_DIR%
%call lupdate
%QT_DIR%\lupdate.exe .. -ts translation_french.ts
%QT_DIR%\lupdate.exe .. -ts translation_chinese.ts
