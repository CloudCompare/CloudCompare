set QT_DIR="<path to Qt installation bin folder>"

echo "Updating translation files"

%call lupdate
%QT_DIR%\lupdate.exe ..\..\qCC -no-obsolete -ts ..\..\qCC\translations\CloudCompare_fr.ts
%QT_DIR%\lupdate.exe ..\..\qCC -no-obsolete -ts ..\..\qCC\translations\CloudCompare_pt.ts
%QT_DIR%\lupdate.exe ..\..\qCC -no-obsolete -ts ..\..\qCC\translations\CloudCompare_ru.ts
