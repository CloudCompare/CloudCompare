#!/bin/bash

dir=${0%/*}
if [ -d "$dir" ]; then
  cd "$dir/../.." || exit
fi

QT_DIR="<path to Qt installation>"

LUPDATE="${QT_DIR}/bin/lupdate"
SRC_DIRS="common qCC plugins/core libs/CCFbo libs/qCC_db libs/qCC_glWindow libs/qCC_io"
TRANSLATION_DIR="qCC/translations"

echo "Updating translation files"

"${LUPDATE}" ${SRC_DIRS} -no-obsolete -ts "${TRANSLATION_DIR}"/CloudCompare_fr.ts
"${LUPDATE}" ${SRC_DIRS} -no-obsolete -ts "${TRANSLATION_DIR}"/CloudCompare_ja.ts
"${LUPDATE}" ${SRC_DIRS} -no-obsolete -ts "${TRANSLATION_DIR}"/CloudCompare_pt.ts
"${LUPDATE}" ${SRC_DIRS} -no-obsolete -ts "${TRANSLATION_DIR}"/CloudCompare_ru.ts
