#pragma once

#include <QString>
#include "ccStdPluginInterface.h"

class ccMainAppInterface;
#ifdef WIN32
#define OS_FILE_SEP '\\'
#else
#define OS_FILE_SEP '/'
#endif
namespace qE57ImageExtractorAction
{

    void extractAndSaveImages( ccMainAppInterface *appInterface, const ccHObject::Container &selectedEntities);

    int saveImages(const QString& directory, const ccHObject::Container& selectedEntities);
}
