//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#pragma once

#include "FileIOFilter.h"

class LasIOFilter : public FileIOFilter
{
  public:
    LasIOFilter();

    // Inherited from FileIOFilter
    CC_FILE_ERROR
    loadFile(const QString &fileName, ccHObject &container, LoadParameters &parameters) override;

    bool canSave(CC_CLASS_ENUM type, bool &multiple, bool &exclusive) const override;
    CC_FILE_ERROR
    saveToFile(ccHObject *entity, const QString &filename, const SaveParameters &parameters) override;
};
