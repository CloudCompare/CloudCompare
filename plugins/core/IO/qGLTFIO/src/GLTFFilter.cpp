//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

//fx-gltf
#include <fx/gltf.h>

GLTFFilter::GLTFFilter(): FileIOFilter(
    {
        "_GLTF Filter",
        DEFAULT_PRIORITY,
        QStringList{ "gltf", "glb" },
        "gltf",
        QStringList{ "GLTF mesh (*.gltf)", "GLTF mesh binary (*.glb)" },
        QStringList{ "GLTF mesh (*.gltf)", "GLTF mesh binary (*.glb)" },
        Import | Export,
    }
) {}

CC_FILE_ERROR GLTFFilter::loadFile(
    const QString& filename, ccHObject& container, LoadParameters& parameters
) {
    fx::gltf::Document doc;
    if (filename.slice(-5, 0) == ".gltf")
        doc = fx::gltf::LoadFromText(filename);
    else if (filename.slice(-5, 0) == ".glb")
        doc = fx::gltf::LoadFromBinary(filename);
    else
        return CC_FERR_BAD_ARGUMENT;
    container.append(doc);
    return CC_FERR_NO_ERROR;
}

bool GLTFFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive)
{
    if (type == CC_TYPES::MESH)
    {
        multiple = true;
        exclusive = true;
        return true;
    }
    return false;
}

CC_FILE_ERROR GLTFFilter::saveToFile(
    ccHObject* entity, const QString& filename, const SaveParameters& parameters
) {
    fx::gltf::Document doc = {entity};
    if (filename.slice(-5, 0) == ".gltf")
        fx::gltf::Save(doc, filename, false);
    else if (filename.slice(-5, 0) == ".glb")
        fx::gltf::Save(doc, filename, true);
    else
        return CC_FERR_BAD_ARGUMENT;
    return CC_FERR_NO_ERROR;
}
