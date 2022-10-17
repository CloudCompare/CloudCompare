#pragma once

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

#include "LasSaveDialog.h"
#include "LasScalarFieldSaver.h"
#include "LasWaveformSaver.h"

#include <FileIOFilter.h>

// LASzip
#include <laszip/laszip_api.h>

#include <memory>

class LasSaveDialog;
class ccPointCloud;

class LasSaver
{
  public:
    LasSaver(ccPointCloud &cloud, const LasSaveDialog &saveDlg);

    CC_FILE_ERROR open(const QString filePath);

    CC_FILE_ERROR saveNextPoint();

    bool savesWaveforms() const;

    ~LasSaver() noexcept;

  private:
    void initLaszipHeader(const LasSaveDialog &saveDialog);

  private:
    unsigned int m_currentPointIndex{0};
    ccPointCloud &m_cloudToSave;
    laszip_header m_laszipHeader{};
    laszip_POINTER m_laszipWriter{nullptr};
    LasScalarFieldSaver m_fieldsSaver{};
    bool m_shouldSaveRGB{false};
    std::unique_ptr<LasWaveformSaver> m_waveformSaver{nullptr};
    laszip_point *m_laszipPoint{nullptr};
};