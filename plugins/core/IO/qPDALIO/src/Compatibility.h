#ifndef CLOUDCOMPAREPROJECTS_COMPATIBILITY_H
#define CLOUDCOMPAREPROJECTS_COMPATIBILITY_H

#include <pdal/io/LasHeader.hpp>
#include <pdal/pdal_features.hpp>

#include <CCGeom.h>


inline bool Has14PointFormat(const pdal::LasHeader &header) {
#if PDAL_VERSION_MINOR <= 2
    return header.has14Format();
#else
  return header.has14PointFormat();
#endif
}

CCVector3d ScalesFromHeader(const pdal::LasHeader &header) {
  return {header.scaleX(), header.scaleY(), header.scaleZ()};
}

CCVector3d OffsetsFromHeader(const pdal::LasHeader &header) {
  return {header.offsetX(), header.offsetY(), header.offsetZ()};
}

uint8_t VersionMinorFromHeader(const pdal::LasHeader& header) {
  return header.versionMinor();
}

#endif // CLOUDCOMPAREPROJECTS_COMPATIBILITY_H
