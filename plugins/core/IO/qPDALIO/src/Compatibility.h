#ifndef CLOUDCOMPAREPROJECTS_COMPATIBILITY_H
#define CLOUDCOMPAREPROJECTS_COMPATIBILITY_H

#include <pdal/io/LasHeader.hpp>
#include <pdal/pdal_features.hpp>

#include <CCGeom.h>

#define CC_PDAL_MASTER 0

#if CC_PDAL_MASTER
namespace pdal {
  using LasHeader = pdal::las::Header;
}
#endif

inline bool Has14PointFormat(const pdal::LasHeader &header) {
#if PDAL_VERSION_MINOR <= 2
    return header.has14Format();
#else
  return header.has14PointFormat();
#endif
}

CCVector3d ScalesFromHeader(const pdal::LasHeader &header) {
#if CC_PDAL_MASTER
  return {header.scale.x, header.scale.y, header.scale.z};
#else
  return {header.scaleX(), header.scaleY(), header.scaleZ()};
#endif
}

CCVector3d OffsetsFromHeader(const pdal::LasHeader &header) {
#if CC_PDAL_MASTER
  return {header.offset.x, header.offset.y, header.offset.z};
#else
  return {header.offsetX(), header.offsetY(), header.offsetZ()};
#endif
}

uint8_t VersionMinorFromHeader(const pdal::LasHeader& header) {
#if CC_PDAL_MASTER
  return header.versionMinor;
#else
  return header.versionMinor();
#endif
}

#endif // CLOUDCOMPAREPROJECTS_COMPATIBILITY_H
