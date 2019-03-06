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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef __STOCKER_PARSER_HEADER__
#define __STOCKER_PARSER_HEADER__

#include "ccHObject.h"

#ifdef USE_STOCKER
#include "builderlod3/builderlod3.h"
#include "builderlod2/builderlod2.h"
#include "buildercore/StBuilder.h"
#include "ioctrl/StFileOperator.hpp"
#endif // USE_STOCKER

stocker::Contour3d GetPointsFromCloud(ccHObject* entity);
stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities);
ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type);
void AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col);
void CalcPlaneIntersections(ccHObject::Container entity_planes, double distance);
void CalcPlaneBoundary(ccHObject* planeObj);
void CalcPlaneOutlines(ccHObject* planeObj);
void PlaneFrameOptimization(ccHObject* planeObj);
#endif
