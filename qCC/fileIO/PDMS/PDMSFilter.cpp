//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "PDMSFilter.h"
#include "PdmsParser.h"

//qCC_db
#include <ccLog.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccCylinder.h>
#include <ccTorus.h>
#include <ccBox.h>
#include <ccCone.h>
#include <ccDish.h>
#include <ccExtru.h>

#include <assert.h>

using namespace CCLib;

CC_FILE_ERROR PDMSFilter::saveToFile(ccHObject* entity, const char* filename)
{
	ccLog::Print("Function is not implemented yet !");
	return CC_FERR_NO_SAVE;
}

CC_FILE_ERROR PDMSFilter::loadFile(const char* filename, ccHObject& container, bool alwaysDisplayLoadDialog/*=true*/, bool* coordinatesShiftEnabled/*=0*/, CCVector3d* coordinatesShift/*=0*/)
{
	PdmsParser parser;
	PdmsFileSession session(filename);

	parser.linkWithSession(&session);
	if(parser.parseSessionContent())
	{
		PdmsTools::PdmsObjects::GenericItem* pdmsmodel = parser.getLoadedObject(true);
		assert(pdmsmodel);

		std::vector< std::pair<PdmsTools::PdmsObjects::GenericItem*,ccHObject*> > treeSync;
		std::pair<PdmsTools::PdmsObjects::GenericItem*,ccHObject*> currentPair;
		currentPair.first = pdmsmodel;
		currentPair.second = &container;
		treeSync.push_back(currentPair);

		while (!treeSync.empty())
		{
			currentPair = treeSync.back();
			treeSync.pop_back();

			if (currentPair.first->isGroupElement())
			{
				PdmsTools::PdmsObjects::GroupElement* group = static_cast<PdmsTools::PdmsObjects::GroupElement*>(currentPair.first);

				//primitives
				for (std::list<PdmsTools::PdmsObjects::DesignElement*>::const_iterator it = group->elements.begin(); it!=group->elements.end();++it)
				{
					std::pair<PdmsTools::PdmsObjects::GenericItem*,ccHObject*> newPair;
					newPair.first = *it;
					newPair.second = currentPair.second;
					treeSync.push_back(newPair);
				}

				//sub-groups
				for (std::list<PdmsTools::PdmsObjects::GroupElement*>::const_iterator it2 = group->subhierarchy.begin(); it2!=group->subhierarchy.end();++it2)
				{
					std::pair<PdmsTools::PdmsObjects::GenericItem*,ccHObject*> newPair;
					newPair.first = *it2;
					newPair.second = new ccHObject((*it2)->name);
					currentPair.second->addChild(newPair.second);
					treeSync.push_back(newPair);
				}
			}
			else
			{
				//Convert PDMS GenericItem to the corresponding ccHObject
				ccMesh* primitive = 0;
				QString unsupportedPrimitiveStr;
				switch(currentPair.first->getType())
				{
				case PDMS_SCYLINDER:
					{
						PdmsTools::PdmsObjects::SCylinder* pdmsCyl = static_cast<PdmsTools::PdmsObjects::SCylinder*>(currentPair.first);
						primitive = new ccCylinder(pdmsCyl->diameter/2,pdmsCyl->height,0,pdmsCyl->name);
					}
					break;
				case PDMS_CTORUS:
					{
						PdmsTools::PdmsObjects::CTorus* pdmsCTor = static_cast<PdmsTools::PdmsObjects::CTorus*>(currentPair.first);
						primitive = new ccTorus(pdmsCTor->inside_radius,pdmsCTor->outside_radius,pdmsCTor->angle/**M_PI/180.0*/,false,0,0,pdmsCTor->name);
					}
					break;
				case PDMS_RTORUS:
					{
						PdmsTools::PdmsObjects::RTorus* pdmsRTor = static_cast<PdmsTools::PdmsObjects::RTorus*>(currentPair.first);
						primitive = new ccTorus(pdmsRTor->inside_radius,pdmsRTor->outside_radius,pdmsRTor->angle/**M_PI/180.0*/,false,pdmsRTor->height,0,pdmsRTor->name);
					}
				case PDMS_DISH:
					{
						PdmsTools::PdmsObjects::Dish* pdmsDish = static_cast<PdmsTools::PdmsObjects::Dish*>(currentPair.first);
						primitive = new ccDish(pdmsDish->diameter/2,pdmsDish->height,pdmsDish->radius,0,pdmsDish->name);
					}
					break;
				case PDMS_CONE:
					{
						PdmsTools::PdmsObjects::Cone* pdmsCone = static_cast<PdmsTools::PdmsObjects::Cone*>(currentPair.first);
						primitive = new ccCone(pdmsCone->dbottom/2,pdmsCone->dtop/2,pdmsCone->height,0,0,0,pdmsCone->name);
					}
					break;
				case PDMS_PYRAMID:
					unsupportedPrimitiveStr = "Pyramid";
					break;
				case PDMS_SNOUT:
					{
						PdmsTools::PdmsObjects::Snout* pdmsSnout = static_cast<PdmsTools::PdmsObjects::Snout*>(currentPair.first);
						primitive = new ccCone(pdmsSnout->dbottom/2,pdmsSnout->dtop/2,pdmsSnout->height,pdmsSnout->xoff,pdmsSnout->yoff,0,pdmsSnout->name);
					}
					break;
				case PDMS_BOX:
					{
						PdmsTools::PdmsObjects::Box* pdmsBox = static_cast<PdmsTools::PdmsObjects::Box*>(currentPair.first);
						primitive = new ccBox(pdmsBox->lengths,0,pdmsBox->name);
					}
					break;
				case PDMS_NBOX:
					unsupportedPrimitiveStr = "NBox";
					break;
				case PDMS_EXTRU:
					{
						PdmsTools::PdmsObjects::Extrusion* pdmsExtru = static_cast<PdmsTools::PdmsObjects::Extrusion*>(currentPair.first);
						size_t count = pdmsExtru->loop->loop.size();
						if (count)
						{
							std::vector<CCVector2> profile;
							profile.reserve(count);
							for (std::list<PdmsTools::PdmsObjects::Vertex*>::const_iterator it=pdmsExtru->loop->loop.begin();it!=pdmsExtru->loop->loop.end();++it)
								profile.push_back((*it)->v);

							primitive = new ccExtru(profile,pdmsExtru->height,0,pdmsExtru->name);
						}
					}
					break;
				case PDMS_NEXTRU:
					unsupportedPrimitiveStr = "NExtru";
					break;
				case PDMS_LOOP:
					unsupportedPrimitiveStr = "Loop";
					break;
				case PDMS_VERTEX:
					unsupportedPrimitiveStr = "Vertex";
					break;
				}

				if (primitive)
				{
					//transformation
					ccGLMatrix trans;
					assert(currentPair.first->isCoordinateSystemUpToDate);
					trans.setTranslation(currentPair.first->position);
					for (unsigned c=0;c<3;++c)
						for (unsigned l=0;l<3;++l)
							trans.getColumn(c)[l] = static_cast<float>(currentPair.first->orientation[c].u[l]);

					primitive->setGLTransformation(trans);
					primitive->setVisible(true);
					currentPair.second->addChild(primitive);
				}
				else
				{
					ccLog::Warning(QString("[PDMSFilter] Primitive '%1' not supported yet!").arg(unsupportedPrimitiveStr));
				}
			}

		}
	}

	container.applyGLTransformation_recursive();

	return CC_FERR_NO_ERROR;
}
