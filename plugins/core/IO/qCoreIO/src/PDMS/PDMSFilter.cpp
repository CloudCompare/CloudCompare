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

#include "PDMSFilter.h"
#include "PdmsParser.h"

//qCC_db
#include <ccBox.h>
#include <ccCone.h>
#include <ccCylinder.h>
#include <ccDish.h>
#include <ccExtru.h>
#include <ccLog.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccTorus.h>

#include <cassert>

using namespace CCLib;

using PdmsAndCCPair = std::pair<PdmsTools::PdmsObjects::GenericItem*, ccHObject*>;

PDMSFilter::PDMSFilter()
	: FileIOFilter( {
					"_PDMS Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "pdms", "pdmsmac", "mac" },
					"pdms",
					QStringList{ "PDMS primitives (*.pdms *.pdmsmac *.mac)" },
					QStringList(),
					Import
					} )
{
}

CC_FILE_ERROR PDMSFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	Q_UNUSED( parameters );
	
	PdmsParser parser;
	PdmsFileSession session(qPrintable(filename)); //DGM: warning, toStdString doesn't preserve "local" characters

	parser.linkWithSession(&session);
	if (parser.parseSessionContent())
	{
		PdmsTools::PdmsObjects::GenericItem* pdmsmodel = parser.getLoadedObject(true);
		assert(pdmsmodel);

		std::vector< PdmsAndCCPair > treeSync;
		treeSync.emplace_back(pdmsmodel,&container);

		while (!treeSync.empty())
		{
			PdmsAndCCPair currentPair = treeSync.back();
			treeSync.pop_back();

			if (currentPair.first->isGroupElement())
			{
				PdmsTools::PdmsObjects::GroupElement* group = static_cast<PdmsTools::PdmsObjects::GroupElement*>(currentPair.first);

				//primitives
				{
					for (std::list<PdmsTools::PdmsObjects::DesignElement*>::const_iterator it = group->elements.begin(); it != group->elements.end(); ++it)
						treeSync.emplace_back(*it, currentPair.second);
				}

				//sub-groups
				{
					for (std::list<PdmsTools::PdmsObjects::GroupElement*>::const_iterator it = group->subhierarchy.begin(); it != group->subhierarchy.end(); ++it)
					{
						ccHObject* subGroup = new ccHObject((*it)->name);
						currentPair.second->addChild(subGroup);

						treeSync.emplace_back(*it, subGroup);
					}
				}
			}
			else
			{
				//Convert PDMS GenericItem to the corresponding ccHObject
				ccMesh* primitive = nullptr;
				QString unsupportedPrimitiveStr("unknown");
				switch (currentPair.first->getType())
				{
				case PDMS_SCYLINDER:
					{
						PdmsTools::PdmsObjects::SCylinder* pdmsCyl = static_cast<PdmsTools::PdmsObjects::SCylinder*>(currentPair.first);
						primitive = new ccCylinder(pdmsCyl->diameter / 2, pdmsCyl->height, nullptr, pdmsCyl->name);
					}
					break;
				case PDMS_CTORUS:
					{
						PdmsTools::PdmsObjects::CTorus* pdmsCTor = static_cast<PdmsTools::PdmsObjects::CTorus*>(currentPair.first);
						primitive = new ccTorus(pdmsCTor->inside_radius, pdmsCTor->outside_radius, pdmsCTor->angle/**M_PI/180.0*/, false, 0, nullptr, pdmsCTor->name);
					}
					break;
				case PDMS_RTORUS:
					{
						PdmsTools::PdmsObjects::RTorus* pdmsRTor = static_cast<PdmsTools::PdmsObjects::RTorus*>(currentPair.first);
						primitive = new ccTorus(pdmsRTor->inside_radius, pdmsRTor->outside_radius, pdmsRTor->angle/**M_PI/180.0*/, false, pdmsRTor->height, nullptr, pdmsRTor->name);
					}
					break;
				case PDMS_DISH:
					{
						PdmsTools::PdmsObjects::Dish* pdmsDish = static_cast<PdmsTools::PdmsObjects::Dish*>(currentPair.first);
						primitive = new ccDish(pdmsDish->diameter / 2, pdmsDish->height, pdmsDish->radius, nullptr, pdmsDish->name);
					}
					break;
				case PDMS_CONE:
					{
						PdmsTools::PdmsObjects::Cone* pdmsCone = static_cast<PdmsTools::PdmsObjects::Cone*>(currentPair.first);
						primitive = new ccCone(pdmsCone->dbottom / 2, pdmsCone->dtop / 2, pdmsCone->height, 0, 0, nullptr, pdmsCone->name);
					}
					break;
				case PDMS_PYRAMID:
					unsupportedPrimitiveStr = "Pyramid";
					break;
				case PDMS_SNOUT:
					{
						PdmsTools::PdmsObjects::Snout* pdmsSnout = static_cast<PdmsTools::PdmsObjects::Snout*>(currentPair.first);
						primitive = new ccCone(pdmsSnout->dbottom / 2, pdmsSnout->dtop / 2, pdmsSnout->height, pdmsSnout->xoff, pdmsSnout->yoff, nullptr, pdmsSnout->name);
					}
					break;
				case PDMS_BOX:
					{
						PdmsTools::PdmsObjects::Box* pdmsBox = static_cast<PdmsTools::PdmsObjects::Box*>(currentPair.first);
						primitive = new ccBox(pdmsBox->lengths, nullptr, pdmsBox->name);
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
							for (std::list<PdmsTools::PdmsObjects::Vertex*>::const_iterator it = pdmsExtru->loop->loop.begin(); it != pdmsExtru->loop->loop.end(); ++it)
								profile.push_back((*it)->v);

							primitive = new ccExtru(profile, pdmsExtru->height, nullptr, pdmsExtru->name);
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
					
				default:
					break;
				}

				if (primitive)
				{
					//transformation
					ccGLMatrix trans;
					assert(currentPair.first->isCoordinateSystemUpToDate);
					trans.setTranslation(currentPair.first->position);
					for (unsigned c = 0; c < 3; ++c)
						for (unsigned l = 0; l < 3; ++l)
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
	else
	{
		return CC_FERR_MALFORMED_FILE;
	}

	container.applyGLTransformation_recursive();

	return CC_FERR_NO_ERROR;
}
