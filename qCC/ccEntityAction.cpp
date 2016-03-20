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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QColorDialog>
#include <QInputDialog>

#include "ScalarField.h"

#include "ccCommon.h"
#include "ccConsole.h"
#include "ccGenericPrimitive.h"
#include "ccHObjectCaster.h"
#include "ccPointCloud.h"
#include "ccMesh.h"
#include "ccScalarField.h"

#include "ccColorScalesManager.h"
#include "ccFacet.h"
#include "ccPolyline.h"

#include "ccColorGradientDlg.h"
#include "ccColorLevelsDlg.h"
#include "ccProgressDialog.h"

#include "ccEntityAction.h"
#include "ccUtils.h"


namespace ccEntityAction
{
	void setColor(ccHObject::Container selectedEntities, bool colorize, QWidget *parent)
	{
		QColor colour = QColorDialog::getColor(Qt::white, parent);
		
		if (!colour.isValid())
			return;
		
		while (!selectedEntities.empty())
		{
			ccHObject* ent = selectedEntities.back();
			selectedEntities.pop_back();
			if (ent->isA(CC_TYPES::HIERARCHY_OBJECT))
			{
				//automatically parse a group's children set
				for (unsigned i=0; i<ent->getChildrenNumber(); ++i)
					selectedEntities.push_back(ent->getChild(i));
			}
			else if (ent->isA(CC_TYPES::POINT_CLOUD) || ent->isA(CC_TYPES::MESH))
			{
				ccPointCloud* cloud = 0;
				if (ent->isA(CC_TYPES::POINT_CLOUD))
				{
					cloud = static_cast<ccPointCloud*>(ent);
				}
				else
				{
					ccMesh* mesh = static_cast<ccMesh*>(ent);
					ccGenericPointCloud* vertices = mesh->getAssociatedCloud();
					if (	!vertices
							||	!vertices->isA(CC_TYPES::POINT_CLOUD)
							||	(vertices->isLocked() && !mesh->isAncestorOf(vertices)) )
					{
						ccLog::Warning(QString("[SetColor] Can't set color for mesh '%1' (vertices are not accessible)").arg(ent->getName()));
						continue;
					}
					
					cloud = static_cast<ccPointCloud*>(vertices);
				}
				
				if (colorize)
				{
					cloud->colorize(static_cast<float>(colour.redF()),
										 static_cast<float>(colour.greenF()),
										 static_cast<float>(colour.blueF()) );
				}
				else
				{
					cloud->setRGBColor(	static_cast<ColorCompType>(colour.red()),
												static_cast<ColorCompType>(colour.green()),
												static_cast<ColorCompType>(colour.blue()) );
				}
				cloud->showColors(true);
				cloud->prepareDisplayForRefresh();
				
				if (ent != cloud)
					ent->showColors(true);
				else if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
					cloud->getParent()->showColors(true);
			}
			else if (ent->isKindOf(CC_TYPES::PRIMITIVE))
			{
				ccGenericPrimitive* prim = ccHObjectCaster::ToPrimitive(ent);
				ccColor::Rgb col(	static_cast<ColorCompType>(colour.red()),
										static_cast<ColorCompType>(colour.green()),
										static_cast<ColorCompType>(colour.blue()) );
				prim->setColor(col);
				ent->showColors(true);
				ent->prepareDisplayForRefresh();
			}
			else if (ent->isA(CC_TYPES::POLY_LINE))
			{
				ccPolyline* poly = ccHObjectCaster::ToPolyline(ent);
				ccColor::Rgb col(	static_cast<ColorCompType>(colour.red()),
										static_cast<ColorCompType>(colour.green()),
										static_cast<ColorCompType>(colour.blue()) );
				poly->setColor(col);
				ent->showColors(true);
				ent->prepareDisplayForRefresh();
			}
			else if (ent->isA(CC_TYPES::FACET))
			{
				ccFacet* facet = ccHObjectCaster::ToFacet(ent);
				ccColor::Rgb col(	static_cast<ColorCompType>(colour.red()),
										static_cast<ColorCompType>(colour.green()),
										static_cast<ColorCompType>(colour.blue()) );
				facet->setColor(col);
				ent->showColors(true);
				ent->prepareDisplayForRefresh();
			}
			else
			{
				ccLog::Warning(QString("[SetColor] Can't change color of entity '%1'").arg(ent->getName()));
			}
		}
	}
	
	void	rgbToGreyScale(ccHObject::Container &selectedEntities)
	{
		size_t selNum = selectedEntities.size();
		
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selNum == 1);
				continue;
			}
			
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				if (pc->hasColors())
				{
					pc->convertRGBToGreyScale();
					pc->showColors(true);
					pc->prepareDisplayForRefresh();
				}
			}
		}
	}
	
	void setColorGradient(ccHObject::Container &selectedEntities, QWidget *parent)
	{	
		ccColorGradientDlg dlg(parent);
		if (!dlg.exec())
			return;
		
		unsigned char dim = dlg.getDimension();
		ccColorGradientDlg::GradientType ramp = dlg.getType();
		
		ccColorScale::Shared colorScale(0);
		if (ramp == ccColorGradientDlg::Default)
		{
			colorScale = ccColorScalesManager::GetDefaultScale();
		}
		else if (ramp == ccColorGradientDlg::TwoColors)
		{
			colorScale = ccColorScale::Create("Temp scale");
			QColor first,second;
			dlg.getColors(first,second);
			colorScale->insert(ccColorScaleElement(0.0,first),false);
			colorScale->insert(ccColorScaleElement(1.0,second),true);
		}
		
		Q_ASSERT(colorScale || ramp == ccColorGradientDlg::Banding);
		
		const int	frequency = dlg.getBandingFrequency();
		
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				
				bool success = false;
				if (ramp == ccColorGradientDlg::Banding)
					success = pc->setRGBColorByBanding(dim, frequency);
				else
					success = pc->setRGBColorByHeight(dim, colorScale);
				
				if (success)
				{
					ent->showColors(true);
					ent->prepareDisplayForRefresh();
				}
			}
		}
	}
	
	void	changeColorLevels(ccHObject::Container &selectedEntities, QWidget *parent)
	{
		if (selectedEntities.size() != 1)
		{
			ccConsole::Error("Select one and only one colored cloud or mesh!");
			return;
		}
		
		bool lockedVertices;
		ccPointCloud* pointCloud = ccHObjectCaster::ToPointCloud(selectedEntities[0],&lockedVertices);
		if (!pointCloud || lockedVertices)
		{
			if (lockedVertices)
				ccUtils::DisplayLockedVerticesWarning(pointCloud->getName(),true);
			return;
		}
		
		if (!pointCloud->hasColors())
		{
			ccConsole::Error("Selected entity has no colors!");
			return;
		}
		
		ccColorLevelsDlg dlg(parent,pointCloud);
		dlg.exec();
	}
	
	void	interpolateColors(ccHObject::Container &selectedEntities, QWidget *parent)
	{
		if (selectedEntities.size() != 2)
		{
			ccConsole::Error("Select 2 entities (clouds or meshes)!");
			return;
		}
		
		ccHObject* ent1 = selectedEntities[0];
		ccHObject* ent2 = selectedEntities[1];
		
		ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(ent1);
		ccGenericPointCloud* cloud2 = ccHObjectCaster::ToGenericPointCloud(ent2);
		
		if (!cloud1 || !cloud2)
		{
			ccConsole::Error("Select 2 entities (clouds or meshes)!");
			return;
		}
		
		if (!cloud1->hasColors() && !cloud2->hasColors())
		{
			ccConsole::Error("None of the selected entities has per-point or per-vertex colors!");
			return;
		}
		else if (cloud1->hasColors() && cloud2->hasColors())
		{
			ccConsole::Error("Both entities have colors! Remove the colors on the entity you wish to import the colors to!");
			return;
		}
		
		ccGenericPointCloud* source = cloud1;
		ccGenericPointCloud* dest = cloud2;
		
		if ( cloud2->hasColors())
		{
			std::swap(source,dest);
			std::swap(cloud1,cloud2);
			std::swap(ent1,ent2);
		}
		
		if (!dest->isA(CC_TYPES::POINT_CLOUD))
		{
			ccConsole::Error("Destination cloud (or vertices) must be a real point cloud!");
			return;
		}
		
		bool ok = false;
		unsigned char defaultLevel = 7;
		int value = QInputDialog::getInt(parent,
													"Interpolate colors",
													"Octree level",
													defaultLevel,
													1,
													CCLib::DgmOctree::MAX_OCTREE_LEVEL,
													1,
													&ok);
		if (!ok)
			return;
		
		defaultLevel = static_cast<unsigned char>(value);
		
		ccProgressDialog pDlg(true, parent);
		
		if (static_cast<ccPointCloud*>(dest)->interpolateColorsFrom(source,&pDlg,defaultLevel))
		{
			ent2->showColors(true);
		}
		else
		{
			ccConsole::Error("An error occurred! (see console)");
		}
		
		ent2->prepareDisplayForRefresh_recursive();
	}
	
	void	invertNormals(ccHObject::Container &selectedEntities)
	{
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
			{
				ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
				if (ccCloud->hasNormals())
				{
					ccCloud->invertNormals();
					ccCloud->showNormals(true);
					ccCloud->prepareDisplayForRefresh_recursive();
				}
			}
		}
	}
	
	void	convertNormalsTo(ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest)
	{
		unsigned errorCount = 0;
		
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
			{
				ccPointCloud* ccCloud = static_cast<ccPointCloud*>(cloud);
				if (ccCloud->hasNormals())
				{
					bool success = true;
					switch(dest)
					{
						case HSV_COLORS:
						{
							success = ccCloud->convertNormalToRGB();
							if (success)
							{
								ccCloud->showSF(false);
								ccCloud->showNormals(false);
								ccCloud->showColors(true);
							}
						}
							break;
						case DIP_DIR_SFS:
						{
							//get/create 'dip' scalar field
							int dipSFIndex = ccCloud->getScalarFieldIndexByName(CC_DEFAULT_DIP_SF_NAME);
							if (dipSFIndex < 0)
								dipSFIndex = ccCloud->addScalarField(CC_DEFAULT_DIP_SF_NAME);
							if (dipSFIndex < 0)
							{
								ccLog::Warning("[ccEntityAction::convertNormalsTo] Not enough memory!");
								success = false;
								break;
							}
							
							//get/create 'dip direction' scalar field
							int dipDirSFIndex = ccCloud->getScalarFieldIndexByName(CC_DEFAULT_DIP_DIR_SF_NAME);
							if (dipDirSFIndex < 0)
								dipDirSFIndex = ccCloud->addScalarField(CC_DEFAULT_DIP_DIR_SF_NAME);
							if (dipDirSFIndex < 0)
							{
								ccCloud->deleteScalarField(dipSFIndex);
								ccLog::Warning("[ccEntityAction::convertNormalsTo] Not enough memory!");
								success = false;
								break;
							}
							
							ccScalarField* dipSF = static_cast<ccScalarField*>(ccCloud->getScalarField(dipSFIndex));
							ccScalarField* dipDirSF = static_cast<ccScalarField*>(ccCloud->getScalarField(dipDirSFIndex));
							assert(dipSF && dipDirSF);
							
							success = ccCloud->convertNormalToDipDirSFs(dipSF, dipDirSF);
							
							if (success)
							{
								//apply default 360 degrees color scale!
								ccColorScale::Shared scale = ccColorScalesManager::GetDefaultScale(ccColorScalesManager::HSV_360_DEG);
								dipSF->setColorScale(scale);
								dipDirSF->setColorScale(scale);
								ccCloud->setCurrentDisplayedScalarField(dipDirSFIndex); //dip dir. seems more interesting by default
								ccCloud->showSF(true);
							}
							else
							{
								ccCloud->deleteScalarField(dipSFIndex);
								ccCloud->deleteScalarField(dipDirSFIndex);
							}
						}
							break;
						default:
							assert(false);
							ccLog::Warning("[ccEntityAction::convertNormalsTo] Internal error: unhandled destination!");
							success = false;
							i = selNum; //no need to process the selected entities anymore!
							break;
					}
					
					if (success)
					{
						ccCloud->prepareDisplayForRefresh_recursive();
					}
					else
					{
						++errorCount;
					}
				}
			}
		}
		
		//errors should have been sent to console as warnings
		if (errorCount)
		{
			ccConsole::Error("Error(s) occurred! (see console)");
		}
	}
}
