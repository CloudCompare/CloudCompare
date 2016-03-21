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
#include <QElapsedTimer>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>

#include "ScalarFieldTools.h"

#include "ccCommon.h"
#include "ccConsole.h"
#include "ccGenericPrimitive.h"
#include "ccHObjectCaster.h"
#include "ccOctree.h"
#include "ccScalarField.h"

#include "ccColorScalesManager.h"
#include "ccFacet.h"

#include "ccAskTwoDoubleValuesDlg.h"
#include "ccColorGradientDlg.h"
#include "ccColorLevelsDlg.h"
#include "ccComputeOctreeDlg.h"
#include "ccExportCoordToSFDlg.h"
#include "ccNormalComputationDlg.h"
#include "ccProgressDialog.h"

#include "ccEntityAction.h"
#include "ccLibAlgorithms.h"
#include "ccUtils.h"

// This is included only for temporarily removing an object from the tree.
//	TODO figure out a cleaner way to do this without having to include all of mainwindow.h
#include "mainwindow.h"


namespace ccEntityAction
{
	//////////
	// Colours
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
	
	void	rgbToGreyScale(const ccHObject::Container &selectedEntities)
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
	
	void setColorGradient(const ccHObject::Container &selectedEntities, QWidget *parent)
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
	
	void	changeColorLevels(const ccHObject::Container &selectedEntities, QWidget *parent)
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
	
	void	interpolateColors(const ccHObject::Container &selectedEntities, QWidget *parent)
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
													"Interpolate colors", "Octree level",
													defaultLevel,
													1, CCLib::DgmOctree::MAX_OCTREE_LEVEL,
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
	
	//////////
	// Scalar Fields
	
	void sfGaussianFilter(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		size_t selNum = selectedEntities.size();
		if (selNum == 0)
			return;
		
		double sigma = ccLibAlgorithms::GetDefaultCloudKernelSize(selectedEntities);
		if (sigma < 0.0)
		{
			ccConsole::Error("No eligible point cloud in selection!");
			return;
		}
		
		bool ok = false;
		sigma = QInputDialog::getDouble(parent,
												  "Gaussian filter", "sigma:",
												  sigma,
												  DBL_MIN,1.0e9,
												  8,
												  &ok);
		if (!ok)
			return;
		
		for (size_t i=0; i<selNum; ++i)
		{
			bool lockedVertices = false;
			ccHObject* ent = selectedEntities[i];
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
			if (!pc || lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			
			//la methode est activee sur le champ scalaire affiche
			CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf != nullptr)
			{
				//on met en lecture (OUT) le champ scalaire actuellement affiche
				int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
				Q_ASSERT(outSfIdx >= 0);
				
				pc->setCurrentOutScalarField(outSfIdx);
				CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
				Q_ASSERT(sf != nullptr);
				
				QString sfName = QString("%1.smooth(%2)").arg(outSF->getName()).arg(sigma);
				int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
				if (sfIdx < 0)
					sfIdx = pc->addScalarField(qPrintable(sfName)); //output SF has same type as input SF
				if (sfIdx >= 0)
					pc->setCurrentInScalarField(sfIdx);
				else
				{
					ccConsole::Error(QString("Failed to create scalar field for cloud '%1' (not enough memory?)").arg(pc->getName()));
					continue;
				}
				
				ccOctree* octree = pc->getOctree();
				if (!octree)
				{
					ccProgressDialog pDlg(true, parent);
					octree = pc->computeOctree(&pDlg);
					if (!octree)
					{
						ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
						continue;
					}
				}
				
				if (octree)
				{
					ccProgressDialog pDlg(true, parent);
					QElapsedTimer eTimer;
					eTimer.start();
					CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(static_cast<PointCoordinateType>(sigma),
																							  pc,
																							  -1,
																							  &pDlg,
																							  octree);
					ccConsole::Print("[GaussianFilter] Timing: %3.2f s.",static_cast<double>(eTimer.elapsed())/1.0e3);
					pc->setCurrentDisplayedScalarField(sfIdx);
					pc->showSF(sfIdx >= 0);
					sf = pc->getCurrentDisplayedScalarField();
					if (sf)
						sf->computeMinAndMax();
					pc->prepareDisplayForRefresh_recursive();
				}
				else
				{
					ccConsole::Error(QString("Failed to compute entity [%1] octree! (not enough memory?)").arg(pc->getName()));
				}
			}
			else
			{
				ccConsole::Warning(QString("Entity [%1] has no active scalar field!").arg(pc->getName()));
			}
		}
	}
	
	void	sfBilateralFilter(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		size_t selNum = selectedEntities.size();
		if (selNum == 0)
			return;
		
		double sigma = ccLibAlgorithms::GetDefaultCloudKernelSize(selectedEntities);
		if (sigma < 0.0)
		{
			ccConsole::Error("No eligible point cloud in selection!");
			return;
		}
		
		//estimate a good value for scalar field sigma, based on the first cloud
		//and its displayed scalar field
		ccPointCloud* pc_test = ccHObjectCaster::ToPointCloud(selectedEntities[0]);
		CCLib::ScalarField* sf_test = pc_test->getCurrentDisplayedScalarField();
		ScalarType range = sf_test->getMax() - sf_test->getMin();
		double scalarFieldSigma = range / 4; // using 1/4 of total range
		
		
		ccAskTwoDoubleValuesDlg dlg("Spatial sigma", "Scalar sigma",
											 DBL_MIN, 1.0e9,
											 sigma, scalarFieldSigma,
											 8, nullptr, parent);
		dlg.doubleSpinBox1->setStatusTip("3*sigma = 98% attenuation");
		dlg.doubleSpinBox2->setStatusTip("Scalar field's sigma controls how much the filter behaves as a Gaussian Filter\n sigma at +inf uses the whole range of scalars ");
		if (!dlg.exec())
			return;
		
		//get values
		sigma = dlg.doubleSpinBox1->value();
		scalarFieldSigma = dlg.doubleSpinBox2->value();
		
		for (size_t i=0; i<selNum; ++i)
		{
			bool lockedVertices = false;
			ccHObject* ent = selectedEntities[i];
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
			if (!pc || lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			
			//the algorithm will use the currently displayed SF
			CCLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf)
			{
				//we set the displayed SF as "OUT" SF
				int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
				assert(outSfIdx >= 0);
				
				pc->setCurrentOutScalarField(outSfIdx);
				CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
				assert(sf);
				
				QString sfName = QString("%1.bilsmooth(%2,%3)").arg(outSF->getName()).arg(sigma).arg(scalarFieldSigma);
				int sfIdx = pc->getScalarFieldIndexByName(qPrintable(sfName));
				if (sfIdx < 0)
					sfIdx = pc->addScalarField(qPrintable(sfName)); //output SF has same type as input SF
				if (sfIdx >= 0)
					pc->setCurrentInScalarField(sfIdx);
				else
				{
					ccConsole::Error(QString("Failed to create scalar field for cloud '%1' (not enough memory?)").arg(pc->getName()));
					continue;
				}
				
				ccOctree* octree = pc->getOctree();
				if (!octree)
				{
					ccProgressDialog pDlg(true,parent);
					octree = pc->computeOctree(&pDlg);
					if (!octree)
					{
						ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
						continue;
					}
				}
				
				assert(octree);
				{
					ccProgressDialog pDlg(true,parent);
					QElapsedTimer eTimer;
					eTimer.start();
					
					CCLib::ScalarFieldTools::applyScalarFieldGaussianFilter(static_cast<PointCoordinateType>(sigma),
																							  pc,
																							  static_cast<PointCoordinateType>(scalarFieldSigma),
																							  &pDlg,
																							  octree);
					ccConsole::Print("[BilateralFilter] Timing: %3.2f s.",eTimer.elapsed()/1.0e3);
					pc->setCurrentDisplayedScalarField(sfIdx);
					pc->showSF(sfIdx >= 0);
					sf = pc->getCurrentDisplayedScalarField();
					if (sf)
						sf->computeMinAndMax();
					pc->prepareDisplayForRefresh_recursive();
				}
			}
			else
			{
				ccConsole::Warning(QString("Entity [%1] has no active scalar field!").arg(pc->getName()));
			}
		}
	}
	
	void sfConvertToRGB(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		//we first ask the user if the SF colors should be mixed with existing colors
		bool mixWithExistingColors = false;
		
		QMessageBox::StandardButton answer = QMessageBox::warning(parent,
																					 "Scalar Field to RGB",
																					 "Mix with existing colors (if any)?",
																					 QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
																					 QMessageBox::Yes );
		if (answer == QMessageBox::Yes)
			mixWithExistingColors = true;
		else if (answer == QMessageBox::Cancel)
			return;
		
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccGenericPointCloud* cloud = nullptr;
			ccHObject* ent = selectedEntities[i];
			
			bool lockedVertices = false;
			cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			if (cloud != nullptr) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				//if there is no displayed SF --> nothing to do!
				if (pc->getCurrentDisplayedScalarField())
				{
					if (pc->setRGBColorWithCurrentScalarField(mixWithExistingColors))
					{
						ent->showColors(true);
						ent->showSF(false);
					}
				}
				
				cloud->prepareDisplayForRefresh_recursive();
			}
		}
	}
	
	void	sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		static int s_randomColorsNumber = 256;
		
		bool ok = false;
		s_randomColorsNumber = QInputDialog::getInt(parent,
																  "Random colors",
																  "Number of random colors (will be regularly sampled over the SF interval):",
																  s_randomColorsNumber,
																  2, 2147483647,
																  16,
																  &ok);
		if (!ok)
			return;
		Q_ASSERT(s_randomColorsNumber > 1);
		
		ColorsTableType* randomColors = new ColorsTableType;
		if (!randomColors->reserve(static_cast<unsigned>(s_randomColorsNumber)))
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
		
		//generate random colors
		for (int i=0; i<s_randomColorsNumber; ++i)
		{
			ccColor::Rgb col = ccColor::Generator::Random();
			randomColors->addElement(col.rgb);
		}
		
		//apply random colors
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccGenericPointCloud* cloud = nullptr;
			ccHObject* ent = selectedEntities[i];
			
			bool lockedVertices = false;
			cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			if (cloud != nullptr) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				ccScalarField* sf = pc->getCurrentDisplayedScalarField();
				//if there is no displayed SF --> nothing to do!
				if (sf && sf->currentSize() >= pc->size())
				{
					if (!pc->resizeTheRGBTable(false))
					{
						ccConsole::Error("Not enough memory!");
						break;
					}
					else
					{
						ScalarType minSF = sf->getMin();
						ScalarType maxSF = sf->getMax();
						
						ScalarType step = (maxSF-minSF)/(s_randomColorsNumber-1);
						if (step == 0)
							step = static_cast<ScalarType>(1.0);
						
						for (unsigned i=0; i<pc->size(); ++i)
						{
							ScalarType val = sf->getValue(i);
							unsigned colIndex = static_cast<unsigned>((val-minSF)/step);
							if (colIndex == s_randomColorsNumber)
								--colIndex;
							
							pc->setPointColor(i,randomColors->getValue(colIndex));
						}
						
						pc->showColors(true);
						pc->showSF(false);
					}
				}
				
				cloud->prepareDisplayForRefresh_recursive();
			}
		}
	}
	
	void sfRename(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(selectedEntities[i]);
			if (cloud != nullptr) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				ccScalarField* sf = pc->getCurrentDisplayedScalarField();
				//if there is no displayed SF --> nothing to do!
				if (sf == nullptr)
				{
					ccConsole::Warning(QString("Cloud %1 has no displayed scalar field!").arg(pc->getName()));
				}
				else
				{
					const char* sfName = sf->getName();
					bool ok = false;
					QString newName = QInputDialog::getText(parent,
																		 "SF name", "name:",
																		 QLineEdit::Normal,
																		 QString(sfName ? sfName : "unknown"),
																		 &ok);
					if (ok)
						sf->setName(qPrintable(newName));
				}
			}
		}
	}
	
	void	sfAddIdField(const ccHObject::Container &selectedEntities)
	{
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(selectedEntities[i]);
			if (cloud != nullptr) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				
				int sfIdx = pc->getScalarFieldIndexByName(CC_DEFAULT_ID_SF_NAME);
				if (sfIdx < 0)
					sfIdx = pc->addScalarField(CC_DEFAULT_ID_SF_NAME);
				if (sfIdx < 0)
				{
					ccLog::Warning("Not enough memory!");
					return;
				}
				
				CCLib::ScalarField* sf = pc->getScalarField(sfIdx);
				Q_ASSERT(sf->currentSize() == pc->size());
				
				for (unsigned j=0 ; j<cloud->size(); j++)
				{
					ScalarType idValue = static_cast<ScalarType>(j);
					sf->setValue(j, idValue);
				}
				
				sf->computeMinAndMax();
				pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(true);
				pc->prepareDisplayForRefresh();
			}
		}
	}
	
	void	sfAsCoord(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		ccExportCoordToSFDlg ectsDlg(parent);
		ectsDlg.warningLabel->setVisible(false);
		ectsDlg.setWindowTitle("Export SF to coordinate(s)");
		
		if (!ectsDlg.exec())
			return;
		
		bool exportDim[3] = {ectsDlg.exportX(), ectsDlg.exportY(), ectsDlg.exportZ()};
		if (!exportDim[0] && !exportDim[1] && !exportDim[2]) //nothing to do?!
			return;
		
		//for each selected cloud (or vertices set)
		size_t selNum = selectedEntities.size();
		for (size_t i=0; i<selNum; ++i)
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(selectedEntities[i]);
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				
				ccScalarField* sf = pc->getCurrentDisplayedScalarField();
				if (sf != nullptr)
				{
					unsigned ptsCount = pc->size();
					bool hasDefaultValueForNaN = false;
					ScalarType defaultValueForNaN = sf->getMin();
					
					for (unsigned i=0; i<ptsCount; ++i)
					{
						ScalarType s = sf->getValue(i);
						
						//handle NaN values
						if (!CCLib::ScalarField::ValidValue(s))
						{
							if (!hasDefaultValueForNaN)
							{
								bool ok = false;
								double out = QInputDialog::getDouble(parent,
																				 "SF --> coordinate",
																				 "Enter the coordinate equivalent for NaN values:",
																				 defaultValueForNaN, -1.0e9,
																				 1.0e9, 6,
																				 &ok);
								if (ok)
									defaultValueForNaN = static_cast<ScalarType>(out);
								else
									ccLog::Warning("[SetSFAsCoord] By default the coordinate equivalent for NaN values will be the minimum SF value");
								hasDefaultValueForNaN = true;
							}
							s = defaultValueForNaN;
						}
						
						CCVector3* P = const_cast<CCVector3*>(pc->getPoint(i));
						
						//test each dimension
						if (exportDim[0])
							P->x = s;
						if (exportDim[1])
							P->y = s;
						if (exportDim[2])
							P->z = s;
					}
					
					pc->invalidateBoundingBox();
				}
			}
		}
	}
	
	//////////
	// Normals
	
	void	computeNormals(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		if (selectedEntities.empty())
		{
			ccConsole::Error("Select at least one point cloud");
			return;
		}
		
		//look for clouds and meshes
		std::vector<ccPointCloud*> clouds;
		size_t cloudsWithScanGrids = 0;
		std::vector<ccMesh*> meshes;
		PointCoordinateType defaultRadius = 0;
		try
		{
			for (size_t i=0; i<selectedEntities.size(); ++i)
			{
				ccHObject	*entity = selectedEntities[i];
				if (entity->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
					clouds.push_back(cloud);
					
					if (cloud->gridCount() != 0)
						++cloudsWithScanGrids;
					
					if (defaultRadius == 0)
					{
						//default radius
						defaultRadius = ccNormalVectors::GuessNaiveRadius(cloud);
					}
				}
				else if (entity->isKindOf(CC_TYPES::MESH))
				{
					if (entity->isA(CC_TYPES::MESH))
					{
						ccMesh* mesh = ccHObjectCaster::ToMesh(entity);
						meshes.push_back(mesh);
					}
					else
					{
						ccConsole::Error(QString("Can't compute normals on sub-meshes! Select the parent mesh instead"));
						return;
					}
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			ccConsole::Error("Not enough memory!");
			return;
		}
		
		//compute normals for each selected cloud
		if (!clouds.empty())
		{
			ccNormalComputationDlg::SelectionMode selectionMode = ccNormalComputationDlg::WITHOUT_SCAN_GRIDS;
			if (cloudsWithScanGrids)
			{
				if (clouds.size() == cloudsWithScanGrids)
				{
					//all clouds have an associated grid
					selectionMode = ccNormalComputationDlg::WITH_SCAN_GRIDS;
				}
				else
				{
					//only a part of the clouds have an associated grid
					selectionMode = ccNormalComputationDlg::MIXED;
				}
			}
			
			static CC_LOCAL_MODEL_TYPES s_lastModelType = LS;
			static ccNormalVectors::Orientation s_lastNormalOrientation = ccNormalVectors::UNDEFINED;
			static int s_lastMSTNeighborCount = 6;
			static int s_lastKernelSize = 2;
			
			ccNormalComputationDlg ncDlg(selectionMode, parent);
			ncDlg.setLocalModel(s_lastModelType);
			ncDlg.setRadius(defaultRadius);
			ncDlg.setPreferredOrientation(s_lastNormalOrientation);
			ncDlg.setMSTNeighborCount(s_lastMSTNeighborCount);
			ncDlg.setGridKernelSize(s_lastKernelSize);
			if (clouds.size() == 1)
			{
				ncDlg.setCloud(clouds.front());
			}
			
			if (!ncDlg.exec())
				return;
			
			//normals computation
			CC_LOCAL_MODEL_TYPES model = s_lastModelType = ncDlg.getLocalModel();
			bool useGridStructure = cloudsWithScanGrids && ncDlg.useScanGridsForComputation();
			defaultRadius = ncDlg.getRadius();
			int kernelSize = s_lastKernelSize = ncDlg.getGridKernelSize();
			
			//normals orientation
			bool orientNormals = ncDlg.orientNormals();
			bool orientNormalsWithGrids = cloudsWithScanGrids && ncDlg.useScanGridsForOrientation();
			ccNormalVectors::Orientation preferredOrientation = s_lastNormalOrientation = ncDlg.getPreferredOrientation();
			bool orientNormalsMST = ncDlg.useMSTOrientation();
			int mstNeighbors = s_lastMSTNeighborCount = ncDlg.getMSTNeighborCount();
			
			size_t errors = 0;
			for (size_t i=0; i<clouds.size(); i++)
			{
				ccPointCloud* cloud = clouds[i];
				Q_ASSERT(cloud != nullptr);
				
				ccProgressDialog pDlg(true,parent);
				
				bool result = false;
				bool orientNormalsForThisCloud = false;
				if (useGridStructure && cloud->gridCount())
				{
#if 0
					ccPointCloud* newCloud = new ccPointCloud("temp");
					newCloud->reserve(cloud->size());
					for (size_t gi=0; gi<cloud->gridCount(); ++gi)
					{
						const ccPointCloud::Grid::Shared& scanGrid = cloud->grid(gi);
						if (scanGrid && scanGrid->indexes.empty())
						{
							//empty grid, we skip it
							continue;
						}
						ccGLMatrixd toSensor = scanGrid->sensorPosition.inverse();
						
						const int* _indexGrid = &(scanGrid->indexes[0]);
						for (int j=0; j<static_cast<int>(scanGrid->h); ++j)
						{
							for (int i=0; i<static_cast<int>(scanGrid->w); ++i, ++_indexGrid)
							{
								if (*_indexGrid >= 0)
								{
									unsigned pointIndex = static_cast<unsigned>(*_indexGrid);
									const CCVector3* P = cloud->getPoint(pointIndex);
									CCVector3 Q = toSensor * (*P);
									newCloud->addPoint(Q);
								}
							}
						}
						
						addToDB(newCloud);
					}
#endif
					
					//compute normals with the associated scan grid(s)
					orientNormalsForThisCloud = orientNormals && orientNormalsWithGrids;
					result = cloud->computeNormalsWithGrids(model, kernelSize, orientNormalsForThisCloud, &pDlg);
				}
				else
				{
					//compute normals with the octree
					orientNormalsForThisCloud = orientNormals && (preferredOrientation != ccNormalVectors::UNDEFINED);
					result = cloud->computeNormalsWithOctree(model, orientNormals ? preferredOrientation : ccNormalVectors::UNDEFINED, defaultRadius, &pDlg);
				}
				
				//do we need to orient the normals? (this may have been already done if 'orientNormalsForThisCloud' is true)
				if (result && orientNormals && !orientNormalsForThisCloud)
				{
					if (cloud->gridCount() && orientNormalsWithGrids)
					{
						//we can still use the grid structure(s) to orient the normals!
						result = cloud->orientNormalsWithGrids();
					}
					else if (orientNormalsMST)
					{
						//use Minimum Spanning Tree to resolve normals direction
						result = cloud->orientNormalsWithMST(mstNeighbors, &pDlg);
					}
				}
				
				if (!result)
				{
					++errors;
				}
				
				cloud->prepareDisplayForRefresh();
			}
			
			if (errors != 0)
			{
				if (errors < clouds.size())
					ccConsole::Error("Failed to compute or orient the normals on some clouds! (see console)");
				else
					ccConsole::Error("Failed to compute or orient the normals! (see console)");
			}
		}
		
		//compute normals for each selected mesh
		if (!meshes.empty())
		{
			QMessageBox question( QMessageBox::Question,
										 "Mesh normals",
										 "Compute per-vertex normals (smooth) or per-triangle (faceted)?",
										 QMessageBox::NoButton,
										 parent);
			
			QPushButton* perVertexButton   = question.addButton("Per-vertex", QMessageBox::YesRole);
			QPushButton* perTriangleButton = question.addButton("Per-triangle", QMessageBox::NoRole);
			
			question.exec();
			
			bool computePerVertexNormals = (question.clickedButton() == perVertexButton);
			
			for (size_t i=0; i<meshes.size(); i++)
			{
				ccMesh* mesh = meshes[i];
				Q_ASSERT(mesh != nullptr);
				
				//we remove temporarily the mesh as its normals may be removed (and they can be a child object)
				MainWindow* instance = dynamic_cast<MainWindow*>(parent);
				MainWindow::ccHObjectContext objContext;
				if (instance)
					objContext = instance->removeObjectTemporarilyFromDBTree(mesh);
				mesh->clearTriNormals();
				mesh->showNormals(false);
				bool result = mesh->computeNormals(computePerVertexNormals);
				if (instance)
					instance->putObjectBackIntoDBTree(mesh,objContext);
				
				if (!result)
				{
					ccConsole::Error(QString("Failed to compute normals on mesh '%1'").arg(mesh->getName()));
					continue;
				}
				mesh->prepareDisplayForRefresh_recursive();
			}
		}
	}
	
	void	invertNormals(const ccHObject::Container &selectedEntities)
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
	
	void	convertNormalsTo(const ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest)
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
	
	//////////
	// Octree
	
	void computeOctree(const ccHObject::Container &selectedEntities, QWidget *parent)
	{
		ccBBox bbox;
		std::unordered_set<ccGenericPointCloud*> clouds;
		size_t selNum = selectedEntities.size();
		PointCoordinateType maxBoxSize = -1;
		for (size_t i=0; i<selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			
			//specific test for locked vertices
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (cloud && lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(),selNum == 1);
				continue;
			}
			clouds.insert(cloud);
			
			//we look for the biggest box so as to define the "minimum cell size"
			const ccBBox thisBBox = cloud->getOwnBB();
			if (thisBBox.isValid())
			{
				CCVector3 dd = thisBBox.maxCorner()-thisBBox.minCorner();
				PointCoordinateType maxd = std::max(dd.x,std::max(dd.y,dd.z));
				if (maxBoxSize < 0.0 || maxd > maxBoxSize)
					maxBoxSize = maxd;
			}
			bbox += thisBBox;
		}
		
		if (clouds.empty() || maxBoxSize < 0.0)
		{
			ccLog::Warning("[doActionComputeOctree] No eligible entities in selection!");
			return;
		}
		
		//min(cellSize) = max(dim)/2^N with N = max subidivision level
		const double minCellSize = static_cast<double>(maxBoxSize)/(1 << ccOctree::MAX_OCTREE_LEVEL);
		
		ccComputeOctreeDlg coDlg(bbox,minCellSize,parent);
		if (!coDlg.exec())
			return;
		
		ccProgressDialog pDlg(true,parent);
		
		//if we must use a custom bounding box, we update 'bbox'
		if (coDlg.getMode() == ccComputeOctreeDlg::CUSTOM_BBOX)
			bbox = coDlg.getCustomBBox();
		
		for (std::unordered_set<ccGenericPointCloud*>::iterator it = clouds.begin(); it != clouds.end(); ++it)
		{
			ccGenericPointCloud* cloud = *it;
			
			//we temporarily detach entity, as it may undergo
			//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::computeOctree
			MainWindow* instance = dynamic_cast<MainWindow*>(parent);
			MainWindow::ccHObjectContext objContext;
			if (instance)
				objContext = instance->removeObjectTemporarilyFromDBTree(cloud);
			
			//computation
			QElapsedTimer eTimer;
			eTimer.start();
			ccOctree* octree = nullptr;
			switch(coDlg.getMode())
			{
				case ccComputeOctreeDlg::DEFAULT:
					octree = cloud->computeOctree(&pDlg);
					break;
				case ccComputeOctreeDlg::MIN_CELL_SIZE:
				case ccComputeOctreeDlg::CUSTOM_BBOX:
				{
					//for a cell-size based custom box, we must update it for each cloud!
					if (coDlg.getMode() == ccComputeOctreeDlg::MIN_CELL_SIZE)
					{
						double cellSize = coDlg.getMinCellSize();
						PointCoordinateType halfBoxWidth = (PointCoordinateType)(cellSize * (1 << ccOctree::MAX_OCTREE_LEVEL) / 2.0);
						CCVector3 C = cloud->getOwnBB().getCenter();
						bbox = ccBBox(	C-CCVector3(halfBoxWidth,halfBoxWidth,halfBoxWidth),
											C+CCVector3(halfBoxWidth,halfBoxWidth,halfBoxWidth));
					}
					cloud->deleteOctree();
					octree = new ccOctree(cloud);
					if (octree->build(bbox.minCorner(),bbox.maxCorner(),0,0,&pDlg) > 0)
					{
						octree->setDisplay(cloud->getDisplay());
						cloud->addChild(octree);
					}
					else
					{
						delete octree;
						octree = nullptr;
					}
				}
					break;
				default:
					Q_ASSERT(false);
					return;
			}
			qint64 elapsedTime_ms = eTimer.elapsed();
			
			//put object back in tree
			if (instance)
				instance->putObjectBackIntoDBTree(cloud,objContext);
			
			if (octree)
			{
				ccConsole::Print("[doActionComputeOctree] Timing: %2.3f s",static_cast<double>(elapsedTime_ms)/1.0e3);
				cloud->setEnabled(true); //for mesh vertices!
				octree->setVisible(true);
				octree->prepareDisplayForRefresh();
			}
			else
			{
				ccConsole::Warning(QString("Octree computation on cloud '%1' failed!").arg(cloud->getName()));
			}
		}
	}
}
