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

//Qt
#include <QColorDialog>
#include <QElapsedTimer>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>

//CCCoreLib
#include <NormalDistribution.h>
#include <ScalarFieldTools.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>
#include <ReferenceCloud.h>

//qCC_db
#include <ccColorScalesManager.h>
#include <ccFacet.h>
#include <ccGenericPrimitive.h>
#include <ccOctreeProxy.h>
#include <ccPointCloud.h>
#include <ccPointCloudInterpolator.h>
#include <ccPolyline.h>
#include <ccSensor.h>

//qCC_gl
#include "ccGuiParameters.h"

//common
#include <ccPickOneElementDlg.h>

//Local
#include "ccAskTwoDoubleValuesDlg.h"
#include "ccAskThreeDoubleValuesDlg.h"
#include "ccColorGradientDlg.h"
#include "ccColorLevelsDlg.h"
#include "ccComputeOctreeDlg.h"
#include "ccExportCoordToSFDlg.h"
#include "ccInterpolationDlg.h"
#include "ccItemSelectionDlg.h"
#include "ccNormalComputationDlg.h"
#include "ccOrderChoiceDlg.h"
#include "ccProgressDialog.h"
#include "ccScalarFieldArithmeticsDlg.h"
#include "ccScalarFieldFromColorDlg.h"
#include "ccSetSFAsVec3Dlg.h"
#include "ccStatisticalTestDlg.h"

#include "ccCommon.h"
#include "ccConsole.h"
#include "ccEntityAction.h"
#include "ccHistogramWindow.h"
#include "ccLibAlgorithms.h"
#include "ccUtils.h"

// This is included only for temporarily removing an object from the tree.
#include "ccMainAppInterface.h"

// System
#include <unordered_set>
#include <array>

namespace ccEntityAction
{
	static QString GetFirstAvailableSFName(const ccPointCloud* cloud, const QString& baseName)
	{
		if (cloud == nullptr)
		{
			Q_ASSERT(false);
			return {};
		}
		
		QString name = baseName;
		for (int trials = 0; trials < 99; ++trials)
		{
			if (cloud->getScalarFieldIndexByName(name.toStdString()) < 0)
			{
				// Scalar field name is available
				return name;
			}
			// else, generate a new name
			name = QString("%1 #%2").arg(baseName).arg(trials + 1);
		}

		// we couldn't find an available name!
		return {};
	}
	
	//////////
	// Colours
	bool setColor(ccHObject::Container selectedEntities, bool colorize, QWidget* parent/*=nullptr*/)
	{
		static QColor s_lastColor = Qt::white;
		QColor colour = QColorDialog::getColor(s_lastColor, parent, QString(), QColorDialog::ShowAlphaChannel);
		
		if (!colour.isValid())
			return false;

		s_lastColor = colour;
		
		while (!selectedEntities.empty())
		{
			ccHObject* ent = selectedEntities.back();
			selectedEntities.pop_back();
			if (ent->isA(CC_TYPES::HIERARCHY_OBJECT))
			{
				//automatically parse a group's children set
				for (unsigned i = 0; i < ent->getChildrenNumber(); ++i)
					selectedEntities.push_back(ent->getChild(i));
			}
			else if (ent->isA(CC_TYPES::POINT_CLOUD) || ent->isA(CC_TYPES::MESH))
			{
				ccPointCloud* cloud = nullptr;
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
						ccLog::Warning(QObject::tr("[SetColor] Can't set color for mesh '%1' (vertices are not accessible)").arg(ent->getName()));
						continue;
					}
					
					cloud = static_cast<ccPointCloud*>(vertices);
				}
				
				if (colorize)
				{
					cloud->colorize(static_cast<float>(colour.redF()),
									static_cast<float>(colour.greenF()),
									static_cast<float>(colour.blueF()),
									static_cast<float>(colour.alphaF()));
				}
				else
				{
					cloud->setColor(ccColor::FromQColora(colour));
				}
				cloud->showColors(true);
				cloud->showSF(false); //just in case
				cloud->prepareDisplayForRefresh();
				
				if (ent != cloud)
				{
					ent->showColors(true);
				}
				else if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
				{
					cloud->getParent()->showColors(true);
					cloud->getParent()->showSF(false); //just in case
				}
			}
			else if (ent->isKindOf(CC_TYPES::PRIMITIVE))
			{
				ccGenericPrimitive* prim = ccHObjectCaster::ToPrimitive(ent);
				ccColor::Rgb col(	static_cast<ColorCompType>(colour.red()),
									static_cast<ColorCompType>(colour.green()),
									static_cast<ColorCompType>(colour.blue()) );
				prim->setColor(col);
				ent->showColors(true);
				ent->showSF(false); //just in case
				ent->prepareDisplayForRefresh();
			}
			else if (ent->isA(CC_TYPES::POLY_LINE))
			{
				ccPolyline* poly = ccHObjectCaster::ToPolyline(ent);
				poly->setColor(ccColor::FromQColor(colour));
				ent->showColors(true);
				ent->showSF(false); //just in case
				ent->prepareDisplayForRefresh();
			}
			else if (ent->isA(CC_TYPES::FACET))
			{
				ccFacet* facet = ccHObjectCaster::ToFacet(ent);
				facet->setColor(ccColor::FromQColor(colour));
				ent->showColors(true);
				ent->showSF(false); //just in case
				ent->prepareDisplayForRefresh();
			}
			else
			{
				ccLog::Warning(QObject::tr("[SetColor] Can't change color of entity '%1'").arg(ent->getName()));
			}
		}
		
		return true;
	}
	
	bool	rgbToGreyScale(const ccHObject::Container &selectedEntities)
	{
		for (ccHObject* ent : selectedEntities)
		{
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}
			
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				if (pc->hasColors())
				{
					pc->convertRGBToGreyScale();
					pc->showColors(true);
					pc->showSF(false); //just in case
					pc->prepareDisplayForRefresh();
				}
			}
		}
		
		return true;
	}
	
	bool setColorGradient(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{	
		ccColorGradientDlg dlg(parent);
		if (!dlg.exec())
			return false;
		
		unsigned char dim = dlg.getDimension();
		ccColorGradientDlg::GradientType ramp = dlg.getType();
		
		ccColorScale::Shared colorScale(nullptr);
		if (ramp == ccColorGradientDlg::Default)
		{
			colorScale = ccColorScalesManager::GetDefaultScale();
		}
		else if (ramp == ccColorGradientDlg::TwoColors)
		{
			colorScale = ccColorScale::Create("Temp scale");
			QColor first;
			QColor second;
			dlg.getColors(first,second);
			colorScale->insert(ccColorScaleElement(0.0, first), false);
			colorScale->insert(ccColorScaleElement(1.0, second), true);
		}
		
		Q_ASSERT(colorScale || ramp == ccColorGradientDlg::Banding);
		
		const double frequency = dlg.getBandingFrequency();
		
		for (ccHObject* ent : selectedEntities)
		{
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
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
					ent->showSF(false); //just in case
					ent->prepareDisplayForRefresh();
				}
			}
		}
		
		return true;
	}
	
	bool	changeColorLevels(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.size() != 1)
		{
			ccConsole::Error(QObject::tr("Select one and only one colored cloud or mesh!"));
			return false;
		}
		
		bool lockedVertices;
		ccPointCloud* pointCloud = ccHObjectCaster::ToPointCloud(selectedEntities[0], &lockedVertices);
		if (!pointCloud || lockedVertices)
		{
			if (lockedVertices && pointCloud)
				ccUtils::DisplayLockedVerticesWarning(pointCloud->getName(), true);
			return false;
		}
		
		if (!pointCloud->hasColors())
		{
			ccConsole::Error(QObject::tr("Selected entity has no colors!"));
			return false;
		}
		
		ccColorLevelsDlg dlg(parent, pointCloud);
		dlg.exec();
		
		return true;
	}
	
	//! Interpolate colors from on entity and transfer them to another one
	bool	interpolateColors(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.size() != 2)
		{
			ccConsole::Error(QObject::tr("Select 2 entities (clouds or meshes)!"));
			return false;
		}
		
		ccHObject* ent1 = selectedEntities[0];
		ccHObject* ent2 = selectedEntities[1];
		
		ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(ent1);
		ccGenericPointCloud* cloud2 = ccHObjectCaster::ToGenericPointCloud(ent2);
		
		if (!cloud1 || !cloud2)
		{
			ccConsole::Error(QObject::tr("Select 2 entities (clouds or meshes)!"));
			return false;
		}
		
		if (!cloud1->hasColors() && !cloud2->hasColors())
		{
			ccConsole::Error(QObject::tr("None of the selected entities has per-point or per-vertex colors!"));
			return false;
		}
		else if (cloud1->hasColors() && cloud2->hasColors())
		{
			ccConsole::Error(QObject::tr("Both entities have colors! Remove the colors on the entity you wish to import the colors to!"));
			return false;
		}
		
		ccGenericPointCloud* source = cloud1;
		ccGenericPointCloud* dest = cloud2;
		
		if ( cloud2->hasColors())
		{
			std::swap(source, dest);
			std::swap(cloud1, cloud2);
			std::swap(ent1, ent2);
		}
		
		if (!dest->isA(CC_TYPES::POINT_CLOUD))
		{
			ccConsole::Error(QObject::tr("Destination cloud (or vertices) must be a real point cloud!"));
			return false;
		}
		
		ccProgressDialog pDlg(true, parent);
		
		if (static_cast<ccPointCloud*>(dest)->interpolateColorsFrom(source, &pDlg))
		{
			ent2->showColors(true);
			ent2->showSF(false); //just in case
		}
		else
		{
			ccConsole::Error(QObject::tr("An error occurred! (see console)"));
		}
		
		ent2->prepareDisplayForRefresh_recursive();
		
		return true;
	}

    //! Interpolate scalar fields from one entity and transfer them to another one
	bool	interpolateSFs(const ccHObject::Container &selectedEntities, ccMainAppInterface* app)
	{
		if (selectedEntities.size() != 2)
		{
			ccConsole::Error(QObject::tr("Select 2 entities (clouds or meshes)!"));
			return false;
		}
		
		ccHObject* ent1 = selectedEntities[0];
		ccHObject* ent2 = selectedEntities[1];
		
		ccPointCloud* cloud1 = ccHObjectCaster::ToPointCloud(ent1);
		ccPointCloud* cloud2 = ccHObjectCaster::ToPointCloud(ent2);

		if (!cloud1 || !cloud2)
		{
			ccConsole::Error(QObject::tr("Select 2 entities (clouds or meshes)!"));
			return false;
		}
		
		if (!cloud1->hasScalarFields() && !cloud2->hasScalarFields())
		{
			ccConsole::Error(QObject::tr("None of the selected entities has per-point or per-vertex colors!"));
			return false;
		}
		else if (cloud1->hasScalarFields() && cloud2->hasScalarFields())
		{
			//ask the user to chose which will be the 'source' cloud
			ccOrderChoiceDlg ocDlg(cloud1, QObject::tr("Source"), cloud2, QObject::tr("Destination"), app);
			if (!ocDlg.exec())
			{
				//process cancelled by the user
				return false;
			}
			if (cloud1 != ocDlg.getFirstEntity())
			{
				std::swap(cloud1, cloud2);
			}
		}
		else if (cloud2->hasScalarFields())
		{
			std::swap(cloud1, cloud2);
		}
		
		ccPointCloud* source = cloud1;
		ccPointCloud* dest = cloud2;

		//show the list of scalar fields available on the source point cloud
		std::vector<int> sfIndexes;
		try
		{
			unsigned sfCount = source->getNumberOfScalarFields();
			if (sfCount == 1)
			{
				sfIndexes.push_back(0);
			}
			else if (sfCount > 1)
			{
				ccItemSelectionDlg isDlg(true, app->getMainWindow(), QObject::tr("entity"));
				QStringList scalarFields;
				{
					for (unsigned i = 0; i < sfCount; ++i)
					{
						scalarFields << QString::fromStdString(source->getScalarFieldName(i));
					}
				}
				isDlg.setItems(scalarFields, 0);
				if (!isDlg.exec())
				{
					//cancelled by the user
					return false;
				}
				isDlg.getSelectedIndexes(sfIndexes);
				if (sfIndexes.empty())
				{
					ccConsole::Error(QObject::tr("No scalar field was selected"));
					return false;
				}
			}
			else
			{
				assert(false);
			}
		}
		catch (const std::bad_alloc&)
		{
			ccConsole::Error(QObject::tr("Not enough memory!"));
			return false;
		}

		//semi-persistent parameters
		static ccPointCloudInterpolator::Parameters::Method s_interpMethod = ccPointCloudInterpolator::Parameters::RADIUS;
		static ccPointCloudInterpolator::Parameters::Algo s_interpAlgo = ccPointCloudInterpolator::Parameters::NORMAL_DIST;
		static int s_interpKNN = 6;

		ccInterpolationDlg iDlg(app->getMainWindow());
		iDlg.setInterpolationMethod(s_interpMethod);
		iDlg.setInterpolationAlgorithm(s_interpAlgo);
		iDlg.knnSpinBox->setValue(s_interpKNN);
		iDlg.radiusDoubleSpinBox->setValue(dest->getOwnBB().getDiagNormd() / 100);

		if (!iDlg.exec())
		{
			//process cancelled by the user
			return false;
		}

		//setup parameters
		ccPointCloudInterpolator::Parameters params;
		params.method = s_interpMethod = iDlg.getInterpolationMethod();
		params.algo = s_interpAlgo = iDlg.getInterpolationAlgorithm();
		params.knn = s_interpKNN = iDlg.knnSpinBox->value();
		params.radius = iDlg.radiusDoubleSpinBox->value();
		params.sigma = iDlg.kernelDoubleSpinBox->value();

		ccProgressDialog pDlg(true, app->getMainWindow());
		unsigned sfCountBefore = dest->getNumberOfScalarFields();

		if (ccPointCloudInterpolator::InterpolateScalarFieldsFrom(dest, source, sfIndexes, params, &pDlg))
		{
			dest->setCurrentDisplayedScalarField(static_cast<int>(std::min(sfCountBefore + 1, dest->getNumberOfScalarFields())) - 1);
			dest->showSF(true);
		}
		else
		{
			ccConsole::Error(QObject::tr("An error occurred! (see console)"));
		}
		
		dest->prepareDisplayForRefresh_recursive();
		
		return true;
	}
	
    //! Interpolate scalar fields from one entity and transfer them to another one without the dialog
    bool	interpolateSFs(ccPointCloud *source, ccPointCloud *dest, int sfIndex, ccPointCloudInterpolator::Parameters& params, QWidget* parent/*=nullptr*/)
    {
        if (!source || !dest)
        {
            ccConsole::Error(QObject::tr("Unexpected null cloud pointers!"));
            return false;
        }

        if (!source->hasScalarFields())
        {
            ccConsole::Error(QObject::tr("[ccEntityAction::interpolateSFs] The source cloud has no scalar field!"));
            return false;
        }

        unsigned sfCount = source->getNumberOfScalarFields();
        if (sfIndex > static_cast<int>(sfCount))
        {
            ccConsole::Error(QObject::tr("[ccEntityAction::interpolateSFs] Invalid scalar field index!"));
            return false;
        }


        ccProgressDialog pDlg(true, parent);

		std::vector<int> sfIndexes({ sfIndex });
		if (!ccPointCloudInterpolator::InterpolateScalarFieldsFrom(dest, source, sfIndexes, params, parent ? &pDlg : nullptr))
		{
			ccConsole::Error(QObject::tr("[ccEntityAction::interpolateSFs] An error occurred! (see console)"));
			return false;
		}
        
		return true;
    }

	bool	convertTextureToColor(const ccHObject::Container& selectedEntities, QWidget* parent/*=nullptr*/)
	{	
		for (ccHObject* ent : selectedEntities)
		{
			if (ent->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
				Q_ASSERT(mesh);
				
				if (!mesh->hasMaterials())
				{
					ccLog::Warning(QObject::tr("[ConvertTextureToColor] Mesh '%1' has no material/texture!").arg(mesh->getName()));
					continue;
				}
				else
				{
					if (	mesh->hasColors()
						&&	QMessageBox::warning(	parent,
													QObject::tr("Mesh already has colors"),
													QObject::tr("Mesh '%1' already has colors! Overwrite them?").arg(mesh->getName()),
													QMessageBox::Yes | QMessageBox::No,
													QMessageBox::No) != QMessageBox::Yes)
					{
						continue;
					}
					
					if (mesh->convertMaterialsToVertexColors())
					{
						mesh->showColors(true);
						mesh->showSF(false); //just in case
						mesh->showMaterials(false);
						mesh->prepareDisplayForRefresh_recursive();
					}
					else
					{
						ccLog::Warning(QObject::tr("[ConvertTextureToColor] Failed to convert texture on mesh '%1'!").arg(mesh->getName()));
					}
				}
			}
		}
		
		return true;
	}

	bool	enhanceRGBWithIntensities(const ccHObject::Container& selectedEntities, QWidget* parent/*=nullptr*/)
	{
		QString defaultSFName("Intensity");

		bool useCustomIntensityRange = false;
		static double s_minI = 0.0;
		static double s_maxI = 1.0;
		if (QMessageBox::question(parent, QObject::tr("Intensity range"), QObject::tr("Do you want to define the theoretical intensity range (yes)\nor use the actual one (no)?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
		{
			ccAskTwoDoubleValuesDlg atdvDlg(QObject::tr("Min"), QObject::tr("Max"), -1000000.0, 1000000.0, s_minI, s_maxI, 3, QObject::tr("Theroetical intensity"), parent);
			if (!atdvDlg.exec())
			{
				//process cancelled by the user
				return false;
			}
			s_minI = atdvDlg.doubleSpinBox1->value();
			s_maxI = atdvDlg.doubleSpinBox2->value();
			useCustomIntensityRange = true;
		}

		for (ccHObject* ent : selectedEntities)
		{
			bool lockedVertices = false;
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
			if (!pc || lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}

			if (!pc->hasColors())
			{
				ccLog::Warning(QObject::tr("[EnhanceRGBWithIntensities] Entity '%1' has no RGB color!").arg(ent->getName()));
				continue;
			}
			if (!pc->hasScalarFields())
			{
				ccLog::Warning(QObject::tr("[EnhanceRGBWithIntensities] Entity '%1' has no scalar field!").arg(ent->getName()));
				continue;
			}

			int sfIdx = -1;
			if (pc->getNumberOfScalarFields() > 1)
			{
				//does the previously selected SF works?
				if (!defaultSFName.isEmpty())
				{
					//if it's valid, we'll keep this SF!
					sfIdx = pc->getScalarFieldIndexByName(defaultSFName.toStdString());
				}
				if (sfIdx < 0)
				{
					//let the user choose the right scalar field
					ccPickOneElementDlg poeDlg(QObject::tr("Intensity scalar field"), QObject::tr("Choose scalar field"), parent);
					for (unsigned i = 0; i < pc->getNumberOfScalarFields(); ++i)
					{
						CCCoreLib::ScalarField* sf = pc->getScalarField(i);
						assert(sf);
						QString sfName = QString::fromStdString(sf->getName());
						poeDlg.addElement(sfName);
						if (sfIdx < 0 && sfName.contains("intensity", Qt::CaseInsensitive))
						{
							sfIdx = static_cast<int>(i);
						}
					}

					poeDlg.setDefaultIndex(std::max(0, sfIdx));
					if (!poeDlg.exec())
					{
						//process cancelled by the user
						return false;
					}
					sfIdx = poeDlg.getSelectedIndex();
					defaultSFName = QString::fromStdString(pc->getScalarField(sfIdx)->getName());
				}
			}
			else
			{
				sfIdx = 0;
			}
			assert(sfIdx >= 0);

			if (pc->enhanceRGBWithIntensitySF(sfIdx, useCustomIntensityRange, s_minI, s_maxI))
			{
				ent->prepareDisplayForRefresh();
				ent->showColors(true);
				ent->showSF(false);
			}
			else
			{
				ccLog::Warning(QObject::tr("[EnhanceRGBWithIntensities] Failed to apply the process on entity '%1'!").arg(ent->getName()));
			}
		}

		return true;
	}

	bool	rgbGaussianFilter(const ccHObject::Container& selectedEntities, ccPointCloud::RgbFilterOptions filterParams, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.empty())
		{
			return false;
		}

		// select only the clouds (or vertices) with RGB colors
		std::vector<std::pair<ccHObject*, ccPointCloud*>> selectedCloudsWithColors;
		double spatialSigma = std::numeric_limits<double>::max();

		for (ccHObject* ent : selectedEntities)
		{
			bool lockedVertices = false;
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
			if (!pc || lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}

			//check if the cloud has color
			if (pc->hasColors())
			{
				if ((filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL) && !pc->hasDisplayedScalarField())
				{
					continue;
				}

				selectedCloudsWithColors.push_back({ ent, pc });

				double sigmaCloud = ccLibAlgorithms::GetDefaultCloudKernelSize(pc);

				//we keep the smallest value
				if (sigmaCloud < spatialSigma)
				{
					spatialSigma = sigmaCloud;
				}
			}
		}

		if (filterParams.spatialSigma > 0)
		{
			spatialSigma = filterParams.spatialSigma;
		}

		if (selectedCloudsWithColors.empty())
		{
			if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
				ccConsole::Error(QObject::tr("Select at least one cloud or mesh with RGB colors and an active scalar field"));
			else
				ccConsole::Error(QObject::tr("Select at least one cloud or mesh with RGB colors"));
			return false;
		}

		double sigmaSF = -1.0;
		if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
		{
			CCCoreLib::ScalarField* sf = selectedCloudsWithColors.front().second->getCurrentDisplayedScalarField();
			if (sf)
			{
				ScalarType sfRange = sf->getMax() - sf->getMin();
				sigmaSF = sfRange / 4; // using 1/4 of total range
			}
			if (filterParams.sigmaSF > 0)
			{
				sigmaSF = filterParams.sigmaSF;
			}
		}

		QScopedPointer<ccProgressDialog> pDlg;
		if (!filterParams.commandLine)
		{
			bool ok = false;

			if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
			{
				ccAskThreeDoubleValuesDlg dlg(	QObject::tr("Spatial sigma"),
												QObject::tr("Scalar sigma"),
												QObject::tr("Color threshold"),
											DBL_MIN,
											1.0e9,
											spatialSigma,
											sigmaSF,
											filterParams.burntOutColorThreshold,
											8,
											nullptr,
											parent);

				dlg.setWindowTitle(QObject::tr("RGB bilateral filter"));

				dlg.doubleSpinBox1->setStatusTip(QObject::tr("3*sigma = 99.7% attenuation"));
				dlg.doubleSpinBox2->setStatusTip(QObject::tr("Scalar sigma controls how much the filter behaves as a Gaussian Filter\nSigma at +inf uses the whole range of scalars"));
				dlg.doubleSpinBox3->setStatusTip(QObject::tr("For averaging, it will only use colors for which all components are in the range[threshold:255 - threshold]"));
				if (!dlg.exec())
				{
					return false;
				}

				//get values
				spatialSigma = dlg.doubleSpinBox1->value();
				sigmaSF = dlg.doubleSpinBox2->value();
				filterParams.burntOutColorThreshold = dlg.doubleSpinBox3->value();
			}
			else
			{
				ccAskTwoDoubleValuesDlg dlg(QObject::tr("Spatial sigma"),
					QObject::tr("Color threshold"),
					DBL_MIN,
					1.0e9,
					spatialSigma,
					filterParams.burntOutColorThreshold,
					8,
					nullptr,
					parent);
				dlg.setWindowTitle(QObject::tr("RGB gaussian/mean/median filter"));

				dlg.doubleSpinBox1->setStatusTip(QObject::tr("3*sigma = 99.7% attenuation"));
				dlg.doubleSpinBox2->setStatusTip(QObject::tr("For averaging, it will only use colors for which all components are in the range [threshold:255-threshold]"));
				if (!dlg.exec())
				{
					return false;
				}

				//get values
				spatialSigma = dlg.doubleSpinBox1->value();
				filterParams.burntOutColorThreshold = dlg.doubleSpinBox2->value();
			}
		}

		if (parent)
		{
			pDlg.reset(new ccProgressDialog(true, parent));
			pDlg->setAutoClose(false);
		}

		for (auto entAndPC : selectedCloudsWithColors )
		{
			ccPointCloud* pc = entAndPC.second;
			assert(pc);
			int sfIdx = 0;
			if ((filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL) || filterParams.applyToSFduringRGB)
			{
				//we set the displayed SF as "OUT" SF
				int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
				Q_ASSERT(outSfIdx >= 0);

				pc->setCurrentOutScalarField(outSfIdx);

				if (filterParams.applyToSFduringRGB)
				{
					CCCoreLib::ScalarField* outSF = pc->getCurrentOutScalarField();
					Q_ASSERT(outSF != nullptr);
					QString sfName;
					if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
					{
						sfName = QString("%1.bilsmooth(%2,%3)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma).arg(sigmaSF);
					}
					else if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::GAUSSIAN)
					{
						sfName = QString("%1.smooth(%2)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma);
					}
					else if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::MEAN)
					{
						sfName = QString("%1.meansmooth(%2)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma);
					}
					else
					{
						sfName = QString("%1.medsmooth(%2)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma);
					}

					sfIdx = pc->getScalarFieldIndexByName(sfName.toStdString());
					if (sfIdx < 0)
						sfIdx = pc->addScalarField(sfName.toStdString()); //output SF has same type as input SF
					if (sfIdx >= 0)
						pc->setCurrentInScalarField(sfIdx);
					else
					{
						ccConsole::Error(QObject::tr("Failed to create scalar field for cloud '%1' (not enough memory?)").arg(pc->getName()));
						return false;
					}
				}

			}

			ccOctree::Shared octree = pc->getOctree();
			if (!octree)
			{
				octree = pc->computeOctree(parent ? pDlg.data() : nullptr);
				if (!octree)
				{
					ccConsole::Error(QObject::tr("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
					continue;
				}
			}

			QElapsedTimer eTimer;
			eTimer.start();
			pc->applyFilterToRGB(	static_cast<PointCoordinateType>(spatialSigma),
											static_cast<PointCoordinateType>(sigmaSF),
											filterParams,
											parent ? pDlg.data() : nullptr);
			ccConsole::Print("[RGBFilter] Timing: %3.2f s.", eTimer.elapsed() / 1000.0);

			if (filterParams.applyToSFduringRGB)
			{
				//calc sf min/max for correct display.
				pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(sfIdx >= 0);
				CCCoreLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
				if (sf)
					sf->computeMinAndMax();
			}
			// automatically hide any SF and show the colors instead
			entAndPC.first->prepareDisplayForRefresh_recursive();
			entAndPC.first->showColors(true);
			entAndPC.first->showSF(false);
		}

		return true;
	}

	//////////
	// Scalar Fields

	bool	sfGaussianFilter(const ccHObject::Container& selectedEntities, ccPointCloud::RgbFilterOptions filterParams, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.empty())
			return false;
		
		double spatialSigma = filterParams.spatialSigma == -1 ? ccLibAlgorithms::GetDefaultCloudKernelSize(selectedEntities) : filterParams.spatialSigma;
		if (spatialSigma < 0.0)
		{
			ccConsole::Error(QObject::tr("No eligible point cloud in selection!"));
			return false;
		}
		
		//estimate a good value for scalar field sigma, based on the first cloud
		//and its displayed scalar field
		double scalarFieldSigma = -1.0;
		if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
		{
			ccPointCloud* testPC = ccHObjectCaster::ToPointCloud(selectedEntities.front());
			CCCoreLib::ScalarField* testSF = testPC->getCurrentDisplayedScalarField();
			if (!testSF)
			{
				ccConsole::Error(QObject::tr("No active scalar field"));
				return false;
			}
			if (filterParams.sigmaSF == -1)
			{
				ScalarType range = testSF->getMax() - testSF->getMin();
				scalarFieldSigma = range / 4; // using 1/4 of total range
			}
			else
			{
				scalarFieldSigma = filterParams.sigmaSF;
			}
		}

		QScopedPointer<ccProgressDialog> pDlg;
		if (!filterParams.commandLine)
		{
			if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
			{
				ccAskTwoDoubleValuesDlg dlg(QObject::tr("Spatial sigma"),
											QObject::tr("Scalar sigma"),
											DBL_MIN,
											1.0e9,
											spatialSigma,
											scalarFieldSigma,
											8,
											nullptr,
											parent);

				dlg.setWindowTitle(QObject::tr("SF bilateral filter"));

				dlg.doubleSpinBox1->setStatusTip(QObject::tr("3*sigma = 99.7% attenuation"));
				dlg.doubleSpinBox2->setStatusTip(QObject::tr("Scalar field's sigma controls how much the filter behaves as a Gaussian Filter\nSigma at +inf uses the whole range of scalars"));
				if (!dlg.exec())
					return false;

				//get values
				spatialSigma = dlg.doubleSpinBox1->value();
				scalarFieldSigma = dlg.doubleSpinBox2->value();
			}
			else
			{
				bool ok = false;

				spatialSigma = QInputDialog::getDouble(	parent,
														QObject::tr("SF gaussian/mean/median filter"),
														"sigma:",
														spatialSigma,
														DBL_MIN,
														1.0e9,
														8,
														&ok);
				if (!ok)
				{
					return false;
				}
			}
		}

		if (parent)
		{
			pDlg.reset(new ccProgressDialog(true, parent));
			pDlg->setAutoClose(false);
		}

		for (ccHObject* ent : selectedEntities)
		{
			bool lockedVertices = false;
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
			if (!pc || lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}
			
			//the algorithm will use the currently displayed SF
			CCCoreLib::ScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf)
			{
				//we set the displayed SF as "OUT" SF
				int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
				Q_ASSERT(outSfIdx >= 0);
				
				pc->setCurrentOutScalarField(outSfIdx);
				CCCoreLib::ScalarField* outSF = pc->getCurrentOutScalarField();
				Q_ASSERT(outSF != nullptr);
				
				QString sfName;
				if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::BILATERAL)
				{
					sfName = QString("%1.bilsmooth(%2,%3)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma).arg(scalarFieldSigma);
				}
				else if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::GAUSSIAN)
				{
					sfName = QString("%1.smooth(%2)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma);
				}
				else if (filterParams.filterType == ccPointCloud::RGB_FILTER_TYPES::MEAN)
				{
					sfName = QString("%1.meansmooth(%2)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma);
				}
				else
				{
					sfName = QString("%1.medsmooth(%2)").arg(QString::fromStdString(outSF->getName())).arg(spatialSigma);
				}

				int sfIdx = pc->getScalarFieldIndexByName(sfName.toStdString());
				if (sfIdx < 0)
					sfIdx = pc->addScalarField(sfName.toStdString()); //output SF has same type as input SF
				if (sfIdx >= 0)
					pc->setCurrentInScalarField(sfIdx);
				else
				{
					ccConsole::Error(QObject::tr("Failed to create scalar field for cloud '%1' (not enough memory?)").arg(pc->getName()));
					return false;
				}
				
				ccOctree::Shared octree = pc->getOctree();
				if (!octree)
				{
					octree = pc->computeOctree(parent ? pDlg.data() : nullptr);
					if (!octree)
					{
						ccConsole::Error(QObject::tr("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
						return false;
					}
				}
				
				QElapsedTimer eTimer;
				eTimer.start();
				
				if (!CCCoreLib::ScalarFieldTools::applyScalarFieldGaussianFilter(	static_cast<PointCoordinateType>(spatialSigma),
																					pc,
																					static_cast<PointCoordinateType>(scalarFieldSigma),
																					parent ? pDlg.data() : nullptr,
																					octree.data()))
				{
					ccConsole::Warning(QObject::tr("[Bilateral/Gaussian/Mean/Median filter]  Failed to apply filter"));
					return false;
				}
				
				ccConsole::Print("SF [Bilateral/Gaussian/Mean/Median filter] Timing: %3.2f s.", eTimer.elapsed() / 1000.0);
				pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(sfIdx >= 0);
				sf = pc->getCurrentDisplayedScalarField();
				if (sf)
					sf->computeMinAndMax();
				pc->prepareDisplayForRefresh_recursive();
			}
			else
			{
				ccConsole::Warning(QObject::tr("Entity [%1] has no active scalar field!").arg(pc->getName()));
			}
		}
		
		return true;
	}
	
	bool	sfConvertToRGB(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		//we first ask the user if the SF colors should be mixed with existing colors
		bool mixWithExistingColors = false;
		
		QMessageBox::StandardButton answer = QMessageBox::warning(	parent,
																	QObject::tr("Scalar Field to RGB"),
																	QObject::tr("Mix with existing colors (if any)?"),
																	QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
																	QMessageBox::Yes );
		if (answer == QMessageBox::Yes)
			mixWithExistingColors = true;
		else if (answer == QMessageBox::Cancel)
			return false;
		
		for (ccHObject* ent : selectedEntities)
		{
			ccGenericPointCloud* cloud = nullptr;
			
			bool lockedVertices = false;
			cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}
			if (cloud != nullptr) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				//if there is no displayed SF --> nothing to do!
				if (pc->getCurrentDisplayedScalarField())
				{
					if (pc->convertCurrentScalarFieldToColors(mixWithExistingColors))
					{
						ent->showColors(true);
						ent->showSF(false); //just in case
					}
				}
				
				cloud->prepareDisplayForRefresh_recursive();
			}
		}
		
		return true;
	}
	
	bool	sfConvertToRandomRGB(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		static int s_randomColorsNumber = 256;
		
		bool ok = false;
		s_randomColorsNumber = QInputDialog::getInt(parent,
													QObject::tr("Random colors"),
													QObject::tr("Number of random colors (will be regularly sampled over the SF interval):"),
													s_randomColorsNumber,
													2,
													INT_MAX,
													16,
													&ok);
		if (!ok)
			return false;
		Q_ASSERT(s_randomColorsNumber > 1);
		
		RGBAColorsTableType* randomColors = new RGBAColorsTableType;
		if (!randomColors->reserveSafe(static_cast<unsigned>(s_randomColorsNumber)))
		{
			ccConsole::Error(QObject::tr("Not enough memory!"));
			return false;
		}
		
		//generate random colors
		for (int i = 0; i < s_randomColorsNumber; ++i)
		{
			ccColor::Rgba col(ccColor::Generator::Random(), ccColor::MAX);
			randomColors->addElement(col);
		}
		
		//apply random colors
		for (ccHObject* ent : selectedEntities)
		{
			ccGenericPointCloud* cloud = nullptr;
			
			bool lockedVertices = false;
			cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
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
						ccConsole::Error(QObject::tr("Not enough memory!"));
						break;
					}
					else
					{
						ScalarType minSF = sf->getMin();
						ScalarType maxSF = sf->getMax();
						
						ScalarType step = (maxSF - minSF) / (s_randomColorsNumber - 1);
						if (step == 0)
							step = static_cast<ScalarType>(1.0);
						
						for (unsigned i = 0; i < pc->size(); ++i)
						{
							ScalarType val = sf->getValue(i);
							unsigned colIndex = static_cast<unsigned>((val - minSF) / step);
							if (colIndex == s_randomColorsNumber)
								--colIndex;
							
							pc->setPointColor(i, randomColors->getValue(colIndex));
						}
						
						pc->showColors(true);
						pc->showSF(false); //just in case
					}
				}
				
				cloud->prepareDisplayForRefresh_recursive();
			}
		}
		
		return true;
	}
	
	bool sfRename(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		for (ccHObject* ent : selectedEntities)
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (cloud != nullptr) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				ccScalarField* sf = pc->getCurrentDisplayedScalarField();
				//if there is no displayed SF --> nothing to do!
				if (sf == nullptr)
				{
					ccConsole::Warning(QObject::tr("Cloud %1 has no displayed scalar field!").arg(pc->getName()));
				}
				else
				{
					const std::string& sfName = sf->getName();
					bool ok = false;
					QString newName = QInputDialog::getText(parent,
															QObject::tr("SF name"),
															QObject::tr("name:"),
															QLineEdit::Normal,
															QString(!sfName.empty() ? QString::fromStdString(sfName) : QObject::tr("unknown")),
															&ok);
					if (ok)
						sf->setName(newName.toStdString());
				}
			}
		}
		
		return true;
	}
	
	bool	sfAddIdField(const ccHObject::Container& selectedEntities, bool storeAsInt)
	{
		for (ccHObject* ent : selectedEntities)
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (nullptr != cloud) //TODO
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				
				int sfIdx = pc->getScalarFieldIndexByName(CC_DEFAULT_ID_SF_NAME);
				if (sfIdx < 0)
					sfIdx = pc->addScalarField(CC_DEFAULT_ID_SF_NAME);
				if (sfIdx < 0)
				{
					ccLog::Warning(QObject::tr("Not enough memory!"));
					return false;
				}
				
				CCCoreLib::ScalarField* sf = pc->getScalarField(sfIdx);
				Q_ASSERT(sf->currentSize() == pc->size());

				for (unsigned j = 0; j < cloud->size(); j++)
				{
					ScalarType idValue = 0;
					if (!storeAsInt)
					{
						idValue = static_cast<ScalarType>(j);
					}
					else
					{
						uint8_t* valuePtr = reinterpret_cast<uint8_t*>(&idValue);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
						valuePtr[0] = static_cast<unsigned char>(j & 0xff);
						valuePtr[1] = static_cast<unsigned char>((j >> 8) & 0xff);
						valuePtr[2] = static_cast<unsigned char>((j >> 16) & 0xff);
						valuePtr[3] = static_cast<unsigned char>((j >> 24) & 0xff);
#else
						unsigned char *valuePtr = (unsigned char *)(&idValue);
						valuePtr[3] = static_cast<unsigned char>(j & 0xff);
						valuePtr[2] = static_cast<unsigned char>((j >> 8) & 0xff);
						valuePtr[1] = static_cast<unsigned char>((j >> 16) & 0xff);
						valuePtr[0] = static_cast<unsigned char>((j >> 24) & 0xff);
#endif
					}
					sf->setValue(j, idValue);
				}
				
				sf->computeMinAndMax();
				pc->setCurrentDisplayedScalarField(sfIdx);
				pc->showSF(true);
				pc->prepareDisplayForRefresh();
			}
		}
		
		return true;
	}
	
	bool sfAddConstant(ccPointCloud* cloud, QString sfName, bool integerValue, QWidget* parent/*=nullptr*/)
	{
		if (!cloud)
		{
			assert(false);
			return false;
		}

		if (sfName.isNull())
		{
			ccLog::Error(QT_TR_NOOP("Invalid name"));
			return false;
		}

		if (cloud->getScalarFieldIndexByName(sfName.toStdString()) >= 0)
		{
			ccLog::Error(QT_TR_NOOP("A SF with a similar name already exists!"));
			return false;
		}

		ScalarType sfValue = 0;
		bool ok = false;
		if (integerValue)
		{
			static int s_constantIntSFValue = 0;
			int iValue = QInputDialog::getInt(parent, QT_TR_NOOP("Add classification SF"), QT_TR_NOOP("value"), s_constantIntSFValue, -1000000, 1000000, 1, &ok);
			if (ok)
			{
				s_constantIntSFValue = iValue;
				sfValue = static_cast<ScalarType>(iValue);
			}
		}
		else
		{
			static double s_constantDoubleSFValue = 0.0;
			double dValue = static_cast<ScalarType>(QInputDialog::getDouble(parent, QT_TR_NOOP("Add constant value"), QT_TR_NOOP("value"), s_constantDoubleSFValue, -1.0e9, 1.0e9, 8, &ok));
			if (ok)
			{
				s_constantDoubleSFValue = dValue;
				sfValue = static_cast<ScalarType>(dValue);
			}
		}

		if (!ok)
		{
			// cancelled by the user
			return false;
		}

		int sfIdx = cloud->getScalarFieldIndexByName(sfName.toStdString());
		if (sfIdx < 0)
			sfIdx = cloud->addScalarField(sfName.toStdString());
		if (sfIdx < 0)
		{
			ccLog::Error(QT_TR_NOOP("An error occurred! (see console)"));
			return false;
		}

		CCCoreLib::ScalarField* sf = cloud->getScalarField(sfIdx);
		assert(sf);
		if (!sf)
		{
			assert(false);
			return false;
		}
		
		sf->fill(sfValue);
		sf->computeMinAndMax();
		cloud->setCurrentDisplayedScalarField(sfIdx);
		cloud->showSF(true);

		ccLog::Print(QObject::tr("New scalar field '%1' added to %2 (value = %3)").arg(sfName).arg(cloud->getName()).arg(sfValue));

		return true;
	}

    bool sfSplitCloud(const ccHObject::Container& selectedEntities, ccMainAppInterface* app)
    {
		bool tooManyCloudsQuestionAsked = false;

        for (ccHObject* ent : selectedEntities)
        {
            ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (!cloud)
			{
				continue;
			}
            
			ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
			if (sf == nullptr)
			{
				ccLog::Warning(QString("Cloud %1 has no active scalar field").arg(cloud->getName()));
				return false;
			}

            // count integer values
            size_t N = sf->size();
            std::set<int> classes;
            for (size_t i = 0; i < sf->size(); ++i)
            {
				classes.insert(static_cast<int>(sf->getValue(i)));
            }
            ccLog::Print("[sfSplitCloud] " + QString::number(classes.size()) + " classe(s) found in the current scalar field");

			if (classes.size() == 1)
			{
				ccLog::Warning(QString("[sfSplitCloud] Cloud %1: SF has only one value").arg(cloud->getName()));
				continue;
			}

			if (classes.size() > 30 && app && app->getMainWindow() && !tooManyCloudsQuestionAsked) // only in GUI mode
			{
				// ask the user if creating this many clouds is expected
				if (QMessageBox::No == QMessageBox::question(app->getMainWindow(), "Too many values", QString("Do you confirm that you want to create %1 clouds").arg(classes.size()), QMessageBox::Yes, QMessageBox::No))
				{
					ccLog::Warning("[sfSplitCloud] Process cancelled by the user");
					return false;
				}
				tooManyCloudsQuestionAsked = true;
			}

			ccHObject* destObject = new ccHObject(cloud->getName() + " classes");
			if (cloud->getParent())
			{
				cloud->getParent()->addChild(destObject);
			}
            
			// create as many clouds as the number of classes
			for (int pointClass : classes)
            {
                ccLog::Print("[sfSplitCloud] build cloud corresponding to class #" + QString::number(pointClass));
                
				try
				{
					// create the reference cloud
					CCCoreLib::ReferenceCloud referenceCloud(cloud);
					
					// populate the cloud with the points which have the selected class
					for (unsigned index = 0; index < static_cast<unsigned>(cloud->size()); index++)
					{
						if (static_cast<int>(sf->getValue(index)) == pointClass)
						{
							referenceCloud.addPointIndex(index);
						}
					}
					ccPointCloud* pc = cloud->partialClone(&referenceCloud);
					if (pc)
					{
						pc->setName("class #" + QString::number(pointClass));
						destObject->addChild(pc);
					}
					else
					{
						ccLog::Warning("[sfSplitCloud] Failed to create cloud");
					}
				}
				catch (const std::bad_alloc&)
				{
					delete destObject;
					destObject = nullptr;
					ccLog::Error(QT_TR_NOOP("Not enough memory"));
					return false;
				}
			}
		
			// add to database
			app->addToDB(destObject);

			cloud->setEnabled(false);
			cloud->prepareDisplayForRefresh();
		}

        return true;
    }

	static void SetValueFromSF(PointCoordinateType& value, int fieldIndex, CCCoreLib::ScalarField* sf, unsigned pointIndex, PointCoordinateType defaultValueForNaN)
	{
		if (sf)
		{
			ScalarType s = sf->getValue(pointIndex);
			if (CCCoreLib::ScalarField::ValidValue(s))
			{
				value = s;
			}
			else
			{
				value = defaultValueForNaN;
			}
		}
		else
		{
			switch (fieldIndex)
			{
			case ccSetSFsAsVec3Dialog::SF_INDEX_ZERO:
				value = 0;
				break;
			case ccSetSFsAsVec3Dialog::SF_INDEX_ONE:
				value = static_cast<ScalarType>(1);
				break;
			case ccSetSFsAsVec3Dialog::SF_INDEX_UNCHANGED:
				// nothing to do
				break;
			case ccSetSFsAsVec3Dialog::SF_INDEX_NO:
			default:
				assert(false);
				value = defaultValueForNaN;
				break;
			}
		}
	}

	static PointCoordinateType GetDefaultValueForNaN(PointCoordinateType minSFValue, QWidget* parent)
	{
		bool ok = false;
		double out = QInputDialog::getDouble(	parent,
												QObject::tr("SF --> coordinate"),
												QObject::tr("Enter the coordinate equivalent to NaN values:"),
												minSFValue,
												-1.0e9,
												1.0e9,
												6,
												&ok);

		if (ok)
		{
			return static_cast<PointCoordinateType>(out);
		}
		else
		{
			ccLog::Warning(QObject::tr("[SetSFAsCoord] By default the coordinate equivalent to NaN values will be the minimum SF value"));
			return minSFValue;
		}
	}

	bool	sfSetAsCoord(ccHObject* entity, QWidget* parent/*=nullptr*/)
	{
		if (!entity)
		{
			assert(false);
			return false;
		}

		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (!cloud || !cloud->isA(CC_TYPES::POINT_CLOUD))
		{
			assert(false);
			ccLog::Warning("[sfSetAsCoord] Expecting a cloud or a mesh");
			return false;
		}

		ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

		ccSetSFsAsVec3Dialog dlg(pc, "X", "Y", "Z", true, parent);
		dlg.setWindowTitle(QObject::tr("Set SFs as coords"));

		static bool s_firstTime = true;
		static int xIndex = ccSetSFsAsVec3Dialog::SF_INDEX_UNCHANGED;
		static int yIndex = ccSetSFsAsVec3Dialog::SF_INDEX_UNCHANGED;
		static int zIndex = ccSetSFsAsVec3Dialog::SF_INDEX_UNCHANGED;

		// restore the previous parameters
		dlg.setSFIndexes(xIndex, yIndex, zIndex);

		if (!dlg.exec())
		{
			return false;
		}

		dlg.getSFIndexes(xIndex, yIndex, zIndex);

		CCCoreLib::ScalarField* sfX = (xIndex >= 0 ? pc->getScalarField(xIndex) : nullptr);
		CCCoreLib::ScalarField* sfY = (yIndex >= 0 ? pc->getScalarField(yIndex) : nullptr);
		CCCoreLib::ScalarField* sfZ = (zIndex >= 0 ? pc->getScalarField(zIndex) : nullptr);

		std::array<CCCoreLib::ScalarField*, 3> scalarFields{ sfX, sfY, sfZ };

		PointCoordinateType defaultCoordForNaN = std::numeric_limits<PointCoordinateType>::quiet_NaN();
		for (CCCoreLib::ScalarField* sf : scalarFields)
		{
			if (sf)
			{
				if (sf->countValidValues() < sf->size())
				{
					//we have some invalid values, let's ask the user what they should be replaced with
					defaultCoordForNaN = GetDefaultValueForNaN(sf->getMin(), parent);
					break;
				}
			}
		}

		unsigned ptsCount = pc->size();
		for (unsigned i = 0; i < ptsCount; ++i)
		{
			const CCVector3* P = pc->getPoint(i);

			CCVector3 newP = *P;
			SetValueFromSF(newP.x, xIndex, sfX, i, defaultCoordForNaN);
			SetValueFromSF(newP.y, yIndex, sfY, i, defaultCoordForNaN);
			SetValueFromSF(newP.z, zIndex, sfZ, i, defaultCoordForNaN);

			*const_cast<CCVector3*>(P) = newP;
		}

		pc->invalidateBoundingBox();

		if (entity->isKindOf(CC_TYPES::MESH))
		{
			static_cast<ccGenericMesh*>(entity)->refreshBB();
		}

		entity->prepareDisplayForRefresh();

		return true;
	}

	bool	sfSetAsCoord(const ccHObject::Container& selectedEntities, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.size() == 1)
		{
			// shortcut for single entities, with a smarter dialog
			return sfSetAsCoord(selectedEntities.front(), parent);
		}

		ccExportCoordToSFDlg ectsDlg(parent);
		ectsDlg.warningLabel->setVisible(false);
		ectsDlg.setWindowTitle(QObject::tr("Export SF to coordinate(s)"));
		
		if (!ectsDlg.exec())
		{
			return false;
		}
		
		bool importDim[3] { ectsDlg.exportX(), ectsDlg.exportY(), ectsDlg.exportZ() };
		if (!importDim[0] && !importDim[1] && !importDim[2]) //nothing to do?!
		{
			return false;
		}
		
		ScalarType defaultValueForNaN = CCCoreLib::NAN_VALUE;

		//for each selected cloud (or vertices set)
		for (ccHObject* ent : selectedEntities)
		{
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent);
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
				
				ccScalarField* sf = pc->getCurrentDisplayedScalarField();
				if (sf != nullptr)
				{
					if (std::isnan(defaultValueForNaN) && (sf->countValidValues() < sf->size()))
					{
						//we have some invalid values, let's ask the user what they should be replaced with
						defaultValueForNaN = GetDefaultValueForNaN(sf->getMin(), parent);
						break;
					}

					pc->setCoordFromSF(importDim, sf, defaultValueForNaN);
				}
			}

			if (ent->isKindOf(CC_TYPES::MESH))
			{
				static_cast<ccGenericMesh*>(ent)->refreshBB();
			}

		}
		
		return true;
	}

	bool	exportCoordToSF(const ccHObject::Container& selectedEntities, QWidget* parent/*=nullptr*/)
	{
		ccExportCoordToSFDlg ectsDlg(parent);

		if (!ectsDlg.exec())
		{
			return false;
		}

		bool exportDims[3] { ectsDlg.exportX(),
							 ectsDlg.exportY(),
							 ectsDlg.exportZ() };

		if (!exportDims[0] && !exportDims[1] && !exportDims[2]) //nothing to do?!
		{
			return false;
		}
		
		//for each selected cloud (or vertices set)
		for (ccHObject* entity : selectedEntities)
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entity);
			if (pc == nullptr)
			{
				// TODO do something with error?
				continue;
			}

			if (!pc->exportCoordToSF(exportDims))
			{
				ccLog::Error(QObject::tr("The process failed!"));
				return true; //true because we want the UI to be updated anyway
			}

			if (entity != pc)
			{
				entity->showSF(true); //for meshes
			}
			entity->prepareDisplayForRefresh_recursive();
		}
		
		return true;
	}

	bool	setSFsAsNormal(ccHObject* entity, QWidget* parent/*=nullptr*/)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud == nullptr)
		{
			// unhandled entity
			return false;
		}

		const bool cloudHadNormals = cloud->hasNormals();

		ccSetSFsAsVec3Dialog dlg(cloud, "Nx", "Ny", "Nz", cloudHadNormals, parent);
		dlg.setWindowTitle(QObject::tr("Set SFs as normals"));

		static bool s_firstTime = true;
		static int nxIndex = ccSetSFsAsVec3Dialog::SF_INDEX_ZERO;
		static int nyIndex = ccSetSFsAsVec3Dialog::SF_INDEX_ZERO;
		static int nzIndex = ccSetSFsAsVec3Dialog::SF_INDEX_ZERO;

		if (s_firstTime)
		{
			s_firstTime = false;
		}
		else
		{
			// restore the previous parameters
			dlg.setSFIndexes(nxIndex, nyIndex, nzIndex);
		}

		if (!dlg.exec())
		{
			return false;
		}

		dlg.getSFIndexes(nxIndex, nyIndex, nzIndex);

		if (!cloud->resizeTheNormsTable())
		{
			ccLog::Error("Not enough memory");
		}

		CCCoreLib::ScalarField* sfX = (nxIndex >= 0 ? cloud->getScalarField(nxIndex) : nullptr);
		CCCoreLib::ScalarField* sfY = (nyIndex >= 0 ? cloud->getScalarField(nyIndex) : nullptr);
		CCCoreLib::ScalarField* sfZ = (nzIndex >= 0 ? cloud->getScalarField(nzIndex) : nullptr);

		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			CCVector3f N(0, 0, 0);

			if (cloudHadNormals)
			{
				N = cloud->getPointNormal(i);
			}

			SetValueFromSF(N.x, nxIndex, sfX, i, 0);
			SetValueFromSF(N.y, nyIndex, sfY, i, 0);
			SetValueFromSF(N.z, nzIndex, sfZ, i, 0);

			N.normalize();
			cloud->setPointNormal(i, N);
		}

		cloud->showNormals(true);
		cloud->prepareDisplayForRefresh();

		if (entity != cloud)
		{
			entity->showNormals(true);
			entity->prepareDisplayForRefresh();

			if (entity->isKindOf(CC_TYPES::MESH))
			{
				static_cast<ccGenericMesh*>(entity)->showTriNorms(false); // hide the per-triangle normals (if any)
			}
		}

		return true;
	}

	bool	exportNormalToSF(const ccHObject::Container& selectedEntities, QWidget* parent/*=nullptr*/, bool* exportDimensions/*=nullptr*/)
	{
		bool exportDims[3] { false, false, false };

		if (exportDimensions)
		{
			exportDims[0] = exportDimensions[0];
			exportDims[1] = exportDimensions[1];
			exportDims[2] = exportDimensions[2];
		}
		else
		{
			//ask the user
			ccExportCoordToSFDlg ectsDlg(parent);
			ectsDlg.setWindowTitle(QObject::tr("Export normals to SF(s)"));

			if (!ectsDlg.exec())
			{
				return false;
			}

			exportDims[0] = ectsDlg.exportX();
			exportDims[1] = ectsDlg.exportY();
			exportDims[2] = ectsDlg.exportZ();
		}

		if (!exportDims[0] && !exportDims[1] && !exportDims[2]) //nothing to do?!
		{
			return false;
		}

		//for each selected cloud (or vertices set)
		for (ccHObject* entity : selectedEntities)
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entity);
			if (pc == nullptr)
			{
				// TODO do something with error?
				continue;
			}

			if (!pc->hasNormals())
			{
				ccLog::Warning(QObject::tr("Cloud '%1' has no normals").arg(pc->getName()));
				continue;
			}

			if (!pc->exportNormalToSF(exportDims))
			{
				ccLog::Error(QObject::tr("The process failed!"));
				return true; //true because we want the UI to be updated anyway
			}

			if (entity != pc)
			{
				entity->showSF(true); //for meshes
			}
			entity->prepareDisplayForRefresh_recursive();
		}

		return true;
	}

	bool	sfArithmetic(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		Q_ASSERT(!selectedEntities.empty());
		
		ccHObject* entity = selectedEntities[0];
		bool lockedVertices;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity, &lockedVertices);
		if (lockedVertices)
		{
			ccUtils::DisplayLockedVerticesWarning(entity->getName(), true);
			return false;
		}
		if (cloud == nullptr)
		{
			return false;
		}
		
		ccScalarFieldArithmeticsDlg sfaDlg(cloud, parent);
		
		if (!sfaDlg.exec())
		{
			return false;
		}
		
		if (!sfaDlg.apply(cloud))
		{
			ccConsole::Error(QObject::tr("An error occurred (see Console for more details)"));
		}
		
		cloud->showSF(true);
		cloud->prepareDisplayForRefresh_recursive();
		
		return true;
	}

	bool	sfFromColor(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		ccScalarFieldFromColorDlg dialog(parent);
		if (!dialog.exec())
			return false;

		const bool exportR = dialog.getRStatus();
		const bool exportG = dialog.getGStatus();
		const bool exportB = dialog.getBStatus();
		const bool exportAlpha = dialog.getAlphaStatus();
		const bool exportComposite = dialog.getCompositeStatus();

		return sfFromColor(selectedEntities, exportR, exportG, exportB, exportAlpha, exportComposite);
	}

	bool	sfFromColor(const ccHObject::Container &selectedEntities, bool exportR, bool exportG, bool exportB, bool exportAlpha, bool exportComposite)
	{
		//candidates
		std::unordered_set<ccPointCloud*> clouds;
		
		for (ccHObject* ent : selectedEntities)
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent);
			if (cloud && ent->hasColors()) //only for clouds (or vertices)
				clouds.insert( cloud );
		}
		
		if (clouds.empty())
			return false;
		
		
		for (const auto cloud : clouds)
		{
			std::vector<ccScalarField*> fields(5, nullptr);
			fields[0] = (exportR ? new ccScalarField(GetFirstAvailableSFName(cloud, "R").toStdString()) : nullptr);
			fields[1] = (exportG ? new ccScalarField(GetFirstAvailableSFName(cloud, "G").toStdString()) : nullptr);
			fields[2] = (exportB ? new ccScalarField(GetFirstAvailableSFName(cloud, "B").toStdString()) : nullptr);
			fields[3] = (exportAlpha ? new ccScalarField(GetFirstAvailableSFName(cloud, "Alpha").toStdString()) : nullptr);
			fields[4] = (exportComposite ? new ccScalarField(GetFirstAvailableSFName(cloud, "Composite").toStdString()) : nullptr);
			
			//try to instantiate memory for each field
			unsigned count = cloud->size();
			for (ccScalarField*& sf : fields)
			{
				if (sf && !sf->reserveSafe(count))
				{
					ccLog::Warning(QObject::tr("[SfFromColor] Not enough memory to instantiate SF '%1' on cloud '%2'").arg(QString::fromStdString(sf->getName()), cloud->getName()));
					sf->release();
					sf = nullptr;
				}
			}
			
			//export points
			for (unsigned j = 0; j < cloud->size(); ++j)
			{
				const ccColor::Rgba& col = cloud->getPointColor(j);
				
				if (fields[0])
					fields[0]->addElement(col.r);
				if (fields[1])
					fields[1]->addElement(col.g);
				if (fields[2])
					fields[2]->addElement(col.b);
				if (fields[3])
					fields[3]->addElement(col.a);
				if (fields[4])
					fields[4]->addElement(static_cast<ScalarType>(col.r + col.g + col.b) / 3);
			}
			
			QString fieldsStr;
			
			for (ccScalarField*& sf : fields)
			{
				if (sf == nullptr)
					continue;
				
				sf->computeMinAndMax();
				
				int sfIdx = cloud->getScalarFieldIndexByName(sf->getName());
				if (sfIdx >= 0)
					cloud->deleteScalarField(sfIdx);
				sfIdx = cloud->addScalarField(sf);
				Q_ASSERT(sfIdx >= 0);
				
				if (sfIdx >= 0)
				{
					cloud->setCurrentDisplayedScalarField(sfIdx);
					cloud->showSF(true);
					cloud->prepareDisplayForRefresh();
					
					//mesh vertices?
					if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH))
					{
						cloud->getParent()->showSF(true);
						cloud->getParent()->prepareDisplayForRefresh();
					}
					
					if (!fieldsStr.isEmpty())
					{
						fieldsStr.append(", ");
					}
					fieldsStr.append(QString::fromStdString(sf->getName()));
				}
				else
				{
					ccConsole::Warning(QObject::tr("[SfFromColor] Failed to add scalar field '%1' to cloud '%2'?!").arg(QString::fromStdString(sf->getName()), cloud->getName()));
					sf->release();
					sf = nullptr;
				}
			}
			
			if (!fieldsStr.isEmpty())
				ccLog::Print(QObject::tr("[SfFromColor] New scalar fields (%1) added to '%2'").arg(fieldsStr, cloud->getName()));
		}

		return true;
	}
	
	bool	processMeshSF(const ccHObject::Container &selectedEntities, ccMesh::MESH_SCALAR_FIELD_PROCESS process, QWidget* parent/*=nullptr*/)
	{
		for (ccHObject* ent : selectedEntities)
		{
			if (ent->isKindOf(CC_TYPES::MESH) || ent->isKindOf(CC_TYPES::PRIMITIVE)) //TODO
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
				if (mesh == nullptr)
					continue;
				
				ccGenericPointCloud* cloud = mesh->getAssociatedCloud();
				if (cloud == nullptr)
					continue;
				
				if (cloud->isA(CC_TYPES::POINT_CLOUD)) //TODO
				{
					ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
					
					//on active le champ scalaire actuellement affiche
					int sfIdx = pc->getCurrentDisplayedScalarFieldIndex();
					if (sfIdx >= 0)
					{
						pc->setCurrentScalarField(sfIdx);
						mesh->processScalarField(process);
						pc->getCurrentInScalarField()->computeMinAndMax();
						mesh->prepareDisplayForRefresh_recursive();
					}
					else
					{
						ccConsole::Warning(QObject::tr("Mesh [%1] vertices have no activated scalar field!").arg(mesh->getName()));
					}
				}
			}
		}
		
		return true;
	}
	
	//////////
	// Normals
	
	bool	computeNormals(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.empty())
		{
			ccConsole::Error(QObject::tr("Select at least one point cloud"));
			return false;
		}
		
		static const QString s_NormalScaleKey("Normal scale");

		//look for clouds and meshes
		std::vector<ccPointCloud*> clouds;
		bool withScanGrid = false;
		bool withSensor = false;
		std::vector<ccMesh*> meshes;
		PointCoordinateType defaultRadius = 0;
		
		try
		{
			for (const auto entity : selectedEntities)
			{
				if (entity->isA(CC_TYPES::POINT_CLOUD))
				{
					ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
					clouds.push_back(cloud);
					
					if (!withScanGrid)
					{
						withScanGrid = (cloud->gridCount() > 0);
					}

					if (!withSensor)
					{
						for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
						{
							if (cloud->hasSensor())
							{
								withSensor = true;
								break; //no need to look anyfurther
							}
						}
					}

					//does the cloud have a former radius value saved as meta-data?
					if (cloud->hasMetaData(s_NormalScaleKey))
					{
						bool ok = false;
						double formerRadius = cloud->getMetaData(s_NormalScaleKey).toDouble(&ok);
						if (ok)
						{
							//remember the largest radius
							defaultRadius = std::max(defaultRadius, static_cast<PointCoordinateType>(formerRadius));
						}
						else
						{
							assert(false);
						}
					}

					if (defaultRadius == 0.0)
					{
						//default radius
						defaultRadius = ccOctree::GuessNaiveRadius(cloud);
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
						ccConsole::Error(QObject::tr("Can't compute normals on sub-meshes! Select the parent mesh instead"));
						return false;
					}
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			ccConsole::Error(QObject::tr("Not enough memory!"));
			return false;
		}
		
		//compute normals for each selected cloud
		if (!clouds.empty())
		{
			static CCCoreLib::LOCAL_MODEL_TYPES s_lastModelType = CCCoreLib::LS;
			static ccNormalVectors::Orientation s_lastNormalOrientation = ccNormalVectors::UNDEFINED;
			static int s_lastMSTNeighborCount = 6;
			static double s_lastMinGridAngle_deg = 1.0;
			static bool s_orientNormals = true;
			
			ccNormalComputationDlg ncDlg(withScanGrid, withSensor, parent);
			ncDlg.setLocalModel(s_lastModelType);
			ncDlg.setRadius(defaultRadius);
			ncDlg.setPreferredOrientation(s_lastNormalOrientation);
			ncDlg.setOrientNormals(s_orientNormals);
			ncDlg.setMSTNeighborCount(s_lastMSTNeighborCount);
			ncDlg.setMinGridAngle_deg(s_lastMinGridAngle_deg);
			if (clouds.size() == 1)
			{
				ncDlg.setCloud(clouds.front());
			}
			
			if (!ncDlg.exec())
			{
				return false;
			}
			
			//normals computation
			CCCoreLib::LOCAL_MODEL_TYPES model = s_lastModelType = ncDlg.getLocalModel();
			bool useGridStructure = withScanGrid && ncDlg.useScanGridsForComputation();
			defaultRadius = ncDlg.getRadius();
			double minGridAngle_deg = s_lastMinGridAngle_deg = ncDlg.getMinGridAngle_deg();
			
			//normals orientation
			s_orientNormals = ncDlg.orientNormals();
			bool orientNormalsWithGrids = withScanGrid && ncDlg.useScanGridsForOrientation();
			bool orientNormalsWithSensors = withSensor && ncDlg.useSensorsForOrientation();
			ccNormalVectors::Orientation preferredOrientation = s_lastNormalOrientation = ncDlg.getPreferredOrientation();
			bool orientNormalsMST = ncDlg.useMSTOrientation();
			int mstNeighbors = s_lastMSTNeighborCount = ncDlg.getMSTNeighborCount();
			
			ccProgressDialog pDlg(true, parent);
			pDlg.setAutoClose(false);

			size_t errors = 0;
			
			for (auto cloud : clouds)
			{
				Q_ASSERT(cloud != nullptr);
				
				bool result = false;
				bool normalsAlreadyOriented = false;
				
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
						
						const int* _indexGrid = scanGrid->indexes.data();
						for (int j = 0; j < static_cast<int>(scanGrid->h); ++j)
						{
							for (int i = 0; i < static_cast<int>(scanGrid->w); ++i, ++_indexGrid)
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
					normalsAlreadyOriented = true;
					result = cloud->computeNormalsWithGrids(minGridAngle_deg, &pDlg);
				}
				else
				{
					//compute normals with the octree
					normalsAlreadyOriented = s_orientNormals && (preferredOrientation != ccNormalVectors::UNDEFINED);
					result = cloud->computeNormalsWithOctree(model, s_orientNormals ? preferredOrientation : ccNormalVectors::UNDEFINED, defaultRadius, &pDlg);
					if (result)
					{
						//save the normal computation radius as meta-data
						cloud->setMetaData(s_NormalScaleKey, defaultRadius);
					}
				}
				
				//do we need to orient the normals? (this may have been already done if 'orientNormalsForThisCloud' is true)
				if (result && s_orientNormals && !normalsAlreadyOriented)
				{
					if (cloud->gridCount() && orientNormalsWithGrids)
					{
						//we can still use the grid structure(s) to orient the normals!
						result = cloud->orientNormalsWithGrids();
					}
					else if (cloud->hasSensor() && orientNormalsWithSensors)
					{
						result = false;

						// RJ: TODO: the issue here is that a cloud can have multiple sensors.
						// As the association to sensor is not explicit in CC, given a cloud
						// some points can belong to one sensor and some others can belongs to others sensors.
						// so it's why here grid orientation has precedence over sensor orientation because in this
						// case association is more explicit.
						// Here we take the first valid viewpoint for now even if it's not a good one...
						for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
						{
							ccHObject* child = cloud->getChild(i);
							if (child && child->isKindOf(CC_TYPES::SENSOR))
							{
								ccSensor* sensor = ccHObjectCaster::ToSensor(child);
								CCVector3 sensorPosition;
								if (sensor->getActiveAbsoluteCenter(sensorPosition))
								{
									result = cloud->orientNormalsTowardViewPoint(sensorPosition, &pDlg);
									break;
								}
							}
						}
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
					ccConsole::Error(QObject::tr("Failed to compute or orient the normals on some clouds! (see console)"));
				else
					ccConsole::Error(QObject::tr("Failed to compute or orient the normals! (see console)"));
			}
		}
		
		//compute normals for each selected mesh
		if (!meshes.empty())
		{
			QMessageBox question( QMessageBox::Question,
										 QObject::tr("Mesh normals"),
										 QObject::tr("Compute per-vertex normals (smooth) or per-triangle (faceted)?"),
										 QMessageBox::NoButton,
										 parent);
			
			QPushButton* perVertexButton   = question.addButton(QObject::tr("Per-vertex"), QMessageBox::YesRole);
			QPushButton* perTriangleButton = question.addButton(QObject::tr("Per-triangle"), QMessageBox::NoRole);
			
			question.exec();
			
			bool computePerVertexNormals = (question.clickedButton() == perVertexButton);
			
			for (auto mesh : meshes)
			{
				Q_ASSERT(mesh != nullptr);
				
				//we remove temporarily the mesh as its normals may be removed (and they can be a child object)
				ccMainAppInterface* instance = dynamic_cast<ccMainAppInterface*>(parent);
				ccMainAppInterface::ccHObjectContext objContext;
				if (instance)
					objContext = instance->removeObjectTemporarilyFromDBTree(mesh);
				mesh->clearTriNormals();
				mesh->showNormals(false);
				bool result = mesh->computeNormals(computePerVertexNormals);
				if (instance)
					instance->putObjectBackIntoDBTree(mesh, objContext);

				if (!result)
				{
					ccConsole::Error(QObject::tr("Failed to compute normals on mesh '%1'").arg(mesh->getName()));
					continue;
				}
				mesh->prepareDisplayForRefresh_recursive();
			}
		}
		
		return true;
	}
	
	bool	invertNormals(const ccHObject::Container &selectedEntities)
	{
		for (ccHObject* ent : selectedEntities)
		{
			// is it a mesh?
			ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
			if (mesh && mesh->hasNormals())
			{
				mesh->invertNormals();
				mesh->showNormals(true);
				mesh->prepareDisplayForRefresh_recursive();
				continue;
			}

			// is it a cloud?
			bool lockedVertices;
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent, &lockedVertices);
			if (cloud && cloud->hasNormals())
			{
				if (lockedVertices)
				{
					ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
					continue;
				}
			
				cloud->invertNormals();
				cloud->showNormals(true);
				cloud->prepareDisplayForRefresh_recursive();
			}
		}
		
		return true;
	}
	
	bool	orientNormalsFM(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.empty())
		{
			ccConsole::Error(QObject::tr("Select at least one point cloud"));
			return false;
		}
		
		bool ok = false;
		const int s_defaultLevel = 6;
		int value = QInputDialog::getInt(	parent,
											QObject::tr("Orient normals (FM)"),
											QObject::tr("Octree level"),
											s_defaultLevel,
											1, CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL,
											1,
											&ok);
		if (!ok)
			return false;
		
		Q_ASSERT(value >= 0 && value <= 255);
		
		unsigned char level = static_cast<unsigned char>(value);
		
		ccProgressDialog pDlg(false, parent);
		pDlg.setAutoClose(false);

		size_t errors = 0;
		for (ccHObject* entity : selectedEntities)
		{
			if (!entity->isA(CC_TYPES::POINT_CLOUD))
				continue;
			
			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
			
			if (!cloud->hasNormals())
			{
				ccConsole::Warning(QObject::tr("Cloud '%1' has no normals!").arg(cloud->getName()));
				continue;
			}
			
			//orient normals with Fast Marching
			if (cloud->orientNormalsWithFM(level, &pDlg))
			{
				cloud->prepareDisplayForRefresh();
			}
			else
			{
				++errors;
			}
		}
		
		if (errors)
		{
			ccConsole::Error(QObject::tr("Process failed (check console)"));
		}
		else
		{
			ccLog::Warning(QObject::tr("Normals have been oriented: you may still have to globally invert the cloud normals however (Edit > Normals > Invert)."));
		}
		
		return true;
	}
	
	bool	orientNormalsMST(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		if (selectedEntities.empty())
		{
			ccConsole::Error(QObject::tr("Select at least one point cloud"));
			return false;
		}
		
		bool ok = false;
		static unsigned s_defaultKNN = 6;
		unsigned kNN = static_cast<unsigned>(QInputDialog::getInt(	parent,
																	QObject::tr("Neighborhood size"),
																	QObject::tr("Neighbors"),
																	s_defaultKNN ,
																	1, 1000,
																	1,
																	&ok));
		if (!ok)
			return false;
		
		s_defaultKNN = kNN;
		
		ccProgressDialog pDlg(true, parent);
		pDlg.setAutoClose(false);
		
		size_t errors = 0;
		for (ccHObject* entity : selectedEntities)
		{
			if (!entity->isA(CC_TYPES::POINT_CLOUD))
				continue;
			
			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
			
			if (!cloud->hasNormals())
			{
				ccConsole::Warning(QObject::tr("Cloud '%1' has no normals!").arg(cloud->getName()));
				continue;
			}
			
			//use Minimum Spanning Tree to resolve normals direction
			if (cloud->orientNormalsWithMST(kNN, &pDlg))
			{
				cloud->prepareDisplayForRefresh();
			}
			else
			{
				ccConsole::Warning(QObject::tr("Process failed on cloud '%1'").arg(cloud->getName()));
				++errors;
			}
		}
		
		if (errors)
		{
			ccConsole::Error(QObject::tr("Process failed (check console)"));
		}
		else
		{
			ccLog::Warning(QObject::tr("Normals have been oriented: you may still have to globally invert the cloud normals however (Edit > Normals > Invert)."));
		}
		
		return true;
	}
	
	bool	convertNormalsTo(const ccHObject::Container &selectedEntities, NORMAL_CONVERSION_DEST dest)
	{
		size_t errorCount = 0;
		size_t successCount = 0;

		size_t selNum = selectedEntities.size();
		for (size_t i = 0; i < selNum; ++i)
		{
			ccHObject* ent = selectedEntities[i];
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selNum == 1);
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
						case NORMAL_CONVERSION_DEST::HSV_COLORS:
						{
							success = ccCloud->convertNormalToRGB();
							if (success)
							{
								ccCloud->showSF(false);
								ccCloud->showNormals(false);
								ccCloud->showColors(true);
								++successCount;
							}
						}
						break;
						
						case NORMAL_CONVERSION_DEST::DIP_DIR_SFS:
						{
							//get/create 'dip' scalar field
							int dipSFIndex = ccCloud->getScalarFieldIndexByName(CC_DEFAULT_DIP_SF_NAME);
							if (dipSFIndex < 0)
								dipSFIndex = ccCloud->addScalarField(CC_DEFAULT_DIP_SF_NAME);
							if (dipSFIndex < 0)
							{
								ccLog::Warning(QObject::tr("[ccEntityAction::convertNormalsTo] Not enough memory!"));
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
								ccLog::Warning(QObject::tr("[ccEntityAction::convertNormalsTo] Not enough memory!"));
								success = false;
								break;
							}
							
							ccScalarField* dipSF = static_cast<ccScalarField*>(ccCloud->getScalarField(dipSFIndex));
							ccScalarField* dipDirSF = static_cast<ccScalarField*>(ccCloud->getScalarField(dipDirSFIndex));
							Q_ASSERT(dipSF && dipDirSF);
							
							success = ccCloud->convertNormalToDipDirSFs(dipSF, dipDirSF);
							
							if (success)
							{
								//apply default 360 degrees color scale!
								ccColorScale::Shared dipScale = ccColorScalesManager::GetDefaultScale(ccColorScalesManager::DIP_BRYW);
								ccColorScale::Shared dipDirScale = ccColorScalesManager::GetDefaultScale(ccColorScalesManager::DIP_DIR_REPEAT);
								dipSF->setColorScale(dipScale);
								dipDirSF->setColorScale(dipDirScale);
								ccCloud->setCurrentDisplayedScalarField(dipDirSFIndex); //dip dir. seems more interesting by default
								ccCloud->showSF(true);
								++successCount;
							}
							else
							{
								ccCloud->deleteScalarField(dipSFIndex);
								ccCloud->deleteScalarField(dipDirSFIndex);
							}
						}
						break;
						
						default:
							Q_ASSERT(false);
							ccLog::Warning(QObject::tr("[ccEntityAction::convertNormalsTo] Internal error: unhandled destination!"));
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
			ccConsole::Error(QObject::tr("Error(s) occurred! (see console)"));
		}
		
		return (successCount != 0);
	}
	
	//////////
	// Octree

	bool computeOctree(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		ccBBox bbox;
		std::unordered_set<ccGenericPointCloud*> clouds;
		PointCoordinateType maxBoxSize = -1;
		for (ccHObject* ent : selectedEntities)
		{
			//specific test for locked vertices
			bool lockedVertices = false;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent, &lockedVertices);
			
			if (cloud == nullptr)
			{
			   continue;
			}
			
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}
			clouds.insert(cloud);

			//we look for the biggest box so as to define the "minimum cell size"
			const ccBBox thisBBox = cloud->getOwnBB();
			if (thisBBox.isValid())
			{
				CCVector3 dd = thisBBox.maxCorner() - thisBBox.minCorner();
				PointCoordinateType maxd = std::max(dd.x, std::max(dd.y, dd.z));
				if (maxBoxSize < 0.0 || maxd > maxBoxSize)
					maxBoxSize = maxd;
			}
			bbox += thisBBox;
		}

		if (clouds.empty() || maxBoxSize < 0.0)
		{
			ccLog::Warning(QObject::tr("[DoActionComputeOctree] No eligible entities in selection!"));
			return false;
		}

		//min(cellSize) = max(dim)/2^N with N = max subidivision level
		const double minCellSize = static_cast<double>(maxBoxSize) / (1 << ccOctree::MAX_OCTREE_LEVEL);

		ccComputeOctreeDlg coDlg(bbox, minCellSize, parent);
		if (!coDlg.exec())
			return false;

		ccProgressDialog pDlg(true, parent);
		pDlg.setAutoClose(false);

		//if we must use a custom bounding box, we update 'bbox'
		if (coDlg.getMode() == ccComputeOctreeDlg::CUSTOM_BBOX)
			bbox = coDlg.getCustomBBox();

		for (const auto cloud : clouds)
		{
			//we temporarily detach entity, as it may undergo
			//'severe' modifications (octree deletion, etc.) --> see ccPointCloud::computeOctree
			ccMainAppInterface* instance = dynamic_cast<ccMainAppInterface*>(parent);
			ccMainAppInterface::ccHObjectContext objContext;
			if (instance)
				objContext = instance->removeObjectTemporarilyFromDBTree(cloud);

			//computation
			QElapsedTimer eTimer;
			eTimer.start();
			ccOctree::Shared octree(nullptr);
			switch (coDlg.getMode())
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
					PointCoordinateType halfBoxWidth = static_cast<PointCoordinateType>(cellSize * (1 << ccOctree::MAX_OCTREE_LEVEL) / 2.0);
					ccBBox bbBox = cloud->getOwnBB();
					CCVector3 C = bbBox.getCenter();
					bbox = ccBBox(	C - CCVector3(halfBoxWidth, halfBoxWidth, halfBoxWidth),
									C + CCVector3(halfBoxWidth, halfBoxWidth, halfBoxWidth),
									bbBox.isValid() );
				}
				cloud->deleteOctree();
				octree = ccOctree::Shared(new ccOctree(cloud));
				if (octree->build(bbox.minCorner(), bbox.maxCorner(), nullptr, nullptr, &pDlg) > 0)
				{
					ccOctreeProxy* proxy = new ccOctreeProxy(octree);
					proxy->setDisplay(cloud->getDisplay());
					cloud->addChild(proxy);
				}
				else
				{
					octree.clear();
				}
			}
			break;
			default:
				Q_ASSERT(false);
				return false;
			}
			qint64 elapsedTime_ms = eTimer.elapsed();

			//put object back in tree
			if (instance)
				instance->putObjectBackIntoDBTree(cloud, objContext);

			if (octree)
			{
				ccConsole::Print("[doActionComputeOctree] Timing: %2.3f s", static_cast<double>(elapsedTime_ms) / 1000.0);
				cloud->setEnabled(true); //for mesh vertices!
				ccOctreeProxy* proxy = cloud->getOctreeProxy();
				assert(proxy);
				proxy->setVisible(true);
				proxy->prepareDisplayForRefresh();
			}
			else
			{
				ccConsole::Warning(QObject::tr("Octree computation on cloud '%1' failed!").arg(cloud->getName()));
			}
		}

		return true;
	}
	
	//////////
	// Properties
	
	bool	clearProperty(ccHObject::Container selectedEntities, CLEAR_PROPERTY property, QWidget* parent/*=nullptr*/)
	{	
		for (ccHObject* ent : selectedEntities)
		{
			//specific case: clear normals on a mesh
			if (property == CLEAR_PROPERTY::NORMALS && ( ent->isA(CC_TYPES::MESH) /*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/ )) //TODO
			{
				ccMesh* mesh = ccHObjectCaster::ToMesh(ent);
				if (!mesh)
				{
					assert(false);
					continue;
				}
				if (mesh->hasTriNormals())
				{
					mesh->showNormals(false);
					
					ccMainAppInterface* instance = dynamic_cast<ccMainAppInterface*>(parent);
					ccMainAppInterface::ccHObjectContext objContext;
					if (instance)
						objContext = instance->removeObjectTemporarilyFromDBTree(mesh);
					mesh->clearTriNormals();
					if (instance)
						instance->putObjectBackIntoDBTree(mesh,objContext);
					
					ent->prepareDisplayForRefresh();
					continue;
				}
				else if (mesh->hasNormals()) //per-vertex normals?
				{
					if (mesh->getParent()
						 && (mesh->getParent()->isA(CC_TYPES::MESH)/*|| mesh->getParent()->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
						 && ccHObjectCaster::ToMesh(mesh->getParent())->getAssociatedCloud() == mesh->getAssociatedCloud())
					{
						ccLog::Warning(QObject::tr("[DoActionClearNormals] Can't remove normals per-vertex on a sub mesh!"));
					}
					else //mesh is alone, we can freely remove normals
					{
						if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
						{
							mesh->showNormals(false);
							static_cast<ccPointCloud*>(mesh->getAssociatedCloud())->unallocateNorms();
							mesh->prepareDisplayForRefresh();
							continue;
						}
					}
				}
			}
			
			bool lockedVertices;
			ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(ent,&lockedVertices);
			if (lockedVertices)
			{
				ccUtils::DisplayLockedVerticesWarning(ent->getName(), selectedEntities.size() == 1);
				continue;
			}
			
			if (cloud && cloud->isA(CC_TYPES::POINT_CLOUD)) // TODO
			{
				auto pointCloud = static_cast<ccPointCloud*>(cloud);
				
				switch (property)
				{
					case CLEAR_PROPERTY::COLORS:
						if (cloud->hasColors())
						{
							pointCloud->unallocateColors();
							ent->prepareDisplayForRefresh();
						}
						break;
						
					case CLEAR_PROPERTY::NORMALS:
						if (cloud->hasNormals())
						{
							pointCloud->unallocateNorms();
							ent->prepareDisplayForRefresh();
						}
						break;
						
					case CLEAR_PROPERTY::CURRENT_SCALAR_FIELD:
						if (cloud->hasDisplayedScalarField())
						{
							pointCloud->deleteScalarField( pointCloud->getCurrentDisplayedScalarFieldIndex() );
							ent->prepareDisplayForRefresh();
						}
						break;
						
					case CLEAR_PROPERTY::ALL_SCALAR_FIELDS:
						if (cloud->hasScalarFields())
						{
							pointCloud->deleteAllScalarFields();
							ent->prepareDisplayForRefresh();
						}
						break;
				}
			}
		}
		
		return true;
	}
	
	bool	toggleProperty(const ccHObject::Container &selectedEntities, TOGGLE_PROPERTY property)
	{
		ccHObject baseEntities;
		ConvertToGroup(selectedEntities, baseEntities, ccHObject::DP_NONE);
		
		for (unsigned i = 0; i < baseEntities.getChildrenNumber(); ++i)
		{
			ccHObject* child = baseEntities.getChild(i);
			switch(property)
			{
				case TOGGLE_PROPERTY::ACTIVE:
					child->toggleActivation/*_recursive*/();
					break;
				case TOGGLE_PROPERTY::VISIBLE:
					child->toggleVisibility_recursive();
					break;
				case TOGGLE_PROPERTY::COLOR:
					child->toggleColors_recursive();
					break;
				case TOGGLE_PROPERTY::NORMALS:
					child->toggleNormals_recursive();
					break;
				case TOGGLE_PROPERTY::SCALAR_FIELD:
					child->toggleSF_recursive();
					break;
				case TOGGLE_PROPERTY::MATERIAL:
					child->toggleMaterials_recursive();
					break;
				case TOGGLE_PROPERTY::NAME:
					child->toggleShowName_recursive();
					break;
				default:
					Q_ASSERT(false);
					return false;
			}
			child->prepareDisplayForRefresh_recursive();
		}
		
		return true;
	}
	
	//////////
	// Stats
	
	bool	statisticalTest(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		ccPickOneElementDlg poeDlg(QObject::tr("Distribution"), QObject::tr("Choose distribution"), parent);
		poeDlg.addElement("Gauss");
		poeDlg.addElement("Weibull");
		poeDlg.setDefaultIndex(0);
		if (!poeDlg.exec())
		{
			return false;
		}
		
		int distribIndex = poeDlg.getSelectedIndex();
		
		ccStatisticalTestDlg* sDlg = nullptr;
		switch (distribIndex)
		{
			case 0: //Gauss
				sDlg = new ccStatisticalTestDlg("mu", "sigma", QString(), QObject::tr("Local Statistical Test (Gauss)"), parent);
				break;
			case 1: //Weibull
				sDlg = new ccStatisticalTestDlg("a", "b", "shift", QObject::tr("Local Statistical Test (Weibull)"), parent);
				break;
			default:
				ccConsole::Error(QObject::tr("Invalid distribution!"));
				return false;
		}
		
		if (!sDlg->exec())
		{
			sDlg->deleteLater();
			return false;
		}
		
		//build up corresponding distribution
		CCCoreLib::GenericDistribution* distrib = nullptr;
		{
			ScalarType a = static_cast<ScalarType>(sDlg->getParam1());
			ScalarType b = static_cast<ScalarType>(sDlg->getParam2());
			ScalarType c = static_cast<ScalarType>(sDlg->getParam3());
			
			switch (distribIndex)
			{
				case 0: //Gauss
				{
					CCCoreLib::NormalDistribution* N = new CCCoreLib::NormalDistribution();
					N->setParameters(a,b*b); //warning: we input sigma2 here (not sigma)
					distrib = static_cast<CCCoreLib::GenericDistribution*>(N);
					break;
				}
				case 1: //Weibull
					CCCoreLib::WeibullDistribution* W = new CCCoreLib::WeibullDistribution();
					W->setParameters(a,b,c);
					distrib = static_cast<CCCoreLib::GenericDistribution*>(W);
					break;
			}
		}
		
		const double pChi2 = sDlg->getProbability();
		const int nn = sDlg->getNeighborsNumber();
		
		ccProgressDialog pDlg(true, parent);
		pDlg.setAutoClose(false);
		
		for (ccHObject* ent : selectedEntities)
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent);
			if (pc == nullptr)
			{
				// TODO handle error?
				continue;
			}
			
			//we apply method on currently displayed SF
			ccScalarField* inSF = pc->getCurrentDisplayedScalarField();
			if (inSF == nullptr)
			{
				// TODO handle error?
				continue;
			}
			
			Q_ASSERT(inSF->capacity() != 0);
			
			//force SF as 'OUT' field (in case of)
			const int outSfIdx = pc->getCurrentDisplayedScalarFieldIndex();
			pc->setCurrentOutScalarField(outSfIdx);
			
			//force Chi2 Distances field as 'IN' field (create it by the way if necessary)
			int chi2SfIdx = pc->getScalarFieldIndexByName(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			
			if (chi2SfIdx < 0)
				chi2SfIdx = pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			
			if (chi2SfIdx < 0)
			{
				ccConsole::Error(QObject::tr("Couldn't allocate a new scalar field for computing chi2 distances! Try to free some memory ..."));
				break;
			}
			pc->setCurrentInScalarField(chi2SfIdx);
			
			//compute octree if necessary
			ccOctree::Shared theOctree = pc->getOctree();
			if (!theOctree)
			{
				theOctree = pc->computeOctree(&pDlg);
				if (!theOctree)
				{
					ccConsole::Error(QObject::tr("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
					break;
				}
			}
			
			QElapsedTimer eTimer;
			eTimer.start();
			
			double chi2dist = CCCoreLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib, pc, nn, pChi2, &pDlg, theOctree.data());
			
			ccConsole::Print("[Chi2 Test] Timing: %3.2f ms.", eTimer.elapsed() / 1000.0);
			ccConsole::Print(QObject::tr("[Chi2 Test] %1 test result = %2").arg(distrib->getName()).arg(chi2dist));
			
			//we set the theoretical Chi2 distance limit as the minimum displayed SF value so that all points below are grayed
			{
				ccScalarField* chi2SF = static_cast<ccScalarField*>(pc->getCurrentInScalarField());
				Q_ASSERT(chi2SF);
				chi2SF->computeMinAndMax();
				chi2dist *= chi2dist;
				chi2SF->setMinDisplayed(static_cast<ScalarType>(chi2dist));
				chi2SF->setSymmetricalScale(false);
				chi2SF->setSaturationStart(static_cast<ScalarType>(chi2dist));
				//chi2SF->setSaturationStop(chi2dist);
				
				pc->setCurrentDisplayedScalarField(chi2SfIdx);
				pc->showSF(true);
				pc->prepareDisplayForRefresh_recursive();
			}
		}
		
		delete distrib;
		distrib = nullptr;
		
		sDlg->deleteLater();
		
		return true;
	}
	
	bool	computeStatParams(const ccHObject::Container &selectedEntities, QWidget* parent/*=nullptr*/)
	{
		ccPickOneElementDlg pDlg(QObject::tr("Distribution"), QObject::tr("Distribution Fitting"), parent);
		pDlg.addElement("Gauss");
		pDlg.addElement("Weibull");
		pDlg.setDefaultIndex(0);
		if (!pDlg.exec())
			return false;
		
		CCCoreLib::GenericDistribution* distrib = nullptr;
		{
			switch (pDlg.getSelectedIndex())
			{
				case 0: //GAUSS
					distrib = new CCCoreLib::NormalDistribution();
					break;
				case 1: //WEIBULL
					distrib = new CCCoreLib::WeibullDistribution();
					break;
				default:
					Q_ASSERT(false);
					return false;
			}
		}
		Q_ASSERT(distrib != nullptr);
		
		for (ccHObject* ent : selectedEntities)
		{
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(ent);
			if (pc == nullptr)
			{
				// TODO report error?
				continue;
			}
			
			//we apply method on currently displayed SF
			ccScalarField* sf = pc->getCurrentDisplayedScalarField();
			if (sf == nullptr)
			{
				// TODO report error?
				continue;
			}

			//compute the number of valid values
			size_t sfValidCount = sf->countValidValues();
			if (sfValidCount == 0)
			{
				ccLog::Warning(QObject::tr("Scalar field '%1' of cloud %2 has no valid values").arg(QString::fromStdString(sf->getName())).arg(pc->getName()));
				continue;
			}
			
			Q_ASSERT(!sf->empty());
			
			if (sf && distrib->computeParameters(CCCoreLib::GenericDistribution::SFAsScalarContainer(*sf)))
			{
				QString description;
				const unsigned precision = ccGui::Parameters().displayedNumPrecision;
				switch (pDlg.getSelectedIndex())
				{
					case 0: //GAUSS
					{
						CCCoreLib::NormalDistribution* normal = static_cast<CCCoreLib::NormalDistribution*>(distrib);
						description = QObject::tr("mean = %1 / std.dev. = %2").arg(normal->getMu(), 0, 'f', precision).arg(sqrt(normal->getSigma2()), 0, 'f', precision);
					}
					break;
					
					case 1: //WEIBULL
					{
						CCCoreLib::WeibullDistribution* weibull = static_cast<CCCoreLib::WeibullDistribution*>(distrib);
						ScalarType a;
						ScalarType b;
						weibull->getParameters(a, b);
						description = QString("a = %1 / b = %2 / shift = %3").arg(a, 0, 'f', precision).arg(b, 0, 'f', precision).arg(weibull->getValueShift(), 0, 'f', precision);
						ccLog::Print(QObject::tr("[Distribution fitting] Additional Weibull distrib. parameters: mode = %1 / skewness = %2").arg(weibull->computeMode()).arg(weibull->computeSkewness()));
					}
					break;

					default:
					{
						Q_ASSERT(false);
						return false;
					}
				}
				description.prepend(QString("%1: ").arg(distrib->getName()));
				ccConsole::Print(QObject::tr("[Distribution fitting] %1").arg(description));
				
				const unsigned numberOfClasses = static_cast<unsigned>(ceil(sqrt(static_cast<double>(sfValidCount))));
				std::vector<unsigned> histo;
				std::vector<double> npis;
				try
				{
					histo.resize(numberOfClasses, 0);
					npis.resize(numberOfClasses, 0.0);
				}
				catch (const std::bad_alloc&)
				{
					ccConsole::Warning(QObject::tr("[Distribution fitting] Not enough memory!"));
					continue;
				}
				
				//compute the Chi2 distance
				{
					unsigned finalNumberOfClasses = 0;
					const double chi2dist = CCCoreLib::StatisticalTestingTools::computeAdaptativeChi2Dist(distrib, pc, numberOfClasses, finalNumberOfClasses, false, nullptr, nullptr, histo.data(), npis.data());

					if (chi2dist >= 0.0)
					{
						ccConsole::Print(QObject::tr("[Distribution fitting] %1: Chi2 Distance = %2").arg(distrib->getName()).arg(chi2dist));
					}
					else
					{
						ccConsole::Warning(QObject::tr("[Distribution fitting] Failed to compute Chi2 distance?!"));
						continue;
					}
				}

				//compute RMS
				{
					unsigned n = pc->size();
					double squareSum = 0;
					double sum = 0;
					for (unsigned i = 0; i < n; ++i)
					{
						ScalarType v = pc->getPointScalarValue(i);
						if (CCCoreLib::ScalarField::ValidValue(v))
						{
							sum += static_cast<double>(v);
							squareSum += static_cast<double>(v) * v;
						}
					}

					double rms = sqrt(squareSum / sfValidCount);
					ccConsole::Print(QObject::tr("Scalar field statistics:"));
					ccConsole::Print(QObject::tr("Number of valid values = %1 / %2 (%3%)").arg(sfValidCount).arg(pc->size()).arg((100.0 * sfValidCount) / pc->size(), 0, 'f', 2));
					ccConsole::Print(QObject::tr("Sum of all valid values = %1").arg(QString::number(sum, 'f', 6)));
					ccConsole::Print(QObject::tr("Sum of all valid squared values = %1").arg(QString::number(squareSum, 'f', 6)));
					ccConsole::Print(QObject::tr("Average value = %1").arg(sum / sfValidCount));
					ccConsole::Print(QObject::tr("RMS (Root Mean Square) = %1").arg(rms));
				}

				//show histogram
				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(parent);
				hDlg->setWindowTitle(QObject::tr("[Distribution fitting]"));
				
				ccHistogramWindow* histogram = hDlg->window();
				histogram->fromBinArray(histo, sf);
				histo.clear();
				histogram->setCurveValues(npis);
				npis.clear();
				histogram->setTitle(description);
				histogram->setAxisLabels(QString::fromStdString(sf->getName()), QObject::tr("Count"));
				histogram->refresh();
				
				hDlg->show();
			}
			else
			{
				ccConsole::Warning(QObject::tr("[Entity: %1]-[SF: %2] Couldn't compute distribution parameters!").arg(pc->getName(), QString::fromStdString(sf->getName())));
			}
		}
		
		delete distrib;
		distrib = nullptr;
		
		return true;
	}
}
