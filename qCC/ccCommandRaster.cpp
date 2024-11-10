
//local
#include "ccRasterizeTool.h"

//Qt
#include <QString>
#include <QMessageBox>

//qCC_db
#include "ccCommandRaster.h"

#include <ccMesh.h>
#include <ccProgressDialog.h>
#include <ccVolumeCalcTool.h>

#include <QDateTime>

//shared commands
constexpr char COMMAND_GRID_VERT_DIR[]					= "VERT_DIR";
constexpr char COMMAND_GRID_STEP[]						= "GRID_STEP";
constexpr char COMMAND_GRID_OUTPUT_CLOUD[]				= "OUTPUT_CLOUD";
constexpr char COMMAND_GRID_OUTPUT_MESH[]				= "OUTPUT_MESH";
constexpr char COMMAND_GRID_OUTPUT_RASTER_Z[]			= "OUTPUT_RASTER_Z";
constexpr char COMMAND_GRID_OUTPUT_RASTER_Z_AND_SF[]	= "OUTPUT_RASTER_Z_AND_SF";
constexpr char COMMAND_GRID_OUTPUT_RASTER_RGB[]			= "OUTPUT_RASTER_RGB";

//Rasterize specific commands
constexpr char COMMAND_RASTERIZE[]						= "RASTERIZE";
constexpr char COMMAND_RASTER_CUSTOM_HEIGHT[]			= "CUSTOM_HEIGHT";
constexpr char COMMAND_RASTER_FILL_EMPTY_CELLS[]		= "EMPTY_FILL";
constexpr char COMMAND_RASTER_FILL_MIN_HEIGHT[]			= "MIN_H";
constexpr char COMMAND_RASTER_FILL_MAX_HEIGHT[]			= "MAX_H";
constexpr char COMMAND_RASTER_FILL_CUSTOM_HEIGHT[]		= "CUSTOM_H";
constexpr char COMMAND_RASTER_FILL_INTERPOLATE[]		= "INTERP";
constexpr char COMMAND_RASTER_FILL_KRIGING[]			= "KRIGING";
constexpr char COMMAND_RASTER_FILL_KRIGING_KNN[]		= "KRIGING_KNN";
constexpr char COMMAND_RASTER_PROJ_TYPE[]				= "PROJ";
constexpr char COMMAND_RASTER_SF_PROJ_TYPE[]			= "SF_PROJ";
constexpr char COMMAND_RASTER_INTERP_MAX_EDGE_LENGTH[]	= "MAX_EDGE_LENGTH";
constexpr char COMMAND_RASTER_PROJ_MIN[]				= "MIN";
constexpr char COMMAND_RASTER_PROJ_MAX[]				= "MAX";
constexpr char COMMAND_RASTER_PROJ_AVG[]				= "AVG";
constexpr char COMMAND_RASTER_PROJ_MED[]				= "MED";
constexpr char COMMAND_RASTER_PROJ_INVERSE_VAR[]		= "INV_VAR";
constexpr char COMMAND_RASTER_RESAMPLE[]				= "RESAMPLE";

//2.5D Volume calculation specific commands
constexpr char COMMAND_VOLUME[] = "VOLUME";
constexpr char COMMAND_VOLUME_GROUND_IS_FIRST[]			= "GROUND_IS_FIRST";
constexpr char COMMAND_VOLUME_CONST_HEIGHT[]			= "CONST_HEIGHT";


static bool ReadProjectionType(ccCommandLineInterface& cmd, ccRasterGrid::ProjectionType& projType, QString& stdDevSFDesc)
{
	QString option = cmd.arguments().takeFirst().toUpper();
	stdDevSFDesc.clear();

	if (option == COMMAND_RASTER_PROJ_MIN)
	{
		projType = ccRasterGrid::PROJ_MINIMUM_VALUE;
	}
	else if (option == COMMAND_RASTER_PROJ_MAX)
	{
		projType = ccRasterGrid::PROJ_MAXIMUM_VALUE;
	}
	else if (option == COMMAND_RASTER_PROJ_AVG)
	{
		projType = ccRasterGrid::PROJ_AVERAGE_VALUE;
	}
	else if (option == COMMAND_RASTER_PROJ_MED)
	{
		projType = ccRasterGrid::PROJ_MEDIAN_VALUE;
	}
	else if (option == COMMAND_RASTER_PROJ_INVERSE_VAR)
	{
		projType = ccRasterGrid::PROJ_INVERSE_VAR_VALUE;

		// we expect the std. dev. SF index as well
		if (cmd.arguments().size() != 0)
		{
			stdDevSFDesc = cmd.arguments().takeFirst();
		}
		else
		{
			cmd.error(QString("Expecting the std. dev. SF index after %1").arg(option));
			return false;
		}
	}
	else
	{
		assert(false);
		cmd.error(QString("Unknown projection type: %1").arg(option));
		return false;
	}

	return true;
}

static ccRasterGrid::EmptyCellFillOption GetEmptyCellFillingStrategy(QString option, ccCommandLineInterface& cmd)
{
	if (option == COMMAND_RASTER_FILL_MIN_HEIGHT)
	{
		return ccRasterGrid::FILL_MINIMUM_HEIGHT;
	}
	else if (option == COMMAND_RASTER_FILL_MAX_HEIGHT)
	{
		return ccRasterGrid::FILL_MAXIMUM_HEIGHT;
	}
	else if (option == COMMAND_RASTER_FILL_CUSTOM_HEIGHT)
	{
		return ccRasterGrid::FILL_CUSTOM_HEIGHT;
	}
	else if (option == COMMAND_RASTER_FILL_INTERPOLATE)
	{
		return ccRasterGrid::INTERPOLATE_DELAUNAY;
	}
	else if (option == COMMAND_RASTER_FILL_KRIGING)
	{
		return ccRasterGrid::KRIGING;
	}
	else
	{
		assert(false);
		cmd.warning(QString("Unknown empty cell filling strategy: %1 (defaulting to 'leave empty')").arg(option));
		return ccRasterGrid::LEAVE_EMPTY;
	}
}

CommandRasterize::CommandRasterize()
    : ccCommandLineInterface::Command("Rasterize", COMMAND_RASTERIZE)
{}

bool CommandRasterize::process(ccCommandLineInterface &cmd)
{
	cmd.print("[RASTERIZE]");

	//look for local options
	double gridStep = 0;
	bool outputCloud = false;
	bool outputRasterZ = false;
	bool outputRasterSFs = false;
	bool outputRasterRGB = false;
	bool outputMesh = false;
	bool resample = false;
	double customHeight = std::numeric_limits<double>::quiet_NaN();
	int vertDir = 2;
	ccRasterGrid::ProjectionType projectionType = ccRasterGrid::PROJ_AVERAGE_VALUE;
	ccRasterGrid::ProjectionType sfProjectionType = ccRasterGrid::PROJ_AVERAGE_VALUE;
	ccRasterGrid::EmptyCellFillOption emptyCellFillStrategy = ccRasterGrid::LEAVE_EMPTY;
	ccRasterGrid::DelaunayInterpolationParams dInterpParams;
	ccRasterGrid::KrigingParams krigingParams;
	{
		// force auto-guess
		krigingParams.autoGuess = true;
	}
	QString projStdDevSFDesc, sfProjStdDevSFDesc;

	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_OUTPUT_CLOUD))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (outputMesh)
			{
				cmd.warning("Can't output the grid as a mesh AND a cloud at the same time");
			}
			else
			{
				outputCloud = true;
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_OUTPUT_MESH))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (outputCloud)
			{
				cmd.warning("Can't output the grid as a mesh AND a cloud at the same time");
			}
			else
			{
				outputMesh = true;
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_OUTPUT_RASTER_Z))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			outputRasterZ = true;
			outputRasterSFs = false;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_OUTPUT_RASTER_Z_AND_SF))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			outputRasterZ = true;
			outputRasterSFs = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_OUTPUT_RASTER_RGB))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			outputRasterRGB = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_STEP))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok;
			gridStep = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok || gridStep <= 0)
			{
				return cmd.error(QString("Invalid grid step value! (after %1)").arg(COMMAND_GRID_STEP));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_CUSTOM_HEIGHT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok;
			customHeight = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok)
			{
				return cmd.error(QString("Invalid custom height value! (after %1)").arg(COMMAND_RASTER_CUSTOM_HEIGHT));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_VERT_DIR))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok;
			vertDir = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || vertDir < 0 || vertDir > 2)
			{
				return cmd.error(QString("Invalid vert. direction! (after %1)").arg(COMMAND_GRID_VERT_DIR));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_FILL_EMPTY_CELLS))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			emptyCellFillStrategy = GetEmptyCellFillingStrategy(cmd.arguments().takeFirst().toUpper(), cmd);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_PROJ_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (!ReadProjectionType(cmd, projectionType, projStdDevSFDesc))
			{
				return false;
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_SF_PROJ_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (!ReadProjectionType(cmd, sfProjectionType, sfProjStdDevSFDesc))
			{
				return false;
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_INTERP_MAX_EDGE_LENGTH))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok = false;
			dInterpParams.maxEdgeLength = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok || dInterpParams.maxEdgeLength < 0.0)
			{
				return cmd.error(QString("Invalid max edge length value! (after %1)").arg(COMMAND_RASTER_INTERP_MAX_EDGE_LENGTH));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_FILL_KRIGING_KNN))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok = false;
			krigingParams.kNN = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || krigingParams.kNN <= 0)
			{
				return cmd.error(QString("Invalid Kriging knn value! (after %1)").arg(COMMAND_RASTER_FILL_KRIGING_KNN));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_RESAMPLE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			resample = true;
		}
		else
		{
			break;
		}
	}

	// there can be only one (std. dev. SF ;)
	QString stdDevSFDesc;
	if (!projStdDevSFDesc.isEmpty())
	{
		if (!sfProjStdDevSFDesc.isEmpty() && projStdDevSFDesc != sfProjStdDevSFDesc)
		{
			cmd.warning("[Rasterize] Can't set 2 different std. dev. SF for inverse variance projection modes. The point projection SF will be used by default.");
		}
		stdDevSFDesc = projStdDevSFDesc;
	}
	else
	{
		stdDevSFDesc = sfProjStdDevSFDesc;
	}

	if (gridStep == 0)
	{
		return cmd.error(QString("Grid step value not defined (use %1)").arg(COMMAND_GRID_STEP));
	}

	if (std::isnan(customHeight))
	{
		if (emptyCellFillStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT)
		{
			cmd.warning("[Rasterize] The filling stragety is set to 'fill with custom height' but no custom height was defined...");
			emptyCellFillStrategy = ccRasterGrid::LEAVE_EMPTY;
		}
		else if (emptyCellFillStrategy == ccRasterGrid::INTERPOLATE_DELAUNAY)
		{
			cmd.warning("[Rasterize] The filling stragety is set to 'Delaunay' but no custom height was defined. Some holes may remain (outside of the convex hull).");
		}
	}

	if (!outputCloud && !outputMesh && !outputRasterZ && !outputRasterRGB)
	{
		//if no export target is specified, we chose the cloud by default
		outputCloud = true;
	}

	if (resample && !outputCloud && !outputMesh)
	{
		cmd.warning("[Rasterize] The 'resample' option is set while the raster won't be exported as a cloud nor as a mesh");
	}

	//we'll get the first two clouds
	for (CLCloudDesc& cloudDesc : cmd.clouds())
	{
		if (!cloudDesc.pc)
		{
			assert(false);
			continue;
		}

		int invVarProjSFIndex = -1;
		if (projectionType == ccRasterGrid::PROJ_INVERSE_VAR_VALUE)
		{
			// let's check if the SF description is its name
			invVarProjSFIndex = cloudDesc.pc->getScalarFieldIndexByName(stdDevSFDesc.toStdString());
			if (invVarProjSFIndex < 0)
			{
				// let's check if it's a (valid) index then
				bool validValue = false;
				invVarProjSFIndex = stdDevSFDesc.toInt(&validValue);
				if (!validValue)
				{
					return cmd.error(QString("[Rasterize] Failed to recognize the std. dev. SF '%1' (neither an existing scalar field name nor a valid index)").arg(stdDevSFDesc));
				}
				else if (invVarProjSFIndex < 0 || static_cast<unsigned>(invVarProjSFIndex) >= cloudDesc.pc->getNumberOfScalarFields())
				{
					return cmd.error("[Rasterize] Invalid std. dev. SF index (negative or greater than the number of scalar fields in the cloud");
				}
			}
		}

		ccBBox gridBBox = cloudDesc.pc->getOwnBB();

		//compute the grid size
		unsigned gridWidth = 0;
		unsigned gridHeight = 0;
		if (!ccRasterGrid::ComputeGridSize(vertDir, gridBBox, gridStep, gridWidth, gridHeight))
		{
			return cmd.error("Failed to compute the grid dimensions (check input cloud(s) bounding-box)");
		}

		cmd.print(QString("Grid size: %1 x %2").arg(gridWidth).arg(gridHeight));

		if (gridWidth * gridHeight > (1 << 26)) //64 million of cells
		{
			if (cmd.silentMode())
			{
				ccLog::Warning("Huge grid detected!");
			}
			else
			{
				static bool s_firstTime = true;
				if (s_firstTime && QMessageBox::warning(cmd.widgetParent(), "Raster grid", "Grid size is huge. Are you sure you want to proceed?\n(you can avoid this message by running in SILENT mode)", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
				{
					return ccLog::Warning("Process cancelled");
				}
				s_firstTime = false;
			}
		}

		ccRasterGrid grid;
		{
			//memory allocation
			CCVector3d minCorner = gridBBox.minCorner();
			if (!grid.init(gridWidth, gridHeight, gridStep, minCorner))
			{
				//not enough memory
				return cmd.error("Not enough memory");
			}

			//progress dialog
			QScopedPointer<ccProgressDialog> pDlg(nullptr);
			if (!cmd.silentMode())
			{
				pDlg.reset(new ccProgressDialog(true, cmd.widgetParent()));
			}

			ccRasterGrid::InterpolationType interpolationType = ccRasterGrid::InterpolationTypeFromEmptyCellFillOption(emptyCellFillStrategy);
			void* interpolationParams = nullptr;
			switch (interpolationType)
			{
			case ccRasterGrid::InterpolationType::DELAUNAY:
				interpolationParams = (void*)&dInterpParams;
				break;
			case ccRasterGrid::InterpolationType::KRIGING:
				interpolationParams = (void*)&krigingParams;
				break;
			default:
				// do nothing
				break;
			}

			if (grid.fillWith(	cloudDesc.pc,
								vertDir,
								projectionType,
								interpolationType,
								interpolationParams,
								sfProjectionType,
								pDlg.data(),
								invVarProjSFIndex )
				)
			{
				grid.fillEmptyCells(emptyCellFillStrategy, customHeight);
				cmd.print(QString("[Rasterize] Raster grid: size: %1 x %2 / heights: [%3 ; %4]").arg(grid.width).arg(grid.height).arg(grid.minHeight).arg(grid.maxHeight));
			}
			else
			{
				return cmd.error("Rasterize process failed");
			}
		}

		//generate the result entity (cloud by default)
		if (outputCloud || outputMesh)
		{
			ccPointCloud* rasterCloud = nullptr;
			try
			{
				//we always compute the default 'height' layer
				std::vector<ccRasterGrid::ExportableFields> exportedStatistics(1);
				exportedStatistics.back() = ccRasterGrid::PER_CELL_VALUE;

				rasterCloud = grid.convertToCloud(	true,
													false,
													exportedStatistics,
													true,
													true,
													resample,
													resample,
													cloudDesc.pc,
													vertDir,
													gridBBox,
													0.0,
													true,
													true,
													nullptr
												);

			}
			catch (const std::bad_alloc&)
			{
				return cmd.error("Not enough memory");
			}

			if (!rasterCloud)
			{
				return cmd.error("Failed to output the raster grid as a cloud");
			}

			rasterCloud->showColors(cloudDesc.pc->hasColors());
			if (rasterCloud->hasScalarFields())
			{
				rasterCloud->showSF(!cloudDesc.pc->hasColors());
				rasterCloud->setCurrentDisplayedScalarField(0);
			}
			//don't forget the original shift
			rasterCloud->copyGlobalShiftAndScale(*cloudDesc.pc);

			if (outputCloud)
			{
				assert(!outputMesh);
				//replace current cloud by the restarized version
				delete cloudDesc.pc;
				cloudDesc.pc = rasterCloud;
				cloudDesc.basename += QString("_RASTER");

				rasterCloud = nullptr;

				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(cloudDesc);
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
			else if (outputMesh)
			{
				std::string errorStr;
				CCCoreLib::GenericIndexedMesh* baseMesh = CCCoreLib::PointProjectionTools::computeTriangulation
				                                      (
				                                          rasterCloud,
				                                          CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED,
				                                          CCCoreLib::PointProjectionTools::IGNORE_MAX_EDGE_LENGTH,
				                                          vertDir,
				                                          errorStr
				                                          );

				if (baseMesh)
				{
					ccMesh* rasterMesh = new ccMesh(baseMesh, rasterCloud);
					delete baseMesh;
					baseMesh = nullptr;

					rasterCloud->setEnabled(false);
					rasterCloud->setVisible(true);
					rasterMesh->addChild(rasterCloud);
					rasterMesh->setName(rasterCloud->getName());
					//rasterCloud->setName("vertices");
					rasterMesh->showSF(rasterCloud->sfShown());
					rasterMesh->showColors(rasterCloud->colorsShown());
					rasterCloud = nullptr; //to avoid deleting it later

					cmd.print(QString("[Rasterize] Mesh '%1' successfully generated").arg(rasterMesh->getName()));

					CLMeshDesc meshDesc;
					meshDesc.mesh = rasterMesh;
					meshDesc.basename = cloudDesc.basename + QString("_RASTER_MESH");
					meshDesc.path = cloudDesc.path;

					QString errorStr = cmd.exportEntity(meshDesc);
					if (!errorStr.isEmpty())
					{
						delete rasterMesh;
						return cmd.error(errorStr);
					}

					//we keep the mesh loaded
					cmd.meshes().push_back(meshDesc);
					//delete rasterMesh;
					//rasterMesh = 0;
				}
				else
				{
					cmd.warning( QStringLiteral("[Rasterize] Failed to create output mesh ('%1')")
								 .arg( QString::fromStdString( errorStr ) ) );
				}
			}

			if (rasterCloud)
			{
				delete rasterCloud;
				rasterCloud = nullptr;
			}
		}

		if (outputRasterZ)
		{
			ccRasterizeTool::ExportBands bands;
			{
				bands.height = true;
				bands.rgb = false; //not a good idea to mix RGB and height values!
				bands.allSFs = outputRasterSFs;
			}
			QString exportFilename = cmd.getExportFilename(cloudDesc, "tif", outputRasterSFs ? "RASTER_Z_AND_SF" : "RASTER_Z", nullptr, !cmd.addTimestamp());
			if (exportFilename.isEmpty())
			{
				exportFilename = "rasterZ.tif";
			}

			ccRasterizeTool::ExportGeoTiff(exportFilename, bands, emptyCellFillStrategy, grid, gridBBox, vertDir, customHeight, cloudDesc.pc);
		}

		if (outputRasterRGB)
		{
			ccRasterizeTool::ExportBands bands;
			{
				bands.rgb = true;
				bands.height = false; //not a good idea to mix RGB and height values!
				bands.allSFs = outputRasterSFs;
			}
			QString exportFilename = cmd.getExportFilename(cloudDesc, "tif", "RASTER_RGB", nullptr, !cmd.addTimestamp());
			if (exportFilename.isEmpty())
			{
				exportFilename = "rasterRGB.tif";
			}

			ccRasterizeTool::ExportGeoTiff(exportFilename, bands, emptyCellFillStrategy, grid, gridBBox, vertDir, customHeight, cloudDesc.pc);
		}
	}

	return true;
}

CommandVolume25D::CommandVolume25D()
    : ccCommandLineInterface::Command("2.5D Volume Calculation", COMMAND_VOLUME)
{}

bool CommandVolume25D::process(ccCommandLineInterface &cmd)
{
	cmd.print("[2.5D VOLUME]");

	//look for local options
	bool groundIsFirst = false;
	double gridStep = 0;
	double constHeight = std::numeric_limits<double>::quiet_NaN();
	bool outputMesh = false;
	int vertDir = 2;

	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_VOLUME_GROUND_IS_FIRST))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			groundIsFirst = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_OUTPUT_MESH))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			outputMesh = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_STEP))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok;
			gridStep = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok || gridStep <= 0)
			{
				return cmd.error(QString("Invalid grid step value! (after %1)").arg(COMMAND_GRID_STEP));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_VOLUME_CONST_HEIGHT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok;
			constHeight = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok)
			{
				return cmd.error(QString("Invalid const. height value! (after %1)").arg(COMMAND_VOLUME_CONST_HEIGHT));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_GRID_VERT_DIR))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			bool ok;
			vertDir = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || vertDir < 0 || vertDir > 2)
			{
				return cmd.error(QString("Invalid vert. direction! (after %1)").arg(COMMAND_GRID_VERT_DIR));
			}
		}
		else
		{
			//unrecognized argument (probably another command?)
			break;
		}
	}

	if (gridStep == 0)
	{
		return cmd.error(QString("Grid step value not defined (use %1)").arg(COMMAND_GRID_STEP));
	}

	//we'll get the first two clouds
	CLCloudDesc *ground = nullptr;
	CLCloudDesc *ceil = nullptr;
	{
		CLCloudDesc* clouds[2] = { nullptr, nullptr };
		int index = 0;
		if (!cmd.clouds().empty())
		{
			clouds[index++] = &cmd.clouds()[0];
			if (std::isnan(constHeight) && cmd.clouds().size() > 1)
			{
				clouds[index++] = &cmd.clouds()[1];
			}
		}

		int expectedCount = std::isnan(constHeight) ? 2 : 1;
		if (index != expectedCount)
		{
			return cmd.error(QString("Not enough loaded entities (%1 found, %2 expected)").arg(index).arg(expectedCount));
		}

		if (index == 2 && groundIsFirst)
		{
			//put them in the right order (ground then ceil)
			std::swap(clouds[0], clouds[1]);
		}

		ceil = clouds[0];
		ground = clouds[1];
	}

	ccBBox gridBBox = ceil ? ceil->pc->getOwnBB() : ccBBox();
	if (ground)
	{
		gridBBox += ground->pc->getOwnBB();
	}

	//compute the grid size
	unsigned gridWidth = 0;
	unsigned gridHeight = 0;
	if (!ccRasterGrid::ComputeGridSize(vertDir, gridBBox, gridStep, gridWidth, gridHeight))
	{
		return cmd.error("Failed to compute the grid dimensions (check input cloud(s) bounding-box)");
	}

	cmd.print(QString("Grid size: %1 x %2").arg(gridWidth).arg(gridHeight));

	if (gridWidth * gridHeight > (1 << 26)) //64 million of cells
	{
		if (cmd.silentMode())
		{
			ccLog::Warning("Huge grid detected!");
		}
		else
		{
			static bool s_firstTime = true;
			if (s_firstTime && QMessageBox::warning(cmd.widgetParent(), "Volume grid", "Grid size is huge. Are you sure you want to proceed?\n(you can avoid this message by running in SILENT mode)", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			{
				return ccLog::Warning("Process cancelled");
			}
			s_firstTime = false;
		}
	}

	ccRasterGrid grid;
	ccVolumeCalcTool::ReportInfo reportInfo;
	if (ccVolumeCalcTool::ComputeVolume(
	            grid,
	            ground ? ground->pc : nullptr,
	            ceil ? ceil->pc : nullptr,
	            gridBBox,
	            vertDir,
	            gridStep,
	            gridWidth,
	            gridHeight,
	            ccRasterGrid::PROJ_AVERAGE_VALUE,
	            ccRasterGrid::LEAVE_EMPTY,
				0.0,
	            ccRasterGrid::LEAVE_EMPTY,
				0.0,
	            reportInfo,
	            constHeight,
	            constHeight,
	            cmd.silentMode() ? nullptr : cmd.widgetParent()))
	{
		CLCloudDesc* desc = ceil ? ceil : ground;
		assert(desc);

		//save repot in a separate text file
		{
			QString txtFilename = QString("%1/VolumeCalculationReport").arg(desc->path);
			if (cmd.addTimestamp())
				txtFilename += QString("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));
			txtFilename += QString(".txt");

			QFile txtFile(txtFilename);
			txtFile.open(QIODevice::WriteOnly | QIODevice::Text);
			QTextStream txtStream(&txtFile);
			txtStream << reportInfo.toText() << endl;
			txtFile.close();
		}

		//generate the result entity (cloud by default)
		{
			ccPointCloud* rasterCloud = ccVolumeCalcTool::ConvertGridToCloud(grid, gridBBox, vertDir, true);
			if (!rasterCloud)
			{
				return cmd.error("Failed to output the volume grid");
			}
			if (rasterCloud->hasScalarFields())
			{
				//convert SF to RGB
				//rasterCloud->setCurrentDisplayedScalarField(0);
				rasterCloud->convertCurrentScalarFieldToColors(false);
				rasterCloud->showColors(true);
			}

			ccMesh* rasterMesh = nullptr;
			if (outputMesh)
			{
				std::string errorStr;
				CCCoreLib::GenericIndexedMesh* baseMesh = CCCoreLib::PointProjectionTools::computeTriangulation(rasterCloud,
				                                                                                        CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED,
				                                                                                        CCCoreLib::PointProjectionTools::IGNORE_MAX_EDGE_LENGTH,
				                                                                                        vertDir,
				                                                                                        errorStr);

				if (baseMesh)
				{
					rasterMesh = new ccMesh(baseMesh, rasterCloud);
					delete baseMesh;
					baseMesh = nullptr;
				}

				if (rasterMesh)
				{
					rasterCloud->setEnabled(false);
					rasterCloud->setVisible(true);
					rasterMesh->addChild(rasterCloud);
					rasterMesh->setName(rasterCloud->getName());
					rasterCloud->setName("vertices");
					rasterMesh->showSF(rasterCloud->sfShown());
					rasterMesh->showColors(rasterCloud->colorsShown());

					cmd.print(QString("[Volume] Mesh '%1' successfully generated").arg(rasterMesh->getName()));
				}
				else
				{
					delete rasterCloud;
					return cmd.error( QStringLiteral("[Voume] Failed to create output mesh ('%1')")
									  .arg( QString::fromStdString( errorStr ) ) );
				}
			}

			CLEntityDesc* outputDesc = nullptr;
			if (rasterMesh)
			{
				CLMeshDesc meshDesc;
				meshDesc.mesh = rasterMesh;
				meshDesc.basename = desc->basename;
				meshDesc.path = desc->path;
				cmd.meshes().push_back(meshDesc);
				outputDesc = &cmd.meshes().back();
			}
			else
			{
				CLCloudDesc cloudDesc;
				cloudDesc.pc = rasterCloud;
				cloudDesc.basename = desc->basename;
				cloudDesc.path = desc->path;
				cmd.clouds().push_back(cloudDesc);
				outputDesc = &cmd.clouds().back();
			}

			//save result
			if (outputDesc && cmd.autoSaveMode())
			{
				QString outputFilename;
				QString errorStr = cmd.exportEntity(*outputDesc, "HEIGHT_DIFFERENCE", &outputFilename);
				if (!errorStr.isEmpty())
					cmd.warning(errorStr);
			}
		}
	}
	else
	{
		return cmd.error("Failed to compte the volume");
	}

	return true;
}
