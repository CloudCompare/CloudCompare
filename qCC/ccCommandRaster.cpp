
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
constexpr char COMMAND_GRID_OUTPUT_RASTER_RGB[]			= "OUTPUT_RASTER_RGB";

//Rasterize specific commands
constexpr char COMMAND_RASTERIZE[]						= "RASTERIZE";
constexpr char COMMAND_RASTER_CUSTOM_HEIGHT[]			= "CUSTOM_HEIGHT";
constexpr char COMMAND_RASTER_FILL_EMPTY_CELLS[]		= "EMPTY_FILL";
constexpr char COMMAND_RASTER_FILL_MIN_HEIGHT[]			= "MIN_H";
constexpr char COMMAND_RASTER_FILL_MAX_HEIGHT[]			= "MAX_H";
constexpr char COMMAND_RASTER_FILL_CUSTOM_HEIGHT[]		= "CUSTOM_H";
constexpr char COMMAND_RASTER_FILL_INTERPOLATE[]		= "INTERP";
constexpr char COMMAND_RASTER_PROJ_TYPE[]				= "PROJ";
constexpr char COMMAND_RASTER_SF_PROJ_TYPE[]			= "SF_PROJ";
constexpr char COMMAND_RASTER_PROJ_MIN[]				= "MIN";
constexpr char COMMAND_RASTER_PROJ_MAX[]				= "MAX";
constexpr char COMMAND_RASTER_PROJ_AVG[]				= "AVG";
constexpr char COMMAND_RASTER_RESAMPLE[]				= "RESAMPLE";

//2.5D Volume calculation specific commands
constexpr char COMMAND_VOLUME[] = "VOLUME";
constexpr char COMMAND_VOLUME_GROUND_IS_FIRST[]			= "GROUND_IS_FIRST";
constexpr char COMMAND_VOLUME_CONST_HEIGHT[]			= "CONST_HEIGHT";


static ccRasterGrid::ProjectionType GetProjectionType(QString option, ccCommandLineInterface &cmd)
{
	if (option == COMMAND_RASTER_PROJ_MIN)
	{
		return ccRasterGrid::PROJ_MINIMUM_VALUE;
	}
	else if (option == COMMAND_RASTER_PROJ_MAX)
	{
		return ccRasterGrid::PROJ_MAXIMUM_VALUE;
	}
	else if (option == COMMAND_RASTER_PROJ_AVG)
	{
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	}
	else
	{
		assert(false);
		cmd.warning(QString("Unknwon projection type: %1 (defaulting to 'average')").arg(option));
		return ccRasterGrid::PROJ_AVERAGE_VALUE;
	}
}

static ccRasterGrid::EmptyCellFillOption GetEmptyCellFillingStrategy(QString option, ccCommandLineInterface &cmd)
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
		return ccRasterGrid::INTERPOLATE;
	}
	else
	{
		assert(false);
		cmd.warning(QString("Unknwon empty cell filling strategy: %1 (defaulting to 'leave empty')").arg(option));
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
	bool outputRasterRGB = false;
	bool outputMesh = false;
	bool resample = false;
	double customHeight = std::numeric_limits<double>::quiet_NaN();
	int vertDir = 2;
	ccRasterGrid::ProjectionType projectionType = ccRasterGrid::PROJ_AVERAGE_VALUE;
	ccRasterGrid::ProjectionType sfProjectionType = ccRasterGrid::PROJ_AVERAGE_VALUE;
	ccRasterGrid::EmptyCellFillOption emptyCellFillStrategy = ccRasterGrid::LEAVE_EMPTY;

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

			projectionType = GetProjectionType(cmd.arguments().takeFirst().toUpper(), cmd);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_RASTER_SF_PROJ_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			sfProjectionType = GetProjectionType(cmd.arguments().takeFirst().toUpper(), cmd);
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

	if (gridStep == 0)
	{
		return cmd.error(QString("Grid step value not defined (use %1)").arg(COMMAND_GRID_STEP));
	}

	if (emptyCellFillStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT && std::isnan(customHeight))
	{
		cmd.warning("[Rasterize] The filling stragety is set to 'fill with custom height' but no custom height was defined...");
		emptyCellFillStrategy = ccRasterGrid::LEAVE_EMPTY;
	}

	if (!outputCloud && !outputMesh && !outputRasterZ && !outputRasterRGB)
	{
		//if no export target is specified, we chose the cloud by default
		outputCloud = true;
	}
	assert(outputCloud || outputMesh);

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
			CCVector3d minCorner = CCVector3d::fromArray(gridBBox.minCorner().u);
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

			if (grid.fillWith(cloudDesc.pc,
			                  vertDir,
			                  projectionType,
			                  emptyCellFillStrategy == ccRasterGrid::INTERPOLATE,
			                  sfProjectionType,
			                  pDlg.data()))
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
			std::vector<ccRasterGrid::ExportableFields> exportedFields;
			try
			{
				//we always compute the default 'height' layer
				exportedFields.push_back(ccRasterGrid::PER_CELL_HEIGHT);
			}
			catch (const std::bad_alloc&)
			{
				return cmd.error("Not enough memory");
			}

			ccPointCloud* rasterCloud = grid.convertToCloud(
			                                exportedFields,
			                                true,
			                                true,
			                                resample,
			                                resample,
			                                cloudDesc.pc,
			                                vertDir,
			                                gridBBox,
			                                emptyCellFillStrategy == ccRasterGrid::FILL_CUSTOM_HEIGHT,
			                                customHeight,
			                                true
			                                );

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
			rasterCloud->setGlobalShift(cloudDesc.pc->getGlobalShift());
			rasterCloud->setGlobalScale(cloudDesc.pc->getGlobalScale());

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
				char errorStr[1024];
				CCLib::GenericIndexedMesh* baseMesh = CCLib::PointProjectionTools::computeTriangulation
				                                      (
				                                          rasterCloud,
				                                          DELAUNAY_2D_AXIS_ALIGNED,
				                                          0,
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
					cmd.warning(QString("[Rasterize] Failed to create output mesh ('%1')").arg(errorStr));
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
				bands.allSFs = true;
			}
			QString exportFilename = cmd.getExportFilename(cloudDesc, "tif", "RASTER_Z", nullptr, !cmd.addTimestamp());
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
				bands.allSFs = false;
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
	            ccRasterGrid::LEAVE_EMPTY,
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
				char errorStr[1024];
				CCLib::GenericIndexedMesh* baseMesh = CCLib::PointProjectionTools::computeTriangulation(rasterCloud,
				                                                                                        DELAUNAY_2D_AXIS_ALIGNED,
				                                                                                        0,
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
					return cmd.error(QString("[Voume] Failed to create output mesh ('%1')").arg(errorStr));
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
