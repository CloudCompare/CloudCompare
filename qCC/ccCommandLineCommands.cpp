//CCLib
#include <AutoSegmentationTools.h>
#include <CCConst.h>
#include <CloudSamplingTools.h>
#include <MeshSamplingTools.h>
#include <NormalDistribution.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccNormalVectors.h>
#include <ccPlane.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccVolumeCalcTool.h>

//qCC_io
#include <AsciiFilter.h>
#include <PlyFilter.h>

//qCC
#include "ccCommon.h"
#include "ccComparisonDlg.h"
#include "ccConsole.h"
#include "ccCropTool.h"
#include "ccLibAlgorithms.h"
#include "ccRegistrationTools.h"
#include "ccScalarFieldArithmeticsDlg.h"

//Qt
#include "ccCommandLineCommands.h"

#include <QDateTime>

//commands
constexpr char COMMAND_CLOUD_EXPORT_FORMAT[]			= "C_EXPORT_FMT";
constexpr char COMMAND_EXPORT_EXTENSION[]				= "EXT";
constexpr char COMMAND_ASCII_EXPORT_PRECISION[]			= "PREC";
constexpr char COMMAND_ASCII_EXPORT_SEPARATOR[]			= "SEP";
constexpr char COMMAND_ASCII_EXPORT_ADD_COL_HEADER[]	= "ADD_HEADER";
constexpr char COMMAND_MESH_EXPORT_FORMAT[]				= "M_EXPORT_FMT";
constexpr char COMMAND_ASCII_EXPORT_ADD_PTS_COUNT[]		= "ADD_PTS_COUNT";
constexpr char COMMAND_OPEN[]							= "O";				//+file name
constexpr char COMMAND_OPEN_SKIP_LINES[]				= "SKIP";			//+number of lines to skip
constexpr char COMMAND_SUBSAMPLE[]						= "SS";				//+ method (RANDOM/SPATIAL/OCTREE) + parameter (resp. point count / spatial step / octree level)
constexpr char COMMAND_EXTRACT_CC[]						= "EXTRACT_CC";
constexpr char COMMAND_CURVATURE[]						= "CURV";			//+ curvature type (MEAN/GAUSS)
constexpr char COMMAND_DENSITY[]						= "DENSITY";		//+ sphere radius
constexpr char COMMAND_DENSITY_TYPE[]					= "TYPE";			//+ density type
constexpr char COMMAND_APPROX_DENSITY[]					= "APPROX_DENSITY";
constexpr char COMMAND_SF_GRADIENT[]					= "SF_GRAD";
constexpr char COMMAND_ROUGHNESS[]						= "ROUGH";
constexpr char COMMAND_APPLY_TRANSFORMATION[]			= "APPLY_TRANS";
constexpr char COMMAND_DROP_GLOBAL_SHIFT[]				= "DROP_GLOBAL_SHIFT";
constexpr char COMMAND_SF_COLOR_SCALE[]					= "SF_COLOR_SCALE";
constexpr char COMMAND_SF_CONVERT_TO_RGB[]				= "SF_CONVERT_TO_RGB";
constexpr char COMMAND_FILTER_SF_BY_VALUE[]				= "FILTER_SF";
constexpr char COMMAND_MERGE_CLOUDS[]					= "MERGE_CLOUDS";
constexpr char COMMAND_MERGE_MESHES[]                   = "MERGE_MESHES";
constexpr char COMMAND_SET_ACTIVE_SF[]					= "SET_ACTIVE_SF";
constexpr char COMMAND_REMOVE_ALL_SFS[]					= "REMOVE_ALL_SFS";
constexpr char COMMAND_REMOVE_SCAN_GRIDS[]				= "REMOVE_SCAN_GRIDS";
constexpr char COMMAND_MATCH_BB_CENTERS[]				= "MATCH_CENTERS";
constexpr char COMMAND_BEST_FIT_PLANE[]					= "BEST_FIT_PLANE";
constexpr char COMMAND_BEST_FIT_PLANE_MAKE_HORIZ[]		= "MAKE_HORIZ";
constexpr char COMMAND_BEST_FIT_PLANE_KEEP_LOADED[]		= "KEEP_LOADED";
constexpr char COMMAND_ORIENT_NORMALS[]					= "ORIENT_NORMS_MST";
constexpr char COMMAND_SOR_FILTER[]						= "SOR";
constexpr char COMMAND_SAMPLE_MESH[]					= "SAMPLE_MESH";
constexpr char COMMAND_CROP[]							= "CROP";
constexpr char COMMAND_CROP_OUTSIDE[]					= "OUTSIDE";
constexpr char COMMAND_CROP_2D[]						= "CROP2D";
constexpr char COMMAND_COLOR_BANDING[]					= "CBANDING";
constexpr char COMMAND_C2M_DIST[]						= "C2M_DIST";
constexpr char COMMAND_C2M_DIST_FLIP_NORMALS[]			= "FLIP_NORMS";
constexpr char COMMAND_C2C_DIST[]						= "C2C_DIST";
constexpr char COMMAND_C2C_SPLIT_XYZ[]					= "SPLIT_XYZ";
constexpr char COMMAND_C2C_LOCAL_MODEL[]				= "MODEL";
constexpr char COMMAND_C2X_MAX_DISTANCE[]				= "MAX_DIST";
constexpr char COMMAND_C2X_OCTREE_LEVEL[]				= "OCTREE_LEVEL";
constexpr char COMMAND_STAT_TEST[]						= "STAT_TEST";
constexpr char COMMAND_DELAUNAY[]						= "DELAUNAY";
constexpr char COMMAND_DELAUNAY_AA[]					= "AA";
constexpr char COMMAND_DELAUNAY_BF[]					= "BEST_FIT";
constexpr char COMMAND_DELAUNAY_MAX_EDGE_LENGTH[]		= "MAX_EDGE_LENGTH";
constexpr char COMMAND_SF_ARITHMETIC[]					= "SF_ARITHMETIC";
constexpr char COMMAND_SF_OP[]							= "SF_OP";
constexpr char COMMAND_COORD_TO_SF[]					= "COORD_TO_SF";
constexpr char COMMAND_EXTRACT_VERTICES[]				= "EXTRACT_VERTICES";
constexpr char COMMAND_ICP[]							= "ICP";
constexpr char COMMAND_ICP_REFERENCE_IS_FIRST[]			= "REFERENCE_IS_FIRST";
constexpr char COMMAND_ICP_MIN_ERROR_DIIF[]				= "MIN_ERROR_DIFF";
constexpr char COMMAND_ICP_ITERATION_COUNT[]			= "ITER";
constexpr char COMMAND_ICP_OVERLAP[]					= "OVERLAP";
constexpr char COMMAND_ICP_ADJUST_SCALE[]				= "ADJUST_SCALE";
constexpr char COMMAND_ICP_RANDOM_SAMPLING_LIMIT[]		= "RANDOM_SAMPLING_LIMIT";
constexpr char COMMAND_ICP_ENABLE_FARTHEST_REMOVAL[]	= "FARTHEST_REMOVAL";
constexpr char COMMAND_ICP_USE_MODEL_SF_AS_WEIGHT[]		= "MODEL_SF_AS_WEIGHTS";
constexpr char COMMAND_ICP_USE_DATA_SF_AS_WEIGHT[]		= "DATA_SF_AS_WEIGHTS";
constexpr char COMMAND_ICP_ROT[]						= "ROT";
constexpr char COMMAND_PLY_EXPORT_FORMAT[]				= "PLY_EXPORT_FMT";
constexpr char COMMAND_COMPUTE_GRIDDED_NORMALS[]		= "COMPUTE_NORMALS";
constexpr char COMMAND_COMPUTE_OCTREE_NORMALS[]			= "OCTREE_NORMALS";
constexpr char COMMAND_CLEAR_NORMALS[]					= "CLEAR_NORMALS";
constexpr char COMMAND_MESH_VOLUME[]                    = "MESH_VOLUME";
constexpr char COMMAND_VOLUME_TO_FILE[]					= "TO_FILE";
constexpr char COMMAND_SAVE_CLOUDS[]					= "SAVE_CLOUDS";
constexpr char COMMAND_SAVE_MESHES[]					= "SAVE_MESHES";
constexpr char COMMAND_AUTO_SAVE[]						= "AUTO_SAVE";
constexpr char COMMAND_LOG_FILE[]						= "LOG_FILE";
constexpr char COMMAND_CLEAR[]							= "CLEAR";
constexpr char COMMAND_CLEAR_CLOUDS[]					= "CLEAR_CLOUDS";
constexpr char COMMAND_POP_CLOUDS[]						= "POP_CLOUDS";
constexpr char COMMAND_CLEAR_MESHES[]					= "CLEAR_MESHES";
constexpr char COMMAND_POP_MESHES[]						= "POP_MESHES";
constexpr char COMMAND_NO_TIMESTAMP[]					= "NO_TIMESTAMP";

//options / modifiers
constexpr char COMMAND_MAX_THREAD_COUNT[]				= "MAX_TCOUNT";
constexpr char OPTION_ALL_AT_ONCE[]						= "ALL_AT_ONCE";
constexpr char OPTION_ON[]								= "ON";
constexpr char OPTION_OFF[]								= "OFF";
constexpr char OPTION_LAST[]							= "LAST";
constexpr char OPTION_FILE_NAMES[]						= "FILE";

CommandChangeOutputFormat::CommandChangeOutputFormat(const QString& name, const QString& keyword)
    : ccCommandLineInterface::Command(name, keyword)
{}

QString CommandChangeOutputFormat::getFileFormatFilter(ccCommandLineInterface &cmd, QString &defaultExt)
{
	QString fileFilter;
	defaultExt = QString();

	if (!cmd.arguments().isEmpty())
	{
		//test if the specified format corresponds to a known file type
		QString argument = cmd.arguments().front().toUpper();
		cmd.arguments().pop_front();

		const FileIOFilter::FilterContainer& filters = FileIOFilter::GetFilters();
		
		for (const auto & filter : filters)
		{
			if (argument == filter->getDefaultExtension().toUpper())
			{
				//found
				fileFilter = filter->getFileFilters(false).first(); //Take the first 'output' file filter by default (could we be smarter?)
				defaultExt = filter->getDefaultExtension();
				break;
			}
		}

		//haven't found anything?
		if (fileFilter.isEmpty())
		{
			cmd.error(QObject::tr("Unhandled format specifier (%1)").arg(argument));
		}
	}
	else
	{
		cmd.error("Missing file format specifier!");
	}

	return fileFilter;
}

CommandChangeCloudOutputFormat::CommandChangeCloudOutputFormat()
	: CommandChangeOutputFormat("Change cloud output format", COMMAND_CLOUD_EXPORT_FORMAT)
{}

bool CommandChangeCloudOutputFormat::process(ccCommandLineInterface &cmd)
{
	QString defaultExt;
	QString fileFilter = getFileFormatFilter(cmd, defaultExt);
	if (fileFilter.isEmpty())
		return false;
	
	cmd.setCloudExportFormat(fileFilter, defaultExt);
	cmd.print(QObject::tr("Output export format (clouds) set to: %1").arg(defaultExt.toUpper()));
	
	//default options for ASCII output
	if (fileFilter == AsciiFilter::GetFileFilter())
	{
		AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
		assert(saveDialog);
		saveDialog->setCoordsPrecision(cmd.numericalPrecision());
		saveDialog->setSfPrecision(cmd.numericalPrecision());
		saveDialog->setSeparatorIndex(0); //space
		saveDialog->enableSwapColorAndSF(false); //default order: point, color, SF, normal
		saveDialog->enableSaveColumnsNamesHeader(false);
		saveDialog->enableSavePointCountHeader(false);
	}
	
	//look for additional parameters
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_EXPORT_EXTENSION))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));
			
			cmd.setCloudExportFormat(cmd.cloudExportFormat(), cmd.arguments().takeFirst());
			cmd.print(QObject::tr("New output extension for clouds: %1").arg(cmd.cloudExportExt()));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_PRECISION))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: precision value after '%1'").arg(COMMAND_ASCII_EXPORT_PRECISION));
			bool ok;
			int precision = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || precision < 0)
				return cmd.error(QObject::tr("Invalid value for precision! (%1)").arg(COMMAND_ASCII_EXPORT_PRECISION));
			
			if (fileFilter != AsciiFilter::GetFileFilter())
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			
			AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			if (saveDialog)
			{
				saveDialog->setCoordsPrecision(precision);
				saveDialog->setSfPrecision(precision);
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_SEPARATOR))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: separator character after '%1'").arg(COMMAND_ASCII_EXPORT_SEPARATOR));
			
			if (fileFilter != AsciiFilter::GetFileFilter())
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			
			QString separatorStr = cmd.arguments().takeFirst().toUpper();
			//printf("%s\n",qPrintable(separatorStr));
			int index = -1;
			if (separatorStr == "SPACE")
				index = 0;
			else if (separatorStr == "SEMICOLON")
				index = 1;
			else if (separatorStr == "COMMA")
				index = 2;
			else if (separatorStr == "TAB")
				index = 3;
			else
				return cmd.error(QObject::tr("Invalid separator! ('%1')").arg(separatorStr));
			
			AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			if (saveDialog)
			{
				saveDialog->setSeparatorIndex(index);
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_ADD_COL_HEADER))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (fileFilter != AsciiFilter::GetFileFilter())
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			
			AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			if (saveDialog)
			{
				saveDialog->enableSaveColumnsNamesHeader(true);
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_ADD_PTS_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (fileFilter != AsciiFilter::GetFileFilter())
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			
			AsciiSaveDlg* saveDialog = AsciiFilter::GetSaveDialog();
			assert(saveDialog);
			if (saveDialog)
			{
				saveDialog->enableSavePointCountHeader(true);
			}
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	return true;
}

CommandChangeMeshOutputFormat::CommandChangeMeshOutputFormat()
	: CommandChangeOutputFormat("Change mesh output format", COMMAND_MESH_EXPORT_FORMAT)
{}

bool CommandChangeMeshOutputFormat::process(ccCommandLineInterface &cmd)
{
	QString defaultExt;
	QString fileFilter = getFileFormatFilter(cmd, defaultExt);
	if (fileFilter.isEmpty())
		return false;
	
	cmd.setMeshExportFormat(fileFilter, defaultExt);
	cmd.print(QObject::tr("Output export format (meshes) set to: %1").arg(defaultExt.toUpper()));
	
	//look for additional parameters
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_EXPORT_EXTENSION))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));
			
			cmd.setMeshExportFormat(cmd.meshExportFormat(), cmd.arguments().takeFirst());
			cmd.print(QObject::tr("New output extension for meshes: %1").arg(cmd.meshExportExt()));
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	return true;
}

CommandLoad::CommandLoad()
	: ccCommandLineInterface::Command("Load", COMMAND_OPEN)
{}

bool CommandLoad::process(ccCommandLineInterface &cmd)
{
	cmd.print("[LOADING]");
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: filename after \"-%1\"").arg(COMMAND_OPEN));
	
	//optional parameters
	int skipLines = 0;

	bool coordinatesShiftWasEnabled = cmd.coordinatesShiftWasEnabled();

	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_OPEN_SKIP_LINES))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: number of lines after '%1'").arg(COMMAND_OPEN_SKIP_LINES));
			}
			
			bool ok;
			skipLines = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok)
			{
				return cmd.error(QObject::tr("Invalid parameter: number of lines after '%1'").arg(COMMAND_OPEN_SKIP_LINES));
			}
			
			cmd.print(QObject::tr("Will skip %1 lines").arg(skipLines));
		}
		else if (cmd.nextCommandIsGlobalShift())
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (!cmd.processGlobalShiftCommand())
			{
				//error message already issued
				return false;
			}
		}
		else
		{
			break;
		}
	}
	
	if (skipLines > 0)
	{
		AsciiOpenDlg* openDialog = AsciiFilter::GetOpenDialog();
		assert(openDialog);
		openDialog->setSkippedLines(skipLines);
	}
	
	//open specified file
	QString filename(cmd.arguments().takeFirst());
	if (!cmd.importFile(filename))
	{
		return false;
	}

	if (!coordinatesShiftWasEnabled)
	{
		//store persistent parameters
		cmd.storeCoordinatesShiftParams();
	}

	return true;
}

CommandClearNormals::CommandClearNormals()
	: ccCommandLineInterface::Command("Clears normals", COMMAND_CLEAR_NORMALS)
{}

bool CommandClearNormals::process(ccCommandLineInterface &cmd)
{
	cmd.print("[CLEAR NORMALS]");
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No entity loaded (be sure to open at least one file with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_CLEAR_NORMALS));
	}
	
	for (const CLCloudDesc& thisCloudDesc : cmd.clouds())
	{
		ccPointCloud* cloud = thisCloudDesc.pc;
		if (cloud)
		{
			cloud->unallocateNorms();
		}
	}
	
	for (const CLMeshDesc& thisMeshDesc : cmd.meshes())
	{
		ccMesh* mesh = ccHObjectCaster::ToMesh(thisMeshDesc.mesh);
		if (!mesh)
		{
			assert(false);
			continue;
		}
		
		mesh->clearTriNormals();
		
		if (mesh->getParent()
				&& (mesh->getParent()->isA(CC_TYPES::MESH)/*|| mesh->getParent()->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
				&& ccHObjectCaster::ToMesh(mesh->getParent())->getAssociatedCloud() == mesh->getAssociatedCloud())
		{
			//Can't remove per-vertex normals on a sub mesh!
		}
		else
		{
			//mesh is alone, we can freely remove normals
			if (mesh->getAssociatedCloud() && mesh->getAssociatedCloud()->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(mesh->getAssociatedCloud())->unallocateNorms();
			}
		}
	}
	
	return true;
}

CommandOctreeNormal::CommandOctreeNormal()
	: ccCommandLineInterface::Command("Compute normals with octree", COMMAND_COMPUTE_OCTREE_NORMALS)
{}

bool CommandOctreeNormal::process(ccCommandLineInterface &cmd)
{
	cmd.print("[OCTREE NORMALS CALCULATION]");
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud to normal calculation (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_COMPUTE_OCTREE_NORMALS));
	}
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: radius after \"-%1\"").arg(COMMAND_COMPUTE_OCTREE_NORMALS));
	}
	
	bool ok;
	float radius = cmd.arguments().takeFirst().toFloat(&ok);
	if (!ok)
	{
		return cmd.error(QObject::tr("Invalid radius"));
	}
	
	cmd.print(QObject::tr("\tRadius: %1").arg(radius));
	
	CC_LOCAL_MODEL_TYPES model = QUADRIC;
	ccNormalVectors::Orientation  orientation = ccNormalVectors::Orientation::UNDEFINED;
	
	while (!cmd.arguments().isEmpty())
	{
		auto arg = cmd.arguments().front().toUpper();
		if (arg.left(6) == "ORIENT")
		{
			cmd.arguments().takeFirst();
			if (!cmd.arguments().isEmpty())
			{
				QString orient_argument = cmd.arguments().takeFirst().toUpper();
				if (orient_argument == "PLUS_ZERO")
					orientation = ccNormalVectors::Orientation::PLUS_ZERO;
				else if (orient_argument == "MINUS_ZERO")
					orientation = ccNormalVectors::Orientation::MINUS_ZERO;
				else if (orient_argument == "PLUS_BARYCENTER")
					orientation = ccNormalVectors::Orientation::PLUS_BARYCENTER;
				else if (orient_argument == "MINUS_BARYCENTER")
					orientation = ccNormalVectors::Orientation::MINUS_BARYCENTER;
				else if (orient_argument == "PLUS_X")
					orientation = ccNormalVectors::Orientation::PLUS_X;
				else if (orient_argument == "MINUS_X")
					orientation = ccNormalVectors::Orientation::MINUS_X;
				else if (orient_argument == "PLUS_Y")
					orientation = ccNormalVectors::Orientation::PLUS_Y;
				else if (orient_argument == "MINUS_Y")
					orientation = ccNormalVectors::Orientation::MINUS_Y;
				else if (orient_argument == "PLUS_Z")
					orientation = ccNormalVectors::Orientation::PLUS_Z;
				else if (orient_argument == "MINUS_Z")
					orientation = ccNormalVectors::Orientation::MINUS_Z;
				else if (orient_argument == "PREVIOUS")
					orientation = ccNormalVectors::Orientation::PREVIOUS;
				else
					return cmd.error(QObject::tr("Invalid parameter: unknown orientation '%1'").arg(orient_argument));
			}
			else
			{
				return cmd.error(QObject::tr("Missing orientation"));
			}
		}
		else if (arg == "MODEL")
		{
			cmd.arguments().takeFirst();
			if (!cmd.arguments().isEmpty())
			{
				QString model_arg = cmd.arguments().takeFirst().toUpper();
				if (model_arg == "LS")
					model = CC_LOCAL_MODEL_TYPES::LS;
				else if (model_arg == "TRI")
					model = CC_LOCAL_MODEL_TYPES::TRI;
				else if (model_arg == "QUADRIC")
					model = CC_LOCAL_MODEL_TYPES::QUADRIC;
				else
					return cmd.error(QObject::tr("Invalid parameter: unknown model '%1'").arg(model_arg));
			}
			else
			{
				return cmd.error(QObject::tr("Missing model"));
			}
		}
		else
		{
			break;
		}
	}
	
	for (const CLCloudDesc& thisCloudDesc : cmd.clouds())
	{
		ccPointCloud* cloud = thisCloudDesc.pc;
		cmd.print("computeNormalsWithOctree started...\n");
		bool success = cloud->computeNormalsWithOctree(model, orientation, radius, nullptr);
		if(success)
		{
			cmd.print("computeNormalsWithOctree success");
			cmd.print(QObject::tr("cloud->hasNormals: %1").arg(cloud->hasNormals()));
		}
		else
		{
			return cmd.error("computeNormalsWithOctree failed");
		}
		
		cloud->setName(cloud->getName() + QObject::tr(".OctreeNormal"));
		if (cmd.autoSaveMode())
		{
			CLCloudDesc cloudDesc(cloud, thisCloudDesc.basename, thisCloudDesc.path, thisCloudDesc.indexInFile);
			QString errorStr = cmd.exportEntity(cloudDesc, "OCTREE_NORMALS");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}
	
	return true;
}

CommandSubsample::CommandSubsample()
	: ccCommandLineInterface::Command("Subsample", COMMAND_SUBSAMPLE)
{}

bool CommandSubsample::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SUBSAMPLING]");
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud to resample (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SUBSAMPLE));
	}
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: resampling method after \"-%1\"").arg(COMMAND_SUBSAMPLE));
	}
	
	QString method = cmd.arguments().takeFirst().toUpper();
	cmd.print(QObject::tr("\tMethod: ") + method);
	if (method == "RANDOM")
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: number of points after \"-%1 RANDOM\"").arg(COMMAND_SUBSAMPLE));
		}
		
		bool ok;
		unsigned count = cmd.arguments().takeFirst().toUInt(&ok);
		if (!ok)
		{
			return cmd.error("Invalid number of points for random resampling!");
		}
		cmd.print(QObject::tr("\tOutput points: %1").arg(count));
		
		for (size_t i = 0; i < cmd.clouds().size(); ++i)
		{
			ccPointCloud* cloud = cmd.clouds()[i].pc;
			cmd.print(QObject::tr("\tProcessing cloud #%1 (%2)").arg(i + 1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));
			
			CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::subsampleCloudRandomly(cloud, count, cmd.progressDialog());
			if (!refCloud)
			{
				return cmd.error("Subsampling process failed!");
			}
			cmd.print(QObject::tr("\tResult: %1 points").arg(refCloud->size()));
			
			//save output
			ccPointCloud* result = cloud->partialClone(refCloud);
			delete refCloud;
			refCloud = nullptr;
			
			if (result)
			{
				result->setName(cmd.clouds()[i].pc->getName() + QObject::tr(".subsampled"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc cloudDesc(result, cmd.clouds()[i].basename, cmd.clouds()[i].path, cmd.clouds()[i].indexInFile);
					QString errorStr = cmd.exportEntity(cloudDesc, "RANDOM_SUBSAMPLED");
					if (!errorStr.isEmpty())
					{
						delete result;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete cmd.clouds()[i].pc;
				cmd.clouds()[i].pc = result;
				cmd.clouds()[i].basename += QObject::tr("_SUBSAMPLED");
				//delete result;
				//result = 0;
			}
			else
			{
				return cmd.error("Not enough memory!");
			}
		}
	}
	else if (method == "SPATIAL")
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: spatial step after \"-%1 SPATIAL\"").arg(COMMAND_SUBSAMPLE));
		}
		bool ok;
		double step = cmd.arguments().takeFirst().toDouble(&ok);
		if (!ok || step <= 0)
		{
			return cmd.error("Invalid step value for spatial resampling!");
		}
		cmd.print(QObject::tr("\tSpatial step: %1").arg(step));
		
		for (size_t i = 0; i < cmd.clouds().size(); ++i)
		{
			ccPointCloud* cloud = cmd.clouds()[i].pc;
			cmd.print(QObject::tr("\tProcessing cloud #%1 (%2)").arg(i + 1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));
			
			CCLib::CloudSamplingTools::SFModulationParams modParams(false);
			CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::resampleCloudSpatially(cloud, static_cast<PointCoordinateType>(step), modParams, nullptr, cmd.progressDialog());
			if (!refCloud)
			{
				return cmd.error("Subsampling process failed!");
			}
			cmd.print(QObject::tr("\tResult: %1 points").arg(refCloud->size()));
			
			//save output
			ccPointCloud* result = cloud->partialClone(refCloud);
			delete refCloud;
			refCloud = nullptr;
			
			if (result)
			{
				result->setName(cmd.clouds()[i].pc->getName() + QObject::tr(".subsampled"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc cloudDesc(result, cmd.clouds()[i].basename, cmd.clouds()[i].path, cmd.clouds()[i].indexInFile);
					QString errorStr = cmd.exportEntity(cloudDesc, "SPATIAL_SUBSAMPLED");
					if (!errorStr.isEmpty())
					{
						delete result;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete cmd.clouds()[i].pc;
				cmd.clouds()[i].pc = result;
				cmd.clouds()[i].basename += QObject::tr("_SUBSAMPLED");
				//delete result;
				//result = 0;
			}
			else
			{
				return cmd.error("Not enough memory!");
			}
		}
	}
	else if (method == "OCTREE")
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: octree level after \"-%1 OCTREE\"").arg(COMMAND_SUBSAMPLE));
		}
		
		bool ok = false;
		int octreeLevel = cmd.arguments().takeFirst().toInt(&ok);
		if (!ok || octreeLevel < 1 || octreeLevel > CCLib::DgmOctree::MAX_OCTREE_LEVEL)
		{
			return cmd.error("Invalid octree level!");
		}
		cmd.print(QObject::tr("\tOctree level: %1").arg(octreeLevel));
		
		QScopedPointer<ccProgressDialog> progressDialog(nullptr);
		if (!cmd.silentMode())
		{
			progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
			progressDialog->setAutoClose(false);
		}
		
		for (size_t i = 0; i < cmd.clouds().size(); ++i)
		{
			ccPointCloud* cloud = cmd.clouds()[i].pc;
			cmd.print(QObject::tr("\tProcessing cloud #%1 (%2)").arg(i + 1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));
			
			CCLib::ReferenceCloud* refCloud = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																											static_cast<unsigned char>(octreeLevel),
																											CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																											progressDialog.data());
			if (!refCloud)
			{
				return cmd.error("Subsampling process failed!");
			}
			cmd.print(QObject::tr("\tResult: %1 points").arg(refCloud->size()));
			
			//save output
			ccPointCloud* result = cloud->partialClone(refCloud);
			delete refCloud;
			refCloud = nullptr;
			
			if (result)
			{
				result->setName(cmd.clouds()[i].pc->getName() + QObject::tr(".subsampled"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc cloudDesc(result, cmd.clouds()[i].basename, cmd.clouds()[i].path, cmd.clouds()[i].indexInFile);
					QString errorStr = cmd.exportEntity(cloudDesc, QObject::tr("OCTREE_LEVEL_%1_SUBSAMPLED").arg(octreeLevel));
					if (!errorStr.isEmpty())
					{
						delete result;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete cmd.clouds()[i].pc;
				cmd.clouds()[i].pc = result;
				cmd.clouds()[i].basename += QObject::tr("_SUBSAMPLED");
				//delete result;
				//result = 0;
			}
			else
			{
				return cmd.error("Not enough memory!");
			}
		}
		
		if (progressDialog)
		{
			progressDialog->close();
			QCoreApplication::processEvents();
		}
	}
	else
	{
		return cmd.error("Unknown method!");
	}
	
	return true;
}

CommandExtractCCs::CommandExtractCCs()
	: ccCommandLineInterface::Command("ExtractCCs", COMMAND_EXTRACT_CC)
{}

bool CommandExtractCCs::process(ccCommandLineInterface &cmd)
{
	cmd.print("[CONNECTED COMPONENTS EXTRACTION]");
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud loaded (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_EXTRACT_CC));
	}
	
	//octree level
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: octree level after \"-%1\"").arg(COMMAND_EXTRACT_CC));
	}
	bool ok;
	unsigned char octreeLevel = std::min<unsigned char>(cmd.arguments().takeFirst().toUShort(&ok), CCLib::DgmOctree::MAX_OCTREE_LEVEL);
	if (!ok)
	{
		return cmd.error("Invalid octree level!");
	}
	cmd.print(QObject::tr("\tOctree level: %1").arg(octreeLevel));
	
	//min number of points
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: minimum number of points per component after \"-%1 [octree level]\"").arg(COMMAND_EXTRACT_CC));
	}
	unsigned minPointCount = cmd.arguments().takeFirst().toUInt(&ok);
	if (!ok)
	{
		return cmd.error("Invalid min. number of points!");
	}
	cmd.print(QObject::tr("\tMin number of points per component: %1").arg(minPointCount));
	
	try
	{
		QScopedPointer<ccProgressDialog> progressDialog(nullptr);
		if (!cmd.silentMode())
		{
			progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
			progressDialog->setAutoClose(false);
		}
		
		std::vector< CLCloudDesc > inputClouds = cmd.clouds();
		cmd.clouds().resize(0);
		for (size_t i = 0; i < inputClouds.size(); ++i)
		{
			ccPointCloud* cloud = inputClouds[i].pc;
			cmd.print(QObject::tr("\tProcessing cloud #%1 (%2)").arg(i + 1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));
			
			//we create/activate CCs label scalar field
			int sfIdx = cloud->getScalarFieldIndexByName(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			if (sfIdx < 0)
			{
				sfIdx = cloud->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			}
			if (sfIdx < 0)
			{
				cmd.error("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ...");
				continue;
			}
			cloud->setCurrentScalarField(sfIdx);
			
			//try to label all CCs
			int componentCount = CCLib::AutoSegmentationTools::labelConnectedComponents(cloud,
																						static_cast<unsigned char>(octreeLevel),
																						false,
																						progressDialog.data());
			
			if (componentCount == 0)
			{
				cmd.error("No component found!");
				continue;
			}
			
			cloud->getCurrentInScalarField()->computeMinAndMax();
			CCLib::ReferenceCloudContainer components;
			bool success = CCLib::AutoSegmentationTools::extractConnectedComponents(cloud, components);
			cloud->deleteScalarField(sfIdx);
			sfIdx = -1;
			
			if (!success)
			{
				cmd.warning("An error occurred (failed to finish the extraction)");
				continue;
			}
			
			//we create "real" point clouds for all input components
			int realIndex = 0;
			for (size_t j = 0; j < components.size(); ++j)
			{
				CCLib::ReferenceCloud* compIndexes = components[j];
				
				//if it has enough points
				if (compIndexes->size() >= minPointCount)
				{
					//we create a new entity
					ccPointCloud* compCloud = cloud->partialClone(compIndexes);
					if (compCloud)
					{
						//'shift on load' information
						compCloud->setGlobalShift(cloud->getGlobalShift());
						compCloud->setGlobalScale(cloud->getGlobalScale());
						compCloud->setName(QString(cloud->getName() + "_CC#%1").arg(j + 1));
						
						CLCloudDesc cloudDesc(compCloud, inputClouds[i].basename + QObject::tr("_COMPONENT_%1").arg(++realIndex), inputClouds[i].path);
						if (cmd.autoSaveMode())
						{
							QString errorStr = cmd.exportEntity(cloudDesc, QString(), nullptr, false, true);
							if (!errorStr.isEmpty())
							{
								cmd.error(errorStr);
							}
						}
						//add cloud to the current pool
						cmd.clouds().push_back(cloudDesc);
					}
					else
					{
						cmd.warning(QObject::tr("Failed to create component #%1! (not enough memory)").arg(j + 1));
					}
				}
				
				delete compIndexes;
				compIndexes = nullptr;
			}
			
			components.clear();
			
			if (cmd.clouds().empty())
			{
				cmd.error("No component was created! Check the minimum size...");
			}
			else
			{
				cmd.print(QObject::tr("%1 component(s) were created").arg(cmd.clouds().size()));
			}
		}
		
		if (progressDialog)
		{
			progressDialog->close();
			QCoreApplication::processEvents();
		}
	}
	catch (const std::bad_alloc&)
	{
		cmd.error("Not enough memory");
		return false;
	}
	
	return true;
}

CommandCurvature::CommandCurvature()
	: ccCommandLineInterface::Command("Curvature", COMMAND_CURVATURE)
{}

bool CommandCurvature::process(ccCommandLineInterface &cmd)
{
	cmd.print("[CURVATURE]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: curvature type after \"-%1\"").arg(COMMAND_CURVATURE));
	
	QString curvTypeStr = cmd.arguments().takeFirst().toUpper();
	CCLib::Neighbourhood::CurvatureType curvType = CCLib::Neighbourhood::MEAN_CURV;
	if (curvTypeStr == "MEAN")
	{
		//curvType = CCLib::Neighbourhood::MEAN_CURV;
	}
	else if (curvTypeStr == "GAUSS")
	{
		curvType = CCLib::Neighbourhood::GAUSSIAN_CURV;
	}
	else if (curvTypeStr == "NORMAL_CHANGE")
	{
		curvType = CCLib::Neighbourhood::NORMAL_CHANGE_RATE;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid curvature type after \"-%1\". Got '%2' instead of MEAN or GAUSS.").arg(COMMAND_CURVATURE, curvTypeStr));
	}
	
	if (cmd.arguments().empty())
		return cmd.error("Missing parameter: kernel size after curvature type");
	
	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
		return cmd.error(QObject::tr("Failed to read a numerical parameter: kernel size (after curvature type). Got '%1' instead.").arg(kernelStr));
	cmd.print(QObject::tr("\tKernel size: %1").arg(kernelSize));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to compute curvature! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_CURVATURE));
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
		entities[i] = cmd.clouds()[i].pc;
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCLib::GeometricalAnalysisTools::Curvature, curvType, kernelSize, entities, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("%1_CURVATURE_KERNEL_%2").arg(curvTypeStr).arg(kernelSize)))
			return false;
	}
	return true;
}

static bool ReadDensityType(ccCommandLineInterface& cmd, CCLib::GeometricalAnalysisTools::Density& density)
{
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
	
	//read option confirmed, we can move on
	QString typeArg = cmd.arguments().takeFirst().toUpper();
	if (typeArg == "KNN")
	{
		density = CCLib::GeometricalAnalysisTools::DENSITY_KNN;
	}
	else if (typeArg == "SURFACE")
	{
		density = CCLib::GeometricalAnalysisTools::DENSITY_2D;
	}
	else if (typeArg == "VOLUME")
	{
		density = CCLib::GeometricalAnalysisTools::DENSITY_3D;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid parameter: density type is expected after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
	}

	return true;
}

CommandApproxDensity::CommandApproxDensity()
	: ccCommandLineInterface::Command("ApproxDensity", COMMAND_APPROX_DENSITY)
{}

bool CommandApproxDensity::process(ccCommandLineInterface &cmd)
{
	cmd.print("[APPROX DENSITY]");
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to compute approx. density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_APPROX_DENSITY));
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
		entities[i] = cmd.clouds()[i].pc;
	
	//optional parameter: density type
	CCLib::GeometricalAnalysisTools::Density densityType = CCLib::GeometricalAnalysisTools::DENSITY_3D;
	if (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_DENSITY_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
			//read option confirmed, we can move on
			if (!ReadDensityType(cmd, densityType))
				return false;
		}
	}
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCLib::GeometricalAnalysisTools::ApproxLocalDensity, densityType, 0, entities, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds("APPROX_DENSITY"))
			return false;
	}
	
	return true;
}

CommandDensity::CommandDensity()
	: ccCommandLineInterface::Command("Density", COMMAND_DENSITY)
{}

bool CommandDensity::process(ccCommandLineInterface &cmd)
{
	cmd.print("[DENSITY]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: sphere radius after \"-%1\"").arg(COMMAND_DENSITY));
	
	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
		return cmd.error(QObject::tr("Failed to read a numerical parameter: sphere radius (after \"-%1\"). Got '%2' instead.").arg(COMMAND_DENSITY, kernelStr));
	cmd.print(QObject::tr("\tSphere radius: %1").arg(kernelSize));
	
	//optional parameter: density type
	CCLib::GeometricalAnalysisTools::Density densityType = CCLib::GeometricalAnalysisTools::DENSITY_3D;
	if (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_DENSITY_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
			//read option confirmed, we can move on
			if (!ReadDensityType(cmd, densityType))
				return false;
		}
	}
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to compute density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_DENSITY));
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
		entities[i] = cmd.clouds()[i].pc;
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCLib::GeometricalAnalysisTools::LocalDensity, densityType, kernelSize, entities, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds("DENSITY"))
			return false;
	}
	
	return true;
}

CommandSFGradient::CommandSFGradient()
	: ccCommandLineInterface::Command("SF gradient", COMMAND_SF_GRADIENT)
{}

bool CommandSFGradient::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SF GRADIENT]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: boolean (whether SF is euclidean or not) after \"-%1\"").arg(COMMAND_SF_GRADIENT));
	
	QString euclideanStr = cmd.arguments().takeFirst().toUpper();
	bool euclidean = false;
	if (euclideanStr == "TRUE")
	{
		euclidean = true;
	}
	else if (euclideanStr != "FALSE")
	{
		return cmd.error(QObject::tr("Invalid boolean value after \"-%1\". Got '%2' instead of TRUE or FALSE.").arg(COMMAND_SF_GRADIENT, euclideanStr));
	}
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to compute SF gradient! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SF_GRADIENT));
	
	//Call MainWindow generic method
	void* additionalParameters[1] = { &euclidean };
	ccHObject::Container entities;
	entities.reserve(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		unsigned sfCount = cmd.clouds()[i].pc->getNumberOfScalarFields();
		if (sfCount == 0)
		{
			cmd.warning(QObject::tr("cmd.warning: cloud '%1' has no scalar field (it will be ignored)").arg(cmd.clouds()[i].pc->getName()));
		}
		else
		{
			if (sfCount > 1)
				cmd.warning(QObject::tr("cmd.warning: cloud '%1' has several scalar fields (the active one will be used by default, or the first one if none is active)").arg(cmd.clouds()[i].pc->getName()));
			
			int activeSFIndex = cmd.clouds()[i].pc->getCurrentOutScalarFieldIndex();
			if (activeSFIndex < 0)
				activeSFIndex = 0;
			
			cmd.clouds()[i].pc->setCurrentDisplayedScalarField(activeSFIndex);
			
			entities.push_back(cmd.clouds()[i].pc);
		}
	}
	
	if (ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_SF_GRADIENT, entities, cmd.widgetParent(), additionalParameters))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(euclidean ? "EUCLIDEAN_SF_GRAD" : "SF_GRAD"))
			return false;
	}
	
	return true;
}

CommandRoughness::CommandRoughness()
	: ccCommandLineInterface::Command("Roughness", COMMAND_ROUGHNESS)
{}

bool CommandRoughness::process(ccCommandLineInterface &cmd)
{
	cmd.print("[ROUGHNESS]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: kernel size after \"-%1\"").arg(COMMAND_ROUGHNESS));
	
	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
		return cmd.error(QObject::tr("Failed to read a numerical parameter: kernel size (after \"-%1\"). Got '%2' instead.").arg(COMMAND_ROUGHNESS, kernelStr));
	cmd.print(QObject::tr("\tKernel size: %1").arg(kernelSize));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to compute roughness! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_ROUGHNESS));
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
		entities[i] = cmd.clouds()[i].pc;
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCLib::GeometricalAnalysisTools::Roughness, 0, kernelSize, entities, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("ROUGHNESS_KERNEL_%2").arg(kernelSize)))
			return false;
	}
	
	return true;
}

CommandApplyTransformation::CommandApplyTransformation()
	: ccCommandLineInterface::Command("Apply Transformation", COMMAND_APPLY_TRANSFORMATION)
{}

bool CommandApplyTransformation::process(ccCommandLineInterface &cmd)
{
	cmd.print("[APPLY TRANSFORMATION]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: transformation file after \"-%1\"").arg(COMMAND_APPLY_TRANSFORMATION));
	
	QString filename = cmd.arguments().takeFirst();
	ccGLMatrix mat;
	if (!mat.fromAsciiFile(filename))
		return cmd.error(QObject::tr("Failed to read transformation matrix file '%1'!").arg(filename));
	
	cmd.print(QObject::tr("Transformation:\n") + mat.toString(6));
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
		return cmd.error(QObject::tr("No entity on which to apply the transformation! (be sure to open one with \"-%1 [filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_APPLY_TRANSFORMATION));
	
	//apply transformation
	if (!cmd.clouds().empty())
	{
		for (const CLCloudDesc& desc : cmd.clouds())
		{
			desc.pc->applyGLTransformation_recursive(&mat);
			desc.pc->setName(desc.pc->getName() + ".transformed");
		}
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds("TRANSFORMED"))
			return false;
	}
	if (!cmd.meshes().empty())
	{
		for (const CLMeshDesc& desc : cmd.meshes())
		{
			desc.mesh->applyGLTransformation_recursive(&mat);
			desc.mesh->setName(desc.mesh->getName() + ".transformed");
		}
		//save output
		if (cmd.autoSaveMode() && !cmd.saveMeshes("TRANSFORMED"))
			return false;
	}
	
	return true;
}

CommandDropGlobalShift::CommandDropGlobalShift()
	: ccCommandLineInterface::Command("Drop global shift", COMMAND_DROP_GLOBAL_SHIFT)
{}

bool CommandDropGlobalShift::process(ccCommandLineInterface &cmd)
{
	cmd.print("[DROP GLOBAL SHIFT]");
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
		return cmd.error(QObject::tr("No loaded entity! (be sure to open one with \"-%1 [filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_DROP_GLOBAL_SHIFT));
	
	//process clouds
	for (const CLCloudDesc& desc : cmd.clouds())
	{
		desc.pc->setGlobalShift(0, 0, 0);
	}
	
	for (const CLMeshDesc& desc : cmd.meshes())
	{
		bool isLocked = false;
		ccShiftedObject* shifted = ccHObjectCaster::ToShifted(desc.mesh, &isLocked);
		if (shifted && !isLocked)
		{
			shifted->setGlobalShift(0, 0, 0);
		}
	}
	
	return true;
}

CommandSFColorScale::CommandSFColorScale()
	: ccCommandLineInterface::Command("SF color scale", COMMAND_SF_COLOR_SCALE)
{}

bool CommandSFColorScale::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SF COLOR SCALE]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: color scale file after \"-%1\"").arg(COMMAND_SF_COLOR_SCALE));
	
	QString filename = cmd.arguments().takeFirst();
	
	ccColorScale::Shared scale = ccColorScale::LoadFromXML(filename);
	
	if (!scale)
		return cmd.error(QObject::tr("Failed to read color scale file '%1'!").arg(filename));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to change the SF color scale! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SF_COLOR_SCALE));
	
	for (auto &cloud : cmd.clouds())
	{
		ccScalarField* sf = static_cast<ccScalarField*>(cloud.pc->getCurrentOutScalarField());
		if (sf)
		{
			sf->setColorScale(scale);
		}
	}
	
	if (cmd.autoSaveMode() && !cmd.saveClouds("COLOR_SCALE"))
		return false;
	
	return true;
}

CommandSFConvertToRGB::CommandSFConvertToRGB()
	: ccCommandLineInterface::Command("SF convert to RGB", COMMAND_SF_CONVERT_TO_RGB)
{}

bool CommandSFConvertToRGB::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SF CONVERT TO RGB]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: boolean (whether to mix with existing colors or not) after \"-%1\"").arg(COMMAND_SF_CONVERT_TO_RGB));
	
	QString mixWithExistingColorsStr = cmd.arguments().takeFirst().toUpper();
	bool mixWithExistingColors = false;
	if (mixWithExistingColorsStr == "TRUE")
	{
		mixWithExistingColors = true;
	}
	else if (mixWithExistingColorsStr != "FALSE")
	{
		return cmd.error(QObject::tr("Invalid boolean value after \"-%1\". Got '%2' instead of TRUE or FALSE.").arg(COMMAND_SF_CONVERT_TO_RGB, mixWithExistingColorsStr));
	}
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to convert SF to RGB! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SF_CONVERT_TO_RGB));
	
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* pc = cmd.clouds()[i].pc;
		unsigned sfCount = pc->getNumberOfScalarFields();
		int activeSFIndex = pc->getCurrentOutScalarFieldIndex();
		
		if (sfCount == 0)
		{
			cmd.warning(QObject::tr("cmd.warning: cloud '%1' has no scalar field (it will be ignored)").arg(pc->getName()));
		}
		else if (activeSFIndex < 0)
		{
			cmd.warning(QObject::tr("cmd.warning: cloud '%1' has no active scalar field (it will be ignored)").arg(pc->getName()));
		}
		else
		{
			int displaySFIndex = pc->getCurrentDisplayedScalarFieldIndex();
			pc->setCurrentDisplayedScalarField(activeSFIndex);
			
			if (pc->setRGBColorWithCurrentScalarField(mixWithExistingColors))
			{
				pc->showColors(true);
				pc->showSF(false);
			}
			else
			{
				cmd.warning(QObject::tr("cmd.warning: cloud '%1' failed to convert SF to RGB").arg(pc->getName()));
			}
			
			pc->setCurrentDisplayedScalarField(displaySFIndex);
		}
	}
	
	if (cmd.autoSaveMode() && !cmd.saveClouds("SF_CONVERT_TO_RGB"))
		return false;
	
	return true;
}

CommandFilterBySFValue::CommandFilterBySFValue()
	: ccCommandLineInterface::Command("Filter by SF value", COMMAND_FILTER_SF_BY_VALUE)
{}

bool CommandFilterBySFValue::process(ccCommandLineInterface &cmd)
{
	cmd.print("[FILTER BY VALUE]");
	
	USE_SPECIAL_SF_VALUE useValForMin = USE_NONE;
	ScalarType minVal = 0;
	QString minValStr;
	{
		if (cmd.arguments().empty())
			return cmd.error(QObject::tr("Missing parameter: min value after \"-%1\"").arg(COMMAND_FILTER_SF_BY_VALUE));
		
		bool paramOk = false;
		minValStr = cmd.arguments().takeFirst();
		if (minValStr.toUpper() == "MIN")
		{
			useValForMin = USE_MIN;
		}
		else if (minValStr.toUpper() == "DISP_MIN")
		{
			useValForMin = USE_DISP_MIN;
		}
		else if (minValStr.toUpper() == "SAT_MIN")
		{
			useValForMin = USE_SAT_MIN;
		}
		else
		{
			minVal = static_cast<ScalarType>(minValStr.toDouble(&paramOk));
			if (!paramOk)
				return cmd.error(QObject::tr("Failed to read a numerical parameter: min value (after \"-%1\"). Got '%2' instead.").arg(COMMAND_FILTER_SF_BY_VALUE, minValStr));
		}
	}
	
	USE_SPECIAL_SF_VALUE useValForMax = USE_NONE;
	ScalarType maxVal = 0;
	QString maxValStr;
	{
		if (cmd.arguments().empty())
			return cmd.error(QObject::tr("Missing parameter: max value after \"-%1\" {min}").arg(COMMAND_FILTER_SF_BY_VALUE));
		
		bool paramOk = false;
		maxValStr = cmd.arguments().takeFirst();
		if (maxValStr.toUpper() == "MAX")
		{
			useValForMax = USE_MAX;
		}
		else if (maxValStr.toUpper() == "DISP_MAX")
		{
			useValForMax = USE_DISP_MAX;
		}
		else if (maxValStr.toUpper() == "SAT_MAX")
		{
			useValForMax = USE_SAT_MAX;
		}
		else
		{
			maxVal = static_cast<ScalarType>(maxValStr.toDouble(&paramOk));
			if (!paramOk)
				return cmd.error(QObject::tr("Failed to read a numerical parameter: max value (after min value). Got '%1' instead.").arg(COMMAND_FILTER_SF_BY_VALUE, maxValStr));
		}
	}
	
	cmd.print(QObject::tr("\tInterval: [%1 - %2]").arg(minValStr, maxValStr));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud on which to filter SF! (be sure to open one or generate one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_FILTER_SF_BY_VALUE));
	
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		CCLib::ScalarField* sf = cmd.clouds()[i].pc->getCurrentOutScalarField();
		if (sf)
		{
			ScalarType thisMinVal = minVal;
			{
				switch (useValForMin)
				{
					case USE_MIN:
						thisMinVal = sf->getMin();
						break;
					case USE_DISP_MIN:
						thisMinVal = static_cast<ccScalarField*>(sf)->displayRange().start();
						break;
					case USE_SAT_MIN:
						thisMinVal = static_cast<ccScalarField*>(sf)->saturationRange().start();
						break;
					default:
						//nothing to do
						break;
				}
			}
			
			ScalarType thisMaxVal = maxVal;
			{
				switch (useValForMax)
				{
					case USE_MAX:
						thisMaxVal = sf->getMax();
						break;
					case USE_DISP_MAX:
						thisMaxVal = static_cast<ccScalarField*>(sf)->displayRange().stop();
						break;
					case USE_SAT_MAX:
						thisMaxVal = static_cast<ccScalarField*>(sf)->saturationRange().stop();
						break;
					default:
						//nothing to do
						break;
				}
			}
			
			ccPointCloud* fitleredCloud = cmd.clouds()[i].pc->filterPointsByScalarValue(thisMinVal, thisMaxVal);
			if (fitleredCloud)
			{
				cmd.print(QObject::tr("\t\tCloud '%1' --> %2/%3 points remaining").arg(cmd.clouds()[i].pc->getName()).arg(fitleredCloud->size()).arg(cmd.clouds()[i].pc->size()));
				
				CLCloudDesc resultDesc(fitleredCloud, cmd.clouds()[i].basename, cmd.clouds()[i].path, cmd.clouds()[i].indexInFile);
				//replace current cloud by this one
				delete cmd.clouds()[i].pc;
				cmd.clouds()[i].pc = fitleredCloud;
				cmd.clouds()[i].basename += QObject::tr("_FILTERED_[%1_%2]").arg(thisMinVal).arg(thisMaxVal);
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(resultDesc);
					if (!errorStr.isEmpty())
					{
						delete fitleredCloud;
						return cmd.error(errorStr);
					}
				}
			}
		}
	}
	
	return true;
}

CommandComputeMeshVolume::CommandComputeMeshVolume()
	: ccCommandLineInterface::Command("Compute mesh volume", COMMAND_MESH_VOLUME)
{}

bool CommandComputeMeshVolume::process(ccCommandLineInterface &cmd)
{
	cmd.print("[COMPUTE MESH VOLUME]");
	
	if (cmd.meshes().empty())
	{
		cmd.warning("No mesh loaded! Nothing to do...");
		return true;
	}
	
	//optional parameters
	QString outputFilename;
	if (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_VOLUME_TO_FILE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (!cmd.arguments().empty())
			{
				outputFilename = cmd.arguments().front();
				cmd.arguments().pop_front();
				cmd.print("Volume report file: " + outputFilename);
			}
			else
			{
				return cmd.error(QObject::tr("Missing argument: filename after '%1'").arg(COMMAND_VOLUME_TO_FILE));
			}
		}
	}
	
	QFile outFile;
	QTextStream outStream(&outFile);
	if (!outputFilename.isEmpty())
	{
		outFile.setFileName(outputFilename);
		if (!outFile.open(QFile::WriteOnly | QFile::Text))
		{
			return cmd.error("Failed to create/open volume report file");
		}
	}
	
	//for each meshe
	for (CLMeshDesc& meshDesc : cmd.meshes())
	{
		//we compute the mesh volume
		double V = CCLib::MeshSamplingTools::computeMeshVolume(meshDesc.mesh);
		
		QString titleStr = QObject::tr("Mesh '%1'").arg(meshDesc.basename);
		if (meshDesc.indexInFile >= 0)
		{
			titleStr += QObject::tr(" (#%2)").arg(meshDesc.indexInFile);
		}
		cmd.print(titleStr);
		QString volumeStr = QObject::tr("V = %2").arg(V, 0, 'f', 8);
		cmd.print(volumeStr);
		
		if (outFile.isOpen())
		{
			outStream << titleStr << endl;
			outStream << volumeStr << endl;
		}
	}
	
	return true;
}

CommandMergeMeshes::CommandMergeMeshes()
	: ccCommandLineInterface::Command("Merge meshes", COMMAND_MERGE_MESHES)
{}

bool CommandMergeMeshes::process(ccCommandLineInterface &cmd)
{
	cmd.print("[MERGE MESHES]");
	
	if (cmd.meshes().size() < 2)
	{
		cmd.warning("Less than 2 meshes are loaded! Nothing to do...");
		return true;
	}
	
	CLMeshDesc mergedMeshDesc;
	bool firstValidMesh = true;
	
	//create the destination mesh
	ccPointCloud* vertices = new ccPointCloud("vertices");
	QScopedPointer<ccMesh> mergedMesh(new ccMesh(vertices));
	mergedMesh->setName("Merged mesh");
	mergedMesh->addChild(vertices);
	vertices->setEnabled(false);
	
	//merge meshes
	for (CLMeshDesc& meshDesc : cmd.meshes())
	{
		//get the mesh
		ccMesh* mesh = dynamic_cast<ccMesh*>(meshDesc.mesh);
		if (!mesh)
		{
			ccLog::Error(QObject::tr("Can't merge mesh '%1' (unhandled type)").arg(meshDesc.basename));
		}
		
		if (mergedMesh->merge(mesh, true)) //merge it
		{
			if (firstValidMesh)
			{
				//copy the first valid mesh description
				mergedMeshDesc = meshDesc;
				mergedMeshDesc.mesh = nullptr;
				firstValidMesh = false;
			}
		}
		else
		{
			return cmd.error("Merge operation failed");
		}
		
		delete meshDesc.mesh;
		meshDesc.mesh = nullptr;
	}
	
	if (mergedMesh->size() == 0)
	{
		return cmd.error("Result is empty");
	}
	
	//clean the 'cmd.meshes()' vector
	cmd.removeMeshes();
	//add the new mesh
	mergedMeshDesc.basename += QObject::tr("_MERGED");
	mergedMeshDesc.mesh = mergedMesh.take();
	cmd.meshes().push_back(mergedMeshDesc);
	
	if (cmd.autoSaveMode())
	{
		QString errorStr = cmd.exportEntity(mergedMeshDesc);
		if (!errorStr.isEmpty())
			return cmd.error(errorStr);
	}
	
	return true;
}

CommandMergeClouds::CommandMergeClouds()
	: ccCommandLineInterface::Command("Merge clouds", COMMAND_MERGE_CLOUDS)
{}

bool CommandMergeClouds::process(ccCommandLineInterface &cmd)
{
	cmd.print("[MERGE CLOUDS]");
	
	if (cmd.clouds().size() < 2)
	{
		cmd.warning("Less than 2 clouds are loaded! Nothing to do...");
		return true;
	}
	
	//merge clouds
	{
		for (size_t i = 1; i < cmd.clouds().size(); ++i)
		{
			unsigned beforePts = cmd.clouds().front().pc->size();
			unsigned newPts = cmd.clouds()[i].pc->size();
			*cmd.clouds().front().pc += cmd.clouds()[i].pc;
			
			//success?
			if (cmd.clouds().front().pc->size() == beforePts + newPts)
			{
				delete cmd.clouds()[i].pc;
				cmd.clouds()[i].pc = nullptr;
			}
			else
			{
				return cmd.error("Fusion failed! (not enough memory?)");
			}
		}
	}
	
	//clean the 'cmd.clouds()' vector
	cmd.clouds().resize(1);
	//update the first one
	cmd.clouds().front().basename += QObject::tr("_MERGED");
	if (cmd.autoSaveMode())
	{
		QString errorStr = cmd.exportEntity(cmd.clouds().front());
		if (!errorStr.isEmpty())
			return cmd.error(errorStr);
	}
	return true;
}

CommandSetActiveSF::CommandSetActiveSF()
	: ccCommandLineInterface::Command("Set active SF", COMMAND_SET_ACTIVE_SF)
{}

bool CommandSetActiveSF::process(ccCommandLineInterface &cmd)
{
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: scalar field index after \"-%1\"").arg(COMMAND_SET_ACTIVE_SF));
	
	bool paramOk = false;
	QString sfIndexStr = cmd.arguments().takeFirst();
	int sfIndex = sfIndexStr.toInt(&paramOk);
	if (!paramOk)
		return cmd.error(QObject::tr("Failed to read a numerical parameter: S.F. index (after \"-%1\"). Got '%2' instead.").arg(COMMAND_SET_ACTIVE_SF, sfIndexStr));
	cmd.print(QObject::tr("Set active S.F. index: %1").arg(sfIndex));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud loaded! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SET_ACTIVE_SF));
	
	for (size_t i = 0; i<cmd.clouds().size(); ++i)
	{
		if (cmd.clouds()[i].pc && cmd.clouds()[i].pc->hasScalarFields())
		{
			if (static_cast<int>(cmd.clouds()[i].pc->getNumberOfScalarFields()) > sfIndex)
				cmd.clouds()[i].pc->setCurrentScalarField(sfIndex);
			else
				cmd.warning(QObject::tr("Cloud '%1' has less scalar fields than the index to select!").arg(cmd.clouds()[i].pc->getName()));
		}
	}
	
	return true;
}

CommandRemoveAllSF::CommandRemoveAllSF()
	: ccCommandLineInterface::Command("Remove all SF", COMMAND_REMOVE_ALL_SFS)
{}

bool CommandRemoveAllSF::process(ccCommandLineInterface &cmd)
{
	//no argument required
	for (auto &cloudDesc : cmd.clouds())
	{
		if (cloudDesc.pc/* && cmd.clouds()[i].pc->hasScalarFields()*/)
		{
			cloudDesc.pc->deleteAllScalarFields();
			cloudDesc.pc->showSF(false);
		}
	}
	
	for (auto &meshDesc : cmd.meshes())
	{
		if (meshDesc.mesh)
		{
			ccGenericPointCloud* cloud = meshDesc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(cloud)->deleteAllScalarFields();
				cloud->showSF(false);
			}
		}
	}
	
	return true;
}

CommandRemoveScanGrids::CommandRemoveScanGrids()
	: ccCommandLineInterface::Command("Remove scan grids", COMMAND_REMOVE_SCAN_GRIDS)
{}

bool CommandRemoveScanGrids::process(ccCommandLineInterface &cmd)
{
	//no argument required
	for (CLCloudDesc& cloudDesc : cmd.clouds())
	{
		if (cloudDesc.pc)
		{
			cloudDesc.pc->removeGrids();
		}
	}
	
	for (CLMeshDesc& meshDesc : cmd.meshes())
	{
		if (meshDesc.mesh)
		{
			ccGenericPointCloud* cloud = meshDesc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(cloud)->removeGrids();
			}
		}
	}
	
	return true;
}

CommandMatchBBCenters::CommandMatchBBCenters()
	: ccCommandLineInterface::Command("Match B.B. centers", COMMAND_MATCH_BB_CENTERS)
{}

bool CommandMatchBBCenters::process(ccCommandLineInterface &cmd)
{
	cmd.print("[MATCH B.B. CENTERS]");
	
	std::vector<CLEntityDesc*> entities;
	for (auto &cloud : cmd.clouds())
		entities.push_back(&cloud);
	for (auto &mesh : cmd.meshes())
		entities.push_back(&mesh);
	
	if (entities.empty())
	{
		return cmd.error("No entity loaded!");
	}
	else if (entities.size() == 1)
	{
		cmd.warning("Nothing to do: only one entity currently loaded!");
		return true;
	}
	
	CCVector3 firstCenter = entities.front()->getEntity()->getOwnBB().getCenter();
	for (size_t i = 1; i < entities.size(); ++i)
	{
		ccHObject* ent = entities[i]->getEntity();
		CCVector3 center = ent->getOwnBB().getCenter();
		CCVector3 T = firstCenter - center;
		
		//transformation (used only for translation)
		ccGLMatrix glTrans;
		glTrans += T;
		
		//apply translation matrix
		ent->applyGLTransformation_recursive(&glTrans);
		cmd.print(QObject::tr("Entity '%1' has been translated: (%2,%3,%4)").arg(ent->getName()).arg(T.x).arg(T.y).arg(T.z));
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(*entities[i]);
			if (!errorStr.isEmpty())
				return cmd.error(errorStr);
		}
	}
	
	return true;
}

CommandMatchBestFitPlane::CommandMatchBestFitPlane()
	: ccCommandLineInterface::Command("Match best fit plane", COMMAND_BEST_FIT_PLANE)
{}

bool CommandMatchBestFitPlane::process(ccCommandLineInterface &cmd)
{
	cmd.print("[COMPUTE BEST FIT PLANE]");
	
	//look for local options
	bool makeCloudsHoriz = false;
	bool keepLoaded = false;
	
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_BEST_FIT_PLANE_MAKE_HORIZ))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			makeCloudsHoriz = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_BEST_FIT_PLANE_KEEP_LOADED))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			keepLoaded = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* pc = cmd.clouds()[i].pc;
		
		//try to fit plane
		double rms = 0.0;
		ccPlane* pPlane = ccPlane::Fit(pc, &rms);
		if (pPlane)
		{
			cmd.print(QObject::tr("Plane successfully fitted: rms = %1").arg(rms));
			
			CCVector3 N = pPlane->getNormal();
			CCVector3 C = *CCLib::Neighbourhood(pc).getGravityCenter();
			
			CLMeshDesc planeDesc;
			planeDesc.mesh = pPlane;
			planeDesc.basename = cmd.clouds()[i].basename;
			planeDesc.path = cmd.clouds()[i].path;
			
			//save plane as a BIN file
			QString outputFilename;
			QString errorStr = cmd.exportEntity(planeDesc, "BEST_FIT_PLANE", &outputFilename);
			if (!errorStr.isEmpty())
				cmd.warning(errorStr);
			
			//open text file to save plane related information
			QString txtFilename = QObject::tr("%1/%2_BEST_FIT_PLANE_INFO").arg(cmd.clouds()[i].path, cmd.clouds()[i].basename);
			if (cmd.addTimestamp())
				txtFilename += QObject::tr("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));
			txtFilename += QObject::tr(".txt");
			QFile txtFile(txtFilename);
			txtFile.open(QIODevice::WriteOnly | QIODevice::Text);
			QTextStream txtStream(&txtFile);
			
			txtStream << QObject::tr("Filename: %1").arg(outputFilename) << endl;
			txtStream << QObject::tr("Fitting RMS: %1").arg(rms) << endl;
			
			//We always consider the normal with a positive 'Z' by default!
			if (N.z < 0.0)
				N *= -1.0;
			
			int precision = cmd.numericalPrecision();
			txtStream << QObject::tr("Normal: (%1,%2,%3)").arg(N.x, 0, 'f', precision).arg(N.y, 0, 'f', precision).arg(N.z, 0, 'f', precision) << endl;
			
			//we compute strike & dip by the way
			{
				PointCoordinateType dip = 0, dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				txtStream << ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir) << endl;
			}
			
			//compute the transformation matrix that would make this normal points towards +Z
			ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, PC_ONE));
			CCVector3 Gt = C;
			makeZPosMatrix.applyRotation(Gt);
			makeZPosMatrix.setTranslation(C - Gt);
			
			txtStream << "Orientation matrix:" << endl;
			txtStream << makeZPosMatrix.toString(precision, ' ') << endl;
			
			//close the text file
			txtFile.close();
			
			if (keepLoaded)
			{
				//add the resulting plane (mesh) to the main set
				cmd.meshes().push_back(planeDesc);
			}
			
			if (makeCloudsHoriz)
			{
				//apply 'horizontal' matrix
				pc->applyGLTransformation_recursive(&makeZPosMatrix);
				cmd.print(QObject::tr("Cloud '%1' has been transformed with the above matrix").arg(pc->getName()));
				cmd.clouds()[i].basename += QObject::tr("_HORIZ");
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(cmd.clouds()[i]);
					if (!errorStr.isEmpty())
						cmd.warning(errorStr);
				}
			}
		}
		else
		{
			cmd.warning(QObject::tr("Failed to compute best fit plane for cloud '%1'").arg(cmd.clouds()[i].pc->getName()));
		}
	}
	
	return true;
}

CommandOrientNormalsMST::CommandOrientNormalsMST()
	: ccCommandLineInterface::Command("Orient normals", COMMAND_ORIENT_NORMALS)
{}

bool CommandOrientNormalsMST::process(ccCommandLineInterface &cmd)
{
	cmd.print("[ORIENT NORMALS (MST)]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: number of neighbors after \"-%1\"").arg(COMMAND_ORIENT_NORMALS));
	
	QString knnStr = cmd.arguments().takeFirst();
	bool ok;
	int knn = knnStr.toInt(&ok);
	if (!ok || knn <= 0)
		return cmd.error(QObject::tr("Invalid parameter: number of neighbors (%1)").arg(knnStr));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* cloud = cmd.clouds()[i].pc;
		assert(cloud);
		
		if (!cloud->hasNormals())
		{
			continue;
		}
		
		//computation
		if (cloud->orientNormalsWithMST(knn, progressDialog.data()))
		{
			cmd.clouds()[i].basename += QObject::tr("_NORMS_REORIENTED");
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.clouds()[i]);
				if (!errorStr.isEmpty())
					cmd.warning(errorStr);
			}
		}
		else
		{
			return cmd.error(QObject::tr("Failed to orient the normals of cloud '%1'!").arg(cloud->getName()));
		}
	}
	
	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}
	
	return true;
}

CommandSORFilter::CommandSORFilter()
	: ccCommandLineInterface::Command("S.O.R. filter", COMMAND_SOR_FILTER)
{}

bool CommandSORFilter::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SOR FILTER]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: number of neighbors mode after \"-%1\"").arg(COMMAND_SOR_FILTER));
	
	QString knnStr = cmd.arguments().takeFirst();
	bool ok;
	int knn = knnStr.toInt(&ok);
	if (!ok || knn <= 0)
		return cmd.error(QObject::tr("Invalid parameter: number of neighbors (%1)").arg(knnStr));
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: sigma multiplier after number of neighbors (SOR)"));
	QString sigmaStr = cmd.arguments().takeFirst();
	double nSigma = sigmaStr.toDouble(&ok);
	if (!ok || nSigma < 0)
		return cmd.error(QObject::tr("Invalid parameter: sigma multiplier (%1)").arg(nSigma));
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* cloud = cmd.clouds()[i].pc;
		assert(cloud);
		
		//computation
		CCLib::ReferenceCloud* selection = CCLib::CloudSamplingTools::sorFilter(cloud,
																				knn,
																				nSigma,
																				nullptr,
																				progressDialog.data());
		
		if (selection)
		{
			ccPointCloud* cleanCloud = cloud->partialClone(selection);
			if (cleanCloud)
			{
				cleanCloud->setName(cloud->getName() + QObject::tr(".clean"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc cloudDesc(cleanCloud, cmd.clouds()[i].basename, cmd.clouds()[i].path, cmd.clouds()[i].indexInFile);
					QString errorStr = cmd.exportEntity(cloudDesc, "SOR");
					if (!errorStr.isEmpty())
					{
						delete cleanCloud;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete cmd.clouds()[i].pc;
				cmd.clouds()[i].pc = cleanCloud;
				cmd.clouds()[i].basename += QObject::tr("_SOR");
				//delete cleanCloud;
				//cleanCloud = 0;
			}
			else
			{
				return cmd.error(QObject::tr("Not enough memory to create a clean version of cloud '%1'!").arg(cloud->getName()));
			}
			
			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			return cmd.error(QObject::tr("Failed to apply SOR filter on cloud '%1'! (not enough memory?)").arg(cloud->getName()));
		}
	}
	
	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}
	
	return true;
}

CommandExtractVertices::CommandExtractVertices()
	: ccCommandLineInterface::Command("Extract vertices (as a standalone 'cloud')", COMMAND_EXTRACT_VERTICES)
{}

bool CommandExtractVertices::process(ccCommandLineInterface &cmd)
{
	cmd.print("[EXTRACT VERTICES]");
	
	if (cmd.meshes().empty())
	{
		cmd.warning(QObject::tr("No mesh available. Be sure to open one first!"));
		return false;
	}
	
	for (size_t i = 0; i < cmd.meshes().size(); ++i)
	{
		ccGenericMesh* mesh = cmd.meshes()[i].mesh;
		ccGenericPointCloud* cloud = mesh->getAssociatedCloud();
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cloud);
		if (!pc)
		{
			assert(false);
			continue;
		}
		
		//add the resulting cloud to the main set
		cmd.clouds().emplace_back(pc, cmd.meshes()[i].basename + QObject::tr(".vertices"), cmd.meshes()[i].path);
		
		//don't forget to detach the cloud before we delete the meshes!
		assert(pc->getParent() == mesh);
		if (pc->getParent())
		{
			pc->getParent()->detachChild(pc);
		}
		
		//save it as well
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(cmd.clouds().back());
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}
	
	cmd.removeMeshes(false);
	
	return true;
}

CommandSampleMesh::CommandSampleMesh()
	: ccCommandLineInterface::Command("Sample mesh", COMMAND_SAMPLE_MESH)
{}

bool CommandSampleMesh::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SAMPLE POINTS ON MESH]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: sampling mode after \"-%1\" (POINTS/DENSITY)").arg(COMMAND_SAMPLE_MESH));
	
	bool useDensity = false;
	double parameter = 0;
	
	QString sampleMode = cmd.arguments().takeFirst().toUpper();
	if (sampleMode == "POINTS")
		useDensity = false;
	else if (sampleMode == "DENSITY")
		useDensity = true;
	else
		return cmd.error(QObject::tr("Invalid parameter: unknown sampling mode \"%1\"").arg(sampleMode));
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: value after sampling mode"));
	bool conversionOk = false;
	parameter = cmd.arguments().takeFirst().toDouble(&conversionOk);
	if (!conversionOk)
		return cmd.error(QObject::tr("Invalid parameter: value after sampling mode"));
	
	if (cmd.meshes().empty())
		return cmd.error(QObject::tr("No mesh available. Be sure to open one first!"));
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (size_t i = 0; i < cmd.meshes().size(); ++i)
	{
		ccPointCloud* cloud = cmd.meshes()[i].mesh->samplePoints(useDensity, parameter, true, true, true, progressDialog.data());
		
		if (!cloud)
		{
			return cmd.error(QObject::tr("Cloud sampling failed!"));
		}
		
		//add the resulting cloud to the main set
		cmd.print(QObject::tr("Sampled cloud created: %1 points").arg(cloud->size()));
		cmd.clouds().emplace_back(cloud, cmd.meshes()[i].basename + QObject::tr("_SAMPLED_POINTS"), cmd.meshes()[i].path);
		
		//save it as well
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(cmd.clouds().back());
			if (!errorStr.isEmpty())
				return cmd.error(errorStr);
		}
	}
	
	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}
	
	return true;
}

CommandCrop::CommandCrop()
	: ccCommandLineInterface::Command("Crop", COMMAND_CROP)
{}

bool CommandCrop::process(ccCommandLineInterface &cmd)
{
	cmd.print("[CROP]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: box extents after \"-%1\" (Xmin:Ymin:Zmin:Xmax:Ymax:Zmax)").arg(COMMAND_CROP));
	if (cmd.clouds().empty() && cmd.meshes().empty())
		return cmd.error(QObject::tr("No point cloud or mesh available. Be sure to open or generate one first!"));
	
	//decode box extents
	CCVector3 boxMin, boxMax;
	{
		QString boxBlock = cmd.arguments().takeFirst();
		QStringList tokens = boxBlock.split(':');
		if (tokens.size() != 6)
			return cmd.error(QObject::tr("Invalid parameter: box extents (expected format is 'Xmin:Ymin:Zmin:Xmax:Ymax:Zmax')").arg(COMMAND_CROP));
		
		for (int i = 0; i < 6; ++i)
		{
			CCVector3* vec = (i < 3 ? &boxMin : &boxMax);
			bool ok = true;
			vec->u[i % 3] = static_cast<PointCoordinateType>(tokens[i].toDouble(&ok));
			if (!ok)
			{
				return cmd.error(QObject::tr("Invalid parameter: box extents (component #%1 is not a valid number)").arg(i + 1));
			}
		}
	}
	
	//optional parameters
	bool inside = true;
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_CROP_OUTSIDE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			inside = false;
		}
		else
		{
			break;
		}
	}
	
	ccBBox cropBox(boxMin, boxMax);
	//crop clouds
	{
		for (size_t i = 0; i < cmd.clouds().size(); ++i)
		{
			ccHObject* croppedCloud = ccCropTool::Crop(cmd.clouds()[i].pc, cropBox, inside);
			if (croppedCloud)
			{
				delete cmd.clouds()[i].pc;
				assert(croppedCloud->isA(CC_TYPES::POINT_CLOUD));
				cmd.clouds()[i].pc = static_cast<ccPointCloud*>(croppedCloud);
				cmd.clouds()[i].basename += "_CROPPED";
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(cmd.clouds()[i]);
					if (!errorStr.isEmpty())
						return cmd.error(errorStr);
				}
			}
			//otherwise an error message has already been issued
		}
	}
	
	//crop meshes
	{
		for (size_t i = 0; i < cmd.meshes().size(); ++i)
		{
			ccHObject* croppedMesh = ccCropTool::Crop(cmd.meshes()[i].mesh, cropBox, inside);
			if (croppedMesh)
			{
				delete cmd.meshes()[i].mesh;
				assert(croppedMesh->isA(CC_TYPES::MESH));
				cmd.meshes()[i].mesh = static_cast<ccMesh*>(croppedMesh);
				cmd.meshes()[i].basename += "_CROPPED";
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(cmd.meshes()[i]);
					if (!errorStr.isEmpty())
						return cmd.error(errorStr);
				}
			}
			//otherwise an error message has already been issued
		}
	}
	
	return true;
}

CommandCoordToSF::CommandCoordToSF()
	: ccCommandLineInterface::Command("Crop", COMMAND_COORD_TO_SF)
{}

bool CommandCoordToSF::process(ccCommandLineInterface &cmd)
{
	cmd.print("[COORD TO SF]");
	
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter after \"-%1\" (DIMENSION)").arg(COMMAND_COORD_TO_SF));
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
	
	//dimension
	bool exportDims[3] = { false, false, false };
	QString dimStr = cmd.arguments().takeFirst().toUpper();
	{
		if (dimStr == "X")
			exportDims[0] = true;
		else if (dimStr == "Y")
			exportDims[1] = true;
		else if (dimStr == "Z")
			exportDims[2] = true;
		else
			return cmd.error(QObject::tr("Invalid parameter: dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_COORD_TO_SF));
	}
	
	//now we can export the corresponding coordinate
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* pc = cmd.clouds()[i].pc;
		if (pc->exportCoordToSF(exportDims))
		{
			cmd.clouds()[i].basename += QObject::tr("_%1_TO_SF").arg(dimStr);
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.clouds()[i]);
				if (!errorStr.isEmpty())
					return cmd.error(errorStr);
			}
		}
		else
		{
			return cmd.error(QObject::tr("Failed to export coord. %1 to SF on cloud '%2'!").arg(dimStr, cmd.clouds()[i].pc->getName()));
		}
	}
	
	return true;
}

CommandCrop2D::CommandCrop2D()
	: ccCommandLineInterface::Command("Crop", COMMAND_CROP_2D)
{}

bool CommandCrop2D::process(ccCommandLineInterface &cmd)
{
	cmd.print("[CROP 2D]");
	
	if (cmd.arguments().size() < 6)
		return cmd.error(QObject::tr("Missing parameter(s) after \"-%1\" (ORTHO_DIM N X1 Y1 X2 Y2 ... XN YN)").arg(COMMAND_CROP_2D));
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
	
	//decode poyline extents
	ccPointCloud vertices("polyline.vertices");
	ccPolyline poly(&vertices);
	
	//orthogonal dimension
	unsigned char orthoDim = 2;
	{
		QString orthoDimStr = cmd.arguments().takeFirst().toUpper();
		if (orthoDimStr == "X")
			orthoDim = 0;
		else if (orthoDimStr == "Y")
			orthoDim = 1;
		else if (orthoDimStr == "Z")
			orthoDim = 2;
		else
			return cmd.error(QObject::tr("Invalid parameter: orthogonal dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_CROP_2D));
	}
	
	//number of vertices
	bool ok = true;
	unsigned N = 0;
	{
		QString countStr = cmd.arguments().takeFirst();
		N = countStr.toUInt(&ok);
		if (!ok)
			return cmd.error(QObject::tr("Invalid parameter: number of vertices for the 2D polyline after \"-%1\"").arg(COMMAND_CROP_2D));
	}
	
	//now read the vertices
	{
		if (!vertices.reserve(N)
				|| !poly.addPointIndex(0, N))
		{
			return cmd.error("Not enough memory!");
		}
		
		for (unsigned i = 0; i < N; ++i)
		{
			if (cmd.arguments().size() < 2)
			{
				return cmd.error(QObject::tr("Missing parameter(s): vertex #%1 data and following").arg(i + 1));
			}
			
			CCVector3 P(0, 0, 0);
			
			QString coordStr = cmd.arguments().takeFirst();
			P.x = static_cast<PointCoordinateType>(coordStr.toDouble(&ok));
			if (!ok)
				return cmd.error(QObject::tr("Invalid parameter: X-coordinate of vertex #%1").arg(i + 1));
			/*QString */coordStr = cmd.arguments().takeFirst();
			P.y = static_cast<PointCoordinateType>(coordStr.toDouble(&ok));
			if (!ok)
				return cmd.error(QObject::tr("Invalid parameter: Y-coordinate of vertex #%1").arg(i + 1));
			
			vertices.addPoint(P); //the polyline must be defined in the XY plane!
		}
		
		poly.setClosed(true);
	}
	
	//optional parameters
	bool inside = true;
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_CROP_OUTSIDE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			inside = false;
		}
		else
		{
			break;
		}
	}
	
	//now we can crop the loaded cloud(s)
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		CCLib::ReferenceCloud* ref = cmd.clouds()[i].pc->crop2D(&poly, orthoDim, inside);
		if (ref)
		{
			if (ref->size() != 0)
			{
				ccPointCloud* croppedCloud = cmd.clouds()[i].pc->partialClone(ref);
				delete ref;
				ref = nullptr;
				
				if (croppedCloud)
				{
					delete cmd.clouds()[i].pc;
					cmd.clouds()[i].pc = croppedCloud;
					croppedCloud->setName(cmd.clouds()[i].pc->getName() + QObject::tr(".cropped"));
					cmd.clouds()[i].basename += "_CROPPED";
					if (cmd.autoSaveMode())
					{
						QString errorStr = cmd.exportEntity(cmd.clouds()[i]);
						if (!errorStr.isEmpty())
							return cmd.error(errorStr);
					}
				}
				else
				{
					return cmd.error(QObject::tr("Not enough memory to crop cloud '%1'!").arg(cmd.clouds()[i].pc->getName()));
				}
			}
			else
			{
				delete ref;
				ref = nullptr;
				cmd.warning(QObject::tr("No point of cloud '%1' falls inside the input box!").arg(cmd.clouds()[i].pc->getName()));
			}
		}
		else
		{
			return cmd.error(QObject::tr("Crop process failed! (not enough memory)"));
		}
	}
	
	return true;
}

CommandColorBanding::CommandColorBanding()
	: ccCommandLineInterface::Command("Color banding", COMMAND_COLOR_BANDING)
{}

bool CommandColorBanding::process(ccCommandLineInterface &cmd)
{
	cmd.print("[COLOR BANDING]");
	
	if (cmd.arguments().size() < 2)
		return cmd.error(QObject::tr("Missing parameter(s) after \"-%1\" (DIM FREQUENCY)").arg(COMMAND_COLOR_BANDING));
	if (cmd.clouds().empty() && cmd.meshes().empty())
		return cmd.error(QObject::tr("No entity available. Be sure to open or generate one first!"));
	
	//dimension
	unsigned char dim = 2;
	QString dimStr = "Z";
	{
		dimStr = cmd.arguments().takeFirst().toUpper();
		if (dimStr == "X")
			dim = 0;
		else if (dimStr == "Y")
			dim = 1;
		else if (dimStr == "Z")
			dim = 2;
		else
			return cmd.error(QObject::tr("Invalid parameter: dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_COLOR_BANDING));
	}
	
	//frequency
	bool ok = true;
	double freq = 0;
	{
		QString countStr = cmd.arguments().takeFirst();
		freq = countStr.toDouble(&ok);
		if (!ok)
			return cmd.error(QObject::tr("Invalid parameter: frequency after \"-%1 DIM\" (in Hz, integer value)").arg(COMMAND_COLOR_BANDING));
	}
	
	//process clouds
	if (!cmd.clouds().empty())
	{
		for (size_t i = 0; i < cmd.clouds().size(); ++i)
		{
			if (cmd.clouds()[i].pc)
				if (!cmd.clouds()[i].pc->setRGBColorByBanding(dim, freq))
					return cmd.error("Not enough memory");
		}
		
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("COLOR_BANDING_%1_%2").arg(dimStr).arg(freq)))
			return false;
	}
	
	if (!cmd.meshes().empty())
	{
		bool hasMeshes = false;
		for (size_t i = 0; i < cmd.meshes().size(); ++i)
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(cmd.meshes()[i].mesh);
			if (cloud)
			{
				if (!cloud->setRGBColorByBanding(dim, freq))
					return cmd.error("Not enough memory");
				cmd.meshes()[i].mesh->showColors(true);
				hasMeshes = true;
			}
			else
			{
				cmd.warning(QObject::tr("Vertices of mesh '%1' are locked (they may be shared by multiple entities for instance). Can't apply the current command on them.").arg(cmd.meshes()[i].mesh->getName()));
			}
		}
		
		//save output
		if (hasMeshes && cmd.autoSaveMode() && !cmd.saveMeshes(QObject::tr("COLOR_BANDING_%1_%2").arg(dimStr).arg(freq)))
			return false;
	}
	
	return true;
}

CommandDist::CommandDist(bool cloud2meshDist, const QString& name, const QString& keyword)
	: ccCommandLineInterface::Command(name, keyword)
	, m_cloud2meshDist(cloud2meshDist)
{}

bool CommandDist::process(ccCommandLineInterface &cmd)
{
	cmd.print("[DISTANCE COMPUTATION]");
	
	//compared cloud
	CLEntityDesc* compEntity = nullptr;
	ccHObject* compCloud = nullptr;
	size_t nextMeshIndex = 0;
	if (cmd.clouds().empty())
	{
		//no cloud loaded
		if (!m_cloud2meshDist || cmd.meshes().size() < 2)
		{
			//we would need at least two meshes
			return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
		}
		else
		{
			cmd.warning(QObject::tr("No point cloud available. Will use the first mesh vertices as compared cloud."));
			compEntity = &(cmd.meshes().front());
			compCloud = dynamic_cast<ccPointCloud*>(cmd.meshes()[nextMeshIndex++].mesh->getAssociatedCloud());
			if (!compCloud)
			{
				return cmd.error(QObject::tr("Unhandled mesh vertices type"));
			}
		}
	}
	else //at least two clouds
	{
		if (m_cloud2meshDist && cmd.clouds().size() != 1)
			cmd.warning("[C2M] Multiple point clouds loaded! Will take the first one by default.");
		compEntity = &(cmd.clouds().front());
		compCloud = cmd.clouds().front().pc;
	}
	assert(compEntity && compCloud);
	
	//reference entity
	ccHObject* refEntity = nullptr;
	if (m_cloud2meshDist)
	{
		if (cmd.meshes().size() <= nextMeshIndex)
			return cmd.error(QObject::tr("No mesh available. Be sure to open one first!"));
		else if (cmd.meshes().size() != nextMeshIndex + 1)
			cmd.warning(QString("Multiple meshes loaded! We take the %1 one by default").arg(nextMeshIndex == 0 ? "first" : "second"));
		refEntity = cmd.meshes()[nextMeshIndex].mesh;
	}
	else
	{
		if (cmd.clouds().size() < 2)
			return cmd.error(QObject::tr("Only one point cloud available. Be sure to open or generate a second one before performing C2C distance!"));
		else if (cmd.clouds().size() > 2)
			cmd.warning("More than 3 point clouds loaded! We take the second one as reference by default");
		refEntity = cmd.clouds()[1].pc;
	}
	
	//inner loop for Distance computation options
	bool flipNormals = false;
	double maxDist = 0.0;
	unsigned octreeLevel = 0;
	int maxThreadCount = 0;
	
	bool splitXYZ = false;
	int modelIndex = 0;
	bool useKNN = true;
	double nSize = 0;
	
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2M_DIST_FLIP_NORMALS))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			flipNormals = true;
			
			if (!m_cloud2meshDist)
				cmd.warning("Parameter \"-%1\" ignored: only for C2M distance!");
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2X_MAX_DISTANCE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: value after \"-%1\"").arg(COMMAND_C2X_MAX_DISTANCE));
			bool conversionOk = false;
			maxDist = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_C2X_MAX_DISTANCE));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2X_OCTREE_LEVEL))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: value after \"-%1\"").arg(COMMAND_C2X_OCTREE_LEVEL));
			bool conversionOk = false;
			octreeLevel = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_C2X_OCTREE_LEVEL));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2C_SPLIT_XYZ))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			splitXYZ = true;
			
			if (m_cloud2meshDist)
				cmd.warning("Parameter \"-%1\" ignored: only for C2C distance!");
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2C_LOCAL_MODEL))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (!cmd.arguments().empty())
			{
				QString modelType = cmd.arguments().takeFirst().toUpper();
				if (modelType == "LS")
					modelIndex = 1;
				else if (modelType == "TRI")
					modelIndex = 2;
				else if (modelType == "HF")
					modelIndex = 3;
				else
					return cmd.error(QObject::tr("Invalid parameter: unknown model type \"%1\"").arg(modelType));
			}
			else
			{
				return cmd.error(QObject::tr("Missing parameter: model type after \"-%1\" (LS/TRI/HF)").arg(COMMAND_C2C_LOCAL_MODEL));
			}
			
			if (!cmd.arguments().empty())
			{
				QString nType = cmd.arguments().takeFirst().toUpper();
				if (nType == "KNN")
					useKNN = true;
				else if (nType == "SPHERE")
					useKNN = false;
				else
					return cmd.error(QObject::tr("Invalid parameter: unknown neighborhood type \"%1\"").arg(nType));
			}
			else
			{
				return cmd.error(QObject::tr("Missing parameter: expected neighborhood type after model type (KNN/SPHERE)"));
			}
			
			//neighborhood size
			if (!cmd.arguments().empty())
			{
				bool conversionOk = false;
				nSize = cmd.arguments().takeFirst().toDouble(&conversionOk);
				if (!conversionOk)
					return cmd.error(QObject::tr("Invalid parameter: neighborhood size"));
			}
			else
			{
				return cmd.error(QObject::tr("Missing parameter: expected neighborhood size after neighborhood type (neighbor count/sphere radius)"));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_MAX_THREAD_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: max thread count after '%1'").arg(COMMAND_MAX_THREAD_COUNT));
			
			bool ok;
			maxThreadCount = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || maxThreadCount < 0)
				return cmd.error(QObject::tr("Invalid thread count! (after %1)").arg(COMMAND_MAX_THREAD_COUNT));
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	//spawn dialog (virtually) so as to prepare the comparison process
	ccComparisonDlg compDlg(compCloud,
							refEntity,
							m_cloud2meshDist ? ccComparisonDlg::CLOUDMESH_DIST : ccComparisonDlg::CLOUDCLOUD_DIST,
							cmd.widgetParent(),
							true);
	
	//update parameters
	if (maxDist > 0)
	{
		compDlg.maxDistCheckBox->setChecked(true);
		compDlg.maxSearchDistSpinBox->setValue(maxDist);
	}
	if (octreeLevel > 0)
	{
		compDlg.octreeLevelComboBox->setCurrentIndex(octreeLevel);
	}
	if (maxThreadCount != 0)
	{
		compDlg.maxThreadCountSpinBox->setValue(maxThreadCount);
	}
	
	//C2M-only parameters
	if (m_cloud2meshDist)
	{
		if (flipNormals)
			compDlg.flipNormalsCheckBox->setChecked(true);
	}
	//C2C-only parameters
	else
	{
		if (splitXYZ)
		{
			//DGM: not true anymore
			//if (maxDist > 0)
			//	cmd.warning("'Split XYZ' option is ignored if max distance is defined!");
			compDlg.split3DCheckBox->setChecked(true);
		}
		if (modelIndex != 0)
		{
			compDlg.localModelComboBox->setCurrentIndex(modelIndex);
			if (useKNN)
			{
				compDlg.lmKNNRadioButton->setChecked(true);
				compDlg.lmKNNSpinBox->setValue(static_cast<int>(nSize));
			}
			else
			{
				compDlg.lmRadiusRadioButton->setChecked(true);
				compDlg.lmRadiusDoubleSpinBox->setValue(nSize);
			}
		}
	}
	
	if (!compDlg.computeDistances())
	{
		compDlg.cancelAndExit();
		return cmd.error("An error occurred during distances computation!");
	}
	
	compDlg.applyAndExit();
	
	QString suffix(m_cloud2meshDist ? "_C2M_DIST" : "_C2C_DIST");
	if (maxDist > 0)
		suffix += QObject::tr("_MAX_DIST_%1").arg(maxDist);
	
	compEntity->basename += suffix;
	
	if (cmd.autoSaveMode())
	{
		QString errorStr = cmd.exportEntity(*compEntity);
		if (!errorStr.isEmpty())
			return cmd.error(errorStr);
	}
	
	return true;
}

CommandC2MDist::CommandC2MDist()
	: CommandDist(true, "C2M distance", COMMAND_C2M_DIST)
{}

CommandC2CDist::CommandC2CDist()
	: CommandDist(false, "C2C distance", COMMAND_C2C_DIST)
{}

CommandStatTest::CommandStatTest()
	: ccCommandLineInterface::Command("Statistical test", COMMAND_STAT_TEST)
{}

bool CommandStatTest::process(ccCommandLineInterface &cmd)
{
	cmd.print("[STATISTICAL TEST]");
	
	//distribution
	CCLib::GenericDistribution* distrib = nullptr;
	{
		if (cmd.arguments().empty())
			return cmd.error(QObject::tr("Missing parameter: distribution type after \"-%1\" (GAUSS/WEIBULL)").arg(COMMAND_STAT_TEST));
		
		QString distribStr = cmd.arguments().takeFirst().toUpper();
		if (distribStr == "GAUSS")
		{
			//mu
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: mean value after \"GAUSS\""));
			bool conversionOk = false;
			double mu = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: mean value after \"GAUSS\""));
			//sigma
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: sigma value after \"GAUSS\" {mu}"));
			conversionOk = false;
			double sigma = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: sigma value after \"GAUSS\" {mu}"));
			
			CCLib::NormalDistribution* N = new CCLib::NormalDistribution();
			N->setParameters(static_cast<ScalarType>(mu), static_cast<ScalarType>(sigma*sigma)); //warning: we input sigma2 here (not sigma)
			distrib = static_cast<CCLib::GenericDistribution*>(N);
		}
		else if (distribStr == "WEIBULL")
		{
			//a
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: a value after \"WEIBULL\""));
			bool conversionOk = false;
			double a = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: a value after \"WEIBULL\""));
			//b
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: b value after \"WEIBULL\" {a}"));
			conversionOk = false;
			double b = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: b value after \"WEIBULL\" {a}"));
			//c
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: shift value after \"WEIBULL\" {a} {b}"));
			conversionOk = false;
			double shift = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
				return cmd.error(QObject::tr("Invalid parameter: shift value after \"WEIBULL\" {a} {b}"));
			
			CCLib::WeibullDistribution* N = new CCLib::WeibullDistribution();
			N->setParameters(static_cast<ScalarType>(a), static_cast<ScalarType>(b), static_cast<ScalarType>(shift));
			distrib = static_cast<CCLib::GenericDistribution*>(N);
		}
		else
		{
			return cmd.error(QObject::tr("Invalid parameter: unknown distribution \"%1\"").arg(distribStr));
		}
	}
	
	//pValue
	double pValue = 0.0005;
	{
		if (cmd.arguments().empty())
			return cmd.error(QObject::tr("Missing parameter: p-value after distribution"));
		bool conversionOk = false;
		pValue = cmd.arguments().takeFirst().toDouble(&conversionOk);
		if (!conversionOk)
			return cmd.error(QObject::tr("Invalid parameter: p-value after distribution"));
	}
	
	//kNN
	unsigned kNN = 16;
	{
		if (cmd.arguments().empty())
			return cmd.error(QObject::tr("Missing parameter: neighbors after p-value"));
		bool conversionOk = false;
		kNN = cmd.arguments().takeFirst().toUInt(&conversionOk);
		if (!conversionOk)
			return cmd.error(QObject::tr("Invalid parameter: neighbors after p-value"));
	}
	
	if (cmd.clouds().empty())
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* pc = cmd.clouds()[i].pc;
		
		//we apply method on currently 'output' SF
		CCLib::ScalarField* outSF = pc->getCurrentOutScalarField();
		if (outSF)
		{
			assert(outSF->capacity() != 0);
			
			//force Chi2 Distances field as 'IN' field (create it by the way if necessary)
			int chi2SfIdx = pc->getScalarFieldIndexByName(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			if (chi2SfIdx < 0)
				chi2SfIdx = pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			if (chi2SfIdx < 0)
			{
				delete distrib;
				return cmd.error("Couldn't allocate a new scalar field for computing chi2 distances! Try to free some memory ...");
			}
			pc->setCurrentInScalarField(chi2SfIdx);
			
			//compute octree if necessary
			ccOctree::Shared theOctree = pc->getOctree();
			if (!theOctree)
			{
				theOctree = pc->computeOctree(progressDialog.data());
				if (!theOctree)
				{
					delete distrib;
					cmd.error(QObject::tr("Couldn't compute octree for cloud '%1'!").arg(pc->getName()));
					break;
				}
			}
			
			double chi2dist = CCLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib, pc, kNN, pValue, progressDialog.data(), theOctree.data());
			
			cmd.print(QObject::tr("[Chi2 Test] %1 test result = %2").arg(distrib->getName()).arg(chi2dist));
			
			//we set the theoretical Chi2 distance limit as the minimum displayed SF value so that all points below are grayed
			{
				ccScalarField* chi2SF = static_cast<ccScalarField*>(pc->getCurrentInScalarField());
				assert(chi2SF);
				chi2SF->computeMinAndMax();
				chi2dist *= chi2dist;
				chi2SF->setMinDisplayed(static_cast<ScalarType>(chi2dist));
				chi2SF->setSymmetricalScale(false);
				chi2SF->setSaturationStart(static_cast<ScalarType>(chi2dist));
				//chi2SF->setSaturationStop(chi2dist);
				pc->setCurrentDisplayedScalarField(chi2SfIdx);
				pc->showSF(true);
			}
			
			cmd.clouds()[i].basename += QObject::tr("_STAT_TEST_%1").arg(distrib->getName());
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.clouds()[i]);
				if (!errorStr.isEmpty())
					return cmd.error(errorStr);
			}
		}
	}
	
	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}
	
	return true;
}

CommandDelaunayTri::CommandDelaunayTri()
	: ccCommandLineInterface::Command("Delaunay triangulation", COMMAND_DELAUNAY)
{}

bool CommandDelaunayTri::process(ccCommandLineInterface &cmd)
{
	cmd.print("[DELAUNAY TRIANGULATION]");
	
	bool axisAligned = true;
	double maxEdgeLength = 0;
	
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_DELAUNAY_AA))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			axisAligned = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_DELAUNAY_BF))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			axisAligned = false;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_DELAUNAY_MAX_EDGE_LENGTH))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: max edge length value after '%1'").arg(COMMAND_DELAUNAY_MAX_EDGE_LENGTH));
			bool ok;
			maxEdgeLength = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok)
				return cmd.error(QObject::tr("Invalid value for max edge length! (after %1)").arg(COMMAND_DELAUNAY_MAX_EDGE_LENGTH));
			cmd.print(QObject::tr("Max edge length: %1").arg(maxEdgeLength));
		}
		else
		{
			break;
		}
	}
	
	cmd.print(QObject::tr("Axis aligned: %1").arg(axisAligned ? "yes" : "no"));
	
	//try to triangulate each cloud
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* cloud = cmd.clouds()[i].pc;
		cmd.print(QObject::tr("\tProcessing cloud #%1 (%2)").arg(i + 1).arg(!cloud->getName().isEmpty() ? cloud->getName() : "no name"));
		
		ccMesh* mesh = ccMesh::Triangulate(cloud,
										   axisAligned ? DELAUNAY_2D_AXIS_ALIGNED : DELAUNAY_2D_BEST_LS_PLANE,
										   false,
										   static_cast<PointCoordinateType>(maxEdgeLength),
										   2 //XY plane by default
										   );
		
		if (mesh)
		{
			cmd.print(QObject::tr("\tResulting mesh: #%1 faces, %2 vertices").arg(mesh->size()).arg(mesh->getAssociatedCloud()->size()));
			
			CLMeshDesc meshDesc;
			meshDesc.mesh = mesh;
			meshDesc.basename = cmd.clouds()[i].basename;
			meshDesc.path = cmd.clouds()[i].path;
			meshDesc.indexInFile = cmd.clouds()[i].indexInFile;
			
			//save mesh
			if (cmd.autoSaveMode())
			{
				QString outputFilename;
				QString errorStr = cmd.exportEntity(meshDesc, "DELAUNAY", &outputFilename);
				if (!errorStr.isEmpty())
					cmd.warning(errorStr);
			}
			
			//add the resulting mesh to the main set
			cmd.meshes().push_back(meshDesc);
			
			//the mesh takes ownership of the cloud.
			//Therefore we have to remove all clouds from the 'cloud set'! (see below)
			//(otherwise bad things will happen when we'll clear it later ;)
			cloud->setEnabled(false);
			mesh->addChild(cloud);
		}
	}
	//mehses have taken ownership of the clouds!
	cmd.clouds().resize(0);
	
	return true;
}

CommandSFArithmetic::CommandSFArithmetic()
	: ccCommandLineInterface::Command("SF arithmetic", COMMAND_SF_ARITHMETIC)
{}

bool CommandSFArithmetic::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SF ARITHMETIC]");
	
	if (cmd.arguments().size() < 2)
	{
		return cmd.error(QObject::tr("Missing parameter(s): SF index and/or operation after '%1' (2 values expected)").arg(COMMAND_SF_ARITHMETIC));
	}
	
	//read sf index
	int sfIndex = -1;
	bool ok = true;
	QString sfIndexStr = cmd.arguments().takeFirst();
	if (sfIndexStr.toUpper() == OPTION_LAST)
		sfIndex = -2;
	else
		sfIndex = sfIndexStr.toInt(&ok);
	if (!ok || sfIndex < 0)
		return cmd.error(QObject::tr("Invalid SF index! (after %1)").arg(COMMAND_SF_ARITHMETIC));
	
	//read operation type
	ccScalarFieldArithmeticsDlg::Operation operation = ccScalarFieldArithmeticsDlg::INVALID;
	{
		QString opName = cmd.arguments().takeFirst();
		operation = ccScalarFieldArithmeticsDlg::GetOperationByName(opName);
		if (operation == ccScalarFieldArithmeticsDlg::INVALID)
		{
			return cmd.error(QObject::tr("Unknown operation! (%1)").arg(opName));
		}
		else if (operation <= ccScalarFieldArithmeticsDlg::DIVIDE)
		{
			return cmd.error(QObject::tr("Operation %1 can't be applied with %2").arg(opName, COMMAND_SF_ARITHMETIC));
		}
	}
	
	//apply operation on clouds
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* cloud = cmd.clouds()[i].pc;
		if (cloud && cloud->getNumberOfScalarFields() != 0 && sfIndex < static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, sfIndex < 0 ? static_cast<int>(cloud->getNumberOfScalarFields()) - 1 : sfIndex, false))
			{
				return cmd.error(QObject::tr("Failed top apply operation on cloud '%1'").arg(cloud->getName()));
			}
			else if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.clouds()[i], "SF_ARITHMETIC");
				if (!errorStr.isEmpty())
				{
					return cmd.error(errorStr);
				}
			}
		}
	}
	
	//and meshes!
	for (size_t j = 0; j < cmd.meshes().size(); ++j)
	{
		bool isLocked = false;
		ccGenericMesh* mesh = cmd.meshes()[j].mesh;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(mesh, &isLocked);
		if (cloud && !isLocked && cloud->getNumberOfScalarFields() != 0 && sfIndex < static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, sfIndex < 0 ? static_cast<int>(cloud->getNumberOfScalarFields()) - 1 : sfIndex, false))
			{
				return cmd.error(QObject::tr("Failed top apply operation on mesh '%1'").arg(mesh->getName()));
			}
			else if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.meshes()[j], "SF_ARITHMETIC");
				if (!errorStr.isEmpty())
				{
					return cmd.error(errorStr);
				}
			}
		}
	}
	
	return true;
}

CommandSFOperation::CommandSFOperation()
	: ccCommandLineInterface::Command("SF operation", COMMAND_SF_OP)
{}

bool CommandSFOperation::process(ccCommandLineInterface &cmd)
{
	cmd.print("[SF OPERATION]");
	
	if (cmd.arguments().size() < 3)
	{
		return cmd.error(QObject::tr("Missing parameter(s): SF index and/or operation and/or scalar value after '%1' (3 values expected)").arg(COMMAND_SF_OP));
	}
	
	//read sf index
	int sfIndex = -1;
	bool ok = true;
	QString sfIndexStr = cmd.arguments().takeFirst();
	if (sfIndexStr.toUpper() == OPTION_LAST)
		sfIndex = -2;
	else
		sfIndex = sfIndexStr.toInt(&ok);
	
	if (!ok || sfIndex < 0)
	{
		return cmd.error(QObject::tr("Invalid SF index! (after %1)").arg(COMMAND_SF_OP));
	}
	
	//read operation type
	ccScalarFieldArithmeticsDlg::Operation operation = ccScalarFieldArithmeticsDlg::INVALID;
	{
		QString opName = cmd.arguments().takeFirst();
		operation = ccScalarFieldArithmeticsDlg::GetOperationByName(opName);
		if (operation == ccScalarFieldArithmeticsDlg::INVALID)
		{
			return cmd.error(QObject::tr("Unknown operation! (%1)").arg(opName));
		}
		else if (operation > ccScalarFieldArithmeticsDlg::DIVIDE)
		{
			return cmd.error(QObject::tr("Operation %1 can't be applied with %2").arg(opName, COMMAND_SF_OP));
		}
	}
	
	//read scalar value
	double value = 1.0;
	{
		bool ok = true;
		value = cmd.arguments().takeFirst().toDouble(&ok);
		if (!ok)
		{
			return cmd.error(QObject::tr("Invalid scalar value! (after %1)").arg(COMMAND_SF_OP));
		}
	}
	
	ccScalarFieldArithmeticsDlg::SF2 sf2;
	{
		sf2.isConstantValue = true;
		sf2.constantValue = value;
	}
	
	//apply operation on clouds
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		ccPointCloud* cloud = cmd.clouds()[i].pc;
		if (cloud && cloud->getNumberOfScalarFields() != 0 && sfIndex < static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, sfIndex < 0 ? static_cast<int>(cloud->getNumberOfScalarFields()) - 1 : sfIndex, true, &sf2))
			{
				return cmd.error(QObject::tr("Failed top apply operation on cloud '%1'").arg(cloud->getName()));
			}
			else if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.clouds()[i], "SF_OP");
				if (!errorStr.isEmpty())
				{
					return cmd.error(errorStr);
				}
			}
		}
	}
	
	//and meshes!
	for (size_t j = 0; j < cmd.meshes().size(); ++j)
	{
		bool isLocked = false;
		ccGenericMesh* mesh = cmd.meshes()[j].mesh;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(mesh, &isLocked);
		if (cloud && !isLocked && cloud->getNumberOfScalarFields() != 0 && sfIndex < static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, sfIndex < 0 ? static_cast<int>(cloud->getNumberOfScalarFields()) - 1 : sfIndex, true, &sf2))
			{
				return cmd.error(QObject::tr("Failed top apply operation on mesh '%1'").arg(mesh->getName()));
			}
			else if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(cmd.meshes()[j], "SF_OP");
				if (!errorStr.isEmpty())
				{
					return cmd.error(errorStr);
				}
			}
		}
	}
	
	return true;
}

CommandICP::CommandICP()
	: ccCommandLineInterface::Command("ICP", COMMAND_ICP)
{}

bool CommandICP::process(ccCommandLineInterface &cmd)
{
	cmd.print("[ICP]");
	
	//look for local options
	bool referenceIsFirst = false;
	bool adjustScale = false;
	bool enableFarthestPointRemoval = false;
	double minErrorDiff = 1.0e-6;
	unsigned iterationCount = 0;
	unsigned randomSamplingLimit = 20000;
	unsigned overlap = 100;
	int modelSFAsWeights = -1;
	int dataSFAsWeights = -1;
	int maxThreadCount = 0;
	int transformationFilters = 0;
	
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_REFERENCE_IS_FIRST))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			referenceIsFirst = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_ADJUST_SCALE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			adjustScale = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_ENABLE_FARTHEST_REMOVAL))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			enableFarthestPointRemoval = true;
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_MIN_ERROR_DIIF))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: min error difference after '%1'").arg(COMMAND_ICP_MIN_ERROR_DIIF));
			bool ok;
			minErrorDiff = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok || minErrorDiff <= 0)
				return cmd.error(QObject::tr("Invalid value for min. error difference! (after %1)").arg(COMMAND_ICP_MIN_ERROR_DIIF));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_ITERATION_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: number of iterations after '%1'").arg(COMMAND_ICP_ITERATION_COUNT));
			bool ok;
			QString arg = cmd.arguments().takeFirst();
			iterationCount = arg.toUInt(&ok);
			if (!ok || iterationCount == 0)
				return cmd.error(QObject::tr("Invalid number of iterations! (%1)").arg(arg));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_OVERLAP))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: overlap percentage after '%1'").arg(COMMAND_ICP_OVERLAP));
			bool ok;
			QString arg = cmd.arguments().takeFirst();
			overlap = arg.toUInt(&ok);
			if (!ok || overlap < 10 || overlap > 100)
				return cmd.error(QObject::tr("Invalid overlap value! (%1 --> should be between 10 and 100)").arg(arg));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_RANDOM_SAMPLING_LIMIT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: random sampling limit value after '%1'").arg(COMMAND_ICP_RANDOM_SAMPLING_LIMIT));
			bool ok;
			randomSamplingLimit = cmd.arguments().takeFirst().toUInt(&ok);
			if (!ok || randomSamplingLimit < 3)
				return cmd.error(QObject::tr("Invalid random sampling limit! (after %1)").arg(COMMAND_ICP_RANDOM_SAMPLING_LIMIT));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_USE_MODEL_SF_AS_WEIGHT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: SF index after '%1'").arg(COMMAND_ICP_USE_MODEL_SF_AS_WEIGHT));
			QString sfIndex = cmd.arguments().takeFirst();
			if (sfIndex.toUpper() == OPTION_LAST)
			{
				modelSFAsWeights = -2;
			}
			else
			{
				bool ok;
				modelSFAsWeights = sfIndex.toInt(&ok);
				if (!ok || modelSFAsWeights < 0)
					return cmd.error(QObject::tr("Invalid SF index! (after %1)").arg(COMMAND_ICP_USE_MODEL_SF_AS_WEIGHT));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_USE_DATA_SF_AS_WEIGHT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: SF index after '%1'").arg(COMMAND_ICP_USE_DATA_SF_AS_WEIGHT));
			QString sfIndex = cmd.arguments().takeFirst();
			if (sfIndex.toUpper() == OPTION_LAST)
			{
				dataSFAsWeights = -2;
			}
			else
			{
				bool ok;
				dataSFAsWeights = sfIndex.toInt(&ok);
				if (!ok || dataSFAsWeights < 0)
					return cmd.error(QObject::tr("Invalid SF index! (after %1)").arg(COMMAND_ICP_USE_DATA_SF_AS_WEIGHT));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_MAX_THREAD_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
				return cmd.error(QObject::tr("Missing parameter: max thread count after '%1'").arg(COMMAND_MAX_THREAD_COUNT));
			
			bool ok;
			maxThreadCount = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || maxThreadCount < 0)
				return cmd.error(QObject::tr("Invalid thread count! (after %1)").arg(COMMAND_MAX_THREAD_COUNT));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_ROT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (!cmd.arguments().empty())
			{
				QString rotation = cmd.arguments().takeFirst().toUpper();
				if (rotation == "XYZ")
					transformationFilters = 0;
				else if (rotation == "X")
					transformationFilters = 2;
				else if (rotation == "Y")
					transformationFilters = 4;
				else if (rotation == "Z")
					transformationFilters = 1;
				else if (rotation == "NONE")
					transformationFilters = 7;
				else
					return cmd.error(QObject::tr("Invalid parameter: unknown rotation filter \"%1\"").arg(rotation));
			}
			else
			{
				return cmd.error(QObject::tr("Missing parameter: rotation filter after \"-%1\" (XYZ/X/Y/Z/NONE)").arg(COMMAND_ICP_ROT));
			}
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	//we'll get the first two entities
	CLEntityDesc* dataAndModel[2] = { nullptr, nullptr };
	{
		int index = 0;
		if (!cmd.clouds().empty())
		{
			dataAndModel[index++] = &cmd.clouds()[0];
			if (cmd.clouds().size() > 1)
				dataAndModel[index++] = &cmd.clouds()[1];
		}
		if (index < 2 && !cmd.meshes().empty())
		{
			dataAndModel[index++] = &cmd.meshes()[0];
			if (index < 2 && cmd.meshes().size() > 1)
				dataAndModel[index++] = &cmd.meshes()[1];
		}
		
		if (index < 2)
			return cmd.error("Not enough loaded entities (expect at least 2!)");
	}
	
	//put them in the right order (data first, model next)
	if (referenceIsFirst)
	{
		std::swap(dataAndModel[0], dataAndModel[1]);
	}
	
	//check that the scalar fields (weights) exist
	if (dataSFAsWeights != -1)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(dataAndModel[0]->getEntity());
		if (!cloud || cloud->getNumberOfScalarFields() == 0 || dataSFAsWeights >= static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			return cmd.error(QObject::tr("Invalid SF index for data entity! (%1)").arg(dataSFAsWeights));
		}
		else
		{
			if (dataSFAsWeights < 0) //last SF
				dataSFAsWeights = static_cast<int>(cloud->getNumberOfScalarFields()) - 1;
			cmd.print(QObject::tr("[ICP] SF #%1 (data entity) will be used as weights").arg(dataSFAsWeights));
			cloud->setCurrentDisplayedScalarField(dataSFAsWeights);
		}
	}
	
	if (modelSFAsWeights != -1)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(dataAndModel[1]->getEntity());
		if (!cloud || cloud->getNumberOfScalarFields() == 0 || modelSFAsWeights >= static_cast<int>(cloud->getNumberOfScalarFields()))
		{
			return cmd.error(QObject::tr("Invalid SF index for model entity! (%1)").arg(modelSFAsWeights));
		}
		else
		{
			if (modelSFAsWeights < 0) //last SF
				modelSFAsWeights = static_cast<int>(cloud->getNumberOfScalarFields()) - 1;
			cmd.print(QObject::tr("[ICP] SF #%1 (model entity) will be used as weights").arg(modelSFAsWeights));
			cloud->setCurrentDisplayedScalarField(modelSFAsWeights);
		}
	}
	
	ccGLMatrix transMat;
	double finalError = 0.0;
	double finalScale = 1.0;
	unsigned finalPointCount = 0;
	if (ccRegistrationTools::ICP(	dataAndModel[0]->getEntity(),
									dataAndModel[1]->getEntity(),
									transMat,
									finalScale,
									finalError,
									finalPointCount,
									minErrorDiff,
									iterationCount,
									randomSamplingLimit,
									enableFarthestPointRemoval,
									iterationCount != 0 ? CCLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE : CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE,
									adjustScale,
									overlap / 100.0,
									dataSFAsWeights >= 0,
									modelSFAsWeights >= 0,
									transformationFilters,
									maxThreadCount,
									cmd.widgetParent()))
	{
		ccHObject* data = dataAndModel[0]->getEntity();
		data->applyGLTransformation_recursive(&transMat);
		cmd.print(QObject::tr("Entity '%1' has been registered").arg(data->getName()));
		cmd.print(QObject::tr("RMS: %1").arg(finalError));
		cmd.print(QObject::tr("Number of points used for final step: %1").arg(finalPointCount));
		
		//save matrix in a separate text file
		{
			QString txtFilename = QObject::tr("%1/%2_REGISTRATION_MATRIX").arg(dataAndModel[0]->path, dataAndModel[0]->basename);
			if (cmd.addTimestamp())
				txtFilename += QObject::tr("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));
			txtFilename += QObject::tr(".txt");
			QFile txtFile(txtFilename);
			txtFile.open(QIODevice::WriteOnly | QIODevice::Text);
			QTextStream txtStream(&txtFile);
			txtStream << transMat.toString(cmd.numericalPrecision(), ' ') << endl;
			txtFile.close();
		}
		
		dataAndModel[0]->basename += QObject::tr("_REGISTERED");
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(*dataAndModel[0]);
			if (!errorStr.isEmpty())
				return cmd.error(errorStr);
		}
	}
	else
	{
		return false;
	}
	
	return true;
}

CommandChangePLYExportFormat::CommandChangePLYExportFormat()
	: ccCommandLineInterface::Command("Change PLY output format", COMMAND_PLY_EXPORT_FORMAT)
{}

bool CommandChangePLYExportFormat::process(ccCommandLineInterface &cmd)
{
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: format (ASCII, BINARY_LE, or BINARY_BE) after '%1'").arg(COMMAND_PLY_EXPORT_FORMAT));
	
	//if (fileFilter != PlyFilter::GetFileFilter())
	//	cmd.warning(QObject::tr("Argument '%1' is only applicable to PLY format!").arg(argument));
	
	QString plyFormat = cmd.arguments().takeFirst().toUpper();
	//printf("%s\n",qPrintable(plyFormat));
	
	if (plyFormat == "ASCII")
		PlyFilter::SetDefaultOutputFormat(PLY_ASCII);
	else if (plyFormat == "BINARY_BE")
		PlyFilter::SetDefaultOutputFormat(PLY_BIG_ENDIAN);
	else if (plyFormat == "BINARY_LE")
		PlyFilter::SetDefaultOutputFormat(PLY_LITTLE_ENDIAN);
	else
		return cmd.error(QObject::tr("Invalid PLY format! ('%1')").arg(plyFormat));
	
	return true;
}

CommandForceNormalsComputation::CommandForceNormalsComputation()
	: ccCommandLineInterface::Command("Compute structured cloud normals", COMMAND_COMPUTE_GRIDDED_NORMALS)
{}

bool CommandForceNormalsComputation::process(ccCommandLineInterface &cmd)
{
	//simply change the default filter behavior
	cmd.fileLoadingParams().autoComputeNormals = true;
	
	return true;
}

CommandSave::CommandSave(const QString& name, const QString& keyword)
	: ccCommandLineInterface::Command(name, keyword)
{}

bool CommandSave::ParseFileNames(ccCommandLineInterface &cmd, QStringList &fileNames)
{
	//
	// File list is space separated, but can use quotes to include spaces in the file names
	//
	auto argument = cmd.arguments().takeFirst();
	while (!argument.isEmpty())
	{
		auto firstChar = argument.at(0);
		if (firstChar == '\'' || firstChar == '\"')
		{
			auto end = argument.indexOf(firstChar, 1);
			if (end == -1)
				return cmd.error(QString("A file starting with %1 does not have a closing %1").arg(firstChar));
			
			fileNames.push_back(argument.mid(1, end - 1));
			argument.remove(0, end + 1);
			if (argument.startsWith(' '))
				argument.remove(0, 1);
		}
		else
		{
			auto end = argument.indexOf(' ');
			if (end == -1)
				end = argument.length();
			fileNames.push_back(argument.left(end));
			argument.remove(0, end + 1);
		}
	}
	return true;
}

void CommandSave::SetFileDesc(CLEntityDesc &desc, const QString &fileName)
{
	QFileInfo fInfo(fileName);
	desc.basename = fInfo.fileName();
	desc.path = fInfo.filePath().left(fInfo.filePath().length() - fInfo.fileName().length());
}

CommandSaveClouds::CommandSaveClouds()
	: CommandSave("Save clouds", COMMAND_SAVE_CLOUDS)
{}

bool CommandSaveClouds::process(ccCommandLineInterface &cmd)
{
	bool allAtOnce = false;
	bool setFileNames = false;
	QStringList fileNames;
	
	//look for additional parameters
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (argument.toUpper() == OPTION_ALL_AT_ONCE)
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			allAtOnce = true;
		}
		else if (argument.left(sizeof(OPTION_FILE_NAMES) - 1).toUpper() == OPTION_FILE_NAMES)
		{
			cmd.arguments().pop_front();
			setFileNames = true;
			if (!ParseFileNames(cmd, fileNames))
				return false;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	if(setFileNames && allAtOnce && fileNames.size() != 1)
		return cmd.error(QString("Invalid parameter: specified %1 file names, but ALL_AT_ONCE is on").arg(fileNames.size()));
	if(setFileNames && !allAtOnce && fileNames.size() != cmd.clouds().size())
		return cmd.error(QString("Invalid parameter: specified %1 file names, but there are %2 clouds").arg(fileNames.size()).arg(cmd.clouds().size()));
	
	QString ext = cmd.cloudExportExt();
	bool timestamp = cmd.addTimestamp();
	if (setFileNames)
	{
		cmd.toggleAddTimestamp(false);
		cmd.setCloudExportFormat(cmd.cloudExportFormat(), QString());
		
		if (!allAtOnce)
		{
			for (int i = 0; i < fileNames.size(); ++i)
			{
				SetFileDesc(cmd.clouds()[i], fileNames[i]);
			}
		}
	}
	
	auto res = cmd.saveClouds(QString(), allAtOnce, setFileNames ? &fileNames[0] : nullptr);
	
	if (setFileNames)
	{
		cmd.toggleAddTimestamp(timestamp);
		cmd.setCloudExportFormat(cmd.cloudExportFormat(), ext);
	}
	
	return res;
}

CommandSaveMeshes::CommandSaveMeshes()
	: CommandSave("Save meshes", COMMAND_SAVE_MESHES)
{}

bool CommandSaveMeshes::process(ccCommandLineInterface &cmd)
{
	bool allAtOnce = false;
	bool setFileNames = false;
	QStringList fileNames;
	
	//look for additional parameters
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (argument.toUpper() == OPTION_ALL_AT_ONCE)
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			allAtOnce = true;
		}
		else if (argument.left(sizeof(OPTION_FILE_NAMES) - 1).toUpper() == OPTION_FILE_NAMES)
		{
			cmd.arguments().pop_front();
			setFileNames = true;
			if (!ParseFileNames(cmd, fileNames))
				return false;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	if (setFileNames && allAtOnce && fileNames.size() != 1)
		return cmd.error(QString("Invalid parameter: specified %1 file names, but ALL_AT_ONCE is on").arg(fileNames.size()));
	if (setFileNames && !allAtOnce && fileNames.size() != cmd.meshes().size())
		return cmd.error(QString("Invalid parameter: specified %1 file names, but there are %2 meshes").arg(fileNames.size()).arg(cmd.meshes().size()));
	
	QString ext = cmd.meshExportExt();
	bool timestamp = cmd.addTimestamp();
	if (setFileNames)
	{
		cmd.toggleAddTimestamp(false);
		cmd.setMeshExportFormat(cmd.meshExportFormat(), QString());
		
		if (!allAtOnce)
		{
			for (int i = 0; i < fileNames.size(); ++i)
			{
				SetFileDesc(cmd.meshes()[i], fileNames[i]);
			}
		}
	}
	
	auto res = cmd.saveMeshes(QString(), allAtOnce, setFileNames ? &fileNames[0] : nullptr);
	
	if (setFileNames)
	{
		cmd.toggleAddTimestamp(timestamp);
		cmd.setMeshExportFormat(cmd.meshExportFormat(), ext);
	}
	
	return res;
}

CommandAutoSave::CommandAutoSave()
	: ccCommandLineInterface::Command("Auto save state", COMMAND_AUTO_SAVE)
{}

bool CommandAutoSave::process(ccCommandLineInterface &cmd)
{
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: option after '%1' (%2/%3)").arg(COMMAND_AUTO_SAVE, OPTION_ON, OPTION_OFF));
	
	QString option = cmd.arguments().takeFirst().toUpper();
	if (option == OPTION_ON)
	{
		cmd.print("Auto-save is enabled");
		cmd.toggleAutoSaveMode(true);
	}
	else if (option == OPTION_OFF)
	{
		cmd.print("Auto-save is disabled");
		cmd.toggleAutoSaveMode(false);
	}
	else
	{
		return cmd.error(QObject::tr("Unrecognized option after '%1' (%2 or %3 expected)").arg(COMMAND_AUTO_SAVE, OPTION_ON, OPTION_OFF));
	}
	
	return true;
}

CommandLogFile::CommandLogFile()
	: ccCommandLineInterface::Command("Set log file", COMMAND_LOG_FILE)
{}

bool CommandLogFile::process(ccCommandLineInterface &cmd)
{
	if (cmd.arguments().empty())
		return cmd.error(QObject::tr("Missing parameter: filename after '%1'").arg(COMMAND_LOG_FILE));
	
	QString filename = cmd.arguments().takeFirst();
	if (!ccConsole::TheInstance(false))
	{
		assert(cmd.silentMode());
		ccConsole::Init();
	}
	
	return ccConsole::TheInstance()->setLogFile(filename);
}

CommandClear::CommandClear()
	: ccCommandLineInterface::Command("Clear", COMMAND_CLEAR)
{}

bool CommandClear::process(ccCommandLineInterface &cmd)
{
	cmd.removeClouds(false);
	cmd.removeMeshes(false);
	return true;
}

CommandClearClouds::CommandClearClouds()
	: ccCommandLineInterface::Command("Clear clouds", COMMAND_CLEAR_CLOUDS)
{}

bool CommandClearClouds::process(ccCommandLineInterface &cmd)
{
	cmd.removeClouds(false);
	return true;
}

CommandPopClouds::CommandPopClouds()
	: ccCommandLineInterface::Command("Pop clouds", COMMAND_POP_CLOUDS)
{}

bool CommandPopClouds::process(ccCommandLineInterface &cmd)
{
	cmd.removeClouds(true);
	return true;
}

CommandClearMeshes::CommandClearMeshes()
	: ccCommandLineInterface::Command("Clear meshes", COMMAND_CLEAR_MESHES)
{}

bool CommandClearMeshes::process(ccCommandLineInterface &cmd)
{
	cmd.removeMeshes(false);
	return true;
}

CommandPopMeshes::CommandPopMeshes()
	: ccCommandLineInterface::Command("Pop meshes", COMMAND_POP_MESHES)
{}

bool CommandPopMeshes::process(ccCommandLineInterface &cmd)
{
	cmd.removeMeshes(true);
	return true;
}

CommandSetNoTimestamp::CommandSetNoTimestamp()
	: ccCommandLineInterface::Command("No timestamp", COMMAND_NO_TIMESTAMP)
{}

bool CommandSetNoTimestamp::process(ccCommandLineInterface &cmd)
{
	cmd.toggleAddTimestamp(false);
	return true;
}
