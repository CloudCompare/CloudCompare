//CCCoreLib
#include <AutoSegmentationTools.h>
#include <CCConst.h>
#include <CloudSamplingTools.h>
#include <MeshSamplingTools.h>
#include <NormalDistribution.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>
#include <DistanceComputationTools.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccNormalVectors.h>
#include <ccPlane.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccVolumeCalcTool.h>
#include <ccSubMesh.h>
#include <ccPointCloudInterpolator.h>

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
#include "ccColorLevelsDlg.h"

//Qt
#include "ccCommandLineCommands.h"

//Local
#include "ccEntityAction.h"

#include <QDateTime>
#include <QFileInfo>

//commands
constexpr char COMMAND_CLOUD_EXPORT_FORMAT[]			= "C_EXPORT_FMT";
constexpr char COMMAND_EXPORT_EXTENSION[]				= "EXT";
constexpr char COMMAND_ASCII_EXPORT_PRECISION[]			= "PREC";
constexpr char COMMAND_ASCII_EXPORT_SEPARATOR[]			= "SEP";
constexpr char COMMAND_ASCII_EXPORT_ADD_COL_HEADER[]	= "ADD_HEADER";
constexpr char COMMAND_ASCII_EXPORT_ADD_PTS_COUNT[]		= "ADD_PTS_COUNT";
constexpr char COMMAND_MESH_EXPORT_FORMAT[]				= "M_EXPORT_FMT";
constexpr char COMMAND_HIERARCHY_EXPORT_FORMAT[]		= "H_EXPORT_FMT";
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
constexpr char COMMAND_ROUGHNESS_UP_DIR[]				= "UP_DIR";
constexpr char COMMAND_APPLY_TRANSFORMATION[]			= "APPLY_TRANS";
constexpr char COMMAND_DROP_GLOBAL_SHIFT[]				= "DROP_GLOBAL_SHIFT";
constexpr char COMMAND_SF_COLOR_SCALE[]					= "SF_COLOR_SCALE";
constexpr char COMMAND_SF_CONVERT_TO_RGB[]				= "SF_CONVERT_TO_RGB";
constexpr char COMMAND_FILTER_SF_BY_VALUE[]				= "FILTER_SF";
constexpr char COMMAND_MERGE_CLOUDS[]					= "MERGE_CLOUDS";
constexpr char COMMAND_MERGE_MESHES[]                   = "MERGE_MESHES";
constexpr char COMMAND_SET_ACTIVE_SF[]					= "SET_ACTIVE_SF";
constexpr char COMMAND_REMOVE_ALL_SFS[]					= "REMOVE_ALL_SFS";
constexpr char COMMAND_REMOVE_SF[]						= "REMOVE_SF";
constexpr char COMMAND_REMOVE_SCAN_GRIDS[]				= "REMOVE_SCAN_GRIDS";
constexpr char COMMAND_REMOVE_RGB[]						= "REMOVE_RGB";
constexpr char COMMAND_REMOVE_NORMALS[]					= "REMOVE_NORMALS";
constexpr char COMMAND_MATCH_BB_CENTERS[]				= "MATCH_CENTERS";
constexpr char COMMAND_BEST_FIT_PLANE[]					= "BEST_FIT_PLANE";
constexpr char COMMAND_BEST_FIT_PLANE_MAKE_HORIZ[]		= "MAKE_HORIZ";
constexpr char COMMAND_BEST_FIT_PLANE_KEEP_LOADED[]		= "KEEP_LOADED";
constexpr char COMMAND_ORIENT_NORMALS[]					= "ORIENT_NORMS_MST";
constexpr char COMMAND_SOR_FILTER[]						= "SOR";
constexpr char COMMAND_NOISE_FILTER[]					= "NOISE";
constexpr char COMMAND_NOISE_FILTER_KNN[]				= "KNN";
constexpr char COMMAND_NOISE_FILTER_RADIUS[]			= "RADIUS";
constexpr char COMMAND_NOISE_FILTER_REL[]				= "REL";
constexpr char COMMAND_NOISE_FILTER_ABS[]				= "ABS";
constexpr char COMMAND_NOISE_FILTER_RIP[]				= "RIP";
constexpr char COMMAND_REMOVE_DUPLICATE_POINTS[]	    = "RDP";
constexpr char COMMAND_SAMPLE_MESH[]					= "SAMPLE_MESH";
constexpr char COMMAND_COMPRESS_FWF[]					= "COMPRESS_FWF";
constexpr char COMMAND_CROP[]							= "CROP";
constexpr char COMMAND_CROP_OUTSIDE[]					= "OUTSIDE";
constexpr char COMMAND_CROP_2D[]						= "CROP2D";
constexpr char COMMAND_COLOR_BANDING[]					= "CBANDING";
constexpr char COMMAND_COLOR_LEVELS[]					= "CLEVELS";
constexpr char COMMAND_C2M_DIST[]						= "C2M_DIST";
constexpr char COMMAND_C2M_DIST_FLIP_NORMALS[]			= "FLIP_NORMS";
constexpr char COMMAND_C2M_DIST_UNSIGNED[]				= "UNSIGNED";
constexpr char COMMAND_C2C_DIST[]						= "C2C_DIST";
constexpr char COMMAND_CLOSEST_POINT_SET[]              = "CLOSEST_POINT_SET";
constexpr char COMMAND_C2C_SPLIT_XYZ[]					= "SPLIT_XYZ";
constexpr char COMMAND_C2C_SPLIT_XY_Z[]                 = "SPLIT_XY_Z";
constexpr char COMMAND_C2C_LOCAL_MODEL[]				= "MODEL";
constexpr char COMMAND_C2X_MAX_DISTANCE[]				= "MAX_DIST";
constexpr char COMMAND_C2X_OCTREE_LEVEL[]				= "OCTREE_LEVEL";
constexpr char COMMAND_STAT_TEST[]						= "STAT_TEST";
constexpr char COMMAND_DELAUNAY[]						= "DELAUNAY";
constexpr char COMMAND_DELAUNAY_AA[]					= "AA";
constexpr char COMMAND_DELAUNAY_BF[]					= "BEST_FIT";
constexpr char COMMAND_DELAUNAY_MAX_EDGE_LENGTH[]		= "MAX_EDGE_LENGTH";
constexpr char COMMAND_SF_ARITHMETIC[]					= "SF_ARITHMETIC";
constexpr char COMMAND_SF_ARITHMETIC_IN_PLACE[]			= "IN_PLACE";
constexpr char COMMAND_SF_OP[]							= "SF_OP";
constexpr char COMMAND_SF_OP_SF[]						= "SF_OP_SF";
constexpr char COMMAND_SF_INTERP[]						= "SF_INTERP";
constexpr char COMMAND_SF_INTERP_DEST_IS_FIRST[]		= "DEST_IS_FIRST";
constexpr char COMMAND_SF_ADD_CONST[]                   = "SF_ADD_CONST";
constexpr char COMMAND_RENAME_SF[]						= "RENAME_SF";
constexpr char COMMAND_COORD_TO_SF[]					= "COORD_TO_SF";
constexpr char COMMAND_SF_TO_COORD[]					= "SF_TO_COORD";
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
constexpr char COMMAND_INVERT_NORMALS[]					= "INVERT_NORMALS";
constexpr char COMMAND_COMPUTE_OCTREE_NORMALS[]			= "OCTREE_NORMALS";
constexpr char COMMAND_CONVERT_NORMALS_TO_DIP[]			= "NORMALS_TO_DIP";
constexpr char COMMAND_CONVERT_NORMALS_TO_SFS[]			= "NORMALS_TO_SFS";
constexpr char COMMAND_CONVERT_NORMALS_TO_HSV[]			= "NORMALS_TO_HSV";
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
constexpr char COMMAND_MOMENT[]							= "MOMENT";
constexpr char COMMAND_FEATURE[]						= "FEATURE";
constexpr char COMMAND_RGB_CONVERT_TO_SF[]				= "RGB_CONVERT_TO_SF";
constexpr char COMMAND_FLIP_TRIANGLES[]					= "FLIP_TRI";
constexpr char COMMAND_DEBUG[]							= "DEBUG";

//options / modifiers
constexpr char COMMAND_MAX_THREAD_COUNT[]				= "MAX_TCOUNT";
constexpr char OPTION_ALL_AT_ONCE[]						= "ALL_AT_ONCE";
constexpr char OPTION_ON[]								= "ON";
constexpr char OPTION_OFF[]								= "OFF";
constexpr char OPTION_LAST[]							= "LAST";
constexpr char OPTION_FILE_NAMES[]						= "FILE";
constexpr char OPTION_ORIENT[]							= "ORIENT";
constexpr char OPTION_MODEL[]							= "MODEL";

static void GetSFIndexOrName(ccCommandLineInterface& cmd, int& sfIndex, QString& sfName)
{
	sfName = cmd.arguments().takeFirst();
	if (sfName.toUpper() == OPTION_LAST)
	{
		sfIndex = -2;
		cmd.print(QObject::tr("Set active SF index: LAST"));
	}
	else
	{
		bool validInt = false;
		sfIndex = sfName.toInt(&validInt);
		if (validInt)
		{
			cmd.print(QObject::tr("Set active SF index: %1").arg(sfIndex));
		}
		else
		{
			cmd.print(QObject::tr("Set active SF name: '%1'").arg(sfName));
			sfIndex = -1;
		}
	}
}

int GetScalarFieldIndex(ccPointCloud* cloud, int sfIndex, const QString& sfName)
{
	if (!cloud)
	{
		assert(false);
		return -1;
	}
	if (!cloud->hasScalarFields())
	{
		return -1;
	}

	if (sfIndex == -2)
	{
		return static_cast<int>(cloud->getNumberOfScalarFields()) - 1;
	}

	if (sfIndex == -1)
	{
		//check if this cloud has a scalar field with the input name
		sfIndex = cloud->getScalarFieldIndexByName(qPrintable(sfName));
		if (sfIndex < 0)
		{
			ccLog::Warning(QObject::tr("Cloud %1 has no SF named '%2'").arg(cloud->getName()).arg(sfName));
			return -1;
		}
	}
	else if (sfIndex >= static_cast<int>(cloud->getNumberOfScalarFields()))
	{
		ccLog::Warning(QObject::tr("Cloud %1 has less scalar fields than the SF index (%2/%3)").arg(cloud->getName()).arg(sfIndex).arg(cloud->getNumberOfScalarFields()));
		return -1;
	}

	return sfIndex;
}

CCCoreLib::ScalarField* GetScalarField(ccPointCloud* cloud, int sfIndex, const QString& sfName)
{
	sfIndex = GetScalarFieldIndex(cloud, sfIndex, sfName);
	if (sfIndex < 0)
	{
		return nullptr;
	}
	else
	{
		return cloud->getScalarField(sfIndex);
	}
}

CommandChangeOutputFormat::CommandChangeOutputFormat(const QString& name, const QString& keyword)
    : ccCommandLineInterface::Command(name, keyword)
{}

QString CommandChangeOutputFormat::getFileFormatFilter(ccCommandLineInterface& cmd, QString& defaultExt)
{
	QString fileFilter;
	defaultExt.clear();

	if (!cmd.arguments().isEmpty())
	{
		//test if the specified format corresponds to a known file type
		QString extension = cmd.arguments().front().toUpper();
		cmd.arguments().pop_front();

		const FileIOFilter::FilterContainer& filters = FileIOFilter::GetFilters();
		
		for (const auto & filter : filters)
		{
			if (extension == filter->getDefaultExtension().toUpper())
			{
				//found a matching filter
				fileFilter = filter->getFileFilters(false).first();
				defaultExt = filter->getDefaultExtension();
				break;
			}
		}

		//haven't found anything?
		if (fileFilter.isEmpty())
		{
			cmd.error(QObject::tr("Unhandled format specifier (%1)").arg(extension));
		}
	}
	else
	{
		cmd.error(QObject::tr("Missing file format specifier!"));
	}

	return fileFilter;
}

CommandChangeCloudOutputFormat::CommandChangeCloudOutputFormat()
	: CommandChangeOutputFormat(QObject::tr("Change cloud output format"), COMMAND_CLOUD_EXPORT_FORMAT)
{}

bool CommandChangeCloudOutputFormat::process(ccCommandLineInterface& cmd)
{
	QString defaultExt;
	QString fileFilter = getFileFormatFilter(cmd, defaultExt);
	if (fileFilter.isEmpty())
	{
		return false;
	}
	
	cmd.setCloudExportFormat(fileFilter, defaultExt);
	cmd.print(QObject::tr("Output export format (clouds) set to: %1").arg(defaultExt.toUpper()));
	
	//default options for ASCII output
	if (fileFilter == AsciiFilter::GetFileFilter())
	{
		AsciiFilter::SetOutputCoordsPrecision(cmd.numericalPrecision());
		AsciiFilter::SetOutputSFPrecision(cmd.numericalPrecision());
		AsciiFilter::SetOutputSeparatorIndex(0); //space
		AsciiFilter::SaveSFBeforeColor(false); //default order: point, color, SF, normal
		AsciiFilter::SaveColumnsNamesHeader(false);
		AsciiFilter::SavePointCountHeader(false);
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
			{
				return cmd.error(QObject::tr("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));
			}
			
			cmd.setCloudExportFormat(cmd.cloudExportFormat(), cmd.arguments().takeFirst());
			cmd.print(QObject::tr("New output extension for clouds: %1").arg(cmd.cloudExportExt()));
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_PRECISION))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: precision value after '%1'").arg(COMMAND_ASCII_EXPORT_PRECISION));
			}
			bool ok;
			int precision = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || precision < 0)
			{
				return cmd.error(QObject::tr("Invalid value for precision! (%1)").arg(COMMAND_ASCII_EXPORT_PRECISION));
			}
			
			if (fileFilter != AsciiFilter::GetFileFilter())
			{
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			}
			
			AsciiFilter::SetOutputCoordsPrecision(precision);
			AsciiFilter::SetOutputSFPrecision(precision);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_SEPARATOR))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: separator character after '%1'").arg(COMMAND_ASCII_EXPORT_SEPARATOR));
			}
			
			if (fileFilter != AsciiFilter::GetFileFilter())
			{
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			}
			
			QString separatorStr = cmd.arguments().takeFirst().toUpper();
			//printf("%s\n",qPrintable(separatorStr));
			int index = -1;
			if (separatorStr == "SPACE")
			{
				index = 0;
			}
			else if (separatorStr == "SEMICOLON")
			{
				index = 1;
			}
			else if (separatorStr == "COMMA")
			{
				index = 2;
			}
			else if (separatorStr == "TAB")
			{
				index = 3;
			}
			else
			{
				return cmd.error(QObject::tr("Invalid separator! ('%1')").arg(separatorStr));
			}
			
			AsciiFilter::SetOutputSeparatorIndex(index);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_ADD_COL_HEADER))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (fileFilter != AsciiFilter::GetFileFilter())
			{
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			}
			
			AsciiFilter::SaveColumnsNamesHeader(true);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ASCII_EXPORT_ADD_PTS_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (fileFilter != AsciiFilter::GetFileFilter())
			{
				cmd.warning(QObject::tr("Argument '%1' is only applicable to ASCII format!").arg(argument));
			}
			
			AsciiFilter::SavePointCountHeader(true);
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	return true;
}

CommandChangeMeshOutputFormat::CommandChangeMeshOutputFormat()
	: CommandChangeOutputFormat(QObject::tr("Change mesh output format"), COMMAND_MESH_EXPORT_FORMAT)
{}

bool CommandChangeMeshOutputFormat::process(ccCommandLineInterface& cmd)
{
	QString defaultExt;
	QString fileFilter = getFileFormatFilter(cmd, defaultExt);
	if (fileFilter.isEmpty())
	{
		return false;
	}
	
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
			{
				return cmd.error(QObject::tr("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));
			}
			
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

CommandChangeHierarchyOutputFormat::CommandChangeHierarchyOutputFormat()
	: CommandChangeOutputFormat(QObject::tr("Change hierarchy output format"), COMMAND_HIERARCHY_EXPORT_FORMAT)
{}

bool CommandChangeHierarchyOutputFormat::process(ccCommandLineInterface& cmd)
{
	QString defaultExt;
	QString fileFilter = getFileFormatFilter(cmd, defaultExt);
	if (fileFilter.isEmpty())
	{
		return false;
	}

	cmd.setHierarchyExportFormat(fileFilter, defaultExt);
	cmd.print(QObject::tr("Output export format (hierarchy) set to: %1").arg(defaultExt.toUpper()));

	//look for additional parameters
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();

		if (ccCommandLineInterface::IsCommand(argument, COMMAND_EXPORT_EXTENSION))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: extension after '%1'").arg(COMMAND_EXPORT_EXTENSION));
			}

			cmd.setHierarchyExportFormat(cmd.hierarchyExportFormat(), cmd.arguments().takeFirst());
			cmd.print(QObject::tr("New output extension for hierarchies: %1").arg(cmd.hierarchyExportExt()));
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}

	return true;
}

CommandLoad::CommandLoad()
	: ccCommandLineInterface::Command(QObject::tr("Load"), COMMAND_OPEN)
{}

bool CommandLoad::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[LOADING]"));
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: filename after \"-%1\"").arg(COMMAND_OPEN));
	}
	
	//optional parameters
	int skipLines = 0;
	ccCommandLineInterface::GlobalShiftOptions globalShiftOptions;

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

			if (!cmd.processGlobalShiftCommand(globalShiftOptions))
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
	
	if (skipLines >= 0)
	{
		AsciiFilter::SetDefaultSkippedLineCount(skipLines);
	}
	
	//open specified file
	QString filename(cmd.arguments().takeFirst());
	if (!cmd.importFile(filename, globalShiftOptions))
	{
		return false;
	}

	return true;
}

CommandClearNormals::CommandClearNormals()
	: ccCommandLineInterface::Command(QObject::tr("Clears normals"), COMMAND_CLEAR_NORMALS)
{}

bool CommandClearNormals::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CLEAR NORMALS]"));
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

CommandInvertNormal::CommandInvertNormal()
	: ccCommandLineInterface::Command(QObject::tr("Invert normals"), COMMAND_INVERT_NORMALS)
{}

bool CommandInvertNormal::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[INVERT NORMALS]"));

	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No input point cloud or mesh (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_INVERT_NORMALS));
	}

	for (CLCloudDesc& thisCloudDesc : cmd.clouds())
	{
		ccPointCloud* cloud = thisCloudDesc.pc;

		if (!cloud->hasNormals())
		{
			cmd.warning(QObject::tr("Cloud %1 has no normals").arg(cloud->getName()));
			continue;
		}

		cloud->invertNormals();

		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(thisCloudDesc, "_INVERTED_NORMALS");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	for (CLMeshDesc& thisMeshDesc : cmd.meshes())
	{
		ccMesh* mesh = ccHObjectCaster::ToMesh(thisMeshDesc.mesh);
		if (!mesh)
		{
			assert(false);
			continue;
		}

		if (!mesh->hasNormals())
		{
			cmd.warning(QObject::tr("Mesh %1 has no normals").arg(mesh->getName()));
			continue;
		}

		mesh->invertNormals();

		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(thisMeshDesc, "_INVERTED_NORMALS");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	return true;
}
CommandOctreeNormal::CommandOctreeNormal()
	: ccCommandLineInterface::Command(QObject::tr("Compute normals with octree"), COMMAND_COMPUTE_OCTREE_NORMALS)
{}

bool CommandOctreeNormal::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[OCTREE NORMALS CALCULATION]"));
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud to compute normals (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_COMPUTE_OCTREE_NORMALS));
	}
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: radius after \"-%1\"").arg(COMMAND_COMPUTE_OCTREE_NORMALS));
	}
	
	float radius = std::numeric_limits<float>::quiet_NaN(); //if this stays 
	QString radiusArg = cmd.arguments().takeFirst();
	if (radiusArg.toUpper() != "AUTO")
	{
		bool ok = false;
		radius = radiusArg.toFloat(&ok);
		if (!ok)
		{
			return cmd.error(QObject::tr("Invalid radius"));
		}
	}
	cmd.print(QObject::tr("\tRadius: %1").arg(radiusArg));

	CCCoreLib::LOCAL_MODEL_TYPES model = CCCoreLib::QUADRIC;
	ccNormalVectors::Orientation  orientation = ccNormalVectors::Orientation::UNDEFINED;
	
	while (!cmd.arguments().isEmpty())
	{
		QString argument = cmd.arguments().front().toUpper();
		if (ccCommandLineInterface::IsCommand(argument, OPTION_ORIENT))
		{
			cmd.arguments().takeFirst();
			if (!cmd.arguments().isEmpty())
			{
				QString orient_argument = cmd.arguments().takeFirst().toUpper();
				if (orient_argument == "PLUS_ZERO" || orient_argument == "PLUS_ORIGIN")
				{
					orientation = ccNormalVectors::Orientation::PLUS_ORIGIN;
				}
				else if (orient_argument == "MINUS_ZERO" || orient_argument == "MINUS_ORIGIN")
				{
					orientation = ccNormalVectors::Orientation::MINUS_ORIGIN;
				}
				else if (orient_argument == "PLUS_BARYCENTER")
				{
					orientation = ccNormalVectors::Orientation::PLUS_BARYCENTER;
				}
				else if (orient_argument == "MINUS_BARYCENTER")
				{
					orientation = ccNormalVectors::Orientation::MINUS_BARYCENTER;
				}
				else if (orient_argument == "PLUS_X")
				{
					orientation = ccNormalVectors::Orientation::PLUS_X;
				}
				else if (orient_argument == "MINUS_X")
				{
					orientation = ccNormalVectors::Orientation::MINUS_X;
				}
				else if (orient_argument == "PLUS_Y")
				{
					orientation = ccNormalVectors::Orientation::PLUS_Y;
				}
				else if (orient_argument == "MINUS_Y")
				{
					orientation = ccNormalVectors::Orientation::MINUS_Y;
				}
				else if (orient_argument == "PLUS_Z")
				{
					orientation = ccNormalVectors::Orientation::PLUS_Z;
				}
				else if (orient_argument == "MINUS_Z")
				{
					orientation = ccNormalVectors::Orientation::MINUS_Z;
				}
				else if (orient_argument == "PREVIOUS")
				{
					orientation = ccNormalVectors::Orientation::PREVIOUS;
				}
				else if (orient_argument == "PLUS_SENSOR_ORIGIN")
				{
					orientation = ccNormalVectors::Orientation::PLUS_SENSOR_ORIGIN;
				}
				else if (orient_argument == "MINUS_SENSOR_ORIGIN")
				{
					orientation = ccNormalVectors::Orientation::MINUS_SENSOR_ORIGIN;
				}
				else
				{
					return cmd.error(QObject::tr("Invalid parameter: unknown orientation '%1'").arg(orient_argument));
				}
			}
			else
			{
				return cmd.error(QObject::tr("Missing orientation"));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, OPTION_MODEL))
		{
			cmd.arguments().takeFirst();
			if (!cmd.arguments().isEmpty())
			{
				QString model_arg = cmd.arguments().takeFirst().toUpper();
				if (model_arg == "LS")
				{
					model = CCCoreLib::LOCAL_MODEL_TYPES::LS;
				}
				else if (model_arg == "TRI")
				{
					model = CCCoreLib::LOCAL_MODEL_TYPES::TRI;
				}
				else if (model_arg == "QUADRIC")
				{
					model = CCCoreLib::LOCAL_MODEL_TYPES::QUADRIC;
				}
				else
				{
					return cmd.error(QObject::tr("Invalid parameter: unknown model '%1'").arg(model_arg));
				}
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

		QScopedPointer<ccProgressDialog> progressDialog(nullptr);
		if (!cmd.silentMode())
		{
			progressDialog.reset(new ccProgressDialog(true, cmd.widgetParent()));
			progressDialog->setAutoClose(false);
		}

		if (!cloud->getOctree())
		{
			if (!cloud->computeOctree(progressDialog.data()))
			{
				return cmd.error(QObject::tr("Failed to compute octree for cloud '%1'").arg(cloud->getName()));
			}
		}

		float thisCloudRadius = radius;
		if (std::isnan(thisCloudRadius))
		{
			ccOctree::BestRadiusParams params;
			{
				params.aimedPopulationPerCell = 16;
				params.aimedPopulationRange = 4;
				params.minCellPopulation = 6;
				params.minAboveMinRatio = 0.97;
			}
			thisCloudRadius = ccOctree::GuessBestRadius(cloud, params, cloud->getOctree().data());
			if (thisCloudRadius == 0)
			{
				return cmd.error(QObject::tr("Failed to determine best normal radius for cloud '%1'").arg(cloud->getName()));
			}
			cmd.print(QObject::tr("\tCloud %1 radius = %2").arg(cloud->getName()).arg(thisCloudRadius));
		}

		cmd.print(QObject::tr("computeNormalsWithOctree started..."));
		bool success = cloud->computeNormalsWithOctree(model, orientation, thisCloudRadius, progressDialog.data());
		if(success)
		{
			cmd.print(QObject::tr("computeNormalsWithOctree success"));
		}
		else
		{
			return cmd.error(QObject::tr("computeNormalsWithOctree failed"));
		}
		
		cloud->setName(cloud->getName() + QObject::tr(".OctreeNormal"));
		if (cmd.autoSaveMode())
		{
			CLCloudDesc desc(cloud, thisCloudDesc.basename, thisCloudDesc.path, thisCloudDesc.indexInFile);
			QString errorStr = cmd.exportEntity(desc, "OCTREE_NORMALS");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}
	
	return true;
}

CommandConvertNormalsToDipAndDipDir::CommandConvertNormalsToDipAndDipDir()
	: ccCommandLineInterface::Command(QObject::tr("Convert normals to dip and dip. dir."), COMMAND_CONVERT_NORMALS_TO_DIP)
{}

bool CommandConvertNormalsToDipAndDipDir::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CONVERT NORMALS TO DIP/DIP DIR]"));
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No input point cloud (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_CONVERT_NORMALS_TO_DIP));
	}

	for (CLCloudDesc& thisCloudDesc : cmd.clouds())
	{
		ccPointCloud* cloud = thisCloudDesc.pc;

		if (!cloud->hasNormals())
		{
			cmd.warning(QObject::tr("Cloud %1 has no normals").arg(cloud->getName()));
			continue;
		}

		ccHObject::Container container;
		container.push_back(cloud);
		if (!ccEntityAction::convertNormalsTo(container, ccEntityAction::NORMAL_CONVERSION_DEST::DIP_DIR_SFS))
		{
			return cmd.error(QObject::tr("Failed to convert normals to dip and dip direction"));
		}

		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(thisCloudDesc, "_DIP_AND_DIP_DIR");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	return true;
}

CommandConvertNormalsToSFs::CommandConvertNormalsToSFs()
	: ccCommandLineInterface::Command(QObject::tr("Convert normals to scalar fields"), COMMAND_CONVERT_NORMALS_TO_SFS)
{}

bool CommandConvertNormalsToSFs::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CONVERT NORMALS TO SCALAR FIELD(S)]"));
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No input point cloud (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_CONVERT_NORMALS_TO_SFS));
	}

	for (CLCloudDesc& thisCloudDesc : cmd.clouds())
	{
		ccPointCloud* cloud = thisCloudDesc.pc;

		if (!cloud->hasNormals())
		{
			cmd.warning(QObject::tr("Cloud %1 has no normals").arg(cloud->getName()));
			continue;
		}

		bool exportDims[3] = { true, true, true };

		ccHObject::Container container;
		container.push_back(cloud);
		if (!ccEntityAction::exportNormalToSF(container, cmd.widgetParent(), exportDims))
		{
			return cmd.error(QObject::tr("Failed to convert normals to scalar fields"));
		}

		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(thisCloudDesc, "_NORM_TO_SF");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	return true;
}

CommandConvertNormalsToHSV::CommandConvertNormalsToHSV()
	: ccCommandLineInterface::Command(QObject::tr("Convert normals to HSV colors"), COMMAND_CONVERT_NORMALS_TO_HSV)
{}

bool CommandConvertNormalsToHSV::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CONVERT NORMALS TO HSV COLORS]"));
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No input point cloud (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_CONVERT_NORMALS_TO_HSV));
	}

	for (CLCloudDesc& thisCloudDesc : cmd.clouds())
	{
		ccPointCloud* cloud = thisCloudDesc.pc;

		if (!cloud->hasNormals())
		{
			cmd.warning(QObject::tr("Cloud %1 has no normals").arg(cloud->getName()));
			continue;
		}

		ccHObject::Container container;
		container.push_back(cloud);
		if (!ccEntityAction::convertNormalsTo(container, ccEntityAction::NORMAL_CONVERSION_DEST::HSV_COLORS))
		{
			return cmd.error(QObject::tr("Failed to convert normals to HSV colors"));
		}

		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(thisCloudDesc, "_HSV");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	return true;
}

CommandSubsample::CommandSubsample()
	: ccCommandLineInterface::Command(QObject::tr("Subsample"), COMMAND_SUBSAMPLE)
{}

bool CommandSubsample::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SUBSAMPLING]"));
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
			return cmd.error(QObject::tr("Invalid number of points for random resampling!"));
		}
		cmd.print(QObject::tr("\tOutput points: %1").arg(count));
		
		for (CLCloudDesc& desc : cmd.clouds())
		{
			cmd.print(QObject::tr("\tProcessing cloud %1").arg(!desc.pc->getName().isEmpty() ? desc.pc->getName() : "no name"));
			
			CCCoreLib::ReferenceCloud* refCloud = CCCoreLib::CloudSamplingTools::subsampleCloudRandomly(desc.pc, count, cmd.progressDialog());
			if (!refCloud)
			{
				return cmd.error(QObject::tr("Subsampling process failed!"));
			}
			cmd.print(QObject::tr("\tResult: %1 points").arg(refCloud->size()));
			
			//save output
			ccPointCloud* result = desc.pc->partialClone(refCloud);
			delete refCloud;
			refCloud = nullptr;
			
			if (result)
			{
				result->setName(desc.pc->getName() + QObject::tr(".subsampled"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc newDesc(result, desc.basename, desc.path, desc.indexInFile);
					QString errorStr = cmd.exportEntity(newDesc, "RANDOM_SUBSAMPLED");
					if (!errorStr.isEmpty())
					{
						delete result;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete desc.pc;
				desc.pc = result;
				desc.basename += QObject::tr("_SUBSAMPLED");
				//delete result;
				//result = 0;
			}
			else
			{
				return cmd.error(QObject::tr("Not enough memory!"));
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
			return cmd.error(QObject::tr("Invalid step value for spatial resampling!"));
		}
		cmd.print(QObject::tr("\tSpatial step: %1").arg(step));
		
		for (CLCloudDesc& desc : cmd.clouds())
		{
			cmd.print(QObject::tr("\tProcessing cloud %1").arg(!desc.pc->getName().isEmpty() ? desc.pc->getName() : "no name"));
			
			CCCoreLib::CloudSamplingTools::SFModulationParams modParams(false);
			CCCoreLib::ReferenceCloud* refCloud = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(desc.pc, static_cast<PointCoordinateType>(step), modParams, nullptr, cmd.progressDialog());
			if (!refCloud)
			{
				return cmd.error("Subsampling process failed!");
			}
			cmd.print(QObject::tr("\tResult: %1 points").arg(refCloud->size()));
			
			//save output
			ccPointCloud* result = desc.pc->partialClone(refCloud);
			delete refCloud;
			refCloud = nullptr;
			
			if (result)
			{
				result->setName(desc.pc->getName() + QObject::tr(".subsampled"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc newDesc(result, desc.basename, desc.path, desc.indexInFile);
					QString errorStr = cmd.exportEntity(newDesc, "SPATIAL_SUBSAMPLED");
					if (!errorStr.isEmpty())
					{
						delete result;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete desc.pc;
				desc.pc = result;
				desc.basename += QObject::tr("_SUBSAMPLED");
				//delete result;
				//result = 0;
			}
			else
			{
				return cmd.error(QObject::tr("Not enough memory!"));
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
		if (!ok || octreeLevel < 1 || octreeLevel > CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL)
		{
			return cmd.error(QObject::tr("Invalid octree level!"));
		}
		cmd.print(QObject::tr("\tOctree level: %1").arg(octreeLevel));
		
		QScopedPointer<ccProgressDialog> progressDialog(nullptr);
		if (!cmd.silentMode())
		{
			progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
			progressDialog->setAutoClose(false);
		}
		
		for (CLCloudDesc& desc : cmd.clouds())
		{
			cmd.print(QObject::tr("\tProcessing cloud %1").arg(!desc.pc->getName().isEmpty() ? desc.pc->getName() : "no name"));
			
			CCCoreLib::ReferenceCloud* refCloud = CCCoreLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	desc.pc,
																													static_cast<unsigned char>(octreeLevel),
																													CCCoreLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																													progressDialog.data());
			if (!refCloud)
			{
				return cmd.error(QObject::tr("Subsampling process failed!"));
			}
			cmd.print(QObject::tr("\tResult: %1 points").arg(refCloud->size()));
			
			//save output
			ccPointCloud* result = desc.pc->partialClone(refCloud);
			delete refCloud;
			refCloud = nullptr;
			
			if (result)
			{
				result->setName(desc.pc->getName() + QObject::tr(".subsampled"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc newDesc(result, desc.basename, desc.path, desc.indexInFile);
					QString errorStr = cmd.exportEntity(newDesc, QObject::tr("OCTREE_LEVEL_%1_SUBSAMPLED").arg(octreeLevel));
					if (!errorStr.isEmpty())
					{
						delete result;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete desc.pc;
				desc.pc = result;
				desc.basename += QObject::tr("_SUBSAMPLED");
				//delete result;
				//result = 0;
			}
			else
			{
				return cmd.error(QObject::tr("Not enough memory!"));
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
		return cmd.error(QObject::tr("Unknown method!"));
	}
	
	return true;
}

CommandExtractCCs::CommandExtractCCs()
	: ccCommandLineInterface::Command(QObject::tr("ExtractCCs"), COMMAND_EXTRACT_CC)
{}

bool CommandExtractCCs::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CONNECTED COMPONENTS EXTRACTION]"));
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
	unsigned char octreeLevel = std::min<unsigned char>(cmd.arguments().takeFirst().toUShort(&ok), CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL);
	if (!ok)
	{
		return cmd.error(QObject::tr("Invalid octree level!"));
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
		return cmd.error(QObject::tr("Invalid min. number of points!"));
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
		
		std::vector<CLCloudDesc> inputClouds = cmd.clouds();
		cmd.clouds().clear();
		for (CLCloudDesc& desc : inputClouds)
		{
			cmd.print(QObject::tr("\tProcessing cloud %1").arg(!desc.pc->getName().isEmpty() ? desc.pc->getName() : "no name"));
			
			//we create/activate CCs label scalar field
			int sfIdx = desc.pc->getScalarFieldIndexByName(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			if (sfIdx < 0)
			{
				sfIdx = desc.pc->addScalarField(CC_CONNECTED_COMPONENTS_DEFAULT_LABEL_NAME);
			}
			if (sfIdx < 0)
			{
				cmd.error(QObject::tr("Couldn't allocate a new scalar field for computing CC labels! Try to free some memory ..."));
				continue;
			}
			desc.pc->setCurrentScalarField(sfIdx);
			
			//try to label all CCs
			int componentCount = CCCoreLib::AutoSegmentationTools::labelConnectedComponents(desc.pc,
																							static_cast<unsigned char>(octreeLevel),
																							false,
																							progressDialog.data());
			
			if (componentCount == 0)
			{
				cmd.error(QObject::tr("No component found!"));
				continue;
			}
			
			desc.pc->getCurrentInScalarField()->computeMinAndMax();
			CCCoreLib::ReferenceCloudContainer components;
			bool success = CCCoreLib::AutoSegmentationTools::extractConnectedComponents(desc.pc, components);
			desc.pc->deleteScalarField(sfIdx);
			sfIdx = -1;
			
			if (!success)
			{
				cmd.warning(QObject::tr("An error occurred (failed to finish the extraction)"));
				continue;
			}
			
			//we create "real" point clouds for all input components
			int realIndex = 0;
			for (size_t j = 0; j < components.size(); ++j)
			{
				CCCoreLib::ReferenceCloud* compIndexes = components[j];
				
				//if it has enough points
				if (compIndexes->size() >= minPointCount)
				{
					//we create a new entity
					ccPointCloud* compCloud = desc.pc->partialClone(compIndexes);
					if (compCloud)
					{
						//'shift on load' information
						compCloud->copyGlobalShiftAndScale(*desc.pc);
						compCloud->setName(QString(desc.pc->getName() + "_CC#%1").arg(j + 1));
						
						CLCloudDesc newDesc(compCloud, desc.basename + QObject::tr("_COMPONENT_%1").arg(++realIndex), desc.path);
						if (cmd.autoSaveMode())
						{
							QString errorStr = cmd.exportEntity(newDesc, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
							if (!errorStr.isEmpty())
							{
								cmd.error(errorStr);
							}
						}
						//add newDesc to the current pool
						cmd.clouds().push_back(newDesc);
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
				cmd.error(QObject::tr("No component was created! Check the minimum size..."));
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
		cmd.error(QObject::tr("Not enough memory"));
		return false;
	}
	
	return true;
}

CommandCurvature::CommandCurvature()
	: ccCommandLineInterface::Command(QObject::tr("Curvature"), COMMAND_CURVATURE)
{}

bool CommandCurvature::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CURVATURE]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: curvature type after \"-%1\"").arg(COMMAND_CURVATURE));
	}
	
	QString curvTypeStr = cmd.arguments().takeFirst().toUpper();
	CCCoreLib::Neighbourhood::CurvatureType curvType = CCCoreLib::Neighbourhood::MEAN_CURV;
	if (curvTypeStr == "MEAN")
	{
		//curvType = CCCoreLib::Neighbourhood::MEAN_CURV;
	}
	else if (curvTypeStr == "GAUSS")
	{
		curvType = CCCoreLib::Neighbourhood::GAUSSIAN_CURV;
	}
	else if (curvTypeStr == "NORMAL_CHANGE")
	{
		curvType = CCCoreLib::Neighbourhood::NORMAL_CHANGE_RATE;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid curvature type after \"-%1\". Got '%2' instead of MEAN or GAUSS.").arg(COMMAND_CURVATURE, curvTypeStr));
	}
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: kernel size after curvature type"));
	}
	
	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
	{
		return cmd.error(QObject::tr("Failed to read a numerical parameter: kernel size (after curvature type). Got '%1' instead.").arg(kernelStr));
	}
	cmd.print(QObject::tr("\tKernel size: %1").arg(kernelSize));
	
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to compute curvature! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_CURVATURE));
	}
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		entities[i] = cmd.clouds()[i].pc;
	}
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCCoreLib::GeometricalAnalysisTools::Curvature, curvType, kernelSize, entities, nullptr, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("%1_CURVATURE_KERNEL_%2").arg(curvTypeStr).arg(kernelSize)))
		{
			return false;
		}
	}
	return true;
}

static bool ReadDensityType(ccCommandLineInterface& cmd, CCCoreLib::GeometricalAnalysisTools::Density& density)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
	}
	
	//read option confirmed, we can move on
	QString typeArg = cmd.arguments().takeFirst().toUpper();
	if (typeArg == "KNN")
	{
		density = CCCoreLib::GeometricalAnalysisTools::DENSITY_KNN;
	}
	else if (typeArg == "SURFACE")
	{
		density = CCCoreLib::GeometricalAnalysisTools::DENSITY_2D;
	}
	else if (typeArg == "VOLUME")
	{
		density = CCCoreLib::GeometricalAnalysisTools::DENSITY_3D;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid parameter: density type is expected after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
	}

	return true;
}

CommandApproxDensity::CommandApproxDensity()
	: ccCommandLineInterface::Command(QObject::tr("ApproxDensity"), COMMAND_APPROX_DENSITY)
{}

bool CommandApproxDensity::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[APPROX DENSITY]"));
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to compute approx. density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_APPROX_DENSITY));
	}
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		entities[i] = cmd.clouds()[i].pc;
	}
	
	//optional parameter: density type
	CCCoreLib::GeometricalAnalysisTools::Density densityType = CCCoreLib::GeometricalAnalysisTools::DENSITY_3D;
	if (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_DENSITY_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
			}
			//read option confirmed, we can move on
			if (!ReadDensityType(cmd, densityType))
			{
				return false;
			}
		}
	}
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCCoreLib::GeometricalAnalysisTools::ApproxLocalDensity, densityType, 0, entities, nullptr, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds("APPROX_DENSITY"))
		{
			return false;
		}
	}
	
	return true;
}

CommandDensity::CommandDensity()
	: ccCommandLineInterface::Command(QObject::tr("Density"), COMMAND_DENSITY)
{}

bool CommandDensity::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[DENSITY]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: sphere radius after \"-%1\"").arg(COMMAND_DENSITY));
	}
	
	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
	{
		return cmd.error(QObject::tr("Failed to read a numerical parameter: sphere radius (after \"-%1\"). Got '%2' instead.").arg(COMMAND_DENSITY, kernelStr));
	}
	cmd.print(QObject::tr("\tSphere radius: %1").arg(kernelSize));
	
	//optional parameter: density type
	CCCoreLib::GeometricalAnalysisTools::Density densityType = CCCoreLib::GeometricalAnalysisTools::DENSITY_3D;
	if (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_DENSITY_TYPE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: density type after \"-%1\" (KNN/SURFACE/VOLUME)").arg(COMMAND_DENSITY_TYPE));
			}
			//read option confirmed, we can move on
			if (!ReadDensityType(cmd, densityType))
			{
				return false;
			}
		}
	}
	
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to compute density! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_DENSITY));
	}
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		entities[i] = cmd.clouds()[i].pc;
	}
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCCoreLib::GeometricalAnalysisTools::LocalDensity, densityType, kernelSize, entities, nullptr, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds("DENSITY"))
		{
			return false;
		}
	}
	
	return true;
}

CommandSFGradient::CommandSFGradient()
	: ccCommandLineInterface::Command(QObject::tr("SF gradient"), COMMAND_SF_GRADIENT)
{}

bool CommandSFGradient::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SF GRADIENT]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: boolean (whether SF is euclidean or not) after \"-%1\"").arg(COMMAND_SF_GRADIENT));
	}
	
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
	{
		return cmd.error(QObject::tr("No point cloud on which to compute SF gradient! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SF_GRADIENT));
	}
	
	//Call MainWindow generic method
	void* additionalParameters[1] = { &euclidean };
	ccHObject::Container entities;
	entities.reserve(cmd.clouds().size());
	for (CLCloudDesc& desc : cmd.clouds())
	{
		unsigned sfCount = desc.pc->getNumberOfScalarFields();
		if (sfCount == 0)
		{
			cmd.warning(QObject::tr("cmd.warning: cloud '%1' has no scalar field (it will be ignored)").arg(desc.pc->getName()));
		}
		else
		{
			if (sfCount > 1)
			{
				cmd.warning(QObject::tr("cmd.warning: cloud '%1' has several scalar fields (the active one will be used by default, or the first one if none is active)").arg(desc.pc->getName()));
			}
			
			int activeSFIndex = desc.pc->getCurrentOutScalarFieldIndex();
			if (activeSFIndex < 0)
			{
				activeSFIndex = 0;
			}
			
			desc.pc->setCurrentDisplayedScalarField(activeSFIndex);
			
			entities.push_back(desc.pc);
		}
	}
	
	if (ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_SF_GRADIENT, entities, cmd.widgetParent(), additionalParameters))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(euclidean ? "EUCLIDEAN_SF_GRAD" : "SF_GRAD"))
		{
			return false;
		}
	}
	
	return true;
}

CommandRoughness::CommandRoughness()
	: ccCommandLineInterface::Command(QObject::tr("Roughness"), COMMAND_ROUGHNESS)
{}

bool CommandRoughness::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[ROUGHNESS]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: kernel size after \"-%1\"").arg(COMMAND_ROUGHNESS));
	}
	
	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
	{
		return cmd.error(QObject::tr("Failed to read a numerical parameter: kernel size (after \"-%1\"). Got '%2' instead.").arg(COMMAND_ROUGHNESS, kernelStr));
	}
	cmd.print(QObject::tr("\tKernel size: %1").arg(kernelSize));

	// optional argument
	CCVector3 roughnessUpDir;
	CCVector3* _roughnessUpDir = nullptr;
	if (cmd.arguments().size() >= 4)
	{
		QString nextArg = cmd.arguments().first();
		if (nextArg.startsWith('-') && nextArg.mid(1).toUpper() == COMMAND_ROUGHNESS_UP_DIR)
		{
			// option confirmed
			cmd.arguments().takeFirst();
			QString xStr = cmd.arguments().takeFirst();
			QString yStr = cmd.arguments().takeFirst();
			QString zStr = cmd.arguments().takeFirst();
			bool okX = false, okY = false, okZ = false;
			roughnessUpDir.x = static_cast<PointCoordinateType>(xStr.toDouble(&okX));
			roughnessUpDir.y = static_cast<PointCoordinateType>(yStr.toDouble(&okY));
			roughnessUpDir.z = static_cast<PointCoordinateType>(zStr.toDouble(&okZ));
			if (!okX || !okY || !okZ)
			{
				return cmd.error(QObject::tr("Invalid 'up direction' vector after option -%1 (3 coordinates expected)").arg(COMMAND_ROUGHNESS_UP_DIR));
			}
			_roughnessUpDir = &roughnessUpDir;
		}
	}
	
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to compute roughness! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_ROUGHNESS));
	}
	
	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		entities[i] = cmd.clouds()[i].pc;
	}
	
	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCCoreLib::GeometricalAnalysisTools::Roughness, 0, kernelSize, entities, _roughnessUpDir, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("ROUGHNESS_KERNEL_%2").arg(kernelSize)))
		{
			return false;
		}
	}
	
	return true;
}

CommandApplyTransformation::CommandApplyTransformation()
	: ccCommandLineInterface::Command(QObject::tr("Apply Transformation"), COMMAND_APPLY_TRANSFORMATION)
{}

bool CommandApplyTransformation::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[APPLY TRANSFORMATION]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: transformation file after \"-%1\"").arg(COMMAND_APPLY_TRANSFORMATION));
	}
	
	QString filename = cmd.arguments().takeFirst();
	ccGLMatrix mat;
	if (!mat.fromAsciiFile(filename))
	{
		return cmd.error(QObject::tr("Failed to read transformation matrix file '%1'!").arg(filename));
	}
	
	cmd.print(QObject::tr("Transformation:\n") + mat.toString(6));
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No entity on which to apply the transformation! (be sure to open one with \"-%1 [filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_APPLY_TRANSFORMATION));
	}
	
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
		{
			return false;
		}
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
		{
			return false;
		}
	}
	
	return true;
}

CommandDropGlobalShift::CommandDropGlobalShift()
	: ccCommandLineInterface::Command(QObject::tr("Drop global shift"), COMMAND_DROP_GLOBAL_SHIFT)
{}

bool CommandDropGlobalShift::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[DROP GLOBAL SHIFT]"));
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No loaded entity! (be sure to open one with \"-%1 [filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_DROP_GLOBAL_SHIFT));
	}
	
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
	: ccCommandLineInterface::Command(QObject::tr("SF color scale"), COMMAND_SF_COLOR_SCALE)
{}

bool CommandSFColorScale::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SF COLOR SCALE]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: color scale file after \"-%1\"").arg(COMMAND_SF_COLOR_SCALE));
	}
	
	QString filename = cmd.arguments().takeFirst();
	
	ccColorScale::Shared scale = ccColorScale::LoadFromXML(filename);
	
	if (!scale)
	{
		return cmd.error(QObject::tr("Failed to read color scale file '%1'!").arg(filename));
	}
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No point cloud or mesh on which to set the SF color scale! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SF_COLOR_SCALE));
	}
	
	// clouds
	if (!cmd.clouds().empty())
	{
		bool hasCandidateClouds = false;
		for (auto& cloud : cmd.clouds())
		{
			ccScalarField* sf = static_cast<ccScalarField*>(cloud.pc->getCurrentOutScalarField());
			if (sf)
			{
				sf->setColorScale(scale);
				hasCandidateClouds = true;
			}
		}

		if (hasCandidateClouds && cmd.autoSaveMode() && !cmd.saveClouds("COLOR_SCALE"))
		{
			return false;
		}
	}
	
	// meshes
	if (!cmd.meshes().empty())
	{
		bool hasCandidateMeshes = false;
		for (auto& mesh : cmd.meshes())
		{
			ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(mesh.mesh->getAssociatedCloud());
			if (vertices)
			{
				ccScalarField* sf = static_cast<ccScalarField*>(vertices->getCurrentOutScalarField());
				if (sf)
				{
					sf->setColorScale(scale);
					hasCandidateMeshes = true;
				}
			}
		}

		if (hasCandidateMeshes && cmd.autoSaveMode() && !cmd.saveMeshes("COLOR_SCALE"))
		{
			return false;
		}
	}

	return true;
}

CommandSFConvertToRGB::CommandSFConvertToRGB()
	: ccCommandLineInterface::Command(QObject::tr("SF convert to RGB"), COMMAND_SF_CONVERT_TO_RGB)
{}

bool CommandSFConvertToRGB::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SF CONVERT TO RGB]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: boolean (whether to mix with existing colors or not) after \"-%1\"").arg(COMMAND_SF_CONVERT_TO_RGB));
	}
	
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
	{
		return cmd.error(QObject::tr("No point cloud on which to convert SF to RGB! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SF_CONVERT_TO_RGB));
	}
	
	for (CLCloudDesc& desc : cmd.clouds())
	{
		unsigned sfCount = desc.pc->getNumberOfScalarFields();
		int activeSFIndex = desc.pc->getCurrentOutScalarFieldIndex();
		
		if (sfCount == 0)
		{
			cmd.warning(QObject::tr("cmd.warning: cloud '%1' has no scalar field (it will be ignored)").arg(desc.pc->getName()));
		}
		else if (activeSFIndex < 0)
		{
			cmd.warning(QObject::tr("cmd.warning: cloud '%1' has no active scalar field (it will be ignored)").arg(desc.pc->getName()));
		}
		else
		{
			int displaySFIndex = desc.pc->getCurrentDisplayedScalarFieldIndex();
			desc.pc->setCurrentDisplayedScalarField(activeSFIndex);
			
			if (desc.pc->convertCurrentScalarFieldToColors(mixWithExistingColors))
			{
				desc.pc->showColors(true);
				desc.pc->showSF(false);
			}
			else
			{
				cmd.warning(QObject::tr("cmd.warning: cloud '%1' failed to convert SF to RGB").arg(desc.pc->getName()));
			}
			
			desc.pc->setCurrentDisplayedScalarField(displaySFIndex);
		}
	}
	
	if (cmd.autoSaveMode() && !cmd.saveClouds("SF_CONVERT_TO_RGB"))
	{
		return false;
	}
	
	return true;
}

CommandRGBConvertToSF::CommandRGBConvertToSF()
	: ccCommandLineInterface::Command(QObject::tr("RGB convert to SF"), COMMAND_RGB_CONVERT_TO_SF)
{}

bool CommandRGBConvertToSF::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[RGB CONVERT TO SF]"));
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to convert RGB to SF! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_RGB_CONVERT_TO_SF));
	}

	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (!desc.pc->hasColors())
		{
			cmd.warning(QObject::tr("Cloud %1 has no colors").arg(desc.pc->getName()));
			continue;
		}

		ccHObject::Container container;
		container.push_back(desc.pc);
		if (!ccEntityAction::sfFromColor(container, /*exportR=*/true, /*exportG=*/true, /*exportB=*/true, /*exportAlpha=*/true, /*exportC=*/true)) //beta version, only composite
		{
			return cmd.error(QObject::tr("Failed to convert RGB to scalar fields"));
		}
        

		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(desc, "_RGB_TO_SF");
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	return true;
}

CommandFilterBySFValue::CommandFilterBySFValue()
	: ccCommandLineInterface::Command(QObject::tr("Filter by SF value"), COMMAND_FILTER_SF_BY_VALUE)
{}

//special SF values that can be used instead of explicit ones
enum USE_SPECIAL_SF_VALUE
{
	USE_NONE,
	USE_MIN,
	USE_DISP_MIN,
	USE_SAT_MIN,
	USE_N_SIGMA_MIN,
	USE_MAX,
	USE_DISP_MAX,
	USE_SAT_MAX,
	USE_N_SIGMA_MAX
};

static std::pair<ScalarType, ScalarType> GetSFRange(const CCCoreLib::ScalarField& sf,
													ScalarType minVal,
													USE_SPECIAL_SF_VALUE useValForMin,
													ScalarType maxVal,
													USE_SPECIAL_SF_VALUE useValForMax)
{
	ScalarType thisMinVal = minVal;
	{
		switch (useValForMin)
		{
		case USE_MIN:
			thisMinVal = sf.getMin();
			break;
		case USE_DISP_MIN:
			thisMinVal = static_cast<const ccScalarField&>(sf).displayRange().start();
			break;
		case USE_SAT_MIN:
			thisMinVal = static_cast<const ccScalarField&>(sf).saturationRange().start();
			break;
		case USE_N_SIGMA_MIN:
			ScalarType mean;
			ScalarType variance;
			sf.computeMeanAndVariance(mean, &variance);
			thisMinVal = mean - (sqrt(variance) * minVal);
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
			thisMaxVal = sf.getMax();
			break;
		case USE_DISP_MAX:
			thisMaxVal = static_cast<const ccScalarField&>(sf).displayRange().stop();
			break;
		case USE_SAT_MAX:
			thisMaxVal = static_cast<const ccScalarField&>(sf).saturationRange().stop();
			break;
		case USE_N_SIGMA_MAX:
			ScalarType mean;
			ScalarType variance;
			sf.computeMeanAndVariance(mean, &variance);
			thisMaxVal = mean + (sqrt(variance) * maxVal);
			break;
		default:
			//nothing to do
			break;
		}
	}

	return { thisMinVal, thisMaxVal };
}

static USE_SPECIAL_SF_VALUE ToSpecialSFValue(QString valString)
{
	valString = valString.toUpper();

	if (valString == "MIN")
	{
		return USE_MIN;
	}
	else if (valString == "DISP_MIN")
	{
		return USE_DISP_MIN;
	}
	else if (valString == "SAT_MIN")
	{
		return USE_SAT_MIN;
	}
	else if (valString == "N_SIGMA_MIN")
	{
		return USE_N_SIGMA_MIN;
	}
	else if (valString == "MAX")
	{
		return USE_MAX;
	}
	else if (valString == "DISP_MAX")
	{
		return USE_DISP_MAX;
	}
	else if (valString == "SAT_MAX")
	{
		return USE_SAT_MAX;
	}
	else if (valString == "N_SIGMA_MAX")
	{
		return USE_N_SIGMA_MAX;
	}
	else
	{
		return USE_NONE;
	}
}


bool CommandFilterBySFValue::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[FILTER BY VALUE]"));
	
	USE_SPECIAL_SF_VALUE useValForMin = USE_NONE;
	ScalarType minVal = 0;
	QString minValStr;
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: min value after \"-%1\"").arg(COMMAND_FILTER_SF_BY_VALUE));
		}
		
		bool paramOk = false;
		minValStr = cmd.arguments().takeFirst();

		useValForMin = ToSpecialSFValue(minValStr);

		if (useValForMin == USE_N_SIGMA_MIN)
		{
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: N value (after \"-%1 N_SIGMA_MIN\").").arg(COMMAND_FILTER_SF_BY_VALUE));
			}
			minValStr = cmd.arguments().takeFirst();
			minVal = static_cast<ScalarType>(minValStr.toDouble(&paramOk));
			if (!paramOk)
			{
				return cmd.error(QObject::tr("Failed to read a numerical parameter: N value (after \"N_SIGMA_MIN\"). Got '%2' instead.").arg(minValStr));
			}
		}
		else if (useValForMin == USE_N_SIGMA_MAX)
		{
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: N value (after \"-%1 N_SIGMA_MAX\").").arg(COMMAND_FILTER_SF_BY_VALUE));
			}
			minValStr = cmd.arguments().takeFirst();
			minVal = static_cast<ScalarType>(minValStr.toDouble(&paramOk));
			if (!paramOk)
			{
				return cmd.error(QObject::tr("Failed to read a numerical parameter: N value (after \"N_SIGMA_MAX\"). Got '%2' instead.").arg(minValStr));
			}
		}
		else if (useValForMin == USE_NONE)
		{
			minVal = static_cast<ScalarType>(minValStr.toDouble(&paramOk));
			if (!paramOk)
			{
				return cmd.error(QObject::tr("Failed to read a numerical parameter: min value (after \"-%1\"). Got '%2' instead.").arg(COMMAND_FILTER_SF_BY_VALUE, minValStr));
			}
		}
	}
	
	USE_SPECIAL_SF_VALUE useValForMax = USE_NONE;
	ScalarType maxVal = 0;
	QString maxValStr;
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: max value after \"-%1\" {min}").arg(COMMAND_FILTER_SF_BY_VALUE));
		}
		
		bool paramOk = false;
		maxValStr = cmd.arguments().takeFirst();

		useValForMax = ToSpecialSFValue(maxValStr);

		if (useValForMax == USE_N_SIGMA_MIN)
		{
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: N value (after \"-%1 XXX N_SIGMA_MIN\").").arg(COMMAND_FILTER_SF_BY_VALUE));
			}
			maxValStr = cmd.arguments().takeFirst();
			maxVal = static_cast<ScalarType>(maxValStr.toDouble(&paramOk));
			if (!paramOk)
			{
				return cmd.error(QObject::tr("Failed to read a numerical parameter: N value (after \"N_SIGMA_MIN\"). Got '%2' instead.").arg(maxValStr));
			}
		}
		else if (useValForMax == USE_N_SIGMA_MAX)
		{
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: N value (after \"-%1 XXX N_SIGMA_MAX\").").arg(COMMAND_FILTER_SF_BY_VALUE));
			}
			maxValStr = cmd.arguments().takeFirst();
			maxVal = static_cast<ScalarType>(maxValStr.toDouble(&paramOk));
			if (!paramOk)
			{
				return cmd.error(QObject::tr("Failed to read a numerical parameter: N value (after \"N_SIGMA_MAX\"). Got '%2' instead.").arg(maxValStr));
			}
		}
		else if (useValForMax == USE_NONE)
		{
			maxVal = static_cast<ScalarType>(maxValStr.toDouble(&paramOk));
			if (!paramOk)
			{
				return cmd.error(QObject::tr("Failed to read a numerical parameter: max value (after min value). Got '%1' instead.").arg(COMMAND_FILTER_SF_BY_VALUE, maxValStr));
			}
		}
	}
	
	cmd.print(QObject::tr("\tInterval: [%1 - %2]").arg(minValStr, maxValStr));
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No point cloud nor mesh on which to filter SF! (be sure to open one or generate one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_FILTER_SF_BY_VALUE));
	}

	// for each cloud
	for (CLCloudDesc& desc : cmd.clouds())
	{
		CCCoreLib::ScalarField* sf = desc.pc->getCurrentOutScalarField();
		if (sf)
		{
			std::pair<ScalarType, ScalarType> range = GetSFRange(*sf, minVal, useValForMin, maxVal, useValForMax);
			
			ccPointCloud* fitleredCloud = desc.pc->filterPointsByScalarValue(range.first, range.second);
			if (fitleredCloud)
			{
				cmd.print(QObject::tr("\t\tCloud '%1' --> %2/%3 points remaining").arg(desc.pc->getName()).arg(fitleredCloud->size()).arg(desc.pc->size()));
				
				if (fitleredCloud != desc.pc)
				{
					//replace current cloud by this one
					delete desc.pc;
					desc.pc = fitleredCloud;
				}
				desc.basename += QObject::tr("_FILTERED_[%1_%2]").arg(range.first).arg(range.second);

				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc);
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
		}
	}
	
	// for each mesh
	for (CLMeshDesc& desc : cmd.meshes())
	{
		ccGenericMesh* mesh = desc.mesh;
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(mesh);
		if (!pc)
		{
			// strange mesh
			continue;
		}
		
		CCCoreLib::ScalarField* sf = pc->getCurrentOutScalarField();
		if (sf)
		{
			std::pair<ScalarType, ScalarType> range = GetSFRange(*sf, minVal, useValForMin, maxVal, useValForMax);

			pc->hidePointsByScalarValue(range.first, range.second);
			ccGenericMesh* filteredMesh = nullptr;
			if (mesh->isA(CC_TYPES::MESH)/*|| ent->isKindOf(CC_TYPES::PRIMITIVE)*/) //TODO
				filteredMesh = ccHObjectCaster::ToMesh(mesh)->createNewMeshFromSelection(false);
			else if (mesh->isA(CC_TYPES::SUB_MESH))
				filteredMesh = ccHObjectCaster::ToSubMesh(mesh)->createNewSubMeshFromSelection(false);
			else
			{
				cmd.warning("Unhandled mesh type for entitiy " + mesh->getName());
				continue;
			}

			if (filteredMesh)
			{
				cmd.print(QObject::tr("\t\tMesh '%1' --> %2/%3 triangles remaining").arg(mesh->getName()).arg(filteredMesh->size()).arg(mesh->size()));

				//replace current mesh by this one
				if (filteredMesh != mesh) //it's technically possible to have the same pointer if all triangles were filtered (with 'createNewMeshFromSelection')
				{
					delete mesh;
					mesh = nullptr;
					desc.mesh = filteredMesh;
				}
				desc.basename += QObject::tr("_FILTERED_[%1_%2]").arg(range.first).arg(range.second);

				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc);
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
		}
	}

	return true;
}

CommandComputeMeshVolume::CommandComputeMeshVolume()
	: ccCommandLineInterface::Command(QObject::tr("Compute mesh volume"), COMMAND_MESH_VOLUME)
{}

bool CommandComputeMeshVolume::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[COMPUTE MESH VOLUME]"));
	
	if (cmd.meshes().empty())
	{
		cmd.warning(QObject::tr("No mesh loaded! Nothing to do..."));
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
				cmd.print(QObject::tr("Volume report file: %1").arg(outputFilename));
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
			return cmd.error(QObject::tr("Failed to create/open volume report file"));
		}
	}
	
	//for each mesh
	for (CLMeshDesc& desc : cmd.meshes())
	{
		//we compute the mesh volume
		double V = CCCoreLib::MeshSamplingTools::computeMeshVolume(desc.mesh);
		
		QString titleStr = QObject::tr("Mesh '%1'").arg(desc.basename);
		if (desc.indexInFile >= 0)
		{
			titleStr += QObject::tr(" (#%2)").arg(desc.indexInFile);
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
	: ccCommandLineInterface::Command(QObject::tr("Merge meshes"), COMMAND_MERGE_MESHES)
{}

bool CommandMergeMeshes::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[MERGE MESHES]"));
	
	if (cmd.meshes().size() < 2)
	{
		cmd.warning(QObject::tr("Less than 2 meshes are loaded! Nothing to do..."));
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
	for (CLMeshDesc& desc : cmd.meshes())
	{
		//get the mesh
		ccMesh* mesh = dynamic_cast<ccMesh*>(desc.mesh);
		if (!mesh)
		{
			ccLog::Error(QObject::tr("Can't merge mesh '%1' (unhandled type)").arg(desc.basename));
		}
		
		if (mergedMesh->merge(mesh, true)) //merge it
		{
			if (firstValidMesh)
			{
				//copy the first valid mesh description
				mergedMeshDesc = desc;
				mergedMeshDesc.mesh = nullptr;
				firstValidMesh = false;
			}
		}
		else
		{
			return cmd.error(QObject::tr("Merge operation failed"));
		}
		
		delete desc.mesh;
		desc.mesh = nullptr;
	}
	
	if (mergedMesh->size() == 0)
	{
		return cmd.error(QObject::tr("Result is empty"));
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
		{
			return cmd.error(errorStr);
		}
	}
	
	return true;
}

CommandMergeClouds::CommandMergeClouds()
	: ccCommandLineInterface::Command(QObject::tr("Merge clouds"), COMMAND_MERGE_CLOUDS)
{}

bool CommandMergeClouds::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[MERGE CLOUDS]"));
	
	if (cmd.clouds().size() < 2)
	{
		cmd.warning(QObject::tr("Less than 2 clouds are loaded! Nothing to do..."));
		return true;
	}
	
	//merge clouds
	if (!cmd.clouds().empty())
	{
		for (size_t i = 1; i < cmd.clouds().size(); ++i)
		{
			unsigned beforePts = cmd.clouds().front().pc->size();

			CLCloudDesc& desc = cmd.clouds()[i];
			unsigned newPts = desc.pc->size();
			*cmd.clouds().front().pc += desc.pc;
			
			//success?
			if (cmd.clouds().front().pc->size() == beforePts + newPts)
			{
				delete desc.pc;
				desc.pc = nullptr;
			}
			else
			{
				return cmd.error(QObject::tr("Fusion failed! (not enough memory?)"));
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
		{
			return cmd.error(errorStr);
		}
	}
	return true;
}

CommandSetActiveSF::CommandSetActiveSF()
	: ccCommandLineInterface::Command(QObject::tr("Set active SF"), COMMAND_SET_ACTIVE_SF)
{}

bool CommandSetActiveSF::process(ccCommandLineInterface& cmd)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: scalar field index after \"-%1\"").arg(COMMAND_SET_ACTIVE_SF));
	}
	
	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);
	
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No point cloud nor mesh loaded! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_SET_ACTIVE_SF));
	}
	
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			int thisSFIndex = GetScalarFieldIndex(desc.pc, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				desc.pc->setCurrentScalarField(thisSFIndex);
			}
		}
	}
	
	for (CLMeshDesc& desc : cmd.meshes())
	{
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(desc.mesh);
		if (pc)
		{
			int thisSFIndex = GetScalarFieldIndex(pc, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				pc->setCurrentScalarField(thisSFIndex);
			}
		}
	}

	return true;
}

CommandRemoveAllSFs::CommandRemoveAllSFs()
	: ccCommandLineInterface::Command(QObject::tr("Remove all SF"), COMMAND_REMOVE_ALL_SFS)
{}

bool CommandRemoveAllSFs::process(ccCommandLineInterface& cmd)
{
	//no argument required
	for (auto& desc : cmd.clouds())
	{
		if (desc.pc/* && desc.pc->hasScalarFields()*/)
		{
			desc.pc->deleteAllScalarFields();
			desc.pc->showSF(false);
		}
	}
	
	for (auto& desc : cmd.meshes())
	{
		if (desc.mesh)
		{
			ccGenericPointCloud* cloud = desc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(cloud)->deleteAllScalarFields();
				cloud->showSF(false);
			}
		}
	}
	
	return true;
}

CommandRemoveSF::CommandRemoveSF()
	: ccCommandLineInterface::Command(QObject::tr("Remove a specific SF"), COMMAND_REMOVE_SF)
{}

bool CommandRemoveSF::removeSF(int sfIndex, ccPointCloud& pc)
{
	if (sfIndex < static_cast<int>(pc.getNumberOfScalarFields()))
	{
		pc.deleteScalarField(sfIndex);
		if (pc.getNumberOfScalarFields() == 0)
		{
			pc.showSF(false);
		}
		else if (pc.getCurrentDisplayedScalarFieldIndex() < 0)
		{
			pc.setCurrentDisplayedScalarField(static_cast<int>(pc.getNumberOfScalarFields()) - 1);
		}

		return true;
	}
	else
	{
		return false;
	}
}

bool CommandRemoveSF::process(ccCommandLineInterface& cmd)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: SF index after %1").arg(COMMAND_REMOVE_SF));
	}

	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

	for (auto& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			int thisSFIndex = GetScalarFieldIndex(desc.pc, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				removeSF(thisSFIndex, *desc.pc);
			}
		}
	}

	for (auto& desc : cmd.meshes())
	{
		if (desc.mesh)
		{
			ccGenericPointCloud* cloud = desc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);

				int thisSFIndex = GetScalarFieldIndex(pc, sfIndex, sfName);
				if (thisSFIndex >= 0)
				{
					removeSF(thisSFIndex, *pc);
					if (pc->getNumberOfScalarFields() == 0)
					{
						desc.mesh->showSF(false);
					}
				}
			}
		}
	}

	return true;
}

CommandRemoveRGB::CommandRemoveRGB()
	: ccCommandLineInterface::Command(QObject::tr("Remove RGB"), COMMAND_REMOVE_RGB)
{}

bool CommandRemoveRGB::process(ccCommandLineInterface& cmd)
{
	//no argument required
	for (auto& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			desc.pc->unallocateColors();
			desc.pc->showColors(false);
		}
	}

	for (auto& desc : cmd.meshes())
	{
		if (desc.mesh)
		{
			ccGenericPointCloud* cloud = desc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(cloud)->unallocateColors();
				cloud->showColors(false);
			}
			desc.mesh->showColors(false);
		}
	}

	return true;
}

CommandRemoveNormals::CommandRemoveNormals()
	: ccCommandLineInterface::Command(QObject::tr("Remove normals"), COMMAND_REMOVE_NORMALS)
{}

bool CommandRemoveNormals::process(ccCommandLineInterface& cmd)
{
	//no argument required
	for (auto& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			desc.pc->unallocateNorms();
			desc.pc->showNormals(false);
		}
	}

	for (auto& desc : cmd.meshes())
	{
		if (desc.mesh)
		{
			ccGenericPointCloud* cloud = desc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(cloud)->unallocateNorms();
				cloud->showNormals(false);
			}
			if (desc.mesh->isA(CC_TYPES::MESH))
			{
				static_cast<ccMesh*>(desc.mesh)->clearTriNormals();
				desc.mesh->showNormals(false);
			}
		}
	}

	return true;
}

CommandRemoveScanGrids::CommandRemoveScanGrids()
	: ccCommandLineInterface::Command(QObject::tr("Remove scan grids"), COMMAND_REMOVE_SCAN_GRIDS)
{}

bool CommandRemoveScanGrids::process(ccCommandLineInterface& cmd)
{
	//no argument required
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			desc.pc->removeGrids();
		}
	}
	
	for (CLMeshDesc& desc : cmd.meshes())
	{
		if (desc.mesh)
		{
			ccGenericPointCloud* cloud = desc.mesh->getAssociatedCloud();
			if (cloud->isA(CC_TYPES::POINT_CLOUD))
			{
				static_cast<ccPointCloud*>(cloud)->removeGrids();
			}
		}
	}
	
	return true;
}

CommandMatchBBCenters::CommandMatchBBCenters()
	: ccCommandLineInterface::Command(QObject::tr("Match B.B. centers"), COMMAND_MATCH_BB_CENTERS)
{}

bool CommandMatchBBCenters::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[MATCH B.B. CENTERS]"));
	
	std::vector<CLEntityDesc*> entities;
	for (auto& cloud : cmd.clouds())
	{
		entities.push_back(&cloud);
	}
	for (auto& mesh : cmd.meshes())
	{
		entities.push_back(&mesh);
	}
	
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
			{
				return cmd.error(errorStr);
			}
		}
	}
	
	return true;
}

CommandMatchBestFitPlane::CommandMatchBestFitPlane()
	: ccCommandLineInterface::Command(QObject::tr("Match best fit plane"), COMMAND_BEST_FIT_PLANE)
{}

bool CommandMatchBestFitPlane::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[COMPUTE BEST FIT PLANE]"));
	
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
	{
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	}
	
	for (CLCloudDesc& desc : cmd.clouds())
	{
		//try to fit plane
		double rms = 0.0;
		ccPlane* pPlane = ccPlane::Fit(desc.pc, &rms);
		if (pPlane)
		{
			cmd.print(QObject::tr("Plane successfully fitted: rms = %1").arg(rms));
			
			CCVector3 N = pPlane->getNormal();
			CCVector3 C = *CCCoreLib::Neighbourhood(desc.pc).getGravityCenter();
			
			CLMeshDesc planeDesc;
			planeDesc.mesh = pPlane;
			planeDesc.basename = desc.basename;
			planeDesc.path = desc.path;
			
			//save plane as a BIN file
			QString outputFilename;
			QString errorStr = cmd.exportEntity(planeDesc, "BEST_FIT_PLANE", &outputFilename);
			if (!errorStr.isEmpty())
			{
				cmd.warning(errorStr);
			}
			
			//open text file to save plane related information
			QString txtFilename = QObject::tr("%1/%2_BEST_FIT_PLANE_INFO").arg(desc.path, desc.basename);
			if (cmd.addTimestamp())
			{
				txtFilename += QObject::tr("_%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_hh'h'mm"));
			}
			txtFilename += QObject::tr(".txt");
			QFile txtFile(txtFilename);
			txtFile.open(QIODevice::WriteOnly | QIODevice::Text);
			QTextStream txtStream(&txtFile);
			
			txtStream << QObject::tr("Filename: %1").arg(outputFilename) << endl;
			txtStream << QObject::tr("Fitting RMS: %1").arg(rms) << endl;
			
			//We always consider the normal with a positive 'Z' by default!
			if (N.z < 0.0)
			{
				N *= -1.0;
			}
			
			int precision = cmd.numericalPrecision();
			txtStream << QObject::tr("Normal: (%1,%2,%3)").arg(N.x, 0, 'f', precision).arg(N.y, 0, 'f', precision).arg(N.z, 0, 'f', precision) << endl;
			
			//we compute strike & dip by the way
			{
				PointCoordinateType dip = 0;
				PointCoordinateType dipDir = 0;
				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
				txtStream << ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir) << endl;
			}
			
			//compute the transformation matrix that would make this normal points towards +Z
			ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, CCCoreLib::PC_ONE));
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
				desc.pc->applyGLTransformation_recursive(&makeZPosMatrix);
				cmd.print(QObject::tr("Cloud '%1' has been transformed with the above matrix").arg(desc.pc->getName()));
				desc.basename += QObject::tr("_HORIZ");
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc);
					if (!errorStr.isEmpty())
					{
						cmd.warning(errorStr);
					}
				}
			}
		}
		else
		{
			cmd.warning(QObject::tr("Failed to compute best fit plane for cloud '%1'").arg(desc.pc->getName()));
		}
	}
	
	return true;
}

CommandOrientNormalsMST::CommandOrientNormalsMST()
	: ccCommandLineInterface::Command(QObject::tr("Orient normals"), COMMAND_ORIENT_NORMALS)
{}

bool CommandOrientNormalsMST::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[ORIENT NORMALS (MST)]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: number of neighbors after \"-%1\"").arg(COMMAND_ORIENT_NORMALS));
	}
	
	QString knnStr = cmd.arguments().takeFirst();
	bool ok;
	int knn = knnStr.toInt(&ok);
	if (!ok || knn <= 0)
	{
		return cmd.error(QObject::tr("Invalid parameter: number of neighbors (%1)").arg(knnStr));
	}
	
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	}
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (CLCloudDesc& desc : cmd.clouds())
	{
		assert(desc.pc);
		
		if (!desc.pc->hasNormals())
		{
			continue;
		}
		
		//computation
		if (desc.pc->orientNormalsWithMST(knn, progressDialog.data()))
		{
			desc.basename += QObject::tr("_NORMS_REORIENTED");
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(desc);
				if (!errorStr.isEmpty())
				{
					cmd.warning(errorStr);
				}
			}
		}
		else
		{
			return cmd.error(QObject::tr("Failed to orient the normals of cloud '%1'!").arg(desc.pc->getName()));
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
	: ccCommandLineInterface::Command(QObject::tr("S.O.R. filter"), COMMAND_SOR_FILTER)
{}

bool CommandSORFilter::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SOR FILTER]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: number of neighbors mode after \"-%1\"").arg(COMMAND_SOR_FILTER));
	}
	
	QString knnStr = cmd.arguments().takeFirst();
	bool ok;
	int knn = knnStr.toInt(&ok);
	if (!ok || knn <= 0)
	{
		return cmd.error(QObject::tr("Invalid parameter: number of neighbors (%1)").arg(knnStr));
	}
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: sigma multiplier after number of neighbors (SOR)"));
	}
	QString sigmaStr = cmd.arguments().takeFirst();
	double nSigma = sigmaStr.toDouble(&ok);
	if (!ok || nSigma < 0)
	{
		return cmd.error(QObject::tr("Invalid parameter: sigma multiplier (%1)").arg(nSigma));
	}
	
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	}
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (CLCloudDesc& desc : cmd.clouds())
	{
		assert(desc.pc);
		
		//computation
		CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::sorFilter(desc.pc,
																						knn,
																						nSigma,
																						nullptr,
																						progressDialog.data());
		
		if (selection)
		{
			ccPointCloud* cleanCloud = desc.pc->partialClone(selection);
			if (cleanCloud)
			{
				cleanCloud->setName(desc.pc->getName() + QObject::tr(".clean"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc newDesc(cleanCloud, desc.basename, desc.path, desc.indexInFile);
					QString errorStr = cmd.exportEntity(newDesc, "SOR");
					if (!errorStr.isEmpty())
					{
						delete cleanCloud;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete desc.pc;
				desc.pc = cleanCloud;
				desc.basename += QObject::tr("_SOR");
				//delete cleanCloud;
				//cleanCloud = 0;
			}
			else
			{
				return cmd.error(QObject::tr("Not enough memory to create a clean version of cloud '%1'!").arg(desc.pc->getName()));
			}
			
			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			return cmd.error(QObject::tr("Failed to apply SOR filter on cloud '%1'! (empty output or not enough memory?)").arg(desc.pc->getName()));
		}
	}
	
	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}
	
	return true;
}

CommandNoiseFilter::CommandNoiseFilter()
	: ccCommandLineInterface::Command(QObject::tr("Noise filter"), COMMAND_NOISE_FILTER)
{}

bool CommandNoiseFilter::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[NOISE FILTER]"));

	if (cmd.arguments().size() < 4)
	{
		return cmd.error(QObject::tr("Missing parameters: 'KNN/RADIUS {value} REL/ABS {value}' expected after \"-%1\"").arg(COMMAND_NOISE_FILTER));
	}

	QString firstOption = cmd.arguments().takeFirst().toUpper();
	int knn = -1;
	double radius = std::numeric_limits<double>::quiet_NaN();

	if (firstOption == COMMAND_NOISE_FILTER_KNN)
	{
		QString knnStr = cmd.arguments().takeFirst();
		bool ok;
		knn = knnStr.toInt(&ok);
		if (!ok || knn <= 0)
		{
			return cmd.error(QObject::tr("Invalid parameter: number of neighbors after KNN (got '%1' instead)").arg(knnStr));
		}
	}
	else if(firstOption == COMMAND_NOISE_FILTER_RADIUS)
	{
		QString radiusStr = cmd.arguments().takeFirst();
		bool ok;
		radius = radiusStr.toDouble(&ok);
		if (!ok || radius <= 0)
		{
			return cmd.error(QObject::tr("Invalid parameter: radius after RADIUS (got '%1' instead)").arg(radiusStr));
		}
	}
	else
	{
		return cmd.error(QObject::tr("Invalid parameter: KNN or RADIUS expected after \"-%1\"").arg(COMMAND_NOISE_FILTER));
	}

	QString secondOption = cmd.arguments().takeFirst().toUpper();
	bool absoluteError = true;
	if (secondOption == COMMAND_NOISE_FILTER_REL)
	{
		absoluteError = false;
	}
	else if (secondOption == COMMAND_NOISE_FILTER_ABS)
	{
		absoluteError = true;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid parameter: REL or ABS expected"));
	}

	double error = std::numeric_limits<double>::quiet_NaN();
	{
		QString errorStr = cmd.arguments().takeFirst();
		bool ok;
		error = errorStr.toDouble(&ok);
		if (!ok || error <= 0)
		{
			return cmd.error(QObject::tr("Invalid parameter: relative or absolute error expected after KNN (got '%1' instead)").arg(errorStr));
		}
	}

	//optional arguments
	bool removeIsolatedPoints = false;
	if (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (argument == COMMAND_NOISE_FILTER_RIP)
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			removeIsolatedPoints = true;
		}
	}

	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}

	for (CLCloudDesc& desc : cmd.clouds())
	{
		assert(desc.pc);

		//computation
		CCCoreLib::ReferenceCloud* selection = CCCoreLib::CloudSamplingTools::noiseFilter(desc.pc,
			static_cast<PointCoordinateType>(radius),
			error,
			removeIsolatedPoints,
			knn > 0,
			knn,
			absoluteError,
			error,
			nullptr,
			progressDialog.data());

		if (selection)
		{
			ccPointCloud* cleanCloud = desc.pc->partialClone(selection);
			if (cleanCloud)
			{
				cleanCloud->setName(desc.pc->getName() + QObject::tr(".clean"));
				if (cmd.autoSaveMode())
				{
					CLCloudDesc newDesc(cleanCloud, desc.basename, desc.path, desc.indexInFile);
					QString errorStr = cmd.exportEntity(newDesc, "DENOISED");
					if (!errorStr.isEmpty())
					{
						delete cleanCloud;
						return cmd.error(errorStr);
					}
				}
				//replace current cloud by this one
				delete desc.pc;
				desc.pc = cleanCloud;
				desc.basename += QObject::tr("_DENOISED");
				//delete cleanCloud;
				//cleanCloud = 0;
			}
			else
			{
				return cmd.error(QObject::tr("Not enough memory to create a clean version of cloud '%1'!").arg(desc.pc->getName()));
			}

			delete selection;
			selection = nullptr;
		}
		else
		{
			//no points fall inside selection!
			return cmd.error(QObject::tr("Failed to apply Noise filter on cloud '%1'! (empty output or not enough memory?)").arg(desc.pc->getName()));
		}
	}

	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}

	return true;
}

CommandRemoveDuplicatePoints::CommandRemoveDuplicatePoints()
        : ccCommandLineInterface::Command(QObject::tr("Remove duplicate points"), COMMAND_REMOVE_DUPLICATE_POINTS)
{}

bool CommandRemoveDuplicatePoints::process(ccCommandLineInterface& cmd)
{
    cmd.print(QObject::tr("[REMOVE DUPLICATE POINTS]"));

    double minDistanceBetweenPoints = std::numeric_limits<double>::epsilon();

    //get optional argument
    if (!cmd.arguments().empty())
	{
        bool paramOk = false;
        double arg = cmd.arguments().front().toDouble(&paramOk);
        if (paramOk)
		{
            if (arg < minDistanceBetweenPoints)
			{
                return cmd.error(QObject::tr("Invalid argument: '%1'").arg(arg));
            }

            minDistanceBetweenPoints = arg;
            cmd.arguments().pop_front();
        }
    }

    cmd.print(QObject::tr("Minimum distance between points: '%1'").arg(minDistanceBetweenPoints));

    QScopedPointer<ccProgressDialog> progressDialog(nullptr);
    if (!cmd.silentMode())
    {
        progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
        progressDialog->setAutoClose(false);
    }

    for (CLCloudDesc& desc : cmd.clouds())
    {
        assert(desc.pc);
		ccPointCloud* filteredCloud = desc.pc->removeDuplicatePoints(minDistanceBetweenPoints, progressDialog.data());
		if (!filteredCloud)
		{
			return cmd.error(QObject::tr("Process failed (see log)"));
		}

		//replace current cloud by filtered one
		delete desc.pc;
		desc.pc = filteredCloud;
		desc.basename += QObject::tr("_REMOVED_DUPLICATE_POINTS");
    }

    if (progressDialog)
    {
        progressDialog->close();
        QCoreApplication::processEvents();
    }

    return true;
}

CommandExtractVertices::CommandExtractVertices()
	: ccCommandLineInterface::Command(QObject::tr("Extract vertices (as a standalone 'cloud')"), COMMAND_EXTRACT_VERTICES)
{}

bool CommandExtractVertices::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[EXTRACT VERTICES]"));
	
	if (cmd.meshes().empty())
	{
		cmd.warning(QObject::tr("No mesh available. Be sure to open one first!"));
		return false;
	}
	
	for (CLMeshDesc& desc : cmd.meshes())
	{
		ccGenericMesh* mesh = desc.mesh;
		ccGenericPointCloud* cloud = mesh->getAssociatedCloud();
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(cloud);
		if (!pc)
		{
			assert(false);
			continue;
		}
		
		//add the resulting cloud to the main set
		cmd.clouds().emplace_back(pc, desc.basename + QObject::tr(".vertices"), desc.path);
		
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


CommandFlipTriangles::CommandFlipTriangles()
	: ccCommandLineInterface::Command(QObject::tr("Flip the vertices order of all opened mesh triangles"), COMMAND_FLIP_TRIANGLES)
{}

bool CommandFlipTriangles::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[FLIP TRIANGLES]"));

	if (cmd.meshes().empty())
	{
		cmd.warning(QObject::tr("No mesh available. Be sure to open one first!"));
		return false;
	}

	for (CLMeshDesc& desc : cmd.meshes())
	{
		ccGenericMesh* mesh = desc.mesh;

		ccMesh* ccMesh = ccHObjectCaster::ToMesh(mesh);
		if (ccMesh)
		{
			ccMesh->flipTriangles();
		}

		desc.basename += QObject::tr("_FLIPPED_TRIANGLES");

		//save it as well
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(desc);
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
	: ccCommandLineInterface::Command(QObject::tr("Sample mesh"), COMMAND_SAMPLE_MESH)
{}

bool CommandSampleMesh::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SAMPLE POINTS ON MESH]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: sampling mode after \"-%1\" (POINTS/DENSITY)").arg(COMMAND_SAMPLE_MESH));
	}
	
	bool useDensity = false;
	double parameter = 0;
	
	QString sampleMode = cmd.arguments().takeFirst().toUpper();
	if (sampleMode == "POINTS")
	{
		useDensity = false;
	}
	else if (sampleMode == "DENSITY")
	{
		useDensity = true;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid parameter: unknown sampling mode \"%1\"").arg(sampleMode));
	}
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: value after sampling mode"));
	}
	bool conversionOk = false;
	parameter = cmd.arguments().takeFirst().toDouble(&conversionOk);
	if (!conversionOk)
	{
		return cmd.error(QObject::tr("Invalid parameter: value after sampling mode"));
	}
	
	if (cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No mesh available. Be sure to open one first!"));
	}
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (CLMeshDesc& desc : cmd.meshes())
	{
		ccPointCloud* cloud = desc.mesh->samplePoints(useDensity, parameter, true, true, true, progressDialog.data());
		
		if (!cloud)
		{
			return cmd.error(QObject::tr("Cloud sampling failed!"));
		}
		
		//add the resulting cloud to the main set
		cmd.print(QObject::tr("Sampled cloud created: %1 points").arg(cloud->size()));
		cmd.clouds().emplace_back(cloud, desc.basename + QObject::tr("_SAMPLED_POINTS"), desc.path);
		
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
	
	if (progressDialog)
	{
		progressDialog->close();
		QCoreApplication::processEvents();
	}
	
	return true;
}

CommandCompressFWF::CommandCompressFWF()
	: ccCommandLineInterface::Command(QObject::tr("Sample mesh"), COMMAND_COMPRESS_FWF)
{}

bool CommandCompressFWF::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[COMPRESS FWF]"));

	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
	}

	for (CLCloudDesc& desc : cmd.clouds())
	{
		desc.pc->compressFWFData();
	}

	return true;
}

CommandCrop::CommandCrop()
	: ccCommandLineInterface::Command(QObject::tr("Crop"), COMMAND_CROP)
{}

bool CommandCrop::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CROP]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: box extents after \"-%1\" (Xmin:Ymin:Zmin:Xmax:Ymax:Zmax)").arg(COMMAND_CROP));
	}
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No point cloud or mesh available. Be sure to open or generate one first!"));
	}
	
	//decode box extents
	CCVector3 boxMin;
	CCVector3 boxMax;
	{
		QString boxBlock = cmd.arguments().takeFirst();
		QStringList tokens = boxBlock.split(':');
		if (tokens.size() != 6)
		{
			return cmd.error(QObject::tr("Invalid parameter: box extents (expected format is 'Xmin:Ymin:Zmin:Xmax:Ymax:Zmax')").arg(COMMAND_CROP));
		}
		
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
	
	ccBBox cropBox(boxMin, boxMax, true);
	//crop clouds
	{
		for (CLCloudDesc& desc : cmd.clouds())
		{
			ccHObject* croppedCloud = ccCropTool::Crop(desc.pc, cropBox, inside);
			if (croppedCloud)
			{
				delete desc.pc;
				assert(croppedCloud->isA(CC_TYPES::POINT_CLOUD));
				desc.pc = static_cast<ccPointCloud*>(croppedCloud);
				desc.basename += "_CROPPED";
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc);
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
			else
			{
				//otherwise an error message has already been issued
				delete desc.pc;
				desc.pc = nullptr; //will be removed after this loop
				//desc.basename += "_FULLY_CROPPED";
			}
		}

		//now clean the set of clouds in case some have been 'cropped out'
		for (auto it = cmd.clouds().begin(); it != cmd.clouds().end(); )
		{
			if (it->pc == nullptr)
			{
				it = cmd.clouds().erase(it);
			}
			else
			{
				++it;
			}
		}
	}
	
	//crop meshes
	{
		for (CLMeshDesc& desc : cmd.meshes())
		{
			ccHObject* croppedMesh = ccCropTool::Crop(desc.mesh, cropBox, inside);
			if (croppedMesh)
			{
				delete desc.mesh;
				assert(croppedMesh->isA(CC_TYPES::MESH));
				desc.mesh = static_cast<ccMesh*>(croppedMesh);
				desc.basename += "_CROPPED";
				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc);
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
			else
			{
				//otherwise an error message has already been issued
				delete desc.mesh;
				desc.mesh = nullptr; //will be removed after this loop
				//desc.basename += "_FULLY_CROPPED";
			}
		}

		//now clean the set of meshes in case some have been 'cropped out'
		for (auto it = cmd.meshes().begin(); it != cmd.meshes().end(); )
		{
			if (it->mesh == nullptr)
			{
				it = cmd.meshes().erase(it);
			}
			else
			{
				++it;
			}
		}
	}
	
	return true;
}


CommandSFToCoord::CommandSFToCoord()
	: ccCommandLineInterface::Command(QObject::tr("SF to Coord"), COMMAND_SF_TO_COORD)
{}

bool CommandSFToCoord::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SF TO COORD]"));

	if (cmd.arguments().size() < 2)
	{
		return cmd.error(QObject::tr("Missing parameter(s) after \"-%1\" (SF INDEX OR NAME) (DIMENSION)").arg(COMMAND_SF_TO_COORD));
	}
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
	}

	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

	//dimension
	QString dimStr = cmd.arguments().takeFirst().toUpper();
	bool exportDims[3] { dimStr == "X", dimStr == "Y", dimStr == "Z" };
	if (!exportDims[0] && !exportDims[1] && !exportDims[2])
	{
		return cmd.error(QObject::tr("Invalid parameter: dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_SF_TO_COORD));
	}

	//now we can export the corresponding coordinate
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			CCCoreLib::ScalarField* sf = GetScalarField(desc.pc, sfIndex, sfName);
			if (sf)
			{
				if (desc.pc->setCoordFromSF(exportDims, sf, std::numeric_limits<PointCoordinateType>::quiet_NaN()))
				{
					desc.basename += QObject::tr("_SF_TO_COORD_%1").arg(dimStr);
					if (cmd.autoSaveMode())
					{
						QString errorStr = cmd.exportEntity(desc);
						if (!errorStr.isEmpty())
						{
							return cmd.error(errorStr);
						}
					}
				}
			}
		}
		else
		{
			return cmd.error(QObject::tr("Failed to set SF %1 as coord %2 on cloud '%3'!").arg(sfName).arg(dimStr).arg(desc.pc->getName()));
		}
	}

	return true;
}

CommandCoordToSF::CommandCoordToSF()
	: ccCommandLineInterface::Command(QObject::tr("Coord to SF"), COMMAND_COORD_TO_SF)
{}

bool CommandCoordToSF::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[COORD TO SF]"));
	
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter after \"-%1\" (DIMENSION)").arg(COMMAND_COORD_TO_SF));
	}
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
	}
	
	//dimension
	QString dimStr = cmd.arguments().takeFirst().toUpper();
	bool exportDims[3]{ dimStr == "X", dimStr == "Y", dimStr == "Z" };
	if (!exportDims[0] && !exportDims[1] && !exportDims[2])
	{
		return cmd.error(QObject::tr("Invalid parameter: dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_COORD_TO_SF));
	}

	//now we can export the corresponding coordinate
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc->exportCoordToSF(exportDims))
		{
			desc.basename += QObject::tr("_%1_TO_SF").arg(dimStr);
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(desc);
				if (!errorStr.isEmpty())
				{
					return cmd.error(errorStr);
				}
			}
		}
		else
		{
			return cmd.error(QObject::tr("Failed to export coord. %1 to SF on cloud '%2'!").arg(dimStr, desc.pc->getName()));
		}
	}
	
	return true;
}

CommandCrop2D::CommandCrop2D()
	: ccCommandLineInterface::Command(QObject::tr("Crop 2D"), COMMAND_CROP_2D)
{}

bool CommandCrop2D::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[CROP 2D]"));
	
	if (cmd.arguments().size() < 6)
	{
		return cmd.error(QObject::tr("Missing parameter(s) after \"-%1\" (ORTHO_DIM N X1 Y1 X2 Y2 ... XN YN)").arg(COMMAND_CROP_2D));
	}
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud available. Be sure to open or generate one first!"));
	}
	
	//decode poyline extents
	ccPointCloud vertices("polyline.vertices");
	ccPolyline poly(&vertices);
	
	//orthogonal dimension
	unsigned char orthoDim = 2;
	{
		QString orthoDimStr = cmd.arguments().takeFirst().toUpper();
		if (orthoDimStr == "X")
		{
			orthoDim = 0;
		}
		else if (orthoDimStr == "Y")
		{
			orthoDim = 1;
		}
		else if (orthoDimStr == "Z")
		{
			orthoDim = 2;
		}
		else
		{
			return cmd.error(QObject::tr("Invalid parameter: orthogonal dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_CROP_2D));
		}
	}
	
	//number of vertices
	bool ok = true;
	unsigned N = 0;
	{
		QString countStr = cmd.arguments().takeFirst();
		N = countStr.toUInt(&ok);
		if (!ok)
		{
			return cmd.error(QObject::tr("Invalid parameter: number of vertices for the 2D polyline after \"-%1\"").arg(COMMAND_CROP_2D));
		}
	}
	
	//now read the vertices
	{
		if (!vertices.reserve(N) || !poly.addPointIndex(0, N))
		{
			return cmd.error(QObject::tr("Not enough memory!"));
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
			{
				return cmd.error(QObject::tr("Invalid parameter: X-coordinate of vertex #%1").arg(i + 1));
			}
			/*QString */coordStr = cmd.arguments().takeFirst();
			P.y = static_cast<PointCoordinateType>(coordStr.toDouble(&ok));
			if (!ok)
			{
				return cmd.error(QObject::tr("Invalid parameter: Y-coordinate of vertex #%1").arg(i + 1));
			}
			
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
	for (CLCloudDesc& desc : cmd.clouds())
	{
		CCCoreLib::ReferenceCloud* ref = desc.pc->crop2D(&poly, orthoDim, inside);
		if (ref)
		{
			if (ref->size() != 0)
			{
				ccPointCloud* croppedCloud = desc.pc->partialClone(ref);
				delete ref;
				ref = nullptr;
				
				if (croppedCloud)
				{
					delete desc.pc;
					desc.pc = croppedCloud;
					croppedCloud->setName(desc.pc->getName() + QObject::tr(".cropped"));
					desc.basename += "_CROPPED";
					if (cmd.autoSaveMode())
					{
						QString errorStr = cmd.exportEntity(desc);
						if (!errorStr.isEmpty())
						{
							return cmd.error(errorStr);
						}
					}
				}
				else
				{
					return cmd.error(QObject::tr("Not enough memory to crop cloud '%1'!").arg(desc.pc->getName()));
				}
			}
			else
			{
				delete ref;
				ref = nullptr;
				cmd.warning(QObject::tr("No point of cloud '%1' falls inside the input box!").arg(desc.pc->getName()));
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
	: ccCommandLineInterface::Command(QObject::tr("Color banding"), COMMAND_COLOR_BANDING)
{}

bool CommandColorBanding::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[COLOR BANDING]"));
	
	if (cmd.arguments().size() < 2)
	{
		return cmd.error(QObject::tr("Missing parameter(s) after \"-%1\" (DIM FREQUENCY)").arg(COMMAND_COLOR_BANDING));
	}
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No entity available. Be sure to open or generate one first!"));
	}
	
	//dimension
	unsigned char dim = 2;
	QString dimStr = "Z";
	{
		dimStr = cmd.arguments().takeFirst().toUpper();
		if (dimStr == "X")
		{
			dim = 0;
		}
		else if (dimStr == "Y")
		{
			dim = 1;
		}
		else if (dimStr == "Z")
		{
			dim = 2;
		}
		else
		{
			return cmd.error(QObject::tr("Invalid parameter: dimension after \"-%1\" (expected: X, Y or Z)").arg(COMMAND_COLOR_BANDING));
		}
	}
	
	//frequency
	bool ok = true;
	double freq = 0;
	{
		QString countStr = cmd.arguments().takeFirst();
		freq = countStr.toDouble(&ok);
		if (!ok)
		{
			return cmd.error(QObject::tr("Invalid parameter: frequency after \"-%1 DIM\" (in Hz, integer value)").arg(COMMAND_COLOR_BANDING));
		}
	}
	
	//process clouds
	if (!cmd.clouds().empty())
	{
		bool hasclouds = false;
		for (CLCloudDesc& desc : cmd.clouds())
		{
			if (desc.pc)
			{
				if (!desc.pc->setRGBColorByBanding(dim, freq))
				{
					return cmd.error(QObject::tr("Not enough memory"));
				}
				else
				{
					hasclouds = true;
				}
			}
		}
		
		//save output
		if (hasclouds && cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("COLOR_BANDING_%1_%2").arg(dimStr).arg(freq)))
		{
			return false;
		}
	}
	
	if (!cmd.meshes().empty())
	{
		bool hasMeshes = false;
		for (CLMeshDesc& desc : cmd.meshes())
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(desc.mesh);
			if (cloud)
			{
				if (!cloud->setRGBColorByBanding(dim, freq))
				{
					return cmd.error(QObject::tr("Not enough memory"));
				}
				else
				{
					desc.mesh->showColors(true);
					hasMeshes = true;
				}
			}
			else
			{
				cmd.warning(QObject::tr("Vertices of mesh '%1' are locked (they may be shared by multiple entities for instance). Can't apply the current command on them.").arg(desc.mesh->getName()));
			}
		}
		
		//save output
		if (hasMeshes && cmd.autoSaveMode() && !cmd.saveMeshes(QObject::tr("COLOR_BANDING_%1_%2").arg(dimStr).arg(freq)))
		{
			return false;
		}
	}
	
	return true;
}

CommandColorLevels::CommandColorLevels()
	: ccCommandLineInterface::Command(QObject::tr("Color levels"), COMMAND_COLOR_LEVELS)
{}

bool CommandColorLevels::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[COLOR LEVELS]"));

	if (cmd.arguments().size() < 5)
	{
		return cmd.error(QObject::tr("Missing parameter(s) after \"-%1\" (COLOR-BANDS MIN-INPUT-LEVEL MAX-INPUT-LEVEL MIN-OUTPUT-LEVEL MAX-OUTPUT-LEVEL)").arg(COMMAND_COLOR_LEVELS));
	}
	if (cmd.clouds().empty() && cmd.meshes().empty())
	{
		return cmd.error(QObject::tr("No entity available. Be sure to open or generate one first!"));
	}

	//color bands
	QString band = cmd.arguments().takeFirst().toUpper();
	bool rgb[3] { band.contains('R'), band.contains('G'), band.contains('B') };
	{
		QString testBand = band;
		testBand.remove('R');
		testBand.remove('G');
		testBand.remove('B');
		if (!testBand.isEmpty())
		{
			return cmd.error(QObject::tr("Invalid parameter: bands after \"-%1\" (expected: any combination of R, G or B)").arg(COMMAND_COLOR_LEVELS));
		}
	}

	//min level
	int levels[4] = { 0 };
	for (int i = 0; i < 4; ++i)
	{
		bool ok = true;
		QString levelStr = cmd.arguments().takeFirst();
		levels[i] = levelStr.toInt(&ok);
		if (!ok || levels[i] < 0 || levels[i] > 255)
		{
			return cmd.error(QObject::tr("Invalid parameter: color level after \"-%1 COLOR-BANDS\" (integer value between 0 and 255 expected)").arg(COMMAND_COLOR_LEVELS));
		}
	}

	//process clouds
	if (!cmd.clouds().empty())
	{
		bool hasClouds = false;
		for (CLCloudDesc& desc : cmd.clouds())
		{
			if (desc.pc && desc.pc->hasColors())
			{
				if (!ccColorLevelsDlg::ScaleColorFields(desc.pc, levels[0], levels[1], levels[2], levels[3], rgb))
				{
					cmd.warning(QObject::tr("Failed to scale the color band(s) of cloud '%1'").arg(desc.pc->getName()));
				}
				else
				{
					hasClouds = true;
				}
			}
		}

		//save output
		if (hasClouds && cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("COLOR_LEVELS_%1_%2_%3").arg(band).arg(levels[2]).arg(levels[3])))
		{
			return false;
		}
	}

	if (!cmd.meshes().empty())
	{
		bool hasMeshes = false;
		for (CLMeshDesc& desc : cmd.meshes())
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(desc.mesh);
			if (cloud && cloud->hasColors())
			{
				if (!ccColorLevelsDlg::ScaleColorFields(cloud, levels[0], levels[1], levels[2], levels[3], rgb))
				{
					cmd.warning(QObject::tr("Failed to scale the color band(s) of mesh '%1'").arg(desc.mesh->getName()));
				}
				else
				{
					hasMeshes = true;
				}
			}
			else if (desc.mesh->hasColors())
			{
				cmd.warning(QObject::tr("Vertices of mesh '%1' are locked (they may be shared by multiple entities for instance). Can't apply the current command on them.").arg(desc.mesh->getName()));
			}
		}

		//save output
		if (hasMeshes && cmd.autoSaveMode() && !cmd.saveMeshes(QObject::tr("COLOR_LEVELS_%1_%2_%3").arg(band).arg(levels[2]).arg(levels[3])))
		{
			return false;
		}
	}

	return true;
}

CommandDist::CommandDist(bool cloud2meshDist, const QString& name, const QString& keyword)
	: ccCommandLineInterface::Command(name, keyword)
	, m_cloud2meshDist(cloud2meshDist)
{}

bool CommandDist::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[DISTANCE COMPUTATION]"));
	
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
		{
			cmd.warning(QObject::tr("[C2M] Multiple point clouds loaded! Will take the first one by default."));
		}
		compEntity = &(cmd.clouds().front());
		compCloud = cmd.clouds().front().pc;
	}
	assert(compEntity && compCloud);
	
	//reference entity
	ccHObject* refEntity = nullptr;
	if (m_cloud2meshDist)
	{
		if (cmd.meshes().size() <= nextMeshIndex)
		{
			return cmd.error(QObject::tr("No mesh available. Be sure to open one first!"));
		}
		else if (cmd.meshes().size() != nextMeshIndex + 1)
		{
			cmd.warning(QString("Multiple meshes loaded! We take the %1 one by default").arg(nextMeshIndex == 0 ? "first" : "second"));
		}
		refEntity = cmd.meshes()[nextMeshIndex].mesh;
	}
	else
	{
		if (cmd.clouds().size() < 2)
		{
			return cmd.error(QObject::tr("Only one point cloud available. Be sure to open or generate a second one before performing C2C distance!"));
		}
		else if (cmd.clouds().size() > 2)
		{
			cmd.warning(QObject::tr("More than 3 point clouds loaded! We take the second one as reference by default"));
		}
		refEntity = cmd.clouds()[1].pc;
	}
	
	//inner loop for Distance computation options
	bool flipNormals = false;
	bool unsignedDistances = false;
	double maxDist = 0.0;
	unsigned octreeLevel = 0;
	int maxThreadCount = 0;
	
	bool splitXYZ = false;
    bool mergeXY = false;
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
			{
				cmd.warning(QObject::tr("Parameter \"-%1\" ignored: only for C2M distance!").arg(COMMAND_C2M_DIST_FLIP_NORMALS));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2M_DIST_UNSIGNED))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			unsignedDistances = true;

			if (!m_cloud2meshDist)
			{
				cmd.warning(QObject::tr("Parameter \"-%1\" ignored: only for C2M distance!").arg(COMMAND_C2M_DIST_UNSIGNED));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2X_MAX_DISTANCE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
			{
                return cmd.error(QObject::tr("Missing parameter: value after \"-%1\"").arg(COMMAND_C2X_MAX_DISTANCE));
			}
			bool conversionOk = false;
			maxDist = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_C2X_MAX_DISTANCE));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2X_OCTREE_LEVEL))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: value after \"-%1\"").arg(COMMAND_C2X_OCTREE_LEVEL));
			}
			bool conversionOk = false;
			octreeLevel = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_C2X_OCTREE_LEVEL));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2C_SPLIT_XYZ))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			splitXYZ = true;
			
			if (m_cloud2meshDist)
			{
				cmd.warning(QObject::tr("Parameter \"-%1\" ignored: only for C2C distance!"));
			}
		}
        else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2C_SPLIT_XY_Z))
        {
            //local option confirmed, we can move on
            cmd.arguments().pop_front();

            splitXYZ = true;
            mergeXY = true;

            if (m_cloud2meshDist)
            {
                cmd.warning(QObject::tr("Parameter \"-%1\" ignored: only for C2C distance!"));
            }
        }
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_C2C_LOCAL_MODEL))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();
			
			if (!cmd.arguments().empty())
			{
				QString modelType = cmd.arguments().takeFirst().toUpper();
				if (modelType == "LS")
				{
					modelIndex = 1;
				}
				else if (modelType == "TRI")
				{
					modelIndex = 2;
				}
				else if (modelType == "HF")
				{
					modelIndex = 3;
				}
				else
				{
					return cmd.error(QObject::tr("Invalid parameter: unknown model type \"%1\"").arg(modelType));
				}
			}
			else
			{
				return cmd.error(QObject::tr("Missing parameter: model type after \"-%1\" (LS/TRI/HF)").arg(COMMAND_C2C_LOCAL_MODEL));
			}
			
			if (!cmd.arguments().empty())
			{
				QString nType = cmd.arguments().takeFirst().toUpper();
				if (nType == "KNN")
				{
					useKNN = true;
				}
				else if (nType == "SPHERE")
				{
					useKNN = false;
				}
				else
				{
					return cmd.error(QObject::tr("Invalid parameter: unknown neighborhood type \"%1\"").arg(nType));
				}
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
				{
					return cmd.error(QObject::tr("Invalid parameter: neighborhood size"));
				}
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
			{
				return cmd.error(QObject::tr("Missing parameter: max thread count after '%1'").arg(COMMAND_MAX_THREAD_COUNT));
			}
			
			bool ok;
			maxThreadCount = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || maxThreadCount < 0)
			{
				return cmd.error(QObject::tr("Invalid thread count! (after %1)").arg(COMMAND_MAX_THREAD_COUNT));
			}
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

	if (!compDlg.initDialog())
	{
		return cmd.error(QObject::tr("Failed to initialize comparison dialog"));
	}
	
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
		compDlg.flipNormalsCheckBox->setChecked(flipNormals);
		compDlg.signedDistCheckBox->setChecked(!unsignedDistances);
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
        if (mergeXY)
            compDlg.compute2DCheckBox->setChecked(true);
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
		return cmd.error(QObject::tr("An error occurred during distances computation!"));
	}
	
	compDlg.applyAndExit();
	
	QString suffix(m_cloud2meshDist ? "_C2M_DIST" : "_C2C_DIST");
	if (maxDist > 0)
	{
		suffix += QObject::tr("_MAX_DIST_%1").arg(maxDist);
	}
	
	compEntity->basename += suffix;
	
	if (cmd.autoSaveMode())
	{
		QString errorStr = cmd.exportEntity(*compEntity);
		if (!errorStr.isEmpty())
		{
			return cmd.error(errorStr);
		}
	}
	
	return true;
}

CommandC2MDist::CommandC2MDist()
	: CommandDist(true, QObject::tr("C2M distance"), COMMAND_C2M_DIST)
{}

CommandC2CDist::CommandC2CDist()
	: CommandDist(false, QObject::tr("C2C distance"), COMMAND_C2C_DIST)
{}

CommandCPS::CommandCPS()
    : ccCommandLineInterface::Command(QObject::tr("Closest Point Set"), COMMAND_CLOSEST_POINT_SET)
{}

bool CommandCPS::process(ccCommandLineInterface& cmd)
{
    cmd.print(QObject::tr("[CLOSEST POINT SET]"));

    if (cmd.clouds().size() < 2)
    {
        return cmd.error(QObject::tr("At least two point clouds are needed to compute the closest point set!"));
    }
    else if (cmd.clouds().size() > 2)
    {
        cmd.warning(QObject::tr("More than 3 point clouds loaded! We take the second one as reference by default"));
    }

    // COMPARED CLOUD / REFERENCE CLOUD
    CLCloudDesc compDesc = cmd.clouds().front();
    CLCloudDesc refDesc = cmd.clouds()[1];
    ccPointCloud* compPointCloud = compDesc.pc;
    ccPointCloud* refPointCloud = refDesc.pc;

    assert(compPointCloud && refPointCloud);

    ccProgressDialog pDlg(true, nullptr);
    CCCoreLib::DistanceComputationTools::Cloud2CloudDistancesComputationParams params;
    CCCoreLib::ReferenceCloud closestPointSet(refPointCloud);
    params.CPSet = &closestPointSet;

    // COMPUTE CLOUD 2 CLOUD DISTANCE, THIS INCLUDES THE CLOSEST POINT SET GENERATION
    int result = CCCoreLib::DistanceComputationTools::computeCloud2CloudDistances(compPointCloud, refPointCloud, params, &pDlg);

    if (result >= 0)
    {
        // the extracted CPS will get the attributes of the reference cloud
        ccPointCloud* newCloud = refPointCloud->partialClone(&closestPointSet);
        // give to newCloud a name similar to the one generated by the GUI Closest Point Set tool
        CLCloudDesc desc(
                    newCloud,
                    "[" + refDesc.basename + "]_CPSet(" + compDesc.basename + ")",
                    cmd.clouds()[0].path);
        QString errorStr = cmd.exportEntity(desc, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
        if (!errorStr.isEmpty())
        {
            cmd.error(errorStr);
        }
        //add cloud to the current pool
        cmd.clouds().push_back(desc);
    }

    return true;
}

CommandStatTest::CommandStatTest()
	: ccCommandLineInterface::Command(QObject::tr("Statistical test"), COMMAND_STAT_TEST)
{}

bool CommandStatTest::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[STATISTICAL TEST]"));
	
	//distribution
	CCCoreLib::GenericDistribution* distrib = nullptr;
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: distribution type after \"-%1\" (GAUSS/WEIBULL)").arg(COMMAND_STAT_TEST));
		}
		
		QString distribStr = cmd.arguments().takeFirst().toUpper();
		if (distribStr == "GAUSS")
		{
			//mu
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: mean value after \"GAUSS\""));
			}
			bool conversionOk = false;
			double mu = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: mean value after \"GAUSS\""));
			}
			//sigma
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: sigma value after \"GAUSS\" {mu}"));
			}
			conversionOk = false;
			double sigma = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: sigma value after \"GAUSS\" {mu}"));
			}
			
			CCCoreLib::NormalDistribution* N = new CCCoreLib::NormalDistribution();
			N->setParameters(static_cast<ScalarType>(mu), static_cast<ScalarType>(sigma*sigma)); //warning: we input sigma2 here (not sigma)
			distrib = static_cast<CCCoreLib::GenericDistribution*>(N);
		}
		else if (distribStr == "WEIBULL")
		{
			//a
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: a value after \"WEIBULL\""));
			}
			bool conversionOk = false;
			double a = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: a value after \"WEIBULL\""));
			}
			//b
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: b value after \"WEIBULL\" {a}"));
			}
			conversionOk = false;
			double b = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: b value after \"WEIBULL\" {a}"));
			}
			//c
			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: shift value after \"WEIBULL\" {a} {b}"));
			}
			conversionOk = false;
			double shift = cmd.arguments().takeFirst().toDouble(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: shift value after \"WEIBULL\" {a} {b}"));
			}
			
			CCCoreLib::WeibullDistribution* N = new CCCoreLib::WeibullDistribution();
			N->setParameters(static_cast<ScalarType>(a), static_cast<ScalarType>(b), static_cast<ScalarType>(shift));
			distrib = static_cast<CCCoreLib::GenericDistribution*>(N);
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
		{
			return cmd.error(QObject::tr("Missing parameter: p-value after distribution"));
		}
		bool conversionOk = false;
		pValue = cmd.arguments().takeFirst().toDouble(&conversionOk);
		if (!conversionOk)
		{
			return cmd.error(QObject::tr("Invalid parameter: p-value after distribution"));
		}
	}
	
	//kNN
	unsigned kNN = 16;
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("Missing parameter: neighbors after p-value"));
		}
		bool conversionOk = false;
		kNN = cmd.arguments().takeFirst().toUInt(&conversionOk);
		if (!conversionOk)
		{
			return cmd.error(QObject::tr("Invalid parameter: neighbors after p-value"));
		}
	}
	
	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No cloud available. Be sure to open one first!"));
	}
	
	QScopedPointer<ccProgressDialog> progressDialog(nullptr);
	if (!cmd.silentMode())
	{
		progressDialog.reset(new ccProgressDialog(false, cmd.widgetParent()));
		progressDialog->setAutoClose(false);
	}
	
	for (CLCloudDesc& desc : cmd.clouds())
	{
		//we apply method on currently 'output' SF
		CCCoreLib::ScalarField* outSF = desc.pc->getCurrentOutScalarField();
		if (outSF)
		{
			assert(outSF->capacity() != 0);
			
			//force Chi2 Distances field as 'IN' field (create it by the way if necessary)
			int chi2SfIdx = desc.pc->getScalarFieldIndexByName(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			if (chi2SfIdx < 0)
			{
				chi2SfIdx = desc.pc->addScalarField(CC_CHI2_DISTANCES_DEFAULT_SF_NAME);
			}
			if (chi2SfIdx < 0)
			{
				delete distrib;
				return cmd.error(QObject::tr("Couldn't allocate a new scalar field for computing chi2 distances! Try to free some memory ..."));
			}
			desc.pc->setCurrentInScalarField(chi2SfIdx);
			
			//compute octree if necessary
			ccOctree::Shared theOctree = desc.pc->getOctree();
			if (!theOctree)
			{
				theOctree = desc.pc->computeOctree(progressDialog.data());
				if (!theOctree)
				{
					delete distrib;
					cmd.error(QObject::tr("Couldn't compute octree for cloud '%1'!").arg(desc.pc->getName()));
					break;
				}
			}
			
			double chi2dist = CCCoreLib::StatisticalTestingTools::testCloudWithStatisticalModel(distrib, desc.pc, kNN, pValue, progressDialog.data(), theOctree.data());
			
			cmd.print(QObject::tr("[Chi2 Test] %1 test result = %2").arg(distrib->getName()).arg(chi2dist));
			
			//we set the theoretical Chi2 distance limit as the minimum displayed SF value so that all points below are grayed
			{
				ccScalarField* chi2SF = static_cast<ccScalarField*>(desc.pc->getCurrentInScalarField());
				assert(chi2SF);
				chi2SF->computeMinAndMax();
				chi2dist *= chi2dist;
				chi2SF->setMinDisplayed(static_cast<ScalarType>(chi2dist));
				chi2SF->setSymmetricalScale(false);
				chi2SF->setSaturationStart(static_cast<ScalarType>(chi2dist));
				//chi2SF->setSaturationStop(chi2dist);
				desc.pc->setCurrentDisplayedScalarField(chi2SfIdx);
				desc.pc->showSF(true);
			}
			
			desc.basename += QObject::tr("_STAT_TEST_%1").arg(distrib->getName());
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(desc);
				if (!errorStr.isEmpty())
				{
					return cmd.error(errorStr);
				}
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
	: ccCommandLineInterface::Command(QObject::tr("Delaunay triangulation"), COMMAND_DELAUNAY)
{}

bool CommandDelaunayTri::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[DELAUNAY TRIANGULATION]"));
	
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
			{
				return cmd.error(QObject::tr("Missing parameter: max edge length value after '%1'").arg(COMMAND_DELAUNAY_MAX_EDGE_LENGTH));
			}
			bool ok;
			maxEdgeLength = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok)
			{
				return cmd.error(QObject::tr("Invalid value for max edge length! (after %1)").arg(COMMAND_DELAUNAY_MAX_EDGE_LENGTH));
				cmd.print(QObject::tr("Max edge length: %1").arg(maxEdgeLength));
			}
		}
		else
		{
			break;
		}
	}
	
	cmd.print(QObject::tr("Axis aligned: %1").arg(axisAligned ? "yes" : "no"));
	
	//try to triangulate each cloud
	for (CLCloudDesc& desc : cmd.clouds())
	{
		cmd.print(QObject::tr("\tProcessing cloud %1").arg(!desc.pc->getName().isEmpty() ? desc.pc->getName() : "no name"));
		
		ccMesh* mesh = ccMesh::Triangulate(desc.pc,
										   axisAligned ? CCCoreLib::DELAUNAY_2D_AXIS_ALIGNED : CCCoreLib::DELAUNAY_2D_BEST_LS_PLANE,
										   false,
										   static_cast<PointCoordinateType>(maxEdgeLength),
										   2 //XY plane by default
										   );
		
		if (mesh)
		{
			cmd.print(QObject::tr("\tResulting mesh: #%1 faces, %2 vertices").arg(mesh->size()).arg(mesh->getAssociatedCloud()->size()));
			
			CLMeshDesc meshDesc;
			{
				meshDesc.mesh = mesh;
				meshDesc.basename = desc.basename;
				meshDesc.path = desc.path;
				meshDesc.indexInFile = desc.indexInFile;
			}
			
			//save mesh
			if (cmd.autoSaveMode())
			{
				QString outputFilename;
				QString errorStr = cmd.exportEntity(meshDesc, "DELAUNAY", &outputFilename);
				if (!errorStr.isEmpty())
				{
					cmd.warning(errorStr);
				}
			}
			
			//add the resulting mesh to the main set
			cmd.meshes().push_back(meshDesc);
			
			//the mesh takes ownership of the cloud.
			//Therefore we have to remove all clouds from the 'cloud set'! (see below)
			//(otherwise bad things will happen when we'll clear it later ;)
			desc.pc->setEnabled(false);
			mesh->addChild(desc.pc);
		}
	}
	//mehses have taken ownership of the clouds!
	cmd.clouds().resize(0);
	
	return true;
}

CommandSFArithmetic::CommandSFArithmetic()
	: ccCommandLineInterface::Command(QObject::tr("SF arithmetic"), COMMAND_SF_ARITHMETIC)
{}

bool CommandSFArithmetic::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SF ARITHMETIC]"));
	
	if (cmd.arguments().size() < 2)
	{
		return cmd.error(QObject::tr("Missing parameter(s): SF index and/or operation after '%1' (2 values expected)").arg(COMMAND_SF_ARITHMETIC));
	}
	
	//read SF index
	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

	//read operation type
	ccScalarFieldArithmeticsDlg::Operation operation = ccScalarFieldArithmeticsDlg::INVALID;
	{
		QString opName = cmd.arguments().takeFirst();
		operation = ccScalarFieldArithmeticsDlg::GetOperationByName(opName);
		if (operation == ccScalarFieldArithmeticsDlg::INVALID)
		{
			return cmd.error(QObject::tr("Unknown operation! (%1)").arg(opName));
		}
		else if (operation <= ccScalarFieldArithmeticsDlg::MAX || operation == ccScalarFieldArithmeticsDlg::SET)
		{
			return cmd.error(QObject::tr("Operation %1 can't be applied with %2. Consider using the %3 command").arg(opName, COMMAND_SF_ARITHMETIC, COMMAND_SF_OP));
		}
	}

	bool inPlace = false;

	//read the optional arguments
	while (!cmd.arguments().empty())
	{
		QString argument = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(argument, COMMAND_SF_ARITHMETIC_IN_PLACE))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			inPlace = true;
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	//apply operation on clouds
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			int thisSFIndex = GetScalarFieldIndex(desc.pc, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				if (!ccScalarFieldArithmeticsDlg::Apply(desc.pc, operation, thisSFIndex, inPlace))
				{
					return cmd.error(QObject::tr("Failed to apply operation on cloud '%1'").arg(desc.pc->getName()));
				}
				else if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc, "SF_ARITHMETIC");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
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
		if (cloud && !isLocked)
		{
			int thisSFIndex = GetScalarFieldIndex(cloud, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, thisSFIndex, inPlace))
				{
					return cmd.error(QObject::tr("Failed to apply operation on mesh '%1'").arg(mesh->getName()));
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
	}
	
	return true;
}

CommandSFOperation::CommandSFOperation()
	: ccCommandLineInterface::Command(QObject::tr("SF operation"), COMMAND_SF_OP)
{}

bool CommandSFOperation::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[SF OPERATION]"));
	
	if (cmd.arguments().size() < 3)
	{
		return cmd.error(QObject::tr("Missing parameter(s): SF index and/or operation and/or scalar value after '%1' (3 values expected)").arg(COMMAND_SF_OP));
	}
	
	//read SF index
	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

	//read operation type
	ccScalarFieldArithmeticsDlg::Operation operation = ccScalarFieldArithmeticsDlg::INVALID;
	{
		QString opName = cmd.arguments().takeFirst();
		operation = ccScalarFieldArithmeticsDlg::GetOperationByName(opName);
		if (operation == ccScalarFieldArithmeticsDlg::INVALID)
		{
			return cmd.error(QObject::tr("Unknown operation! (%1)").arg(opName));
		}
		else if (operation > ccScalarFieldArithmeticsDlg::MAX && operation != ccScalarFieldArithmeticsDlg::SET)
		{
			return cmd.error(QObject::tr("Operation %1 can't be applied with %2. Consider using the %3 command").arg(opName, COMMAND_SF_OP, COMMAND_SF_ARITHMETIC));
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
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			int thisSFIndex = GetScalarFieldIndex(desc.pc, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				if (!ccScalarFieldArithmeticsDlg::Apply(desc.pc, operation, thisSFIndex, true, &sf2))
				{
					return cmd.error(QObject::tr("Failed to apply operation on cloud '%1'").arg(desc.pc->getName()));
				}
				else if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc, "SF_OP");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
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
		if (cloud && !isLocked)
		{
			int thisSFIndex = GetScalarFieldIndex(cloud, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, thisSFIndex, true, &sf2))
				{
					return cmd.error(QObject::tr("Failed to apply operation on mesh '%1'").arg(mesh->getName()));
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
	}
	
	return true;
}

CommandSFOperationSF::CommandSFOperationSF()
    : ccCommandLineInterface::Command(QObject::tr("SF (add, sub, mult, div) SF"), COMMAND_SF_OP_SF)
{}

bool CommandSFOperationSF::process(ccCommandLineInterface& cmd)
{
    cmd.print(QObject::tr("[SF OP [ADD | SUB | MULT | DIV] SF]"));

    if (cmd.arguments().size() < 3)
    {
        return cmd.error(QObject::tr("Missing parameter(s): SF index and operation and SF index '%1' (3 values expected)").arg(COMMAND_SF_OP_SF));
    }

    //read SF index 1
	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

    //read operation type
    ccScalarFieldArithmeticsDlg::Operation operation = ccScalarFieldArithmeticsDlg::INVALID;
    {
        QString opName = cmd.arguments().takeFirst();
        operation = ccScalarFieldArithmeticsDlg::GetOperationByName(opName);
        if (operation == ccScalarFieldArithmeticsDlg::INVALID)
        {
            return cmd.error(QObject::tr("Unknown operation! (%1)").arg(opName));
        }
        else if (operation > ccScalarFieldArithmeticsDlg::MAX)
        {
            return cmd.error(QObject::tr("Operation %1 can't be applied with %2").arg(opName, COMMAND_SF_OP_SF));
        }
    }

    //read SF index 2
	int sfIndex2 = -1;
	QString sfName2;
	GetSFIndexOrName(cmd, sfIndex2, sfName2);

	//apply operation on clouds
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			int thisSFFIndex = GetScalarFieldIndex(desc.pc, sfIndex, sfName);
			int thisSFFIndex2 = GetScalarFieldIndex(desc.pc, sfIndex2, sfName2);
			if (thisSFFIndex >= 0 && thisSFFIndex2 >= 0)
			{
				ccScalarFieldArithmeticsDlg::SF2 sf2;
				{
					sf2.isConstantValue = false;
					sf2.sfIndex = thisSFFIndex2;
				}

				if (!ccScalarFieldArithmeticsDlg::Apply(desc.pc, operation, thisSFFIndex, true, &sf2))
				{
					return cmd.error(QObject::tr("Failed top apply operation on cloud '%1'").arg(desc.pc->getName()));
				}
				else if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc, "SF_OP_SF");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
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
		if (cloud && !isLocked)
		{
			int thisSFFIndex = GetScalarFieldIndex(cloud, sfIndex, sfName);
			int thisSFFIndex2 = GetScalarFieldIndex(cloud, sfIndex2, sfName2);
			if (thisSFFIndex >= 0 && thisSFFIndex2 >= 0)
			{
				ccScalarFieldArithmeticsDlg::SF2 sf2;
				{
					sf2.isConstantValue = false;
					sf2.sfIndex = thisSFFIndex2;
				}

				if (!ccScalarFieldArithmeticsDlg::Apply(cloud, operation, thisSFFIndex, true, &sf2))
				{
					return cmd.error(QObject::tr("Failed top apply operation on mesh '%1'").arg(mesh->getName()));
				}
				else if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(cmd.meshes()[j], "SF_OP_SF");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
		}
	}

    return true;
}

CommandSFInterpolation::CommandSFInterpolation()
    : ccCommandLineInterface::Command(QObject::tr("SF interpolation"), COMMAND_SF_INTERP)
{}

bool CommandSFInterpolation::process(ccCommandLineInterface& cmd)
{
    cmd.print(QObject::tr("[SF INTERPOLATION]"));

    if (cmd.arguments().size() < 1)
        return cmd.error(QObject::tr("Missing parameter(s): SF index after '%1' (1 value expected)").arg(COMMAND_SF_INTERP));

    if (cmd.clouds().size() < 2)
        return cmd.error(QObject::tr("Unexpected number of clouds for '%1' (at least 2 clouds expected: first = source, second = dest)").arg(COMMAND_SF_INTERP));

    //read sf index or name
	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

    bool destIsFirst = false;
    while (!cmd.arguments().empty())
    {
        QString argument = cmd.arguments().front();
        if (ccCommandLineInterface::IsCommand(argument, COMMAND_SF_INTERP_DEST_IS_FIRST))
        {
            cmd.print(QObject::tr("[DEST_IS_FIRST]"));
            //local option confirmed, we can move on
            cmd.arguments().pop_front();
            destIsFirst = true;
        }
        else
        {
            break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
        }
    }

    ccPointCloud* source = cmd.clouds()[0].pc;
    ccPointCloud* dest = cmd.clouds()[1].pc;
    if (destIsFirst) // swap source and destination
    {
        source = cmd.clouds()[1].pc;
		dest = cmd.clouds()[0].pc;
    }

	sfIndex = GetScalarFieldIndex(source, sfIndex, sfName);
	if (sfIndex < 0)
	{
		return false;
	}

	cmd.print("SF to interpolate: index " + QString::number(sfIndex) + ", name " + source->getScalarField(sfIndex)->getName());

	//semi-persistent parameters
	ccPointCloudInterpolator::Parameters params;
	{
		params.method = ccPointCloudInterpolator::Parameters::NEAREST_NEIGHBOR; // nearest neighbor
		params.algo = ccPointCloudInterpolator::Parameters::NORMAL_DIST; // normal distribution
		params.knn = 6;
		params.radius = static_cast<float>(dest->getOwnBB().getDiagNormd() / 100);
		params.sigma = params.radius / 2.5; // see ccInterpolationDlg::onRadiusUpdated
	}

	return ccEntityAction::interpolateSFs(source, dest, sfIndex, params, cmd.widgetParent());
}

CommandSFRename::CommandSFRename()
	: ccCommandLineInterface::Command(QObject::tr("Rename SF"), COMMAND_RENAME_SF)
{}

bool CommandSFRename::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[RENAME SF]"));

	if (cmd.arguments().size() < 2)
	{
		return cmd.error(QObject::tr("Missing parameter(s): SF index and/or scalar field name after '%1' (2 values expected)").arg(COMMAND_RENAME_SF));
	}

	//read SF index
	int sfIndex = -1;
	QString sfName;
	GetSFIndexOrName(cmd, sfIndex, sfName);

	//read the SF name
	QString newSFName = cmd.arguments().takeFirst();

	//apply operation on clouds
	for (CLCloudDesc& desc : cmd.clouds())
	{
		if (desc.pc)
		{
			int thisSFIndex = GetScalarFieldIndex(desc.pc, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				int indexOfSFWithSameName = desc.pc->getScalarFieldIndexByName(qPrintable(newSFName));
				if (indexOfSFWithSameName >= 0 && thisSFIndex != indexOfSFWithSameName)
				{
					return cmd.error("A SF with the same name is already defined on cloud " + desc.pc->getName());
				}
				CCCoreLib::ScalarField* sf = desc.pc->getScalarField(thisSFIndex);
				if (!sf)
				{
					assert(false);
					return cmd.error("Internal error: invalid SF index");
				}
				sf->setName(qPrintable(newSFName));

				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc, "SF_RENAMED");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
		}
	}

	//and meshes!
	for (CLMeshDesc& desc : cmd.meshes())
	{
		bool isLocked = false;
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(desc.mesh, &isLocked);
		if (cloud && !isLocked)
		{
			int thisSFIndex = GetScalarFieldIndex(cloud, sfIndex, sfName);
			if (thisSFIndex >= 0)
			{
				int indexOfSFWithSameName = cloud->getScalarFieldIndexByName(qPrintable(newSFName));
				if (indexOfSFWithSameName >= 0 && thisSFIndex != indexOfSFWithSameName)
				{
					return cmd.error("A SF with the same name is already defined on cloud " + cloud->getName());
				}
				CCCoreLib::ScalarField* sf = cloud->getScalarField(thisSFIndex);
				if (!sf)
				{
					assert(false);
					return cmd.error("Internal error: invalid SF index");
				}
				sf->setName(qPrintable(newSFName));

				if (cmd.autoSaveMode())
				{
					QString errorStr = cmd.exportEntity(desc, "SF_RENAMED");
					if (!errorStr.isEmpty())
					{
						return cmd.error(errorStr);
					}
				}
			}
		}
	}

	return true;
}

CommandSFAddConst::CommandSFAddConst()
    : ccCommandLineInterface::Command(QObject::tr("SF add constant scalar field"), COMMAND_SF_ADD_CONST)
{}

bool CommandSFAddConst::process(ccCommandLineInterface& cmd)
{
    cmd.print(QObject::tr("[ADD CONST SF] Note: this operation is only done on clouds"));

    if (cmd.arguments().size() < 2)
    {
        return cmd.error(QObject::tr("Missing parameter(s): SF name and value after '%1' (2 values expected)").arg(COMMAND_SF_ADD_CONST));
    }

    //read the SF name
    QString sfName = cmd.arguments().takeFirst();

    //read constant value
    bool ok = true;
	ScalarType value = static_cast<ScalarType>(cmd.arguments().takeFirst().toDouble(&ok));
    if (!ok)
    {
        return cmd.error(QObject::tr("Invalid constant value! (after %1)").arg(COMMAND_SF_ADD_CONST));
    }

    //apply operation on clouds
    for (CLCloudDesc& desc : cmd.clouds())
    {
        if (desc.pc)
        {
            // check that there is no existing scalar field with the same name
            int indexOfSFWithSameName = desc.pc->getScalarFieldIndexByName(qPrintable(sfName));
            if (indexOfSFWithSameName >= 0)
                return cmd.error("A SF with the same name is already defined on cloud " + desc.pc->getName());

            // add the new scalar field
            int sfIndex = desc.pc->addScalarField(qPrintable(sfName));
            if (sfIndex == -1)
            {
                return cmd.error("Internal error: addScalarField failed");
            }
            CCCoreLib::ScalarField* sf = desc.pc->getScalarField(sfIndex);
			assert(sf);
			for (unsigned index = 0; index < desc.pc->size(); index++)
			{
				sf->setValue(index, value);
			}
            
			if (cmd.autoSaveMode())
            {
                QString errorStr = cmd.exportEntity(desc, "SF_ADDED");
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

bool CommandICP::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[ICP]"));

	//look for local options
	bool referenceIsFirst = false;
	bool adjustScale = false;
	bool enableFarthestPointRemoval = false;
	double minErrorDiff = 1.0e-6;
	unsigned iterationCount = 0;
	unsigned randomSamplingLimit = 20000;
	unsigned overlap = 100;
	int modelWeightsSFIndex = -1;
	QString modelWeightsSFIndexName;
	int dataWeightsSFIndex = -1;
	QString dataWeightsSFIndexName;
	int maxThreadCount = 0;
	int transformationFilters = CCCoreLib::RegistrationTools::SKIP_NONE;

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
			{
				return cmd.error(QObject::tr("Missing parameter: min error difference after '%1'").arg(COMMAND_ICP_MIN_ERROR_DIIF));
			}
			bool ok;
			minErrorDiff = cmd.arguments().takeFirst().toDouble(&ok);
			if (!ok || minErrorDiff <= 0)
			{
				return cmd.error(QObject::tr("Invalid value for min. error difference! (after %1)").arg(COMMAND_ICP_MIN_ERROR_DIIF));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_ITERATION_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: number of iterations after '%1'").arg(COMMAND_ICP_ITERATION_COUNT));
			}
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
			{
				return cmd.error(QObject::tr("Missing parameter: overlap percentage after '%1'").arg(COMMAND_ICP_OVERLAP));
			}
			bool ok;
			QString arg = cmd.arguments().takeFirst();
			overlap = arg.toUInt(&ok);
			if (!ok || overlap < 10 || overlap > 100)
			{
				return cmd.error(QObject::tr("Invalid overlap value! (%1 --> should be between 10 and 100)").arg(arg));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_RANDOM_SAMPLING_LIMIT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: random sampling limit value after '%1'").arg(COMMAND_ICP_RANDOM_SAMPLING_LIMIT));
			}
			bool ok;
			randomSamplingLimit = cmd.arguments().takeFirst().toUInt(&ok);
			if (!ok || randomSamplingLimit < 3)
			{
				return cmd.error(QObject::tr("Invalid random sampling limit! (after %1)").arg(COMMAND_ICP_RANDOM_SAMPLING_LIMIT));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_USE_MODEL_SF_AS_WEIGHT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: SF index after '%1'").arg(COMMAND_ICP_USE_MODEL_SF_AS_WEIGHT));
			}

			//read SF index
			GetSFIndexOrName(cmd, modelWeightsSFIndex, modelWeightsSFIndexName);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_USE_DATA_SF_AS_WEIGHT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: SF index after '%1'").arg(COMMAND_ICP_USE_DATA_SF_AS_WEIGHT));
			}

			//read SF index
			GetSFIndexOrName(cmd, dataWeightsSFIndex, dataWeightsSFIndexName);
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_MAX_THREAD_COUNT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (cmd.arguments().empty())
			{
				return cmd.error(QObject::tr("Missing parameter: max thread count after '%1'").arg(COMMAND_MAX_THREAD_COUNT));
			}

			bool ok;
			maxThreadCount = cmd.arguments().takeFirst().toInt(&ok);
			if (!ok || maxThreadCount < 0)
			{
				return cmd.error(QObject::tr("Invalid thread count! (after %1)").arg(COMMAND_MAX_THREAD_COUNT));
			}
		}
		else if (ccCommandLineInterface::IsCommand(argument, COMMAND_ICP_ROT))
		{
			//local option confirmed, we can move on
			cmd.arguments().pop_front();

			if (!cmd.arguments().empty())
			{
				QString rotation = cmd.arguments().takeFirst().toUpper();
				if (rotation == "XYZ")
				{
					transformationFilters = CCCoreLib::RegistrationTools::SKIP_NONE;
				}
				else if (rotation == "X")
				{
					transformationFilters = CCCoreLib::RegistrationTools::SKIP_RYZ;
				}
				else if (rotation == "Y")
				{
					transformationFilters = CCCoreLib::RegistrationTools::SKIP_RXZ;
				}
				else if (rotation == "Z")
				{
					transformationFilters = CCCoreLib::RegistrationTools::SKIP_RXY;
				}
				else if (rotation == "NONE")
				{
					transformationFilters = CCCoreLib::RegistrationTools::SKIP_ROTATION;
				}
				else
				{
					return cmd.error(QObject::tr("Invalid parameter: unknown rotation filter \"%1\"").arg(rotation));
				}
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
	CLEntityDesc* dataAndModel[2]{ nullptr, nullptr };
	{
		int index = 0;
		if (!cmd.clouds().empty())
		{
			dataAndModel[index++] = &cmd.clouds()[0];
			if (cmd.clouds().size() > 1)
			{
				dataAndModel[index++] = &cmd.clouds()[1];
			}
		}
		if (index < 2 && !cmd.meshes().empty())
		{
			dataAndModel[index++] = &cmd.meshes()[0];
			if (index < 2 && cmd.meshes().size() > 1)
			{
				dataAndModel[index++] = &cmd.meshes()[1];
			}
		}

		if (index < 2)
		{
			return cmd.error(QObject::tr("Not enough loaded entities (expect at least 2!)"));
		}
	}

	//put them in the right order (data first, model next)
	if (referenceIsFirst)
	{
		std::swap(dataAndModel[0], dataAndModel[1]);
	}

	//check that the weights (scalar fields) exist
	if (dataWeightsSFIndex >= 0 || !dataWeightsSFIndexName.isEmpty())
	{
		ccPointCloud* dataAsCloud = ccHObjectCaster::ToPointCloud(dataAndModel[0]->getEntity());
		if (!dataAsCloud)
		{
			dataWeightsSFIndex = GetScalarFieldIndex(dataAsCloud, dataWeightsSFIndex, dataWeightsSFIndexName);
			if (dataWeightsSFIndex >= 0)
			{
				cmd.print(QObject::tr("[ICP] SF #%1 (data entity) will be used as weights").arg(dataWeightsSFIndex));
				dataAsCloud->setCurrentDisplayedScalarField(dataWeightsSFIndex);
			}
		}
	}

	if (modelWeightsSFIndex >= 0 || !modelWeightsSFIndexName.isEmpty())
	{
		ccPointCloud* modelAsCloud = ccHObjectCaster::ToPointCloud(dataAndModel[1]->getEntity());
		if (!modelAsCloud)
		{
			modelWeightsSFIndex = GetScalarFieldIndex(modelAsCloud, modelWeightsSFIndex, modelWeightsSFIndexName);
			if (modelWeightsSFIndex >= 0)
			{
				cmd.print(QObject::tr("[ICP] SF #%1 (model entity) will be used as weights").arg(modelWeightsSFIndex));
				modelAsCloud->setCurrentDisplayedScalarField(modelWeightsSFIndex);
			}
		}
	}
	
	ccGLMatrix transMat;
	double finalError = 0.0;
	double finalScale = 1.0;
	unsigned finalPointCount = 0;

	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	{
		parameters.convType					= (iterationCount != 0 ? CCCoreLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE : CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE);
		parameters.minRMSDecrease			= minErrorDiff;
		parameters.nbMaxIterations			= iterationCount;
		parameters.adjustScale				= adjustScale;
		parameters.filterOutFarthestPoints	= enableFarthestPointRemoval;
		parameters.samplingLimit			= randomSamplingLimit;
		parameters.finalOverlapRatio		= overlap / 100.0;
		parameters.transformationFilters	= transformationFilters;
		parameters.maxThreadCount			= maxThreadCount;
		parameters.useC2MSignedDistances	= false; //TODO
		parameters.normalsMatching			= CCCoreLib::ICPRegistrationTools::NO_NORMAL; //TODO
	}

	if (ccRegistrationTools::ICP(	dataAndModel[0]->getEntity(),
									dataAndModel[1]->getEntity(),
									transMat,
									finalScale,
									finalError,
									finalPointCount,
									parameters,
									dataWeightsSFIndex >= 0,
									modelWeightsSFIndex >= 0,
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
			{
				return cmd.error(errorStr);
			}
		}
	}
	else
	{
		return false;
	}
	
	return true;
}

CommandChangePLYExportFormat::CommandChangePLYExportFormat()
	: ccCommandLineInterface::Command(QObject::tr("Change PLY output format"), COMMAND_PLY_EXPORT_FORMAT)
{}

bool CommandChangePLYExportFormat::process(ccCommandLineInterface& cmd)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: format (ASCII, BINARY_LE, or BINARY_BE) after '%1'").arg(COMMAND_PLY_EXPORT_FORMAT));
	}
	
	//if (fileFilter != PlyFilter::GetFileFilter())
	//	cmd.warning(QObject::tr("Argument '%1' is only applicable to PLY format!").arg(argument));
	
	QString plyFormat = cmd.arguments().takeFirst().toUpper();
	//printf("%s\n",qPrintable(plyFormat));
	
	if (plyFormat == "ASCII")
	{
		PlyFilter::SetDefaultOutputFormat(PLY_ASCII);
	}
	else if (plyFormat == "BINARY_BE")
	{
		PlyFilter::SetDefaultOutputFormat(PLY_BIG_ENDIAN);
	}
	else if (plyFormat == "BINARY_LE")
	{
		PlyFilter::SetDefaultOutputFormat(PLY_LITTLE_ENDIAN);
	}
	else
	{
		return cmd.error(QObject::tr("Invalid PLY format! ('%1')").arg(plyFormat));
	}
	
	return true;
}

CommandForceNormalsComputation::CommandForceNormalsComputation()
	: ccCommandLineInterface::Command(QObject::tr("Compute structured cloud normals"), COMMAND_COMPUTE_GRIDDED_NORMALS)
{}

bool CommandForceNormalsComputation::process(ccCommandLineInterface& cmd)
{
	//simply change the default filter behavior
	cmd.fileLoadingParams().autoComputeNormals = true;
	
	return true;
}

CommandSave::CommandSave(const QString& name, const QString& keyword)
	: ccCommandLineInterface::Command(name, keyword)
{}

bool CommandSave::ParseFileNames(ccCommandLineInterface& cmd, QStringList &fileNames)
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
			{
				return cmd.error(QObject::tr("A file starting with %1 does not have a closing %1").arg(firstChar));
			}
			
			fileNames.push_back(argument.mid(1, end - 1));
			argument.remove(0, end + 1);
			if (argument.startsWith(' '))
			{
				argument.remove(0, 1);
			}
		}
		else
		{
			auto end = argument.indexOf(' ');
			if (end == -1)
			{
				end = argument.length();
			}
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
	: CommandSave(QObject::tr("Save clouds"), COMMAND_SAVE_CLOUDS)
{}

bool CommandSaveClouds::process(ccCommandLineInterface& cmd)
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
			{
				return false;
			}
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	if (setFileNames && allAtOnce && fileNames.size() != 1)
	{
		return cmd.error(QObject::tr("Invalid parameter: specified %1 file names, but ALL_AT_ONCE is on").arg(fileNames.size()));
	}
	if (setFileNames && !allAtOnce && fileNames.size() != cmd.clouds().size())
	{
		return cmd.error(QObject::tr("Invalid parameter: specified %1 file names, but there are %2 clouds").arg(fileNames.size()).arg(cmd.clouds().size()));
	}
	
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
	: CommandSave(QObject::tr("Save meshes"), COMMAND_SAVE_MESHES)
{}

bool CommandSaveMeshes::process(ccCommandLineInterface& cmd)
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
			{
				return false;
			}
		}
		else
		{
			break; //as soon as we encounter an unrecognized argument, we break the local loop to go back to the main one!
		}
	}
	
	if (setFileNames && allAtOnce && fileNames.size() != 1)
	{
		return cmd.error(QObject::tr("Invalid parameter: specified %1 file names, but ALL_AT_ONCE is on").arg(fileNames.size()));
	}
	if (setFileNames && !allAtOnce && fileNames.size() != cmd.meshes().size())
	{
		return cmd.error(QObject::tr("Invalid parameter: specified %1 file names, but there are %2 meshes").arg(fileNames.size()).arg(cmd.meshes().size()));
	}
	
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
	: ccCommandLineInterface::Command(QObject::tr("Auto save state"), COMMAND_AUTO_SAVE)
{}

bool CommandAutoSave::process(ccCommandLineInterface& cmd)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: option after '%1' (%2/%3)").arg(COMMAND_AUTO_SAVE, OPTION_ON, OPTION_OFF));
	}
	
	QString option = cmd.arguments().takeFirst().toUpper();
	if (option == OPTION_ON)
	{
		cmd.print(QObject::tr("Auto-save is enabled"));
		cmd.toggleAutoSaveMode(true);
	}
	else if (option == OPTION_OFF)
	{
		cmd.print(QObject::tr("Auto-save is disabled"));
		cmd.toggleAutoSaveMode(false);
	}
	else
	{
		return cmd.error(QObject::tr("Unrecognized option after '%1' (%2 or %3 expected)").arg(COMMAND_AUTO_SAVE, OPTION_ON, OPTION_OFF));
	}
	
	return true;
}

CommandLogFile::CommandLogFile()
	: ccCommandLineInterface::Command(QObject::tr("Set log file"), COMMAND_LOG_FILE)
{}

bool CommandLogFile::process(ccCommandLineInterface& cmd)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: filename after '%1'").arg(COMMAND_LOG_FILE));
	}
	
	QString filename = cmd.arguments().takeFirst();
	if (!ccConsole::TheInstance(false))
	{
		assert(cmd.silentMode());
		ccConsole::Init();
	}
	
	return ccConsole::TheInstance()->setLogFile(filename);
}

CommandClear::CommandClear()
	: ccCommandLineInterface::Command(QObject::tr("Clear"), COMMAND_CLEAR)
{}

bool CommandClear::process(ccCommandLineInterface& cmd)
{
	cmd.removeClouds(false);
	cmd.removeMeshes(false);
	return true;
}

CommandClearClouds::CommandClearClouds()
	: ccCommandLineInterface::Command(QObject::tr("Clear clouds"), COMMAND_CLEAR_CLOUDS)
{}

bool CommandClearClouds::process(ccCommandLineInterface& cmd)
{
	cmd.removeClouds(false);
	return true;
}

CommandPopClouds::CommandPopClouds()
	: ccCommandLineInterface::Command(QObject::tr("Pop clouds"), COMMAND_POP_CLOUDS)
{}

bool CommandPopClouds::process(ccCommandLineInterface& cmd)
{
	cmd.removeClouds(true);
	return true;
}

CommandClearMeshes::CommandClearMeshes()
	: ccCommandLineInterface::Command(QObject::tr("Clear meshes"), COMMAND_CLEAR_MESHES)
{}

bool CommandClearMeshes::process(ccCommandLineInterface& cmd)
{
	cmd.removeMeshes(false);
	return true;
}

CommandPopMeshes::CommandPopMeshes()
	: ccCommandLineInterface::Command(QObject::tr("Pop meshes"), COMMAND_POP_MESHES)
{}

bool CommandPopMeshes::process(ccCommandLineInterface& cmd)
{
	cmd.removeMeshes(true);
	return true;
}

CommandSetNoTimestamp::CommandSetNoTimestamp()
	: ccCommandLineInterface::Command(QObject::tr("No timestamp"), COMMAND_NO_TIMESTAMP)
{}

bool CommandSetNoTimestamp::process(ccCommandLineInterface& cmd)
{
	cmd.toggleAddTimestamp(false);
	return true;
}

CommandMoment::CommandMoment()
	: ccCommandLineInterface::Command(QObject::tr("1st order moment"), COMMAND_MOMENT)
{}

bool CommandMoment::process(ccCommandLineInterface& cmd)
{
	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: kernel size after %1").arg(COMMAND_MOMENT));
	}

	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
	{
		return cmd.error(QObject::tr("Failed to read a numerical parameter: kernel size. Got '%1' instead.").arg(kernelStr));
	}
	cmd.print(QObject::tr("\tKernel size: %1").arg(kernelSize));

	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to compute first order moment! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_MOMENT));
	}

	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		entities[i] = cmd.clouds()[i].pc;
	}

	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCCoreLib::GeometricalAnalysisTools::MomentOrder1, 0, kernelSize, entities, nullptr, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("MOMENT_KERNEL_%2").arg(kernelSize)))
		{
			return false;
		}
	}
	return true;
}

CommandFeature::CommandFeature()
	: ccCommandLineInterface::Command(QObject::tr("Feature"), COMMAND_FEATURE)
{}

bool CommandFeature::process(ccCommandLineInterface& cmd)
{
	cmd.print(QObject::tr("[FEATURE]"));

	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: feature type after \"-%1\"").arg(COMMAND_FEATURE));
	}

	QString featureTypeStr = cmd.arguments().takeFirst().toUpper();
	CCCoreLib::Neighbourhood::GeomFeature featureType;

	if (featureTypeStr == "SUM_OF_EIGENVALUES")
	{
		featureType = CCCoreLib::Neighbourhood::EigenValuesSum;
	}
	else if (featureTypeStr == "OMNIVARIANCE")
	{
		featureType = CCCoreLib::Neighbourhood::Omnivariance;
	}
	else if (featureTypeStr == "EIGENTROPY")
	{
		featureType = CCCoreLib::Neighbourhood::EigenEntropy;
	}
	else if (featureTypeStr == "ANISOTROPY")
	{
		featureType = CCCoreLib::Neighbourhood::Anisotropy;
	}
	else if (featureTypeStr == "PLANARITY")
	{
		featureType = CCCoreLib::Neighbourhood::Planarity;
	}
	else if (featureTypeStr == "LINEARITY")
	{
		featureType = CCCoreLib::Neighbourhood::Linearity;
	}
	else if (featureTypeStr == "PCA1")
	{
		featureType = CCCoreLib::Neighbourhood::PCA1;
	}
	else if (featureTypeStr == "PCA2")
	{
		featureType = CCCoreLib::Neighbourhood::PCA2;
	}
	else if (featureTypeStr == "SURFACE_VARIATION")
	{
		featureType = CCCoreLib::Neighbourhood::SurfaceVariation;
	}
	else if (featureTypeStr == "SPHERICITY")
	{
		featureType = CCCoreLib::Neighbourhood::Sphericity;
	}
	else if (featureTypeStr == "VERTICALITY")
	{
		featureType = CCCoreLib::Neighbourhood::Verticality;
	}
	else if (featureTypeStr == "EIGENVALUE1")
	{
		featureType = CCCoreLib::Neighbourhood::EigenValue1;
	}
	else if (featureTypeStr == "EIGENVALUE2")
	{
		featureType = CCCoreLib::Neighbourhood::EigenValue2;
	}
	else if (featureTypeStr == "EIGENVALUE3")
	{
		featureType = CCCoreLib::Neighbourhood::EigenValue3;
	}
	else
	{
		return cmd.error(QObject::tr("Invalid feature type after \"-%1\". Got '%2' instead of:\n\
- SUM_OF_EIGENVALUES\n\
- OMNIVARIANCE\n\
- EIGENTROPY\n\
- ANISOTROPY\n\
- PLANARITY\n\
- LINEARITY\n\
- PCA1\n\
- PCA2\n\
- SURFACE_VARIATION\n\
- SPHERICITY\n\
- VERTICALITY\n\
- EIGENVALUE1\n\
- EIGENVALUE2\n\
- EIGENVALUE3").arg(COMMAND_FEATURE, featureTypeStr));
	}

	if (cmd.arguments().empty())
	{
		return cmd.error(QObject::tr("Missing parameter: kernel size after feature type"));
	}

	bool paramOk = false;
	QString kernelStr = cmd.arguments().takeFirst();
	PointCoordinateType kernelSize = static_cast<PointCoordinateType>(kernelStr.toDouble(&paramOk));
	if (!paramOk)
	{
		return cmd.error(QObject::tr("Failed to read a numerical parameter: kernel size. Got '%1' instead.").arg(kernelStr));
	}
	cmd.print(QObject::tr("\tKernel size: %1").arg(kernelSize));

	if (cmd.clouds().empty())
	{
		return cmd.error(QObject::tr("No point cloud on which to compute feature! (be sure to open one with \"-%1 [cloud filename]\" before \"-%2\")").arg(COMMAND_OPEN, COMMAND_FEATURE));
	}

	//Call MainWindow generic method
	ccHObject::Container entities;
	entities.resize(cmd.clouds().size());
	for (size_t i = 0; i < cmd.clouds().size(); ++i)
	{
		entities[i] = cmd.clouds()[i].pc;
	}

	if (ccLibAlgorithms::ComputeGeomCharacteristic(CCCoreLib::GeometricalAnalysisTools::Feature, featureType, kernelSize, entities, nullptr, cmd.widgetParent()))
	{
		//save output
		if (cmd.autoSaveMode() && !cmd.saveClouds(QObject::tr("%1_FEATURE_KERNEL_%2").arg(featureTypeStr).arg(kernelSize)))
		{
			return false;
		}
	}
	return true;
}

CommandDebugCmdLine::CommandDebugCmdLine()
	: ccCommandLineInterface::Command(QObject::tr("Debug Command Line"), COMMAND_DEBUG)
{}

bool CommandDebugCmdLine::process(ccCommandLineInterface& cmd)
{
	cmd.print("[DEBUG]");

	cmd.print("******************************************");
	cmd.print("Number of clouds: " + QString::number(cmd.clouds().size()));
	cmd.print("Number of meshes: " + QString::number(cmd.meshes().size()));

	cmd.print("******************************************");
	const QStringList& arguments = cmd.arguments();
	cmd.print("Number of arguments: " + QString::number(arguments.size()));
	for (int i = 0; i < arguments.size(); ++i)
	{
		cmd.print(QString("Argument #%1: < %2 >").arg(i + 1).arg(arguments[i]));
	}

	cmd.print("******************************************");
	cmd.print("[Loading parameters]");
	cmd.print(QObject::tr("Global shift set: ") + (cmd.fileLoadingParams().coordinatesShiftEnabled ? "yes" : "no"));
	cmd.print(QObject::tr("Global shift: (%1, %2, %3)").arg(cmd.fileLoadingParams().coordinatesShift.x).arg(cmd.fileLoadingParams().coordinatesShift.y).arg(cmd.fileLoadingParams().coordinatesShift.z));

	cmd.print("******************************************");
	cmd.print("[Export parameters]");
	cmd.print("Cloud export format: " + cmd.cloudExportFormat());
	cmd.print("Mesh export format: " + cmd.meshExportFormat());
	cmd.print("Group export format: " + cmd.hierarchyExportFormat());

	cmd.print("******************************************");
	cmd.print("[Other parameters]");
	cmd.print(QObject::tr("Silent mode: ") + (cmd.silentMode() ? "ON" : "OFF"));
	cmd.print(QObject::tr("Auto save: ") + (cmd.autoSaveMode() ? "ON" : "OFF"));
	cmd.print(QObject::tr("Auto add timestamp: ") + (cmd.addTimestamp() ? "ON" : "OFF"));
	cmd.print(QObject::tr("Numerical precision: %1").arg(cmd.numericalPrecision()));

	return true;
}
