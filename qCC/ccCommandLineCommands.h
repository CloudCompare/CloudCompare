#ifndef COMMAND_LINE_COMMANDS_HEADER
#define COMMAND_LINE_COMMANDS_HEADER

#include "ccCommandLineInterface.h"

#include <QStringList>

struct CommandChangeOutputFormat : public ccCommandLineInterface::Command
{
	CommandChangeOutputFormat(const QString& name, const QString& keyword);

	QString getFileFormatFilter(ccCommandLineInterface& cmd, QString& defaultExt);
};

struct CommandChangeCloudOutputFormat : public CommandChangeOutputFormat
{
	CommandChangeCloudOutputFormat();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandChangeMeshOutputFormat : public CommandChangeOutputFormat
{
	CommandChangeMeshOutputFormat();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandChangeHierarchyOutputFormat : public CommandChangeOutputFormat
{
	CommandChangeHierarchyOutputFormat();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandLoad : public ccCommandLineInterface::Command
{
	CommandLoad();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandLoadCommandFile : public ccCommandLineInterface::Command
{
	CommandLoadCommandFile();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandClearNormals : public ccCommandLineInterface::Command
{
	CommandClearNormals();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandOctreeNormal : public ccCommandLineInterface::Command
{
	CommandOctreeNormal();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandInvertNormal : public ccCommandLineInterface::Command
{
	CommandInvertNormal();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandConvertNormalsToDipAndDipDir : public ccCommandLineInterface::Command
{
	CommandConvertNormalsToDipAndDipDir();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandConvertNormalsToSFs : public ccCommandLineInterface::Command
{
	CommandConvertNormalsToSFs();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandConvertNormalsToHSV : public ccCommandLineInterface::Command
{
	CommandConvertNormalsToHSV();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSubsample : public ccCommandLineInterface::Command
{
	CommandSubsample();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandExtractCCs : public ccCommandLineInterface::Command
{
	CommandExtractCCs();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandCurvature : public ccCommandLineInterface::Command
{
	CommandCurvature();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandApproxDensity : public ccCommandLineInterface::Command
{
	CommandApproxDensity();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandDensity : public ccCommandLineInterface::Command
{
	CommandDensity();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFGradient : public ccCommandLineInterface::Command
{
	CommandSFGradient();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRoughness : public ccCommandLineInterface::Command
{
	CommandRoughness();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandApplyTransformation : public ccCommandLineInterface::Command
{
	CommandApplyTransformation();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandDropGlobalShift : public ccCommandLineInterface::Command
{
	CommandDropGlobalShift();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFColorScale : public ccCommandLineInterface::Command
{
	CommandSFColorScale();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFConvertToRGB : public ccCommandLineInterface::Command
{
	CommandSFConvertToRGB();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRGBConvertToSF : public ccCommandLineInterface::Command
{
	CommandRGBConvertToSF();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandFilterBySFValue : public ccCommandLineInterface::Command
{
	CommandFilterBySFValue();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandComputeMeshVolume : public ccCommandLineInterface::Command
{
	CommandComputeMeshVolume();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandMergeMeshes : public ccCommandLineInterface::Command
{
	CommandMergeMeshes();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandMergeClouds : public ccCommandLineInterface::Command
{
	CommandMergeClouds();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSetActiveSF : public ccCommandLineInterface::Command
{
	CommandSetActiveSF();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSetGlobalShift : public ccCommandLineInterface::Command
{
	CommandSetGlobalShift();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRemoveAllSFs : public ccCommandLineInterface::Command
{
	CommandRemoveAllSFs();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRemoveSF : public ccCommandLineInterface::Command
{
	CommandRemoveSF();

	bool process(ccCommandLineInterface& cmd) override;

	bool removeSF(int sfIndex, ccPointCloud& pc);
};

struct CommandRemoveRGB : public ccCommandLineInterface::Command
{
	CommandRemoveRGB();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRemoveNormals : public ccCommandLineInterface::Command
{
	CommandRemoveNormals();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRemoveScanGrids : public ccCommandLineInterface::Command
{
	CommandRemoveScanGrids();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRemoveSensors : public ccCommandLineInterface::Command
{
	CommandRemoveSensors();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandMatchBBCenters : public ccCommandLineInterface::Command
{
	CommandMatchBBCenters();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandMatchBestFitPlane : public ccCommandLineInterface::Command
{
	CommandMatchBestFitPlane();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandOrientNormalsMST : public ccCommandLineInterface::Command
{
	CommandOrientNormalsMST();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSORFilter : public ccCommandLineInterface::Command
{
	CommandSORFilter();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandNoiseFilter : public ccCommandLineInterface::Command
{
	CommandNoiseFilter();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRemoveDuplicatePoints : public ccCommandLineInterface::Command
{
	CommandRemoveDuplicatePoints();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandExtractVertices : public ccCommandLineInterface::Command
{
	CommandExtractVertices();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandFlipTriangles : public ccCommandLineInterface::Command
{
	CommandFlipTriangles();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSampleMesh : public ccCommandLineInterface::Command
{
	CommandSampleMesh();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandCompressFWF : public ccCommandLineInterface::Command
{
	CommandCompressFWF();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandCrop : public ccCommandLineInterface::Command
{
	CommandCrop();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandCoordToSF : public ccCommandLineInterface::Command
{
	CommandCoordToSF();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFToCoord : public ccCommandLineInterface::Command
{
	CommandSFToCoord();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandCrop2D : public ccCommandLineInterface::Command
{
	CommandCrop2D();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandColorBanding : public ccCommandLineInterface::Command
{
	CommandColorBanding();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandColorLevels : public ccCommandLineInterface::Command
{
	CommandColorLevels();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandDist : public ccCommandLineInterface::Command
{
	CommandDist(bool cloud2meshDist, const QString& name, const QString& keyword);

	bool process(ccCommandLineInterface& cmd) override;

	bool m_cloud2meshDist;
};

struct CommandC2MDist : public CommandDist
{
	CommandC2MDist();
};

struct CommandC2CDist : public CommandDist
{
	CommandC2CDist();
};

struct CommandCPS : public ccCommandLineInterface::Command
{
	CommandCPS();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandStatTest : public ccCommandLineInterface::Command
{
	CommandStatTest();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandDelaunayTri : public ccCommandLineInterface::Command
{
	CommandDelaunayTri();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFArithmetic : public ccCommandLineInterface::Command
{
	CommandSFArithmetic();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFOperation : public ccCommandLineInterface::Command
{
	CommandSFOperation();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFOperationSF : public ccCommandLineInterface::Command
{
	CommandSFOperationSF();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFInterpolation : public ccCommandLineInterface::Command
{
	CommandSFInterpolation();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandColorInterpolation : public ccCommandLineInterface::Command
{
	CommandColorInterpolation();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandFilter : public ccCommandLineInterface::Command
{
	CommandFilter();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandRenameEntities : public ccCommandLineInterface::Command
{
	CommandRenameEntities();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFRename : public ccCommandLineInterface::Command
{
	CommandSFRename();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFAddConst : public ccCommandLineInterface::Command
{
	CommandSFAddConst();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSFAddId : public ccCommandLineInterface::Command
{
	CommandSFAddId();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandICP : public ccCommandLineInterface::Command
{
	CommandICP();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandChangePLYExportFormat : public ccCommandLineInterface::Command
{
	CommandChangePLYExportFormat();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandForceNormalsComputation : public ccCommandLineInterface::Command
{
	CommandForceNormalsComputation();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSave : public ccCommandLineInterface::Command
{
	CommandSave(const QString& name, const QString& keyword);

	static bool ParseFileNames(ccCommandLineInterface& cmd, QStringList& fileNames);

	static void SetFileDesc(CLEntityDesc& desc, const QString& fileName);
};

struct CommandSaveClouds : public CommandSave
{
	CommandSaveClouds();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSaveMeshes : public CommandSave
{
	CommandSaveMeshes();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandAutoSave : public ccCommandLineInterface::Command
{
	CommandAutoSave();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandLogFile : public ccCommandLineInterface::Command
{
	CommandLogFile();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSelectEntities : public ccCommandLineInterface::Command
{
	CommandSelectEntities();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandClear : public ccCommandLineInterface::Command
{
	CommandClear();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandClearClouds : public ccCommandLineInterface::Command
{
	CommandClearClouds();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandPopClouds : public ccCommandLineInterface::Command
{
	CommandPopClouds();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandClearMeshes : public ccCommandLineInterface::Command
{
	CommandClearMeshes();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandPopMeshes : public ccCommandLineInterface::Command
{
	CommandPopMeshes();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSetNoTimestamp : public ccCommandLineInterface::Command
{
	CommandSetNoTimestamp();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandMoment : public ccCommandLineInterface::Command
{
	CommandMoment();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandFeature : public ccCommandLineInterface::Command
{
	CommandFeature();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandDebugCmdLine : public ccCommandLineInterface::Command
{
	CommandDebugCmdLine();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandSetVerbosity : public ccCommandLineInterface::Command
{
	CommandSetVerbosity();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandComputeDistancesFromSensor : public ccCommandLineInterface::Command
{
	CommandComputeDistancesFromSensor();

	bool process(ccCommandLineInterface& cmd) override;
};

struct CommandComputeScatteringAngles : public ccCommandLineInterface::Command
{
	CommandComputeScatteringAngles();

	bool process(ccCommandLineInterface& cmd) override;
};

#endif // COMMAND_LINE_COMMANDS_HEADER
