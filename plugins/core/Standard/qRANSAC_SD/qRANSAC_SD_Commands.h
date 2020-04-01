//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qRANSAC_SD                   #
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
//#                         COPYRIGHT: Chris S Brown                       #
//#                                                                        #
//##########################################################################

#ifndef RANSAC_PLUGIN_COMMANDS_HEADER
#define RANSAC_PLUGIN_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h"
#include <ccMesh.h>
#include <ccGenericMesh.h>

//Local
#include "qRANSAC_SD.h"

constexpr char COMMAND_RANSAC[] = "RANSAC";
constexpr char EPSILON_ABSOLUTE[] = "EPSILON_ABSOLUTE";
constexpr char EPSILON_PERCENTAGE_OF_SCALE[] = "EPSILON_PERCENTAGE_OF_SCALE";
constexpr char BITMAP_EPSILON_PERCENTAGE_OF_SCALE[] = "BITMAP_EPSILON_PERCENTAGE_OF_SCALE";
constexpr char BITMAP_EPSILON_ABSOLUTE[] = "BITMAP_EPSILON_ABSOLUTE";
constexpr char SUPPORT_POINTS[] = "SUPPORT_POINTS";
constexpr char MAX_NORMAL_DEV[] = "MAX_NORMAL_DEV";
constexpr char PROBABILITY[] = "PROBABILITY";
constexpr char ENABLE_PRIMITIVE[] = "ENABLE_PRIMITIVE";
constexpr char OUT_CLOUD_DIR[] = "OUT_CLOUD_DIR";
constexpr char OUT_MESH_DIR[] = "OUT_MESH_DIR";
constexpr char OUT_PAIR_DIR[] = "OUT_PAIR_DIR";
constexpr char OUT_GROUP_DIR[] = "OUT_GROUP_DIR";
constexpr char OUTPUT_INDIVIDUAL_PRIMITIVES[] = "OUTPUT_INDIVIDUAL_PRIMITIVES";
constexpr char OUTPUT_INDIVIDUAL_SUBCLOUDS[] = "OUTPUT_INDIVIDUAL_SUBCLOUDS";
constexpr char OUTPUT_INDIVIDUAL_PAIRED_CLOUD_PRIMITIVE[] = "OUTPUT_INDIVIDUAL_PAIRED_CLOUD_PRIMITIVE";
constexpr char OUTPUT_GROUPED[] = "OUTPUT_GROUPED";

constexpr char PRIM_PLANE[] = "PLANE";
constexpr char PRIM_SPHERE[] = "SPHERE";
constexpr char PRIM_CYLINDER[] = "CYLINDER";
constexpr char PRIM_CONE[] = "CONE";
constexpr char PRIM_TORUS[] = "TORUS";


struct CommandRANSAC : public ccCommandLineInterface::Command
{
	CommandRANSAC() : ccCommandLineInterface::Command(QObject::tr("RANSAC"), COMMAND_RANSAC) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		if (cmd.clouds().empty())
		{
			return cmd.error(QObject::tr("No point cloud to attempt RANSAC on (be sure to open one with \"-O [cloud filename]\" before \"-%2\")").arg(COMMAND_RANSAC));
		}
		qRansacSD::RansacParams params;
		QStringList paramNames = QStringList() << EPSILON_ABSOLUTE << EPSILON_PERCENTAGE_OF_SCALE <<
			BITMAP_EPSILON_PERCENTAGE_OF_SCALE << BITMAP_EPSILON_ABSOLUTE <<
			SUPPORT_POINTS << MAX_NORMAL_DEV << PROBABILITY << ENABLE_PRIMITIVE <<
			OUT_CLOUD_DIR << OUT_MESH_DIR << OUT_GROUP_DIR << OUT_PAIR_DIR << OUTPUT_INDIVIDUAL_PRIMITIVES <<
			OUTPUT_INDIVIDUAL_SUBCLOUDS << OUTPUT_GROUPED << OUTPUT_INDIVIDUAL_PAIRED_CLOUD_PRIMITIVE;
		QStringList primitiveNames = QStringList() << PRIM_PLANE << PRIM_SPHERE << PRIM_CYLINDER << PRIM_CONE << PRIM_TORUS;
		QString outputCloudsDir;
		QString outputMeshesDir;
		QString outputPairDir;
		QString outputGroupDir;
		bool outputIndividualClouds = false;
		bool outputIndividualPrimitives = false;
		bool outputIndividualPairs = false;
		bool outputGrouped = false;



		float epsilonABS = -1.0f;
		float epsilonPercentage = -1.0f;
		float bitmapEpsilonABS = -1.0f;
		float bitmapEpsilonPercentage = -1.0f;
		params.epsilon = -1.0f;
		params.bitmapEpsilon = -1.0f;

		for (unsigned char k = 0; k < 5; ++k)
		{
			params.primEnabled[k] = false;
		}

		if (!cmd.arguments().empty())
		{

			QString param = cmd.arguments().takeFirst().toUpper();
			while (paramNames.contains(param))
			{
				cmd.print(QObject::tr("\t%1 : %2").arg(COMMAND_RANSAC, param));
				if (param == EPSILON_ABSOLUTE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, EPSILON_ABSOLUTE));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for epsilon!");
					}
					cmd.print(QObject::tr("\tEpsilon : %1").arg(val));
					epsilonABS = val;
				}
				else if (param == EPSILON_PERCENTAGE_OF_SCALE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, EPSILON_PERCENTAGE_OF_SCALE));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok || (val <= 0.0f || val >= 1.0f))
					{
						return cmd.error("Invalid number for epsilon percentage must be a float greater than 0.0 and less than 1.0!");
					}
					cmd.print(QObject::tr("\tEpsilon : %1").arg(val));
					epsilonPercentage = val;
				}
				else if (param == BITMAP_EPSILON_ABSOLUTE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, BITMAP_EPSILON_ABSOLUTE));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for Bitmap epsilon!");
					}
					cmd.print(QObject::tr("\tBitmap Epsilon : %1").arg(val));
					bitmapEpsilonABS = val;
				}
				else if (param == BITMAP_EPSILON_PERCENTAGE_OF_SCALE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, BITMAP_EPSILON_PERCENTAGE_OF_SCALE));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok || (val <= 0.0f || val >= 1.0f))
					{
						return cmd.error("Invalid number for Bitmap epsilon must be a float greater than 0.0 and less than 1.0!!");
					}
					cmd.print(QObject::tr("\tBitmap Epsilon : %1").arg(val));
					bitmapEpsilonPercentage = val;
				}
				else if (param == SUPPORT_POINTS)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, SUPPORT_POINTS));
					}
					bool ok;
					unsigned count = cmd.arguments().takeFirst().toUInt(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number of for support points!");
					}
					cmd.print(QObject::tr("\tsupport points: %1").arg(count));
					params.supportPoints = count;
				}
				else if (param == MAX_NORMAL_DEV)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, MAX_NORMAL_DEV));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for Max Normal Deviation!");
					}
					cmd.print(QObject::tr("\tMax Normal Deviation : %1").arg(val));
					params.maxNormalDev_deg = val;
				}
				else if (param == PROBABILITY)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 %2\"").arg(COMMAND_RANSAC, PROBABILITY));
					}
					bool ok;
					float val = cmd.arguments().takeFirst().toFloat(&ok);
					if (!ok)
					{
						return cmd.error("Invalid number for Probability!");
					}
					cmd.print(QObject::tr("\tProbability : %1").arg(val));
					params.probability = val;
				}
				else if (param == OUT_CLOUD_DIR)
				{
					if (!makePathIfPossible(cmd, param, &outputCloudsDir, &outputIndividualClouds))
					{
						return false;
					}
				}
				else if (param == OUT_MESH_DIR)
				{
					if (!makePathIfPossible(cmd, param, &outputMeshesDir, &outputIndividualPrimitives))
					{
						return false;
					}
				}
				else if (param == OUT_GROUP_DIR)
				{
					if (!makePathIfPossible(cmd, param, &outputGroupDir, &outputGrouped))
					{
						return false;
					}
				}
				else if (param == OUT_PAIR_DIR)
				{
					if (!makePathIfPossible(cmd, param, &outputPairDir, &outputIndividualPairs))
					{
						return false;
					}				
				}
				else if (param == OUTPUT_INDIVIDUAL_SUBCLOUDS)
				{
					outputIndividualClouds = true;
				}
				else if (param == OUTPUT_INDIVIDUAL_PRIMITIVES)
				{
					outputIndividualPrimitives = true;
				}
				else if (param == OUTPUT_INDIVIDUAL_PAIRED_CLOUD_PRIMITIVE)
				{
					outputIndividualPairs = true;
				}
				else if (param == OUTPUT_GROUPED)
				{
					outputGrouped = true;
				}
				else if (param == ENABLE_PRIMITIVE)
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: primitive type after \"-%1 %2\"").arg(COMMAND_RANSAC, ENABLE_PRIMITIVE));
					}
					QString prim = cmd.arguments().takeFirst().toUpper();
					unsigned primCount = 0;
					while (primitiveNames.contains(prim))
					{
						if (prim == PRIM_PLANE)
						{
							params.primEnabled[qRansacSD::RPT_PLANE] = true;
							primCount++;
						}
						else if (prim == PRIM_SPHERE)
						{
							params.primEnabled[qRansacSD::RPT_SPHERE] = true;
							primCount++;
						}
						else if (prim == PRIM_CYLINDER)
						{
							params.primEnabled[qRansacSD::RPT_CYLINDER] = true;
							primCount++;
						}
						else if (prim == PRIM_CONE)
						{
							params.primEnabled[qRansacSD::RPT_CONE] = true;
							primCount++;
						}
						else if (prim == PRIM_TORUS)
						{
							params.primEnabled[qRansacSD::RPT_TORUS] = true;
							primCount++;
						}
						if (cmd.arguments().empty())
						{
							break;
						}
						prim = cmd.arguments().takeFirst().toUpper();
					}
					if (primCount > 0)
					{
						if (!primitiveNames.contains(prim))
						{
							cmd.arguments().push_front(prim);
						}
					}
					else
					{
						return cmd.error(QObject::tr("No valid parameter: primitive type after \"-%1 %2\"").arg(COMMAND_RANSAC, ENABLE_PRIMITIVE));
					}
				}
				if (cmd.arguments().empty())
				{
					break;
				}
				param = cmd.arguments().takeFirst().toUpper();
			}
			if (!paramNames.contains(param))
			{
				cmd.arguments().push_front(param);
			}
		}
		unsigned char primCount = 0;
		for (unsigned char k = 0; k < 5; ++k)
		{
			primCount += static_cast<unsigned>(params.primEnabled[k]);
		}
		if (primCount == 0)
		{
			cmd.print(QObject::tr("\tDefault Shape Search == %1").arg(PRIM_PLANE));
			params.primEnabled[qRansacSD::RPT_PLANE] = true;
		}
		if (!outputIndividualClouds && !outputIndividualPrimitives && !outputGrouped && !outputIndividualPairs)
		{
			cmd.print(QObject::tr("\tDefault output == %1").arg(OUTPUT_GROUPED));
			outputGrouped = true;
		}

		for (CLCloudDesc clCloud : cmd.clouds())
		{

			CCVector3 bbMin, bbMax;
			clCloud.pc->getBoundingBox(bbMin, bbMax);
			CCVector3 diff = bbMax - bbMin;
			float scale = std::max(std::max(diff[0], diff[1]), diff[2]);
			if (epsilonPercentage > 0.0f)
			{
				params.epsilon = (epsilonPercentage * scale);
			}
			if (bitmapEpsilonPercentage > 0.0f)
			{
				params.bitmapEpsilon = (bitmapEpsilonPercentage * scale);
			}
			if (epsilonABS > 0.0f)
			{
				params.epsilon = epsilonABS;
			}
			if (bitmapEpsilonABS > 0.0f)
			{
				params.bitmapEpsilon = bitmapEpsilonABS;
			}
			if (params.epsilon < 0.0f)
			{
				params.epsilon = (0.005f * scale);
			}
			if (params.bitmapEpsilon < 0.0f)
			{
				params.bitmapEpsilon = (0.01f * scale);
			}

			ccHObject* group = qRansacSD::executeRANSAC(clCloud.pc, params, cmd.silentMode());
			
			if (group)
			{
				if (outputGrouped)
				{
					CLGroupDesc clGroup(group, clCloud.basename + "_" + clCloud.pc->getName() + "_RANSAC_DETECTED_SHAPES", outputGroupDir != "" ? outputGroupDir : clCloud.path);
					QString errorStr = cmd.exportEntity(clGroup, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceHierarchy);
					if (!errorStr.isEmpty())
					{
						cmd.warning(errorStr);
					}
				}
				if (outputIndividualPrimitives || outputIndividualClouds || outputIndividualPairs)
				{
					ccHObject::Container meshGroup;
					unsigned meshCount = group->filterChildren(meshGroup, true, CC_TYPES::MESH);
					unsigned planeCount = 1;
					unsigned sphereCount = 1;
					unsigned cylinderCount = 1;
					unsigned coneCount = 1;
					unsigned torusCount = 1;

					for (auto meshObj : meshGroup)
					{
						auto mesh = ccHObjectCaster::ToMesh(meshObj);
						if (mesh)
						{
							QString suffix;
							if (meshObj->isA(CC_TYPES::PLANE))
							{
								suffix = QString("_PLANE_%1").arg(planeCount, 4, 10, QChar('0'));
								planeCount++;
							}
							else if (meshObj->isA(CC_TYPES::SPHERE))
							{
								suffix = QString("_SPHERE_%1").arg(sphereCount, 4, 10, QChar('0'));
								sphereCount++;
							}
							else if (meshObj->isA(CC_TYPES::CYLINDER))
							{
								suffix = QString("_CYLINDER_%1").arg(cylinderCount, 4, 10, QChar('0'));
								cylinderCount++;
							}
							else if (meshObj->isA(CC_TYPES::CONE))
							{
								suffix = QString("_CONE_%1").arg(coneCount, 4, 10, QChar('0'));
								coneCount++;
							}
							else if (meshObj->isA(CC_TYPES::TORUS))
							{
								suffix = QString("_TORUS_%1").arg(torusCount, 4, 10, QChar('0'));
								torusCount++;
							}
							auto cld = ccHObjectCaster::ToPointCloud(mesh->getParent());
							if (outputIndividualPairs)
							{
								CLGroupDesc clPair(cld, clCloud.basename + "_" + clCloud.pc->getName() + suffix + QString("_pair"), outputPairDir != "" ? outputPairDir : clCloud.path);
								QString errorStr = cmd.exportEntity(clPair, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceHierarchy);
								if (!errorStr.isEmpty())
								{
									cmd.warning(errorStr);
								}							
							}
							if (cld)
							{
								cld->detachChild(mesh);
							}
							CLCloudDesc clCloudp(cld, clCloud.basename + "_" + clCloud.pc->getName() + suffix + QString("_cloud"), outputCloudsDir != "" ? outputCloudsDir : clCloud.path);
							cmd.clouds().push_back(clCloudp);
							CLMeshDesc clMesh(mesh, clCloud.basename + "_" + clCloud.pc->getName() + suffix, outputMeshesDir != "" ? outputMeshesDir : clCloud.path);
							cmd.meshes().push_back(clMesh);
							if (outputIndividualClouds)
							{
								QString errorStr = cmd.exportEntity(clCloudp);
								if (!errorStr.isEmpty())
								{
									cmd.warning(errorStr);
								}
							}
							if (outputIndividualPrimitives)
							{
								QString errorStr = cmd.exportEntity(clMesh);
								if (!errorStr.isEmpty())
								{
									cmd.warning(errorStr);
								}
							}
						}
					}
				}
			}
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(clCloud); // The original cloud may have had normals added
				if (!errorStr.isEmpty())
				{
					cmd.warning(errorStr);
				}
			}
		}


		return true;
	}

	bool makePathIfPossible(ccCommandLineInterface& cmd, const QString &param, QString *outputPath, bool *performOutput)
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QObject::tr("\nMissing parameter: Directory after \"-%1 %2\"").arg(COMMAND_RANSAC, param));
		}
		QString arg = cmd.arguments().takeFirst();
		QDir dir(arg);
		bool pathExists = dir.exists();
		if (!pathExists)
		{
			cmd.print(QObject::tr("\n%1 Does not exist\tcreating path").arg(arg));
			pathExists = dir.mkpath(arg);
		}
		if (pathExists)
		{
			*outputPath = dir.cleanPath(arg);
			cmd.print(QObject::tr("\t%1 : %2").arg(param, *outputPath));
			*performOutput = true;
		}
		else
		{
			cmd.print(QObject::tr("\n%1 path could not be created, skipping %2").arg(arg, param));
		}
		return true;
	}

};

#endif //RANSAC_PLUGIN_COMMANDS_HEADER
