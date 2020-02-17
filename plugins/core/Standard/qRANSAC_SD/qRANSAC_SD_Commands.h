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

#ifndef M3C2_PLUGIN_COMMANDS_HEADER
#define M3C2_PLUGIN_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h"
#include <ccMesh.h>
#include <ccGenericMesh.h>

//Local
#include "qRANSAC_SD.h"

static const char COMMAND_RANSAC[] = "RANSAC";

struct CommandRANSAC : public ccCommandLineInterface::Command
{
	CommandRANSAC() : ccCommandLineInterface::Command("RANSAC", COMMAND_RANSAC) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		if (cmd.clouds().empty())
		{
			return cmd.error(QObject::tr("No point cloud to attempt RANSAC on (be sure to open one with \"-O [cloud filename]\" before \"-%2\")").arg(COMMAND_RANSAC));
		}
		qRansacSD::RansacParams params;
		QStringList paramNames = QStringList() << "EPSILON_ABSOLUTE" << "EPSILON_PERCENTAGE_OF_SCALE" << "BITMAP_EPSILON_PERCENTAGE_OF_SCALE" << "BITMAP_EPSILON_ABSOLUTE" << "SUPPORT_POINTS" << "MAX_NORMAL_DEV" << "PROBABILITY" << "ENABLE_PRIMITIVE";
		QStringList primitiveNames = QStringList() << "PLANE" << "SPHERE" << "CYLINDER" << "CONE" << "TORUS";
		float epsilonABS = -1.0f;
		float epsilonPercentage = -1.0f;
		float bitmapEpsilonABS = -1.0f;
		float bitmapEpsilonPercentage = -1.0f;
		params.epsilon = -1.0f;
		params.bitmapEpsilon = -1.0f;

		for (unsigned char k = 0; k < 5; ++k)
			params.primEnabled[k] = false;
		
		if (!cmd.arguments().empty())
		{

			QString param = cmd.arguments().takeFirst().toUpper();
			cmd.print(QObject::tr("\tMethod: ") + param);
			while (paramNames.contains(param))
			{
				if (param == "EPSILON_ABSOLUTE")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 EPSILON_ABSOLUTE\"").arg(COMMAND_RANSAC));
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
				else if (param == "EPSILON_PERCENTAGE_OF_SCALE")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 EPSILON_PERCENTAGE_OF_SCALE\"").arg(COMMAND_RANSAC));
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
				else if (param == "BITMAP_EPSILON_ABSOLUTE")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 BITMAP_EPSILON_ABSOLUTE\"").arg(COMMAND_RANSAC));
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
				else if (param == "BITMAP_EPSILON_PERCENTAGE_OF_SCALE")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 BITMAP_EPSILON_PERCENTAGE_OF_SCALE\"").arg(COMMAND_RANSAC));
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
				else if (param == "SUPPORT_POINTS")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 SUPPORT_POINTS\"").arg(COMMAND_RANSAC));
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
				else if (param == "MAX_NORMAL_DEV")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 MAX_NORMAL_DEV\"").arg(COMMAND_RANSAC));
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
				else if (param == "PROBABILITY")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: number after \"-%1 PROBABILITY\"").arg(COMMAND_RANSAC));
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
				else if (param == "ENABLE_PRIMITIVE")
				{
					if (cmd.arguments().empty())
					{
						return cmd.error(QObject::tr("Missing parameter: primitive type after \"-%1 ENABLE_PRIMITIVE\"").arg(COMMAND_RANSAC));
					}
					QString prim = cmd.arguments().takeFirst().toUpper();
					unsigned primCount = 0;
					while (primitiveNames.contains(prim))
					{
						if (prim == "PLANE")
						{
							params.primEnabled[qRansacSD::RPT_PLANE] = true;
							primCount++;
						}
						else if (prim == "SPHERE")
						{
							params.primEnabled[qRansacSD::RPT_SPHERE] = true;
							primCount++;
						}
						else if (prim == "CYLINDER")
						{
							params.primEnabled[qRansacSD::RPT_CYLINDER] = true;
							primCount++;
						}
						else if (prim == "CONE")
						{
							params.primEnabled[qRansacSD::RPT_CONE] = true;
							primCount++;
						}
						else if (prim == "TORUS")
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
						return cmd.error(QObject::tr("No valid parameter: primitive type after \"-%1 ENABLE_PRIMITIVE\"").arg(COMMAND_RANSAC));
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
			primCount += (unsigned)params.primEnabled[k];
		if (primCount == 0)
		{
			params.primEnabled[qRansacSD::RPT_PLANE] = true;
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
			if (cmd.autoSaveMode())
			{
				QString errorStr = cmd.exportEntity(clCloud); // The original cloud may have had normals added
				if (!errorStr.isEmpty())
					cmd.warning(errorStr);
			}
			if (group)
			{
				ccHObject::Container meshGroup;
				unsigned meshCount = group->filterChildren(meshGroup, true, CC_TYPES::MESH);
				for (auto meshObj : meshGroup)
				{
					auto mesh = ccHObjectCaster::ToMesh(meshObj);
					CLMeshDesc clMesh(mesh, clCloud.basename + mesh->getName(), clCloud.path);
					cmd.meshes().push_back(clMesh);
					if (cmd.autoSaveMode())
					{
						QString errorStr = cmd.exportEntity(clMesh); // The original cloud may have had normals added
						if (!errorStr.isEmpty())
							cmd.warning(errorStr);
					}
				}
			}
		}


		return true;
	}
};

#endif //RANSAC_PLUGIN_COMMANDS_HEADER
