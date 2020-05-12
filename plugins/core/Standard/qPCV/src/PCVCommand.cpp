#include "PCVCommand.h"
#include "PCV.h"
#include "qPCV.h"

//qCC_db
#include <ccColorScalesManager.h>
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

constexpr char CC_PCV_FIELD_LABEL_NAME[] = "Illuminance (PCV)";

constexpr char COMMAND_PCV[] = "PCV";
constexpr char COMMAND_PCV_N_RAYS[] = "N_RAYS";
constexpr char COMMAND_PCV_IS_CLOSED[] = "IS_CLOSED";
constexpr char COMMAND_PCV_180[] = "180";
constexpr char COMMAND_PCV_RESOLUTION[] = "RESOLUTION";

PCVCommand::PCVCommand()
	: Command("PCV", COMMAND_PCV)
{
}

bool PCVCommand::Process(	const ccHObject::Container& candidates,
							const std::vector<CCVector3>& rays,
							bool meshIsClosed,
							unsigned resolution,
							ccProgressDialog* progressDlg/*=nullptr*/,
							ccMainAppInterface* app/*=nullptr*/)
{
	size_t count = 0;
	size_t errorCount = 0;

	for (ccHObject* obj : candidates)
	{
		ccPointCloud* cloud = nullptr;
		ccGenericMesh* mesh = nullptr;
		QString objName("unknown");

		assert(obj);
		if (obj->isA(CC_TYPES::POINT_CLOUD))
		{
			//we need a real point cloud
			cloud = ccHObjectCaster::ToPointCloud(obj);
			objName = cloud->getName();
		}
		else if (obj->isKindOf(CC_TYPES::MESH))
		{
			mesh = ccHObjectCaster::ToGenericMesh(obj);
			cloud = ccHObjectCaster::ToPointCloud(mesh->getAssociatedCloud());
			objName = mesh->getName();
		}

		if (cloud == nullptr)
		{
			assert(false);
			if (app)
				app->dispToConsole(QObject::tr("Invalid object type"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			++errorCount;
			continue;
		}

		//we get the PCV field if it already exists
		int sfIdx = cloud->getScalarFieldIndexByName(CC_PCV_FIELD_LABEL_NAME);

		//otherwise we create it
		if (sfIdx < 0)
		{
			sfIdx = cloud->addScalarField(CC_PCV_FIELD_LABEL_NAME);
		}

		if (sfIdx < 0)
		{
			if (app)
				app->dispToConsole("Couldn't allocate a new scalar field for computing PCV field! Try to free some memory...", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
		cloud->setCurrentScalarField(sfIdx);

		QString objNameForPorgressDialog = objName;
		if (candidates.size() > 1)
		{
			objNameForPorgressDialog += QStringLiteral("(%1/%2)").arg(++count).arg(candidates.size());
		}

		bool wasEnabled = obj->isEnabled();
		bool wasVisible = obj->isVisible();
		obj->setEnabled(true);
		obj->setVisible(true);
		bool success = PCV::Launch(rays, cloud, mesh, meshIsClosed, resolution, resolution, progressDlg, objNameForPorgressDialog);
		obj->setEnabled(wasEnabled);
		obj->setVisible(wasVisible);

		if (!success)
		{
			cloud->deleteScalarField(sfIdx);
			if (app)
				app->dispToConsole(QObject::tr("An error occurred during entity '%1' illumination!").arg(objName), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			++errorCount;
		}
		else
		{
			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
			if (sf)
			{
				sf->computeMinAndMax();
				cloud->setCurrentDisplayedScalarField(sfIdx);
				sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::GREY));
				if (obj->hasNormals() && obj->normalsShown())
				{
					if (app)
						app->dispToConsole(QObject::tr("Entity '%1' normals have been automatically disabled").arg(objName), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
				obj->showNormals(false);
				obj->showSF(true);
				if (obj != cloud)
				{
					cloud->showSF(true);
				}
				obj->prepareDisplayForRefresh_recursive();
			}
			else
			{
				assert(false);
			}
		}

		if (progressDlg && progressDlg->wasCanceled())
		{
			if (app)
				app->dispToConsole(QObject::tr("Process has been cancelled by the user"), ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			++errorCount;
			break;
		}
	}

	return (errorCount == 0);
}

bool PCVCommand::process(ccCommandLineInterface& cmd)
{
	cmd.print("[PCV]");

	if (cmd.meshes().empty() && cmd.clouds().empty())
	{
		return cmd.error(qPCV::tr("No entity is loaded."));
	}

	// Initialize to match PCV::Launch defaults
	unsigned rayCount = 256;
	bool meshIsClosed = false;
	bool mode360 = true;
	unsigned resolution = 1024;

	while (!cmd.arguments().empty())
	{
		const QString& arg = cmd.arguments().front();
		if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_IS_CLOSED))
		{
			cmd.arguments().pop_front();
			meshIsClosed = true;
		}

		// PCV::Launch mode360 defaults to true. To make absence / presence of command map to false / true,
		// the command is named "180," and is the inverse of mode360.
		else if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_180))
		{
			cmd.arguments().pop_front();
			mode360 = false;
		}
		else if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_N_RAYS))
		{
			cmd.arguments().pop_front();
			bool conversionOk = false;
			rayCount = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_PCV_N_RAYS));
			}
		}
		else if (ccCommandLineInterface::IsCommand(arg, COMMAND_PCV_RESOLUTION))
		{
			cmd.arguments().pop_front();
			bool conversionOk = false;
			resolution = cmd.arguments().takeFirst().toUInt(&conversionOk);
			if (!conversionOk)
			{
				return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_PCV_RESOLUTION));
			}
		}
		else
		{
			break;
		}
	}

	//generates light directions
	std::vector<CCVector3> rays;
	if (!PCV::GenerateRays(rayCount, rays, mode360))
	{
		return cmd.error(QObject::tr("Failed to generate the set of rays"));
	}

	ccProgressDialog pcvProgressCb(true);
	pcvProgressCb.setAutoClose(false);

	ccHObject::Container candidates;
	try
	{
		candidates.reserve(cmd.clouds().size() + cmd.meshes().size());
	}
	catch (const std::bad_alloc)
	{
		return cmd.error(QObject::tr("Not enough memory"));
	}

	for (CLCloudDesc& desc : cmd.clouds())
		candidates.push_back(desc.pc);
	for (CLMeshDesc& desc : cmd.meshes())
		candidates.push_back(desc.mesh);

	if (!Process(candidates, rays, meshIsClosed, resolution, &pcvProgressCb, nullptr))
	{
		return cmd.error(QObject::tr("Process failed"));
	}

	for (CLCloudDesc& desc : cmd.clouds())
	{
		desc.basename += QString("_PCV");

		//save output
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(desc);
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	for (CLMeshDesc& desc : cmd.meshes())
	{
		desc.basename += QString("_PCV");

		//save output
		if (cmd.autoSaveMode())
		{
			QString errorStr = cmd.exportEntity(desc);
			if (!errorStr.isEmpty())
			{
				return cmd.error(errorStr);
			}
		}
	}

	return true;
}
