#include "ccCommandCrossSection.h"
#include "ccCommandLineCommands.h"

#include <ccHObjectCaster.h>
#include <ccMesh.h>

#include "ccCropTool.h"

//to read the 'Cross Section' tool XML parameters file
#include <QXmlStreamReader>


constexpr char COMMAND_CROSS_SECTION[] = "CROSS_SECTION";

CommandCrossSection::CommandCrossSection()
    : ccCommandLineInterface::Command("Cross section", COMMAND_CROSS_SECTION)
{}

bool CommandCrossSection::process(ccCommandLineInterface &cmd)
{
	cmd.print("[CROSS SECTION]");

	static QString s_xmlCloudCompare = "CloudCompare";
	static QString s_xmlBoxThickness = "BoxThickness";
	static QString s_xmlBoxCenter = "BoxCenter";
	static QString s_xmlRepeatDim = "RepeatDim";
	static QString s_xmlRepeatGap = "RepeatGap";
	static QString s_xmlFilePath = "FilePath";
	static QString s_outputXmlFilePath = "OutputFilePath";

	//expected argument: XML file
	if (cmd.arguments().empty())
		return cmd.error(QString("Missing parameter: XML parameters file after \"-%1\"").arg(COMMAND_CROSS_SECTION));
	QString xmlFilename = cmd.arguments().takeFirst();

	//read the XML file
	CCVector3 boxCenter(0, 0, 0), boxThickness(0, 0, 0);
	bool repeatDim[3] = { false, false, false };
	double repeatGap = 0.0;
	bool inside = true;
	bool autoCenter = true;
	QString inputFilePath;
	QString outputFilePath;
	{
		QFile file(xmlFilename);
		if (!file.open(QFile::ReadOnly | QFile::Text))
		{
			return cmd.error(QString("Couldn't open XML file '%1'").arg(xmlFilename));
		}

		//read file content
		QXmlStreamReader stream(&file);

		//expected: CloudCompare
		if (!stream.readNextStartElement()
		        || stream.name() != s_xmlCloudCompare)
		{
			return cmd.error(QString("Invalid XML file (should start by '<%1>')").arg(s_xmlCloudCompare));
		}

		unsigned mandatoryCount = 0;
		while (stream.readNextStartElement()) //loop over the elements
		{
			if (stream.name() == s_xmlBoxThickness)
			{
				QXmlStreamAttributes attributes = stream.attributes();
				if (!readVector(attributes, boxThickness, s_xmlBoxThickness, cmd))
					return false;
				stream.skipCurrentElement();
				++mandatoryCount;
			}
			else if (stream.name() == s_xmlBoxCenter)
			{
				QXmlStreamAttributes attributes = stream.attributes();
				if (!readVector(attributes, boxCenter, s_xmlBoxCenter, cmd))
					return false;
				stream.skipCurrentElement();
				autoCenter = false;
			}
			else if (stream.name() == s_xmlRepeatDim)
			{
				QString itemValue = stream.readElementText();
				bool ok = false;
				int dim = itemValue.toInt(&ok);
				if (!ok || dim < 0 || dim > 2)
				{
					return cmd.error(QString("Invalid XML file (invalid value for '<%1>')").arg(s_xmlRepeatDim));
				}
				repeatDim[dim] = true;
			}
			else if (stream.name() == s_xmlRepeatGap)
			{
				QString itemValue = stream.readElementText();
				bool ok = false;
				repeatGap = itemValue.toDouble(&ok);
				if (!ok)
				{
					return cmd.error(QString("Invalid XML file (invalid value for '<%1>')").arg(s_xmlRepeatGap));
				}
			}
			else if (stream.name() == s_xmlFilePath)
			{
				inputFilePath = stream.readElementText();
				if (!QDir(inputFilePath).exists())
				{
					return cmd.error(QString("Invalid file path (directory pointed by '<%1>' doesn't exist)").arg(s_xmlFilePath));
				}
				//++mandatoryCount;
			}
			else if (stream.name() == s_outputXmlFilePath)
			{
				outputFilePath = stream.readElementText();
				if (!QDir(outputFilePath).exists())
				{
					return cmd.error(QString("Invalid output file path (directory pointed by '<%1>' doesn't exist)").arg(s_outputXmlFilePath));
				}
				//++mandatoryCount;
			}
			else
			{
				cmd.warning(QString("Unknown element: %1").arg(stream.name().toString()));
				stream.skipCurrentElement();
			}
		}

		if (mandatoryCount < 1 || (!repeatDim[0] && !repeatDim[1] && !repeatDim[2]))
		{
			return cmd.error(QString("Some mandatory elements are missing in the XML file (see documentation)"));
		}
	}

	//safety checks
	if (	boxThickness.x < ZERO_TOLERANCE
	        ||	boxThickness.y < ZERO_TOLERANCE
	        ||	boxThickness.z < ZERO_TOLERANCE
	        )
	{
		return cmd.error(QString("Invalid box thickness"));
	}

	CCVector3 repeatStep = boxThickness + CCVector3(repeatGap, repeatGap, repeatGap);
	if (	(repeatDim[0] && repeatStep.x < ZERO_TOLERANCE)
	        ||	(repeatDim[1] && repeatStep.y < ZERO_TOLERANCE)
	        ||	(repeatDim[2] && repeatStep.z < ZERO_TOLERANCE)
	        )
	{
		return cmd.error(QString("Repeat gap can't be equal or smaller than 'minus' box width"));
	}

	if (outputFilePath.isEmpty())
	{
		outputFilePath = inputFilePath;
	}

	int iterationCount = 1;

	//shall we load the entities?
	QStringList files;
	QDir dir;
	bool fromFiles = false;
	if (!inputFilePath.isEmpty())
	{
		//look for all files in the input directory
		dir = QDir(inputFilePath);
		assert(dir.exists());
		files = dir.entryList(QDir::Files);
		iterationCount = files.size();
		fromFiles = true;

		//remove any cloud or mesh in memory!
		cmd.removeClouds();
		cmd.removeMeshes();
	}

	for (int f = 0; f < iterationCount; ++f)
	{
		//shall we load files?
		QString filename;
		if (fromFiles)
		{
			assert(f < files.size());
			filename = dir.absoluteFilePath(files[f]);
			QFileInfo fileinfo(filename);
			if (!fileinfo.isFile() || fileinfo.suffix().toUpper() == "XML")
			{
				continue;
			}

			//let's try to load the file
			cmd.print(QString("Processing file: '%1'").arg(files[f]));

			bool result = false;
			{
				//hack: replace the current argument list by a fake 'load file' sequence
				QStringList realArguments = cmd.arguments();

				QStringList loadArguments;
				loadArguments << filename;
				cmd.arguments() = loadArguments;
				result = CommandLoad().process(cmd);

				//end of hack: restore the current argument list
				cmd.arguments() = realArguments;
			}

			if (!result)
			{
				cmd.warning("\tFailed to load file!");
				continue;
			}
		}
		else
		{
			assert(iterationCount == 1);
		}

		//repeat crop process on each file (or do it only once on the currently loaded entities)
		{
			ccHObject::Container entities;
			try
			{
				for (size_t i = 0; i < cmd.clouds().size(); ++i)
					entities.push_back(cmd.clouds()[i].pc);
				for (size_t j = 0; j < cmd.meshes().size(); ++j)
					entities.push_back(cmd.meshes()[j].mesh);
			}
			catch (const std::bad_alloc&)
			{
				return cmd.error("Not enough memory!");
			}

			for (size_t i = 0; i < entities.size(); ++i)
			{
				//check entity bounding-box
				ccHObject* ent = entities[i];
				ccBBox bbox = ent->getOwnBB();
				if (!bbox.isValid())
				{
					cmd.warning(QString("Entity '%1' has an invalid bounding-box!").arg(ent->getName()));
					continue;
				}

				//browse to/create a subdirectory with the (base) filename as name
				QString basename;
				if (fromFiles)
				{
					basename = QFileInfo(filename).baseName();
				}
				else
				{
					basename = i < cmd.clouds().size() ? cmd.clouds()[i].basename : cmd.meshes()[i - cmd.clouds().size()].basename;
				}

				if (entities.size() > 1)
					basename += QString("_%1").arg(i + 1);

				QDir outputDir(outputFilePath);
				if (outputFilePath.isEmpty())
				{
					if (fromFiles)
					{
						assert(false);
						outputDir = QDir::current();
					}
					else
					{
						outputDir = QDir(i < cmd.clouds().size() ? cmd.clouds()[i].path : cmd.meshes()[i - cmd.clouds().size()].path);
					}
				}

				assert(outputDir.exists());
				if (outputDir.cd(basename))
				{
					//if the directory already exists...
					cmd.warning(QString("Subdirectory '%1' already exists").arg(basename));
				}
				else if (outputDir.mkdir(basename))
				{
					outputDir.cd(basename);
				}
				else
				{
					cmd.warning(QString("Failed to create subdirectory '%1' (check access rights and base name validity!)").arg(basename));
					continue;
				}

				int toto = ceil(-0.4);
				int toto2 = ceil(-0.6);

				//place the initial box at the beginning of the entity bounding box
				CCVector3 C0 = autoCenter ? bbox.getCenter() : boxCenter;
				unsigned steps[3] = { 1, 1, 1 };
				for (unsigned d = 0; d < 3; ++d)
				{
					if (repeatDim[d])
					{
						PointCoordinateType boxHalfWidth = boxThickness.u[d] / 2;
						PointCoordinateType distToMinBorder = C0.u[d] - boxHalfWidth - bbox.minCorner().u[d];
						int stepsToMinBorder = static_cast<int>(ceil(distToMinBorder / repeatStep.u[d]));
						C0.u[d] -= stepsToMinBorder * repeatStep.u[d];

						PointCoordinateType distToMaxBorder = bbox.maxCorner().u[d] - C0.u[d] - boxHalfWidth;
						int stepsToMaxBoder = static_cast<int>(ceil(distToMaxBorder / repeatStep.u[d]) + 1);
						assert(stepsToMaxBoder >= 0);
						steps[d] = std::max<unsigned>(stepsToMaxBoder, 1);
					}
				}

				cmd.print(QString("Will extract up to (%1 x %2 x %3) = %4 sections").arg(steps[0]).arg(steps[1]).arg(steps[2]).arg(steps[0] * steps[1] * steps[2]));

				//now extract the slices
				for (unsigned dx = 0; dx < steps[0]; ++dx)
				{
					for (unsigned dy = 0; dy < steps[1]; ++dy)
					{
						for (unsigned dz = 0; dz < steps[2]; ++dz)
						{
							CCVector3 C = C0 + CCVector3(dx*repeatStep.x, dy*repeatStep.y, dz*repeatStep.z);
							ccBBox cropBox(C - boxThickness / 2, C + boxThickness / 2);
							cmd.print(QString("Box (%1;%2;%3) --> (%4;%5;%6)")
							          .arg(cropBox.minCorner().x).arg(cropBox.minCorner().y).arg(cropBox.minCorner().z)
							          .arg(cropBox.maxCorner().x).arg(cropBox.maxCorner().y).arg(cropBox.maxCorner().z)
							          );
							ccHObject* croppedEnt = ccCropTool::Crop(ent, cropBox, inside);
							if (croppedEnt)
							{
								QString outputBasename = basename + QString("_%1_%2_%3").arg(C.x).arg(C.y).arg(C.z);
								QString errorStr;
								//original entity is a cloud?
								if (i < cmd.clouds().size())
								{
									CLCloudDesc desc(static_cast<ccPointCloud*>(croppedEnt),
									                 outputBasename,
									                 outputDir.absolutePath(),
									                 entities.size() > 1 ? static_cast<int>(i) : -1);
									errorStr = cmd.exportEntity(desc);
								}
								else //otherwise it's a mesh
								{
									CLMeshDesc desc(static_cast<ccMesh*>(croppedEnt),
									                outputBasename,
									                outputDir.absolutePath(),
									                entities.size() > 1 ? static_cast<int>(i) : -1);
									errorStr = cmd.exportEntity(desc);
								}

								delete croppedEnt;
								croppedEnt = 0;

								if (!errorStr.isEmpty())
									return cmd.error(errorStr);
							}
						}
					}
				}
			}

			if (fromFiles)
			{
				//unload entities
				cmd.removeClouds();
				cmd.removeMeshes();
			}
		}
	}

	return true;
}

bool CommandCrossSection::readVector(const QXmlStreamAttributes &attributes, CCVector3 &P, QString element, const ccCommandLineInterface &cmd)
{
	if (attributes.size() < 3)
	{
		return cmd.error(QString("Invalid XML file (3 attributes expected for element '<%1>')").arg(element));
	}

	int count = 0;
	for (int i = 0; i < attributes.size(); ++i)
	{
		QString name = attributes[i].name().toString().toUpper();
		QString value = attributes[i].value().toString();

		bool ok = false;
		if (name == "X")
		{
			P.x = value.toDouble(&ok);
			++count;
		}
		else if (name == "Y")
		{
			P.y = value.toDouble(&ok);
			++count;
		}
		else if (name == "Z")
		{
			P.z = value.toDouble(&ok);
			++count;
		}
		else
		{
			ok = true;
		}

		if (!ok)
		{
			return cmd.error(QString("Invalid XML file (numerical attribute expected for attribute '%1' of element '<%2>')").arg(name, element));
		}
	}

	if (count < 3)
	{
		return cmd.error(QString("Invalid XML file (attributes 'X','Y' and 'Z' are mandatory for element '<%1>')").arg(element));
	}

	return true;
}
