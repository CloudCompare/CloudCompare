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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "profileLoader.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccPolyline.h>

//qCC_plugins
#include <ccMainAppInterface.h>

//Qt
#include <QFile>
#include <QTextStream>
#include <QFileInfo>

ccPolyline* ProfileLoader::Load(QString filename, CCVector3& origin, ccMainAppInterface* app/*=0*/)
{
	//load profile as a polyline
	QFile file(filename);
	assert(file.exists());
	if (!file.open(QFile::ReadOnly | QFile::Text))
	{
		if (app)
			app->dispToConsole(QString("Failed to open file for reading! Check access rights"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return nullptr;
	}

	QTextStream stream(&file);

	ccPolyline* polyline  = nullptr;

	bool error = false;
	for (unsigned n=0; n<1; ++n) //fake loop for easy break ;)
	{
		//read origin
		{
			QString headerLine = stream.readLine();
			if (headerLine.isEmpty() || !headerLine.startsWith("X"))
			{
				if (app)
					app->dispToConsole(QString("Malformed file (origin header expected on first line)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				error = true;
				break;
			}
			QString centerLine = stream.readLine();
			{
				QStringList tokens = centerLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
				bool validLine = false;
				if (tokens.size() == 3)
				{
					bool ok[3] = {false, false, false};
					origin.x = static_cast<PointCoordinateType>(tokens[0].toDouble(ok+0));
					origin.y = static_cast<PointCoordinateType>(tokens[1].toDouble(ok+1));
					origin.z = static_cast<PointCoordinateType>(tokens[2].toDouble(ok+2));
					validLine = ok[0] && ok[1] && ok[2];
				}
				if (!validLine)
				{
					if (app)
						app->dispToConsole(QString("Malformed file (origin coordinates expected on second line)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					error = true;
					break;
				}
			}
		}

		//read elevations
		{
			QString headerLine = stream.readLine();
			if (headerLine.isEmpty() || !headerLine.startsWith("R"))
			{
				if (app)
					app->dispToConsole(QString("Malformed file (radii/heights header expected on third line)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				error = true;
				break;
			}

			QString line = stream.readLine();
			std::vector< CCVector2d > points;
			while (!line.isEmpty())
			{
				QStringList tokens = line.simplified().split(QChar(' '), QString::SkipEmptyParts);
				if (tokens.size() < 2)
				{
					if (app)
						app->dispToConsole(QString("Malformed file (radius/height couple expected from the 4th line and afterwards)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					error = true;
					break;
				}

				CCVector2d P;
				P.x = tokens[0].toDouble(); //radius
				P.y = tokens[1].toDouble(); //height

				try
				{
					points.push_back(P);
				}
				catch (const std::bad_alloc&)
				{
					//not enough memory
					if (app)
						app->dispToConsole(QString("Not enough memory!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					error = true;
					break;
				}

				line = stream.readLine();
			} 

			//convert 2D points to polyline
			{
				unsigned count = static_cast<unsigned>(points.size());
				if (count > 1)
				{
					ccPointCloud* vertices = new ccPointCloud("vertices");
					polyline = new ccPolyline(vertices);
					polyline->addChild(vertices);
					if (!vertices->reserve(count) || !polyline->reserve(count))
					{
						//not enough memory
						if (app)
							app->dispToConsole(QString("Not enough memory!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
						error = true;
						break;
					}

					//add vertices
					{
						for (unsigned i=0; i<count; ++i)
						{
							vertices->addPoint(CCVector3(	static_cast<PointCoordinateType>(points[i].x),
															static_cast<PointCoordinateType>(points[i].y),
															0));
						}
					}

					//add segments
					polyline->addPointIndex(0,count);
					polyline->setClosed(false); //just to be sure
					polyline->set2DMode(true);

					//add to DB
					polyline->setName(QFileInfo(filename).baseName());
					polyline->setColor(ccColor::green);
					polyline->showColors(true);
					polyline->setEnabled(true);
					polyline->setLocked(true); //as we have applied a purely visual transformation, we can't let the user rotate it!!!
					vertices->setEnabled(false);
				}
				else
				{
					if (app)
						app->dispToConsole(QString("Not enough points in profile?!"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					
					error = true;
					break;
				}
			}
		}

	}

	file.close();

	if (error)
	{
		delete polyline;
		polyline = nullptr;
	}

	return polyline;
}

