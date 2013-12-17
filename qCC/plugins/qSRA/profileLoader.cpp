//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "profileLoader.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccPolyline.h>

//qCC
#include <plugins/ccMainAppInterface.h>

//Qt
#include <QFile>
#include <QTextStream>
#include <QFileInfo>

ccPolyline* ProfileLoader::Load(QString filename, bool ignoreAxisShift/*=true*/, int heightDim/*=2*/, ccMainAppInterface* app/*=0*/)
{
	//load profile as a polyline
	QFile file(filename);
	assert(file.exists());
	if (!file.open(QFile::ReadOnly | QFile::Text))
	{
		if (app)
			app->dispToConsole(QString("Failed to open file for reading! Check access rights"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return 0;
	}

	QTextStream stream(&file);

	ccPolyline* polyline  = 0;

	bool error = false;
	for (unsigned n=0; n<1; ++n) //fake loop for easy break ;)
	{
		//read center
		CCVector3d G(0,0,0);
		{
			QString headerLine = stream.readLine();
			if (headerLine.isEmpty() || !headerLine.startsWith("X"))
			{
				if (app)
					app->dispToConsole(QString("Malformed file (center header expected on first line)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				error = true;
				break;
			}
			QString centerLine = stream.readLine();
			{
				QStringList tokens = centerLine.split(QRegExp("\\s+"),QString::SkipEmptyParts);
				if (tokens.size() < 3)
				{
					if (app)
						app->dispToConsole(QString("Malformed file (center coordinates expected on second line)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					error = true;
					break;
				}
				G.x = tokens[0].toDouble();
				G.y = tokens[1].toDouble();
				G.z = tokens[2].toDouble();
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
			std::vector< Vector2Tpl<double> > points;
			while (!line.isEmpty())
			{
				QStringList tokens = line.split(QRegExp("\\s+"),QString::SkipEmptyParts);
				if (tokens.size() < 2)
				{
					if (app)
						app->dispToConsole(QString("Malformed file (radius/height couple expected from the 4th line and afterwards)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					error = true;
					break;
				}

				Vector2Tpl<double> P;
				P.x = tokens[0].toDouble(); //radius
				P.y = tokens[1].toDouble(); //height

				if (ignoreAxisShift)
					P.y -= G.u[heightDim];

				try
				{
					points.push_back(P);
				}
				catch(std::bad_alloc)
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
				unsigned count = (unsigned)points.size();
				if (count > 1)
				{
					ccPointCloud* vertices = new ccPointCloud("vertices");
					polyline = new ccPolyline(vertices);
					polyline->addChild(vertices);
					if (!vertices->reserve(count) || !polyline->reserve(count-1))
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
							vertices->addPoint(CCVector3((PointCoordinateType)points[i].x,(PointCoordinateType)points[i].y,0));
						}

						vertices->setOriginalShift(G.x,G.y,G.z);
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
						app->dispToConsole(QString("Not enough points in profile?!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					
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
		polyline = 0;
	}

	return polyline;
}

