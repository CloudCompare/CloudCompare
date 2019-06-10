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

#include "SalomeHydroFilter.h"

//qCC_db
#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>

//Qt
#include <QFile>
#include <QStringList>
#include <QTextStream>


SalomeHydroFilter::SalomeHydroFilter()
	: FileIOFilter( {
					"_SalomeHydro Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList{ "poly" },
					"poly",
					QStringList{ "Salome Hydro polylines (*.poly)" },
					QStringList{ "Salome Hydro polylines (*.poly)" },
					Import | Export
					} )
{	
}

bool SalomeHydroFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POLY_LINE)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR SalomeHydroFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	Q_UNUSED( parameters );
	
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	//get all polylines
	std::vector<ccPolyline*> candidates;
	try
	{
		if (entity->isA(CC_TYPES::POLY_LINE))
		{
			candidates.push_back(static_cast<ccPolyline*>(entity));
		}
		else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			for (unsigned i=0; i<entity->getChildrenNumber(); ++i)
				if (entity->getChild(i) && entity->getChild(i)->isA(CC_TYPES::POLY_LINE))
					candidates.push_back(static_cast<ccPolyline*>(entity->getChild(i)));
		}
	}
	catch (const std::bad_alloc&)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	if (candidates.empty())
		return CC_FERR_NO_SAVE;

	//open ASCII file for writing
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return CC_FERR_WRITING;

	QTextStream outFile(&file);
	const int c_precision = 12;

	CC_FILE_ERROR result = CC_FERR_NO_SAVE;

	//for each polyline
	for (size_t i=0; i<candidates.size(); ++i)
	{
		ccPolyline* poly = candidates[i];
		unsigned vertCount = poly ? poly->size() : 0;
		if (vertCount < 2)
		{
			//invalid size
			ccLog::Warning(QString("[Salome Hydro] Polyline '%1' does not have enough vertices")
						   .arg(poly ? poly->getName() : QStringLiteral("unnamed")));
			continue;
		}
		
		//a simple empty line is used as separator between each polyline!
		if (i != 0)
			outFile << endl;

		for (unsigned j=0; j<vertCount; ++j)
		{
			const CCVector3* P = poly->getPoint(j);

			//convert to 'local' coordinate system
			CCVector3d Pg = poly->toGlobal3d(*P);
			outFile << QString::number(Pg.x,'E',c_precision) << " ";
			outFile << QString::number(Pg.y,'E',c_precision) << " ";
			outFile << QString::number(Pg.z,'E',c_precision) << endl;
		}

		result = CC_FERR_NO_ERROR;
	}

	file.close();

	return result;
}

CC_FILE_ERROR SalomeHydroFilter::loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters)
{
	//we open the file (ASCII mode)
	QFile file(filename);
	if (!file.open(QFile::ReadOnly))
	{
		return CC_FERR_READING;
	}
	QTextStream stream(&file);

	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	CCVector3d Pshift(0, 0, 0);
	bool preserveCoordinateShift = true;
	bool firstPoint = true;

	ccPointCloud* currentVertices = nullptr;
	unsigned index = 0;
	while (true)
	{
		QString currentLine = stream.readLine().trimmed();
		if (currentLine.isNull() || currentLine.isEmpty())
		{
			//close any ongoing polyline
			if (currentVertices)
			{
				if (currentVertices->size() < 2)
				{
					delete currentVertices;
					currentVertices = nullptr;
					ccLog::Warning("[Salome Hydro] An invalid polyline (single vertex) will be ignored");
				}
				else
				{
					currentVertices->shrinkToFit();
					
					//create the corresponding polyline
					ccPolyline* newPoly = new ccPolyline(currentVertices);
					newPoly->setName(QString(QString("Polyline #%1").arg(++index)));
					newPoly->addChild(currentVertices);
					newPoly->set2DMode(false);

					if (!newPoly->reserve(currentVertices->size()))
					{
						delete newPoly;
						result = CC_FERR_NOT_ENOUGH_MEMORY;
						break;
					}
					newPoly->addPointIndex(0,currentVertices->size());
					currentVertices->setEnabled(false);
					container.addChild(newPoly);
					currentVertices = nullptr;
				}
			}

			if (currentLine.isNull())
			{
				//end of file
				break;
			}
		}
		else
		{
			if (!currentVertices)
			{
				currentVertices = new ccPointCloud("vertices");
				if (!firstPoint && preserveCoordinateShift)
				{
					currentVertices->setGlobalShift(Pshift);
				}
			}

			QStringList parts = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);
			if (parts.size() == 3)
			{
				//(X,Y,Z)
				CCVector3d P(	parts[0].toDouble(),
								parts[1].toDouble(),
								parts[2].toDouble() );

				//first point: check for 'big' coordinates
				if (firstPoint)
				{
					if (HandleGlobalShift(P, Pshift, preserveCoordinateShift, parameters))
					{
						if (preserveCoordinateShift)
						{
							currentVertices->setGlobalShift(Pshift);
						}
						ccLog::Warning("[Salome Hydro] Polylines will be recentered! Translation: (%.2f ; %.2f ; %.2f)", Pshift.x, Pshift.y, Pshift.z);
					}
					firstPoint = false;
				}

				//add point
				if (currentVertices->size() == currentVertices->capacity())
				{
					if (!currentVertices->reserve(currentVertices->size() + 64))
					{
						delete currentVertices;
						currentVertices = nullptr;
						result = CC_FERR_NOT_ENOUGH_MEMORY;
						break;
					}
				}
				currentVertices->addPoint(CCVector3::fromArray((P + Pshift).u));
			}
			else
			{
				ccLog::Warning("[Salome Hydro] Malformed file: 3 values per line expected");
				result = CC_FERR_MALFORMED_FILE;
				break;
			}
		}
	}
	
	delete currentVertices;
	currentVertices = nullptr;
	
	return result;
}
