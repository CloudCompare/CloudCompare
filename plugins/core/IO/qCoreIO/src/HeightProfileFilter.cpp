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

#include "HeightProfileFilter.h"

//qCC_db
#include <ccPolyline.h>

//Qt
#include <QFile>
#include <QTextStream>


HeightProfileFilter::HeightProfileFilter()
	: FileIOFilter( {
					"_Height profile Filter",
					21.0f,	// priority
					QStringList(),
					"",
					QStringList(),
					QStringList{ "Height profile (*.csv)" },
					Export
					} )
{
}

bool HeightProfileFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POLY_LINE)
	{
		multiple = false;
		exclusive = true;
		return true;
	}
	return false;
}

CC_FILE_ERROR HeightProfileFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	Q_UNUSED( parameters )
	
	if (!entity || filename.isEmpty())
	{
		return CC_FERR_BAD_ARGUMENT;
	}

	//get the polyline
	if (!entity->isA(CC_TYPES::POLY_LINE))
	{
		return CC_FERR_BAD_ENTITY_TYPE;
	}
	ccPolyline* poly = static_cast<ccPolyline*>(entity);
	unsigned vertCount = poly->size();
	if (vertCount == 0)
	{
		//invalid size
		ccLog::Warning(QString("[Height profile] Polyline '%1' is empty").arg(poly->getName()));
		return CC_FERR_NO_SAVE;
	}

	//open ASCII file for writing
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		return CC_FERR_WRITING;
	}

	QTextStream outFile(&file);
	outFile.setRealNumberNotation(QTextStream::FixedNotation);
	outFile.setRealNumberPrecision(sizeof(PointCoordinateType) == 4 && !poly->isShifted() ? 8 : 12);
	outFile << "Curvilinear abscissa; Z" << endl;

	//curvilinear abscissa
	double s = 0;
	const CCVector3* lastP = nullptr;
	for (unsigned j = 0; j < vertCount; ++j)
	{
		const CCVector3* P = poly->getPoint(j);
		//update the curvilinear abscissa
		if (lastP)
		{
			s += (*P - *lastP).normd();
		}
		lastP = P;

		//convert to 'local' coordinate system
		CCVector3d Pg = poly->toGlobal3d(*P);
		outFile << s << "; " << Pg.z << endl;
	}

	file.close();

	return CC_FERR_NO_ERROR;
}
