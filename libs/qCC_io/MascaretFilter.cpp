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

#include "MascaretFilter.h"

//local
#include "ui_saveMascaretFileDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>

//Qt
#include <QFile>
#include <QTextStream>
#include <QDialog>

//System
#include <string.h>

//! Mascaret File Save dialog
class SaveMascaretFileDlg : public QDialog, public Ui::SaveMascaretFileDlg
{
public:
	//! Default constructor
	SaveMascaretFileDlg(QWidget* parent = 0)
		: QDialog(parent)
		, Ui::SaveMascaretFileDlg()
	{
		setupUi(this);
		setWindowFlags(Qt::Tool);
	}
};

bool MascaretFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
	if (type == CC_TYPES::POLY_LINE)
	{
		multiple = true;
		exclusive = true;
		return true;
	}
	return false;
}

QString MakeMascaretName(QString name)
{
	//max 30 characeters
	name = name.left(30);
	//no space characters
	name.replace(' ','_');
	
	return name;
}

CC_FILE_ERROR MascaretFilter::saveToFile(ccHObject* entity, QString filename)
{
	if (!entity || filename.isEmpty())
		return CC_FERR_BAD_ARGUMENT;

	//look for either a cloud or a mesh
	std::vector<ccPolyline*> profiles;
	try
	{
		if (entity->isA(CC_TYPES::POLY_LINE))
		{
			profiles.push_back(static_cast<ccPolyline*>(entity));
		}
		else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			for (unsigned i=0; i<entity->getChildrenNumber(); ++i)
				if (entity->getChild(i) && entity->getChild(i)->isA(CC_TYPES::POLY_LINE))
					profiles.push_back(static_cast<ccPolyline*>(entity->getChild(i)));
		}
	}
	catch(std::bad_alloc)
	{
		return CC_FERR_NOT_ENOUGH_MEMORY;
	}

	if (profiles.empty())
		return CC_FERR_NO_SAVE;

	//open ASCII file for writing
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return CC_FERR_WRITING;

	QTextStream outFile(&file);
	outFile.setRealNumberPrecision(12);

	//ask some parameters
	SaveMascaretFileDlg smfDlg;
	if (!smfDlg.exec())
		return CC_FERR_CANCELED_BY_USER;

	QString biefName = smfDlg.biefNameLineEdit->text();
	QString type("T"); //B or T --> ask the user
	switch(smfDlg.typeComboBox->currentIndex())
	{
	case 0:
		type = "B"; //bathy
		break;
	case 1:
		type = "T"; //topo
		break;
	default:
		assert(false);
	}

	//sanitize the 'bief' (pound) name
	biefName = MakeMascaretName(biefName);

	//sort the sections by their abscissa
	if (profiles.size() > 1)
	{
		for (size_t i=0; i<profiles.size()-1; ++i)
		{
			size_t smallestIndex = i;
			double smallestAbscissa = profiles[i]->getMetaData(KeyAbscissa()).toDouble();
			for (size_t j=i+1; j<profiles.size(); ++j)
			{
				double a = profiles[j]->getMetaData(KeyAbscissa()).toDouble();
				if (a < smallestAbscissa)
				{
					smallestAbscissa = a;
					smallestIndex = j;
				}
			}

			if (i != smallestIndex)
			{
				std::swap(profiles[i],profiles[smallestIndex]);
			}
		}
	}

	CC_FILE_ERROR result = CC_FERR_NO_SAVE;

	//for each profile
	for (size_t i=0; i<profiles.size(); ++i)
	{
		ccPolyline* poly = profiles[i];
		bool ok = true;
		int upDir = 2;
		double absc = 0.0;
		CCVector3d Cd(0,0,0);
		CCVector3d Ud(0,0,0);
		while (true) //fake loop for easy break
		{
			upDir = poly->getMetaData(KeyUpDir()).toInt(&ok);
			if (!ok) break;
			absc  = poly->getMetaData(KeyAbscissa()).toDouble(&ok);
			if (!ok) break;
			Cd.x  = poly->getMetaData(KeyCenter()+".x").toDouble(&ok);
			if (!ok) break;
			Cd.y  = poly->getMetaData(KeyCenter()+".y").toDouble(&ok);
			if (!ok) break;
			Cd.z  = poly->getMetaData(KeyCenter()+".z").toDouble(&ok);
			if (!ok) break;
			Ud.x  = poly->getMetaData(KeyDirection()+".x").toDouble(&ok);
			if (!ok) break;
			Ud.y  = poly->getMetaData(KeyDirection()+".y").toDouble(&ok);
			if (!ok) break;
			Ud.z  = poly->getMetaData(KeyDirection()+".z").toDouble(&ok);
			break;
		}
		if (!ok)
		{
			ccLog::Warning(QString("Polyline '%1' has not the right meta-data to be saved in a Mascaret file").arg(poly->getName()));
			continue;
		}

		QString profileName = poly->getName();
		profileName = MakeMascaretName(profileName);

		CCVector3 C = CCVector3::fromArray(Cd.u);
		CCVector3 U = CCVector3::fromArray(Ud.u);
		U.normalize();
		
		//write header
		outFile << "PROFIL " << biefName << " " << profileName << " " << absc << endl;

		for (unsigned j=0; j<poly->size(); ++j)
		{
			const CCVector3* P = poly->getPoint(j);

			//convert to 'local' coordinate system
			CCVector3 Q = *P - C;
			Q.x = Q.dot(U);
			Q.y = P->u[upDir];
			Q.z = 0;

			outFile << Q.x << " " << Q.y << " " << type << endl;
		}

		result = CC_FERR_NO_ERROR;
	}

	file.close();

	return result;
}
