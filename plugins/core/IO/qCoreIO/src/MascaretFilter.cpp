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

#include "MascaretFilter.h"

//local
#include "ui_saveMascaretFileDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>

//Qt
#include <QDialog>
#include <QFile>
#include <QTextStream>

//System
#include <cstring>

//! Mascaret File Save dialog
class SaveMascaretFileDlg : public QDialog, public Ui::SaveMascaretFileDlg
{
	Q_OBJECT
	
public:
	//! Default constructor
	SaveMascaretFileDlg(QWidget* parent = nullptr)
		: QDialog(parent, Qt::Tool)
		, Ui::SaveMascaretFileDlg()
	{
		setupUi(this);
	}
};


MascaretFilter::MascaretFilter()
	: FileIOFilter( {
					"_Mascaret Filter",
					DEFAULT_PRIORITY,	// priority
					QStringList(),
					"georef",
					QStringList(),
					QStringList{ "(Geo-)Mascaret profile (*.georef)" },
					Export
					} )
{
}

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

inline QString MakeMascaretName(QString name)
{
	//max 30 characeters
	name = name.left(30);
	//no space characters
	name.replace(' ','_');
	
	return name;
}

inline void ToLocalAbscissa(const CCVector3& P, const CCVector3& C, const CCVector3& U, unsigned char upDir, CCVector2& localP)
{
	//convert to 'local' coordinate system
	localP.x = (P-C).dot(U);
	localP.y = P.u[upDir];
}

CC_FILE_ERROR MascaretFilter::saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters)
{
	Q_UNUSED( parameters );
	
	if (!entity || filename.isEmpty())
	{
		return CC_FERR_BAD_ARGUMENT;
	}
	
	//look for valid profiles
	std::vector<ccPolyline*> profiles;
	try
	{
		//get all polylines
		std::vector<ccPolyline*> candidates;
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
		
		//then keep the valid profiles only
		for (size_t i=0; i<candidates.size(); ++i)
		{
			ccPolyline* poly = candidates[i];
			if (	!poly->hasMetaData(ccPolyline::MetaKeyUpDir())
				||	!poly->hasMetaData(ccPolyline::MetaKeyAbscissa())
				||	!poly->hasMetaData(ccPolyline::MetaKeyPrefixCenter()+".x")
				||	!poly->hasMetaData(ccPolyline::MetaKeyPrefixCenter()+".y")
				||	!poly->hasMetaData(ccPolyline::MetaKeyPrefixCenter()+".z")
				||	!poly->hasMetaData(ccPolyline::MetaKeyPrefixDirection()+".x")
				||	!poly->hasMetaData(ccPolyline::MetaKeyPrefixDirection()+".y")
				||	!poly->hasMetaData(ccPolyline::MetaKeyPrefixDirection()+".z") )
			{
				ccLog::Warning(QString("[Mascaret] Polyline '%1' is not a valid profile (missing meta-data)").arg(poly->getName()));
				break;
			}
			else
			{
				profiles.push_back(poly);
			}
		}
	}
	catch (const std::bad_alloc&)
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
	outFile.setRealNumberNotation(QTextStream::FixedNotation);
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

	//sanitize the 'bief' (reach) name
	biefName = MakeMascaretName(biefName);

	//sort the sections by their abscissa
	if (profiles.size() > 1)
	{
		for (size_t i=0; i<profiles.size()-1; ++i)
		{
			size_t smallestIndex = i;
			double smallestAbscissa = profiles[i]->getMetaData(ccPolyline::MetaKeyAbscissa()).toDouble();
			for (size_t j=i+1; j<profiles.size(); ++j)
			{
				double a = profiles[j]->getMetaData(ccPolyline::MetaKeyAbscissa()).toDouble();
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
		unsigned vertCount = poly ? poly->size() : 0;
		if (vertCount < 2)
		{
			//invalid size
			ccLog::Warning(QString("[Mascaret] Polyline '%1' does not have enough vertices")
						   .arg(poly ? poly->getName() : QStringLiteral("unnamed")));
			continue;
		}
		
		//decode meta-data
		bool ok = true;
		int upDir = 2;
		double absc = 0.0;
		CCVector3d Cd(0,0,0);
		CCVector3d Ud(0,0,0);
		while (true) //fake loop for easy break
		{
			upDir = poly->getMetaData(ccPolyline::MetaKeyUpDir()).toInt(&ok);
			if (!ok) break;
			absc  = poly->getMetaData(ccPolyline::MetaKeyAbscissa()).toDouble(&ok);
			if (!ok) break;
			Cd.x  = poly->getMetaData(ccPolyline::MetaKeyPrefixCenter()+".x").toDouble(&ok);
			if (!ok) break;
			Cd.y  = poly->getMetaData(ccPolyline::MetaKeyPrefixCenter()+".y").toDouble(&ok);
			if (!ok) break;
			Cd.z  = poly->getMetaData(ccPolyline::MetaKeyPrefixCenter()+".z").toDouble(&ok);
			if (!ok) break;
			Ud.x  = poly->getMetaData(ccPolyline::MetaKeyPrefixDirection()+".x").toDouble(&ok);
			if (!ok) break;
			Ud.y  = poly->getMetaData(ccPolyline::MetaKeyPrefixDirection()+".y").toDouble(&ok);
			if (!ok) break;
			Ud.z  = poly->getMetaData(ccPolyline::MetaKeyPrefixDirection()+".z").toDouble(&ok);
			break;
		}
		if (!ok)
		{
			ccLog::Warning(QString("[Mascaret] At least one of the meta-data entry of polyline '%1' is invalid?!").arg(poly->getName()));
			continue;
		}

		QString profileName = poly->getName();
		profileName = MakeMascaretName(profileName);

		CCVector3 C = CCVector3::fromArray(Cd.u);
		CCVector3 U = CCVector3::fromArray(Ud.u);
		U.normalize();

		//write header
		outFile << "PROFIL " << biefName << " " << profileName << " " << absc;
#define SAVE_AS_GEO_MASCARET
#ifdef SAVE_AS_GEO_MASCARET
		int xDir = upDir == 2 ? 0 : upDir+1;
		int yDir =  xDir == 2 ? 0 :  xDir+1;
		//for "geo"-mascaret, we add some more information:
		// - first point
		{
			const CCVector3* firstP = poly->getPoint(0);
			CCVector3d firstPg = poly->toGlobal3d(*firstP);
			outFile << " ";
			outFile << firstPg.u[xDir] << " " << firstPg.u[yDir];
		}
		// - last point
		{
			const CCVector3* lastP = poly->getPoint(vertCount-1);
			CCVector3d lastPg = poly->toGlobal3d(*lastP);
			outFile << " ";
			outFile << lastPg.u[xDir] << " " << lastPg.u[yDir];
		}
		// - profile/path intersection point
		{
			outFile << " AXE ";
			CCVector3d Cdg = poly->toGlobal3d(Cd);
			outFile << Cdg.u[xDir] << " " << Cdg.u[yDir];
		}
#endif
		outFile << endl;

		//check the abscissa values order (must be increasing!)
		bool inverted = false;
		{
			const CCVector3* P0 = poly->getPoint(0);
			//convert to 'local' coordinate system
			CCVector2 Q0;
			ToLocalAbscissa(*P0, C, U, upDir, Q0);

			const CCVector3* P1 = poly->getPoint(vertCount-1);
			//convert to 'local' coordinate system
			CCVector2 Q1;
			ToLocalAbscissa(*P1, C, U, upDir, Q1);

			inverted = (Q1.x < Q0.x);
		}


		for (unsigned j=0; j<vertCount; ++j)
		{
			const CCVector3* P = poly->getPoint(inverted ? vertCount-1-j : j);

			//convert to 'local' coordinate system
			CCVector2 Q;
			ToLocalAbscissa(*P, C, U, upDir, Q);

			outFile << Q.x << " " << Q.y << " " << type;
#ifdef SAVE_AS_GEO_MASCARET
			{
				//for "geo"-mascaret, we add some more information:
				// - real coordinates of the point
				outFile << " ";
				CCVector3d Pg = poly->toGlobal3d(*P);
				outFile << Pg.u[xDir] << " " << Pg.u[yDir];
			}
#endif
			outFile << endl;
		}

		result = CC_FERR_NO_ERROR;
	}

	file.close();

	return result;
}

#include "MascaretFilter.moc"
