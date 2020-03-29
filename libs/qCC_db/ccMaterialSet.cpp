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

#include "ccMaterialSet.h"

//Local
#include "ccGenericGLDisplay.h"

//Qt
#include <QFileInfo>
#include <QImage>
#include <QSet>

//System
#include <set>


ccMaterialSet::ccMaterialSet(const QString& name)
	: std::vector<ccMaterial::CShared>()
	, CCShareable()
	, ccHObject(name)
{
	setFlagState(CC_LOCKED,true);
}

int ccMaterialSet::findMaterialByName(const QString& mtlName)
{
	ccLog::PrintDebug(QString("[ccMaterialSet::findMaterialByName] Query: ") + mtlName);
	
	int i = 0;
	for (ccMaterialSet::const_iterator it = begin(); it != end(); ++it,++i)
	{
		ccMaterial::CShared mtl = *it;
		ccLog::PrintDebug(QString("\tmaterial #%1 name: %2").arg(i).arg(mtl->getName()));
		if (mtl->getName() == mtlName)
			return i;
	}

	return -1;
}

int ccMaterialSet::findMaterialByUniqueID(const QString& uniqueID)
{
	ccLog::PrintDebug(QString("[ccMaterialSet::findMaterialByUniqueID] Query: ") + uniqueID);
	
	int i = 0;
	for (ccMaterialSet::const_iterator it = begin(); it != end(); ++it,++i)
	{
		ccMaterial::CShared mtl = *it;
		ccLog::PrintDebug(QString("\tmaterial #%1 ID: %2").arg(i).arg(mtl->getUniqueIdentifier()));
		if (mtl->getUniqueIdentifier() == uniqueID)
			return i;
	}

	return -1;
}

int ccMaterialSet::addMaterial(ccMaterial::CShared mtl, bool allowDuplicateNames/*=false*/)
{
	if (!mtl)
	{
		//invalid input material
		return -1;
	}
	 
	//material already exists?
	int previousIndex = findMaterialByName(mtl->getName());
	//DGM: warning, the materials may have the same name, but they may be different in reality (other texture, etc.)!
	if (previousIndex >= 0)
	{
		const ccMaterial::CShared& previousMtl = (*this)[previousIndex];
		if (!previousMtl->compare(*mtl))
		{
			//in fact the material is a bit different
			previousIndex = -1;
			if (!allowDuplicateNames)
			{
				//generate a new name
				static const unsigned MAX_ATTEMPTS = 100;
				for (unsigned i=1 ; i < MAX_ATTEMPTS; i++)
				{
					QString newMtlName = previousMtl->getName() + QString("_%1").arg(i);
					if (findMaterialByName(newMtlName) < 0)
					{
						//we duplicate the material and we change its name
						ccMaterial::Shared newMtl(new ccMaterial(*mtl));
						newMtl->setName(newMtlName);
						mtl = newMtl;
						break;
					}
				}
			}
		}
	}
	if (previousIndex >= 0 && !allowDuplicateNames)
		return previousIndex;

	try
	{
		push_back(mtl);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return -1;
	}

	return static_cast<int>(size())-1;
}

//MTL PARSER INSPIRED FROM KIXOR.NET "objloader" (http://www.kixor.net/dev/objloader/)
bool ccMaterialSet::ParseMTL(QString path, const QString& filename, ccMaterialSet &materials, QStringList& errors)
{
	//open mtl file
	QString fullPathFilename = path + QString('/') + filename;
	QFile file(fullPathFilename);
	if (!file.open(QFile::ReadOnly))
	{
		errors << QString("Error reading file: %1").arg(filename);
		return false;
	}

	//update path (if the input filename has already a relative path)
	path = QFileInfo(fullPathFilename).absolutePath();
	
	QTextStream stream(&file);

	QString currentLine = stream.readLine();
	unsigned currentLineIndex = 0;
	ccMaterial::Shared currentMaterial( nullptr );
	
	while( !currentLine.isNull() )
	{
		++currentLineIndex;

		QStringList tokens = currentLine.simplified().split(QChar(' '), QString::SkipEmptyParts);

		//skip comments & empty lines
		if( tokens.empty() || tokens.front().startsWith('/',Qt::CaseInsensitive) || tokens.front().startsWith('#',Qt::CaseInsensitive) )
		{
			currentLine = stream.readLine();
			continue;
		}

		//start material
		if (tokens.front() == "newmtl")
		{
			//push the previous material (if any)
			if (currentMaterial)
			{
				materials.addMaterial(currentMaterial);
				currentMaterial = ccMaterial::Shared( nullptr );
			}

			// get the name
			QString materialName = currentLine.mid(7).trimmed(); //we must take the whole line! (see OBJ filter)
			if (materialName.isEmpty())
				materialName = "undefined";
			currentMaterial = ccMaterial::Shared(new ccMaterial(materialName));

		}
		else if (currentMaterial) //we already have a "current" material
		{
			//ambient
			if (tokens.front() == "Ka")
			{
				if (tokens.size() > 3)
				{
					ccColor::Rgbaf ambient(	tokens[1].toFloat(),
											tokens[2].toFloat(),
											tokens[3].toFloat(),
											1.0f);
					currentMaterial->setAmbient(ambient);
				}
			}

			//diff
			else if (tokens.front() == "Kd")
			{
				if (tokens.size() > 3)
				{
					ccColor::Rgbaf diffuse(	tokens[1].toFloat(),
											tokens[2].toFloat(),
											tokens[3].toFloat(),
											1.0f);
					currentMaterial->setDiffuse(diffuse);
				}
			}

			//specular
			else if (tokens.front() == "Ks")
			{
				if (tokens.size() > 3)
				{
					ccColor::Rgbaf specular(tokens[1].toFloat(),
											tokens[2].toFloat(),
											tokens[3].toFloat(),
											1.0f);
					currentMaterial->setSpecular(specular);
				}
			}
			//shiny
			else if (tokens.front() == "Ns")
			{
				if (tokens.size() > 1)
					currentMaterial->setShininess(tokens[1].toFloat());
			}
			//transparent
			else if (tokens.front() == "d")
			{
				if (tokens.size() > 1)
					currentMaterial->setTransparency(tokens[1].toFloat());
			}
			else if (tokens.front() == "Tr")
			{
				if (tokens.size() > 1)
					currentMaterial->setTransparency(1.0f - tokens[1].toFloat());
			}
			//reflection
			else if (tokens.front() == "r")
			{
				//ignored
				//if (tokens.size() > 1)
				//	currentMaterial->reflect = tokens[1].toFloat();
			}
			//glossy
			else if (tokens.front() == "sharpness")
			{
				//ignored
				//if (tokens.size() > 1)
				//	currentMaterial->glossy = tokens[1].toFloat();
			}
			//refract index
			else if (tokens.front() == "Ni")
			{
				//ignored
				//if (tokens.size() > 1)
				//	currentMaterial->refract_index = tokens[1].toFloat();
			}
			// illumination type
			else if (tokens.front() == "illum")
			{
				//ignored
			}
			// Transmission filter
			else if (tokens.front() == "Tf")
			{
				//ignored
			}
			// texture map
			else if (tokens.front() == "map_Ka"
					|| tokens.front() == "map_Kd"
					|| tokens.front() == "map_Ks")
			{
				//DGM: in case there's hidden or space characters at the beginning of the line...
				int shift = currentLine.indexOf("map_K",0);
				QString textureFilename = (shift + 7 < currentLine.size() ? currentLine.mid(shift+7).trimmed() : QString());
				//remove any quotes around the filename (Photoscan 1.4 bug)
				if (textureFilename.startsWith("\""))
				{
					textureFilename = textureFilename.right(textureFilename.size() - 1);
				}
				if (textureFilename.endsWith("\""))
				{
					textureFilename = textureFilename.left(textureFilename.size() - 1);
				}

				QString fullTexName = path + QString('/') + textureFilename;
				if (!currentMaterial->loadAndSetTexture(fullTexName))
				{
					errors << QString("Failed to load texture file: %1").arg(fullTexName);
				}
			}
			else
			{
				errors << QString("Unknown command '%1' at line %2").arg(tokens.front()).arg(currentLineIndex);
			}
		}

		currentLine = stream.readLine();
	}

	file.close();

	//don't forget to push the last material!
	if (currentMaterial)
		materials.addMaterial(currentMaterial);

	return true;
}

bool ccMaterialSet::saveAsMTL(const QString& path, const QString& baseFilename, QStringList& errors) const
{
	//open mtl file
	QString filename = path+QString('/')+baseFilename+QString(".mtl");
	QFile file(filename);
	if (!file.open(QFile::WriteOnly))
	{
		errors << QString("Error writing file: %1").arg(filename);
		return false;
	}
	QTextStream stream(&file);

	stream << "# Generated by CloudCompare" << endl;

	//texture filenames already used
	QMap<QString,QString> absFilenamesSaved;
	QSet<QString> filenamesUsed;

	size_t matIndex = 0;
	for (ccMaterialSet::const_iterator it=begin(); it!=end(); ++it, ++matIndex)
	{
		ccMaterial::CShared mtl = *it;
		stream << endl << "newmtl " << mtl->getName() << endl;

		const ccColor::Rgbaf& Ka = mtl->getAmbient();
		const ccColor::Rgbaf& Kd = mtl->getDiffuseFront();
		const ccColor::Rgbaf& Ks = mtl->getSpecular();
		stream << "Ka " << Ka.r << " " << Ka.g << " " << Ka.b << endl;
		stream << "Kd " << Kd.r << " " << Kd.g << " " << Kd.b << endl;
		stream << "Ks " << Ks.r << " " << Ks.g << " " << Ks.b << endl;
		// we take the ambient's alpha by default
		// openGL "a = 1.0" is for "full opacity" / MTL "Tr = 1.0" is for "full transparency" 
		stream << "Tr " << 1.0 - Ka.a << endl; 
		stream << "illum 1" << endl;
		stream << "Ns " << mtl->getShininessFront()	<< endl; //we take the front's by default

		if (mtl->hasTexture())
		{
			QString absFilename = mtl->getTextureFilename();

			//if the file has not already been saved
			if (!absFilenamesSaved.contains(absFilename))
			{
				QFileInfo fileInfo(absFilename);
				
				QString texName = fileInfo.fileName();
				if (fileInfo.suffix().isEmpty())
				{
					texName += QString(".jpg");
				}

				//make sure that the local filename is unique!
				if (filenamesUsed.contains(texName))
				{
					texName.prepend(QString("t%1_").arg(matIndex));
					assert(!filenamesUsed.contains(texName));
				}
				filenamesUsed.insert(texName);

				QString destFilename = path + QString('/') + texName;
				if (mtl->getTexture().mirrored().save(destFilename)) //mirrored: see ccMaterial
				{
					//new absolute filemane
					absFilenamesSaved[absFilename] = texName;
				}
				else
				{
					errors << QString("Failed to save the texture of material '%1' to file '%2'!").arg(mtl->getName(),destFilename);
				}
			}

			if (absFilenamesSaved.contains(absFilename))
			{
				assert(!absFilenamesSaved[absFilename].isEmpty());
				stream << "map_Kd " << absFilenamesSaved[absFilename] << endl;
			}
		}
	}

	file.close();


	return true;
}

bool ccMaterialSet::append(const ccMaterialSet& source)
{
	try
	{
		for (const auto &mtl : source)
		{
			if (addMaterial(mtl) <= 0)
			{
				ccLog::WarningDebug(QString("[ccMaterialSet::append] Material %1 couldn't be added to material set and will be ignored").arg(mtl->getName()));
			}
		}
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		ccLog::Warning("[ccMaterialSet::append] Not enough memory");
		return false;
	}

	return true;
}

ccMaterialSet* ccMaterialSet::clone() const
{
	ccMaterialSet* cloneSet = new ccMaterialSet(getName());
	if (!cloneSet->append(*this))
	{
		ccLog::Warning("[ccMaterialSet::clone] Not enough memory");
		cloneSet->release();
		cloneSet = nullptr;
	}

	return cloneSet;
}

bool ccMaterialSet::toFile_MeOnly(QFile& out) const
{
	if (!ccHObject::toFile_MeOnly(out))
		return false;

	//Materials count (dataVersion>=20)
	uint32_t count = (uint32_t)size();
	if (out.write((const char*)&count, 4) < 0)
		return WriteError();

	//texture filenames
	std::set<QString> texFilenames;

	//Write each material
	for (const auto &mtl : *this)
	{
		mtl->toFile(out);

		//remember its texture as well (if any)
		QString texFilename = mtl->getTextureFilename();
		if (!texFilename.isEmpty())
			texFilenames.insert(texFilename);
	}

	//now save the number of textures (dataVersion>=37)
	QDataStream outStream(&out);
	outStream << static_cast<uint32_t>(texFilenames.size());
	//and save the textures (dataVersion>=37)
	{
		for (const auto &texFilename : texFilenames)
		{
			outStream << texFilename; //name
			outStream << ccMaterial::GetTexture(texFilename); //then image
		}
	}

	return true;
}

bool ccMaterialSet::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//Materials count (dataVersion>=20)
	uint32_t count = 0;;
	if (in.read((char*)&count, 4) < 0)
		return ReadError();
	if (count == 0)
		return true;

	//Load each material
	{
		for (uint32_t i = 0; i < count; ++i)
		{
			ccMaterial::Shared mtl(new ccMaterial);
			if (!mtl->fromFile(in, dataVersion, flags, oldToNewIDMap))
				return false;
			addMaterial(mtl, true); //if we load a file, we can't allow that materials are not in the same order as before!
		}
	}

	if (dataVersion >= 37)
	{
		QDataStream inStream(&in);

		//now load the number of textures (dataVersion>=37)
		uint32_t texCount = 0;
		inStream >> texCount;
		//and load the textures (dataVersion>=37)
		{
			for (uint32_t i = 0; i < texCount; ++i)
			{
				QString filename;
				inStream >> filename;
				QImage image;
				inStream >> image;
				ccMaterial::AddTexture(image, filename);
			}
		}
	}

	return true;
}
