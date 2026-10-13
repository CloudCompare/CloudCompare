// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

#include "ccIndividualFilesCollector.h"

// qCC_db
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>

// qCC_io
#include <BinFilter.h>

// Qt
#include <QFileInfo>
#include <QRegularExpression>
#include <QStringList>

// Helper: returns whether a child entity is the vertices of its parent (mesh or polyline)
static bool IsVerticesOf(ccHObject* parent, const ccHObject* child)
{
	if (parent->isKindOf(CC_TYPES::MESH))
	{
		return (ccHObjectCaster::ToGenericMesh(parent)->getAssociatedCloud() == child);
	}
	if (parent->isKindOf(CC_TYPES::POLY_LINE))
	{
		return (dynamic_cast<ccPointCloud*>(ccHObjectCaster::ToPolyline(parent)->getAssociatedCloud()) == child);
	}
	return false;
}

void ccIndividualFilesCollector::collect(ccHObject* entity, const QString& relativeFolder /*=QString()*/, bool reportIfIgnored /*=true*/)
{
	if (!entity)
	{
		return;
	}

	// we don't want to save an entity twice
	if (!m_visited.insert(entity->getUniqueID()).second)
	{
		return;
	}

	// groups become sub-folders
	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		QString childFolder = subFolder(relativeFolder, GetSubFolderName(entity));
		for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
		{
			collect(entity->getChild(i), childFolder, true);
		}
		return;
	}

	QString fileFilter;
	if (entity->isA(CC_TYPES::POINT_CLOUD))
	{
		fileFilter = cloudFilter;
		++m_cloudCount;
	}
	else if (entity->isKindOf(CC_TYPES::MESH) && !entity->isA(CC_TYPES::SUB_MESH))
	{
		fileFilter = meshFilter;
		++m_meshCount;
	}
	else if (entity->isKindOf(CC_TYPES::POLY_LINE))
	{
		fileFilter = polylineFilter;
		++m_polylineCount;
	}
	else
	{
		if (reportIfIgnored)
		{
			m_ignored.push_back(entity);
		}
		// there may be compatible entities below it
		collectChildren(entity, subFolder(relativeFolder, GetSubFolderName(entity)));
		return;
	}

	m_files.push_back({entity, fileFilter, relativeFolder, ToValidFileName(entity->getName())});

	// a BIN file already contains all the children of the entity
	if (fileFilter == BinFilter::GetFileFilter())
	{
		return;
	}

	// the children of an entity are saved in a sub-folder named after the entity
	collectChildren(entity, subFolder(relativeFolder, GetSubFolderName(entity)));
}

void ccIndividualFilesCollector::collectChildren(ccHObject* entity, const QString& relativeFolder)
{
	for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
	{
		ccHObject* child = entity->getChild(i);
		if (!IsVerticesOf(entity, child))
		{
			collect(child, relativeFolder, false);
		}
	}
}

QString ccIndividualFilesCollector::subFolder(const QString& relativeFolder, const QString& name) const
{
	if (!createSubFolders)
	{
		return relativeFolder;
	}
	return relativeFolder.isEmpty() ? name : relativeFolder + '/' + name;
}

QString ccIndividualFilesCollector::ToValidFileName(QString name)
{
	// characters forbidden in file names (at least on Windows)
	static const QRegularExpression s_invalidChars(R"([<>:"/\\|?*\x00-\x1f])");
	name.replace(s_invalidChars, "_");

	// trailing dots and spaces are not allowed on Windows
	name = name.trimmed();
	while (name.endsWith('.'))
	{
		name.chop(1);
		name = name.trimmed();
	}

	if (name.isEmpty())
	{
		return QStringLiteral("unnamed");
	}

	// reserved names on Windows
	static const QRegularExpression s_reservedNames(R"(^(CON|PRN|AUX|NUL|COM\d|LPT\d)(\..*)?$)", QRegularExpression::CaseInsensitiveOption);
	if (s_reservedNames.match(name).hasMatch())
	{
		name.prepend('_');
	}

	return name;
}

QString ccIndividualFilesCollector::GetSubFolderName(const ccHObject* entity)
{
	QString name = entity->getName();

	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		// groups created when loading a file are named 'filename.ext (fullpath)'
		static const QRegularExpression s_loadedFileGroup(R"(^(.+\.\w+) \(.*\)$)");
		QRegularExpressionMatch         match = s_loadedFileGroup.match(name);
		if (match.hasMatch())
		{
			name = QFileInfo(match.captured(1)).completeBaseName();
		}
	}

	return ToValidFileName(name);
}

QString ccIndividualFilesCollector::GetParentRelativeFolder(const ccHObject* entity, const ccHObject* root)
{
	QStringList folders;
	for (const ccHObject* parent = entity->getParent(); parent && parent != root; parent = parent->getParent())
	{
		folders.prepend(GetSubFolderName(parent));
	}
	return folders.join('/');
}

ccHObject::Container ccIndividualFilesCollector::RemoveDescendants(const ccHObject::Container& entities)
{
	std::set<const ccHObject*> entitySet(entities.begin(), entities.end());

	ccHObject::Container result;
	for (ccHObject* entity : entities)
	{
		bool ancestorInList = false;
		for (const ccHObject* parent = entity->getParent(); parent; parent = parent->getParent())
		{
			if (entitySet.find(parent) != entitySet.end())
			{
				ancestorInList = true;
				break;
			}
		}
		if (!ancestorInList)
		{
			result.push_back(entity);
		}
	}
	return result;
}

QString ccIndividualFilesCollector::MakeUniqueFilename(QSet<QString>& usedFilenames, const QString& folder, const QString& baseName, const QString& extension)
{
	QString filename = QString("%1/%2.%3").arg(folder, baseName, extension);
	for (unsigned index = 1; usedFilenames.contains(filename.toLower()); ++index)
	{
		filename = QString("%1/%2_%3.%4").arg(folder, baseName, QString::number(index), extension);
	}
	usedFilenames.insert(filename.toLower());
	return filename;
}
