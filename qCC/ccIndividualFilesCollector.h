#pragma once

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

// qCC_db
#include <ccHObject.h>

// Qt
#include <QSet>
#include <QString>

// System
#include <map>
#include <set>
#include <vector>

//! Collects the entities to be saved as individual files (one file per entity)
/** Groups of the DB tree become sub-folders. The children of a saved entity
    are saved in a sub-folder named after the entity. Sibling folders with the
    same name get a suffix (_1, _2, etc.). The vertices of meshes and polylines,
    and the sub-meshes of a saved mesh, are not saved separately.
**/
class ccIndividualFilesCollector
{
  public:
	//! An entity to be saved in its own file
	struct File
	{
		ccHObject* entity = nullptr;
		//! Output 'file filter'
		QString fileFilter;
		//! Folder, relative to the output folder ('/' separated, empty = output folder)
		QString relativeFolder;
		//! File name, without extension
		QString baseName;
	};

	//! Output 'file filter' for clouds
	QString cloudFilter;
	//! Output 'file filter' for meshes
	QString meshFilter;
	//! Output 'file filter' for polylines
	QString polylineFilter;
	//! Whether the hierarchy should be reproduced as sub-folders
	bool createSubFolders = true;

	//! Collects an entity and its children (recursively)
	/** \param entity entity to collect
	    \param relativeFolder folder of the entity, relative to the output folder
	    \param reportIfIgnored whether to add the entity to the 'ignored' list if it can't be saved this way
	**/
	void collect(ccHObject* entity, const QString& relativeFolder = QString(), bool reportIfIgnored = true);

	//! Returns the folder of an entity, relative to the root entity (i.e. the folders of its ancestors)
	QString getParentRelativeFolder(const ccHObject* entity, const ccHObject* root);

	//! Collected entities
	const std::vector<File>& files() const
	{
		return m_files;
	}
	//! Entities that can't be saved this way
	const std::vector<ccHObject*>& ignored() const
	{
		return m_ignored;
	}

	unsigned cloudCount() const
	{
		return m_cloudCount;
	}
	unsigned meshCount() const
	{
		return m_meshCount;
	}
	unsigned polylineCount() const
	{
		return m_polylineCount;
	}

  public: // helpers
	//! Converts an entity name to a valid file (or folder) name
	static QString ToValidFileName(QString name);

	//! Returns the sub-folder name corresponding to an entity of the DB tree
	/** Groups created when loading a file (named 'filename.ext (fullpath)') get the file base name.
	 **/
	static QString GetSubFolderName(const ccHObject* entity);

	//! Removes the entities that have an ancestor in the same list
	static ccHObject::Container RemoveDescendants(const ccHObject::Container& entities);

	//! Returns a filename that is not in the 'usedFilenames' set yet (and adds it to the set)
	/** A suffix (_1, _2, etc.) is added if necessary. The comparison is not case-sensitive (as on Windows).
	 **/
	static QString MakeUniqueFilename(QSet<QString>& usedFilenames, const QString& folder, const QString& baseName, const QString& extension);

  protected:
	void collectChildren(ccHObject* entity, const QString& relativeFolder);
	//! Returns the (unique) sub-folder of an entity
	QString subFolder(const ccHObject* entity, const QString& parentFolder);

	std::vector<File>       m_files;
	std::vector<ccHObject*> m_ignored;
	std::set<unsigned>      m_visited;
	//! Folder of each entity (by unique ID)
	std::map<unsigned, QString> m_entityFolders;
	//! Already used folders (in lower case)
	QSet<QString> m_usedFolders;
	unsigned      m_cloudCount    = 0;
	unsigned      m_meshCount     = 0;
	unsigned      m_polylineCount = 0;
};
