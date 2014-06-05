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

#ifndef CC_DB_ROOT_HEADER
#define CC_DB_ROOT_HEADER

//Qt
#include <QAbstractItemModel>
#include <QItemSelection>
#include <QPoint>
#include <QTreeView>

//CCLib
#include <CCConst.h>

//qCC_db
#include <ccObject.h>
#include <ccHObject.h>
#include <ccDrawableObject.h>

//System
#include <string.h>
#include <set>

class QStandardItemModel;
class QAction;
class ccPropertiesTreeDelegate;
class ccHObject;
class ccGLWindow;

//! Precise statistics about current selection
struct dbTreeSelectionInfo
{
	int selCount;

	int sfCount;
	int colorCount;
	int normalsCount;
	int octreeCount;
	int cloudCount;
	int polylineCount;
	int meshCount;
	int imageCount;
	int sensorCount;
	int gblSensorCount;
	int cameraSensorCount;
	int kdTreeCount;

	void reset()
	{
		memset(this,0,sizeof(dbTreeSelectionInfo));
	}
};

//! Custom QTreeView widget (for advanced selection behavior)
class ccCustomQTreeView : public QTreeView
{
public:

	//! Default constructor
	ccCustomQTreeView(QWidget* parent) : QTreeView(parent) {}

protected:
	virtual QItemSelectionModel::SelectionFlags selectionCommand(const QModelIndex& index, const QEvent* event=0) const;
};

//! GUI database tree root
class ccDBRoot : public QAbstractItemModel
{
	Q_OBJECT

public:

	//! Default constructor
	/** \param dbTreeWidget widget for DB tree display
		\param propertiesTreeWidget widget for selected entity's properties tree display
		\param parent widget QObject parent
	**/
	ccDBRoot(ccCustomQTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QObject* parent = 0);

	//! Destructor
	virtual ~ccDBRoot();

	//! Returns associated root object
	ccHObject* getRootEntity();

	//! Hides properties view
	void hidePropertiesView();
	//! Updates properties view
	void updatePropertiesView();

	//! Adds an element to the DB tree
	void addElement(ccHObject* anObject, bool autoExpand = true);

	//! Removes an element from the DB tree
	void removeElement(ccHObject* anObject);

	//! Finds an element in DB
	ccHObject* find(int uniqueID) const;

	//! Returns the number of selected entities in DB tree (optionally with a given type)
	int countSelectedEntities(CC_CLASS_ENUM filter = CC_TYPES::OBJECT);

	//! Returns selected entities in DB tree (optionally with a given type and additional information)
	int getSelectedEntities(ccHObject::Container& selEntities,
							CC_CLASS_ENUM filter = CC_TYPES::OBJECT,
							dbTreeSelectionInfo* info = NULL);

	//! Expands tree at a given node
	void expandElement(ccHObject* anObject, bool state);

	//! Selects a given entity
	/** If ctrl is pressed by the user at the same time,
		previous selection will be simply updated accordingly.
		\param obj entity to select
		\param forceAdditiveSelection whether to force additive selection (just as if CTRL key is pressed) or not
	**/
	void selectEntity(ccHObject* obj, bool forceAdditiveSelection = false);

	//! Unselects a given entity
	void unselectEntity(ccHObject* obj);

	//! Unselects all entities
	void unselectAllEntities();

	//! Unloads all entities
	void unloadAll();

	//inherited from QAbstractItemModel
	virtual QVariant data(const QModelIndex &index, int role) const;
	virtual QModelIndex index(int row, int column, const QModelIndex &parentIndex = QModelIndex()) const;
	virtual QModelIndex index(ccHObject* object);
	virtual QModelIndex parent(const QModelIndex &index) const;
	virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
	virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;
	virtual Qt::ItemFlags flags(const QModelIndex &index) const;
	virtual bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
	virtual Qt::DropActions supportedDropActions() const;
	virtual bool dropMimeData(const QMimeData* data, Qt::DropAction action, int row, int column, const QModelIndex& parent);
	virtual QMap<int,QVariant> itemData(const QModelIndex& index) const;
#ifdef CC_QT5
	virtual Qt::DropActions supportedDragActions() const { return Qt::MoveAction; }
#endif

public slots:
	void changeSelection(const QItemSelection & selected, const QItemSelection & deselected);
	void reflectObjectPropChange(ccHObject* obj);
	void redrawCCObject(ccHObject* anObject);
	void redrawCCObjectAndChildren(ccHObject* anObject);
	void updateCCObject(ccHObject* anObject);
	void deleteSelectedEntities();

	//! Shortcut to selectEntity(ccHObject*)
	void selectEntity(int uniqueID);

	//! Selects multiple entities at once
	/** If ctrl is pressed by the user at the same time,
		previous selection will be simply updated accordingly.
	**/
	void selectEntities(std::set<int> entIDs);

protected slots:
	void showContextMenu(const QPoint&);

	void expandBranch();
	void collapseBranch();
	void gatherRecursiveInformation();
	void sortSiblingsAZ();
	void sortSiblingsZA();
	void sortSiblingsType();
	void toggleSelectedEntities();
	void toggleSelectedEntitiesVisibility();
	void toggleSelectedEntitiesColor();
	void toggleSelectedEntitiesNormals();
	void toggleSelectedEntitiesSF();
	void toggleSelectedEntitiesMat();
	void toggleSelectedEntities3DName();
	void addEmptyGroup();
	void alignCameraWithEntityDirect() { alignCameraWithEntity(false); }
	void alignCameraWithEntityIndirect() { alignCameraWithEntity(true); }

signals:
	void selectionChanged();

protected:

	//! Aligns the camera with the currently selected entity
	/** \param reverse whether to use the entity's normal (false) or its inverse (true)
	**/
	void alignCameraWithEntity(bool reverse);

	//! Shows properties view for a given element
	void showPropertiesView(ccHObject* obj);

	//! Toggles a given property (enable state, visibility, normal, color, SF, etc.) on selected entities
	/** Properties are:
			0 - enable state
			1 - visibility
			2 - normal
			3 - color
			4 - SF
			5 - materials/textures
			6 - 3D name
	**/
	void toggleSelectedEntitiesProperty(unsigned prop);

	//! Entities sorting schemes
	enum SortRules { SORT_A2Z, SORT_Z2A, SORT_BY_TYPE };

	//! Sorts selected entities siblings
	void sortSelectedEntitiesSiblings(SortRules rule);

	//! Expands or collapses hovered item
	void expandOrCollapseHoveredBranch(bool expand);

	//! Associated DB root
	ccHObject* m_treeRoot;

	//! Associated widget for DB tree
	QTreeView* m_dbTreeWidget;

	//! Associated widget for selected entity's properties tree
	QTreeView* m_propertiesTreeWidget;

	//! Selected entity's properties data model
	QStandardItemModel* m_propertiesModel;
	//! Selected entity's properties delegate
	ccPropertiesTreeDelegate* m_ccPropDelegate;

	//! Context menu action: expand tree branch
	QAction* m_expandBranch;
	//! Context menu action: collapse tree branch
	QAction* m_collapseBranch;
	//! Context menu action: gather (recursive) information on selected entities
	QAction* m_gatherInformation;
	//! Context menu action: sort siblings in alphabetical order
	QAction* m_sortSiblingsAZ;
	//! Context menu action: sort siblings in reverse alphabetical order
	QAction* m_sortSiblingsZA;
	//! Context menu action: sort siblings by type
	QAction* m_sortSiblingsType;
	//! Context menu action: delete selected entities
	QAction* m_deleteSelectedEntities;
	//! Context menu action: enabled/disable selected entities
	QAction* m_toggleSelectedEntities;
	//! Context menu action: hide/show selected entities
	QAction* m_toggleSelectedEntitiesVisibility;
	//! Context menu action: hide/show selected entities color
	QAction* m_toggleSelectedEntitiesColor;
	//! Context menu action: hide/show selected entities normals
	QAction* m_toggleSelectedEntitiesNormals;
	//! Context menu action: hide/show selected entities materials/textures
	QAction* m_toggleSelectedEntitiesMat;
	//! Context menu action: hide/show selected entities SF
	QAction* m_toggleSelectedEntitiesSF;
	//! Context menu action: hide/show selected entities 3D name
	QAction* m_toggleSelectedEntities3DName;	
	//! Context menu action: add empty group
	QAction* m_addEmptyGroup;
	//! Context menu action: use 3-points labels or planes to orient camera
	QAction* m_alignCameraWithEntity;
	//! Context menu action: reverse of m_alignCameraWithEntity
	QAction* m_alignCameraWithEntityReverse;

	//! Last context menu pos
	QPoint m_contextMenuPos;

};

#endif
