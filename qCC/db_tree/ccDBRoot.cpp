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

#include "ccDBRoot.h"

//Local
#include "ccGLWindow.h"

//Qt
#include <QTreeView>
#include <QStandardItemModel>
#include <QHeaderView>
#include <QMenu>
#include <QMimeData>
#include <QMessageBox>
#include <QRegExp>

//qCC_db
#include <ccLog.h>
#include <ccHObject.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccMaterialSet.h>
#include <cc2DLabel.h>
#include <ccGenericPrimitive.h>
#include <ccPlane.h>
#include <ccPolyline.h>
#include <ccFacet.h>
#include <ccGBLSensor.h>

//CClib
#include <CCMiscTools.h>

//local
#include "ccPropertiesTreeDelegate.h"
#include "../mainwindow.h"
#include "../ccPickOneElementDlg.h"
#include "../ccSelectChildrenDlg.h"

//system
#include <assert.h>
#include <algorithm>
#include <string.h>

//Minimum width of the left column of the properties tree view
static const int c_propViewLeftColumnWidth = 115;

//test whether a cloud can be deleted or moved
static bool CanDetachCloud(const ccHObject* obj)
{
	if (!obj)
	{
		assert(false);
		return false;
	}

	ccHObject* parent = obj->getParent();
	if (!parent)
	{
		assert(false);
		return true;
	}

	//can't delete the vertices of a mesh or the verties of a polyline
	bool blocked = (	(parent->isKindOf(CC_TYPES::MESH) && (ccHObjectCaster::ToGenericMesh(parent)->getAssociatedCloud() == obj))
						||	(parent->isKindOf(CC_TYPES::POLY_LINE) && (dynamic_cast<ccPointCloud*>(ccHObjectCaster::ToPolyline(parent)->getAssociatedCloud()) == obj)) );

	return !blocked;
}

ccDBRoot::ccDBRoot(ccCustomQTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QObject* parent) : QAbstractItemModel(parent)
{
	m_treeRoot = new ccHObject("DB Tree");

	//DB Tree
	assert(dbTreeWidget);
	m_dbTreeWidget = dbTreeWidget;
	m_dbTreeWidget->setModel(this);
	m_dbTreeWidget->header()->hide();

	//drag & drop support
	m_dbTreeWidget->setDragEnabled(true);
	m_dbTreeWidget->setAcceptDrops(true);
	m_dbTreeWidget->setDropIndicatorShown(true);

	//already done in ui file!
	//m_dbTreeWidget->setDragDropMode(QAbstractItemView::InternalMove);
	//m_dbTreeWidget->setEditTriggers(QAbstractItemView::EditKeyPressed);
	//m_dbTreeWidget->setDragDropMode(QAbstractItemView::InternalMove);
	//m_dbTreeWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
	//m_dbTreeWidget->setUniformRowHeights(true);

	//context menu on DB tree elements
	m_dbTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
	m_expandBranch = new QAction("Expand branch",this);
	m_collapseBranch = new QAction("Collapse branch",this);
	m_gatherInformation = new QAction("Information (recursive)",this);
	m_sortChildrenType = new QAction("Sort children by type",this);
	m_sortChildrenAZ = new QAction("Sort children by name (A-Z)",this);
	m_sortChildrenZA = new QAction("Sort children by name (Z-A)",this);
	m_selectByTypeAndName = new QAction("Select children by type and/or name",this);
	m_deleteSelectedEntities = new QAction("Delete",this);
	m_toggleSelectedEntities = new QAction("Toggle",this);
	m_toggleSelectedEntitiesVisibility = new QAction("Toggle visibility",this);
	m_toggleSelectedEntitiesColor = new QAction("Toggle color",this);
	m_toggleSelectedEntitiesNormals = new QAction("Toggle normals",this);
	m_toggleSelectedEntitiesMat = new QAction("Toggle materials/textures",this);
	m_toggleSelectedEntitiesSF = new QAction("Toggle SF",this);
	m_toggleSelectedEntities3DName = new QAction("Toggle 3D name",this);
	m_addEmptyGroup = new QAction("Add empty group",this);
	m_alignCameraWithEntity = new QAction("Align camera",this);
	m_alignCameraWithEntityReverse = new QAction("Align camera (reverse)",this);
	m_enableBubbleViewMode = new QAction("Bubble-view",this);

	m_contextMenuPos = QPoint(-1,-1);

	//connect custom context menu actions
	connect(m_dbTreeWidget,						SIGNAL(customContextMenuRequested(const QPoint&)),	this, SLOT(showContextMenu(const QPoint&)));
	connect(m_expandBranch,						SIGNAL(triggered()),								this, SLOT(expandBranch()));
	connect(m_collapseBranch,					SIGNAL(triggered()),								this, SLOT(collapseBranch()));
	connect(m_gatherInformation,				SIGNAL(triggered()),								this, SLOT(gatherRecursiveInformation()));
	connect(m_sortChildrenAZ,					SIGNAL(triggered()),								this, SLOT(sortChildrenAZ()));
	connect(m_sortChildrenZA,					SIGNAL(triggered()),								this, SLOT(sortChildrenZA()));
	connect(m_sortChildrenType,					SIGNAL(triggered()),								this, SLOT(sortChildrenType()));
	connect(m_selectByTypeAndName,              SIGNAL(triggered()),								this, SLOT(selectByTypeAndName()));
	connect(m_deleteSelectedEntities,			SIGNAL(triggered()),								this, SLOT(deleteSelectedEntities()));
	connect(m_toggleSelectedEntities,			SIGNAL(triggered()),								this, SLOT(toggleSelectedEntities()));
	connect(m_toggleSelectedEntitiesVisibility,	SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesVisibility()));
	connect(m_toggleSelectedEntitiesColor,		SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesColor()));
	connect(m_toggleSelectedEntitiesNormals,	SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesNormals()));
	connect(m_toggleSelectedEntitiesMat,		SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesMat()));
	connect(m_toggleSelectedEntitiesSF,			SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesSF()));
	connect(m_toggleSelectedEntities3DName,		SIGNAL(triggered()),								this, SLOT(toggleSelectedEntities3DName()));
	connect(m_addEmptyGroup,					SIGNAL(triggered()),								this, SLOT(addEmptyGroup()));
	connect(m_alignCameraWithEntity,			SIGNAL(triggered()),								this, SLOT(alignCameraWithEntityDirect()));
	connect(m_alignCameraWithEntityReverse,		SIGNAL(triggered()),								this, SLOT(alignCameraWithEntityIndirect()));
	connect(m_enableBubbleViewMode,				SIGNAL(triggered()),								this, SLOT(enableBubbleViewMode()));

	//other DB tree signals/slots connection
	connect(m_dbTreeWidget->selectionModel(), SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)), this, SLOT(changeSelection(const QItemSelection&, const QItemSelection&)));

	//Properties Tree
	assert(propertiesTreeWidget);
	m_propertiesTreeWidget = propertiesTreeWidget;
	m_propertiesModel = new QStandardItemModel(0, 2, parent);
	m_ccPropDelegate = new ccPropertiesTreeDelegate(m_propertiesModel, m_propertiesTreeWidget);
	m_propertiesTreeWidget->setItemDelegate(m_ccPropDelegate);
	m_propertiesTreeWidget->setModel(m_propertiesModel);
	//already done in ui file!
	//m_propertiesTreeWidget->setSelectionMode(QAbstractItemView::NoSelection);
	//m_propertiesTreeWidget->setAllColumnsShowFocus(true);
	//m_propertiesTreeWidget->header()->setStretchLastSection(true);
	m_propertiesTreeWidget->header()->setSectionResizeMode(QHeaderView::Interactive);
	m_propertiesTreeWidget->setEnabled(false);

	//Properties tree signals/slots connection
	connect(m_ccPropDelegate, SIGNAL(ccObjectPropertiesChanged(ccHObject*)), this, SLOT(updateCCObject(ccHObject*)));
	connect(m_ccPropDelegate, SIGNAL(ccObjectAppearanceChanged(ccHObject*)), this, SLOT(redrawCCObject(ccHObject*)));
	connect(m_ccPropDelegate, SIGNAL(ccObjectAndChildrenAppearanceChanged(ccHObject*)), this, SLOT(redrawCCObjectAndChildren(ccHObject*)));
}

ccDBRoot::~ccDBRoot()
{
	if (m_ccPropDelegate)
		delete m_ccPropDelegate;

	if (m_propertiesModel)
		delete m_propertiesModel;

	if (m_treeRoot)
		delete m_treeRoot;
}

void ccDBRoot::unloadAll()
{
	if (!m_treeRoot)
	{
		return;
	}

	while (m_treeRoot->getChildrenNumber() > 0)
	{
		int i = static_cast<int>(m_treeRoot->getChildrenNumber())-1;
		ccHObject* object = m_treeRoot->getChild(i);
		assert(object);

		object->prepareDisplayForRefresh_recursive();

		beginRemoveRows(index(object).parent(),i,i);
		m_treeRoot->removeChild(i);
		endRemoveRows();
	}

	emit dbIsEmpty();

	updatePropertiesView();

	MainWindow::RefreshAllGLWindow(false);
}

ccHObject* ccDBRoot::getRootEntity()
{
	return m_treeRoot;
}

void ccDBRoot::addElement(ccHObject* object, bool autoExpand/*=true*/)
{
	if (!m_treeRoot)
	{
		assert(false);
		return;
	}
	if (!object)
	{
		assert(false);
		return;
	}

	bool wasEmpty = (m_treeRoot->getChildrenNumber() == 0);

	//look for object's parent
	ccHObject* parentObject = object->getParent();
	if (!parentObject)
	{
		//if the object has no parent, it will be inserted at tree root
		parentObject = m_treeRoot;
		m_treeRoot->addChild(object);
	}
	else
	{
		//DGM TODO: how could we check that the object is not already inserted in the DB tree?
		//The double insertion can cause serious damage to it (not sure why excatly though).

		//The code below doesn't work because the 'index' method will always return a valid index
		//as soon as the object has a parent (index creation is a purely 'logical' approach)
		//QModelIndex nodeIndex = index(object);
		//if (nodeIndex.isValid())
		//	return;
	}

	//look for insert node index in tree
	QModelIndex insertNodeIndex = index(parentObject);
	int childPos = parentObject->getChildIndex(object);

	//row insertion operation (start)
	beginInsertRows(insertNodeIndex, childPos, childPos);

	//row insertion operation (end)
	endInsertRows();

	if (autoExpand)
	{
		//expand the parent (just in case)
		m_dbTreeWidget->expand(index(parentObject));
		//and the child
		m_dbTreeWidget->expand(index(object));
	}
	else //if (parentObject)
	{
		m_dbTreeWidget->expand(insertNodeIndex);
	}

	if (wasEmpty && m_treeRoot->getChildrenNumber() != 0)
	{
		emit dbIsNotEmptyAnymore();
	}
}

void ccDBRoot::expandElement(ccHObject* object, bool state)
{
	if (!object || !m_dbTreeWidget)
	{
		return;
	}

	m_dbTreeWidget->setExpanded(index(object), state);
}

void ccDBRoot::removeElements(ccHObject::Container& objects)
{
	if (objects.empty())
	{
		assert(false);
		return;
	}

	//we hide properties view in case this is the deleted object that is currently selected
	hidePropertiesView();

	//every object in tree must have a parent!
	for (ccHObject* object : objects)
	{
		ccHObject* parent = object->getParent();
		if (!parent)
		{
			ccLog::Warning(QString("[ccDBRoot::removeElements] Internal error: object '%1' has no parent").arg(object->getName()));
			continue;
		}

		//just in case
		object->prepareDisplayForRefresh();

		int childPos = parent->getChildIndex(object);
		assert(childPos >= 0);
		{
			//row removal operation (start)
			beginRemoveRows(index(parent), childPos, childPos);

			parent->removeChild(childPos);

			//row removal operation (end)
			endRemoveRows();
		}
	}

	//we restore properties view
	updatePropertiesView();

	if (m_treeRoot->getChildrenNumber() == 0)
	{
		emit dbIsEmpty();
	}
}

void ccDBRoot::removeElement(ccHObject* object)
{
	if (!object)
	{
		assert(false);
		return;
	}

	//we hide properties view in case this is the deleted object that is currently selected
	hidePropertiesView();

	//every object in tree must have a parent!
	ccHObject* parent = object->getParent();
	if (!parent)
	{
		ccLog::Warning("[ccDBRoot::removeElement] Internal error: object has no parent");
		return;
	}

	//just in case
	object->prepareDisplayForRefresh();

	int childPos = parent->getChildIndex(object);
	assert(childPos >= 0);
	{
		//row removal operation (start)
		beginRemoveRows(index(parent), childPos, childPos);

		parent->removeChild(childPos);

		//row removal operation (end)
		endRemoveRows();
	}

	//we restore properties view
	updatePropertiesView();

	if (m_treeRoot->getChildrenNumber() == 0)
	{
		emit dbIsEmpty();
	}
}

void ccDBRoot::deleteSelectedEntities()
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	if (selectedIndexes.size() < 1)
	{
		return;
	}
	unsigned selCount = static_cast<unsigned>(selectedIndexes.size());

	hidePropertiesView();
	bool verticesWarningIssued = false;

	//we remove all objects that are children of other deleted ones!
	//(otherwise we may delete the parent before the child!)
	//TODO DGM: not sure this is still necessary with the new dependency mechanism
	std::vector<ccHObject*> toBeDeleted;
	for (unsigned i = 0; i < selCount; ++i)
	{
		ccHObject* obj = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		//we don't take care of parent-less objects (i.e. the tree root)
		if (!obj->getParent() || obj->isLocked())
		{
			ccLog::Warning(QString("Object '%1' can't be deleted this way (locked)").arg(obj->getName()));
			continue;
		}

		//we don't consider objects that are 'descendent' of others in the selection
		bool isDescendent = false;
		for (unsigned j = 0; j < selCount; ++j)
		{
			if (i != j)
			{
				ccHObject* otherObj = static_cast<ccHObject*>(selectedIndexes[j].internalPointer());
				if (otherObj->isAncestorOf(obj))
				{
					isDescendent = true;
					break;
				}
			}
		}

		if (!isDescendent)
		{
			//last check: mesh vertices
			if (obj->isKindOf(CC_TYPES::POINT_CLOUD) && !CanDetachCloud(obj))
			{
				if (!verticesWarningIssued)
				{
					ccLog::Warning("Vertices can't be deleted without their parent mesh");
					verticesWarningIssued = true;
				}
				continue;
			}

			toBeDeleted.push_back(obj);
		}
	}

	qism->clear();

	while (!toBeDeleted.empty())
	{
		ccHObject* object = toBeDeleted.back();
		assert(object);
		toBeDeleted.pop_back();

		object->prepareDisplayForRefresh_recursive();

		if (object->isKindOf(CC_TYPES::MESH))
		{
			//specific case: the object is a mesh and its parent is its vertices!
			//(can happen if a Delaunay mesh is computed directly in CC)
			if (object->getParent() && object->getParent() == ccHObjectCaster::ToGenericMesh(object)->getAssociatedCloud())
			{
				object->getParent()->setVisible(true);
			}
		}

		ccHObject* parent = object->getParent();
		int childPos = parent->getChildIndex(object);
		assert(childPos >= 0);

		beginRemoveRows(index(object).parent(), childPos, childPos);
		parent->removeChild(childPos);
		endRemoveRows();
	}

	updatePropertiesView();

	if (m_treeRoot->getChildrenNumber() == 0)
	{
		emit dbIsEmpty();
	}

	MainWindow::RefreshAllGLWindow(false);
}

QVariant ccDBRoot::data(const QModelIndex &index, int role) const
{
	if (!index.isValid())
	{
		return QVariant();
	}

	const ccHObject *item = static_cast<ccHObject*>(index.internalPointer());
	assert(item);
	if (!item)
	{
		return QVariant();
	}

	switch (role)
	{
	case Qt::DisplayRole:
	{
		QString baseName(item->getName());
		if (baseName.isEmpty())
			baseName = QStringLiteral("no name");
		//specific case
		if (item->isA(CC_TYPES::LABEL_2D))
			baseName = QStringLiteral("2D label: ")+baseName;
		else if (item->isA(CC_TYPES::VIEWPORT_2D_LABEL))
			baseName = QStringLiteral("2D area label: ")+baseName;

		return QVariant(baseName);
	}
	
	case Qt::EditRole:
	{
		return QVariant(item->getName());
	}

	case Qt::DecorationRole:
	{
		// does the object have an "embedded icon"? - It may be the case for ccHObject defined in plugins
		QIcon icon = item->getIcon();
		if (!icon.isNull())
		{
			return icon;
		}

		bool locked = item->isLocked();
		switch (item->getClassID())
		{
		case CC_TYPES::HIERARCHY_OBJECT:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbHObjectSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbHObjectSymbol.png"));
		case CC_TYPES::POINT_CLOUD:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbCloudSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbCloudSymbol.png"));
			//all primitives
		case CC_TYPES::PLANE:
		case CC_TYPES::SPHERE:
		case CC_TYPES::TORUS:
		case CC_TYPES::CYLINDER:
		case CC_TYPES::CONE:
		case CC_TYPES::BOX:
		case CC_TYPES::DISH:
		case CC_TYPES::EXTRU:
		case CC_TYPES::FACET:
		case CC_TYPES::QUADRIC:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbMiscGeomSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbMiscGeomSymbol.png"));
		case CC_TYPES::MESH:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbMeshSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbMeshSymbol.png"));
		case CC_TYPES::MESH_GROUP:
		case CC_TYPES::SUB_MESH:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbSubMeshSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbSubMeshSymbol.png"));
		case CC_TYPES::POLY_LINE:
			return QIcon(QStringLiteral(":/CC/images/dbPolylineSymbol.png"));
		case CC_TYPES::POINT_OCTREE:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbOctreeSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbOctreeSymbol.png"));
		case CC_TYPES::CALIBRATED_IMAGE:
			return QIcon(QStringLiteral(":/CC/images/dbCalibratedImageSymbol.png"));
		case CC_TYPES::IMAGE:
			return QIcon(QStringLiteral(":/CC/images/dbImageSymbol.png"));
		case CC_TYPES::SENSOR:
		case CC_TYPES::GBL_SENSOR:
			return QIcon(QStringLiteral(":/CC/images/dbGBLSensorSymbol.png"));
		case CC_TYPES::CAMERA_SENSOR:
			return QIcon(QStringLiteral(":/CC/images/dbCamSensorSymbol.png"));
		case CC_TYPES::MATERIAL_SET:
			return QIcon(QStringLiteral(":/CC/images/dbMaterialSymbol.png"));
		case CC_TYPES::NORMALS_ARRAY:
		case CC_TYPES::NORMAL_INDEXES_ARRAY:
		case CC_TYPES::RGB_COLOR_ARRAY:
		case CC_TYPES::TEX_COORDS_ARRAY:
		case CC_TYPES::TRANS_BUFFER:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbContainerSymbolLocked.png"));
			else
				return QIcon(QStringLiteral(":/CC/images/dbContainerSymbol.png"));
		case CC_TYPES::LABEL_2D:
			return QIcon(QStringLiteral(":/CC/images/dbLabelSymbol.png"));
		case CC_TYPES::VIEWPORT_2D_OBJECT:
			return QIcon(QStringLiteral(":/CC/images/dbViewportSymbol.png"));
		case CC_TYPES::VIEWPORT_2D_LABEL:
			return QIcon(QStringLiteral(":/CC/images/dbAreaLabelSymbol.png"));
		default:
			if (locked)
				return QIcon(QStringLiteral(":/CC/images/dbLockSymbol.png"));
			else
				return QVariant();
		}
		break;
	}

	case Qt::CheckStateRole:
	{
		if (item->isEnabled())
			return Qt::Checked;
		else
			return Qt::Unchecked;
	}

	default:
		//unhandled role
		break;
	}

	return QVariant();
}

bool ccDBRoot::setData(const QModelIndex &index, const QVariant &value, int role)
{
	if (index.isValid())
	{
		if (role == Qt::EditRole)
		{
			if (value.toString().isEmpty())
			{
				return false;
			}

			ccHObject *item = static_cast<ccHObject*>(index.internalPointer());
			assert(item);
			if (item)
			{
				item->setName(value.toString());

				//particular cases:
				// - labels name is their title (so we update them)
				// - name might be displayed in 3D
				if (item->nameShownIn3D() || item->isKindOf(CC_TYPES::LABEL_2D))
					if (item->isEnabled() && item->isVisible() && item->getDisplay())
						item->getDisplay()->redraw();

				reflectObjectPropChange(item);

				emit dataChanged(index, index);
			}

			return true;
		}
		else if (role == Qt::CheckStateRole)
		{
			ccHObject *item = static_cast<ccHObject*>(index.internalPointer());
			assert(item);
			if (item)
			{
				if (value == Qt::Checked)
					item->setEnabled(true);
				else
					item->setEnabled(false);

				redrawCCObjectAndChildren(item);
				//reflectObjectPropChange(item);
			}

			return true;
		}
	}

	return false;
}


QModelIndex ccDBRoot::index(int row, int column, const QModelIndex &parentIndex) const
{
	if (!hasIndex(row, column, parentIndex))
	{
		return QModelIndex();
	}

	ccHObject *parent = (parentIndex.isValid() ? static_cast<ccHObject*>(parentIndex.internalPointer()) : m_treeRoot);
	assert(parent);
	if (!parent)
	{
		return QModelIndex();
	}
	
	ccHObject *child = parent->getChild(row);
	return child ? createIndex(row, column, child) : QModelIndex();
}

QModelIndex ccDBRoot::index(ccHObject* object)
{
	assert(object);

	if (object == m_treeRoot)
	{
		return QModelIndex();
	}

	ccHObject* parent = object->getParent();
	if (!parent)
	{
		ccLog::Error(QString("An error occurred while creating DB tree index: object '%1' has no parent").arg(object->getName()));
		return QModelIndex();
	}

	int pos = parent->getChildIndex(object);
	assert(pos >= 0);

	return createIndex(pos, 0, object);
}

QModelIndex ccDBRoot::parent(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return QModelIndex();
	}

	ccHObject *childItem = static_cast<ccHObject*>(index.internalPointer());
	if (!childItem)
	{
		assert(false);
		return QModelIndex();
	}
	ccHObject *parentItem = childItem->getParent();

	assert(parentItem);
	if (!parentItem || parentItem == m_treeRoot)
	{
		return QModelIndex();
	}

	return createIndex(parentItem->getIndex(), 0, parentItem);
}

int ccDBRoot::rowCount(const QModelIndex &parent) const
{
	ccHObject *parentItem = 0;
	if (!parent.isValid())
		parentItem = m_treeRoot;
	else
		parentItem = static_cast<ccHObject*>(parent.internalPointer());

	assert(parentItem);
	return (parentItem ? parentItem->getChildrenNumber() : 0);
}

int ccDBRoot::columnCount(const QModelIndex &parent) const
{
	return 1;
	//if (parent.isValid())
	//	return static_cast<ccHObject*>(parent.internalPointer())->columnCount();
	//else
	//	return m_treeRoot->columnCount();
}

void ccDBRoot::changeSelection(const QItemSelection & selected, const QItemSelection & deselected)
{
	//first unselect
	QModelIndexList deselectedItems = deselected.indexes();
	{
		for (int i = 0; i < deselectedItems.count(); ++i)
		{
			ccHObject* element = static_cast<ccHObject*>(deselectedItems.at(i).internalPointer());
			assert(element);
			if (element)
			{
				element->setSelected(false);
				element->prepareDisplayForRefresh();
			}
		}
	}

	//then select
	QModelIndexList selectedItems = selected.indexes();
	{
		for (int i = 0; i < selectedItems.count(); ++i)
		{
			ccHObject* element = static_cast<ccHObject*>(selectedItems.at(i).internalPointer());
			assert(element);
			if (element)
			{
				element->setSelected(true);
				element->prepareDisplayForRefresh();
			}
		}
	}

	updatePropertiesView();

	MainWindow::RefreshAllGLWindow();

	emit selectionChanged();
}

void ccDBRoot::unselectEntity(ccHObject* obj)
{
	if (obj && obj->isSelected())
	{
		QModelIndex objIndex = index(obj);
		if (objIndex.isValid())
		{
			QItemSelectionModel* selectionModel = m_dbTreeWidget->selectionModel();
			assert(selectionModel);
			selectionModel->select(objIndex, QItemSelectionModel::Deselect);
		}
	}
}

void ccDBRoot::unselectAllEntities()
{
	QItemSelectionModel* selectionModel = m_dbTreeWidget->selectionModel();
	assert(selectionModel);

	selectionModel->clear();
}

void ccDBRoot::selectEntity(ccHObject* obj, bool forceAdditiveSelection/*=false*/)
{
	bool additiveSelection = forceAdditiveSelection || (QApplication::keyboardModifiers () & Qt::ControlModifier);

	QItemSelectionModel* selectionModel = m_dbTreeWidget->selectionModel();
	assert(selectionModel);

	//valid object? then we will try to select (or toggle) it
	if (obj)
	{
		QModelIndex selectedIndex = index(obj);
		if (selectedIndex.isValid())
		{
			//if CTRL is pushed (or additive selection is forced)
			if (additiveSelection)
			{
				//default case: toggle current item selection state
				if (!obj->isSelected())
				{
					QModelIndexList selectedIndexes = selectionModel->selectedIndexes();
					if (!selectedIndexes.empty())
					{
						//special case: labels can only be merged with labels!
						if (obj->isA(CC_TYPES::LABEL_2D) != static_cast<ccHObject*>(selectedIndexes[0].internalPointer())->isA(CC_TYPES::LABEL_2D))
						{
							ccLog::Warning("[Selection] Labels and other entities can't be mixed (release the CTRL key to start a new selection)");
							return;
						}
					}
				}
				selectionModel->select(selectedIndex,QItemSelectionModel::Toggle);
				obj->setSelected(true);
			}
			else
			{
				if (selectionModel->isSelected(selectedIndex))	//nothing to do
					return;
				selectionModel->select(selectedIndex,QItemSelectionModel::ClearAndSelect);
				obj->setSelected(true);
			}

			//hack: auto-scroll to selected element
			if (obj->isSelected() && !additiveSelection)
				m_dbTreeWidget->scrollTo(selectedIndex);
		}
	}
	//otherwise we clear current selection (if CTRL is not pushed)
	else if (!additiveSelection)
	{
		selectionModel->clear();
	}
}

void ccDBRoot::selectEntities(std::unordered_set<int> entIDs)
{
	bool ctrlPushed = (QApplication::keyboardModifiers () & Qt::ControlModifier);

	//convert input list of IDs to proper entities
	ccHObject::Container entities;
	{
		try
		{
			entities.reserve(entIDs.size());
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning("[ccDBRoot::selectEntities] Not enough memory");
			return;
		}

		for (std::unordered_set<int>::const_iterator it = entIDs.begin(); it != entIDs.end(); ++it)
		{
			ccHObject* obj = find(*it);
			if (obj)
				entities.push_back(obj);
		}
	}

	selectEntities(entities, ctrlPushed);
}

void ccDBRoot::selectEntities(const ccHObject::Container& entities, bool incremental/*=false*/)
{
	//selection model
	QItemSelectionModel* selectionModel = m_dbTreeWidget->selectionModel();
	assert(selectionModel);

	//count the number of lables
	size_t labelCount = 0;
	{
		for (size_t i = 0; i < entities.size(); ++i)
		{
			ccHObject* ent = entities[i];
			if (!ent)
			{
				assert(false);
				continue;
			}
			if (ent->isA(CC_TYPES::LABEL_2D))
				++labelCount;
		}
	}

	//create new selection structure
	QItemSelection newSelection;
	{
		//shall we keep labels?
		bool keepLabels = false;
		{
			QModelIndexList formerSelectedIndexes = selectionModel->selectedIndexes();
			if (formerSelectedIndexes.isEmpty() || !incremental)
				keepLabels = (labelCount == entities.size()); //yes if they are the only selected entities
			else if (incremental)
				keepLabels = static_cast<ccHObject*>(formerSelectedIndexes[0].internalPointer())->isA(CC_TYPES::LABEL_2D); //yes if previously selected entities were already labels
		}

		for (size_t i = 0; i < entities.size(); ++i)
		{
			ccHObject* ent = entities[i];
			if (ent)
			{
				//filter input selection (can't keep both labels and standard entities --> we can't mix them!)
				bool isLabel = ent->isA(CC_TYPES::LABEL_2D);
				if (isLabel == keepLabels && (!incremental || !ent->isSelected()))
				{
					QModelIndex selectedIndex = index(ent);
					if (selectedIndex.isValid())
						newSelection.merge(QItemSelection(selectedIndex,selectedIndex),QItemSelectionModel::Select);
				}
			}
		}
	}

	//default behavior: clear previous selection if CTRL is not pushed
	selectionModel->select(newSelection,incremental ? QItemSelectionModel::Select : QItemSelectionModel::ClearAndSelect);
}

ccHObject* ccDBRoot::find(int uniqueID) const
{
	return m_treeRoot->find(uniqueID);
}

void ccDBRoot::showPropertiesView(ccHObject* obj)
{
	m_ccPropDelegate->fillModel(obj);

	m_propertiesTreeWidget->setEnabled(true);
	m_propertiesTreeWidget->setColumnWidth(0, c_propViewLeftColumnWidth);
	//m_propertiesTreeWidget->setColumnWidth(1, m_propertiesTreeWidget->width() - c_propViewLeftColumnWidth);
}

void ccDBRoot::hidePropertiesView()
{
	m_ccPropDelegate->unbind();
	m_propertiesModel->clear();
	m_propertiesTreeWidget->setEnabled(false);
}

void ccDBRoot::reflectObjectPropChange(ccHObject* obj)
{
	assert(m_ccPropDelegate);
	assert(m_propertiesTreeWidget);
	if (!m_propertiesTreeWidget->isEnabled() || m_ccPropDelegate->getCurrentObject() != obj)
	{
		showPropertiesView(obj);
	}
}

void ccDBRoot::updatePropertiesView()
{
	assert(m_dbTreeWidget);
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	if (selectedIndexes.size() == 1)
		showPropertiesView(static_cast<ccHObject*>(selectedIndexes[0].internalPointer()));
	else
		hidePropertiesView();
}

void ccDBRoot::updateCCObject(ccHObject* object)
{
	assert(object);

	QModelIndex idx = index(object);

	if (idx.isValid())
		emit dataChanged(idx,idx);
}

void ccDBRoot::redrawCCObject(ccHObject* object)
{
	assert(object);

	object->redrawDisplay();
}

void ccDBRoot::redrawCCObjectAndChildren(ccHObject* object)
{
	assert(object);

	object->prepareDisplayForRefresh_recursive();
	object->refreshDisplay_recursive(/*only2D=*/false);
}

int ccDBRoot::countSelectedEntities(CC_CLASS_ENUM filter)
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	int selCount = selectedIndexes.size();

	if (selCount == 0 || filter == CC_TYPES::OBJECT)
		return selCount;

	int realCount = 0;
	for (int i = 0; i < selCount; ++i)
	{
		ccHObject* object = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		if (object && object->isKindOf(filter))
			++realCount;
	}

	return realCount;
}

size_t ccDBRoot::getSelectedEntities(	ccHObject::Container& selectedEntities,
										CC_CLASS_ENUM filter/*=CC_TYPES::OBJECT*/,
										dbTreeSelectionInfo* info/*=NULL*/ )
{
	selectedEntities.clear();

	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();

	try
	{
		int selCount = selectedIndexes.size();
		for (int i = 0; i < selCount; ++i)
		{
			ccHObject* object = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
			if (object && object->isKindOf(filter))
				selectedEntities.push_back(object);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory!
	}

	if (info)
	{
		info->reset();
		info->selCount = selectedIndexes.size();

		for (size_t i = 0; i < info->selCount; ++i)
		{
			ccHObject* obj = selectedEntities[i];

			info->sfCount += obj->hasScalarFields() ? 1 : 0;
			info->colorCount += obj->hasColors() ? 1 : 0;
			info->normalsCount += obj->hasNormals() ? 1 : 0;

			if (obj->isA(CC_TYPES::HIERARCHY_OBJECT))
			{
				info->groupCount++;
			}
			else if(obj->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				ccGenericPointCloud* genericCloud = ccHObjectCaster::ToGenericPointCloud(obj);
				info->cloudCount++;
				info->octreeCount += genericCloud->getOctree() != NULL ? 1 : 0;
				ccPointCloud* qccCloud = ccHObjectCaster::ToPointCloud(obj);
				info->gridCound += qccCloud->gridCount();
			}
			else if (obj->isKindOf(CC_TYPES::MESH))
			{
				info->meshCount++;

				if (obj->isKindOf(CC_TYPES::PLANE))
					info->planeCount++;
			}
			else if (obj->isKindOf(CC_TYPES::POLY_LINE))
			{
				info->polylineCount++;
			}
			else if(obj->isKindOf(CC_TYPES::SENSOR))
			{
				info->sensorCount++;
				if (obj->isKindOf(CC_TYPES::GBL_SENSOR))
					info->gblSensorCount++;
				if (obj->isKindOf(CC_TYPES::CAMERA_SENSOR))
					info->cameraSensorCount++;
			}
			else if (obj->isKindOf(CC_TYPES::POINT_KDTREE))
			{
				info->kdTreeCount++;
			}
		}
	}

	return selectedEntities.size();
}

Qt::DropActions ccDBRoot::supportedDropActions() const
{
	return Qt::MoveAction;
}

Qt::ItemFlags ccDBRoot::flags(const QModelIndex &index) const
{
	if (!index.isValid())
		return 0;

	Qt::ItemFlags defaultFlags = QAbstractItemModel::flags(index);

	//common flags
	defaultFlags |= (Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable);

	//class type based filtering
	const ccHObject *item = static_cast<ccHObject*>(index.internalPointer());
	assert(item);
	if (item && !item->isLocked()) //locked items cannot be drag-dropped
	{
		if (item->isA(CC_TYPES::HIERARCHY_OBJECT)								||
			(item->isKindOf(CC_TYPES::POINT_CLOUD) && CanDetachCloud(item))		|| //vertices can't be displaced
			(item->isKindOf(CC_TYPES::MESH) && !item->isA(CC_TYPES::SUB_MESH))	|| //a sub-mesh can't leave its parent mesh
			item->isKindOf(CC_TYPES::IMAGE)										||
			item->isKindOf(CC_TYPES::LABEL_2D)									||
			item->isKindOf(CC_TYPES::CAMERA_SENSOR)								||
			item->isKindOf(CC_TYPES::PRIMITIVE)									||
			item->isKindOf(CC_TYPES::FACET)										||
			item->isKindOf(CC_TYPES::CUSTOM_H_OBJECT))

		{
			defaultFlags |= (Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
		}
		else if (item->isKindOf(CC_TYPES::POLY_LINE))
		{
			const ccPolyline* poly = static_cast<const ccPolyline*>(item);
			//we can only displace a polyline if it is not dependent on it's father!
			const ccHObject* polyVertices = dynamic_cast<const ccHObject*>(poly->getAssociatedCloud());
			if (polyVertices != poly->getParent())
			{
				defaultFlags |= (Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
			}
		}
		else if (item->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT))
		{
			defaultFlags |= Qt::ItemIsDragEnabled;
		}
	}

	return defaultFlags;
}

QMap<int,QVariant> ccDBRoot::itemData(const QModelIndex& index) const
{
	QMap<int,QVariant> map = QAbstractItemModel::itemData(index);

	if (index.isValid())
	{
		ccHObject* object = static_cast<ccHObject*>(index.internalPointer());
		if (object)
			map.insert(Qt::UserRole,QVariant(object->getUniqueID()));
	}

	return map;
}


bool ccDBRoot::dropMimeData(const QMimeData* data, Qt::DropAction action, int destRow, int destColumn, const QModelIndex& destParent)
{
	if (action != Qt::MoveAction)
	{
		return false;
	}

	//default mime type for QAbstractItemModel items)
	if (!data->hasFormat("application/x-qabstractitemmodeldatalist"))
	{
		return false;
	}

	//new parent (can't be a leaf object!)
	ccHObject* newParent = destParent.isValid() ? static_cast<ccHObject*>(destParent.internalPointer()) : m_treeRoot;
	if (newParent && newParent->isLeaf())
	{
		return false;
	}

	//decode data
	QByteArray encoded = data->data("application/x-qabstractitemmodeldatalist");
	QDataStream stream(&encoded, QIODevice::ReadOnly);
	while (!stream.atEnd())
	{
		//decode current item index data (row, col, data 'roles' map)
		int srcRow, srcCol;
		QMap<int, QVariant> roleDataMap;
		stream >> srcRow >> srcCol >> roleDataMap;
		if (!roleDataMap.contains(Qt::UserRole))
			continue;

		//selected item
		int uniqueID = roleDataMap.value(Qt::UserRole).toInt();
		ccHObject *item = m_treeRoot->find(uniqueID);
		if (!item)
			continue;
		//ccLog::Print(QString("[Drag & Drop] Source: %1").arg(item->getName()));

		//old parent
		ccHObject* oldParent = item->getParent();
		//ccLog::Print(QString("[Drag & Drop] Parent: %1").arg(oldParent ? oldParent->getName() : "none")));

		//let's check if we can actually move the entity
		if (oldParent)
		{
			if (item->isKindOf(CC_TYPES::POINT_CLOUD))
			{
				//point cloud == mesh vertices?
				if (oldParent->isKindOf(CC_TYPES::MESH) && ccHObjectCaster::ToGenericMesh(oldParent)->getAssociatedCloud() == item)
				{
					if (oldParent != newParent)
					{
						ccLog::Error("Vertices can't leave their parent mesh");
						return false;
					}
				}
			}
			else if (item->isKindOf(CC_TYPES::MESH))
			{
				//a sub-mesh can't leave its parent mesh
				if (item->isA(CC_TYPES::SUB_MESH))
				{
					assert(false);
					ccLog::Error("Sub-meshes can't leave their mesh group");
					return false;
				}
				//a mesh can't leave its associated cloud
				else if (oldParent->isKindOf(CC_TYPES::POINT_CLOUD) && ccHObjectCaster::ToGenericMesh(item)->getAssociatedCloud() == oldParent)
				{
					if (oldParent != newParent)
					{
						ccLog::Error("Meshes can't leave their associated cloud (vertices set)");
						return false;
					}
				}
			}
			else if (/*item->isKindOf(CC_TYPES::PRIMITIVE) || */item->isKindOf(CC_TYPES::IMAGE))
			{
				if (oldParent != newParent)
				{
					ccLog::Error("This kind of entity can't leave their parent");
					return false;
				}
			}
			else if (oldParent != newParent)
			{
				//a label or a group of labels can't be moved to another cloud!
				//ccHObject::Container labels;
				//if (item->isA(CC_TYPES::LABEL_2D))
				//	labels.push_back(item);
				//else
				//	item->filterChildren(labels, true, CC_TYPES::LABEL_2D);

				////for all labels in the sub-tree
				//for (ccHObject::Container::const_iterator it = labels.begin(); it != labels.end(); ++it)
				//{
				//	if ((*it)->isA(CC_TYPES::LABEL_2D)) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
				//	{
				//		cc2DLabel* label = static_cast<cc2DLabel*>(*it);
				//		bool canMove = false;
				//		for (unsigned j = 0; j < label->size(); ++j)
				//		{
				//			assert(label->getPoint(j).cloud);
				//			//3 options to allow moving a label:
				//			if (item->isAncestorOf(label->getPoint(j).cloud) //label's cloud is inside sub-tree
				//				|| newParent == label->getPoint(j).cloud //destination is label's cloud
				//				|| label->getPoint(j).cloud->isAncestorOf(newParent)) //destination is below label's cloud
				//			{
				//				canMove = true;
				//				break;
				//			}
				//		}

				//		if (!canMove)
				//		{
				//			ccLog::Error("Labels (or group of) can't leave their parent");
				//			return false;
				//		}
				//	}
				//}
			}
		}

		//special case: moving an item inside the same 'parent'
		if (oldParent && newParent == oldParent)
		{
			int oldRow = oldParent->getChildIndex(item);
			if (destRow < 0)
			{
				assert(oldParent->getChildrenNumber() != 0);
				destRow = static_cast<int>(oldParent->getChildrenNumber())-1;
			}
			else if (oldRow < destRow)
			{
				assert(destRow > 0);
				--destRow;
			}
			else if (oldRow == destRow)
			{
				return false; //nothing to do
			}
		}

		//remove link with old parent (only CHILD/PARENT related flags!)
		int itemDependencyFlags = item->getDependencyFlagsWith(oldParent); //works even with NULL
		int fatherDependencyFlags = oldParent ? oldParent->getDependencyFlagsWith(item) : 0;
		if (oldParent)
		{
			oldParent->removeDependencyFlag(item,ccHObject::DP_PARENT_OF_OTHER);
			item->removeDependencyFlag(oldParent,ccHObject::DP_PARENT_OF_OTHER);
		}

		//remove item from current position
		removeElement(item);

		//sets new parent
		assert(newParent);
		newParent->addChild(item,fatherDependencyFlags & ccHObject::DP_PARENT_OF_OTHER,destRow);
		item->addDependency(newParent,itemDependencyFlags & ccHObject::DP_PARENT_OF_OTHER);
		//restore other flags on old parent (as all flags have been removed when calling removeElement!)
		if (oldParent)
		{
			oldParent->addDependency(item,fatherDependencyFlags & (~ccHObject::DP_PARENT_OF_OTHER));
			item->addDependency(oldParent,itemDependencyFlags & (~ccHObject::DP_PARENT_OF_OTHER));
		}

		if (newParent->getDisplay() == 0)
			newParent->setDisplay(item->getDisplay());
		else if (item->getDisplay() == 0)
			item->setDisplay(newParent->getDisplay());

		//add item back
		addElement(item,false);
		item->prepareDisplayForRefresh();
	}

	MainWindow::RefreshAllGLWindow(false);

	return true;
}

void ccDBRoot::expandBranch()
{
	expandOrCollapseHoveredBranch(true);
}

void ccDBRoot::collapseBranch()
{
	expandOrCollapseHoveredBranch(false);
}

void ccDBRoot::expandOrCollapseHoveredBranch(bool expand)
{
	//not initialized?
	if (m_contextMenuPos.x() < 0 || m_contextMenuPos.y() < 0)
		return;

	QModelIndex clickIndex = m_dbTreeWidget->indexAt(m_contextMenuPos);
	if (!clickIndex.isValid())
		return;
	ccHObject* item = static_cast<ccHObject*>(clickIndex.internalPointer());
	assert(item);

	if (!item || item->getChildrenNumber() == 0)
		return;

	//we recursively expand sub-branches
	ccHObject::Container toExpand;
	try
	{
		toExpand.push_back(item);
		while (!toExpand.empty())
		{
			item = toExpand.back();
			toExpand.pop_back();

			QModelIndex itemIndex = index(item);
			if (itemIndex.isValid())
			{
				if (expand)
					m_dbTreeWidget->expand(itemIndex);
				else
					m_dbTreeWidget->collapse(itemIndex);
			}

			assert(item->getChildrenNumber() != 0);
			for (unsigned i=0; i<item->getChildrenNumber(); ++i)
			{
				if (item->getChild(i)->getChildrenNumber() != 0)
					toExpand.push_back(item->getChild(i));
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory!
	}
}

void ccDBRoot::alignCameraWithEntity(bool reverse)
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	int selCount = selectedIndexes.size();
	if (selCount == 0)
		return;

	ccHObject* obj = static_cast<ccHObject*>(selectedIndexes[0].internalPointer());
	if (!obj)
		return;
	ccGenericGLDisplay* display = obj->getDisplay();
	if (!display)
	{
		ccLog::Warning("[alignCameraWithEntity] Selected entity has no associated display");
		return;
	}
	assert(display);
	ccGLWindow* win = static_cast<ccGLWindow*>(display);

	//plane normal
	CCVector3d planeNormal;
	CCVector3d planeVertDir;
	CCVector3 center;

	if (obj->isA(CC_TYPES::LABEL_2D)) //2D label with 3 points?
	{
		cc2DLabel* label = static_cast<cc2DLabel*>(obj);
		//work only with labels with 3 points!
		if (label->size() == 3)
		{
			const cc2DLabel::PickedPoint& A = label->getPoint(0);
			const CCVector3* _A = A.cloud->getPoint(A.index);
			const cc2DLabel::PickedPoint& B = label->getPoint(1);
			const CCVector3* _B = B.cloud->getPoint(B.index);
			const cc2DLabel::PickedPoint& C = label->getPoint(2);
			const CCVector3* _C = C.cloud->getPoint(C.index);
			CCVector3 N = (*_B-*_A).cross(*_C-*_A);
			planeNormal = CCVector3d::fromArray(N.u);
			planeVertDir = /*(*_B-*_A)*/win->getCurrentUpDir();
			center = (*_A + *_B + *_C) / 3;
		}
		else
		{
			assert(false);
			return;
		}
	}
	else if (obj->isA(CC_TYPES::PLANE)) //plane
	{
		ccPlane* plane = static_cast<ccPlane*>(obj);
		//3rd column = plane normal!
		planeNormal = CCVector3d::fromArray(plane->getNormal().u);
		planeVertDir = CCVector3d::fromArray(plane->getTransformation().getColumnAsVec3D(1).u);
		center = plane->getOwnBB().getCenter();
	}
	else if (obj->isA(CC_TYPES::FACET)) //facet
	{
		ccFacet* facet = static_cast<ccFacet*>(obj);
		planeNormal = CCVector3d::fromArray(facet->getNormal().u);
		CCVector3d planeHorizDir(0, 1, 0);
		CCLib::CCMiscTools::ComputeBaseVectors(planeNormal,planeHorizDir,planeVertDir);
		center = facet->getBB_recursive(false,false).getCenter();
	}
	else
	{
		assert(false);
		return;
	}

	//we can now make the camera look in the direction of the normal
	if (!reverse)
		planeNormal *= -1;
	win->setCustomView(planeNormal,planeVertDir);

	//output the transformation matrix that would make this normal points towards +Z
	{
		ccGLMatrixd transMat;
		transMat.setTranslation(-center);
		ccGLMatrixd viewMat = win->getViewportParameters().viewMat;
		viewMat = viewMat * transMat;
		viewMat.setTranslation(viewMat.getTranslationAsVec3D() + CCVector3d::fromArray(center.u));

		ccLog::Print("[Align camera] Corresponding view matrix:");
		ccLog::Print(viewMat.toString(12,' ')); //full precision
		ccLog::Print("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog");
	}
}

void ccDBRoot::gatherRecursiveInformation()
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	int selCount = selectedIndexes.size();
	if (selCount == 0)
		return;

	struct GlobalInfo
	{
		//properties
		unsigned pointCount;
		unsigned triangleCount;
		unsigned colorCount;
		unsigned normalCount;
		unsigned materialCount;
		unsigned scalarFieldCount;

		//entities
		unsigned cloudCount;
		unsigned meshCount;
		unsigned octreeCount;
		unsigned imageCount;
		unsigned sensorCount;
		unsigned labelCount;
	}
	info;

	memset(&info, 0, sizeof(GlobalInfo));

	//init the list of entities to process
	ccHObject::Container toProcess;
	try
	{
		toProcess.resize(selCount);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not engough memory");
		return;
	}

	for (int i = 0; i < selCount; ++i)
	{
			toProcess[i] = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
	}

	ccHObject::Container alreadyProcessed;
	while (!toProcess.empty())
	{
		ccHObject* ent = toProcess.back();
		toProcess.pop_back();

		//we don't process entities twice!
		if (std::find(alreadyProcessed.begin(), alreadyProcessed.end(), ent) != alreadyProcessed.end())
		{
			continue;
		}

		//gather information from current entity
		if (ent->isA(CC_TYPES::POINT_CLOUD))
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(ent);
			info.cloudCount++;

			unsigned cloudSize = cloud->size();
			info.pointCount += cloudSize;
			info.colorCount += (cloud->hasColors() ? cloudSize : 0);
			info.normalCount += (cloud->hasNormals() ? cloudSize : 0);
			info.scalarFieldCount += cloud->getNumberOfScalarFields();
		}
		else if (ent->isKindOf(CC_TYPES::MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(ent);

			info.meshCount++;
			unsigned meshSize = mesh->size();
			info.triangleCount += meshSize;
			info.normalCount += (mesh->hasTriNormals() ? meshSize : 0);
			info.materialCount += (mesh->getMaterialSet() ? (unsigned)mesh->getMaterialSet()->size() : 0);
		}
		else if (ent->isKindOf(CC_TYPES::LABEL_2D))
		{
			info.labelCount++;
		}
		else if (ent->isKindOf(CC_TYPES::SENSOR))
		{
			info.sensorCount++;
		}
		else if (ent->isKindOf(CC_TYPES::POINT_OCTREE))
		{
			info.octreeCount++;
		}
		else if (ent->isKindOf(CC_TYPES::IMAGE))
		{
			info.imageCount++;
		}

		//we can add its children to the 'toProcess' list and itself to the 'processed' list
		try
		{
			for (unsigned i = 0; i < ent->getChildrenNumber(); ++i)
			{
				toProcess.push_back(ent->getChild(i));
			}
			alreadyProcessed.push_back(ent);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not engough memory");
			return;
		}
	}

	//output information
	{
		QStringList infoStr;

		const QString separator("--------------------------");

		infoStr << QString("Point(s):\t\t%L1").arg(info.pointCount);
		infoStr << QString("Triangle(s):\t\t%L1").arg(info.triangleCount);

		infoStr << separator;
		if (info.colorCount)
			infoStr << QString("Color(s):\t\t%L1").arg(info.colorCount);
		if (info.normalCount)
			infoStr << QString("Normal(s):\t\t%L1").arg(info.normalCount);
		if (info.scalarFieldCount)
			infoStr << QString("Scalar field(s):\t\t%L1").arg(info.scalarFieldCount);
		if (info.materialCount)
			infoStr << QString("Material(s):\t\t%L1").arg(info.materialCount);

		infoStr << separator;
		infoStr << QString("Cloud(s):\t\t%L1").arg(info.cloudCount);
		infoStr << QString("Mesh(es):\t\t%L1").arg(info.meshCount);
		if (info.octreeCount)
			infoStr << QString("Octree(s):\t\t%L1").arg(info.octreeCount);
		if (info.imageCount)
			infoStr << QString("Image(s):\t\t%L1").arg(info.imageCount);
		if (info.labelCount)
			infoStr << QString("Label(s):\t\t%L1").arg(info.labelCount);
		if (info.sensorCount)
			infoStr << QString("Sensor(s):\t\t%L1").arg(info.sensorCount);

		//display info box
		QMessageBox::information(MainWindow::TheInstance(),
								 "Information",
								 infoStr.join("\n"));
	}
}

void ccDBRoot::sortChildrenAZ()
{
	sortSelectedEntitiesChildren(SORT_A2Z);
}

void ccDBRoot::sortChildrenZA()
{
	sortSelectedEntitiesChildren(SORT_Z2A);
}

void ccDBRoot::sortChildrenType()
{
	sortSelectedEntitiesChildren(SORT_BY_TYPE);
}

void ccDBRoot::sortSelectedEntitiesChildren(SortRules sortRule)
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	int selCount = selectedIndexes.size();
	if (selCount == 0)
		return;

	for (int i = 0; i < selCount; ++i)
	{
		ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		unsigned childCount = (item ? item->getChildrenNumber() : 0);
		if (childCount > 1)
		{
			//remove all children from DB tree
			beginRemoveRows(selectedIndexes[i], 0, childCount - 1);

			//row removal operation (end)
			endRemoveRows();

			//sort
			for (unsigned k = 0; k < childCount - 1; ++k)
			{
				unsigned firstChildIndex = k;
				ccHObject* firstChild = item->getChild(k);
				QString firstChildName = firstChild->getName().toUpper();

				for (unsigned j = k + 1; j < childCount; ++j)
				{
					bool swap = false;
					QString currentName = item->getChild(j)->getName().toUpper();
					switch (sortRule)
					{
					case SORT_A2Z:
						swap = (firstChildName.compare(currentName) > 0);
						break;
					case SORT_Z2A:
						swap = (firstChildName.compare(currentName) < 0);
						break;
					case SORT_BY_TYPE:
						if (firstChild->getClassID() == item->getChild(j)->getClassID())
							swap = (firstChildName.compare(currentName) > 0); //A2Z in second choice
						else
							swap = (firstChild->getClassID() > item->getChild(j)->getClassID());
						break;
					}

					if (swap)
					{
						firstChildIndex = j;
						firstChildName = currentName;
					}
				}

				if (k != firstChildIndex)
					item->swapChildren(k, firstChildIndex);
			}

			//add children back
			beginInsertRows(selectedIndexes[i], 0, childCount - 1);

			//row insertion operation (end)
			endInsertRows();
		}
	}
}

void ccDBRoot::selectByTypeAndName()
{
	ccSelectChildrenDlg scDlg(MainWindow::TheInstance());
	scDlg.addType("Point cloud",       CC_TYPES::POINT_CLOUD);
	scDlg.addType("Poly-line",         CC_TYPES::POLY_LINE);
	scDlg.addType("Mesh",              CC_TYPES::MESH);
	scDlg.addType("  Sub-mesh",        CC_TYPES::SUB_MESH);
	scDlg.addType("  Primitive",       CC_TYPES::PRIMITIVE);
	scDlg.addType("    Plane",         CC_TYPES::PLANE);
	scDlg.addType("    Sphere",        CC_TYPES::SPHERE);
	scDlg.addType("    Torus",         CC_TYPES::TORUS);
	scDlg.addType("    Cylinder",      CC_TYPES::CYLINDER);
	scDlg.addType("    Cone",          CC_TYPES::CONE);
	scDlg.addType("    Box",           CC_TYPES::BOX);
	scDlg.addType("    Dish",          CC_TYPES::DISH);
	scDlg.addType("    Extrusion",     CC_TYPES::EXTRU);
	scDlg.addType("Sensor",            CC_TYPES::SENSOR);
	scDlg.addType("  GBL/TLS sensor",  CC_TYPES::GBL_SENSOR);
	scDlg.addType("  Camera sensor",   CC_TYPES::CAMERA_SENSOR);
	scDlg.addType("Image",             CC_TYPES::IMAGE);
	scDlg.addType("Facet",             CC_TYPES::FACET);
	scDlg.addType("Label",             CC_TYPES::LABEL_2D);
	scDlg.addType("Area label",        CC_TYPES::VIEWPORT_2D_LABEL);
	scDlg.addType("Octree",            CC_TYPES::POINT_OCTREE);
	scDlg.addType("Kd-tree",           CC_TYPES::POINT_KDTREE);
	scDlg.addType("Viewport",          CC_TYPES::VIEWPORT_2D_OBJECT);
	scDlg.addType("Custom Types",      CC_TYPES::CUSTOM_H_OBJECT);

	if (!scDlg.exec())
		return;

	// for type checking
	CC_CLASS_ENUM type = CC_TYPES::OBJECT; // all objects are matched by def
	bool exclusive = false;

	if (scDlg.getTypeIsUsed()) // we are using type checking
	{
		type = scDlg.getSelectedType();

		//some types are exclusive, some are generic, and some can be both
		//(e.g. Meshes)
		//
		//For generic-only types the match type gets overridden and forced to
		//false because exclusive match makes no sense!
		switch (type)
		{
		case CC_TYPES::HIERARCHY_OBJECT: //returned if no type is selected (i.e. all objects are selected!)
		case CC_TYPES::PRIMITIVE:
		case CC_TYPES::SENSOR:
		case CC_TYPES::IMAGE:
			exclusive = false;
			break;
		default:
			exclusive = scDlg.getStrictMatchState();
			break;
		}
	}


	// for name matching - def values
	bool regex = false;
	QString name; // an empty string by default

	if (scDlg.getNameMatchIsUsed())
	{
		regex = scDlg.getNameIsRegex();
		name = scDlg.getSelectedName();
	}


	selectChildrenByTypeAndName(type, exclusive, name, regex);
}

/* name is optional, if passed it is used to restrict the selection by type */
void ccDBRoot::selectChildrenByTypeAndName(CC_CLASS_ENUM type,
										   bool typeIsExclusive/*=true*/,
										   QString name/*=QString()*/,
										   bool nameIsRegex/*= false*/)
{
	//not initialized?
	if (m_contextMenuPos.x() < 0 || m_contextMenuPos.y() < 0)
		return;

	QModelIndex clickIndex = m_dbTreeWidget->indexAt(m_contextMenuPos);
	if (!clickIndex.isValid())
		return;
	ccHObject* item = static_cast<ccHObject*>(clickIndex.internalPointer());
	assert(item);

	if (!item || item->getChildrenNumber() == 0)
		return;

	ccHObject::Container filteredByType;
	item->filterChildren(filteredByType, true, type, typeIsExclusive);

	// The case of an empty filteredByType is handled implicitly, to make
	// the ctrlPushed behavior below more consistent (i.e. when no object
	// is found and Control was NOT pressed the selection will still be
	// cleared).
	ccHObject::Container toSelect;
	try
	{
		if (name.isEmpty())
		{
			toSelect = filteredByType;
		}
		else
		{
			for (size_t i=0; i<filteredByType.size(); ++i)
			{
				ccHObject* child = filteredByType[i];

				if (nameIsRegex) // regex matching
				{

					QRegularExpression re(name);
					QRegularExpressionMatch match = re.match(child->getName());
					bool hasMatch = match.hasMatch(); // true
					if (hasMatch)
						toSelect.push_back(child);

				}

				else if (child->getName().compare(name) == 0) // simple comparison
					toSelect.push_back(child);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[selectChildrenByTypeAndName] Not enough memory");
		return;
	}

	bool ctrlPushed = (QApplication::keyboardModifiers () & Qt::ControlModifier);
	selectEntities(toSelect, ctrlPushed);
}

void ccDBRoot::toggleSelectedEntitiesProperty(TOGGLE_PROPERTY prop)
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	int selCount = selectedIndexes.size();
	if (selCount == 0)
		return;

	//hide properties view
	hidePropertiesView();

	for (int i=0; i<selCount; ++i)
	{
		ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		if (!item)
		{
			assert(false);
			continue;
		}
		switch (prop)
		{
		case TG_ENABLE: //enable state
			item->setEnabled(!item->isEnabled());
			break;
		case TG_VISIBLE: //visibility
			item->toggleVisibility();
			break;
		case TG_COLOR: //color
			item->toggleColors();
			break;
		case TG_NORMAL: //normal
			item->toggleNormals();
			break;
		case TG_SF: //SF
			item->toggleSF();
			break;
		case TG_MATERIAL: //Materials/textures
			item->toggleMaterials();
			break;
		case TG_3D_NAME: //3D name
			item->toggleShowName();
			break;
		}
		item->prepareDisplayForRefresh();
	}

	//we restablish properties view
	updatePropertiesView();

	MainWindow::RefreshAllGLWindow(false);
}

void ccDBRoot::addEmptyGroup()
{
	//not initialized?
	if (m_contextMenuPos.x() < 0 || m_contextMenuPos.y() < 0)
		return;

	QModelIndex index = m_dbTreeWidget->indexAt(m_contextMenuPos);
	ccHObject* newGroup = new ccHObject("Group");
	if (index.isValid())
	{
		ccHObject* parent = static_cast<ccHObject*>(index.internalPointer());
		if (parent)
			parent->addChild(newGroup);
	}

	addElement(newGroup);
}

void ccDBRoot::enableBubbleViewMode()
{
	QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
	int selCount = selectedIndexes.size();
	if (selCount == 0)
		return;

	for (int i = 0; i < selCount; ++i)
	{
		ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		if (item &&item->isA(CC_TYPES::GBL_SENSOR))
		{
			static_cast<ccGBLSensor*>(item)->applyViewport();
		}
	}

	MainWindow::RefreshAllGLWindow(false);
}

void ccDBRoot::showContextMenu(const QPoint& menuPos)
{
	m_contextMenuPos = menuPos;

	//build custom context menu
	QMenu menu;

	QModelIndex index = m_dbTreeWidget->indexAt(menuPos);
	if (index.isValid())
	{
		QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();

		//selected items?
		QModelIndexList selectedIndexes = qism->selectedIndexes();
		int selCount = selectedIndexes.size();
		if (selCount)
		{
			bool toggleVisibility = false;
			bool toggleOtherProperties = false;
			bool toggleMaterials = false;
			bool hasMoreThanOneChild = false;
			bool hasExactlyOnePlanarEntity = false;
			bool leafObject = false;
			bool hasExacltyOneGBLSenor = false;
			bool hasExactlyOnePlane = false;
			for (int i = 0; i < selCount; ++i)
			{
				ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
				if (!item)
				{
					assert(false);
					continue;
				}
				if (item->getChildrenNumber() > 1)
				{
					hasMoreThanOneChild = true;
				}
				leafObject |= item->isLeaf();
				if (!item->isA(CC_TYPES::HIERARCHY_OBJECT))
				{
					toggleVisibility = true;
					if (item->isKindOf(CC_TYPES::POINT_CLOUD))
					{
						toggleOtherProperties = true;
					}
					else if (item->isKindOf(CC_TYPES::MESH))
					{
						toggleMaterials = true;
						toggleOtherProperties = true;
					}

					if (selCount == 1)
					{
						if (item->isA(CC_TYPES::LABEL_2D))
						{
							hasExactlyOnePlanarEntity = (static_cast<cc2DLabel*>(item)->size() == 3);
						}
						else if (item->isA(CC_TYPES::PLANE) || item->isA(CC_TYPES::FACET))
						{
							hasExactlyOnePlanarEntity = true;
							hasExactlyOnePlane = item->isKindOf(CC_TYPES::PLANE);
						}
						else if (item->isA(CC_TYPES::GBL_SENSOR))
						{
							hasExacltyOneGBLSenor = true;
						}
					}
				}
			}

			if (hasExactlyOnePlanarEntity)
			{
				menu.addAction(m_alignCameraWithEntity);
				menu.addAction(m_alignCameraWithEntityReverse);
				menu.addSeparator();
			}
			if (hasExactlyOnePlane)
			{
				MainWindow::TheInstance()->addEditPlaneAction( menu );
			}
			if (hasExacltyOneGBLSenor)
			{
				menu.addAction(m_enableBubbleViewMode);
			}
			menu.addAction(m_gatherInformation);
			menu.addSeparator();
			menu.addAction(m_toggleSelectedEntities);
			if (toggleVisibility)
				menu.addAction(m_toggleSelectedEntitiesVisibility);
			if (toggleOtherProperties)
			{
				menu.addAction(m_toggleSelectedEntitiesColor);
				menu.addAction(m_toggleSelectedEntitiesNormals);
				menu.addAction(m_toggleSelectedEntitiesSF);
			}
			if (toggleMaterials)
			{
				menu.addAction(m_toggleSelectedEntitiesMat);
			}
			menu.addAction(m_toggleSelectedEntities3DName);
			menu.addSeparator();
			menu.addAction(m_deleteSelectedEntities);
			if (selCount == 1 && hasMoreThanOneChild)
			{
				menu.addSeparator();
				menu.addAction(m_sortChildrenAZ);
				menu.addAction(m_sortChildrenZA);
				menu.addAction(m_sortChildrenType);
			}

			if (selCount == 1 && !leafObject)
			{
				menu.addSeparator();
				menu.addAction(m_selectByTypeAndName);
				menu.addSeparator();
				menu.addAction(m_addEmptyGroup);
			}
			menu.addSeparator();
		}

		menu.addAction(m_expandBranch);
		menu.addAction(m_collapseBranch);
	}
	else
	{
		menu.addSeparator();
		menu.addAction(m_addEmptyGroup);
	}

	menu.exec(m_dbTreeWidget->mapToGlobal(menuPos));
}

QItemSelectionModel::SelectionFlags ccCustomQTreeView::selectionCommand(const QModelIndex& index, const QEvent* event/*=0*/) const
{
	if (index.isValid())
	{
		//special case: labels can only be merged with labels!
		QModelIndexList selectedIndexes = selectionModel()->selectedIndexes();
		if (!selectedIndexes.empty() && !selectionModel()->isSelected(index))
		{
			ccHObject* selectedItem = static_cast<ccHObject*>(index.internalPointer());
			if (selectedItem && selectedItem->isA(CC_TYPES::LABEL_2D) != static_cast<ccHObject*>(selectedIndexes[0].internalPointer())->isA(CC_TYPES::LABEL_2D))
				return QItemSelectionModel::ClearAndSelect;
		}
	}

	return QTreeView::selectionCommand(index,event);
}
