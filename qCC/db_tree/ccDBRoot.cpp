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

#include "ccDBRoot.h"

//Local
#include "ccGLWindow.h"

//Qt
#include <QTreeView>
#include <QStandardItemModel>
#include <QHeaderView>
#include <QMimeData>
#include <QMessageBox>

//qCC_db
#include <ccHObject.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccMaterialSet.h>
#include <cc2DLabel.h>
#include <ccGenericPrimitive.h>
#include <ccPlane.h>

//local
#include "ccPropertiesTreeDelegate.h"
#include "../ccConsole.h"
#include "../mainwindow.h"

//system
#include <assert.h>
#include <algorithm>

//Minimum width of the left column of the properties tree view
static const int c_propViewLeftColumnWidth = 115;

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
	//m_dbTreeWidget->viewport()->setAcceptDrops(true);
	m_dbTreeWidget->setDropIndicatorShown(true);
	m_dbTreeWidget->setDragDropMode(QAbstractItemView::InternalMove);
	setSupportedDragActions(Qt::MoveAction);
    /*//already done in ui file!
    m_dbTreeWidget->setEditTriggers(QAbstractItemView::EditKeyPressed);
    m_dbTreeWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_dbTreeWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_dbTreeWidget->setUniformRowHeights(true);
    //*/

	//context menu on DB tree elements
	m_dbTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
	m_expandBranch = new QAction("Expand branch",this);
	m_collapseBranch = new QAction("Collapse branch",this);
	m_gatherInformation = new QAction("Information (recursive)",this);
	m_sortSiblingsType = new QAction("Sort siblings by type",this);
	m_sortSiblingsAZ = new QAction("Sort siblings by name (A-Z)",this);
	m_sortSiblingsZA = new QAction("Sort siblings by name (Z-A)",this);
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

	m_contextMenuPos = QPoint(-1,-1);

	//connect custom context menu actions
	connect(m_dbTreeWidget,						SIGNAL(customContextMenuRequested(const QPoint&)),	this, SLOT(showContextMenu(const QPoint&)));
	connect(m_expandBranch,						SIGNAL(triggered()),								this, SLOT(expandBranch()));
	connect(m_collapseBranch,					SIGNAL(triggered()),								this, SLOT(collapseBranch()));
	connect(m_gatherInformation,				SIGNAL(triggered()),								this, SLOT(gatherRecursiveInformation()));
	connect(m_sortSiblingsAZ,					SIGNAL(triggered()),								this, SLOT(sortSiblingsAZ()));
	connect(m_sortSiblingsZA,					SIGNAL(triggered()),								this, SLOT(sortSiblingsZA()));
	connect(m_sortSiblingsType,					SIGNAL(triggered()),								this, SLOT(sortSiblingsType()));
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

    //other DB tree signals/slots connection
    connect(m_dbTreeWidget->selectionModel(), SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)), this, SLOT(changeSelection(const QItemSelection&, const QItemSelection&)));

    //Properties Tree
    assert(propertiesTreeWidget);
    m_propertiesTreeWidget = propertiesTreeWidget;
    m_propertiesModel = new QStandardItemModel(0, 2, parent);
    /*//already done in ui file!
    m_propertiesTreeWidget->header()->hide();
    m_propertiesTreeWidget->setSelectionMode(QAbstractItemView::NoSelection);
    m_propertiesTreeWidget->setAllColumnsShowFocus(true);
    //*/
    m_ccPropDelegate = new ccPropertiesTreeDelegate(m_propertiesModel, m_propertiesTreeWidget);
    m_propertiesTreeWidget->setItemDelegate(m_ccPropDelegate);
    m_propertiesTreeWidget->setModel(m_propertiesModel);
	m_propertiesTreeWidget->header()->setResizeMode(QHeaderView::Interactive);
	m_propertiesTreeWidget->header()->setStretchLastSection(true);
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

ccHObject* ccDBRoot::getRootEntity()
{
    return m_treeRoot;
}

void ccDBRoot::addElement(ccHObject* anObject, bool autoExpand/*=true*/)
{
    if (!anObject)
        return;

    //look for object's parent
    ccHObject* parentObject = anObject->getParent();
    if (!parentObject)
    {
        //if the object has no parent, it will be inserted at tree root
        parentObject = m_treeRoot;
        m_treeRoot->addChild(anObject);
    }

    //look for insert node index in tree
    QModelIndex insertNodeIndex = index(parentObject);
    int childPos = parentObject->getChildIndex(anObject);

    //row insertion operation (start)
    beginInsertRows(insertNodeIndex, childPos, childPos);

    //row insertion operation (end)
    endInsertRows();

	if (autoExpand)
	{
		QModelIndex childIndex = index(anObject);
		if (childIndex.isValid())
			m_dbTreeWidget->expand(childIndex);
	}
	else //if (parentObject)
	{
		m_dbTreeWidget->expand(insertNodeIndex);
	}
}

void ccDBRoot::expandElement(ccHObject* anObject, bool state)
{
    if (!anObject || !m_dbTreeWidget)
        return;

	m_dbTreeWidget->setExpanded(index(anObject),state);
}

void ccDBRoot::removeElement(ccHObject* anObject)
{
    if (!anObject)
        return;

    //we hide properties view in case this is the deleted object that is currently selected
    hidePropertiesView();

    //every object in tree must have a parent!
    ccHObject* parent = anObject->getParent();
    if (!parent)
    {
		ccConsole::Warning("[ccDBRoot::removeElement] Internal error: object has no parent!");
        return;
    }

    int childPos = parent->getChildIndex(anObject);
    assert(childPos>=0);

    //row removal operation (start)
    beginRemoveRows(index(parent),childPos,childPos);

    parent->removeChild(childPos);

    //row removal operation (end)
    endRemoveRows();

    //we restablish properties view
    updatePropertiesView();
}

void ccDBRoot::deleteSelectedEntities()
{
    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    if (selectedIndexes.size() < 1)
        return;
    unsigned selCount = (unsigned)selectedIndexes.size();

    hidePropertiesView();

	//specific case: shared labels
	ccHObject::Container allLabels;
	bool hasSharedLabels = false;
	std::set<ccHObject*> cloudsToBeDeleted; //will only be used if 'hasSharedLabels' is true
	if (m_treeRoot->filterChildren(allLabels,true,CC_2D_LABEL) != 0)
	{
		for (unsigned i=0; i<allLabels.size(); ++i)
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(allLabels[i]);
			//shared labels are labels shared by at least 2 different clouds
			if (label->size() > 1 && label->getPoint(0).cloud != label->getPoint(1).cloud
				|| label->size() > 2 && label->getPoint(1).cloud != label->getPoint(2).cloud)
			{
				hasSharedLabels = true;
				break;
			}
		}
	}

	//we remove all objects that are children of other deleted ones!
	//(otherwise we may delete the parent before the child!)
    std::vector<ccHObject*> toBeDeleted;
    for (unsigned i=0;i<selCount;++i)
    {
        ccHObject* obj = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
        //we don't take care of parentless objects (i.e. the tree root)
		if (!obj->getParent() || obj->isLocked())
		{
			ccConsole::Warning(QString("Object '%1' can't be deleted this way (locked)").arg(obj->getName()));
			continue;
		}

		//we don't take objects that are siblings of others
		bool isSiblingOfAnotherOne = false;
		for (unsigned j=0;j<selCount;++j)
		{
			if (i != j)
			{
				ccHObject* otherObj = static_cast<ccHObject*>(selectedIndexes[j].internalPointer());
				if (otherObj->isAncestorOf(obj))
				{
					isSiblingOfAnotherOne = true;
					break;
				}
			}
		}

		if (!isSiblingOfAnotherOne)
		{
			//last check: mesh vertices
			if (obj->isKindOf(CC_POINT_CLOUD) && obj->getParent()->isKindOf(CC_MESH))
				if (ccHObjectCaster::ToGenericMesh(obj->getParent())->getAssociatedCloud() == obj)
				{
					ccConsole::Warning("Mesh vertices can't be deleted without their parent mesh!");
					continue;
				}

			toBeDeleted.push_back(obj);

			if (hasSharedLabels)
			{
				//we must keep a parallel list for clouds only
				if (obj->isA(CC_POINT_CLOUD))
				{
					cloudsToBeDeleted.insert(obj);
				}
				else
				{
					ccHObject::Container subClouds;
					if (obj->filterChildren(subClouds,true,CC_POINT_CLOUD) != 0)
						for (size_t i=0; i<subClouds.size(); ++i)
							cloudsToBeDeleted.insert(subClouds[i]);
				}
			}
		}
	}

    qism->clear();

	//check now that we don't delete clouds on which some labels are dependent
	if (hasSharedLabels)
	{
		size_t labelCount = allLabels.size();
		for (size_t i=0;i<labelCount;++i)
		{
			cc2DLabel* label = static_cast<cc2DLabel*>(allLabels[i]);
			if (label->size() > 1) //there's no issue with 1-point labels!
			{
				for (unsigned j=1;j<label->size();++j) //1st cloud is always the parent!
					if (label->getPoint(j).cloud
						&& label->getPoint(j).cloud != label->getPoint(0).cloud
						&& cloudsToBeDeleted.find(label->getPoint(j).cloud) != cloudsToBeDeleted.end())
					{
						ccLog::Warning(QString("Label '%1' has been deleted as it is dependent on '%2'").arg(label->getName()).arg(label->getPoint(j).cloud->getName()));
						label->clear();
						toBeDeleted.push_back(label);
					}
			}
		}
	}

	while (!toBeDeleted.empty())
	{
		ccHObject* anObject = toBeDeleted.back();
		assert(anObject);
		toBeDeleted.pop_back();

		anObject->prepareDisplayForRefresh_recursive();

		if (anObject->isKindOf(CC_MESH))
		{
			//specific case: the object is a mesh and its parent is its vertices!
			//(can happen if a Delaunay mesh is computed directly in CC)
			if (anObject->getParent() && anObject->getParent() == ccHObjectCaster::ToGenericMesh(anObject)->getAssociatedCloud())
				anObject->getParent()->setVisible(true);
		}

        ccHObject* parent = anObject->getParent();
        int childPos = parent->getChildIndex(anObject);
        assert(childPos>=0);

        beginRemoveRows(index(anObject).parent(),childPos,childPos);
        parent->removeChild(childPos);
        endRemoveRows();
    }

    updatePropertiesView();

    MainWindow::RefreshAllGLWindow();
}

QVariant ccDBRoot::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    const ccHObject *item = static_cast<ccHObject*>(index.internalPointer());
	assert(item);
	if (!item)
        return QVariant();

    if (role == Qt::DisplayRole)
    {
		QString baseName(item->getName());
		if (baseName.isEmpty())
			baseName = QString("no name");
		//specific case
		if (item->isA(CC_2D_LABEL))
			baseName = QString("2D label: ")+baseName;
		else if (item->isA(CC_2D_VIEWPORT_LABEL))
			baseName = QString("2D area label: ")+baseName;

       return QVariant(baseName);
    }
    if (role == Qt::EditRole)
    {
       return QVariant(item->getName());
    }
    else if (role == Qt::DecorationRole)
    {
        bool locked = item->isLocked();
        switch (item->getClassID())
        {
            case CC_HIERARCHY_OBJECT:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbHObjectSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbHObjectSymbol.png"));
            case CC_POINT_CLOUD:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbCloudSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbCloudSymbol.png"));
			//all primitives
			case CC_PLANE:
			case CC_SPHERE:
			case CC_TORUS:	
			case CC_CYLINDER:
			case CC_CONE:	
			case CC_BOX:	
			case CC_DISH:	
			case CC_EXTRU:	
			case CC_FACET:	
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbMiscGeomSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbMiscGeomSymbol.png"));
            case CC_MESH:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbMeshSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbMeshSymbol.png"));
            case CC_MESH_GROUP:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbMeshGroupSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbMeshGroupSymbol.png"));
            case CC_POINT_OCTREE:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbOctreeSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbOctreeSymbol.png"));
            case CC_CALIBRATED_IMAGE:
                return QIcon(QString::fromUtf8(":/CC/images/dbCalibratedImageSymbol.png"));
            case CC_IMAGE:
                return QIcon(QString::fromUtf8(":/CC/images/dbImageSymbol.png"));
            case CC_SENSOR:
            case CC_GBL_SENSOR:
                return QIcon(QString::fromUtf8(":/CC/images/dbSensorSymbol.png"));
			case CC_MATERIAL_SET:
                return QIcon(QString::fromUtf8(":/CC/images/dbMaterialSymbol.png"));
			case CC_NORMALS_ARRAY:
			case CC_NORMAL_INDEXES_ARRAY:
			case CC_RGB_COLOR_ARRAY:
			case CC_TEX_COORDS_ARRAY:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/images/dbContainerSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/images/dbContainerSymbol.png"));
			case CC_2D_LABEL:
				return QIcon(QString::fromUtf8(":/CC/images/dbLabelSymbol.png"));
			case CC_2D_VIEWPORT_OBJECT:
				return QIcon(QString::fromUtf8(":/CC/images/dbViewportSymbol.png"));
			case CC_2D_VIEWPORT_LABEL:
				return QIcon(QString::fromUtf8(":/CC/images/dbAreaLabelSymbol.png"));
            default:
                if (locked)
                   return QIcon(QString::fromUtf8(":/CC/images/dbLockSymbol.png"));
                else
                    return QVariant();
         }
     }
     else if (role == Qt::CheckStateRole)
     {
         if (item->isEnabled())
            return Qt::Checked;
        else
            return Qt::Unchecked;
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
                return false;

            ccHObject *item = static_cast<ccHObject*>(index.internalPointer());
			assert(item);
			if (item)
			{
				item->setName(value.toString());

				//particular cases:
				// - labels name is their title (so we update them)
				// - name might be displayed in 3D
				if (item->nameShownIn3D() || item->isKindOf(CC_2D_LABEL))
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
        return QModelIndex();

    ccHObject *parent = (parentIndex.isValid() ? static_cast<ccHObject*>(parentIndex.internalPointer()) : m_treeRoot);
	assert(parent);
	if (!parent)
        return QModelIndex();
    ccHObject *child = parent->getChild(row);
    if (child)
        return createIndex(row, column, child);
    else
        return QModelIndex();
}

QModelIndex ccDBRoot::index(ccHObject* object)
{
    assert(object);

    if (object == m_treeRoot)
        return QModelIndex();

    ccHObject* parent = object->getParent();
    if (!parent)
    {
        ccConsole::Error(QString("An error while creating DB tree index: object '%1' has no parent!").arg(object->getName()));
        return QModelIndex();
    }

    int pos = parent->getChildIndex(object);
    assert(pos>=0);

    return createIndex(pos,0,object);
}

QModelIndex ccDBRoot::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    ccHObject *childItem = static_cast<ccHObject*>(index.internalPointer());
	assert(childItem);
	if (!childItem)
        return QModelIndex();
    ccHObject *parentItem = childItem->getParent();

	assert(parentItem);
    if (!parentItem || parentItem == m_treeRoot)
        return QModelIndex();

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
    /*if (parent.isValid())
        return static_cast<ccHObject*>(parent.internalPointer())->columnCount();
    else
        return m_treeRoot->columnCount();
    //*/
}

void ccDBRoot::changeSelection(const QItemSelection & selected, const QItemSelection & deselected)
{
    //first unselect
	QModelIndexList deselectedItems = deselected.indexes();
	{
		for (int i=0;i<deselectedItems.count();++i)
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
		for (int i=0;i<selectedItems.count();++i)
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
			selectionModel->select(objIndex,QItemSelectionModel::Deselect);
		}
	}
}

void ccDBRoot::selectEntity(ccHObject* obj)
{
    bool ctrlPushed = (QApplication::keyboardModifiers () & Qt::ControlModifier);

    QItemSelectionModel* selectionModel = m_dbTreeWidget->selectionModel();
	assert(selectionModel);

	//valid object? then we will try to select (or toggle) it
	if (obj)
	{
		QModelIndex selectedIndex = index(obj);
		if (selectedIndex.isValid())
		{
			//if CTRL is pushed
			if (ctrlPushed)
			{
				//default case: toggle current item selection state
				if (!obj->isSelected())
				{
					QModelIndexList selectedIndexes = selectionModel->selectedIndexes();
					if (!selectedIndexes.empty())
					{
						//special case: labels can only be merged with labels!
						if (obj->isA(CC_2D_LABEL) != static_cast<ccHObject*>(selectedIndexes[0].internalPointer())->isA(CC_2D_LABEL))
						{
							ccLog::Warning("[Selection] Labels and other entities can't be mixed! (release the CTRL key to start a new selection)");
							return;
						}
					}
				}
				selectionModel->select(selectedIndex,QItemSelectionModel::Toggle);
			}
			else
			{
				if (selectionModel->isSelected(selectedIndex))  //nothing to do
					return;
				selectionModel->select(selectedIndex,QItemSelectionModel::ClearAndSelect);
			}

			//hack: auto-scroll to selected element
			if (obj->isSelected() && !ctrlPushed)
				m_dbTreeWidget->scrollTo(selectedIndex);
		}
	}
	//otherwise we clear current selection (if CTRL is not pushed)
	else if (!ctrlPushed)
	{
		selectionModel->clear();
	}
}

void ccDBRoot::selectEntity(int uniqueID)
{
	ccHObject* obj = 0;
    //minimum unqiue ID is 1 (0 means 'deselect')
    if (uniqueID>0)
        obj = find(uniqueID);

	selectEntity(obj);
}

void ccDBRoot::selectEntities(std::set<int> entIDs)
{
	//convert input list of IDs to proper entities
	ccHObject::Container entities;
	size_t labelCount = 0;
	{
		try
		{
			entities.reserve(entIDs.size());
		}
		catch(std::bad_alloc)
		{
			ccLog::Error("[ccDBRoot::selectEntities] Not enough memory!");
			return;
		}

		for (std::set<int>::const_iterator it = entIDs.begin(); it != entIDs.end(); ++it)
		{
			ccHObject* obj = find(*it);
			if (obj)
			{
				entities.push_back(obj);
				if (obj->isA(CC_2D_LABEL))
					++labelCount;
			}
		}
	}

	//selection model
	QItemSelectionModel* selectionModel = m_dbTreeWidget->selectionModel();
	assert(selectionModel);

	bool ctrlPushed = (QApplication::keyboardModifiers () & Qt::ControlModifier);

	//create new selection structure
	QItemSelection newSelection;
	{
		//shall we keep labels?
		bool keepLabels = false;
		{
			QModelIndexList formerSelectedIndexes = selectionModel->selectedIndexes();
			if (formerSelectedIndexes.isEmpty() || !ctrlPushed)
				keepLabels = (labelCount == entities.size()); //yes if they are the only selected entities
			else if (ctrlPushed)
				keepLabels = static_cast<ccHObject*>(formerSelectedIndexes[0].internalPointer())->isA(CC_2D_LABEL); //yes if previously selected entities were already labels
		}

		for (ccHObject::Container::const_iterator it = entities.begin(); it != entities.end(); ++it)
		{
			//filter input selection (can't keep both labels and standard entities --> we can't mix them!)
			bool isLabel = (*it)->isA(CC_2D_LABEL);
			if (isLabel == keepLabels && (!ctrlPushed || !(*it)->isSelected()))
			{
				QModelIndex selectedIndex = index(*it);
				if (selectedIndex.isValid())
					newSelection.merge(QItemSelection(selectedIndex,selectedIndex),QItemSelectionModel::Select);
			}
		}
	}

	//default behavior: clear previous selection if CTRL is not pushed
	selectionModel->select(newSelection,ctrlPushed ? QItemSelectionModel::Select : QItemSelectionModel::ClearAndSelect);
}

ccHObject* ccDBRoot::find(int uniqueID) const
{
	return m_treeRoot->find(uniqueID);
}

void ccDBRoot::showPropertiesView(ccHObject* obj)
{
    m_ccPropDelegate->fillModel(obj);

    m_propertiesTreeWidget->setEnabled(true);
	m_propertiesTreeWidget->setColumnWidth(0,c_propViewLeftColumnWidth);
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
    if (!m_propertiesTreeWidget->isEnabled() || m_ccPropDelegate->getCurrentObject()!=obj)
        showPropertiesView(obj);
}

void ccDBRoot::updatePropertiesView()
{
    assert(m_propertiesTreeWidget);
    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    if (selectedIndexes.size()==1)
        showPropertiesView(static_cast<ccHObject*>(selectedIndexes[0].internalPointer()));
    else
        hidePropertiesView();
}

void ccDBRoot::updateCCObject(ccHObject* anObject)
{
    assert(anObject);

    QModelIndex idx = index(anObject);

    if (idx.isValid())
        emit dataChanged(idx,idx);
}

void ccDBRoot::redrawCCObject(ccHObject* anObject)
{
    assert(anObject);

    anObject->redrawDisplay();
}

void ccDBRoot::redrawCCObjectAndChildren(ccHObject* anObject)
{
    assert(anObject);

    anObject->prepareDisplayForRefresh_recursive();
    anObject->refreshDisplay_recursive();
}

int ccDBRoot::countSelectedEntities(CC_CLASS_ENUM filter)
{
    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    int selCount = selectedIndexes.size();

    if (selCount == 0 || filter == CC_OBJECT)
        return selCount;

    int i,realCount = 0;
    for (i=0;i<selCount; ++i)
    {
        ccHObject* anObject = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
        if (anObject && anObject->isKindOf(filter))
            ++realCount;
    }

    return realCount;
}

int ccDBRoot::getSelectedEntities(ccHObject::Container& selEntities,
                                    CC_CLASS_ENUM filter/*=CC_OBJECT*/,
                                    dbTreeSelectionInfo* info/*=NULL*/)
{
    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    int i,selCount = selectedIndexes.size();

    for (i=0;i<selCount; ++i)
    {
        ccHObject* anObject = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
        if (anObject && anObject->isKindOf(filter))
            selEntities.push_back(anObject);
    }

    if (info)
    {
        info->reset();
        info->selCount=selCount;

        for (i=0;i<selCount;++i)
        {
            ccHObject* obj = selEntities[i];

            info->sfCount += int(obj->hasScalarFields());
            info->colorCount += int(obj->hasColors());
            info->normalsCount += int(obj->hasNormals());

            if (obj->isKindOf(CC_POINT_CLOUD))
            {
                ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj);
                info->cloudCount++;
                info->octreeCount += int(cloud->getOctree()!=NULL);
            }

            if (obj->isKindOf(CC_MESH))
                info->meshCount++;

            if (obj->isKindOf(CC_SENSOR))
            {
                info->sensorCount++;
                if (obj->isKindOf(CC_GBL_SENSOR))
                    info->gblSensorCount++;
            }

            if (obj->isKindOf(CC_POINT_KDTREE))
				info->kdTreeCount++;
        }
    }

    return int(selEntities.size());
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
	if (item)
	{
		if (item->isA(CC_HIERARCHY_OBJECT)	||
			item->isKindOf(CC_POINT_CLOUD)	||
			item->isKindOf(CC_MESH)			||
			item->isKindOf(CC_IMAGE)		||
			item->isKindOf(CC_2D_LABEL)		||
			item->isKindOf(CC_PRIMITIVE))
		{
			defaultFlags |= (Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
		}
		else if (item->isKindOf(CC_2D_VIEWPORT_OBJECT))
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
		ccHObject* anObject = static_cast<ccHObject*>(index.internalPointer());
		if (anObject)
			map.insert(Qt::UserRole,QVariant(anObject->getUniqueID()));
	}
	
	return map;
}


bool ccDBRoot::dropMimeData(const QMimeData* data, Qt::DropAction action, int destRow, int destColumn, const QModelIndex& destParent)
{
	if (action != Qt::MoveAction)
         return false;

	//default mime type for QAbstractItemModel items)
	if (!data->hasFormat("application/x-qabstractitemmodeldatalist"))
		return false;

	//new parent (can't be a leaf object!)
	ccHObject *newParent = destParent.isValid() ? static_cast<ccHObject*>(destParent.internalPointer()) : m_treeRoot;
	if (newParent && newParent->isLeaf())
		return false;

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
		//ccConsole::Print(QString("[Drag & Drop] Source: %1").arg(item->getName()));

		//old parent
		ccHObject* oldParent = item->getParent();
		//ccConsole::Print(QString("[Drag & Drop] Parent: %1").arg(oldParent ? oldParent->getName() : "none")));

		//let's check if we can actually move the entity
		if (oldParent)
		{
			if (item->isKindOf(CC_POINT_CLOUD))
			{
				//point cloud == mesh vertices?
				if (oldParent->isKindOf(CC_MESH) && ccHObjectCaster::ToGenericMesh(oldParent)->getAssociatedCloud() == item)
					if (oldParent != newParent)
					{
						ccConsole::Error("Vertices can't leave their parent mesh!");
						return false;
					}
			}
			else if (item->isKindOf(CC_MESH))
			{
				//a mesh can't leave it's mesh group
				if (oldParent->isA(CC_MESH_GROUP) && ccHObjectCaster::ToGenericMesh(item)->getAssociatedCloud() == ccHObjectCaster::ToGenericMesh(oldParent)->getAssociatedCloud())
				{
					if (oldParent != newParent)
					{
						ccConsole::Error("Sub-meshes can't leave their mesh group!");
						return false;
					}
				}
				//a mesh can't leave it's associated cloud
				else if (oldParent->isKindOf(CC_POINT_CLOUD) &&  ccHObjectCaster::ToGenericMesh(item)->getAssociatedCloud() == oldParent)
				{
					if (oldParent != newParent)
					{
						ccConsole::Error("Sub-meshes can't leave their associated cloud!");
						return false;
					}
				}
				//a mesh can't be inserted in a mesh group
				else if (newParent->isA(CC_MESH_GROUP) && ccHObjectCaster::ToGenericMesh(item)->getAssociatedCloud() != ccHObjectCaster::ToGenericMesh(newParent)->getAssociatedCloud())
				{
					ccConsole::Error("Outside meshes can't be added to mesh groups!");
					return false;
				}
			}
			else if (/*item->isKindOf(CC_PRIMITIVE) || */item->isKindOf(CC_IMAGE))
			{
				if (oldParent != newParent)
				{
					ccConsole::Error("This kind of entity can't leave their parent!");
					return false;
				}
			}
			else if (oldParent != newParent)
			{
				//a label or a group of labels can't be moved to another cloud!
				ccHObject::Container labels;
				if (item->isA(CC_2D_LABEL))
					labels.push_back(item);
				else
					item->filterChildren(labels,true,CC_2D_LABEL);

				//for all labels in the sub-tree
				for (ccHObject::Container::const_iterator it = labels.begin(); it != labels.end(); ++it)
				{
					cc2DLabel* label = static_cast<cc2DLabel*>(*it);
					bool canMove = false;
					for (unsigned j=0;j<label->size();++j)
					{
						assert(label->getPoint(j).cloud);
						//3 options to allow moving a label:
						if (item->isAncestorOf(label->getPoint(j).cloud) //label's cloud is inside sub-tree
							|| newParent == label->getPoint(j).cloud //destination is label's cloud
							|| label->getPoint(j).cloud->isAncestorOf(newParent)) //destination is below label's cloud
						{
							canMove = true;
							break;
						}
					}

					if (!canMove)
					{
						ccConsole::Error("Labels (or group of) can't leave their parent!");
						return false;
					}
				}
			}
		}

		//special case: moving an item inside the same 'parent'
		if (oldParent && newParent == oldParent)
		{
			int oldRow = newParent->getChildIndex(item);
			if (destRow<0)
			{
				assert(newParent->getChildrenNumber()>0);
				destRow = (int)newParent->getChildrenNumber()-1;
			}
			else if (oldRow<destRow)
			{
				assert(destRow>0);
				--destRow;
			}
			else if (oldRow==destRow)
				return false; //nothing to do
		}

		//remove link from old parent
		bool fatherDependant = false;
		if (item->getFlagState(CC_FATHER_DEPENDANT))
		{
			fatherDependant = true;
			item->setFlagState(CC_FATHER_DEPENDANT,false);
		}

		//remove item from current position
		removeElement(item);

		//sets new parent
		assert(newParent);
		newParent->addChild(item,fatherDependant,destRow);

		if (newParent->getDisplay() == 0)
			newParent->setDisplay(item->getDisplay());
		else if (item->getDisplay() == 0)
			item->setDisplay(newParent->getDisplay());

		//add item back
		addElement(item,false);
	}

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
	if (m_contextMenuPos.x()<0 || m_contextMenuPos.y()<0)
		return;

	QModelIndex clickIndex = m_dbTreeWidget->indexAt(m_contextMenuPos);
	if (!clickIndex.isValid())
		return;
	ccHObject* item = static_cast<ccHObject*>(clickIndex.internalPointer());
	assert(item);

	if (!item || item->getChildrenNumber()==0)
		return;
	
	//we recursively expand sub-branches
	ccHObject::Container toExpand;
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

		assert(item->getChildrenNumber()>0);
		for (unsigned i=0;i<item->getChildrenNumber();++i)
		{
			if (item->getChild(i)->getChildrenNumber()>0)
				toExpand.push_back(item->getChild(i));
		}
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
		ccLog::Warning("[alignCameraWithEntity] Selected entity has no associated display!");
		return;
	}
	assert(display);
	ccGLWindow* win = static_cast<ccGLWindow*>(display);

	//plane normal
	CCVector3 planeNormal;
	CCVector3 planeVertDir;

	//2D label with 3 points?
	if (obj->isA(CC_2D_LABEL))
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
			planeNormal = (*_B-*_A).cross(*_C-*_A);
			planeVertDir = /*(*_B-*_A)*/win->getCurrentUpDir();
		}
		else
		{
			assert(false);
			return;
		}
	}
	//plane ?
	else if (obj->isA(CC_PLANE))
	{
		ccPlane* plane = static_cast<ccPlane*>(obj);
		//3rd column = plane normal!
		planeNormal = plane->getNormal();
		planeVertDir = CCVector3(plane->getTransformation().getColumn(1));
	}
	else
	{
		assert(false);
		return;
	}

	//we can now make the camera look in the direction of the normal
	CCVector3 forward = (reverse ? planeNormal : -planeNormal);
	win->setCustomView(forward,planeVertDir);
}

void ccDBRoot::gatherRecursiveInformation()
{
    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    int i,selCount = selectedIndexes.size();
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
	} info;

	memset(&info,0,sizeof(GlobalInfo));

	//init the list of entities to process
	ccHObject::Container toProcess;
	try
	{
		toProcess.resize(selCount);
	}
	catch(std::bad_alloc)
	{
		ccLog::Error("Not engough memory!");
		return;
	}
	{
		for (i=0;i<selCount;++i)
			toProcess[i] = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
	}

	ccHObject::Container alreadyProcessed;
	while (!toProcess.empty())
	{
		ccHObject* ent = toProcess.back();
		toProcess.pop_back();

		//we don't process entities twice!
		if (std::find(alreadyProcessed.begin(), alreadyProcessed.end(), ent) != alreadyProcessed.end())
			continue;

		//gather information from current entity
		if (ent->isA(CC_POINT_CLOUD))
		{
			ccPointCloud* cloud = static_cast<ccPointCloud*>(ent);
			info.cloudCount++;
			
			unsigned cloudSize = cloud->size();
			info.pointCount += cloudSize;
			info.colorCount += (cloud->hasColors() ? cloudSize : 0);
			info.normalCount += (cloud->hasNormals() ? cloudSize : 0);
			info.scalarFieldCount += cloud->getNumberOfScalarFields();
		}
		else if (ent->isKindOf(CC_MESH))
		{
			ccMesh* mesh = static_cast<ccMesh*>(ent);

			if (!ent->isA(CC_MESH_GROUP)) //CC_MESH_GROUPs already return the sum of their children sizes!
			{
				info.meshCount++;
				unsigned meshSize = mesh->size();
				info.triangleCount += meshSize;
				info.normalCount += (mesh->hasTriNormals() ? meshSize : 0);
			}
			info.materialCount += (mesh->getMaterialSet() ? (unsigned)mesh->getMaterialSet()->size() : 0);
		}
		else if (ent->isKindOf(CC_2D_LABEL))
		{
			info.labelCount++;
		}
		else if (ent->isKindOf(CC_SENSOR))
		{
			info.sensorCount++;
		}
		else if (ent->isKindOf(CC_POINT_OCTREE))
		{
			info.octreeCount++;
		}
		else if (ent->isKindOf(CC_IMAGE))
		{
			info.imageCount++;
		}

		//we can add its children to the 'toProcess' list and itself to the 'processed' list 
		try
		{
			for (unsigned i=0;i<ent->getChildrenNumber();++i)
				toProcess.push_back(ent->getChild(i));
			alreadyProcessed.push_back(ent);
		}
		catch(std::bad_alloc)
		{
			ccLog::Error("Not engough memory!");
			return;
		}
	}

	//output information
	{
		QStringList infoStr;
		QLocale locale(QLocale::English);
		QString separator("--------------------------");

		infoStr << QString("Point(s):\t\t%1").arg(locale.toString(info.pointCount));
		infoStr << QString("Triangle(s):\t\t%1").arg(locale.toString(info.triangleCount));

		infoStr << separator;
		if (info.colorCount)
			infoStr << QString("Color(s):\t\t%1").arg(locale.toString(info.colorCount));
		if (info.normalCount)
			infoStr << QString("Normal(s):\t\t%1").arg(locale.toString(info.normalCount));
		if (info.scalarFieldCount)
			infoStr << QString("Scalar field(s):\t\t%1").arg(locale.toString(info.scalarFieldCount));
		if (info.materialCount)
			infoStr << QString("Material(s):\t\t%1").arg(locale.toString(info.materialCount));

		infoStr << separator;
		infoStr << QString("Cloud(s):\t\t%1").arg(locale.toString(info.cloudCount));
		infoStr << QString("Mesh(es):\t\t%1").arg(locale.toString(info.meshCount));
		if (info.octreeCount)
			infoStr << QString("Octree(s):\t\t%1").arg(locale.toString(info.octreeCount));
		if (info.imageCount)
			infoStr << QString("Image(s):\t\t%1").arg(locale.toString(info.imageCount));
		if (info.labelCount)
			infoStr << QString("Label(s):\t\t%1").arg(locale.toString(info.labelCount));
		if (info.sensorCount)
			infoStr << QString("Sensor(s):\t\t%1").arg(locale.toString(info.sensorCount));

		//display info box
		QMessageBox::information(MainWindow::TheInstance(),
								"Information",
								infoStr.join("\n"));
	}
}

void ccDBRoot::sortSiblingsAZ()
{
	sortSelectedEntitiesSiblings(SORT_A2Z);
}

void ccDBRoot::sortSiblingsZA()
{
	sortSelectedEntitiesSiblings(SORT_Z2A);
}

void ccDBRoot::sortSiblingsType()
{
	sortSelectedEntitiesSiblings(SORT_BY_TYPE);
}

void ccDBRoot::sortSelectedEntitiesSiblings(SortRules sortRule)
{
    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    int i,selCount = selectedIndexes.size();
    if (selCount == 0)
        return;

	for (i=0;i<selCount;++i)
    {
        ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		unsigned childCount = (item ? item->getChildrenNumber() : 0);
		if (childCount>1)
		{
			//remove all children from DB tree
			beginRemoveRows(selectedIndexes[i],0,childCount-1);

			//row removal operation (end)
			endRemoveRows();

			//sort
			for (unsigned k=0;k<childCount-1;++k)
			{
				unsigned firstChildIndex = k;
				ccHObject* firstChild = item->getChild(k);
				QString firstChildName = firstChild->getName().toUpper();

				for (unsigned j=k+1;j<childCount;++j)
				{
					bool swap = false;
					QString currentName = item->getChild(j)->getName().toUpper();
					switch(sortRule)
					{
					case SORT_A2Z:
						swap = (firstChildName.compare(currentName)>0);
						break;
					case SORT_Z2A:
						swap = (firstChildName.compare(currentName)<0);
						break;
					case SORT_BY_TYPE:
						if (firstChild->getClassID() == item->getChild(j)->getClassID())
							swap = (firstChildName.compare(currentName)>0); //A2Z in second choice
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

				if (k!=firstChildIndex)
					item->swapChildren(k,firstChildIndex);
			}

			//add children back
			beginInsertRows(selectedIndexes[i],0,childCount-1);

			//row insertion operation (end)
			endInsertRows();
		}
	}
}

void ccDBRoot::toggleSelectedEntities()
{
	toggleSelectedEntitiesProperty(0);
}

void ccDBRoot::toggleSelectedEntitiesVisibility()
{
	toggleSelectedEntitiesProperty(1);
}

void ccDBRoot::toggleSelectedEntitiesColor()
{
	toggleSelectedEntitiesProperty(2);
}

void ccDBRoot::toggleSelectedEntitiesNormals()
{
	toggleSelectedEntitiesProperty(3);
}

void ccDBRoot::toggleSelectedEntitiesSF()
{
	toggleSelectedEntitiesProperty(4);
}

void ccDBRoot::toggleSelectedEntitiesMat()
{
	toggleSelectedEntitiesProperty(5);
}

void ccDBRoot::toggleSelectedEntities3DName()
{
	toggleSelectedEntitiesProperty(6);
}

void ccDBRoot::toggleSelectedEntitiesProperty(unsigned prop)
{
	if (prop>6)
	{
		ccConsole::Warning("[ccDBRoot::toggleSelectedEntitiesProperty] Internal error: invalid 'prop' value");
		return;
	}

    QItemSelectionModel* qism = m_dbTreeWidget->selectionModel();
	QModelIndexList selectedIndexes = qism->selectedIndexes();
    int i,selCount = selectedIndexes.size();
    if (selCount == 0)
        return;

	//hide properties view
	hidePropertiesView();

	for (i=0;i<selCount;++i)
    {
        ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
		assert(item);
		if (!item)
			continue;
		switch(prop)
		{
		case 0: //enable state
			item->setEnabled(!item->isEnabled());
			break;
		case 1: //visibility
			item->toggleVisibility();
			break;
		case 2: //color
			item->toggleColors();
			break;
		case 3: //normal
			item->toggleNormals();
			break;
		case 4: //SF
			item->toggleSF();
			break;
		case 5: //Materials/textures
			item->toggleMaterials();
			break;
		case 6: //3D name
			item->toggleShowName();
			break;
		}
		item->prepareDisplayForRefresh();
	}

    //we restablish properties view
    updatePropertiesView();

	MainWindow::RefreshAllGLWindow();
}

void ccDBRoot::addEmptyGroup()
{
	//not initialized?
	if (m_contextMenuPos.x()<0 || m_contextMenuPos.y()<0)
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
		int i,selCount = selectedIndexes.size();
		if (selCount)
		{
			bool toggleVisibility=false;
			bool toggleOtherProperties=false;
			bool toggleMaterials=false;
			bool hasMoreThan2Children=false;
			bool hasExactlyOnePlanarEntity=false;
			bool leafObject=false;
			for (i=0;i<selCount;++i)
			{
				ccHObject* item = static_cast<ccHObject*>(selectedIndexes[i].internalPointer());
				assert(item);
				if (!item)
					continue;
				if (item->getChildrenNumber()>1)
					hasMoreThan2Children=true;
				leafObject |= item->isLeaf();
				if (!item->isA(CC_HIERARCHY_OBJECT))
				{
					toggleVisibility = true;
					if (item->isKindOf(CC_POINT_CLOUD))
					{
						toggleOtherProperties = true;
					}
					else if (item->isKindOf(CC_MESH))
					{
						toggleMaterials = true;
						toggleOtherProperties = true;
					}
					
					if (selCount == 1)
					{
						if (item->isKindOf(CC_2D_LABEL))
						{
							hasExactlyOnePlanarEntity = (static_cast<cc2DLabel*>(item)->size() == 3);
						}
						else if (item->isA(CC_PLANE))
						{
							hasExactlyOnePlanarEntity = true;
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
			if (hasMoreThan2Children)
			{
				menu.addSeparator();
				menu.addAction(m_sortSiblingsAZ);
				menu.addAction(m_sortSiblingsZA);
				menu.addAction(m_sortSiblingsType);
			}
			if (selCount==1 && !leafObject)
			{
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
			if (selectedItem && selectedItem->isA(CC_2D_LABEL) != static_cast<ccHObject*>(selectedIndexes[0].internalPointer())->isA(CC_2D_LABEL))
				return QItemSelectionModel::ClearAndSelect;
		}
	}

	return QTreeView::selectionCommand(index,event);
}
