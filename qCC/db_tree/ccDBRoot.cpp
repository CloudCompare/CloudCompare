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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2275                                                              $
//$LastChangedDate:: 2012-10-17 23:30:43 +0200 (mer., 17 oct. 2012)        $
//**************************************************************************
//

#include "ccDBRoot.h"

//Qt
#include <QTreeView>
#include <QStandardItemModel>
#include <QHeaderView>
#include <QMimeData>

//qCC_db
#include <ccHObject.h>
#include <ccGenericPointCloud.h>
#include <cc2DLabel.h>

#include "ccPropertiesTreeDelegate.h"
#include "../ccConsole.h"
#include "../mainwindow.h"

//system
#include <assert.h>
#include <algorithm>

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
	m_sortSiblingsType = new QAction("Sort siblings by type",this);
	m_sortSiblingsAZ = new QAction("Sort siblings by name (A-Z)",this);
	m_sortSiblingsZA = new QAction("Sort siblings by name (Z-A)",this);
	m_deleteSelectedEntities = new QAction("Delete",this);
	m_toggleSelectedEntities = new QAction("Toggle",this);
	m_toggleSelectedEntitiesVisibility = new QAction("Toggle visibility",this);
	m_toggleSelectedEntitiesColor = new QAction("Toggle color",this);
	m_toggleSelectedEntitiesNormals = new QAction("Toggle normals",this);
	m_toggleSelectedEntitiesSF = new QAction("Toggle SF",this);
	m_addEmptyGroup = new QAction("Add empty group",this);

	m_contextMenuPos = QPoint(-1,-1);

	//connect custom context menu actions
	connect(m_dbTreeWidget,						SIGNAL(customContextMenuRequested(const QPoint&)),	this, SLOT(showContextMenu(const QPoint&)));
	connect(m_expandBranch,						SIGNAL(triggered()),								this, SLOT(expandBranch()));
	connect(m_collapseBranch,					SIGNAL(triggered()),								this, SLOT(collapseBranch()));
	connect(m_sortSiblingsAZ,					SIGNAL(triggered()),								this, SLOT(sortSiblingsAZ()));
	connect(m_sortSiblingsZA,					SIGNAL(triggered()),								this, SLOT(sortSiblingsZA()));
	connect(m_sortSiblingsType,					SIGNAL(triggered()),								this, SLOT(sortSiblingsType()));
	connect(m_deleteSelectedEntities,			SIGNAL(triggered()),								this, SLOT(deleteSelectedEntities()));
	connect(m_toggleSelectedEntities,			SIGNAL(triggered()),								this, SLOT(toggleSelectedEntities()));
	connect(m_toggleSelectedEntitiesVisibility,	SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesVisibility()));
	connect(m_toggleSelectedEntitiesColor,		SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesColor()));
	connect(m_toggleSelectedEntitiesNormals,	SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesNormals()));
	connect(m_toggleSelectedEntitiesSF,			SIGNAL(triggered()),								this, SLOT(toggleSelectedEntitiesSF()));
	connect(m_addEmptyGroup,					SIGNAL(triggered()),								this, SLOT(addEmptyGroup()));
	

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
				if (static_cast<ccGenericMesh*>(obj->getParent())->getAssociatedCloud() == obj)
				{
					ccConsole::Warning("Mesh vertices can't be deleted without their parent mesh!");
					continue;
				}

			toBeDeleted.push_back(obj);
		}
	}

    qism->clear();

	while (!toBeDeleted.empty())
	{
		ccHObject* anObject = toBeDeleted.back();
		assert(anObject);
		toBeDeleted.pop_back();

		anObject->prepareDisplayForRefresh_recursive();

		//DGM FIXME: what a burden... we should find something simpler (shared pointers?)
		//specific case: if the entity is a cloud, we must look for 2-points
		//or 3-points labels that may have a dependence to it
		if (anObject->isKindOf(CC_POINT_CLOUD))
		{
			ccHObject::Container allLabels;
			if (m_treeRoot->filterChildren(allLabels,true,CC_2D_LABEL) != 0)
			{
				unsigned labelCount = allLabels.size();
				for (unsigned i=0;i<allLabels.size();++i)
				{
					cc2DLabel* label = static_cast<cc2DLabel*>(allLabels[i]);
					for (unsigned j=1;j<label->size();++j) //the first point is always the parent cloud!
						if (label->getPoint(j).cloud == anObject)
						{
							ccLog::Warning(QString("Label '%1' has been deleted as it is dependent on '%2'").arg(label->getName()).arg(anObject->getName()));
							label->clear();
							toBeDeleted.push_back(label);
						}
				}
			}
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
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/hObjectSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/hObjectSymbol.png"));
            case CC_POINT_CLOUD:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/cloudSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/cloudSymbol.png"));
			//all primitives
			case CC_PLANE:
			case CC_SPHERE:
			case CC_TORUS:	
			case CC_CYLINDER:
			case CC_CONE:	
			case CC_BOX:	
			case CC_DISH:	
			case CC_EXTRU:	
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/miscGeomSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/miscGeomSymbol.png"));
            case CC_MESH:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/meshSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/meshSymbol.png"));
            case CC_MESH_GROUP:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/meshGroupSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/meshGroupSymbol.png"));
            case CC_POINT_OCTREE:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/octreeSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/octreeSymbol.png"));
            case CC_CALIBRATED_IMAGE:
                return QIcon(QString::fromUtf8(":/CC/Tree/images/calibratedImageSymbol.png"));
            case CC_IMAGE:
                return QIcon(QString::fromUtf8(":/CC/Tree/images/imageSymbol.png"));
            case CC_SENSOR:
            case CC_GBL_SENSOR:
                return QIcon(QString::fromUtf8(":/CC/Tree/images/sensorSymbol.png"));
			case CC_MATERIAL_SET:
                return QIcon(QString::fromUtf8(":/CC/Tree/images/materialSymbol.png"));
			case CC_NORMALS_ARRAY:
			case CC_NORMAL_INDEXES_ARRAY:
			case CC_RGB_COLOR_ARRAY:
			case CC_TEX_COORDS_ARRAY:
                if (locked)
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/dbSymbolLocked.png"));
                else
                    return QIcon(QString::fromUtf8(":/CC/Tree/images/dbSymbol.png"));
			case CC_2D_LABEL:
				return QIcon(QString::fromUtf8(":/CC/Tree/images/labelSymbol.png"));
			case CC_2D_VIEWPORT_OBJECT:
				return QIcon(QString::fromUtf8(":/CC/Tree/images/viewportSymbol.png"));
			case CC_2D_VIEWPORT_LABEL:
				return QIcon(QString::fromUtf8(":/CC/Comp/images/comp/rectangleSelect.png"));
            default:
                if (locked)
                   return QIcon(QString::fromUtf8(":/CC/Tree/images/lock.png"));
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

				//particular case: labels name is their title!
				if (item->isKindOf(CC_2D_LABEL))
					if (item->isVisible() && item->isEnabled() && item->getDisplay())
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
        ccConsole::Error("An error occured while inserting element in DB tree: object with no parent!");
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

void ccDBRoot::selectEntity(int uniqueID)
{
	ccHObject* obj = 0;
    //minimum unqiue ID is 1 (0 means 'deselect')
    if (uniqueID>0)
        obj = find(uniqueID);

	selectEntity(obj);
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
				QItemSelectionModel::SelectionFlags ctrlSelFlags = QItemSelectionModel::Toggle;
				//special case: labels can only be merged with labels!
				if (!obj->isSelected())
				{
					QModelIndexList selectedIndexes = selectionModel->selectedIndexes();
					if (!selectedIndexes.empty())
					{
						if (obj->isA(CC_2D_LABEL) != static_cast<ccHObject*>(selectedIndexes[0].internalPointer())->isA(CC_2D_LABEL))
							ctrlSelFlags = QItemSelectionModel::ClearAndSelect;
					}
				}
				selectionModel->select(selectedIndex,ctrlSelFlags);
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
		selectionModel->clear();
}

ccHObject* ccDBRoot::find(int uniqueID) const
{
	return m_treeRoot->find(uniqueID);
}

void ccDBRoot::showPropertiesView(ccHObject* obj)
{
    m_ccPropDelegate->fillModel(obj);

    m_propertiesTreeWidget->setEnabled(true);
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
                ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(obj);
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
		if (item->isA(CC_HIERARCHY_OBJECT) ||
			item->isKindOf(CC_POINT_CLOUD) ||
			item->isKindOf(CC_MESH)        ||
			item->isKindOf(CC_IMAGE)       ||
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
				if (oldParent->isKindOf(CC_MESH) && static_cast<ccGenericMesh*>(oldParent)->getAssociatedCloud() == item)
					if (oldParent != newParent)
					{
						ccConsole::Error("Vertices can't leave their parent mesh!");
						return false;
					}
			}
			else if (item->isKindOf(CC_MESH))
			{
				//a mesh can't leave it's mesh group
				if (oldParent->isA(CC_MESH_GROUP) && static_cast<ccGenericMesh*>(item)->getAssociatedCloud() == static_cast<ccGenericMesh*>(oldParent)->getAssociatedCloud())
				{
					if (oldParent != newParent)
					{
						ccConsole::Error("Sub-meshes can't leave their mesh group!");
						return false;
					}
				}
				//a mesh can't leave it's associated cloud
				else if (oldParent->isKindOf(CC_POINT_CLOUD) &&  static_cast<ccGenericMesh*>(item)->getAssociatedCloud() == oldParent)
				{
					if (oldParent != newParent)
					{
						ccConsole::Error("Sub-meshes can't leave their associated cloud!");
						return false;
					}
				}
				//a mesh can't be inserted in a mesh group
				else if (newParent->isA(CC_MESH_GROUP) && static_cast<ccGenericMesh*>(item)->getAssociatedCloud() != static_cast<ccGenericMesh*>(newParent)->getAssociatedCloud())
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

//void ccDBRoot::expandOrCollapseHoveredBranch(bool expand)
//{
//	//not initialized?
//	if (m_contextMenuPos.x()<0 || m_contextMenuPos.y()<0)
//		return;
//
//	QModelIndex index = m_dbTreeWidget->indexAt(m_contextMenuPos);
//	if (index.isValid())
//	{
//		if (expand)
//			m_dbTreeWidget->expand(index);
//		else
//			m_dbTreeWidget->collapse(index);
//	}
//}

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

void ccDBRoot::toggleSelectedEntitiesProperty(unsigned prop)
{
	if (prop>4)
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
			item->setVisible(!item->isVisible());
			break;
		case 2: //color
			item->showColors(!item->colorsShown());
			break;
		case 3: //normal
			item->showNormals(!item->normalsShown());
			break;
		case 4: //SF
			item->showSF(!item->sfShown());
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

void ccDBRoot::showContextMenu(const QPoint& pnt)
{
	m_contextMenuPos = pnt;

	//build custom context menu
	QMenu menu;

	QModelIndex index = m_dbTreeWidget->indexAt(pnt);
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
			bool hasMoreThan2Children=false;
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
					if (item->isKindOf(CC_POINT_CLOUD) || item->isKindOf(CC_MESH))
					{
						toggleOtherProperties = true;
						if (hasMoreThan2Children)
							break; //no need to get more information!
					}
				}
			}

			menu.addAction(m_toggleSelectedEntities);
			if (toggleVisibility)
				menu.addAction(m_toggleSelectedEntitiesVisibility);
			if (toggleOtherProperties)
			{
				menu.addAction(m_toggleSelectedEntitiesColor);
				menu.addAction(m_toggleSelectedEntitiesNormals);
				menu.addAction(m_toggleSelectedEntitiesSF);
			}
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

	menu.exec(m_dbTreeWidget->mapToGlobal(pnt));
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
