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

#include "ccPropertiesTreeDelegate.h"

//Local
#include "sfEditDlg.h"
#include "../ccConsole.h"
#include "../ccGLWindow.h"
#include "../mainwindow.h"
#include "../ccGuiParameters.h"
#include "../ccColorScaleEditorDlg.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccOctree.h>
#include <ccKdTree.h>
#include <ccImage.h>
#include <cc2DLabel.h>
#include <cc2DViewportLabel.h>
#include <cc2DViewportObject.h>
#include <ccCalibratedImage.h>
#include <ccGBLSensor.h>
#include <ccMaterialSet.h>
#include <ccAdvancedTypes.h>
#include <ccGenericPrimitive.h>

//Qt
#include <QStandardItemModel>
#include <QAbstractItemView>
#include <QSpinBox>
#include <QSlider>
#include <QComboBox>
#include <QCheckBox>
#include <QLocale>
#include <QPushButton>
#include <QScrollBar>
#include <QHBoxLayout>
#include <QToolButton>

//System
#include <assert.h>

// Default 'None' string
const QString c_noDisplayString = QString("None");
const QString c_defaultPointSizeString = QString("Default");

// Default separator colors
const QBrush SEPARATOR_BACKGROUND_BRUSH(Qt::darkGray);
const QBrush SEPARATOR_TEXT_BRUSH(Qt::white);

//! Advanced editor for color scales
class QColorScaleSelector : public QFrame
{
public:

	QColorScaleSelector(QWidget* parent)
		: QFrame(parent)
		, m_comboBox(new QComboBox())
		, m_button(new QToolButton())
	{
		setLayout(new QHBoxLayout());
		layout()->setContentsMargins(0,0,0,0);
		setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

		//combox box
		if (m_comboBox)
		{
			layout()->addWidget(m_comboBox);
		}
		
		//tool button
		if (m_button)
		{
			m_button->setIcon(QIcon(QString::fromUtf8(":/CC/images/ccGear.png")));
			layout()->addWidget(m_button);
		}
	}
	
	QComboBox* m_comboBox;
	QToolButton* m_button;
};

ccPropertiesTreeDelegate::ccPropertiesTreeDelegate(QStandardItemModel* model,
												   QAbstractItemView* view,
												   QObject *parent)
		: QStyledItemDelegate(parent)
		, m_currentObject(0)
		, m_model(model)
		, m_view(view)
{
	assert(m_model && m_view);
}

ccPropertiesTreeDelegate::~ccPropertiesTreeDelegate()
{
    unbind();
}

QSize ccPropertiesTreeDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	assert(m_model);

	QStandardItem* item = m_model->itemFromIndex(index);

	if (item && item->data().isValid())
    {
        switch (item->data().toInt())
        {
        case OBJECT_CURRENT_DISPLAY:
        case OBJECT_CURRENT_SCALAR_FIELD:
        case OBJECT_OCTREE_TYPE:
        case OBJECT_COLOR_RAMP_STEPS:
        case OBJECT_CLOUD_POINT_SIZE:
            return QSize(50,18);
        case OBJECT_CURRENT_COLOR_RAMP:
            return QSize(70,22);
		case OBJECT_CLOUD_SF_EDITOR:
            return QSize(250,160);
        }
    }

    return QStyledItemDelegate::sizeHint(option,index);
}
//*/

void ccPropertiesTreeDelegate::unbind()
{
    if (m_model)
		m_model->disconnect(this);
}

ccHObject* ccPropertiesTreeDelegate::getCurrentObject()
{
    return m_currentObject;
}

void ccPropertiesTreeDelegate::fillModel(ccHObject* hObject)
{
    if (!hObject)
        return;

    unbind();

    m_currentObject = hObject;

	//save current scroll position
	int scrollPos = (m_view && m_view->verticalScrollBar() ? m_view->verticalScrollBar()->value() : 0);

	if (m_model)
	{
		m_model->removeRows(0,m_model->rowCount());
		m_model->setColumnCount(2);
		m_model->setHeaderData(0, Qt::Horizontal, "Property");
		m_model->setHeaderData(1, Qt::Horizontal, "State/Value");
	}

    if (m_currentObject->isHierarchy())
		if (!m_currentObject->isA(CC_2D_VIEWPORT_LABEL)) //don't need to display this kind of info for viewport labels!
			fillWithHObject(m_currentObject);

    if (m_currentObject->isKindOf(CC_POINT_CLOUD))
    {
        fillWithPointCloud(ccHObjectCaster::ToGenericPointCloud(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_MESH))
    {
		fillWithMesh(ccHObjectCaster::ToGenericMesh(m_currentObject));
    
		if (m_currentObject->isKindOf(CC_PRIMITIVE))
			fillWithPrimitive(ccHObjectCaster::ToPrimitive(m_currentObject));
    }
    else if (m_currentObject->isA(CC_POINT_OCTREE))
    {
		fillWithPointOctree(ccHObjectCaster::ToOctree(m_currentObject));
    }
    else if (m_currentObject->isA(CC_POINT_KDTREE))
    {
		fillWithPointKdTree(ccHObjectCaster::ToKdTree(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_IMAGE))
    {
		fillWithImage(ccHObjectCaster::ToImage(m_currentObject));

        if (m_currentObject->isA(CC_CALIBRATED_IMAGE))
			fillWithCalibratedImage(ccHObjectCaster::ToCalibratedImage(m_currentObject));
    }
    else if (m_currentObject->isA(CC_2D_LABEL))
    {
		fillWithLabel(ccHObjectCaster::To2DLabel(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_2D_VIEWPORT_OBJECT))
    {
		fillWithViewportObject(ccHObjectCaster::To2DViewportLabel(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_GBL_SENSOR))
    {
		fillWithGBLSensor(ccHObjectCaster::ToGBLSensor(m_currentObject));
    }
    else if (m_currentObject->isA(CC_MATERIAL_SET))
    {
        fillWithMaterialSet(static_cast<ccMaterialSet*>(m_currentObject));
    }
    else if (m_currentObject->isA(CC_NORMAL_INDEXES_ARRAY))
    {
        fillWithChunkedArray(static_cast<NormsIndexesTableType*>(m_currentObject));
    }
    else if (m_currentObject->isA(CC_TEX_COORDS_ARRAY))
    {
        fillWithChunkedArray(static_cast<TextureCoordsContainer*>(m_currentObject));
    }
    else if (m_currentObject->isA(CC_NORMALS_ARRAY))
    {
        fillWithChunkedArray(static_cast<NormsTableType*>(m_currentObject));
    }
    else if (m_currentObject->isA(CC_RGB_COLOR_ARRAY))
    {
        fillWithChunkedArray(static_cast<ColorsTableType*>(m_currentObject));
    }
	
	//go back to original position
	if (scrollPos>0)
		m_view->verticalScrollBar()->setSliderPosition(scrollPos);

	if (m_model)
		connect(m_model, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateItem(QStandardItem*)));
}

void ccPropertiesTreeDelegate::appendRow(QStandardItem* leftItem, QStandardItem* rightItem, bool openPersistentEditor/*=false*/)
{
	assert(leftItem && rightItem);
    assert(m_model);

	if (m_model)
	{
		//append row
		QList<QStandardItem*> rowItems;
		{
			rowItems.push_back(leftItem);
			rowItems.push_back(rightItem);
		}
		m_model->appendRow(rowItems);

		//the presistent editor (if any) is always the right one!
		if (openPersistentEditor)
			m_view->openPersistentEditor(m_model->index(m_model->rowCount()-1,1));
	}
}

void ccPropertiesTreeDelegate::addSeparator(QString title)
{
    //name
    QStandardItem* leftItem = new QStandardItem(title);
    leftItem->setForeground(SEPARATOR_TEXT_BRUSH);
    leftItem->setBackground(SEPARATOR_BACKGROUND_BRUSH);

	//empty
    QStandardItem* rightItem = new QStandardItem();
    rightItem->setForeground(SEPARATOR_TEXT_BRUSH);
    rightItem->setBackground(SEPARATOR_BACKGROUND_BRUSH);
    
	//append row
	appendRow(leftItem,rightItem);
}

//Shortcut to create a delegate item
QStandardItem* ITEM(QString name,
					Qt::ItemFlag additionalFlags = Qt::NoItemFlags,
					ccPropertiesTreeDelegate::CC_PROPERTY_ROLE role = ccPropertiesTreeDelegate::OBJECT_NO_PROPERTY)
{
	QStandardItem* item = new QStandardItem(name);
	//flags
	item->setFlags(Qt::ItemIsEnabled | additionalFlags);
	//role (if any)
	if (role != ccPropertiesTreeDelegate::OBJECT_NO_PROPERTY)
		item->setData(role);

	return item;
}

//Shortcut to create a checkable delegate item
QStandardItem* CHECKABLE_ITEM(bool checkState, ccPropertiesTreeDelegate::CC_PROPERTY_ROLE role)
{
	QStandardItem* item = ITEM("",Qt::ItemIsUserCheckable,role);
	//check  state
	item->setCheckState(checkState ? Qt::Checked : Qt::Unchecked);

	return item;
}

//Shortcut to create a persistent editor item
QStandardItem* PERSISTENT_EDITOR(ccPropertiesTreeDelegate::CC_PROPERTY_ROLE role)
{
	return ITEM(QString(),Qt::ItemIsEditable,role);
}

void ccPropertiesTreeDelegate::fillWithHObject(ccHObject* _obj)
{
    assert(_obj && m_model);

    addSeparator("CC Object");

    //name
	appendRow( ITEM("Name"), ITEM(_obj->getName(),Qt::ItemIsEditable,OBJECT_NAME) );

    //unique ID
	appendRow( ITEM("Unique ID"), ITEM(QString::number(_obj->getUniqueID())) );

    //number of children
	appendRow( ITEM("Children"), ITEM(QString::number(_obj->getChildrenNumber())) );

    //visiblity
    if (!_obj->isVisiblityLocked())
		appendRow( ITEM("Visible"), CHECKABLE_ITEM(_obj->isVisible(),OBJECT_VISIBILITY) );

    //colors
    if (_obj->hasColors())
		appendRow( ITEM("Colors"), CHECKABLE_ITEM(_obj->colorsShown(),OBJECT_COLORS_SHOWN) );

    //normals
    if (_obj->hasNormals())
		appendRow( ITEM("Normals"), CHECKABLE_ITEM(_obj->normalsShown(),OBJECT_NORMALS_SHOWN) );

    //scalar fields
    if (_obj->hasScalarFields())
		appendRow( ITEM("Scalar Field"), CHECKABLE_ITEM(_obj->sfShown(),OBJECT_SCALAR_FIELD_SHOWN) );

    //name in 3D
	appendRow( ITEM("Show name (in 3D)"), CHECKABLE_ITEM(_obj->nameShownIn3D(),OBJECT_NAME_IN_3D) );

	//display window
    if (!_obj->isLocked())
		appendRow( ITEM("Current Display"), PERSISTENT_EDITOR(OBJECT_CURRENT_DISPLAY), true );

    //Bounding-box
	{
		ccBBox box;
		bool fitBBox = false;
		if (_obj->getSelectionBehavior() == ccHObject::SELECTION_FIT_BBOX)
		{
			ccGLMatrix trans;
			box = _obj->getFitBB(trans);
			box += CCVector3(trans.getTranslation());
			fitBBox = true;
		}
		else
		{
			box = _obj->getBB();
		}

		if (box.isValid())
		{
			//Box dimensions
			CCVector3 bboxDiag = box.getDiagVec();
			appendRow(	ITEM(fitBBox ? "Local box dimensions" : "Box dimensions"),
						ITEM(QString("X: %0\nY: %1\nZ: %2").arg(bboxDiag.x).arg(bboxDiag.y).arg(bboxDiag.z)) );

			//Box center
			CCVector3 bboxCenter = box.getCenter();
			appendRow(	ITEM("Box center"),
						ITEM(QString("X: %0\nY: %1\nZ: %2").arg(bboxCenter.x).arg(bboxCenter.y).arg(bboxCenter.z)) );
		}
	}
}

void ccPropertiesTreeDelegate::fillWithPointCloud(ccGenericPointCloud* _obj)
{
    assert(_obj && m_model);

    addSeparator("Cloud");

    //number of points
	appendRow( ITEM("Points"), ITEM(QLocale(QLocale::English).toString(_obj->size())) );

    //shift
	{
		const double* shift = _obj->getOriginalShift();
		appendRow( ITEM("Global shift"), ITEM(QString("(%1;%2;%3)").arg(shift[0],0,'f',2).arg(shift[1],0,'f',2).arg(shift[2],0,'f',2)) );
	}

	//custom point size
	appendRow( ITEM("Point size"), PERSISTENT_EDITOR(OBJECT_CLOUD_POINT_SIZE), true );

	fillSFWithPointCloud(_obj);
}

void ccPropertiesTreeDelegate::fillSFWithPointCloud(ccGenericPointCloud* _obj)
{
	assert(m_model);

    //for "real" point clouds only
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(_obj);
    if (!cloud)
        return;

    //Scalar fields
    unsigned sfCount = cloud->getNumberOfScalarFields();
    if (sfCount != 0)
    {
        addSeparator(sfCount > 1 ? "Scalar Fields" : "Scalar Field");

        //fields number
		appendRow( ITEM("Count"), ITEM(QString::number(sfCount)) );

        //fields list combo
		appendRow( ITEM("Active"), PERSISTENT_EDITOR(OBJECT_CURRENT_SCALAR_FIELD), true );

		//no need to go any further if no SF is currently active
		CCLib::ScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (sf)
		{
			addSeparator("Color Scale");

			//color scale selection combo box
			appendRow( ITEM("Current"), PERSISTENT_EDITOR(OBJECT_CURRENT_COLOR_RAMP), true );

			//color scale steps
			appendRow( ITEM("Steps"), PERSISTENT_EDITOR(OBJECT_COLOR_RAMP_STEPS), true );

			//scale visible?
			appendRow( ITEM("Visible"), CHECKABLE_ITEM(cloud->sfColorScaleShown(),OBJECT_SF_SHOW_SCALE) );

			addSeparator("SF display params");

			//SF edit dialog (warning: 2 columns)
			m_model->appendRow(PERSISTENT_EDITOR(OBJECT_CLOUD_SF_EDITOR));
			m_view->openPersistentEditor(m_model->index(m_model->rowCount()-1,0));
		}
    }
}

void ccPropertiesTreeDelegate::fillWithPrimitive(ccGenericPrimitive* _obj)
{
    assert(_obj && m_model);

    addSeparator("Primitive");

    //type
	appendRow( ITEM("Type"), ITEM(_obj->getTypeName()) );

	//drawing steps
	if (_obj->hasDrawingPrecision())
		appendRow( ITEM("Drawing precision"), PERSISTENT_EDITOR(OBJECT_PRIMITIVE_PRECISION), true );
}

void ccPropertiesTreeDelegate::fillWithMesh(ccGenericMesh* _obj)
{
    assert(_obj && m_model);

    addSeparator("Mesh");

    //number of facets
	appendRow( ITEM("Faces"), ITEM(QLocale(QLocale::English).toString(_obj->size())) );

	//material/texture
	if (_obj->hasMaterials())
		appendRow( ITEM("Materials/textures"), CHECKABLE_ITEM(_obj->materialsShown(),OBJECT_MATERIALS) );

    //wireframe
	appendRow( ITEM("Wireframe"), CHECKABLE_ITEM(_obj->isShownAsWire(),OBJECT_MESH_WIRE) );

    //stippling (ccMesh only)
	if (_obj->isA(CC_MESH))
		appendRow( ITEM("Stippling"), CHECKABLE_ITEM(static_cast<ccMesh*>(_obj)->stipplingEnabled(),OBJECT_MESH_STIPPLING) );

	//we also integrate vertices SF into mesh properties
    ccGenericPointCloud* vertices = _obj->getAssociatedCloud();
    if (vertices && (_obj->isA(CC_MESH_GROUP) || !vertices->isLocked() || _obj->isAncestorOf(vertices)))
        fillSFWithPointCloud(vertices);
}

void ccPropertiesTreeDelegate::fillWithPointOctree(ccOctree* _obj)
{
    assert(_obj && m_model);

    addSeparator("Octree");

    //display mode
	appendRow( ITEM("Display mode"), PERSISTENT_EDITOR(OBJECT_OCTREE_TYPE), true );

    //level
	appendRow( ITEM("Display level"), PERSISTENT_EDITOR(OBJECT_OCTREE_LEVEL), true );

    addSeparator("Current level");

	//current display level
	int level = _obj->getDisplayedLevel();
	assert(level>0 && level<=ccOctree::MAX_OCTREE_LEVEL);

	//cell size
	PointCoordinateType cellSize = _obj->getCellSize(level);
	appendRow( ITEM("Cell size"), ITEM(QString::number(cellSize)) );

	//cell count
	unsigned cellCount = _obj->getCellNumber(level);
	appendRow( ITEM("Cell count"), ITEM(QLocale(QLocale::English).toString(cellCount)) );

	//total volume of filled cells
	appendRow( ITEM("Filled volume"), ITEM(QString::number((double)cellCount*pow((double)cellSize,3.0))) );
}

void ccPropertiesTreeDelegate::fillWithPointKdTree(ccKdTree* _obj)
{
    assert(_obj && m_model);

    addSeparator("Kd-tree");

    //max rms
	appendRow( ITEM("Max RMS"), ITEM(QString::number(_obj->getMaxRMS())) );
}

void ccPropertiesTreeDelegate::fillWithImage(ccImage* _obj)
{
    assert(_obj && m_model);

    addSeparator("Image");

    //image width
	appendRow( ITEM("Width"), ITEM(QString::number(_obj->getW())) );

    //image height
	appendRow( ITEM("Height"), ITEM(QString::number(_obj->getH())) );

    //transparency
	appendRow( ITEM("Alpha"), PERSISTENT_EDITOR(OBJECT_IMAGE_ALPHA), true );
}

void ccPropertiesTreeDelegate::fillWithCalibratedImage(ccCalibratedImage* _obj)
{
    assert(_obj && m_model);

    addSeparator("Calibrated Image");

    //"Set Viewport" button
	appendRow( ITEM("Apply Viewport"), PERSISTENT_EDITOR(OBJECT_APPLY_IMAGE_VIEWPORT), true );

    //field of view
	appendRow( ITEM("f.o.v. (deg)"), ITEM(QString::number(_obj->getFov())) );

	//get camera matrix parameters
	PointCoordinateType angle_rad;
	CCVector3 axis3D, t3D;
	_obj->getCameraMatrix().getParameters(angle_rad, axis3D, t3D);

    //camera position
	appendRow( ITEM("Optical center"), ITEM(QString("(%0,%1,%2)").arg(t3D.x).arg(t3D.y).arg(t3D.z)) );

    //camera orientation axis
	appendRow( ITEM("Orientation"), ITEM(QString("(%0,%1,%2)").arg(axis3D.x).arg(axis3D.y).arg(axis3D.z)) );

    //camera orientation angle
	appendRow( ITEM("Angle (degrees)"), ITEM(QString("%0°").arg(angle_rad*CC_RAD_TO_DEG)) );
}

void ccPropertiesTreeDelegate::fillWithLabel(cc2DLabel* _obj)
{
    assert(_obj && m_model);

    addSeparator("Label");

    //Body
	QStringList body = _obj->getLabelContent(ccGui::Parameters().displayedNumPrecision);
	appendRow( ITEM("Body"), ITEM(body.join("\n")) );

	//Show label in 2D
	appendRow( ITEM("Show 2D label"), CHECKABLE_ITEM(_obj->isDisplayedIn2D(),OBJECT_LABEL_DISP_2D) );

	//Show label in 3D
	appendRow( ITEM("Show 3D legend(s)"), CHECKABLE_ITEM(_obj->isDisplayedIn3D(),OBJECT_LABEL_DISP_3D) );
}

void ccPropertiesTreeDelegate::fillWithViewportObject(cc2DViewportObject* _obj)
{
    assert(_obj && m_model);

    addSeparator("Viewport");

    //Name
	appendRow( ITEM("Name"), ITEM(_obj->getName().isEmpty() ? "undefined" : _obj->getName()) );

    //"Apply Viewport" button
	appendRow( ITEM("Apply Viewport"), PERSISTENT_EDITOR(OBJECT_APPLY_LABEL_VIEWPORT), true );
}

void ccPropertiesTreeDelegate::fillWithGBLSensor(ccGBLSensor* _obj)
{
    assert(_obj && m_model);

    addSeparator("GBL Sensor");

    //Angular steps (phi)
	appendRow( ITEM("dPhi"), ITEM(QString::number(_obj->getDeltaPhi())) );

    //Angular steps (theta)
	appendRow( ITEM("dTheta"), ITEM(QString::number(_obj->getDeltaTheta())) );

    //Uncertainty
	appendRow( ITEM("Uncertainty"), ITEM(QString::number(_obj->getUncertainty())) );

	//Sensor drawing scale
	appendRow( ITEM("Drawing scale"), PERSISTENT_EDITOR(OBJECT_SENSOR_DISPLAY_SCALE), true );
}

void ccPropertiesTreeDelegate::fillWithMaterialSet(ccMaterialSet* _obj)
{
    assert(_obj && m_model);

    addSeparator("Material set");

    //Count
	appendRow( ITEM("Count"), ITEM(QString::number(_obj->size())) );

	//ccMaterialSet objects are 'shareable'
	fillWithShareable(_obj);
}

void ccPropertiesTreeDelegate::fillWithShareable(CCShareable* _obj)
{
    assert(_obj && m_model);

    addSeparator("Array");

	//Link count
	unsigned linkCount = _obj->getLinkCount(); //if we display it, it means it is a member of the DB --> ie. link is already >1
	appendRow( ITEM("Shared"), ITEM(linkCount < 3 ? QString("No") : QString("Yes (%1)").arg(linkCount-1)) );
}

template<int N, class ElementType> void ccPropertiesTreeDelegate::fillWithChunkedArray(ccChunkedArray<N,ElementType>* _obj)
{
    assert(_obj && m_model);

    addSeparator("Array");

    //Name
	appendRow( ITEM("Name"), ITEM(_obj->getName().isEmpty() ? "undefined" : _obj->getName() ) );

    //Count
	appendRow( ITEM("Elements"), ITEM(QLocale(QLocale::English).toString(_obj->currentSize()) ) );

	//Capacity
	appendRow( ITEM("Capacity"), ITEM(QLocale(QLocale::English).toString(_obj->capacity()) ) );

	//Memory
	appendRow( ITEM("Memory"), ITEM(QString("%1 Mb").arg((double)_obj->memory()/1048576.0,0,'f',2)) );

	//ccChunkedArray objects are 'shareable'
	fillWithShareable(_obj);
}

QWidget* ccPropertiesTreeDelegate::createEditor(QWidget *parent,
        const QStyleOptionViewItem &option,
        const QModelIndex &index) const
{
    if (!m_model || !m_currentObject)
        return NULL;

    QStandardItem* item = m_model->itemFromIndex(index);

    if (!item || !item->data().isValid() || (item->column()==0 && item->data().toInt()!=OBJECT_CLOUD_SF_EDITOR))
        return NULL;

    switch (item->data().toInt())
    {
    case OBJECT_CURRENT_DISPLAY:
    {
        QComboBox *comboBox = new QComboBox(parent);

        std::vector<ccGLWindow*> glWindows;
        MainWindow::GetGLWindows(glWindows);

        comboBox->addItem(c_noDisplayString);

        for (unsigned i=0;i<glWindows.size();++i)
            comboBox->addItem(glWindows[i]->windowTitle());

        connect(comboBox, SIGNAL(currentIndexChanged(const QString)), this, SLOT(objectDisplayChanged(const QString)));

		comboBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return comboBox;
    }
    case OBJECT_CURRENT_SCALAR_FIELD:
    {
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        QComboBox *comboBox = new QComboBox(parent);

        comboBox->addItem(QString("none"));
        int i,nsf = cloud->getNumberOfScalarFields();
        for (i=0;i<nsf;++i)
            comboBox->addItem(QString(cloud->getScalarFieldName(i)));

        connect(comboBox, SIGNAL(activated(int)), this, SLOT(scalarFieldChanged(int)));

		comboBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return comboBox;
    }
    case OBJECT_CURRENT_COLOR_RAMP:
    {
		QColorScaleSelector* selector = new QColorScaleSelector(parent);

		//fill combox box
		if (selector->m_comboBox)
		{
			selector->m_comboBox->blockSignals(true);
			selector->m_comboBox->clear();
			//add all available color scales
			ccColorScalesManager* csManager = ccColorScalesManager::GetUniqueInstance();
			assert(csManager);
			for (ccColorScalesManager::ScalesMap::const_iterator it = csManager->map().begin(); it != csManager->map().end(); ++it)
				selector->m_comboBox->addItem((*it)->getName(),(*it)->getUuid());
			selector->m_comboBox->blockSignals(false);

			connect(selector->m_comboBox, SIGNAL(activated(int)), this, SLOT(colorScaleChanged(int)));
		}
		//advanced tool button
		if (selector->m_button)
		{
			connect(selector->m_button, SIGNAL(clicked()), this, SLOT(spawnColorRampEditor()));
		}

		selector->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return selector;
    }
    case OBJECT_COLOR_RAMP_STEPS:
    {
        QSpinBox *spinBox = new QSpinBox(parent);
		spinBox->setRange(ccColorScale::MIN_STEPS,ccColorScale::MAX_STEPS);
        spinBox->setSingleStep(4);

        connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(colorRampStepsChanged(int)));

		spinBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return spinBox;
    }
    case OBJECT_CLOUD_SF_EDITOR:
    {
        sfEditDlg* sfd = new sfEditDlg(parent);

		//DGM: why does this widget can't follow its 'policy' ?!
		//QSizePolicy pol = sfd->sizePolicy();
		//QSizePolicy::Policy hpol = pol.horizontalPolicy();
		//sfd->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Maximum);
		//parent->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Maximum);

        connect(sfd, SIGNAL(entitySFHasChanged()), this, SLOT(updateDisplay()));

		sfd->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return sfd;
    }
    case OBJECT_OCTREE_TYPE:
    {
        QComboBox *comboBox = new QComboBox(parent);

        for (int i=0;i<OCTREE_DISPLAY_TYPE_NUMBERS;++i)
            comboBox->addItem(COCTREE_DISPLAY_TYPE_TITLES[i]);

        connect(comboBox, SIGNAL(activated(int)), this, SLOT(octreeDisplayTypeChanged(int)));

		comboBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return comboBox;
    }
    case OBJECT_OCTREE_LEVEL:
    {
        QSpinBox *spinBox = new QSpinBox(parent);
        spinBox->setRange(1,CCLib::DgmOctree::MAX_OCTREE_LEVEL);

        connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(octreeDisplayedLevelChanged(int)));

		spinBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return spinBox;
    }
	case OBJECT_PRIMITIVE_PRECISION:
    {
        QSpinBox *spinBox = new QSpinBox(parent);
        spinBox->setRange(4,360);
		spinBox->setSingleStep(4);

        connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(primitivePrecisionChanged(int)));

		spinBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return spinBox;
    }
    case OBJECT_IMAGE_ALPHA:
    {
        QSlider* slider = new QSlider(Qt::Horizontal,parent);
        slider->setRange(0,255);
        slider->setSingleStep(1);
        slider->setPageStep(16);
        slider->setTickPosition(QSlider::NoTicks);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(imageAlphaChanged(int)));

		slider->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return slider;
    }
    case OBJECT_APPLY_IMAGE_VIEWPORT:
    {
        QPushButton* button = new QPushButton("Apply",parent);
        connect(button, SIGNAL(clicked()), this, SLOT(applyImageViewport()));

		button->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
		button->setMinimumHeight(30);
        return button;
    }
    case OBJECT_APPLY_LABEL_VIEWPORT:
    {
        QPushButton* button = new QPushButton("Apply",parent);
        connect(button, SIGNAL(clicked()), this, SLOT(applyLabelViewport()));

		button->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
		button->setMinimumHeight(30);
        return button;
    }
    case OBJECT_SENSOR_DISPLAY_SCALE:
    {
        QDoubleSpinBox *spinBox = new QDoubleSpinBox(parent);
        spinBox->setRange(1e-6,1e6);
        spinBox->setSingleStep(1e-3);

        connect(spinBox, SIGNAL(valueChanged(double)), this, SLOT(sensorScaleChanged(double)));

		spinBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return spinBox;
    }
    case OBJECT_CLOUD_POINT_SIZE:
    {
        QComboBox *comboBox = new QComboBox(parent);

        comboBox->addItem(c_defaultPointSizeString); //size = 0
        for (unsigned i=1;i<=10;++i)
            comboBox->addItem(QString::number(i));

        connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(cloudPointSizeChanged(int)));

		comboBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return comboBox;
    }
    default:
        return QStyledItemDelegate::createEditor(parent, option, index);
    }

    return NULL;
}

void ccPropertiesTreeDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	QStyledItemDelegate::updateEditorGeometry(editor, option, index);

    if (!m_model || !editor)
        return;

	QStandardItem* item = m_model->itemFromIndex(index);

	if (item && item->data().isValid() && item->data().toInt()==OBJECT_CLOUD_SF_EDITOR && item->column()==0)
	{
		sfEditDlg *sfd = qobject_cast<sfEditDlg*>(editor);
		if (!sfd)
			return;
		//we must resize the SF edit widget so that it spans on both columns!
		QRect rect = m_view->visualRect(m_model->index(item->row(),1)); //second column width
		sfd->resize(option.rect.width()+rect.width(),sfd->height());
	}
}

void ccPropertiesTreeDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    if (!m_model || !m_currentObject)
        return;

    QStandardItem* item = m_model->itemFromIndex(index);

    if (!item || !item->data().isValid() || (item->column()==0 && item->data().toInt()!=OBJECT_CLOUD_SF_EDITOR))
        return;

    switch (item->data().toInt())
    {
    case OBJECT_CURRENT_DISPLAY:
    {
        QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
        if (!comboBox)
            return;

        ccGLWindow* win = static_cast<ccGLWindow*>(m_currentObject->getDisplay());
        int pos = (win ? comboBox->findText(win->windowTitle()) : 0);

        comboBox->setCurrentIndex(std::max(pos,0)); //0 = "NONE"
        break;
    }
    case OBJECT_CURRENT_SCALAR_FIELD:
    {
        QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
        if (!comboBox)
            return;

        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        int pos = cloud->getCurrentDisplayedScalarFieldIndex();
        comboBox->setCurrentIndex(pos+1);
        break;
    }
    case OBJECT_CURRENT_COLOR_RAMP:
    {
        QFrame *selectorFrame = qobject_cast<QFrame*>(editor);
        if (!selectorFrame)
            return;
		QColorScaleSelector* selector = static_cast<QColorScaleSelector*>(selectorFrame);

        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (sf)
		{
			int pos = -1;
			//search right index by UUID
			if (sf->getColorScale())
				pos = selector->m_comboBox->findData(sf->getColorScale()->getUuid());
			selector->m_comboBox->setCurrentIndex(pos);
		}
        break;
    }
    case OBJECT_COLOR_RAMP_STEPS:
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(editor);
        if (!spinBox)
            return;

        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (sf)
			spinBox->setValue(sf->getColorRampSteps());
        break;
    }
    case OBJECT_CLOUD_SF_EDITOR:
    {
        sfEditDlg *sfd = qobject_cast<sfEditDlg*>(editor);
        if (!sfd)
            return;

        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (sf)
			sfd->fillDialogWith(sf);
        break;
    }
    case OBJECT_OCTREE_TYPE:
    {
        QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
        if (!comboBox)
            return;

        ccOctree* octree = ccHObjectCaster::ToOctree(m_currentObject);
        assert(octree);
        comboBox->setCurrentIndex(int(octree->getDisplayType()));
        break;
    }
    case OBJECT_OCTREE_LEVEL:
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(editor);
        if (!spinBox)
            return;

        ccOctree* octree = ccHObjectCaster::ToOctree(m_currentObject);
        assert(octree);
        spinBox->setValue(octree->getDisplayedLevel());
        break;
    }
    case OBJECT_PRIMITIVE_PRECISION:
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(editor);
        if (!spinBox)
            return;

		ccGenericPrimitive* primitive = ccHObjectCaster::ToPrimitive(m_currentObject);
		assert(primitive);
        spinBox->setValue(primitive->getDrawingPrecision());
        break;
    }
    case OBJECT_IMAGE_ALPHA:
    {
        QSlider *slider = qobject_cast<QSlider*>(editor);
        if (!slider)
            return;

        ccImage* image = ccHObjectCaster::ToImage(m_currentObject);
        assert(image);
        slider->setValue(int(image->getAlpha()*255.0));
        //slider->setTickPosition(QSlider::NoTicks);
        break;
    }
    case OBJECT_SENSOR_DISPLAY_SCALE:
    {
        QDoubleSpinBox *spinBox = qobject_cast<QDoubleSpinBox*>(editor);
        if (!spinBox)
            return;

        ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(m_currentObject);
        assert(sensor);
        spinBox->setValue(sensor->getGraphicScale());
        break;
    }
    case OBJECT_CLOUD_POINT_SIZE:
    {
        QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
        if (!comboBox)
            return;

		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_currentObject);
        assert(cloud);
        comboBox->setCurrentIndex(cloud->getPointSize());
        break;
    }
    default:
        QStyledItemDelegate::setEditorData(editor, index);
        break;
    }
}

void ccPropertiesTreeDelegate::updateItem(QStandardItem * item)
{
    if (!m_currentObject || item->column()==0 || !item->data().isValid())
        return;

    bool redraw=false;
	switch (item->data().toInt())
	{
	case OBJECT_NAME:
		m_currentObject->setName(item->text());
		emit ccObjectPropertiesChanged(m_currentObject);
		break;
	case OBJECT_VISIBILITY:
		{
			bool objectWasDisplayed = m_currentObject->isDisplayed();
			m_currentObject->setVisible(item->checkState() == Qt::Checked);
			bool objectIsDisplayed = m_currentObject->isDisplayed();
			if (objectWasDisplayed != objectIsDisplayed)
			{
				if (m_currentObject->isGroup())
					emit ccObjectAndChildrenAppearanceChanged(m_currentObject);
				else
					emit ccObjectAppearanceChanged(m_currentObject);
			}
		}
		break;
	case OBJECT_COLORS_SHOWN:
		m_currentObject->showColors(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	case OBJECT_NORMALS_SHOWN:
		m_currentObject->showNormals(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	case OBJECT_MATERIALS:
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(m_currentObject);
			assert(mesh);
			mesh->showMaterials(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_SCALAR_FIELD_SHOWN:
		m_currentObject->showSF(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	case OBJECT_SF_SHOW_SCALE:
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
			assert(cloud);
			cloud->showSFColorsScale(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_MESH_WIRE:
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(m_currentObject);
			assert(mesh);
			mesh->showWired(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_MESH_STIPPLING:
		{
			ccMesh* mesh = ccHObjectCaster::ToMesh(m_currentObject);
			assert(mesh);
			mesh->enableStippling(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_LABEL_DISP_2D:
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(m_currentObject);
			assert(label);
			label->setDisplayedIn2D(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_LABEL_DISP_3D:
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(m_currentObject);
			assert(label);
			label->setDisplayedIn3D(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_NAME_IN_3D:
		m_currentObject->showNameIn3D(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	}

    if (redraw)
		updateDisplay();
}

void ccPropertiesTreeDelegate::updateDisplay()
{
	ccHObject* object = m_currentObject;
	if (!object)
		return;

	bool objectIsDisplayed = object->isDisplayed();
	if (!objectIsDisplayed)
	{
		//DGM: point clouds may be mesh vertices of meshes which may depend on several of their parameters
		if (object->isKindOf(CC_POINT_CLOUD))
		{
			ccHObject* parent = object->getParent();
			if (parent && parent->isKindOf(CC_MESH) && parent->isDisplayed()) //specific case: vertices
			{
				object = parent;
				objectIsDisplayed = true;
			}
		}
	}

	if (objectIsDisplayed)
	{
		if (object->isGroup())
			emit ccObjectAndChildrenAppearanceChanged(m_currentObject);
		else
			emit ccObjectAppearanceChanged(m_currentObject);
	}
}

void ccPropertiesTreeDelegate::updateModel()
{
	//simply re-fill model!
	fillModel(m_currentObject);
}

void ccPropertiesTreeDelegate::scalarFieldChanged(int pos)
{
    if (!m_currentObject)
        return;

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    if (cloud && cloud->getCurrentDisplayedScalarFieldIndex()+1 != pos)
    {
        cloud->setCurrentDisplayedScalarField(pos-1);
		cloud->showSF(pos>0);

		updateDisplay();
        //we must also reset the properties display!
        updateModel();
    }
}

void ccPropertiesTreeDelegate::spawnColorRampEditor()
{
    if (!m_currentObject)
        return;

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    assert(cloud);
	ccScalarField* sf = (cloud ? static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField()) : 0);
	if (sf)
	{
		ccColorScaleEditorDialog* editorDialog = new ccColorScaleEditorDialog(sf->getColorScale(),static_cast<ccGLWindow*>(cloud->getDisplay()));
		editorDialog->setAssociatedScalarField(sf);
		if (editorDialog->exec())
		{
			if (editorDialog->getActiveScale())
			{
				sf->setColorScale(editorDialog->getActiveScale());
				updateDisplay();
			}

			//save current scale manager state to persistent settings
			ccColorScalesManager::GetUniqueInstance()->toPersistentSettings();

			updateModel();
		}
	}
}

void ccPropertiesTreeDelegate::colorScaleChanged(int pos)
{
    if (!m_currentObject)
        return;

	if (pos < 0)
	{
		assert(false);
		return;
	}

	QComboBox* comboBox = dynamic_cast<QComboBox*>(QObject::sender());
	if (!comboBox)
		return;

	QString UUID = comboBox->itemData(pos).toString();
	ccColorScale::Shared colorScale = ccColorScalesManager::GetUniqueInstance()->getScale(UUID);

	if (!colorScale)
	{
		ccLog::Error("Internal error: color scale doesn't seem to exist anymore!");
		return;
	}

	//get current SF
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    assert(cloud);
	ccScalarField* sf = cloud ? static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField()) : 0;
	if (sf && sf->getColorScale() != colorScale)
	{
		sf->setColorScale(colorScale);
		updateDisplay();
		updateModel();
	}
}

void ccPropertiesTreeDelegate::colorRampStepsChanged(int pos)
{
    if (!m_currentObject)
        return;

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    assert(cloud);
	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
	if (sf)
	{
		sf->setColorRampSteps(pos);
		updateDisplay();
	}
}

void ccPropertiesTreeDelegate::octreeDisplayTypeChanged(int pos)
{
    if (!m_currentObject)
        return;

    ccOctree* octree = ccHObjectCaster::ToOctree(m_currentObject);
    assert(octree);

    octree->setDisplayType(OCTREE_DISPLAY_TYPE_ENUMS[pos]);
	updateDisplay();
}

void ccPropertiesTreeDelegate::octreeDisplayedLevelChanged(int val)
{
    if (!m_currentObject)
        return;

    ccOctree* octree = ccHObjectCaster::ToOctree(m_currentObject);
    assert(octree);

	if (octree->getDisplayedLevel() != val) //to avoid infinite loops!
	{
		octree->setDisplayedLevel(val);
		updateDisplay();
		//we must also reset the properties display!
		updateModel();
	}
}

void ccPropertiesTreeDelegate::primitivePrecisionChanged(int val)
{
    if (!m_currentObject)
        return;

    ccGenericPrimitive* primitive = ccHObjectCaster::ToPrimitive(m_currentObject);
    assert(primitive);

	if (val == primitive->getDrawingPrecision())
		return;

	bool wasVisible = primitive->isVisible();
    primitive->setDrawingPrecision(val);
	primitive->setVisible(wasVisible);

	updateDisplay();
	//we must also reset the properties display!
	updateModel();
}


void ccPropertiesTreeDelegate::imageAlphaChanged(int val)
{
    ccImage* image = ccHObjectCaster::ToImage(m_currentObject);
    if (!image)
        return;
    image->setAlpha(float(val)/255.0);

	updateDisplay();
}

void ccPropertiesTreeDelegate::applyImageViewport()
{
    if (!m_currentObject)
        return;

    ccCalibratedImage* image = ccHObjectCaster::ToCalibratedImage(m_currentObject);
    assert(image);

    ccGLWindow* win = static_cast<ccGLWindow*>(image->getDisplay());
    if (!win)
        return;

    win->applyImageViewport(image);
}

void ccPropertiesTreeDelegate::applyLabelViewport()
{
    if (!m_currentObject)
        return;

    cc2DViewportObject* viewport = ccHObjectCaster::To2DViewportObject(m_currentObject);
    assert(viewport);

	ccGLWindow* win = MainWindow::GetActiveGLWindow();
    if (!win)
        return;

	win->setViewportParameters(viewport->getParameters());
	win->redraw();
}

void ccPropertiesTreeDelegate::sensorScaleChanged(double val)
{
    if (!m_currentObject)
        return;

    ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(m_currentObject);
    assert(sensor);

	sensor->setGraphicScale(val);
	updateDisplay();
}

void ccPropertiesTreeDelegate::cloudPointSizeChanged(int size)
{
    if (!m_currentObject)
        return;

	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_currentObject);
    assert(cloud);

	cloud->setPointSize(size);
	updateDisplay();
}

void ccPropertiesTreeDelegate::objectDisplayChanged(const QString& newDisplayTitle)
{
    if (!m_currentObject)
        return;

    QString actualDisplayTitle;

    ccGLWindow* win = static_cast<ccGLWindow*>(m_currentObject->getDisplay());
    if (win)
        actualDisplayTitle = win->windowTitle();
    else
        actualDisplayTitle = c_noDisplayString;

    if (actualDisplayTitle != newDisplayTitle)
    {
        //we first mark the "old displays" before removal,
        //to be sure that they wiil also be redrawn!
        m_currentObject->prepareDisplayForRefresh_recursive();

        ccGLWindow* win = MainWindow::GetGLWindow(newDisplayTitle);
		m_currentObject->setDisplay_recursive(win);
        if (win)
        {
            m_currentObject->prepareDisplayForRefresh_recursive();
            win->zoomGlobal();
        }

        MainWindow::RefreshAllGLWindow();
    }
}
