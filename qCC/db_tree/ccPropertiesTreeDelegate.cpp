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
//$Rev:: 2265                                                              $
//$LastChangedDate:: 2012-10-13 22:22:51 +0200 (sam., 13 oct. 2012)        $
//**************************************************************************
//

#include "ccPropertiesTreeDelegate.h"

//Local
#include "sfEditDlg.h"
#include "../ccConsole.h"
#include "../ccGLWindow.h"
#include "../mainwindow.h"
#include "../ccGuiParameters.h"
#include <QPushButton>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccOctree.h>
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

//System
#include <assert.h>

// Default 'None' string
const QString c_noDisplayString = QString("None");

ccPropertiesTreeDelegate::ccPropertiesTreeDelegate(QStandardItemModel* model,
												   QAbstractItemView* view,
												   QObject *parent)
		: QItemDelegate(parent)
		, m_model(model)
		, m_view(view)
		, m_currentObject(0)
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
        case OBJECT_DISPLAY:
        case OBJECT_CURRENT_SCALAR_FIELD:
        case OBJECT_CURRENT_COLOR_RAMP:
        case OBJECT_OCTREE_TYPE:
        case OBJECT_COLOR_RAMP_STEPS:
            return QSize(50,18);
        }
    }

    return QItemDelegate::sizeHint(option,index);
}
//*/

void ccPropertiesTreeDelegate::unbind()
{
    if (m_model)
        disconnect(m_model, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateItem(QStandardItem*)));
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

	if (m_model)
	{
		m_model->clear();
		m_model->setColumnCount(2);
		m_model->setHeaderData(0, Qt::Horizontal, "Property");
		m_model->setHeaderData(1, Qt::Horizontal, "State/Value");
	}

    if (m_currentObject->isHierarchy())
		if (!m_currentObject->isA(CC_2D_VIEWPORT_LABEL)) //don't need to display this kind of info for viewport labels!
        fillWithHObject(m_currentObject);

    if (m_currentObject->isKindOf(CC_POINT_CLOUD))
    {
        fillWithPointCloud(static_cast<ccGenericPointCloud*>(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_MESH))
    {
        fillWithMesh(static_cast<ccGenericMesh*>(m_currentObject));
    
		if (m_currentObject->isKindOf(CC_PRIMITIVE))
			fillWithPrimitive(static_cast<ccGenericPrimitive*>(m_currentObject));
    }
    else if (m_currentObject->isA(CC_POINT_OCTREE))
    {
        fillWithPointOctree(static_cast<ccOctree*>(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_IMAGE))
    {
        fillWithImage(static_cast<ccImage*>(m_currentObject));

        if (m_currentObject->isA(CC_CALIBRATED_IMAGE))
            fillWithCalibratedImage(static_cast<ccCalibratedImage*>(m_currentObject));
    }
    else if (m_currentObject->isA(CC_2D_LABEL))
    {
        fillWithLabel(static_cast<cc2DLabel*>(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_2D_VIEWPORT_OBJECT))
    {
        fillWithViewportObject(static_cast<cc2DViewportObject*>(m_currentObject));
    }
    else if (m_currentObject->isKindOf(CC_GBL_SENSOR))
    {
        fillWithGBLSensor(static_cast<ccGBLSensor*>(m_currentObject));
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
	
	if (m_model)
		connect(m_model, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateItem(QStandardItem*)));
}

void ccPropertiesTreeDelegate::addSeparator(const char* title)
{
    assert(m_model);

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    QBrush backgBrush(Qt::darkGray);
    QBrush textBrush(Qt::white);

    //name
    m_model->setRowCount(curRow+1);
    item = new QStandardItem(title);
    item->setFlags(Qt::ItemIsEnabled);
    item->setForeground(textBrush);
    item->setBackground(backgBrush);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled);
    item->setForeground(textBrush);
    item->setBackground(backgBrush);
    m_model->setItem(curRow,1,item);
}

void ccPropertiesTreeDelegate::fillWithHObject(ccHObject* _obj)
{
    assert(_obj && m_model);

    addSeparator("CC Object");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //name
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Name");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(_obj->getName());
    item->setData(OBJECT_NAME);
    item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //unique ID
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Unique ID");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	item = new QStandardItem(QString::number(_obj->getUniqueID()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //number of children
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Children");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getChildrenNumber()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //visiblity
    if (!_obj->isVisiblityLocked())
    {
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Visible");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(_obj->isVisible() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_VISIBILITY);
        m_model->setItem(curRow,1,item);
    }

    //colors
    if (_obj->hasColors())
    {
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Colors");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(_obj->colorsShown() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_COLORS);
        m_model->setItem(curRow,1,item);
    }

    //normals
    if (_obj->hasNormals())
    {
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Normals");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(_obj->normalsShown() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_NORMALS);
        m_model->setItem(curRow,1,item);
    }

    //scalar fields
    if (_obj->hasScalarFields())
    {
        //Colors
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Scalar Field");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(_obj->sfShown() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_SCALAR_FIELD);
        m_model->setItem(curRow,1,item);
    }

    //display window
    if (!_obj->isLocked())
    {
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Current Display");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem();
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
        item->setData(OBJECT_DISPLAY);
        m_model->setItem(curRow,1,item);
        m_view->openPersistentEditor(m_model->index(curRow,1));
    }

    //Bounding-box
    ccBBox box = _obj->getBB();
	if (box.isValid())
	{
    //Box dimensions
		m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Box dimensions");
	item->setFlags(Qt::ItemIsEnabled);
		m_model->setItem(curRow,0,item);

	CCVector3 bboxDiag = box.getDiagVec();
    item = new QStandardItem(QString("X: %0\nY: %1\nZ: %2").arg(bboxDiag.x).arg(bboxDiag.y).arg(bboxDiag.z));
	item->setFlags(Qt::ItemIsEnabled);
		m_model->setItem(curRow,1,item);

    //Box center
		m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Box center");
	item->setFlags(Qt::ItemIsEnabled);
		m_model->setItem(curRow,0,item);

    CCVector3 bboxCenter = box.getCenter();
    item = new QStandardItem(QString("X: %0\nY: %1\nZ: %2").arg(bboxCenter.x).arg(bboxCenter.y).arg(bboxCenter.z));
    item->setFlags(Qt::ItemIsEnabled);
		m_model->setItem(curRow,1,item);
	}

}

void ccPropertiesTreeDelegate::fillWithPointCloud(ccGenericPointCloud* _obj)
{
    assert(_obj && m_model);

    addSeparator("Cloud");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //number of points
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Points");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QLocale(QLocale::English).toString(_obj->size()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //shift
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Global shift");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    const double* shift = _obj->getOriginalShift();
    item = new QStandardItem(QString("(%1;%2;%3)").arg(shift[0],0,'f',2).arg(shift[1],0,'f',2).arg(shift[2],0,'f',2));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

	fillSFWithPointCloud(_obj);
}

void ccPropertiesTreeDelegate::fillSFWithPointCloud(ccGenericPointCloud* _obj)
{
	assert(m_model);

    //for "real" point clouds only
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(_obj);
    if (!cloud)
        return;

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //Scalar fields
    int nsf = cloud->getNumberOfScalarFields();
    if (nsf>0)
    {
        addSeparator((nsf>1 ? "Scalar Fields" : "Scalar Field"));
        ++curRow;

        //fields number
        m_model->setRowCount(curRow+1);
        item = new QStandardItem("Number");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem(QString::number(nsf));
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,1,item);

        //fields combo
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Current");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem();
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
        item->setData(OBJECT_CURRENT_SCALAR_FIELD);
        m_model->setItem(curRow,1,item);
        m_view->openPersistentEditor(m_model->index(curRow,1));

		CCLib::ScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (!sf)
			return;

        //color-ramp combo
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Color ramp");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem();
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
        item->setData(OBJECT_CURRENT_COLOR_RAMP);
        m_model->setItem(curRow,1,item);
        m_view->openPersistentEditor(m_model->index(curRow,1));

        //color-ramp steps
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Color ramp steps");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem();
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
        item->setData(OBJECT_COLOR_RAMP_STEPS);
        m_model->setItem(curRow,1,item);
        m_view->openPersistentEditor(m_model->index(curRow,1));

        //Positive state
		m_model->setRowCount(++curRow+1);
		item = new QStandardItem("Positive");
		item->setFlags(Qt::ItemIsEnabled);
		m_model->setItem(curRow,0,item);

		item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
		item->setData(OBJECT_SCALAR_FIELD_POSITIVE);
		m_model->setItem(curRow,1,item);
        m_view->openPersistentEditor(m_model->index(curRow,1));

        ++curRow;
        addSeparator("SF Scale");

        //sliders (warning: 2 columns)
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem();
        item->setFlags(Qt::ItemIsEnabled);
        item->setData(OBJECT_CLOUD_SF_EDITOR);
        //QSize sz = item->sizeHint();
        //DGM: with last QT versions, vertical size of item is ignored!
        //--> new widgets are displayed on top of this one ... so we must
        //setup its size manually
        //item->setSizeHint(QSize(240,116));
        m_model->setItem(curRow,0,item);
        m_view->openPersistentEditor(m_model->index(curRow,0));

        //NaN distances in gray
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("NaN in grey");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(cloud->areNanScalarValuesInGrey() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_NAN_IN_GREY);
        m_model->setItem(curRow,1,item);

        //scale
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Display color scale");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(cloud->sfColorScaleShown() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_SCALAR_SCALE);
        m_model->setItem(curRow,1,item);
    }
}

void ccPropertiesTreeDelegate::fillWithPrimitive(ccGenericPrimitive* _obj)
{
    assert(_obj && m_model);

    addSeparator("Primitive");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //type
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Type");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	QString type;
	switch (_obj->getClassID())
	{
	case CC_PLANE:
		type="Plane";
		break;
	case CC_SPHERE:
		type="Sphere";
		break;
	case CC_TORUS:
		type="Torus";
		break;
	case CC_CYLINDER:
		type="Cylinder";
		break;
	case CC_CONE:
		type="Cone";
		break;
	case CC_BOX:
		type="Box";
		break;
	case CC_DISH:
		type="Dish";
		break;
	case CC_EXTRU:
		type="Extrusion";
		break;
	default:
		type="Unknown"; //DGM: ?!
		assert(false);
		break;
	}

    item = new QStandardItem(type);
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

	//drawing steps
	if (_obj->hasDrawingPrecision())
	{
		m_model->setRowCount(++curRow+1);
		item = new QStandardItem("Drawing precision");
		item->setFlags(Qt::ItemIsEnabled);
		m_model->setItem(curRow,0,item);

		item = new QStandardItem();
		item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
		item->setData(OBJECT_PRIMITIVE_PRECISION);
		m_model->setItem(curRow,1,item);
		m_view->openPersistentEditor(m_model->index(curRow,1));
	}
}

void ccPropertiesTreeDelegate::fillWithMesh(ccGenericMesh* _obj)
{
    assert(_obj && m_model);

    addSeparator("Mesh");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //number of facets
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Faces");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QLocale(QLocale::English).toString(_obj->size()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

	//material/texture
	if (_obj->hasMaterials())
    {
        m_model->setRowCount(++curRow+1);
        item = new QStandardItem("Materials/textures");
        item->setFlags(Qt::ItemIsEnabled);
        m_model->setItem(curRow,0,item);

        item = new QStandardItem("");
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
		item->setCheckState(_obj->materialsShown() ? Qt::Checked : Qt::Unchecked);
        item->setData(OBJECT_MATERIALS);
        m_model->setItem(curRow,1,item);
    }

    //wireframe
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("wireframe");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem("");
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
    item->setCheckState(_obj->isShownAsWire() ? Qt::Checked : Qt::Unchecked);
    item->setData(OBJECT_MESH_WIRE);
    m_model->setItem(curRow,1,item);

    //we also integrate vertices SF into mesh properties
    ccGenericPointCloud* vertices = _obj->getAssociatedCloud();
    if (vertices && (_obj->isA(CC_MESH_GROUP) || !vertices->isLocked() || _obj->isAncestorOf(vertices)))
        fillSFWithPointCloud(vertices);
}

void ccPropertiesTreeDelegate::fillWithPointOctree(ccOctree* _obj)
{
    assert(_obj && m_model);

    addSeparator("Octree");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //type
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Display type");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
    item->setData(OBJECT_OCTREE_TYPE);
    m_model->setItem(curRow,1,item);
    m_view->openPersistentEditor(m_model->index(curRow,1));

    //level
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Display level");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
    item->setData(OBJECT_OCTREE_LEVEL);
    m_model->setItem(curRow,1,item);
    m_view->openPersistentEditor(m_model->index(curRow,1));
}

void ccPropertiesTreeDelegate::fillWithImage(ccImage* _obj)
{
    assert(_obj && m_model);

    addSeparator("Image");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //image width
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Width");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getW()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //image height
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Height");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getH()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //transparency
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Alpha");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
    item->setData(OBJECT_IMAGE_ALPHA);
    m_model->setItem(curRow,1,item);
    m_view->openPersistentEditor(m_model->index(curRow,1));
}

void ccPropertiesTreeDelegate::fillWithCalibratedImage(ccCalibratedImage* _obj)
{
    assert(_obj && m_model);

    addSeparator("Calibrated Image");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //"Set Viewport" button
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Apply Viewport");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled /*| Qt::ItemIsEditable*/);
    item->setData(OBJECT_APPLY_IMAGE_VIEWPORT);
    m_model->setItem(curRow,1,item);
    m_view->openPersistentEditor(m_model->index(curRow,1));

    //field of view
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("f.o.v. (deg)");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getFov()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

	//get camera matrix parameters
	PointCoordinateType angle_rad;
	CCVector3 axis3D, t3D;
	_obj->getCameraMatrix().getParameters(angle_rad, axis3D, t3D);

    //camera position
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Optical center");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString("(%0,%1,%2)").arg(t3D.x).arg(t3D.y).arg(t3D.z));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //camera orientation
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Orientation");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString("(%0,%1,%2)").arg(axis3D.x).arg(axis3D.y).arg(axis3D.z));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //camera orientation (angle)
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Angle (degrees)");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString("%0°").arg(angle_rad*CC_RAD_TO_DEG));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);
}

void ccPropertiesTreeDelegate::fillWithLabel(cc2DLabel* _obj)
{
    assert(_obj && m_model);

    addSeparator("Label");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //Body
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Body");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	QStringList body = _obj->getLabelContent(ccGui::Parameters().displayedNumPrecision);
    item = new QStandardItem(body.join("\n"));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

	//Show label in 2D
	m_model->setRowCount(++curRow+1);
	item = new QStandardItem("Show 2D label");
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,0,item);

	item = new QStandardItem("");
	item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
	item->setCheckState(_obj->isDisplayedIn2D() ? Qt::Checked : Qt::Unchecked);
	item->setData(OBJECT_LABEL_DISP_2D);
	m_model->setItem(curRow,1,item);

	//Show label in 3D
	m_model->setRowCount(++curRow+1);
	item = new QStandardItem("Show 3D legend(s)");
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,0,item);

	item = new QStandardItem("");
	item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
	item->setCheckState(_obj->isDisplayedIn3D() ? Qt::Checked : Qt::Unchecked);
	item->setData(OBJECT_LABEL_DISP_3D);
	m_model->setItem(curRow,1,item);
}

void ccPropertiesTreeDelegate::fillWithViewportObject(cc2DViewportObject* _obj)
{
    assert(_obj && m_model);

    addSeparator("Viewport");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //Name
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Name");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	item = new QStandardItem(!_obj->getName().isEmpty() ? _obj->getName() : "undefined");
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,1,item);

    //"Apply Viewport" button
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Apply Viewport");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled /*| Qt::ItemIsEditable*/);
    item->setData(OBJECT_APPLY_LABEL_VIEWPORT);
    m_model->setItem(curRow,1,item);
    m_view->openPersistentEditor(m_model->index(curRow,1));
}

void ccPropertiesTreeDelegate::fillWithGBLSensor(ccGBLSensor* _obj)
{
    assert(_obj && m_model);

    addSeparator("GBL Sensor");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //Angular steps (phi)
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("dPhi");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getDeltaPhi()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //Angular steps (theta)
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("dTheta");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getDeltaTheta()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //Uncertainty
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Uncertainty");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->getUncertainty()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

    //SF color scale
    m_model->setRowCount(++curRow+1);
    item = new QStandardItem("Display color scale");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem();
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable);
    item->setData(OBJECT_SENSOR_DISPLAY_SCALE);
    m_model->setItem(curRow,1,item);
    m_view->openPersistentEditor(m_model->index(curRow,1));
}

void ccPropertiesTreeDelegate::fillWithMaterialSet(ccMaterialSet* _obj)
{
    assert(_obj && m_model);

    addSeparator("Material set");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //Count
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Count");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

    item = new QStandardItem(QString::number(_obj->size()));
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,1,item);

	//ccMaterialSet objects are 'shareable'
	fillWithShareable(_obj);
}

void ccPropertiesTreeDelegate::fillWithShareable(CCShareable* _obj)
{
    assert(_obj && m_model);

    addSeparator("Array");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

	//Link count
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Shared");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	unsigned linkCount = _obj->getLinkCount();
	if (linkCount<3) //if we display it, it means it is a member of the DB --> ie. link is already >1
		item = new QStandardItem(QString("No"));
	else
		item = new QStandardItem(QString("Yes (%1)").arg(linkCount-1)); //see above remark
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,1,item);
}

template<int N, class ScalarType> void ccPropertiesTreeDelegate::fillWithChunkedArray(ccChunkedArray<N,ScalarType>* _obj)
{
    assert(_obj && m_model);

    addSeparator("Array");

    int curRow = m_model->rowCount();
    QStandardItem* item = NULL;

    //Name
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Name");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	item = new QStandardItem(!_obj->getName().isEmpty() ? _obj->getName() : "undefined");
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,1,item);

    //Count
    m_model->setRowCount(++curRow+1);
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Elements");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	item = new QStandardItem(QString("%1/%2").arg(_obj->currentSize()).arg(_obj->capacity()));
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,1,item);

	//Memory
    m_model->setRowCount(++curRow+1);
    m_model->setRowCount(curRow+1);
    item = new QStandardItem("Memory");
    item->setFlags(Qt::ItemIsEnabled);
    m_model->setItem(curRow,0,item);

	item = new QStandardItem(QString("%1 Mb").arg((double)_obj->memory()/1048576.0,0,'f',2));
	item->setFlags(Qt::ItemIsEnabled);
	m_model->setItem(curRow,1,item);

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
    case OBJECT_DISPLAY:
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
	case OBJECT_SCALAR_FIELD_POSITIVE:
	{
        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        QCheckBox *checkBox = new QCheckBox(parent);

        ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (sf)
			checkBox->setChecked(sf->isPositive());
		else
			checkBox->setDisabled(true);

        connect(checkBox, SIGNAL(toggled(bool)), this, SLOT(scalarFieldTypeChanged(bool)));

		checkBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return checkBox;
	}
    case OBJECT_CURRENT_COLOR_RAMP:
    {
        QComboBox *comboBox = new QComboBox(parent);

        for (int i=0;i<COLOR_RAMPS_NUMBER;++i)
            comboBox->addItem(COLOR_RAMPS_TITLES[i]);

        connect(comboBox, SIGNAL(activated(int)), this, SLOT(colorRampChanged(int)));

		comboBox->setFocusPolicy(Qt::StrongFocus); //Qt doc: << The returned editor widget should have Qt::StrongFocus >>
        return comboBox;
    }
    case OBJECT_COLOR_RAMP_STEPS:
    {
        QSpinBox *spinBox = new QSpinBox(parent);
        spinBox->setRange(2,DEFAULT_COLOR_RAMP_SIZE);
        spinBox->setSingleStep(8);

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

        connect(sfd, SIGNAL(entitySFHasChanged()), this, SLOT(redrawObjectSF()));

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
    default:
        return QItemDelegate::createEditor(parent, option, index);
    }

    return NULL;
}

//bool ccPropertiesTreeDelegate::editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index)
//{
//	if (event->type() == QEvent::Resize)
//	{
//		if (!m_model || !m_currentObject)
//			return false;
//		
//		assert(m_model == model);
//
//		QStandardItem* item = m_model->itemFromIndex(index);
//
//		if (!item || !item->data().isValid() || (item->column()==0 && item->data().toInt()!=OBJECT_CLOUD_SF_EDITOR))
//			return false;
//
//		switch (item->data().toInt())
//		{
//		case OBJECT_CLOUD_SF_EDITOR:
//			//sfEditDlg *sfd = qobject_cast<sfEditDlg*>(editor);
//			//if (!sfd)
//			//	return;
//			event->accept();
//			return true;
//		}
//	}
//
//	return false;
//}

void ccPropertiesTreeDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	QItemDelegate::updateEditorGeometry(editor, option, index);

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
    case OBJECT_DISPLAY:
    {
        QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
        if (!comboBox)
            return;

        int pos = -1;
        ccGLWindow* win = static_cast<ccGLWindow*>(m_currentObject->getDisplay());
        if (win)
            pos = comboBox->findText(win->windowTitle());

        comboBox->setCurrentIndex(ccMax(pos,0)); //0 = "NONE"
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
	case OBJECT_SCALAR_FIELD_POSITIVE:
	{
        QCheckBox *checkBox = qobject_cast<QCheckBox*>(editor);
		if (!checkBox)
			return;

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (!sf)
			return;

		checkBox->setChecked(sf->isPositive());
		break;
	}
    case OBJECT_CURRENT_COLOR_RAMP:
    {
        QComboBox *comboBox = qobject_cast<QComboBox*>(editor);
        if (!comboBox)
            return;

        ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
        assert(cloud);

        ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
        if (sf)
		{
			int pos = int(sf->getColorRamp());
			comboBox->setCurrentIndex(pos);
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
            sfd->SetValuesWith(sf);
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
    default:
        QItemDelegate::setEditorData(editor, index);
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
		m_currentObject->setVisible(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	case OBJECT_COLORS:
		m_currentObject->showColors(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	case OBJECT_NORMALS:
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
	case OBJECT_SCALAR_FIELD:
		m_currentObject->showSF(item->checkState() == Qt::Checked);
		redraw=true;
		break;
	case OBJECT_NAN_IN_GREY:
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
			assert(cloud);
			cloud->setGreyForNanScalarValues(item->checkState() == Qt::Checked);
		}
		redraw=true;
		break;
	case OBJECT_SCALAR_SCALE:
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
	}

    if (redraw)
    {
        //DGM: point clouds may be mesh vertices which are generally hidden,
        //while the mesh depends on their parameters
        if (m_currentObject->isEnabled() ||
                (m_currentObject->isKindOf(CC_POINT_CLOUD) && m_currentObject->getParent() && m_currentObject->getParent()->isKindOf(CC_MESH)))
        {
            if (m_currentObject->isGroup())
                emit ccObjectAndChildrenAppearanceChanged(m_currentObject);
            else
                emit ccObjectAppearanceChanged(m_currentObject);
        }
    }
}

void ccPropertiesTreeDelegate::scalarFieldChanged(int pos)
{
    if (!m_currentObject)
        return;

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    assert(cloud);
    if (cloud->getCurrentDisplayedScalarFieldIndex()+1 != pos)
    {
        cloud->setCurrentDisplayedScalarField(pos-1);
		cloud->showSF(pos>0);
        if (cloud->isEnabled() || (cloud->getParent() && cloud->getParent()->isKindOf(CC_MESH)))
            emit ccObjectAppearanceChanged(m_currentObject);

        //we must reset the properties display!
        fillModel(m_currentObject);
    }
}

void ccPropertiesTreeDelegate::scalarFieldTypeChanged(bool positive)
{
    if (!m_currentObject)
        return;

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    assert(cloud);
	ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
	if (!sf)
		return;
    if (sf->isPositive() != positive)
    {
		sf->setPositive(positive);
		sf->computeMinAndMax();
        if (cloud->isEnabled() || (cloud->getParent() && cloud->getParent()->isKindOf(CC_MESH)))
            emit ccObjectAppearanceChanged(m_currentObject);

        //we must reset the properties display!
        fillModel(m_currentObject);
    }
}

void ccPropertiesTreeDelegate::colorRampChanged(int pos)
{
    if (!m_currentObject)
        return;

    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_currentObject);
    assert(cloud);

	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
	if (sf)
	{
		if (int(sf->getColorRamp()) != pos)
		{
			sf->setColorRamp(COLOR_RAMPS_ENUMS[pos]);

			if (cloud->isEnabled() || (cloud->getParent() && cloud->getParent()->isKindOf(CC_MESH)))
				emit ccObjectAppearanceChanged(m_currentObject);
		}
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

		if (cloud->isEnabled() || (cloud->getParent() && cloud->getParent()->isKindOf(CC_MESH)))
			emit ccObjectAppearanceChanged(m_currentObject);
	}
}

void ccPropertiesTreeDelegate::octreeDisplayTypeChanged(int pos)
{
    if (!m_currentObject)
        return;

    ccOctree* octree = ccHObjectCaster::ToOctree(m_currentObject);
    assert(octree);

    octree->setDisplayType(OCTREE_DISPLAY_TYPE_ENUMS[pos]);

    if (octree->isVisible() && octree->isEnabled())
        emit ccObjectAppearanceChanged(m_currentObject);
}

void ccPropertiesTreeDelegate::octreeDisplayedLevelChanged(int val)
{
    if (!m_currentObject)
        return;

    ccOctree* octree = ccHObjectCaster::ToOctree(m_currentObject);
    assert(octree);
    octree->setDisplayedLevel(val);

    if (octree->isVisible() && octree->isEnabled())
        emit ccObjectAppearanceChanged(m_currentObject);
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

    if (primitive->isVisible() && primitive->isEnabled())
        emit ccObjectAppearanceChanged(m_currentObject);

	//update delegate!
	fillModel(primitive);
}


void ccPropertiesTreeDelegate::imageAlphaChanged(int val)
{
    ccImage* image = ccHObjectCaster::ToImage(m_currentObject);
    if (!image)
        return;
    image->setAlpha(float(val)/255.0);

    if (image->isVisible() && image->isEnabled())
        emit ccObjectAppearanceChanged(m_currentObject);
}

void ccPropertiesTreeDelegate::applyImageViewport()
{
    if (!m_currentObject)
        return;

    ccCalibratedImage* image = ccHObjectCaster::ToCalibratedImage(m_currentObject);
    assert(image);

    //ccGLWindow* win = (image->getParent() ? image->getParent()->getDisplay() : 0);
    ccGLWindow* win = static_cast<ccGLWindow*>(image->getDisplay());
    if (!win)
        //win = GetActiveGLWindow();
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

    if (sensor->isVisible() && sensor->isEnabled())
        emit ccObjectAppearanceChanged(m_currentObject);
}

void ccPropertiesTreeDelegate::redrawObjectSF()
{
    if (!m_currentObject)
        return;

    if (m_currentObject->isEnabled() || (m_currentObject->getParent() && m_currentObject->getParent()->isKindOf(CC_MESH)))
        emit ccObjectAppearanceChanged(m_currentObject);
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
