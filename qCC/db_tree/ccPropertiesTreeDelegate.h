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
//$Rev:: 2251                                                              $
//$LastChangedDate:: 2012-10-08 23:56:41 +0200 (lun., 08 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_ITEM_DELEGATE_HEADER
#define CC_ITEM_DELEGATE_HEADER

//qCC_db
#include <ccChunkedArray.h>

//Qt
#include  <QItemDelegate>

class ccHObject;
class ccGenericPointCloud;
class ccGenericMesh;
class ccGenericPrimitive;
class ccOctree;
class ccImage;
class ccCalibratedImage;
class ccGBLSensor;
class ccMaterialSet;
class cc2DLabel;
class cc2DViewportObject;
class CCShareable;

class QStandardItemModel;
class QStandardItem;
class QAbstractItemView;

enum CC_PROPERTY_ROLE { OBJECT_NAME                     =   1,
                        OBJECT_VISIBILITY               =   2,
                        OBJECT_DISPLAY                  =   3,
                        OBJECT_COLORS                   =   4,
                        OBJECT_NORMALS                  =   5,
                        OBJECT_SCALAR_FIELD             =   6,
                        OBJECT_SCALAR_FIELD_POSITIVE	=   7,
                        OBJECT_NAN_IN_GREY              =   8,
                        OBJECT_SCALAR_SCALE             =   9,
                        OBJECT_OCTREE_LEVEL             =   10,
                        OBJECT_OCTREE_TYPE              =   11,
                        OBJECT_MESH_WIRE                =   12,
                        OBJECT_CURRENT_SCALAR_FIELD     =   13,
                        OBJECT_CURRENT_COLOR_RAMP       =   14,
                        OBJECT_IMAGE_ALPHA              =   15,
                        OBJECT_APPLY_IMAGE_VIEWPORT     =   16,
                        OBJECT_CLOUD_SF_EDITOR          =   17,
                        OBJECT_SENSOR_DISPLAY_SCALE     =   18,
                        OBJECT_COLOR_RAMP_STEPS         =   19,
						OBJECT_MATERIALS				=	20,
						OBJECT_APPLY_LABEL_VIEWPORT		=	21,
						OBJECT_LABEL_DISP_2D			=	22,
						OBJECT_LABEL_DISP_3D			=	23,
                        OBJECT_PRIMITIVE_PRECISION      =   24,
};

//! GUI properties list dialog element
class ccPropertiesTreeDelegate : public QItemDelegate
{
    Q_OBJECT

public:

    //! Default constructor
    ccPropertiesTreeDelegate(QStandardItemModel* _model, QAbstractItemView* _view, QObject *parent = 0);

    //! Default destructor
    virtual ~ccPropertiesTreeDelegate();

    //inherited from QItemDelegate
    virtual QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index ) const;
    virtual QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;
	//virtual bool editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index);
	virtual void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const;
    virtual void setEditorData(QWidget *editor, const QModelIndex &index) const;
    virtual void unbind();

    //! Fill property view with QItems corresponding to object's type
    void fillModel(ccHObject* hObject);

    //! Returns currently bound object
    ccHObject* getCurrentObject();

signals:
    void ccObjectPropertiesChanged(ccHObject* hObject) const;
    void ccObjectAppearanceChanged(ccHObject* hObject) const;
    void ccObjectAndChildrenAppearanceChanged(ccHObject* hObject) const;

protected slots:
    void updateItem(QStandardItem* item);
    void scalarFieldChanged(int pos);
	void scalarFieldTypeChanged(bool positive);
    void colorRampChanged(int pos);
    void octreeDisplayTypeChanged(int pos);
    void octreeDisplayedLevelChanged(int val);
	void primitivePrecisionChanged(int val);
    void colorRampStepsChanged(int val);
    void imageAlphaChanged(int val);
    void applyImageViewport();
	void applyLabelViewport();
    void redrawObjectSF();
    void objectDisplayChanged(const QString &newDisplayTitle);
    void sensorScaleChanged(double val);

protected:

    void addSeparator(const char* title);

    void fillWithHObject(ccHObject*);
    void fillWithPointCloud(ccGenericPointCloud*);
    void fillSFWithPointCloud(ccGenericPointCloud*);
    void fillWithMesh(ccGenericMesh*);
    void fillWithPrimitive(ccGenericPrimitive*);
    void fillWithPointOctree(ccOctree*);
    void fillWithImage(ccImage*);
    void fillWithCalibratedImage(ccCalibratedImage*);
	void fillWithLabel(cc2DLabel*);
	void fillWithViewportObject(cc2DViewportObject*);
    void fillWithGBLSensor(ccGBLSensor*);
	void fillWithMaterialSet(ccMaterialSet*);
	void fillWithShareable(CCShareable*);
	template<int N, class ScalarType> void fillWithChunkedArray(ccChunkedArray<N,ScalarType>*);

    ccHObject* m_currentObject;
    QStandardItemModel* m_model;
    QAbstractItemView* m_view;
};

#endif
