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

#ifndef CC_ITEM_DELEGATE_HEADER
#define CC_ITEM_DELEGATE_HEADER

//qCC_db
#include <ccArray.h>

//Qt
#include <QStyledItemDelegate>

class cc2DLabel;
class cc2DViewportObject;
class ccCameraSensor;
class ccFacet;
class ccGBLSensor;
class ccGenericMesh;
class ccGenericPointCloud;
class ccGenericPrimitive;
class ccHObject;
class ccImage;
class ccIndexedTransformationBuffer;
class ccKdTree;
class ccMaterialSet;
class ccOctree;
class ccPlanarEntityInterface;
class ccPolyline;
class ccSensor;
class CCShareable;
class ccShiftedObject;

class QAbstractItemView;
class QStandardItem;
class QStandardItemModel;

//! GUI properties list dialog element
class ccPropertiesTreeDelegate : public QStyledItemDelegate
{
	Q_OBJECT

public:

	//! Delegate items roles
	enum CC_PROPERTY_ROLE { OBJECT_NO_PROPERTY					= 0	,
							OBJECT_NAME								,
							OBJECT_VISIBILITY						,
							OBJECT_CURRENT_DISPLAY					,
							OBJECT_NORMALS_SHOWN					,
							OBJECT_COLOR_SOURCE						,
							OBJECT_POLYLINE_WIDTH					,
							OBJECT_SENSOR_DRAW_FRUSTUM				,
							OBJECT_SENSOR_DRAW_FRUSTUM_PLANES		,
							OBJECT_SF_SHOW_SCALE					,
							OBJECT_OCTREE_LEVEL						,
							OBJECT_OCTREE_TYPE						,
							OBJECT_MESH_WIRE						,
							OBJECT_MESH_STIPPLING					,
							OBJECT_CURRENT_SCALAR_FIELD				,
							OBJECT_CURRENT_COLOR_RAMP				,
							OBJECT_IMAGE_ALPHA						,
							OBJECT_APPLY_IMAGE_VIEWPORT				,
							OBJECT_APPLY_SENSOR_VIEWPORT			,
							OBJECT_CLOUD_SF_EDITOR					,
							OBJECT_SENSOR_MATRIX_EDITOR				,
							OBJECT_SENSOR_DISPLAY_SCALE				,
							OBJECT_SENSOR_UNCERTAINTY				,
							OBJECT_COLOR_RAMP_STEPS					,
							OBJECT_MATERIALS						,
							OBJECT_APPLY_LABEL_VIEWPORT				,
							OBJECT_UPDATE_LABEL_VIEWPORT			,
							OBJECT_LABEL_DISP_2D					,
							OBJECT_LABEL_POINT_LEGEND				,
							OBJECT_PRIMITIVE_PRECISION				,
							OBJECT_SPHERE_RADIUS					,
							OBJECT_CONE_HEIGHT						,
							OBJECT_CONE_BOTTOM_RADIUS				,
							OBJECT_CONE_TOP_RADIUS					,
							OBJECT_CLOUD_POINT_SIZE					,
							OBJECT_NAME_IN_3D						,
							OBJECT_FACET_CONTOUR					,
							OBJECT_FACET_MESH						,
							OBJECT_PLANE_NORMAL_VECTOR				,
							OBJECT_SENSOR_INDEX						,
							OBJECT_SHOW_TRANS_BUFFER_PATH			,
							OBJECT_SHOW_TRANS_BUFFER_TRIHDERONS		,
							OBJECT_TRANS_BUFFER_TRIHDERONS_SCALE	,
							OBJECT_HISTORY_MATRIX_EDITOR			,
							OBJECT_GLTRANS_MATRIX_EDITOR			,
							TREE_VIEW_HEADER						,
	};

	//! Default constructor
	ccPropertiesTreeDelegate(QStandardItemModel* _model, QAbstractItemView* _view, QObject *parent = nullptr);

	//! Default destructor
	~ccPropertiesTreeDelegate() override;

	//inherited from QStyledItemDelegate
	QSize sizeHint(const QStyleOptionViewItem& option, const QModelIndex& index ) const override;
	QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
	void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	void setEditorData(QWidget *editor, const QModelIndex &index) const override;
	
	void unbind();

	//! Fill property view with QItems corresponding to object's type
	void fillModel(ccHObject* hObject);

	//! Returns currently bound object
	ccHObject* getCurrentObject();

signals:
	void ccObjectPropertiesChanged(ccHObject* hObject) const;
	void ccObjectAppearanceChanged(ccHObject* hObject) const;
	void ccObjectAndChildrenAppearanceChanged(ccHObject* hObject) const;

private:
	static const char* s_noneString;
	static const char* s_rgbColor;
	static const char* s_sfColor;
	static const char* s_defaultPointSizeString;
	static const char* s_defaultPolyWidthSizeString;
	
	void updateItem(QStandardItem*);
	void scalarFieldChanged(int);
	void colorScaleChanged(int);
	void colorRampStepsChanged(int);
	void spawnColorRampEditor();
	void octreeDisplayModeChanged(int);
	void octreeDisplayedLevelChanged(int);
	void primitivePrecisionChanged(int);
	void sphereRadiusChanged(double);
	void coneHeightChanged(double);
	void coneBottomRadiusChanged(double);
	void coneTopRadiusChanged(double);
	void imageAlphaChanged(int);
	void applyImageViewport();
	void applySensorViewport();
	void applyLabelViewport();
	void updateLabelViewport();
	void updateDisplay();
	void objectDisplayChanged(const QString &);
	void colorSourceChanged(const QString &);
	void sensorScaleChanged(double);
	void sensorUncertaintyChanged();
	void sensorIndexChanged(double);
	void cloudPointSizeChanged(int);
	void polyineWidthChanged(int);
	void trihedronsScaleChanged(double);

	void addSeparator(const QString& title);
	void appendRow(QStandardItem* leftItem, QStandardItem* rightItem, bool openPersistentEditor = false);
	void appendWideRow(QStandardItem* item, bool openPersistentEditor = true);

	void fillWithHObject(ccHObject*);
	void fillWithPointCloud(ccGenericPointCloud*);
	void fillSFWithPointCloud(ccGenericPointCloud*);
	void fillWithMesh(const ccGenericMesh*);
	void fillWithFacet(const ccFacet*);
	void fillWithPlanarEntity(const ccPlanarEntityInterface*);
	void fillWithSensor(const ccSensor*);
	void fillWithTransBuffer(const ccIndexedTransformationBuffer*);
	void fillWithPolyline(const ccPolyline*);
	void fillWithPrimitive(const ccGenericPrimitive*);
	void fillWithPointOctree(const ccOctree*);
	void fillWithPointKdTree(const ccKdTree*);
	void fillWithImage(const ccImage*);
	void fillWithLabel(const cc2DLabel*);
	void fillWithViewportObject(const cc2DViewportObject*);
	void fillWithGBLSensor(const ccGBLSensor*);
	void fillWithCameraSensor(const ccCameraSensor*);
	void fillWithMaterialSet(const ccMaterialSet*);
	void fillWithShareable(const CCShareable*);
	void fillWithMetaData(const ccObject*);
	void fillWithShifted(const ccShiftedObject*);
	
	template<class Type, int N, class ComponentType>
	void fillWithCCArray(const ccArray<Type, N, ComponentType>*);

	//! Returns whether the editor is wide (i.e. spans on two columns) or not
	bool isWideEditor(int itemData) const;

	//! Updates the current model (assuming object is the same)
	void updateModel();

	ccHObject* m_currentObject;
	QStandardItemModel* m_model;
	QAbstractItemView* m_view;
};

#endif
