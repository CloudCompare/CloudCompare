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

#include "bdr2.5DimEditor.h"

//Local
#include <QtWidgets/QHBoxLayout>
#include "ccPersistentSettings.h"
#include "mainwindow.h"

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccProgressDialog.h>
#include <ccColorTypes.h>
#include <ccImage.h>
#include <ccCameraSensor.h>
#include <ccPolyline.h>
#include <ccGLWindow.h>
#include <ccBBox.h>

//qCC_gl
#include <ccGLWidget.h>

//Qt
#include <QFrame>
#include <QSettings>
#include <QCoreApplication>

//System
#include <assert.h>

#include "stocker_parser.h"

bdr2Point5DimEditor::bdr2Point5DimEditor()
	: m_glWindow(nullptr)
	, m_associate_3DView(nullptr)
	, m_image(nullptr)
	, m_cursor_cross(nullptr)
{
}

bdr2Point5DimEditor::~bdr2Point5DimEditor()
{
	clearAll();
	if (m_cursor_cross) {
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_cursor_cross);
		delete m_cursor_cross;
		m_cursor_cross = nullptr;
	}
}

void bdr2Point5DimEditor::clearAll()
{
	if (m_image) {
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_image);
		delete m_image;
		m_image = nullptr;
	}
}

void bdr2Point5DimEditor::updateCursorPos(const CCVector3d& P, bool b3d, bool move)
{
	CCVector3 image_pt;
	if (!m_cursor_cross || m_cursor_cross->size() < 5) { return; }
	if (!b3d || !m_associate_3DView || !m_image) {
		m_cursor_cross->setVisible(false);
	}
	else {
		if (FromGlobalToImage(CCVector3::fromArray(P.u), image_pt)) {
			if (move) {
				ccGLCameraParameters cam;
				m_glWindow->getGLCameraParameters(cam);
				CCVector3d p2d;
				if (!cam.project(image_pt, p2d, true)) {
					m_glWindow->setPivotPoint(CCVector3d::fromArray(image_pt.u));
					m_glWindow->setCameraPos(CCVector3d::fromArray(image_pt.u));
				}
			}
			
			image_pt.z = IMAGE_MARKER_DISPLAY_Z;

			//! update image cursor pos
			*const_cast<CCVector3*>(m_cursor_cross->getPoint_local(0)) = image_pt;
			*const_cast<CCVector3*>(m_cursor_cross->getPoint_local(1)) = image_pt + CCVector3(0, 50, 0);
			*const_cast<CCVector3*>(m_cursor_cross->getPoint_local(2)) = image_pt - CCVector3(50, 0, 0);
			*const_cast<CCVector3*>(m_cursor_cross->getPoint_local(3)) = image_pt - CCVector3(0, 50, 0);
			*const_cast<CCVector3*>(m_cursor_cross->getPoint_local(4)) = image_pt + CCVector3(50, 0, 0);
			m_cursor_cross->setVisible(true);
		}
	}
	MainWindow::TheInstance()->setStatusImageCoord(CCVector3d::fromArray(image_pt.u), b3d);
	m_glWindow->redraw();
}

bool bdr2Point5DimEditor::FromGlobalToImage(const CCVector3 & P_global, CCVector3 & P_local, bool withLensError)
{
	if (!m_image || !m_image->getAssociatedSensor()) {
		return false;
	}
	CCVector2 p_2d;
	bool b_in_image = m_image->getAssociatedSensor()->fromGlobalCoordToImageCoord(P_global, p_2d, withLensError);
	p_2d.y = m_image->getH() - p_2d.y;
	P_local = CCVector3(p_2d, 0);

	return b_in_image;
}

void bdr2Point5DimEditor::init2DView()
{
	if (!m_glWindow) {
		return;
	}
	ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
	//black (text) & white (background) display by default
	params.backgroundCol = ccColor::white;
	params.textDefaultCol = ccColor::black;
	params.drawBackgroundGradient = false;
	params.decimateMeshOnMove = false;
	params.displayCross = false;
	params.colorScaleUseShader = false;
	m_glWindow->setDisplayParameters(params, true);
	m_glWindow->setPerspectiveState(false, true);
	m_glWindow->setInteractionMode(ccGLWindow::INTERACT_PAN | ccGLWindow::INTERACT_ZOOM_CAMERA | ccGLWindow::INTERACT_CLICKABLE_ITEMS | ccGLWindow::INTERACT_ROTATE);
	m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
	m_glWindow->displayOverlayEntities(true);
	m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_HIDE, false);
	m_glWindow->showCursorCoordinates(true);
	m_glWindow->setBBoxDisplayType(ccGLWindow::BBOX_HIDE);
	m_glWindow->setWindowEditorType(ccGLWindow::IMAGE_EDITOR_25D);
	m_glWindow->lockRotationAxis(true, CCVector3d(0, 0, 1));
}

void bdr2Point5DimEditor::create2DView(QFrame* parentFrame)
{
	if (m_glWindow) return;
	
	QWidget* glWidget = 0;
	CreateGLWindow(m_glWindow, glWidget, false, true);
	assert(m_glWindow && glWidget);

	init2DView();

	ccPointCloud* pc = new ccPointCloud();
	for (size_t i = 0; i < 5; i++) { pc->addPoint(CCVector3(0, 0, 0)); }	
	m_cursor_cross = new ccPolyline(pc);
	
	m_cursor_cross->addPointIndex(0); m_cursor_cross->addPointIndex(1);
	m_cursor_cross->addPointIndex(0); m_cursor_cross->addPointIndex(2);
	m_cursor_cross->addPointIndex(0); m_cursor_cross->addPointIndex(3);
	m_cursor_cross->addPointIndex(0); m_cursor_cross->addPointIndex(4);
	m_cursor_cross->setVisible(false);
	m_cursor_cross->setDisplay(m_glWindow);
	m_cursor_cross->setColor(ccColor::red);
	m_cursor_cross->showColors(true);
	m_cursor_cross->setWidth(1);
	m_glWindow->addToOwnDB(m_cursor_cross);

	//add window to the input frame (if any)
	if (parentFrame)
	{
		auto	layout = new QHBoxLayout;

		layout->setContentsMargins(0, 0, 0, 0);
		layout->addWidget(glWidget);

		parentFrame->setLayout(layout);
	}	
}

void bdr2Point5DimEditor::setAssociate3DView(ccGLWindow * win)
{
	if (!win) {
		return;
	}
	if (m_associate_3DView && m_associate_3DView == win) {
		return;
	}
	m_associate_3DView = win;
}

void bdr2Point5DimEditor::update2DDisplayZoom(ccBBox& box, CCVector3d up)
{
	if (!m_glWindow /*|| !m_grid.isValid()*/)
		return;
//	m_glWindow->setView(CC_TOP_VIEW, false);
	//equivalent to 'ccGLWindow::updateConstellationCenterAndZoom' but we take aspect ratio into account

	//we compute the pixel size (in world coordinates)
	{
		ccViewportParameters params = m_glWindow->getViewportParameters();
		static const int screnMargin = 5;
		int screenWidth = std::max(1, m_glWindow->glWidth() - 2 * screnMargin);
		int screenHeight = std::max(1, m_glWindow->glHeight() - 2 * screnMargin);

		params.pixelSize = 1.0f/*static_cast<float>(std::max(realGridWidth / screenWidth, realGridHeight / screenHeight))*/;
		params.zoom = static_cast<float>(std::min((double)screenWidth / (double)box.getDiagVec().x, (double)screenHeight / (double)box.getDiagVec().y));
		
		m_glWindow->setViewportParameters(params);
		m_glWindow->setPointSize(1.0f);
	}
	
	//we set the pivot point on the box center
	CCVector3 P = box.getCenter();
//	P.y = m_image->getH() - P.y;
	m_glWindow->setPivotPoint(CCVector3d::fromArray(P.u));
	m_glWindow->setCameraPos(CCVector3d::fromArray(P.u));

	ccGLMatrixd viewMat = ccGLMatrixd::FromViewDirAndUpDir(CCVector3d(0, 0, -1), up);
	m_glWindow->setBaseViewMat(viewMat);

	m_glWindow->invalidateViewport();
	m_glWindow->invalidateVisualization();
	m_glWindow->deprecate3DLayer();
	m_glWindow->redraw();
}

void bdr2Point5DimEditor::setImage(QString image_path)
{
	if (m_image) {
		if (m_glWindow)
			m_glWindow->removeFromOwnDB(m_image);
		delete m_image;
		m_image = nullptr;
	}
	m_image = new ccImage;
	m_image->setDisplayType(ccImage::IMAGE_DISPLAY_2P5D);
	m_image->setDisplay(m_glWindow);
	QString error;
	m_image->load(image_path, error);
	m_glWindow->addToOwnDB(m_image);
	//ZoomFit();	// called outside
}

void bdr2Point5DimEditor::setImageAndCamera(ccCameraSensor * cam)
{
	setImage(cam->imagePath());
	m_image->setAssociatedSensor(cam);
}

ccHObject* bdr2Point5DimEditor::projectToImage(ccHObject * obj)
{
	if (!getImage()) { return nullptr; }
	ccCameraSensor* cam = m_image->getAssociatedSensor();
	if (!cam) { return nullptr; }
	
	ccHObject* entity_in_image_2d = nullptr;
	//! keep the index but project the point in associate cloud to 2d image
	//! get the associate cloud
	ccGenericPointCloud* associate_cloud = nullptr;

	if (obj->isKindOf(CC_TYPES::POINT_CLOUD)) {
		associate_cloud = ccHObjectCaster::ToGenericPointCloud(obj)->clone();
		entity_in_image_2d = associate_cloud;
	}
	else if (obj->isA(CC_TYPES::MESH)) {
		ccMesh* mesh_clone = ccHObjectCaster::ToMesh(obj)->cloneMesh();
		associate_cloud = mesh_clone->getAssociatedCloud();
		entity_in_image_2d = mesh_clone;
	}
	else if (obj->isKindOf(CC_TYPES::POLY_LINE)) {
		ccPolyline* poly = ccHObjectCaster::ToPolyline(obj);
		
		ccPolyline* new_poly = new ccPolyline(*poly); if (!new_poly) return nullptr;
		associate_cloud = dynamic_cast<ccPointCloud*>(new_poly->getAssociatedCloud());
		if (!associate_cloud) { 
			delete new_poly;
			new_poly = nullptr;
			return nullptr;
		}

		entity_in_image_2d = new_poly;
	}
	else if (obj->isA(CC_TYPES::LABEL_2D)) {

	}
	else if (obj->isA(CC_TYPES::ST_BLOCK)) {
		//! get top
		StBlock* block = ccHObjectCaster::ToStBlock(obj); if (!block) return nullptr;
		ccFacet* top_facet = block->getTopFacet(); if (!top_facet) return nullptr;
		ccPolyline* contour = top_facet->getContour(); if (!contour) return nullptr;
		ccPolyline* new_poly = new ccPolyline(*contour); if (!new_poly) return nullptr;

		associate_cloud = dynamic_cast<ccPointCloud*>(new_poly->getAssociatedCloud());
		if (!associate_cloud) {
			delete new_poly;
			new_poly = nullptr;
			return nullptr;
		}

		entity_in_image_2d = new_poly;
	}

	//! project to image
	unsigned inside_image(0);
	if (associate_cloud && entity_in_image_2d) {
		associate_cloud->setGlobalShift(CCVector3d());
		associate_cloud->setGlobalScale(0);

		for (size_t i = 0; i < associate_cloud->size(); i++) {
			CCVector3* v = const_cast<CCVector3*>(associate_cloud->getPoint(i));
			CCVector3 image_pt;
			if (FromGlobalToImage(*v, image_pt)) {
				inside_image++;
			}			
			image_pt.z = IMAGE_MARKER_DISPLAY_Z;
			*v = image_pt;
		}
		// project even if the point is out of range, but remove that no point is inside
		if (inside_image == 0) {
			delete associate_cloud;
			associate_cloud = nullptr;
			delete entity_in_image_2d;
			entity_in_image_2d = nullptr;
		}
	}
	

	//! add to cam
	if (entity_in_image_2d) {
		ccShiftedObject* shift = ccHObjectCaster::ToShifted(entity_in_image_2d);
		if (shift) {
			shift->setGlobalShift(CCVector3d());
			shift->setGlobalScale(0);
		}

		entity_in_image_2d->setDisplay_recursive(m_glWindow);
		entity_in_image_2d->setEnabled(true);
		cam->addChild(entity_in_image_2d);
		MainWindow* win = MainWindow::TheInstance(); assert(win);
		win->addToDB_Image(entity_in_image_2d, true, false, true, true);
	}
	return entity_in_image_2d;
}

void bdr2Point5DimEditor::ZoomFit()
{
	if (!m_image) {
		return;
	}
	ccBBox box;
	box.add(CCVector3(0, 0, 0));
	box.add(CCVector3(m_image->getW(), m_image->getH(), 0));
	update2DDisplayZoom(box);
}

