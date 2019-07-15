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
			CCVector3 image_pt = CCVector3::fromArray(P.u);
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

void bdr2Point5DimEditor::projectToImage(ccHObject * obj)
{
	if (!getImage()) { return; }
	ccCameraSensor* cam = m_image->getAssociatedSensor();
	if (!cam) { return; }
	
	ccHObject* to_add = nullptr;
	ccGenericPointCloud* point_clone = nullptr;
	if (obj->isKindOf(CC_TYPES::POINT_CLOUD)) {
		point_clone = ccHObjectCaster::ToGenericPointCloud(obj)->clone();
		to_add = point_clone;
	}
	else if (obj->isA(CC_TYPES::MESH)) {
		ccMesh* mesh_clone = ccHObjectCaster::ToMesh(obj)->cloneMesh();
		mesh_clone->getAssociatedCloud();
	}
	else if (obj->isA(CC_TYPES::POLY_LINE)) {
		ccPolyline* poly = ccHObjectCaster::ToPolyline(obj);
		to_add = (poly ? new ccPolyline(*poly) : 0);
	}
	else if (obj->isA(CC_TYPES::LABEL_2D)) {

	}
	//! add to cam
	if (to_add && point_clone) {

		to_add->setDisplay(m_glWindow);
		cam->addChild(to_add);
		MainWindow* win = MainWindow::TheInstance(); assert(win);
		win->addToDB(to_add, true, false, true, true, CC_TYPES::DB_IMAGE);
	}
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

