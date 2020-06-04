//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBroom                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      COPYRIGHT: Wesley Grimes (Collision Engineering Associates)       #
//#                                                                        #
//##########################################################################

#include "qBroomDlg.h"
#include <ccMainAppInterface.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccBox.h>
#include <cc2DLabel.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>
#include <ccAdvancedTypes.h>
#include <ccPlane.h>
#include <ccPolyline.h>

//qCC_fbo
#include <ccGlFilter.h>
#include <ccGLWidget.h>

//CCLib
#include <DgmOctreeReferenceCloud.h>
#include <Neighbourhood.h>

//Qt
#include <QMessageBox>
#include <QMainWindow>
#include <QSettings>
#include <QCloseEvent>

//intersection between a plane (the broom plane) and a line (represented by two points)
static bool Intersection(const ccGLMatrix& broomTrans, const CCVector3& A, const CCVector3& B, CCVector3& I)
{
	//plane normal and center
	CCVector3 N = broomTrans.getColumnAsVec3D(2);
	CCVector3 O = broomTrans.getTranslationAsVec3D();

	CCVector3 AB = B-A;
	CCVector3 OA = A-O;
	PointCoordinateType ABdotN = AB.dot(N);
	PointCoordinateType OAdotN = OA.dot(N);
	if (fabs(ABdotN) < std::numeric_limits<PointCoordinateType>::epsilon())
	{
		//the plane and the line are parallel!
		if (fabs(OAdotN) < std::numeric_limits<PointCoordinateType>::epsilon())
		{
			//the line lies totally in the plane
			I = O;
			return true;
		}
		else
		{
			return false;
		}
	}

	PointCoordinateType s = - OAdotN / ABdotN;
	I = A + s * AB;

	return true;

}

qBroomDlg::qBroomDlg(ccMainAppInterface* app/*=0*/)
	: QDialog(app ? app->getMainWindow() : nullptr, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, Ui::BroomDialog()
	, m_glWindow(nullptr)
	, m_broomBox(new ccBox("Broom"))
	, m_selectionBox(new ccBox("Selection"))
	, m_boxes(new ccHObject("Boxes"))
	, m_lastMousePos(0,0)
	, m_lastMousePos3D(0,0,0)
	, m_hasLastMousePos3D(false)
	, m_broomSelected(false)
	, m_selectionMode(ABOVE)
	, m_app(app)
	, m_initialCloud(nullptr)
{
	setupUi(this);

	lostTrackFrame->setVisible(false);

	//load persistent settings
	int selectionMode = selectionModeComboBox->currentIndex();
	{
		QSettings settings;
		settings.beginGroup("qBroom");
		selectionMode = settings.value("selectionMode", selectionMode).toInt();
		bool stickToTheFloor = settings.value("stickToTheFloor", stickCheckBox->isChecked()).toBool();
		bool animateAutomation = settings.value("animatedAutomation", animateAutomationCheckBox->isChecked()).toBool();

		stickCheckBox->setChecked(stickToTheFloor);
		animateAutomationCheckBox->setChecked(animateAutomation);
	}

	//force update
	selectionModeComboBox->setCurrentIndex(selectionMode);
	onSelectionModeChanged(selectionMode);

	//create 3D view
	{
		QWidget* glWidget = nullptr;
		m_app->createGLWindow(m_glWindow, glWidget);
		assert(m_glWindow && glWidget);

		m_glWindow->setPerspectiveState(false, true);
		m_glWindow->displayOverlayEntities(true);
		m_glWindow->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
		
		//add window to the input frame (if any)
		viewFrame->setLayout(new QHBoxLayout());
		viewFrame->layout()->addWidget(glWidget);
	}

	//items
	{
		m_broomBox->setEnabled(true);
		m_broomBox->setDisplay(m_glWindow);

		m_selectionBox->setEnabled(true);
		m_selectionBox->setDisplay(m_glWindow);
		
		m_boxes->setEnabled(false);
		m_boxes->addChild(m_broomBox);
		m_boxes->addChild(m_selectionBox);
		m_glWindow->addToOwnDB(m_boxes);
	}

	//connect signals/slots
	{
		connect(m_glWindow, SIGNAL(itemPicked(ccHObject*, unsigned, int, int, const CCVector3&, const CCVector3d&)), this, SLOT(handlePickedItem(ccHObject*, unsigned, int, int, const CCVector3&, const CCVector3d&)));

		connect(m_glWindow, SIGNAL(leftButtonClicked(int,int)), this, SLOT(onLeftButtonClicked(int,int)));
		//connect(m_glWindow, SIGNAL(rightButtonClicked(int,int)), this, SLOT(onRightButtonClicked(int,int)));
		connect(m_glWindow, SIGNAL(mouseMoved(int,int,Qt::MouseButtons)), this, SLOT(onMouseMoved(int,int,Qt::MouseButtons)));
		connect(m_glWindow, SIGNAL(buttonReleased()), this, SLOT(onButtonReleased()));

		connect(cleanHeightDoubleSpinBox,    SIGNAL(valueChanged(double)), this, SLOT(onCleanHeightChanged(double)));
		connect(broomLengthDoubleSpinBox,    SIGNAL(valueChanged(double)), this, SLOT(onDimensionChanged(double)));
		connect(broomWidthDoubleSpinBox,     SIGNAL(valueChanged(double)), this, SLOT(onDimensionChanged(double)));
		connect(broomThicknessDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onDimensionChanged(double)));

		connect(selectionModeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onSelectionModeChanged(int)));
		connect(undoPushButton,        SIGNAL(clicked()), this, SLOT(doUndo()));
		connect(undo10PushButton,      SIGNAL(clicked()), this, SLOT(doUndo10()));
		connect(repositionPushButton,  SIGNAL(clicked()), this, SLOT(onReposition()));
		connect(automatePushButton,    SIGNAL(clicked()), this, SLOT(onAutomate()));

		connect(cancelPushButton,   SIGNAL(clicked()), this, SLOT(cancel()));
		connect(applyPushButton,    SIGNAL(clicked()), this, SLOT(apply()));
		connect(validatePushButton, SIGNAL(clicked()), this, SLOT(validate()));

		//view buttons
		connect(topViewToolButton,    &QToolButton::clicked, [&]() { if (m_glWindow) m_glWindow->setView(CC_TOP_VIEW  ); });
		connect(frontViewToolButton,  &QToolButton::clicked, [&]() { if (m_glWindow) m_glWindow->setView(CC_FRONT_VIEW); });
		connect(leftViewToolButton,   &QToolButton::clicked, [&]() { if (m_glWindow) m_glWindow->setView(CC_LEFT_VIEW ); });
		connect(backViewToolButton,   &QToolButton::clicked, [&]() { if (m_glWindow) m_glWindow->setView(CC_BACK_VIEW ); });
		connect(rightViewToolButton,  &QToolButton::clicked, [&]() { if (m_glWindow) m_glWindow->setView(CC_RIGHT_VIEW); });
		connect(bottomViewToolButton, &QToolButton::clicked, [&]() { if (m_glWindow) m_glWindow->setView(CC_TOP_VIEW  ); });
	}

	freezeUI(true);
}

qBroomDlg::~qBroomDlg()
{
	if (m_glWindow)
	{
		m_glWindow->getOwnDB()->removeAllChildren();
		if (m_app)
		{
			m_app->destroyGLWindow(m_glWindow);
			m_glWindow = nullptr;
		}
	}
	if (m_boxes)
	{
		delete m_boxes;
		m_boxes = nullptr;
	}
}

cc2DLabel* qBroomDlg::Picking::addLabel(ccGenericPointCloud* cloud, unsigned pointIndex)
{
	cc2DLabel* label = new cc2DLabel(QString("%1").arg(labels.size()+1));
	label->addPickedPoint(cloud, pointIndex);

	//label->setEnabled(false);
	label->setDisplayedIn2D(false);

	try
	{
		labels.push_back(label);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory");
		delete label;
		label = nullptr;
	}

	return label;
}
	
void qBroomDlg::AutomationArea::clear()
{
	//remove the automation area polyline (if any)
	if (polyline)
	{
		//auto remove from display
		if (polyline->getDisplay())
		{
			static_cast<ccGLWindow*>(polyline->getDisplay())->removeFromOwnDB(polyline);
		}

		delete polyline;
		polyline = nullptr;
	}

	clickedPoints.resize(0);
}

void qBroomDlg::Picking::clear()
{
	//remove the 'labels' (if any)
	for (size_t i = 0; i < labels.size(); ++i)
	{
		cc2DLabel* label = labels[i];
		if (label)
		{
			//auto remove from display
			if (label->getDisplay())
			{
				static_cast<ccGLWindow*>(label->getDisplay())->removeFromOwnDB(label);
			}

			delete label;
		}
	}

	labels.resize(0);
}

void qBroomDlg::freezeUI(bool state)
{
	broomDimGroupBox->setDisabled(state);
	cleanAreaGroupBox->setDisabled(state);
}

void qBroomDlg::CloudBackup::backup(ccPointCloud* cloud)
{
	//save state
	assert(!colors);
	wasVisible = cloud->isVisible();
	wasEnabled = cloud->isEnabled();
	wasSelected = cloud->isSelected();
	hadColors = cloud->hasColors();
	displayedSFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
	originDisplay = cloud->getDisplay();
	colorsWereDisplayed = cloud->colorsShown();
	sfWasDisplayed = cloud->sfShown();
	hadOctree = (cloud->getOctree() != nullptr);
	ref = cloud;
}

bool qBroomDlg::CloudBackup::backupColors()
{
	if (!ref)
	{
		assert(false);
		return false;
	}
	
	//we backup the colors (as we are going to change them)
	if (ref->hasColors())
	{
		colors = new RGBAColorsTableType;
		if (!colors->resizeSafe(ref->size()))
		{
			//not enough memory
			colors->release();
			colors = nullptr;
			return false;
		}

		//copy the existing colors
		for (unsigned i = 0; i < ref->size(); ++i)
		{
			colors->setValue(i, ref->getPointColor(i));
		}
	}

	return true;
}

void qBroomDlg::CloudBackup::restore()
{
	if (!ref)
	{
		//nothing to do
		return;
	}

	if (!hadOctree)
	{
		//we can only delete the octree if it has not already been added to the DB tree!!!
		if (!ref->getParent())
		{
			ref->deleteOctree();
		}
	}

	if (hadColors)
	{
		//restore original colors
		if (colors)
		{
			assert(ref->hasColors());
			for (unsigned i=0; i<ref->size(); ++i)
			{
				ref->setPointColor(i, colors->getValue(i));
			}
		}
	}
	else
	{
		ref->unallocateColors();
	}

	ref->setEnabled(wasEnabled);
	ref->setVisible(wasVisible);
	ref->setSelected(wasSelected);
	ref->showColors(colorsWereDisplayed);
	ref->showSF(sfWasDisplayed);
	ref->setCurrentDisplayedScalarField(displayedSFIndex);
	ref->setDisplay(originDisplay);
}

void qBroomDlg::CloudBackup::clear()
{
	if (colors)
	{
		colors->release();
		colors = nullptr;
	}

	if (ref)
	{
		if (ownCloud)
		{
			//the dialog takes care of its own clouds!
			delete ref;
		}
		ref = nullptr;
	}
}

bool qBroomDlg::setCloud(ccPointCloud* cloud, bool ownCloud/*=false*/, bool autoRedraw/*=true*/)
{
	if (!m_glWindow)
	{
		assert(false);
		return false;
	}

	if (m_cloud.ref == cloud)
	{
		//just in case
		m_cloud.ownCloud = ownCloud;
		//nothing to do
		return true;
	}
	
	if (m_cloud.ref)
	{
		m_glWindow->removeFromOwnDB(m_cloud.ref);
		m_cloud.restore();
		m_cloud.clear();
	}

	if (cloud)
	{
		unsigned pointCount = cloud->size();
		if (pointCount < 10)
		{
			ccLog::Error("Cloud is too small!");
			return false;
		}
		
		//backup the cloud original state
		m_cloud.backup(cloud);

		if (!m_cloud.backupColors())
		{
			//failed to backup the cloud colors
			ccLog::Error("Not enough memory");
			return false;
		}

		//we need a selection table
		try
		{
			m_selectionTable.clear();
			m_selectionTable.resize(pointCount, 0);
			m_undoPositions.resize(0);
			m_undoPositions.reserve(1);
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Error("Not enough memory");
			return false;
		}
		undoPushButton->setEnabled(false);
		undo10PushButton->setEnabled(false);
		applyPushButton->setEnabled(false);
		validatePushButton->setEnabled(ownCloud);

		//we need an octree
		if (!cloud->getOctree())
		{
			ccProgressDialog pDlg(true, this);
			ccOctree::Shared octree = cloud->computeOctree(&pDlg);
			if (!octree)
			{
				ccLog::Error("Failed to compute octree!");
				return false;
			}
			else if (cloud->getParent())
			{
				m_app->addToDB(cloud->getOctreeProxy());
			}
		}

		//we need colors
		if (!cloud->hasColors())
		{
			//either we convert the current scalar field to RGB colors
			bool success = false;
			if (cloud->hasDisplayedScalarField())
			{
				success = cloud->convertCurrentScalarFieldToColors();
			}
			//or we simply set white colors
			else
			{
				success = cloud->setColor(ccColor::white);
			}
			if (!success)
			{
				ccLog::Error("Not enough memory");
				return false;
			}
		}
		//at this point we should always have colors!
		assert(cloud->hasColors());
		//but we want them to be grey scaled
		cloud->convertRGBToGreyScale();

		//force visibility and other parameters
		cloud->setEnabled(true);
		cloud->setVisible(true);
		cloud->setSelected(false);
		cloud->showColors(true);
		cloud->showSF(false);
		
		//from now on, the cloud will be automatically deleted if it's 'owned' by the dialog
		m_cloud.ownCloud = ownCloud;

		m_glWindow->addToOwnDB(m_cloud.ref); //warning: will change the display!

		if (m_cloud.originDisplay)
		{
			//apply the same viewport as the original display
			ccViewportParameters viewport = m_cloud.originDisplay->getViewportParameters();
			m_glWindow->setViewportParameters(viewport);

			//by the way, update some parameters of the 3D view (in case they have changed since last call)
			//const ccGui::ParamStruct& displayParams = ccGui::Parameters();
			//m_glWindow->setDisplayParameters(displayParams);

			//import GL filter so as to get the same rendering aspect!
			ccGlFilter* filter = static_cast<ccGLWindow*>(m_cloud.originDisplay)->getGlFilter();
			if (filter)
			{
				m_glWindow->setGlFilter(filter->clone());
			}
		}
		else
		{
			ccBBox bbox = m_cloud.ref->getOwnBB();
			m_glWindow->updateConstellationCenterAndZoom(&bbox);
		}

		if (!m_initialCloud && !ownCloud)
		{
			m_initialCloud = cloud;
		}
	}

	if (autoRedraw)
	{
		m_glWindow->redraw();
	}

	return true;
}

void qBroomDlg::onSelectionModeChanged(int mode)
{
	SelectionModes formerMode = m_selectionMode;
	m_selectionMode = static_cast<SelectionModes>(selectionModeComboBox->currentIndex());
	m_selectionBox->setEnabled(m_selectionMode != INSIDE);
	cleanHeightDoubleSpinBox->setEnabled(m_selectionMode != INSIDE);
	if (m_selectionMode != formerMode)
	{
		removeSelectedPointsCheckBox->setChecked(mode != 0); //mode '0' is inside (--> we'll probably want to keep these points)
		if (m_glWindow)
		{
			updateBroomBox();
			updateSelectionBox();
			m_glWindow->redraw();
		}
	}
}

void qBroomDlg::updateAutomationAreaPolyline(int x, int y)
{
	if (!m_glWindow)
	{
		assert(false);
		return;
	}

	if (!m_broomBox || !m_boxes->isEnabled())
	{
		return;
	}
	
	if (m_autoArea.clickedPoints.empty() || m_autoArea.clickedPoints.size() > 2)
	{
		assert(false);
		return;
	}

	//current 3D view parameters
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);

	ccPointCloud* vertices = nullptr;
	if (!m_autoArea.polyline)
	{
		vertices = new ccPointCloud();
		m_autoArea.polyline = new ccPolyline(vertices);
		m_autoArea.polyline->addChild(vertices);
		vertices->setVisible(false);
		if (!vertices->reserve(4) || !m_autoArea.polyline->reserve(4))
		{
			//not enough memory
			delete m_autoArea.polyline;
			m_autoArea.polyline = nullptr;
			return;
		}

		//add 4 fake points (will be updated with the new clicked point
		vertices->addPoint(CCVector3(0, 0, 0));
		vertices->addPoint(CCVector3(0, 0, 0));
		vertices->addPoint(CCVector3(0, 0, 0));
		vertices->addPoint(CCVector3(0, 0, 0));
		m_autoArea.polyline->addPointIndex(0, 2); //we only display a segment at first
		m_autoArea.polyline->setEnabled(true);
		m_autoArea.polyline->setVisible(true);
		m_autoArea.polyline->setWidth(2);
		m_autoArea.polyline->setColor(ccColor::red);
		m_autoArea.polyline->showColors(true);
		m_autoArea.polyline->set2DMode(true);
		m_autoArea.polyline->setClosed(true);
		m_glWindow->addToOwnDB(m_autoArea.polyline);
	}
	else
	{
		vertices = dynamic_cast<ccPointCloud*>(m_autoArea.polyline->getAssociatedCloud());
	}
	
	if (!vertices)
	{
		assert(false);
		return;
	}

	//always reproject the first point (if we move the camera)
	//same thing for the second point (if any)
	for (size_t i=0; i<std::min<size_t>(m_autoArea.clickedPoints.size(), 2); ++i)
	{
		const CCVector3& P03D = m_autoArea.clickedPoints[i];
		CCVector3d C02D;
		camera.project(P03D, C02D);
		C02D.x -= camera.viewport[2] / 2.0;
		C02D.y -= camera.viewport[3] / 2.0;
		*const_cast<CCVector3*>(vertices->getPoint(static_cast<unsigned>(i))) = CCVector3::fromArray(C02D.u);
	}

	//broom position/orientation
	const ccGLMatrix& broomTrans = m_boxes->getGLTransformation();

	//project the current mouse position in 3D AND on the broom plane
	CCVector3 P3D;
	{
		CCVector3d M03D;
		CCVector3d M13D;
		QPointF pos2D = m_glWindow->toCornerGLCoordinates(x, y);
		camera.unproject(CCVector3(pos2D.x(), pos2D.y(), 0), M03D);
		camera.unproject(CCVector3(pos2D.x(), pos2D.y(), 1), M13D);
 		if (!Intersection(broomTrans, CCVector3::fromArray(M03D.u), CCVector3::fromArray(M13D.u), P3D))
		{
			return;
		}
	}

	if (m_autoArea.clickedPoints.size() == 1)
	{
		//simply update the second vertex position
		CCVector3d C12D;
		camera.project(P3D, C12D);
		C12D.x -= camera.viewport[2] / 2.0;
		C12D.y -= camera.viewport[3] / 2.0;
		*const_cast<CCVector3*>(vertices->getPoint(1)) = CCVector3::fromArray(C12D.u);
	}
	else if (m_autoArea.clickedPoints.size() == 2)
	{
		//update the 3rd and 4th vertices
		const CCVector3& P03D = m_autoArea.clickedPoints[0];
		const CCVector3& P13D = m_autoArea.clickedPoints[1];

		CCVector3 Xp = P13D-P03D;
		CCVector3 Zp = broomTrans.getColumnAsVec3D(2);
		CCVector3 Yp = Zp.cross(Xp); //should be already normal to Zp and Xp as P03D and P13D are on the broom plane!
		Yp.normalize();

		//distance from the new point to (Xp)
		PointCoordinateType dist = (P3D-P03D).dot(Yp);

		CCVector3d C2D;
		camera.project(P13D + dist * Yp, C2D);
		C2D.x -= camera.viewport[2] / 2.0;
		C2D.y -= camera.viewport[3] / 2.0;
		*const_cast<CCVector3*>(vertices->getPoint(2)) = CCVector3::fromArray(C2D.u);

		CCVector3d D2D;
		camera.project(P03D + dist * Yp, D2D);
		D2D.x -= camera.viewport[2] / 2.0;
		D2D.y -= camera.viewport[3] / 2.0;
		*const_cast<CCVector3*>(vertices->getPoint(3)) = CCVector3::fromArray(D2D.u);

		if (m_autoArea.polyline->size() == 2)
		{
			m_autoArea.polyline->addPointIndex(2, 4); //we now display a rectangle
		}
	}

	vertices->invalidateBoundingBox();

	m_glWindow->redraw(true);
}

void qBroomDlg::getBroomDimensions(BroomDimensions& dimensions) const
{
	//now we can start the automation process
	dimensions.length = static_cast<PointCoordinateType>(broomLengthDoubleSpinBox->value());
	dimensions.width = static_cast<PointCoordinateType>(broomWidthDoubleSpinBox->value() / 100.0) * dimensions.length;
	dimensions.thick = static_cast<PointCoordinateType>(broomThicknessDoubleSpinBox->value() / 100.0) * dimensions.length;
	dimensions.height = static_cast<PointCoordinateType>(cleanHeightDoubleSpinBox->value());
}

bool qBroomDlg::startAutomation()
{
	if (!m_broomBox || !m_boxes->isEnabled())
	{
		assert(false);
		return false;
	}

	if (m_autoArea.clickedPoints.size() != 3)
	{
		assert(false);
		return false;
	}

	CCVector3 P0 = m_autoArea.clickedPoints[0];
	CCVector3 P1 = m_autoArea.clickedPoints[1];
	CCVector3 P2 = m_autoArea.clickedPoints[2];
	
	//current broom orientation
	ccGLMatrix initialBroomTrans = m_boxes->getGLTransformation();
	ccGLMatrix originalBroomTrans = initialBroomTrans;

	//compute the min and max corners of the automation area
	CCVector3 origin = initialBroomTrans.getTranslationAsVec3D();
	CCVector3 Xo = initialBroomTrans.getColumnAsVec3D(0);
	CCVector3 Yo = initialBroomTrans.getColumnAsVec3D(1);
	CCVector3 Zo = initialBroomTrans.getColumnAsVec3D(2);

	//automation are
	// A --------------- B
	// |                 |
	// |                 |
	// D --------------- C
	CCVector3 AB = P1 - P0;
	CCVector3 BC = P2 - P1;
	CCVector3 X = AB;
	CCVector3 Y = Zo.cross(X);
	X.normalize();
	Y.normalize();

	//compute the opposite corner (to P0)
	P1 += BC.dot(Y) * Y;

	//we make the broom orientation match with the most similar direction
	if (fabs(X.dot(Xo)) < fabs(X.dot(Yo)))
	{
		CCVector3 temp = X;
		X = Y;
		Y = -temp;
	}

	//update the broom orientation to match the automation area!
	initialBroomTrans.setColumn(0, X);
	initialBroomTrans.setColumn(1, Y);

	CCVector3 BB0( (P0 - origin).dot(X), (P0 - origin).dot(Y), 0);
	CCVector3 BB1( (P1 - origin).dot(X), (P1 - origin).dot(Y), 0);

	if (BB0.x > BB1.x)
	{
		std::swap(BB0.x, BB1.x);
		std::swap(P0.x, P1.x);
	}
	if (BB0.y > BB1.y)
	{
		std::swap(BB0.y, BB1.y);
		std::swap(P0.y, P1.y);
	}

	if (	(BB0.x > 0 && BB0.y > 0)
		||	(BB1.x < 0 && BB1.y < 0) )
	{
		QMessageBox::critical(this, "Misplaced area", "The broom is not inside the automation area");
		return false;
	}

	CCVector3 diagonal = BB1 - BB0;

	//debug: show the automation area
	ccPlane* plane = nullptr;
	if (false)
	{
		CCVector3 center = (BB1 + BB0) / 2;
		ccGLMatrix planeTrans = initialBroomTrans;
		planeTrans.setTranslation(initialBroomTrans.getTranslationAsVec3D() + center.x * X + center.y * Y);
		ccPlane* plane = new ccPlane(diagonal.x, diagonal.y, &planeTrans, "automation area");
		m_glWindow->addToOwnDB(plane, false);
		m_glWindow->redraw();
	}

	//now we can start the automation process
	BroomDimensions broom;
	getBroomDimensions(broom);

	//we reduce the broom size if necessary 
	if (broom.length > diagonal.x)
	{
		broom.length = diagonal.x;
	}
	if (broom.width > diagonal.y)
	{
		broom.width = diagonal.y;
	}

	ccGLMatrix broomTrans = initialBroomTrans;
	CCVector3d Xd = CCVector3d::fromArray(X.u);
	CCVector3d Yd = CCVector3d::fromArray(Y.u);

	bool stickToTheFloor = stickCheckBox->isChecked();
	bool animateAutomation = animateAutomationCheckBox->isChecked();

	QProgressDialog progressDialog("Automation in progress...", "Cancel", 0, 0, this);
	progressDialog.show();

	//overlap between each position
	double overlapRatio = 0.9;

	double sign = -1.0;
	bool lostTrack = false;
	while (true)
	{
		//detect if this row is the last one (for the current scan direction)
		bool lastRow = false;
		{
			CCVector3 broomCenter = broomTrans.getTranslationAsVec3D() - origin;
			if (sign < 0.0 && broomCenter.dot(X) <= BB0.x + broom.length / 2 )
			{
				lastRow = true;
				broomCenter = origin + (BB0.x + broom.length / 2) * X + broomCenter.dot(Y) * Y;
				broomTrans.setTranslation(broomCenter);
			}
			else if (sign > 0.0 && broomCenter.dot(X) >= BB1.x - broom.length / 2)
			{
				lastRow = true;
				broomCenter = origin + (BB1.x - broom.length / 2) * X + broomCenter.dot(Y) * Y;
				broomTrans.setTranslation(broomCenter);
			}
		}
		
		//get the points at the current position
		selectPoints(broomTrans, &broom);
		if (animateAutomation)
		{
			m_boxes->setGLTransformation(broomTrans);
			m_glWindow->redraw(false);
		}

		bool cancelled = false;

		//test backward
		{
			ccGLMatrix backwardBroomTrans = broomTrans;
			bool lastColumn = false;
			while (!lastColumn)
			{
				double broomDeltaY = -(broom.width * overlapRatio);

				//clip inside automation area
				{
					CCVector3 broomCenter = backwardBroomTrans.getTranslationAsVec3D() - origin;
					if (broomCenter.dot(Y) - broom.width / 2 + broomDeltaY <= BB0.y)
					{
						lastColumn = true;
						broomDeltaY = BB0.y - broomCenter.dot(Y) + broom.width / 2;
					}
				}

				CCVector3d broomDelta = broomDeltaY * Yd;

				if (	!moveBroom(backwardBroomTrans, broomDelta, stickToTheFloor)
					||	broomDelta.norm() < broom.width / 20.0)
				{
					//hole or too steep slope? we stop
					lostTrack = true;
					break;
				}

				selectPoints(backwardBroomTrans, &broom);
				if (animateAutomation)
				{
					m_boxes->setGLTransformation(backwardBroomTrans);
					m_glWindow->redraw(false);
				}

				QCoreApplication::processEvents();
				if (progressDialog.wasCanceled())
				{
					cancelled = true;
					break;
				}
			}
		}

		if (cancelled)
		{
			ccLog::Warning("Automation process cancelled by the user");
			break;
		}
		
		//test forward
		{
			ccGLMatrix forwardBroomTrans = broomTrans;
			bool lastColumn = false;
			while (!lastColumn)
			{
				double broomDeltaY = (broom.width * overlapRatio);

				//clip inside automation area
				{
					CCVector3 broomCenter = forwardBroomTrans.getTranslationAsVec3D() - origin;
					if (broomCenter.dot(Y) + broom.width / 2 + broomDeltaY >= BB1.y)
					{
						lastColumn = true;
						broomDeltaY = BB1.y - broomCenter.dot(Y) - broom.width / 2;
					}
				}

				CCVector3d broomDelta = broomDeltaY * Yd;

				if (	!moveBroom(forwardBroomTrans, broomDelta, stickToTheFloor)
					||	broomDelta.norm() < broom.width / 20.0)
				{
					//hole or too steep slope? we stop
					lostTrack = true;
					break;
				}

				selectPoints(forwardBroomTrans, &broom);
				if (animateAutomation)
				{
					m_boxes->setGLTransformation(forwardBroomTrans);
					m_glWindow->redraw(false);
				}

				QCoreApplication::processEvents();
				if (progressDialog.wasCanceled())
				{
					cancelled = true;
					break;
				}
			}
		}

		if (cancelled)
		{
			ccLog::Warning("Automation process cancelled by the user");
			break;
		}
		
		//next row
		{
			ccGLMatrix nextRowBroomTrans = broomTrans;
			if (lastRow)
			{
				if (sign < 0.0)
				{
					//change the sign
					sign = 1.0;
					//reset the current position
					nextRowBroomTrans = initialBroomTrans;
				}
				else
				{
					//we can stop
					break;
				}
			}

			//try to shift the row
			CCVector3d broomDelta = (sign * broom.length * overlapRatio) * Xd;
			if (	!moveBroom(nextRowBroomTrans, broomDelta, stickToTheFloor)
				||	broomDelta.norm() < broom.length / 20.0)
			{
				//hole or too steep slope?
				//we stop
				lostTrack = true;
			}
			broomTrans = nextRowBroomTrans;
		}
	}

	if (lostTrack)
	{
		ccLog::Warning("Holes or too steep slope encountered during automation process");
	}

	//restore original state
	m_boxes->setGLTransformation(originalBroomTrans);
	m_glWindow->redraw();

	return true;

}

bool qBroomDlg::positionBroom(const CCVector3& P0, const CCVector3& P1)
{
	CCVector3 X = P1-P0;
	PointCoordinateType length = X.norm();
	
	//update UI
	{
		broomLengthDoubleSpinBox->blockSignals(true);
		broomLengthDoubleSpinBox->setValue(length);
		broomLengthDoubleSpinBox->setSingleStep(length/10.0); //10 percent
		broomLengthDoubleSpinBox->blockSignals(false);

		cleanHeightDoubleSpinBox->blockSignals(true);
		cleanHeightDoubleSpinBox->setValue(length);
		cleanHeightDoubleSpinBox->setSingleStep(length/10.0); //10 percent
		cleanHeightDoubleSpinBox->blockSignals(false);
	}

	//PointCoordinateType width = static_cast<PointCoordinateType>(broomWidthDoubleSpinBox->value() / 100.0) * length;
	//PointCoordinateType thick = static_cast<PointCoordinateType>(broomThicknessDoubleSpinBox->value() / 100.0) * length;

	//we need to probe the cloud to get the right vertical orientation
	ccOctree::Shared octree = m_cloud.ref->getOctree();
	if (!octree)
	{
		assert(false);
		return false;
	}

#define USE_CYLINDRICAL_NEIGHBORHOOD
#ifdef USE_CYLINDRICAL_NEIGHBORHOOD

	CCLib::DgmOctree::CylindricalNeighbourhood nn;
	nn.center = (P0 + P1) / 2;
	nn.dir = X;
	nn.dir.normalize();
	nn.radius = length * 0.05; //5% of the broom length?
	nn.level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(nn.radius);
	nn.maxHalfLength = length / 2;
	nn.onlyPositiveDir = false;

	size_t count = octree->getPointsInCylindricalNeighbourhood(nn);

#else

	CCLib::DgmOctree::BoxNeighbourhood nn;
	{
		BroomDimensions broom;
		getBroomDimensions(broom);
		nn.dimensions = CCVector3(broom.length, broom.width, broom.thick);
		nn.center = (P0 + P1) / 2;
		nn.axes = new CCVector3[3];
		{
			nn.axes[0] = X;
			nn.axes[0].normalize();

			//generate an arbitrary coordinate system
			CCVector3 Z(0, 0, 1);
			if (Z.dot(nn.axes[0]) < 0.1)
			{
				Z = CCVector3(1, 0, 0);
			}
			CCVector3 Y = Z.cross(X);
			Y.normalize();
			Z = X.cross(Y);

			nn.axes[1] = Y;
			nn.axes[2] = Z;
		}
		nn.level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(std::max(broom.length/5, std::max(broom.width, broom.thick)));
	}

	size_t count = octree->getPointsInBoxNeighbourhood(nn);

	delete[] nn.axes;
	nn.axes = 0;

#endif

	if (count < 3)
	{
		ccLog::Error("Not enough points between the two picked points");
		return false;
	}

	CCLib::DgmOctreeReferenceCloud neighboursCloud(&nn.neighbours);

	CCLib::Neighbourhood n(&neighboursCloud);
	const CCVector3* N = n.getLSPlaneNormal();
	if (!N)
	{
		ccLog::Error("Failed to detect a planar surface below the two points");
		return false;
	}

	//extract the local coordinate system basis
	CCVector3 Z = *N;
	Z.normalize();
	CCVector3 Y = Z.cross(X);
	Y.normalize();
	X = Y.cross(Z);

	ccGLMatrix trans(X, Y, Z, nn.center);
	m_boxes->setGLTransformation(trans);

#if 0
	{
		//new selection
		addUndoStep(cn.center);
		for (size_t i=0; i<count; ++i)
		{
			selectPoint(cn.neighbours[i].pointIndex);
		}
	}
#endif

	//show the broom
	m_boxes->setEnabled(true);
	onDimensionChanged(0.0); //to update the display
	//m_glWindow->redraw();

	//now we wait for the user to click on the broom
	m_glWindow->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA() | ccGLWindow::INTERACT_SIG_LB_CLICKED);
		
	return true;
}

void qBroomDlg::handlePickedItem(ccHObject* entity, unsigned itemIdx, int x, int y, const CCVector3&, const CCVector3d&)
{
	assert(m_glWindow && m_glWindow->getPickingMode() == ccGLWindow::POINT_PICKING);

	if (m_picking.mode != Picking::BROOM_PICKING)
	{
		assert(false);
		return;
	}

	if (!m_cloud.ref)
	{
		assert(false);
		return;
	}

	if (entity != m_cloud.ref)
	{
		//could be the broom!
		return;
	}

	assert(m_picking.labels.size() < 2);
	cc2DLabel* label = m_picking.addLabel(m_cloud.ref, itemIdx);
	if (!label)
	{
		return;
	}

	label->setVisible(true);
	label->setEnabled(true);
	m_glWindow->addToOwnDB(label);
	m_glWindow->redraw();

	if (m_picking.labels.size() == 2)
	{
		const cc2DLabel::PickedPoint& L0 = m_picking.labels[0]->getPickedPoint(0);
		const cc2DLabel::PickedPoint& L1 = m_picking.labels[1]->getPickedPoint(0);

		CCVector3 P0 = L0.getPointPosition();
		CCVector3 P1 = L1.getPointPosition();

		switch (m_picking.mode)
		{
		case Picking::BROOM_PICKING:
			{
				//extract the points around the broom
				bool success = positionBroom(P0, P1);
				automatePushButton->setEnabled(success);
				stopBroomPicking();
			}
			break;

		default:
			assert(false);
		}
	}
}

void qBroomDlg::stopBroomPicking()
{
	freezeUI(!m_boxes->isEnabled()); //we only unfreeze the UI if the broom is valid (enabled)
	
	repositionPushButton->setText("Reposition");
	m_glWindow->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
	m_picking.mode = Picking::NO_PICKING;
	m_picking.clear();
	m_glWindow->redraw();
	undoPushButton->setFocus();
}

void qBroomDlg::onReposition()
{
	//if we click on the 'Reposition' button while we are
	//already 'repositioning', then it means we cancel it!
	if (m_picking.mode == Picking::BROOM_PICKING)
	{
		stopBroomPicking();
		assert(!automatePushButton->isEnabled());
		return;
	}

	//enable picking
	repositionPushButton->setText("Cancel");
	automatePushButton->setEnabled(false);
	m_glWindow->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
	m_glWindow->setPickingMode(ccGLWindow::POINT_PICKING);
	m_picking.mode = Picking::BROOM_PICKING;

	//hide the broom
	lostTrackFrame->setVisible(false);
	m_boxes->setEnabled(false);
	
	freezeUI(true);

	//display message
	m_glWindow->displayNewMessage("Pick two points on the cloud", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::CUSTOM_MESSAGE);

	m_glWindow->redraw();
}

void qBroomDlg::stopAutomation()
{
	freezeUI(false); //we only unfreeze the UI if the broom is valid (enabled)
	
	automatePushButton->setText("Automate");
	m_glWindow->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	//m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
	m_glWindow->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA() | ccGLWindow::INTERACT_SIG_LB_CLICKED); //Standard interaction for manual broom displacement!
	m_picking.mode = Picking::NO_PICKING;
	//m_picking.clear();
	m_autoArea.clear();
	m_glWindow->redraw();
}

void qBroomDlg::onAutomate()
{
	//if we click on the 'Automate' button while we are
	//already picking points for it, then it means we cancel it!
	if (m_picking.mode == Picking::AUTO_AREA_PICKING)
	{
		stopAutomation();
		return;
	}

	//enable picking
	automatePushButton->setText("Cancel");
	m_glWindow->setInteractionMode(ccGLWindow::PAN_ONLY() | ccGLWindow::INTERACT_SIG_LB_CLICKED);
	//m_glWindow->setPickingMode(ccGLWindow::POINT_PICKING); //We don't actually 'pick' an existing point!
	m_picking.mode = Picking::AUTO_AREA_PICKING;
	m_autoArea.clear(); //just in case

	freezeUI(true);

	//display message
	m_glWindow->displayNewMessage("Pick two corners of the area to clean", ccGLWindow::UPPER_CENTER_MESSAGE, false, 3600, ccGLWindow::CUSTOM_MESSAGE);

	m_glWindow->redraw();
}

void qBroomDlg::onLeftButtonClicked(int x, int y)
{
	if (!m_glWindow)
	{
		assert(false);
		return;
	}
	if (!m_broomBox || !m_boxes->isEnabled())
	{
		return;
	}

	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);

	//Definition of the automation area
	if (m_picking.mode == Picking::AUTO_AREA_PICKING)
	{
		//project the currrent mouse position in 3D AND on the broom plane
		CCVector3 P3D;
		{
			ccGLMatrix broomTrans = m_boxes->getGLTransformation();
			CCVector3d M03D;
			CCVector3d M13D;
			QPointF pos2D = m_glWindow->toCornerGLCoordinates(x, y);
			camera.unproject(CCVector3(pos2D.x(), pos2D.y(), 0), M03D);
			camera.unproject(CCVector3(pos2D.x(), pos2D.y(), 1), M13D);
			if (!Intersection(broomTrans, CCVector3::fromArray(M03D.u), CCVector3::fromArray(M13D.u), P3D))
			{
				ccLog::Warning("Failed to project the clicked point on the bromm plane");
				return;
			}

			m_autoArea.clickedPoints.reserve(3);
			m_autoArea.clickedPoints.push_back(P3D);
		}

		switch (m_autoArea.clickedPoints.size())
		{
		case 1:
			//we switch to mouse tracking in order to display the current area polyline
			m_glWindow->setInteractionMode( m_glWindow->getInteractionMode() | ccGLWindow::INTERACT_SIG_MOUSE_MOVED );
			break;
		case 2:
			//nothing to do
			break;
		case 3:
			startAutomation();
			stopAutomation();
			break;
		default:
			assert(false);
			return;
		}
	}
	else //selection of the broom
	{
		m_lastMousePos = QPoint(x, y);
		m_broomSelected = false;
		{
			int nearestTriIndex = -1;
			double nearestSquareDist = 0;
			CCVector3d P;
			QPointF pos2D = m_glWindow->toCornerGLCoordinates(x, y);
			m_broomSelected = m_broomBox->trianglePicking(	CCVector2d(pos2D.x(), pos2D.y()),
															camera,
															nearestTriIndex,
															nearestSquareDist,
															P);
		}

		m_hasLastMousePos3D = false;
		if (m_broomSelected)
		{
			//compute the click position in 3D
			if (stickCheckBox->isChecked())
			{
				//compute click direction in 3D
				QPointF pos2D = m_glWindow->toCornerGLCoordinates(m_lastMousePos.x(), m_lastMousePos.y());
				CCVector3d A2D(pos2D.x(), pos2D.y(), 0);
				CCVector3d B2D(pos2D.x(), pos2D.y(), 1);
				CCVector3d A3D;
				CCVector3d B3D;
				if (	camera.unproject(A2D, A3D)
					&&	camera.unproject(B2D, B3D) )
				{
					m_hasLastMousePos3D = Intersection(m_boxes->getGLTransformation(), CCVector3::fromArray(A3D.u), CCVector3::fromArray(B3D.u), m_lastMousePos3D);

					//test
					if (false && m_hasLastMousePos3D)
					{
						ccPointCloud* fakeCloud = new ccPointCloud;
						fakeCloud->reserve(1);
						fakeCloud->addPoint(m_lastMousePos3D);
						cc2DLabel* la = new cc2DLabel;
						la->addPickedPoint(fakeCloud, 0);
						la->setDisplayedIn2D(false);
						la->setDisplay(m_glWindow);
						la->setVisible(true);
						la->setEnabled(true);
						fakeCloud->addChild(la);
						m_glWindow->addToOwnDB(fakeCloud, false);
					}
				}
			}

			m_glWindow->setInteractionMode(	ccGLWindow::INTERACT_PAN
										|	ccGLWindow::INTERACT_ZOOM_CAMERA
										|	ccGLWindow::INTERACT_SIG_MOUSE_MOVED
										|	ccGLWindow::INTERACT_SIG_BUTTON_RELEASED);
			m_broomBox->setTempColor(ccColor::red);
			m_glWindow->redraw();
		}
	}
}

void qBroomDlg::onMouseMoved(int x, int y, Qt::MouseButtons button)
{
	if (!m_broomBox || !m_boxes->isEnabled())
	{
		return;
	}
	if (!m_glWindow)
	{
		assert(false);
		return;
	}
	
	//normally the broom must be selected and the left mouse button pressed
	if (!m_broomSelected || button != Qt::LeftButton)
	{
		if (m_picking.mode == Picking::AUTO_AREA_PICKING)
		{
			updateAutomationAreaPolyline(x, y);
		}
		return;
	}

	QPoint mousePos(x, y);
	{
		bool stickToTheFloor = stickCheckBox->isChecked();
		bool hasAlreadyLostTrack = lostTrackFrame->isVisible();
		ccGLMatrix broomTrans = m_boxes->getGLTransformation();

		//compute broom translation
		CCVector3d broomDelta(0,0,0);
		{
			m_hasLastMousePos3D = true;
			m_lastMousePos3D = broomTrans.getTranslationAsVec3D();

			//compute translation in 3D
			if (!stickToTheFloor || !m_hasLastMousePos3D)
			{
				double pixSize = m_glWindow->computeActualPixelSize();
				QPoint d = mousePos - m_lastMousePos;
				CCVector3d u(d.x()*pixSize, -d.y()*pixSize, 0);
				m_glWindow->getViewportParameters().viewMat.transposed().applyRotation(u);
				broomDelta = u;
			}

			if (stickToTheFloor)
			{
				//compute the click position in 3D
				ccGLCameraParameters camera;
				m_glWindow->getGLCameraParameters(camera);

				//compute click direction in 3D
				QPointF pos2D = m_glWindow->toCornerGLCoordinates(m_lastMousePos.x(), m_lastMousePos.y());
				CCVector3d A2D(pos2D.x(), pos2D.y(), 0);
				CCVector3d B2D(pos2D.x(), pos2D.y(), 1);
				CCVector3d A3D;
				CCVector3d B3D;

				bool hasMousePos3D = false;
				CCVector3 mousePos3D;
				
				if (	camera.unproject(A2D, A3D)
					&&	camera.unproject(B2D, B3D) )
				{
					hasMousePos3D = Intersection(m_boxes->getGLTransformation(), CCVector3::fromArray(A3D.u), CCVector3::fromArray(B3D.u), mousePos3D);
				}

				if (m_hasLastMousePos3D && hasMousePos3D)
				{

					broomDelta = CCVector3d::fromArray((mousePos3D - m_lastMousePos3D).u);
				}
				else
				{
					//we can't bound the displacement to the floor "porperly"
					//but we can remove the normal component (the broom stays on the floor)
					CCVector3d Zbox = CCVector3d::fromArray(broomTrans.getColumn(2));
					broomDelta -= (broomDelta.dot(Zbox) * Zbox); //filter the Z component
				}

				m_hasLastMousePos3D = hasMousePos3D;
				m_lastMousePos3D = mousePos3D;
			}
		}

		bool lostTrack = !moveBroom(broomTrans, broomDelta, stickToTheFloor);
		if (!lostTrack)
		{
			m_boxes->setGLTransformation(broomTrans);
			selectPoints(broomTrans);
			if (hasAlreadyLostTrack)
			{
				lostTrackFrame->setVisible(false);
			}
		}
		else if (!hasAlreadyLostTrack)
		{
			lostTrackFrame->setVisible(true);
			//stickCheckBox->setChecked(false);

			//auto-deselect the cloud
			m_broomBox->setTempColor(ccColor::yellow);
			m_broomSelected = false;
		}

		m_glWindow->redraw();
	}

	m_lastMousePos = mousePos;
}

bool qBroomDlg::moveBroom(ccGLMatrix& broomTrans, CCVector3d& broomDelta, bool stickToTheFloor) const
{
	//we will need the octree (intensively ;)
	ccOctree::Shared octree = m_cloud.ref ? m_cloud.ref->getOctree() : ccOctree::Shared(nullptr);
	if (!octree)
	{
		assert(false);
		return false;
	}
	
	//broom dimensions
	BroomDimensions broom;
	getBroomDimensions(broom);

	if (stickToTheFloor)
	{
		//make sure the broom doesn't go too far too fast
		CCVector3d Xbox = CCVector3d::fromArray(broomTrans.getColumn(0));
		CCVector3d Ybox = CCVector3d::fromArray(broomTrans.getColumn(1));
		double dx = broomDelta.dot(Xbox);
		if (dx < -broom.length)
			dx = -broom.length;
		else if (dx > broom.length)
			dx = broom.length;

		double dy = broomDelta.dot(Ybox);
		if (dy < -broom.width)
			dy = -broom.width;
		else if (dy > broom.width)
			dy = broom.width;

		broomDelta = dx * Xbox + dy * Ybox;
	}

	//translate the broom
	broomTrans.setTranslation(broomTrans.getTranslationAsVec3D() + CCVector3::fromArray(broomDelta.u));

	if (stickToTheFloor)
	{
		//extract the points inside the broom
		CCLib::DgmOctree::BoxNeighbourhood bn;
		{
			bn.dimensions = CCVector3(broom.length, broom.width, broom.thick);

			bn.center = broomTrans.getTranslationAsVec3D();
			bn.axes = new CCVector3[3];
			{
				bn.axes[0] = broomTrans.getColumnAsVec3D(0);
				bn.axes[1] = broomTrans.getColumnAsVec3D(1);
				bn.axes[2] = broomTrans.getColumnAsVec3D(2);
			}
			bn.level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(std::max(broom.length/5, std::max(broom.width, broom.thick)));
		}

		size_t count = octree->getPointsInBoxNeighbourhood(bn);

		delete[] bn.axes;
		bn.axes = nullptr;

		//try to fit the box to the extracted points
		if (count < 10)
		{
			ccLog::Warning("Failed to extract enough point inside the broom. Lost track.");
			return false;
		}
		else
		{
			CCLib::DgmOctreeReferenceCloud neighboursCloud(&bn.neighbours);
			CCLib::Neighbourhood n(&neighboursCloud);
			const CCVector3* N = n.getLSPlaneNormal();
			if (N)
			{
				//extract the local coordinate system basis
				CCVector3 X = broomTrans.getColumnAsVec3D(0);
				CCVector3 Z = *N;
				Z.normalize();
				CCVector3 Y = Z.cross(X);
				Y.normalize();
				X = Y.cross(Z);

				CCVector3 O = bn.center; //the original center
				O.z = n.getGravityCenter()->z;

				broomTrans = ccGLMatrix(X, Y, Z, O);
			}
			else
			{
				ccLog::Error("Failed to detect a planar surface below the broom. Lost track.");
				return false;
			}
		}
	}

	return true;
}

bool qBroomDlg::selectPoints(const ccGLMatrix& broomTrans, BroomDimensions* _broom/*=0*/)
{
	//we will need the octree (intensively ;)
	ccOctree::Shared octree = m_cloud.ref ? m_cloud.ref->getOctree() : ccOctree::Shared(nullptr);
	if (!octree)
	{
		assert(false);
		return false;
	}

	CCVector3 broomCenter = broomTrans.getTranslationAsVec3D();
	CCVector3 broomNormal = broomTrans.getColumnAsVec3D(2);

	//broom dimensions
	BroomDimensions broom;
	if (_broom)
	{
		broom = *_broom;
	}
	else
	{
		getBroomDimensions(broom);
	}

	//extract the points inside the selection area
	CCLib::DgmOctree::BoxNeighbourhood bn;
	{
		CCVector3 centerShift(0, 0, 0);

		switch (m_selectionMode)
		{
		case INSIDE:
			bn.dimensions = CCVector3(broom.length, broom.width, broom.thick);
			break;

		case ABOVE:
		case ABOVE_AND_BELOW: //we start by ABOVE and we'll treat BELOW later
			bn.dimensions = CCVector3(broom.length, broom.width, broom.height);
			centerShift = ((broom.thick + broom.height) / 2) * broomNormal;
			break;

		case BELOW:
			bn.dimensions = CCVector3(broom.length, broom.width, broom.height);
			centerShift = (-(broom.thick + broom.height) / 2) * broomNormal;
			break;

		default:
			assert(false);
			break;
		}

		bn.center = broomCenter + centerShift;
		bn.axes = new CCVector3[3];
		{
			bn.axes[0] = broomTrans.getColumnAsVec3D(0);
			bn.axes[1] = broomTrans.getColumnAsVec3D(1);
			bn.axes[2] = broomNormal;
		}

		PointCoordinateType radius = std::max(bn.dimensions.x, std::max(bn.dimensions.y, bn.dimensions.z)) / 5; //emprirical ;)
		bn.level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(radius);
	}

	if (m_selectionMode == ABOVE_AND_BELOW)
	{
		//extract the first half
		octree->getPointsInBoxNeighbourhood(bn);
		//and prepare the next one
		bn.center = broomCenter - ((broom.thick + broom.height) / 2) * broomNormal;
	}

	size_t count = octree->getPointsInBoxNeighbourhood(bn);

	delete[] bn.axes;
	bn.axes = nullptr;

	if (count)
	{
		//new selection
		addUndoStep(broomTrans);
		size_t selectedCount = 0;
		for (size_t i=0; i<count; ++i)
		{
			if (selectPoint(bn.neighbours[i].pointIndex))
			{
				++selectedCount;
			}
		}

		//if (selectedCount == 0)
		//{
		//	//no point were selected! auto undo
		//	undo(1);
		//}

		m_cloud.ref->showSF(false); //just in case!
	}

	return true;
}

void qBroomDlg::onButtonReleased()
{
	if (!m_glWindow)
	{
		assert(false);
		return;
	}
		
	m_glWindow->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA() | ccGLWindow::INTERACT_SIG_LB_CLICKED);

	if (m_broomSelected)
	{
		m_broomBox->setTempColor(ccColor::yellow);
		m_broomSelected = false;
		m_glWindow->redraw();
	}
}
	
void qBroomDlg::updateSelectionBox()
{
	if (!m_selectionBox)
	{
		assert(false);
		return;
	}

	PointCoordinateType length = static_cast<PointCoordinateType>(broomLengthDoubleSpinBox->value());
	PointCoordinateType width = static_cast<PointCoordinateType>(broomWidthDoubleSpinBox->value() / 100.0) * length;
	PointCoordinateType thick = static_cast<PointCoordinateType>(broomThicknessDoubleSpinBox->value() / 100.0) * length;

	PointCoordinateType height = static_cast<PointCoordinateType>(cleanHeightDoubleSpinBox->value());

	CCVector3 dims(length, width, 0);
	CCVector3 shift(0,0,0);

	switch (m_selectionMode)
	{
	case INSIDE:
		//no need to do anything (simply hide the box)
		m_selectionBox->setEnabled(false);
		return;

	case ABOVE:
		dims.z = height;
		shift.z = (thick + height)/2;
		break;

	case BELOW:
		dims.z = height;
		shift.z = -(thick + height)/2;
		break;

	case ABOVE_AND_BELOW:
		dims.z = 2*height + thick;
		shift.z = 0;
		break;

	default:
		assert(false);
		break;
	}

	m_selectionBox->enableStippling(true);
	m_selectionBox->setEnabled(true);
	m_selectionBox->setDimensions(dims);
	ccGLMatrix trans;
	trans.setTranslation(shift);
	m_selectionBox->setTempColor(ccColor::red);
	m_selectionBox->setGLTransformation(trans);
}

void qBroomDlg::onCleanHeightChanged(double)
{
	if (m_selectionBox)
	{
		updateSelectionBox();
		if (m_glWindow && m_boxes->isEnabled())
		{
			m_glWindow->redraw();
		}
	}
}

void qBroomDlg::updateBroomBox()
{
	if (!m_broomBox)
	{
		assert(false);
		return;
	}

	PointCoordinateType length = static_cast<PointCoordinateType>(broomLengthDoubleSpinBox->value());
	PointCoordinateType width = static_cast<PointCoordinateType>(broomWidthDoubleSpinBox->value() / 100.0) * length;
	PointCoordinateType thick = static_cast<PointCoordinateType>(broomThicknessDoubleSpinBox->value() / 100.0) * length;

	CCVector3 dims(length, width, thick);
	m_broomBox->setDimensions(dims);

	m_broomBox->enableStippling(m_selectionMode == INSIDE); //the cleaning area box is already stippled!
	m_broomBox->setTempColor(ccColor::yellow);
}

void qBroomDlg::onDimensionChanged(double)
{
	if (m_broomBox)
	{
		updateBroomBox();
		//the selection area depends on the broom dimensions as well!
		updateSelectionBox();

		if (m_glWindow && m_boxes->isEnabled())
		{
			m_glWindow->redraw();
		}
	}
}

bool qBroomDlg::selectPoint(unsigned index)
{
	if (!m_cloud.ref)
	{
		assert(false);
		return false;
	}

	assert(index < m_selectionTable.size());
	if (m_selectionTable[index] != 0)
	{
		//already selected
		return false;
	}

	m_cloud.ref->setPointColor(index, ccColor::red);

	assert(!m_undoPositions.empty());
	m_selectionTable[index] = static_cast<uint32_t>(m_undoPositions.size());

	return true;
}

uint32_t qBroomDlg::addUndoStep(const ccGLMatrix& broomPos)
{
	applyPushButton->setEnabled(true);
	validatePushButton->setEnabled(true);

	//new selection
	try
	{
		m_undoPositions.push_back(broomPos);
		undoPushButton->setEnabled(true);
		undo10PushButton->setEnabled(true);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory (undo will be broken)
	}

	return static_cast<uint32_t>(m_undoPositions.size());
}

void qBroomDlg::undo(uint32_t undoCount)
{
	if (	!m_cloud.ref
		||	m_selectionTable.size() != m_cloud.ref->size())
	{
		assert(false);
		return;
	}

	if (undoCount == 0 || m_undoPositions.empty())
	{
		//nothing to do
		return;
	}

	uint32_t newCursor = static_cast<uint32_t>(m_undoPositions.size());
	ccGLMatrix newPosition;
	if (newCursor <= undoCount)
	{
		newCursor = 0;
		newPosition = m_undoPositions[0];
	}
	else
	{
		newCursor -= undoCount;
		newPosition = m_undoPositions[newCursor];
	}

	for (unsigned i=0; i<m_cloud.ref->size(); ++i)
	{
		if (m_selectionTable[i] > newCursor)
		{
			m_selectionTable[i] = 0;

			//restore the point color
			if (m_cloud.colors)
			{
				m_cloud.ref->setPointColor(i, m_cloud.colors->getValue(i));
			}
			else
			{
				assert(false);
			}
		}
	}

	m_undoPositions.resize(newCursor);
	undoPushButton->setEnabled(newCursor != 0);
	undo10PushButton->setEnabled(newCursor != 0);
	applyPushButton->setEnabled(newCursor != 0);
	validatePushButton->setEnabled(m_cloud.ownCloud || newCursor != 0);

	m_boxes->setGLTransformation(newPosition);

	if (m_glWindow)
	{
		m_glWindow->redraw();
	}
}

void qBroomDlg::cancel()
{
	setCloud(nullptr);
	
	reject();
}

void qBroomDlg::displayError(QString message)
{
	if (m_app)
	{
		m_app->dispToConsole(message, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
	else
	{
		QMessageBox::critical(this, tr("Error"), message);
	}
}

ccPointCloud* qBroomDlg::createSegmentedCloud(ccPointCloud* cloud, bool removeSelected, bool& error)
{
	error = false;

	if (!cloud || m_selectionTable.size() != cloud->size())
	{
		//we shouldn't be here ;)
		assert(false);
		//nothing to do
		return nullptr;
	}

	unsigned selectedCount = 0;
	{
		for (unsigned i=0; i<cloud->size(); ++i)
			if (m_selectionTable[i] != 0)
				++selectedCount;

		if (!removeSelected)
		{
			selectedCount = cloud->size() - selectedCount;
		}
		
		if (selectedCount == 0)
		{
			assert(false);
			//nothing to do
			accept();
			return nullptr;
		}
	}

	CCLib::ReferenceCloud selection(cloud);
	{
		if (!selection.reserve(selectedCount))
		{
			displayError(tr("Not enough memory"));
			error = true;
			return nullptr;
		}

		for (unsigned i=0; i<cloud->size(); ++i)
		{
			if (	( removeSelected && m_selectionTable[i] == 0) //keep non selected
				||	(!removeSelected && m_selectionTable[i] != 0) //keep selected
				)
			{
				selection.addPointIndex(i);
			}
		}
	}

	ccPointCloud* newCloud = cloud->partialClone(&selection);
	if (!newCloud)
	{
		displayError(tr("Not enough memory"));
		error = true;
		return nullptr;
	}

	QString name = cloud->getName();
	if (!name.endsWith(".segmented"))
	{
		name += ".segmented";
	}
	newCloud->setName(name);

	return newCloud;
}

void qBroomDlg::apply()
{
	//save persistent settings
	savePersistentSettings();

	ccViewportParameters formerViewport = m_glWindow->getViewportParameters();
	m_cloud.restore();
	
	bool error;
	ccPointCloud* newCloud = createSegmentedCloud(m_cloud.ref, removeSelectedPointsCheckBox->isChecked(), error);
	if (!newCloud)
	{
		if (error)
		{
			displayError(tr("Not enough memory"));
		}
		ccPointCloud* formerCloud = m_cloud.ref;
		bool ownCloud = m_cloud.ownCloud;
		m_cloud.ownCloud = false; //to prevent if from being deleted!
		
		setCloud(nullptr, false);
		setCloud(formerCloud, ownCloud, false);
	}

	setCloud(newCloud, true, false);

	//restore the previous viewport
	m_glWindow->setViewportParameters(formerViewport);
	m_glWindow->redraw();
}

void qBroomDlg::closeEvent(QCloseEvent* e)
{
	if (!m_undoPositions.empty() || m_cloud.ownCloud)
	{
		if (QMessageBox::warning(this, "Cancel", "The selection/segmentation will be lost. Do you confirm?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
		{
			e->ignore();
			return;
		}
	}

	e->accept();
}

void qBroomDlg::savePersistentSettings()
{
	QSettings settings;
	settings.beginGroup("qBroom");
	settings.setValue("selectionMode", selectionModeComboBox->currentIndex());
	settings.setValue("stickToTheFloor", stickCheckBox->isChecked());
	settings.setValue("animatedAutomation", animateAutomationCheckBox->isChecked());
}

void qBroomDlg::validate()
{
	//save persistent settings
	savePersistentSettings();

	//we don't want the cloud to be managed by the dialog anymore
	ccPointCloud* cloud = m_cloud.ref;
	assert(cloud);
	bool ownCloud = m_cloud.ownCloud;
	m_cloud.ownCloud = false;

	setCloud(nullptr);
	//m_cloud.restore(); //already called by setCloud

	ccPointCloud* newCloud = nullptr;
	if (!m_undoPositions.empty())
	{
		bool error;
		newCloud = createSegmentedCloud(cloud, removeSelectedPointsCheckBox->isChecked(), error);
		if (!newCloud)
		{
			if (ownCloud)
			{
				if (error)
				{
					displayError(tr("Not enough memory to apply the last segmentation"));
				}
				newCloud = cloud;
			}
			else if (error)
			{
				displayError(tr("Not enough memory"));
				reject();
				return;
			}
		}
	}
	else if (ownCloud)
	{
		newCloud = cloud;
	}

	if (newCloud)
	{
		if (newCloud->getDisplay() == m_glWindow)
		{
			assert(false);
			newCloud->setDisplay(nullptr);
		}

		if (m_initialCloud)
		{
			if (m_initialCloud->getParent())
				m_initialCloud->getParent()->addChild(newCloud);
			m_initialCloud->setEnabled(false);
		}
		m_app->addToDB(newCloud);
		m_app->setSelectedInDB(newCloud, true);
	}

	accept();
}
