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

#include <ccPointPairRegistrationDlg.h>

//Local
#include "mainwindow.h"
#include "ccAskThreeDoubleValuesDlg.h"

//common
#include <ccPickingHub.h>

//qCC_gl
#include <ccGLWindowInterface.h>

//qCC_db
#include <ccGenericPointCloud.h>
#include <cc2DLabel.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccSphere.h>

//qCC_io
#include <ccGlobalShiftManager.h>

//CC_FBO
#include <ccGlFilter.h>

//CCCoreLib
#include <RegistrationTools.h>
#include <GeometricalAnalysisTools.h>

//Qt
#include <QMdiSubWindow>
#include <QMessageBox>
#include <QToolButton>
#include <QSettings>

#ifdef CC_USES_EIGEN
//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry> //Umeyama
#endif

//default position of each columns in the to-be-aligned and ref. table widgets
static const int XYZ_COL_INDEX			= 0;
static const int RMS_COL_INDEX			= 6;
static const int DEL_BUTTON_COL_INDEX	= 7;

//minimum number of pairs to let the user click on the align button
static const unsigned MIN_PAIRS_COUNT = 3;

ccPointPairRegistrationDlg::ccPointPairRegistrationDlg(ccPickingHub* pickingHub, ccMainAppInterface* app, QWidget* parent/*=nullptr*/)
	: ccOverlayDialog(parent)
	, m_alignedPoints("to-be-aligned points")
	, m_refPoints("reference points")
	, m_paused(false)
	, m_pickingHub(pickingHub)
	, m_app(app)
{
	assert(m_pickingHub);

	setupUi(this);

	//restore from persistent settings
	{
		QSettings settings;
		settings.beginGroup("PointPairAlign");
		bool pickSpheres    = settings.value("PickSpheres",  useSphereToolButton->isChecked()).toBool();
		double sphereRadius = settings.value("SphereRadius", radiusDoubleSpinBox->value()).toDouble();
		int maxRMS          = settings.value("MaxRMS",       maxRmsSpinBox->value()).toInt();
		bool adjustScale    = settings.value("AdjustScale",  adjustScaleCheckBox->isChecked()).toBool();
		bool autoUpdateZoom = settings.value("AutoUpdateZom",autoZoomCheckBox->isChecked()).toBool();
		settings.endGroup();

		useSphereToolButton->setChecked(pickSpheres);
		radiusDoubleSpinBox->setValue(sphereRadius);
		maxRmsSpinBox->setValue(maxRMS);
		adjustScaleCheckBox->setChecked(adjustScale);
		autoZoomCheckBox->setChecked(autoUpdateZoom);
	}

	connect(showAlignedCheckBox,	&QCheckBox::toggled,	this,	&ccPointPairRegistrationDlg::showAlignedEntities);
	connect(showReferenceCheckBox,	&QCheckBox::toggled,	this,	&ccPointPairRegistrationDlg::showReferenceEntities);

	connect(typeAlignToolButton,	&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::addManualAlignedPoint);
	connect(typeRefToolButton,		&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::addManualRefPoint);

	connect(unstackAlignToolButton,	&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::unstackAligned);
	connect(unstackRefToolButton,	&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::unstackRef);

	connect(alignToolButton,		&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::align);
	connect(resetToolButton,		&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::reset);

	connect(validToolButton,		&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::apply);
	connect(cancelToolButton,		&QToolButton::clicked,	this,	&ccPointPairRegistrationDlg::cancel);

	connect(adjustScaleCheckBox,	&QCheckBox::toggled,	this,	&ccPointPairRegistrationDlg::updateAlignInfo);
	connect(TxCheckBox,				&QCheckBox::toggled,	this,	&ccPointPairRegistrationDlg::updateAlignInfo);
	connect(TyCheckBox,				&QCheckBox::toggled,	this,	&ccPointPairRegistrationDlg::updateAlignInfo);
	connect(TzCheckBox,				&QCheckBox::toggled,	this,	&ccPointPairRegistrationDlg::updateAlignInfo);
	connect(rotComboBox, qOverload<int>(&QComboBox::currentIndexChanged),	this, &ccPointPairRegistrationDlg::updateAlignInfo);

	m_alignedPoints.setEnabled(true);
	m_alignedPoints.setVisible(false);

	m_refPoints.setEnabled(true);
	m_refPoints.setVisible(false);
}

ccPointPairRegistrationDlg::EntityContext::EntityContext(ccHObject* ent)
	: entity(ent)
	, originalDisplay(entity ? entity->getDisplay() : nullptr)
	, wasVisible(entity ? entity->isVisible() : false)
	, wasEnabled(entity ? entity->isEnabled() : false)
	, wasSelected(entity ? entity->isSelected() : false)
{
}

void ccPointPairRegistrationDlg::EntityContext::restore()
{
	if (entity)
	{
		entity->setDisplay(originalDisplay);
		entity->setVisible(wasVisible);
		entity->setEnabled(wasEnabled);
		entity->setSelected(wasSelected);
		if (originalDisplay)
			originalDisplay->redraw();
	}
}

void ccPointPairRegistrationDlg::EntityContexts::fill(const ccHObject::Container& entities)
{
	clear();
	isShifted = false;

	for (ccHObject* entity : entities)
	{
		if (!entity)
		{
			assert(false);
			continue;
		}

		if (!isShifted)
		{
			ccShiftedObject* shiftedEntity = ccHObjectCaster::ToShifted(entity);
			if (shiftedEntity && shiftedEntity->isShifted())
			{
				isShifted = true;
				shift = shiftedEntity->getGlobalShift(); //we can only consider the first shift!
				scale = shiftedEntity->getGlobalScale();
			}
		}

		insert(entity, EntityContext(entity));
	}
}

void ccPointPairRegistrationDlg::clear()
{
	alignToolButton->setEnabled(false);
	validToolButton->setEnabled(false);

	while (alignedPointsTableWidget->rowCount() != 0)
		alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount() - 1);
	while (refPointsTableWidget->rowCount() != 0)
		refPointsTableWidget->removeRow(refPointsTableWidget->rowCount() - 1);

	m_alignedPoints.removeAllChildren();
	m_alignedPoints.resize(0);
	m_alignedPoints.setGlobalShift(0, 0, 0);
	m_alignedPoints.setGlobalScale(1.0);
	m_alignedEntities.clear();
	m_refPoints.removeAllChildren();
	m_refPoints.resize(0);
	m_refPoints.setGlobalShift(0, 0, 0);
	m_refPoints.setGlobalScale(1.0);
	m_referenceEntities.clear();
}

bool ccPointPairRegistrationDlg::linkWith(ccGLWindowInterface* win)
{
	ccGLWindowInterface* oldWin = m_associatedWin;
	if (oldWin)
	{
		if (oldWin != win)
		{
			//TODO FIXME: is it still necessary?
			oldWin->signalEmitter()->disconnect(this);
		}

		oldWin->removeFromOwnDB(&m_alignedPoints);
		m_alignedPoints.setDisplay(nullptr);
		oldWin->removeFromOwnDB(&m_refPoints);
		m_refPoints.setDisplay(nullptr);

		m_pickingHub->removeListener(this);
	}

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	m_alignedEntities.restoreAll();
	m_referenceEntities.restoreAll();

	if (oldWin && MainWindow::TheInstance())
	{
		QMdiSubWindow* subWindow = MainWindow::TheInstance()->getMDISubWindow(oldWin);
		if (subWindow)
			subWindow->close();
	}

	if (m_associatedWin)
	{
		if (!m_pickingHub->addListener(this, true, true, ccGLWindowInterface::POINT_OR_TRIANGLE_OR_LABEL_PICKING))
		{
			ccLog::Error("Picking mechanism is already in use! Close the other tool first, and then restart this one.");
			return false;
		}
		
		m_associatedWin->addToOwnDB(&m_alignedPoints);
		m_associatedWin->addToOwnDB(&m_refPoints);

		m_associatedWin->displayNewMessage(QString(), ccGLWindowInterface::LOWER_LEFT_MESSAGE);
		m_associatedWin->displayNewMessage("(you can add points 'manually' if necessary)", ccGLWindowInterface::LOWER_LEFT_MESSAGE, true, 3600);
		m_associatedWin->displayNewMessage(QString("Pick equivalent points on both clouds (at least %1 pairs - mind the order)").arg(MIN_PAIRS_COUNT), ccGLWindowInterface::LOWER_LEFT_MESSAGE, true, 3600);
	}

	return true;
}

bool ccPointPairRegistrationDlg::start()
{
	assert(!m_alignedEntities.empty());
	return ccOverlayDialog::start();
}

void ccPointPairRegistrationDlg::stop(bool state)
{
	reset();
	ccOverlayDialog::stop(state);
}

static void SetEnabled_recursive(ccHObject* ent)
{
	assert(ent);
	ent->setEnabled(true);
	if (ent->getParent())
		SetEnabled_recursive(ent->getParent());
}

bool ccPointPairRegistrationDlg::init(	ccGLWindowInterface* win,
										const ccHObject::Container& alignedEntities,
										const ccHObject::Container* referenceEntities/*=nullptr*/)
{
	if (!win)
	{
		assert(false);
		return false;
	}

	clear();

	if (alignedEntities.empty())
	{
		ccLog::Error("[PointPairRegistration] Need at least one to-be-aligned entity!");
		return false;
	}

	m_alignedEntities.fill(alignedEntities);
	if (referenceEntities)
	{
		m_referenceEntities.fill(*referenceEntities);
	}
	
	//create dedicated 3D view
	if (!m_associatedWin)
	{
		//import GL filter so as to get the same rendering aspect!
		{
			//find an orgin display (we'll take the first valid one)
			ccGenericGLDisplay* sourceDisplay = nullptr;
			for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
			{
				sourceDisplay = it.key()->getDisplay();
				if (sourceDisplay)
					// we found a display
					break;
			}
			if (!sourceDisplay && !m_referenceEntities.empty())
			{
				for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
				{
					sourceDisplay = it.key()->getDisplay();
					if (sourceDisplay)
						// we found a display
						break;
				}
			}
			if (sourceDisplay)
			{
				ccGlFilter* filter = static_cast<ccGLWindowInterface*>(sourceDisplay)->getGlFilter();
				if (filter)
					// transfer the GL filter to the dedicated tool window
					win->setGlFilter(filter->clone());
			}
		}
		linkWith(win);
		assert(m_associatedWin);
	}

	//add aligned entity to display
	ccViewportParameters originViewportParams;
	bool hasOriginViewportParams = false;
	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
	{
		ccHObject* aligned = it.key();
		if (!hasOriginViewportParams && aligned->getDisplay())
		{
			hasOriginViewportParams = true;
			originViewportParams = aligned->getDisplay()->getViewportParameters();
		}
		//DGM: it's already in the global DB!
		//m_associatedWin->addToOwnDB(aligned);
		aligned->setDisplay(m_associatedWin);
		aligned->setVisible(true);
		SetEnabled_recursive(aligned);
		//SetVisible_recursive(aligned);

		//add the 1-point child labels as well (if any)
		for (unsigned i = 0; i < aligned->getChildrenNumber(); ++i)
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(aligned->getChild(i));
			if (label && label->size() == 1)
			{
				m_alignedEntities.insert(label, EntityContext(label));
				m_associatedWin->addToOwnDB(label);
				label->setDisplay(m_associatedWin);
				label->setVisible(true);
			}
		}
	}

	//add reference entity (if any) to display
	for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
	{
		ccHObject* reference = it.key();
		if (!hasOriginViewportParams && reference->getDisplay())
		{
			hasOriginViewportParams = true;
			originViewportParams = reference->getDisplay()->getViewportParameters();
		}
		//DGM: it's already in the global DB!
		//m_associatedWin->addToOwnDB(reference);
		reference->setDisplay(m_associatedWin);
		reference->setVisible(true);
		SetEnabled_recursive(reference);
		//SetVisible_recursive(reference);

		//add the 1-point child labels as well (if any)
		for (unsigned i = 0; i < reference->getChildrenNumber(); ++i)
		{
			cc2DLabel* label = ccHObjectCaster::To2DLabel(reference->getChild(i));
			if (label && label->size() == 1)
			{
				m_referenceEntities.insert(label, EntityContext(label));
				m_associatedWin->addToOwnDB(label);
				label->setDisplay(m_associatedWin);
				label->setVisible(true);
			}
		}
	}

	showReferenceCheckBox->setChecked(!m_referenceEntities.empty());
	showReferenceCheckBox->setEnabled(!m_referenceEntities.empty());
	showAlignedCheckBox->setChecked(true);

	m_associatedWin->doShowMaximized();
	resetTitle();

	if (hasOriginViewportParams)
	{
		m_associatedWin->setViewportParameters(originViewportParams);
		m_associatedWin->redraw();
	}
	else
	{
		m_associatedWin->zoomGlobal();
		m_associatedWin->redraw(); //already called by zoomGlobal
	}

	onPointCountChanged();
	
	return true;
}

static QString s_aligned_tooltip = QObject::tr("Whether the point is expressed in the entity original coordinate system (before being shifted by CC) or not");
static double  s_last_ax = 0;
static double  s_last_ay = 0;
static double  s_last_az = 0;
static bool    s_lastAlignePointIsGlobal = true;
void ccPointPairRegistrationDlg::addManualAlignedPoint()
{
	ccAskThreeDoubleValuesDlg ptsDlg("x", "y", "z", -1.0e12, 1.0e12, s_last_ax, s_last_ay, s_last_az, 8, "Add aligned point", this);

	//if the aligned entity is shifted, the user has the choice to input virtual point either
	//in the original coordinate system or the shifted one
	if (m_alignedEntities.isShifted)
		ptsDlg.showCheckbox("Not shifted", s_lastAlignePointIsGlobal, s_aligned_tooltip);

	if (!ptsDlg.exec())
		return;

	//save values for current session
	s_last_ax = ptsDlg.doubleSpinBox1->value();
	s_last_ay = ptsDlg.doubleSpinBox2->value();
	s_last_az = ptsDlg.doubleSpinBox3->value();
	bool shifted = true;
	if (m_alignedEntities.isShifted)
	{
		s_lastAlignePointIsGlobal = ptsDlg.getCheckboxState();
		shifted = !s_lastAlignePointIsGlobal;
	}

	CCVector3d P(s_last_ax, s_last_ay, s_last_az);

	addAlignedPoint(P, nullptr, shifted);
}

static double s_last_rx = 0;
static double s_last_ry = 0;
static double s_last_rz = 0;
static bool s_lastRefPointisGlobal = true;
void ccPointPairRegistrationDlg::addManualRefPoint()
{
	ccAskThreeDoubleValuesDlg ptsDlg("x", "y", "z", -1.0e12, 1.0e12, s_last_rx, s_last_ry, s_last_rz, 8, "Add reference point", this);

	//if the reference entity is shifted, the user has the choice to input virtual
	//points either in the original coordinate system or the shifted one
	//(if there's no reference entity, we use a 'global'	one by default)
	if (m_referenceEntities.isShifted)
		ptsDlg.showCheckbox("Not shifted", s_lastRefPointisGlobal, s_aligned_tooltip);

	if (!ptsDlg.exec())
		return;

	//save values for current session
	s_last_rx = ptsDlg.doubleSpinBox1->value();
	s_last_ry = ptsDlg.doubleSpinBox2->value();
	s_last_rz = ptsDlg.doubleSpinBox3->value();
	bool shifted = (!m_referenceEntities.empty());
	if (m_referenceEntities.isShifted)
	{
		s_lastRefPointisGlobal = ptsDlg.getCheckboxState();
		shifted = !s_lastRefPointisGlobal;
	}

	CCVector3d P(s_last_rx, s_last_ry, s_last_rz);

	addReferencePoint(P, nullptr, shifted);
}

void ccPointPairRegistrationDlg::pause(bool state)
{
	m_paused = state;
	setDisabled(state);
}

bool ccPointPairRegistrationDlg::convertToSphereCenter(CCVector3d& P, ccHObject* entity, PointCoordinateType& sphereRadius)
{
	sphereRadius = 0;
	if (	!entity
		||	!useSphereToolButton->isChecked()
		||	!entity->isKindOf(CC_TYPES::POINT_CLOUD) ) //only works with cloud right now
	{
		//nothing to do
		return true;
	}

	//we'll now try to detect the sphere
	double searchRadius = radiusDoubleSpinBox->value();
	double maxRMSPercentage = maxRmsSpinBox->value() / 100.0;
	ccGenericPointCloud* cloud = static_cast<ccGenericPointCloud*>(entity);
	assert(cloud);

	//crop points inside a box centered on the current point
	ccBBox box;
	box.add((P - CCVector3d(1, 1, 1)*searchRadius).toPC());
	box.add((P + CCVector3d(1, 1, 1)*searchRadius).toPC());
	CCCoreLib::ReferenceCloud* part = cloud->crop(box,true);

	bool success = false;
	if (part && part->size() > 16)
	{
		PointCoordinateType radius;
		CCVector3 C;
		double rms;
		ccProgressDialog pDlg(true, this);
		//first roughly search for the sphere
		if (CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(part, 0.5, C, radius, rms, &pDlg, 0.9) == CCCoreLib::GeometricalAnalysisTools::NoError)
		{
			if (radius / searchRadius < 0.5 || radius / searchRadius > 2.0)
			{
				ccLog::Warning(QString("[ccPointPairRegistrationDlg] Detected sphere radius (%1) is too far from search radius!").arg(radius));
			}
			else
			{
				//now look again (more precisely)
				{
					delete part;
					box.clear();
					box.add(C - CCVector3(1, 1, 1)*radius*static_cast<PointCoordinateType>(1.05)); //add 5%
					box.add(C + CCVector3(1, 1, 1)*radius*static_cast<PointCoordinateType>(1.05)); //add 5%
					part = cloud->crop(box, true);
					if (part && part->size() > 16)
						CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(part, 0.5, C, radius, rms, &pDlg, 0.99);
				}
				ccLog::Print(QString("[ccPointPairRegistrationDlg] Detected sphere radius = %1 (rms = %2)").arg(radius).arg(rms));
				if (radius / searchRadius < 0.5 || radius / searchRadius > 2.0)
				{
					ccLog::Warning("[ccPointPairRegistrationDlg] Sphere radius is too far from search radius!");
				}
				else if (rms / searchRadius >= maxRMSPercentage)
				{
					ccLog::Warning("[ccPointPairRegistrationDlg] RMS is too high!");
				}
				else
				{
					sphereRadius = radius;
					P = C;
					success = true;
				}
			}
		}
		else
		{
			ccLog::Warning("[ccPointPairRegistrationDlg] Failed to fit a sphere around the picked point!");
		}
	}
	else
	{
		//not enough memory? No points inside the 
		ccLog::Warning("[ccPointPairRegistrationDlg] Failed to crop points around the picked point?!");
	}

	if (part)
		delete part;

	return success;
}

void ccPointPairRegistrationDlg::onItemPicked(const PickedItem& pi)
{
	//no point picking when paused!
	if (m_paused)
		return;

	if (!pi.entity)
		return;

	if (m_alignedEntities.contains(pi.entity))
	{
		CCVector3d P = pi.P3D.toDouble();
		addAlignedPoint(P, pi.entity, true); //picked points are always shifted by default
	}
	else if (m_referenceEntities.contains(pi.entity))
	{
		CCVector3d P = pi.P3D.toDouble();
		addReferencePoint(P, pi.entity, true); //picked points are always shifted by default
	}
	else
	{
		if (pi.entity && pi.entity->isA(CC_TYPES::LABEL_2D))
		{
			ccLog::Error(tr("Point/label already picked"));
		}
		else
		{
			assert(false);
		}
		return;
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void ccPointPairRegistrationDlg::onPointCountChanged()
{
	bool canAlign = (m_alignedPoints.size() == m_refPoints.size() && m_refPoints.size() >= MIN_PAIRS_COUNT);
	alignToolButton->setEnabled(canAlign);
	validToolButton->setEnabled(false);

	unstackAlignToolButton->setEnabled(m_alignedPoints.size() != 0);
	unstackRefToolButton->setEnabled(m_refPoints.size() != 0);

	updateAlignInfo();
}

static QToolButton* CreateDeleteButton()
{
	QToolButton* delButton = new QToolButton();
	delButton->setIcon(QIcon(":/CC/images/smallCancel.png"));
	return delButton;
}

static cc2DLabel* CreateLabel(cc2DLabel* label, ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = nullptr)
{
	assert(label);
	label->addPickedPoint(cloud, pointIndex);
	label->setName(pointName);
	label->setVisible(true);
	label->setDisplayedIn2D(false);
	label->displayPointLegend(true);
	label->setDisplay(display);

	return label;
}

static cc2DLabel* CreateLabel(ccPointCloud* cloud, unsigned pointIndex, QString pointName, ccGenericGLDisplay* display = nullptr)
{
	return CreateLabel(new cc2DLabel, cloud, pointIndex, pointName, display);
}

void ccPointPairRegistrationDlg::onDelButtonPushed()
{
	QObject* senderButton = sender();

	//go through all the buttons and find which one has been pushed!
	bool alignedPoint = true;
	int pointIndex = -1;
	//test 'aligned' buttons first
	{
		for (int i = 0; i < alignedPointsTableWidget->rowCount(); ++i)
		{
			if (alignedPointsTableWidget->cellWidget(i, DEL_BUTTON_COL_INDEX) == senderButton)
			{
				pointIndex = i;
				break;
			}
		}
	}

	if (pointIndex < 0)
	{
		//test reference points if necessary
		alignedPoint = false;
		for (int i = 0; i < refPointsTableWidget->rowCount(); ++i)
		{
			if (refPointsTableWidget->cellWidget(i, DEL_BUTTON_COL_INDEX) == senderButton)
			{
				pointIndex = i;
				break;
			}
		}
	}

	if (pointIndex < 0)
	{
		assert(false);
		return;
	}

	if (alignedPoint)
		removeAlignedPoint(pointIndex);
	else
		removeRefPoint(pointIndex);
}

void ccPointPairRegistrationDlg::addPointToTable(	QTableWidget* tableWidget,
													int rowIndex,
													const CCVector3d& P,
													QString pointName )
{
	assert(tableWidget);
	if (!tableWidget)
		return;

	//add corresponding row in table
	tableWidget->setRowCount(std::max<int>(rowIndex + 1, tableWidget->rowCount()));
	tableWidget->setVerticalHeaderItem(rowIndex, new QTableWidgetItem(pointName));

	//add point coordinates
	for (int d = 0; d < 3; ++d)
	{
		QTableWidgetItem* item = new QTableWidgetItem();
		item->setData(Qt::EditRole, QString::number(P.u[d], 'f', 6));
		tableWidget->setItem(rowIndex, XYZ_COL_INDEX + d, item);
	}

	//add 'remove' button
	{
		if (rowIndex == 0)
			tableWidget->setColumnWidth(DEL_BUTTON_COL_INDEX, 20);
		//QTableWidgetItem* item = new QTableWidgetItem();
		//tableWidget->setItem(rowIndex, DEL_BUTTON_COL_INDEX, item);
		QToolButton* delButton = CreateDeleteButton();
		connect(delButton, &QToolButton::clicked, this, &ccPointPairRegistrationDlg::onDelButtonPushed);
		tableWidget->setCellWidget(rowIndex, DEL_BUTTON_COL_INDEX, delButton);
	}
}

bool ccPointPairRegistrationDlg::addAlignedPoint(CCVector3d& Pin, ccHObject* entity/*=nullptr*/, bool shifted/*=false*/)
{
	//if the input point is not shifted, we shift it to the aligned coordinate system
	assert(entity == nullptr || m_alignedEntities.contains(entity));

	//first point?
	if (m_alignedPoints.size() == 0)
	{
		//simply copy the cloud global shift/scale
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		if (cloud)
		{
			m_alignedPoints.copyGlobalShiftAndScale(*cloud);
		}
	}

	PointCoordinateType sphereRadius = 0;
	if (!convertToSphereCenter(Pin, entity, sphereRadius))
		return false;

	//transform the input point in the 'global world' by default
	if (shifted)
		Pin = m_alignedPoints.toGlobal3d<double>(Pin);

	//check that we don't duplicate points
	for (unsigned i = 0; i < m_alignedPoints.size(); ++i)
	{
		CCVector3d Pi = m_alignedPoints.toGlobal3d<PointCoordinateType>(*m_alignedPoints.getPoint(i));
		if (CCCoreLib::LessThanSquareEpsilon((Pi - Pin).norm2d()))
		{
			ccLog::Error("Point already picked or too close to an already selected one!");
			return false;
		}
	}

	unsigned newPointIndex = m_alignedPoints.size();
	if (newPointIndex == m_alignedPoints.capacity() && !m_alignedPoints.reserve(newPointIndex + 1))
	{
		ccLog::Error("Not enough memory?!");
		return false;
	}

	//shift point to the local coordinate system before pushing it
	CCVector3 P = m_alignedPoints.toLocal3pc<double>(Pin);
	m_alignedPoints.addPoint(P);
	
	QString pointName = QString("A%1").arg(newPointIndex);

	//eventually add a label (or a sphere)
	if (sphereRadius <= 0)
	{
		//if the picked point is associated to a label
		cc2DLabel* originLabel = nullptr;
		if (entity && entity->isA(CC_TYPES::LABEL_2D))
		{
			for (const EntityContext& ec : m_alignedEntities)
			{
				if (ec.entity == entity)
				{
					originLabel = ccHObjectCaster::To2DLabel(ec.entity);

					//we've found a label corresponding to the chosen point
					pointName += QString(" (%1)").arg(originLabel->getName());
					originLabel->setEnabled(false);

					//TODO add the label name to a new column?
				}
			}
		}

		cc2DLabel* label = CreateLabel(&m_alignedPoints, newPointIndex, pointName, m_associatedWin);
		if (originLabel)
		{
			//remember the associated label
			label->setMetaData("AssociatedLabelID", originLabel->getUniqueID());
		}
		m_alignedPoints.addChild(label);
	}
	else
	{
		ccGLMatrix trans;
		trans.setTranslation(P);
		ccSphere* sphere = new ccSphere(sphereRadius, &trans, pointName);
		sphere->showNameIn3D(true);
		sphere->setTempColor(ccColor::red,true);
		m_alignedPoints.addChild(sphere);
	}

	//add corresponding row in table
	addPointToTable(alignedPointsTableWidget, newPointIndex, Pin, pointName);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	return true;
}

void ccPointPairRegistrationDlg::removeLabel(	ccPointCloud& points,
												unsigned childIndex,
												const EntityContexts& entities )
{
	if (childIndex >= points.getChildrenNumber())
	{
		assert(false);
		return;
	}
	
	cc2DLabel* label = ccHObjectCaster::To2DLabel(points.getChild(childIndex));
	if (label)
	{
		//if the label had an associated label
		if (label->hasMetaData("AssociatedLabelID"))
		{
			unsigned uuid = label->metaData()["AssociatedLabelID"].toUInt();
			for (const EntityContext& ec : entities)
			{
				if (ec.entity && ec.entity->getUniqueID() == uuid)
				{
					//restore the original entity status (normally it was enabled!)
					ec.entity->setEnabled(ec.wasEnabled);
					break;
				}
			}
		}
	}
	else
	{
		//should be a label!
		assert(false);
	}
	points.removeChild(childIndex);
}

void ccPointPairRegistrationDlg::unstackAligned()
{
	unsigned pointCount = m_alignedPoints.size();
	if (pointCount == 0) //nothing to do
		return;

	assert(alignedPointsTableWidget->rowCount() > 0);
	alignedPointsTableWidget->removeRow(alignedPointsTableWidget->rowCount() - 1);

	//remove label
	assert(m_alignedPoints.getChildrenNumber() == pointCount);
	--pointCount;
	removeLabel(m_alignedPoints, pointCount, m_alignedEntities);
	//remove point
	m_alignedPoints.resize(pointCount);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::removeAlignedPoint(int index, bool autoRemoveDualPoint/*=false*/)
{
	if (index >= static_cast<int>(m_alignedPoints.size()))
	{
		ccLog::Error("[ccPointPairRegistrationDlg::removeAlignedPoint] Invalid index!");
		assert(false);
		return;
	}

	int pointCount = static_cast<int>(m_alignedPoints.size());
	//remove the label (or sphere)
	removeLabel(m_alignedPoints, index, m_alignedEntities);
	//remove array row
	alignedPointsTableWidget->removeRow(index);

	//shift points & rename labels
	for (int i = index + 1; i < pointCount; ++i)
	{
		*const_cast<CCVector3*>(m_alignedPoints.getPoint(i - 1)) = *m_alignedPoints.getPoint(i);

		//new name
		QString pointName = QString("A%1").arg(i - 1);
		//update the label (if any)
		ccHObject* child = m_alignedPoints.getChild(i - 1);
		if (child)
		{
			if (child->isKindOf(CC_TYPES::LABEL_2D))
			{
				cc2DLabel* label = static_cast<cc2DLabel*>(child);
				label->clear();
				CreateLabel(label, &m_alignedPoints, static_cast<unsigned>(i - 1), pointName, m_associatedWin);
			}
			else //probably a sphere
			{
				child->setName(pointName);
			}
		}
		//update array
		alignedPointsTableWidget->setVerticalHeaderItem(i - 1, new QTableWidgetItem(pointName));
	}
	m_alignedPoints.invalidateBoundingBox();
	
	pointCount--;
	assert(pointCount >= 0);
	m_alignedPoints.resize(static_cast<unsigned>(pointCount));

	if (m_alignedPoints.size() == 0)
	{
		//reset global shift (if any)
		m_alignedPoints.setGlobalShift(0, 0, 0);
		m_alignedPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	//auto-remove the other point?
	if (	autoRemoveDualPoint
		&&	index < static_cast<int>(m_refPoints.size())
		&&	QMessageBox::question(nullptr, tr("Remove dual point"), tr("Remove the equivalent reference point as well?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
	{
		removeRefPoint(index, false);
	}
}

bool ccPointPairRegistrationDlg::addReferencePoint(CCVector3d& Pin, ccHObject* entity/*=nullptr*/, bool shifted/*=true*/)
{
	assert(entity == nullptr || m_referenceEntities.contains(entity));

	ccGenericPointCloud* cloud = entity ? ccHObjectCaster::ToGenericPointCloud(entity) : nullptr;

	//first point?
	if (m_refPoints.size() == 0)
	{
		if (entity) //picked point
		{
			//simply copy the cloud global shift/scale
			if (cloud)
			{
				m_refPoints.copyGlobalShiftAndScale(*cloud);
			}
		}
		else //virtual point
		{
			m_refPoints.setGlobalScale(1.0);
			m_refPoints.setGlobalShift(0, 0, 0);

			if (!shifted)
			{
				//test that the input point has not too big coordinates
				//we use the aligned shift by default (if any)
				bool shiftEnabled = m_alignedEntities.isShifted;
				CCVector3d Pshift = m_alignedEntities.shift;
				double scale = 1.0;
				if (ccGlobalShiftManager::Handle(	Pin,
													0.0,
													ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT,
													shiftEnabled,
													Pshift,
													nullptr,
													&scale)
					)
				{
					m_refPoints.setGlobalShift(Pshift);
					m_refPoints.setGlobalScale(scale);
				}
			}
		}
	}

	PointCoordinateType sphereRadius = 0;
	if (!convertToSphereCenter(Pin, entity, sphereRadius))
		return false;

	//transform the input point in the 'global world' by default
	if (shifted && cloud)
	{
		Pin = cloud->toGlobal3d<double>(Pin);
	}

	//check that we don't duplicate points
	for (unsigned i = 0; i < m_refPoints.size(); ++i)
	{
		//express the 'Pi' point in the current global coordinate system
		CCVector3d Pi = m_refPoints.toGlobal3d<PointCoordinateType>(*m_refPoints.getPoint(i));
		if (CCCoreLib::LessThanSquareEpsilon((Pi - Pin).norm2d()))
		{
			ccLog::Error("Point already picked or too close to an already selected one!");
			return false;
		}
	}

	//add point to the 'reference' set
	unsigned newPointIndex = m_refPoints.size();
	if (newPointIndex == m_refPoints.capacity() && !m_refPoints.reserve(newPointIndex + 1))
	{
		ccLog::Error("Not enough memory?!");
		return false;
	}
	
	//shift point to the local coordinate system before pushing it
	CCVector3 P = m_refPoints.toLocal3pc<double>(Pin);
	m_refPoints.addPoint(P);

	QString pointName = QString("R%1").arg(newPointIndex);

	//eventually add a label (or a sphere)
	if (sphereRadius <= 0)
	{
		//if the picked point is associated to a label
		cc2DLabel* originLabel = nullptr;
		if (entity && entity->isA(CC_TYPES::LABEL_2D))
		{
			for (const EntityContext& ec : m_referenceEntities)
			{
				if (ec.entity == entity)
				{
					originLabel = ccHObjectCaster::To2DLabel(ec.entity);

					//we've found a label corresponding to the chosen point
					pointName += QString(" (%1)").arg(originLabel->getName());
					originLabel->setEnabled(false);

					//TODO add the label name to a new column?
				}
			}
		}

		cc2DLabel* label = CreateLabel(&m_refPoints, newPointIndex, pointName, m_associatedWin);
		if (originLabel)
		{
			//remember the associated label
			label->setMetaData("AssociatedLabelID", originLabel->getUniqueID());
		}
		m_refPoints.addChild(label);
	}
	else
	{
		ccGLMatrix trans;
		trans.setTranslation(P);
		ccSphere* sphere = new ccSphere(sphereRadius, &trans, pointName);
		sphere->showNameIn3D(true);
		sphere->setTempColor(ccColor::yellow, true);
		m_refPoints.addChild(sphere);
	}

	//add corresponding row in table
	addPointToTable(refPointsTableWidget, newPointIndex, Pin, pointName);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	return true;
}

void ccPointPairRegistrationDlg::unstackRef()
{
	unsigned pointCount = m_refPoints.size();
	if (pointCount == 0)
		return;

	assert(refPointsTableWidget->rowCount() > 0);
	refPointsTableWidget->removeRow(refPointsTableWidget->rowCount() - 1);

	//remove label
	assert(m_refPoints.getChildrenNumber() == pointCount);
	pointCount--;
	removeLabel(m_refPoints, pointCount, m_referenceEntities);
	//remove point
	m_refPoints.resize(pointCount);

	if (pointCount == 0)
	{
		//reset global shift (if any)
		m_refPoints.setGlobalShift(0, 0, 0);
		m_refPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();
}

void ccPointPairRegistrationDlg::removeRefPoint(int index, bool autoRemoveDualPoint/*=false*/)
{
	if (index >= static_cast<int>(m_refPoints.size()))
	{
		ccLog::Error("[ccPointPairRegistrationDlg::removeRefPoint] Invalid index!");
		assert(false);
		return;
	}

	int pointCount = static_cast<int>(m_refPoints.size());
	//remove the label (or sphere)
	removeLabel(m_refPoints, index, m_referenceEntities);
	//remove array row
	refPointsTableWidget->removeRow(index);

	//shift points & rename labels
	for (int i = index + 1; i < pointCount; ++i)
	{
		*const_cast<CCVector3*>(m_refPoints.getPoint(i - 1)) = *m_refPoints.getPoint(i);

		//new name
		QString pointName = QString("R%1").arg(i - 1);
		//update the label (if any)
		ccHObject* child = m_refPoints.getChild(i - 1);
		if (child)
		{
			if (child->isKindOf(CC_TYPES::LABEL_2D))
			{
				cc2DLabel* label = static_cast<cc2DLabel*>(child);
				label->clear();
				CreateLabel(label, &m_refPoints, static_cast<unsigned>(i - 1), pointName, m_associatedWin);
			}
			else //probably a sphere
			{
				child->setName(pointName);
			}
		}
		//update array
		refPointsTableWidget->setVerticalHeaderItem(i - 1, new QTableWidgetItem(pointName));
	}
	m_refPoints.invalidateBoundingBox();

	pointCount--;
	assert(pointCount >= 0);
	m_refPoints.resize(static_cast<unsigned>(pointCount));

	if (m_refPoints.size() == 0)
	{
		//reset global shift (if any)
		m_refPoints.setGlobalShift(0, 0, 0);
		m_refPoints.setGlobalScale(1.0);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	onPointCountChanged();

	//auto-remove the other point?
	if (	autoRemoveDualPoint
		&&	index < static_cast<int>(m_alignedPoints.size())
		&&	QMessageBox::question(nullptr, "Remove dual point", "Remove the equivalent to-be-aligned point as well?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes)
	{
		removeAlignedPoint(index, false);
	}
}

void ccPointPairRegistrationDlg::showAlignedEntities(bool state)
{
	if (m_alignedEntities.empty())
	{
		return;
	}

	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
	{
		it.key()->setVisible(state);
	}
	m_alignedPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
		{
			m_associatedWin->zoomGlobal();
		}
		m_associatedWin->redraw();
	}
}

void ccPointPairRegistrationDlg::showReferenceEntities(bool state)
{
	if (m_referenceEntities.empty())
	{
		return;
	}

	for (auto it = m_referenceEntities.begin(); it != m_referenceEntities.end(); ++it)
	{
		it.key()->setVisible(state);
	}
	m_refPoints.setEnabled(state);

	if (m_associatedWin)
	{
		if (autoZoomCheckBox->isChecked())
		{
			m_associatedWin->zoomGlobal();
		}
		m_associatedWin->redraw();
	}
}

static bool UmeyamaRegistration(CCCoreLib::GenericCloud* toBeAlignedPoints,
								CCCoreLib::GenericCloud* referencePoints,
								CCCoreLib::PointProjectionTools::Transformation& trans,
								bool fixedScale)
{
#ifdef CC_USES_EIGEN
	//output transformation (R is invalid on initialization, T is (0,0,0) and s==1)
	trans.R.invalidate();
	trans.T = CCVector3d(0, 0, 0);
	trans.s = 1.0;

	if (toBeAlignedPoints == nullptr
		|| referencePoints == nullptr
		|| toBeAlignedPoints->size() != referencePoints->size()
		|| toBeAlignedPoints->size() < 3)
	{
		return false;
	}

	unsigned pointCount = toBeAlignedPoints->size();

	// Use Eigen Umeyama with 3D vectors
	Eigen::MatrixXd srcMatrix(3, pointCount), destMatrix(3, pointCount);

	toBeAlignedPoints->placeIteratorAtBeginning();
	referencePoints->placeIteratorAtBeginning();
	for (size_t i = 0; i < pointCount; ++i)
	{
		const CCVector3* P = toBeAlignedPoints->getNextPoint();
		srcMatrix(0, i) = P->x;
		srcMatrix(1, i) = P->y;
		srcMatrix(2, i) = P->z;

		const CCVector3* Q = referencePoints->getNextPoint();
		destMatrix(0, i) = Q->x;
		destMatrix(1, i) = Q->y;
		destMatrix(2, i) = Q->z;
	}

	Eigen::MatrixXd resultUmeyama = Eigen::umeyama(srcMatrix, destMatrix, !fixedScale);

	//scale
	if (!fixedScale)
	{
		//calculate the scale
		trans.s = CCVector3d(resultUmeyama(0, 0), resultUmeyama(1, 0), resultUmeyama(2, 0)).normd();
		if (trans.s == 0)
		{
			assert(false);
			return false;
		}
	}

	//rotation
	{
		trans.R = CCCoreLib::SquareMatrixd(3);
		//1st column
		trans.R.m_values[0][0] = resultUmeyama(0, 0) / trans.s;
		trans.R.m_values[0][1] = resultUmeyama(0, 1) / trans.s;
		trans.R.m_values[0][2] = resultUmeyama(0, 2) / trans.s;
		//2nd column
		trans.R.m_values[1][0] = resultUmeyama(1, 0) / trans.s;
		trans.R.m_values[1][1] = resultUmeyama(1, 1) / trans.s;
		trans.R.m_values[1][2] = resultUmeyama(1, 2) / trans.s;
		//3rd column
		trans.R.m_values[2][0] = resultUmeyama(2, 0) / trans.s;
		trans.R.m_values[2][1] = resultUmeyama(2, 1) / trans.s;
		trans.R.m_values[2][2] = resultUmeyama(2, 2) / trans.s;
	}

	//translation
	{
		trans.T.x = resultUmeyama(0, 3);
		trans.T.y = resultUmeyama(1, 3);
		trans.T.z = resultUmeyama(2, 3);
	}

	return true;
#else
	// Eigen is not supported
	return false;
#endif
}

bool ccPointPairRegistrationDlg::callRegistration(	CCCoreLib::PointProjectionTools::Transformation& trans,
													double& rms,
													bool autoUpdateTab,
													bool& withUmeyama,
													QStringList* report/*=nullptr*/ )
{
	withUmeyama = false;

	if (report)
	{
		report->clear();
	}

	if (m_alignedEntities.empty())
	{
		assert(false);
		return false;
	}

	if (m_alignedPoints.size() != m_refPoints.size() || m_refPoints.size() < MIN_PAIRS_COUNT)
	{
		assert(false);
		ccLog::Error(QString("Need at least %1 points for each entity (and the same number of points in both subsets)!").arg(MIN_PAIRS_COUNT));
		return false;
	}

	//fixed scale?
	bool adjustScale = adjustScaleCheckBox->isChecked();

	//! Calls Horn or Umeyama registration procedure (depending on which one is available)
	bool success = false;
#ifdef CC_USES_EIGEN
	// we prefer using Umeyama's method if available
	withUmeyama = true;

	success = UmeyamaRegistration(&m_alignedPoints, &m_refPoints, trans, !adjustScale);
#else
	success = CCCoreLib::HornRegistrationTools::FindAbsoluteOrientation(&m_alignedPoints, &m_refPoints, trans, !adjustScale);
#endif

	if (!success)
	{
		ccLog::Error("Registration failed! (points are all aligned?)");
		return false;
	}

	//apply constraints (if any)
	{
		int filters = 0;
		switch (rotComboBox->currentIndex())
		{
		case 1:
			filters |= CCCoreLib::RegistrationTools::SKIP_RYZ;
			break;
		case 2:
			filters |= CCCoreLib::RegistrationTools::SKIP_RXZ;
			break;
		case 3:
			filters |= CCCoreLib::RegistrationTools::SKIP_RXY;
			break;
		case 4:
			filters |= CCCoreLib::RegistrationTools::SKIP_ROTATION;
			break;
		default:
			//nothing to do
			break;
		}

		if (!TxCheckBox->isChecked())
			filters |= CCCoreLib::RegistrationTools::SKIP_TX;
		if (!TyCheckBox->isChecked())
			filters |= CCCoreLib::RegistrationTools::SKIP_TY;
		if (!TzCheckBox->isChecked())
			filters |= CCCoreLib::RegistrationTools::SKIP_TZ;

		if (filters != 0)
		{
			CCCoreLib::RegistrationTools::FilterTransformation(	trans,
																filters,
																m_alignedPoints.computeGravityCenter(),
																m_refPoints.computeGravityCenter(),
																trans);
		}
	}

	//compute RMS
	rms = CCCoreLib::HornRegistrationTools::ComputeRMS(&m_alignedPoints, &m_refPoints, trans);

	if (autoUpdateTab || report)
	{
		//display resulting RMS in the right column
		if (rms >= 0)
		{
			assert(m_alignedPoints.size() == m_refPoints.size());

			if (report)
			{
				// add header (first column is the timestamp)
				report->append("; Ref.name; Ref.x; Ref.y; Ref.z; Aligned.name; Aligned.x; Aligned.y; Aligned.z; Delta X; Delta Y; Delta Z; Distance");
			}

			for (unsigned i = 0; i < m_alignedPoints.size(); ++i)
			{
				const CCVector3* Ri = m_refPoints.getPoint(i);
				CCVector3d Rid = Ri->toDouble();
				const CCVector3* Li = m_alignedPoints.getPoint(i);
				CCVector3d Lit = trans.apply(*Li);
				double dist = (Rid - Lit).norm();

				if (report)
				{
					// create a new line in the report
					QString alignedName;
					auto* alignedHeader = alignedPointsTableWidget->verticalHeaderItem(static_cast<int>(i));
					if (alignedHeader)
					{
						alignedName = alignedHeader->text();
					}
					else
					{
						assert(false);
						alignedName = QString("A%1").arg(i);
					}

					QString refName;
					auto* refHeader = refPointsTableWidget->verticalHeaderItem(static_cast<int>(i));
					if (refHeader)
					{
						refName = refHeader->text();
					}
					else
					{
						assert(false);
						refName = QString("R%1").arg(i);
					}

					//first column timestamp
					QString reportLine = QString(";%1; %2; %3; %4").arg(refName)
						.arg(Ri->x)
						.arg(Ri->y)
						.arg(Ri->z);

					reportLine += QString("; %1; %2; %3; %4").arg(alignedName)
						.arg(Lit.x)
						.arg(Lit.y)
						.arg(Lit.z);

					//errors along axis
					for (unsigned coordIndex = 0; coordIndex < 3; ++coordIndex)
					{
						reportLine += QString("; %1").arg(Lit[coordIndex]- Rid[coordIndex]);
					}

					reportLine += QString("; %1").arg(dist);

					report->append(reportLine);
				}

				if (report)
				{
					QString registrationMethod = tr("Registration method: ") + (withUmeyama ? "Umeyama" : "Horn");
					report->append(registrationMethod);
				}

				if (autoUpdateTab)
				{
					QTableWidgetItem* itemA = new QTableWidgetItem();
					itemA->setData(Qt::EditRole, dist);
					alignedPointsTableWidget->setItem(i, RMS_COL_INDEX, itemA);

					QTableWidgetItem* itemR = new QTableWidgetItem();
					itemR->setData(Qt::EditRole, dist);
					refPointsTableWidget->setItem(i, RMS_COL_INDEX, itemR);

					//update errors along Axis
					for (unsigned j = 0; j < 3; ++j) {
						double diffAlongAxis = Lit[j] - Rid[j];
						QTableWidgetItem* itemAAlongAxis = new QTableWidgetItem();
						itemAAlongAxis->setData(Qt::EditRole, diffAlongAxis);
						alignedPointsTableWidget->setItem(i, RMS_COL_INDEX - 3 + j, itemAAlongAxis);

						QTableWidgetItem* itemRAlongAxis = new QTableWidgetItem();
						itemRAlongAxis->setData(Qt::EditRole, diffAlongAxis);
						refPointsTableWidget->setItem(i, RMS_COL_INDEX - 3 + j, itemRAlongAxis);
					}
				}
			}
		}
		else
		{
			//clear RMS columns
			clearRMSColumns();
		}
	}

	return true;
}

void ccPointPairRegistrationDlg::clearRMSColumns()
{
	for (int i = 0; alignedPointsTableWidget->rowCount(); ++i)
		alignedPointsTableWidget->setItem(i, RMS_COL_INDEX, new QTableWidgetItem());
	for (int i = 0; refPointsTableWidget->rowCount(); ++i)
		refPointsTableWidget->setItem(i, RMS_COL_INDEX, new QTableWidgetItem());
}

void ccPointPairRegistrationDlg::resetTitle()
{
	if (m_associatedWin)
	{
		m_associatedWin->displayNewMessage(QString(), ccGLWindowInterface::UPPER_CENTER_MESSAGE, false);
		m_associatedWin->displayNewMessage("[Point-pair registration]", ccGLWindowInterface::UPPER_CENTER_MESSAGE, true, 3600);
	}
}

void ccPointPairRegistrationDlg::updateAlignInfo()
{
	//reset title
	resetTitle();

	CCCoreLib::PointProjectionTools::Transformation trans;
	double rms = 0.0;
	bool withUmeyama = false;

	if (	m_alignedPoints.size() == m_refPoints.size()
		&&	m_refPoints.size() >= MIN_PAIRS_COUNT
		&&	callRegistration(trans, rms, true, withUmeyama))
	{
		QString rmsString = tr("Achievable RMS: %1").arg(rms);
		if (m_associatedWin)
		{
			m_associatedWin->displayNewMessage(rmsString, ccGLWindowInterface::UPPER_CENTER_MESSAGE, true, 60 * 60);
		}
		rmsString.prepend(withUmeyama ? "[Umeyama] " : "[Horn] ");
		ccLog::Print(rmsString);
		resetToolButton->setEnabled(true);
		validToolButton->setEnabled(true);
	}
	else
	{
		resetToolButton->setEnabled(false);
		validToolButton->setEnabled(false);
	}

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void ccPointPairRegistrationDlg::align()
{
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	CCCoreLib::PointProjectionTools::Transformation trans;
	double rms = std::numeric_limits<double>::quiet_NaN();

	//reset title
	resetTitle();
	m_associatedWin->refresh(true);

	bool withUmeyama = false;
	if (callRegistration(trans, rms, true, withUmeyama))
	{
		if (rms >= 0)
		{
			QString rmsString = tr("Current RMS: %1").arg(rms);
			ccLog::Print(QString("[PointPairRegistration][%1] ").arg(withUmeyama ? "Umeyama" : "Horn") + rmsString);
			m_associatedWin->displayNewMessage(rmsString, ccGLWindowInterface::UPPER_CENTER_MESSAGE, true, 60 * 60);
		}
		else
		{
			ccLog::Warning("[PointPairRegistration] Internal error (negative RMS?!)");
			return;
		}

		//apply (scaled) transformation (if not fixed)
		bool adjustScale = adjustScaleCheckBox->isChecked();
		if (adjustScale)
		{
			if (trans.R.isValid())
				trans.R.scale(trans.s);

			QString scaleString = QString("Scale: %1").arg(trans.s);
			ccLog::Print(QString("[PointPairRegistration] ") + scaleString);
		}
		else
		{
			ccLog::Print(tr("[PointPairRegistration] Scale: fixed (1.0)"));
		}

		ccGLMatrix transMat = FromCCLibMatrix<double, float>(trans.R, trans.T);
		//...virtually
		for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
		{
			it.key()->setGLTransformation(transMat);
		}
		m_alignedPoints.setGLTransformation(transMat);
		//DGM: we have to 'counter-scale' the markers (otherwise they might appear very big or very small!)
		for (unsigned i = 0; i < m_alignedPoints.getChildrenNumber(); ++i)
		{
			ccHObject* child = m_alignedPoints.getChild(i);
			if (child->isA(CC_TYPES::LABEL_2D))
			{
				static_cast<cc2DLabel*>(child)->setRelativeMarkerScale(static_cast<float>(1.0 / trans.s));
			}
		}

		//force clouds visibility
		{
			//we don't want the window zoom to change or the window to be be redrawn
			ccGLWindowInterface* associatedWin = nullptr;
			std::swap(m_associatedWin, associatedWin);
			if (!showAlignedCheckBox->isChecked())
				showAlignedCheckBox->setChecked(true);
			if (!showReferenceCheckBox->isChecked())
				showReferenceCheckBox->setChecked(true);
			//restore window ref
			std::swap(m_associatedWin, associatedWin);
		}

		if (m_associatedWin)
		{
			if (autoZoomCheckBox->isChecked())
			{
				m_associatedWin->zoomGlobal();
			}
			m_associatedWin->redraw();
		}

		resetToolButton->setEnabled(true);
		validToolButton->setEnabled(true);
	}
}

void ccPointPairRegistrationDlg::reset()
{
	if (m_alignedEntities.empty())
	{
		return;
	}

	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
	{
		it.key()->enableGLTransformation(false);
	}
	m_alignedPoints.enableGLTransformation(false);

	//DGM: we have to reset the markers scale
	for (unsigned i = 0; i < m_alignedPoints.getChildrenNumber(); ++i)
	{
		ccHObject* child = m_alignedPoints.getChild(i);
		if (child->isA(CC_TYPES::LABEL_2D))
		{
			static_cast<cc2DLabel*>(child)->setRelativeMarkerScale(1.0f);
		}
	}

	if (m_associatedWin && autoZoomCheckBox->isChecked())
	{
		m_associatedWin->zoomGlobal();
	}

	updateAlignInfo();
}

void ccPointPairRegistrationDlg::apply()
{
	CCCoreLib::PointProjectionTools::Transformation trans;
	double rms = -1.0;
	QStringList report;
	bool withUmeyama = false;

	if (callRegistration(trans, rms, false, withUmeyama, &report))
	{
		QStringList summary;
		if (rms >= 0)
		{
			QString rmsString = QString("Final RMS: %1").arg(rms);
			report << rmsString;
			summary << rmsString;
			summary << "----------------";

			//output the report to the Console/Log
			ccLog::Print("[PointPairRegistration] Final alignment report:");
			for (const QString& line : report)
			{
				ccLog::Print(line);
			}
		}

		//apply (scaled) transformation (if not fixed)
		bool adjustScale = adjustScaleCheckBox->isChecked();
		if (adjustScale && trans.R.isValid())
		{
			trans.R.scale(trans.s);
		}
		ccGLMatrix transMat = FromCCLibMatrix<double, float>(trans.R, trans.T);
		//...for real this time!
		assert(!m_alignedEntities.empty());
		//we temporarily detach entity, as it may undergo
		//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyGLTransformation
		for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
		{
			ccMainAppInterface::ccHObjectContext objContext;
			if (m_app)
				objContext = m_app->removeObjectTemporarilyFromDBTree(it.key());
			it.key()->applyGLTransformation_recursive();
			if (m_app)
				m_app->putObjectBackIntoDBTree(it.key(), objContext);
		}
		m_alignedPoints.setGLTransformation(transMat);
		summary << QString("Transformation matrix");
		summary << transMat.toString(3, '\t'); //low precision, just for display
		summary << "----------------";

		ccLog::Print("[PointPairRegistration] Applied transformation matrix:");
		ccLog::Print(transMat.toString(12, ' ')); //full precision

		if (adjustScale)
		{
			QString scaleString = QString("Scale: %1 (already integrated in above matrix!)").arg(trans.s);
			ccLog::Warning(QString("[PointPairRegistration] ") + scaleString);
			summary << scaleString;
		}
		else
		{
			ccLog::Print(QString("[PointPairRegistration] Scale: fixed (1.0)"));
			summary << "Scale: fixed (1.0)";
		}
		summary << "----------------";

		//pop-up summary
		summary << "Refer to Console (F8) for more details";
		QMessageBox::information(this, "Align info", summary.join("\n"));

		//don't forget global shift:
		//reference shift takes precedence on the aligned entities'
		bool referenceIsShifted = false;
		CCVector3d referenceShift(0, 0, 0);
		double referenceScale = 1.0;
		if (!m_referenceEntities.isEmpty())
		{
			referenceIsShifted = m_referenceEntities.isShifted;
			referenceShift = m_referenceEntities.shift;
			referenceScale = m_referenceEntities.scale;

			ccGLMatrixd transMatD = FromCCLibMatrix<double, double>(trans.R, trans.T);
			transMatD.scale(1.0 / referenceScale);
			transMatD.setTranslation(transMatD.getTranslationAsVec3D() - referenceShift);
			ccLog::Print("[PointPairRegistration] Transformation to global coordinates:");
			ccLog::Print(transMatD.toString(12, ' ')); //full precision
		}
		else if (m_refPoints.isShifted())
		{
			// shift was automatically applied (temporarily, and for display purposes)
			CCVector3d Pshift = m_refPoints.getGlobalShift();
			double scale = m_refPoints.getGlobalScale();
			CCVector3d Pin = m_refPoints.toGlobal3d(*m_refPoints.getPoint(0));
			if (ccGlobalShiftManager::Handle(	Pin,
												0.0,
												ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG,
												true,
												Pshift,
												nullptr,
												&scale)
				)
			{
				referenceIsShifted = true;
				referenceShift = Pshift;
				referenceScale = scale;
			}
		}

		bool alwaysDropShift = false;
		bool alwaysDropShiftQuestionAsked = false;
		for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
		{
			ccGenericPointCloud* alignedCloud = ccHObjectCaster::ToGenericPointCloud(it.key());
			if (alignedCloud)
			{
				if (referenceIsShifted)
				{
					alignedCloud->setGlobalShift(referenceShift);
					alignedCloud->setGlobalScale(referenceScale);
					ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been updated to match the reference: (%1,%2,%3) [x%4]")
						.arg(alignedCloud->getName())
						.arg(referenceShift.x)
						.arg(referenceShift.y)
						.arg(referenceShift.z)
						.arg(referenceScale));
				}
				else if (alignedCloud->isShifted()) // the aligned cloud is shifted, but not the reference cloud
				{
					//we'll ask the user confirmation before dropping the shift information on the aligned cloud
					if (!alwaysDropShiftQuestionAsked)
					{
						alwaysDropShift = (QMessageBox::question(this, tr("Drop shift information?"), tr("To-be-aligned cloud is shifted but reference cloud is not: drop global shift information?"), QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
						alwaysDropShiftQuestionAsked = true;
					}

					if (alwaysDropShift)
					{
						alignedCloud->setGlobalShift(0, 0, 0);
						alignedCloud->setGlobalScale(1.0);
						ccLog::Warning(tr("[PointPairRegistration] Cloud %1: global shift has been reset to match the reference!").arg(alignedCloud->getName()));
					}
				}
			}
		}
	}
	else
	{
		ccLog::Warning(QString("[PointPairRegistration] Failed to register entities?!"));
	}

	//save persistent settings
	{
		QSettings settings;
		settings.beginGroup("PointPairAlign");
		settings.setValue("PickSpheres",  useSphereToolButton->isChecked());
		settings.setValue("SphereRadius", radiusDoubleSpinBox->value());
		settings.setValue("MaxRMS",       maxRmsSpinBox->value());
		settings.setValue("AdjustScale",  adjustScaleCheckBox->isChecked());
		settings.setValue("AutoUpdateZom",autoZoomCheckBox->isChecked());
		settings.endGroup();
	}
	
	stop(true);
}

void ccPointPairRegistrationDlg::cancel()
{
	for (auto it = m_alignedEntities.begin(); it != m_alignedEntities.end(); ++it)
		it.key()->enableGLTransformation(false);

	stop(false);
}
