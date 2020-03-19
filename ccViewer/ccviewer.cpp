//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initiated under funding from ANR/CIFRE          #
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
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

#include "ccviewer.h"
#include "ccViewerApplication.h"

//Qt
#include <QMessageBox>

//qCC_glWindow
#include <ccGLWidget.h>

//common dialogs
#include <ccCameraParamEditDlg.h>
#include <ccDisplayOptionsDlg.h>
#include <ccStereoModeDlg.h>

//qCC_db
#include <ccGenericMesh.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>

//plugins
#include "ccGLPluginInterface.h"
#include "ccIOPluginInterface.h"
#include "ccPluginManager.h"

//3D mouse handler
#ifdef CC_3DXWARE_SUPPORT
#include <devices/3dConnexion/Mouse3DInput.h>
#endif

//Camera parameters dialog
static ccCameraParamEditDlg* s_cpeDlg = nullptr;

ccViewer::ccViewer(QWidget *parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags)
	, m_glWindow(nullptr)
	, m_selectedObject(nullptr)
	, m_3dMouseInput(nullptr)
{
	ui.setupUi(this);

#ifdef Q_OS_LINUX
	 //we reset the whole stylesheet but we keep the StatusBar style
	setStyleSheet("");	
	setStyleSheet("QStatusBar{background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,stop:0 rgb(200,200,200), stop:1 rgb(255,255,255));}");
#endif
	
	setWindowTitle(QString("ccViewer v%1").arg(ccApp->versionLongStr( false )));

	//insert GL window in a vertical layout
	{
		QVBoxLayout* verticalLayout = new QVBoxLayout(ui.GLframe);
		verticalLayout->setSpacing(0);
		const int margin = 10;
		verticalLayout->setContentsMargins(margin, margin, margin, margin);

		bool stereoMode = QSurfaceFormat::defaultFormat().stereo();

		QWidget* glWidget = nullptr;
		CreateGLWindow(m_glWindow, glWidget, stereoMode);
		assert(m_glWindow && glWidget);

		verticalLayout->addWidget(glWidget);
	}

	updateGLFrameGradient();

	m_glWindow->setRectangularPickingAllowed(false); //multiple entities picking not supported

	//UI/display synchronization
	ui.actionFullScreen->setChecked(false);
	ui.menuSelected->setEnabled(false);
	reflectLightsState();
	reflectPerspectiveState();
	reflectPivotVisibilityState();

#ifdef CC_3DXWARE_SUPPORT
	enable3DMouse(true);
#else
	ui.actionEnable3DMouse->setEnabled(false);
#endif

	//Signals & slots connection
	connect(m_glWindow,								&ccGLWindow::filesDropped,				this, static_cast<void (ccViewer::*)(QStringList)>(&ccViewer::addToDB), Qt::QueuedConnection);
	connect(m_glWindow,								&ccGLWindow::entitySelectionChanged,	this,	&ccViewer::selectEntity);
	connect(m_glWindow,								&ccGLWindow::exclusiveFullScreenToggled,this,	&ccViewer::onExclusiveFullScreenToggled);
	
	//"Options" menu
	connect(ui.actionDisplayParameters,				&QAction::triggered,					this,	&ccViewer::showDisplayParameters);
	connect(ui.actionEditCamera,					&QAction::triggered,					this,	&ccViewer::doActionEditCamera);
	//"Display > Standard views" menu
	connect(ui.actionSetViewTop,					&QAction::triggered,					this,	&ccViewer::setTopView);
	connect(ui.actionSetViewBottom,					&QAction::triggered,					this,	&ccViewer::setBottomView);
	connect(ui.actionSetViewFront,					&QAction::triggered,					this,	&ccViewer::setFrontView);
	connect(ui.actionSetViewBack,					&QAction::triggered,					this,	&ccViewer::setBackView);
	connect(ui.actionSetViewLeft,					&QAction::triggered,					this,	&ccViewer::setLeftView);
	connect(ui.actionSetViewRight,					&QAction::triggered,					this,	&ccViewer::setRightView);
	connect(ui.actionSetViewIso1,					&QAction::triggered,					this,	&ccViewer::setIsoView1);
	connect(ui.actionSetViewIso2,					&QAction::triggered,					this,	&ccViewer::setIsoView2);

	//"Options > Perspective" menu
	connect(ui.actionSetOrthoView,					&QAction::triggered,					this,	&ccViewer::setOrthoView);
	connect(ui.actionSetCenteredPerspectiveView,	&QAction::triggered,					this,	&ccViewer::setCenteredPerspectiveView);
	connect(ui.actionSetViewerPerspectiveView,		&QAction::triggered,					this,	&ccViewer::setViewerPerspectiveView);
	//"Options > Rotation symbol" menu
	connect(ui.actionSetPivotAlwaysOn,				&QAction::triggered,					this,	&ccViewer::setPivotAlwaysOn);
	connect(ui.actionSetPivotRotationOnly,			&QAction::triggered,					this,	&ccViewer::setPivotRotationOnly);
	connect(ui.actionSetPivotOff,					&QAction::triggered,					this,	&ccViewer::setPivotOff);
	//"Options > 3D mouse" menu
	connect(ui.actionEnable3DMouse,					&QAction::toggled,						this,	&ccViewer::enable3DMouse);
	//"Display > Lights & Materials" menu
	connect(ui.actionToggleSunLight,				&QAction::toggled,						this,	&ccViewer::toggleSunLight);
	connect(ui.actionToggleCustomLight,				&QAction::toggled,						this,	&ccViewer::toggleCustomLight);
	//"Options" menu
	connect(ui.actionGlobalZoom,					&QAction::triggered,					this,	&ccViewer::setGlobalZoom);
	connect(ui.actionEnableStereo,					&QAction::toggled,						this,	&ccViewer::toggleStereoMode);
	connect(ui.actionFullScreen,					&QAction::toggled,						this,	&ccViewer::toggleFullScreen);
	connect(ui.actionLockRotationVertAxis,			&QAction::triggered,					this,   &ccViewer::toggleRotationAboutVertAxis);

	//"Options > Selected" menu
	connect(ui.actionShowColors,					&QAction::toggled,						this,	&ccViewer::toggleColorsShown);
	connect(ui.actionShowNormals,					&QAction::toggled,						this,	&ccViewer::toggleNormalsShown);
	connect(ui.actionShowMaterials,					&QAction::toggled,						this,	&ccViewer::toggleMaterialsShown);
	connect(ui.actionShowScalarField,				&QAction::toggled,						this,	&ccViewer::toggleScalarShown);
	connect(ui.actionShowColorRamp,					&QAction::toggled,						this,	&ccViewer::toggleColorbarShown);
	connect(ui.actionZoomOnSelectedEntity,			&QAction::triggered,					this,	&ccViewer::zoomOnSelectedEntity);
	connect(ui.actionDelete,						&QAction::triggered,					this,	&ccViewer::doActionDeleteSelectedEntity);


	//"Shaders" menu
	connect(ui.actionNoFilter,						&QAction::triggered,					this,	&ccViewer::doDisableGLFilter);

	//"Help" menu
	connect(ui.actionAbout,							&QAction::triggered,					this,	&ccViewer::doActionAbout);
	connect(ui.actionHelpShortctus,					&QAction::triggered,					this,	&ccViewer::doActionDisplayShortcuts);

	loadPlugins();
}

ccViewer::~ccViewer()
{
	release3DMouse();

	if (s_cpeDlg)
	{
		delete s_cpeDlg;
		s_cpeDlg = nullptr;
	}

	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		m_glWindow->setSceneDB(nullptr);
		//m_glWindow->redraw();
		delete currentRoot;
	}
#ifdef CC_GL_WINDOW_USE_QWINDOW
	m_glWindow->setParent(0);
#endif
}

void ccViewer::loadPlugins()
{
	ui.menuPlugins->setEnabled(false);

	ccPluginManager::get().loadPlugins();

	for ( ccPluginInterface *plugin : ccPluginManager::get().pluginList() )
	{
		if ( plugin == nullptr )
		{
			Q_ASSERT( false );
			continue;
		}

		// is this a GL plugin?
		if ( plugin->getType() == CC_GL_FILTER_PLUGIN )
		{
			ccGLPluginInterface *glPlugin = static_cast<ccGLPluginInterface*>( plugin );
			
			const QString pluginName = glPlugin->getName();
			
			Q_ASSERT( !pluginName.isEmpty() );
			
			if ( pluginName.isEmpty() )
			{
				// should be unreachable - we have already checked for this in ccPlugins::Find()
				continue;
			}
			
			ccLog::Print( QStringLiteral( "Plugin name: [%1] (GL filter)" ).arg( pluginName ) );

			QAction* action = new QAction( pluginName, this );
			action->setToolTip( glPlugin->getDescription() );
			action->setIcon( glPlugin->getIcon() );
			
			// store the plugin's interface pointer in the QAction data so we can access it in doEnableGLFilter()
			QVariant v;
	  
			v.setValue( glPlugin );
	  
			action->setData( v );

			connect(action, &QAction::triggered, this, &ccViewer::doEnableGLFilter);

			ui.menuPlugins->addAction( action );
			ui.menuPlugins->setEnabled( true );
			ui.menuPlugins->setVisible( true );
		}
	}
}

void ccViewer::doDisableGLFilter()
{
	if (m_glWindow)
	{
		m_glWindow->setGlFilter(nullptr);
		m_glWindow->redraw();
	}
}

void ccViewer::doEnableGLFilter()
{
	if (!m_glWindow)
	{
		ccLog::Warning("[GL filter] No active 3D view!");
		return;
	}

	QAction *action = qobject_cast<QAction*>(sender());

	if ( action == nullptr )
	{
		Q_ASSERT( false );
		return;
	}
	
	ccGLPluginInterface	*plugin = action->data().value<ccGLPluginInterface *>();
	
	if ( plugin == nullptr )
	{
		return;
	}

	Q_ASSERT( plugin->getType() == CC_GL_FILTER_PLUGIN );

	ccGlFilter* filter = plugin->getFilter();
	
	if ( filter != nullptr )
	{
		if ( m_glWindow->areGLFiltersEnabled() )
		{
			m_glWindow->setGlFilter( filter );
			
			ccLog::Print( "Note: go to << Display > Shaders & Filters > No filter >> to disable GL filter" );
		}
		else
		{
			ccLog::Error( "GL filters not supported" );
		}
	}
	else
	{
		ccLog::Error( "Can't load GL filter (an error occurred)!" );
	}
}

void ccViewer::doActionDeleteSelectedEntity()
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (!currentRoot)
		return;

	ccHObject::Container toCheck;
	toCheck.push_back(currentRoot);

	while (!toCheck.empty())
	{
		ccHObject* obj = toCheck.back();
		toCheck.pop_back();

		if (obj->isSelected())
		{
			if (obj->getParent())
			{
				obj->getParent()->addDependency(obj,ccHObject::DP_DELETE_OTHER); //we force deletion!
				obj->getParent()->removeChild(obj);
			}
			else
			{
				delete obj;
			}
		}
		else
		{
			for (unsigned i=0; i<obj->getChildrenNumber(); ++i)
				toCheck.push_back(obj->getChild(i));
		}
	}

	m_glWindow->redraw();
}

void ccViewer::selectEntity(ccHObject* toSelect)
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (!currentRoot)
		return;

	currentRoot->setSelected_recursive(false);
	ui.menuSelectSF->clear();
	ui.menuSelected->setEnabled(false);

	if (toSelect)
	{
		toSelect->setSelected(true);

		ui.actionShowColors->blockSignals(true);
		ui.actionShowNormals->blockSignals(true);
		ui.actionShowMaterials->blockSignals(true);
		ui.actionShowScalarField->blockSignals(true);
		ui.actionShowColorRamp->blockSignals(true);

		ui.actionShowColors->setEnabled(toSelect->hasColors());
		ui.actionShowColors->setChecked(toSelect->colorsShown());
		ui.actionShowNormals->setEnabled(toSelect->hasNormals());
		ui.actionShowNormals->setChecked(toSelect->normalsShown());

		if (toSelect->isKindOf(CC_TYPES::MESH))
		{
			ccGenericMesh* mesh = static_cast<ccGenericMesh*>(toSelect);
			ui.actionShowMaterials->setEnabled(mesh->hasMaterials());
			ui.actionShowMaterials->setChecked(mesh->materialsShown());
		}
		else
		{
			ui.actionShowMaterials->setEnabled(false);
			ui.actionShowMaterials->setChecked(false);
		}

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(toSelect);
		bool hasSF = (cloud ? cloud->hasScalarFields() : false);
		ui.actionShowScalarField->setEnabled(hasSF);
		ui.actionShowScalarField->setChecked(toSelect->sfShown());
		ui.actionShowColorRamp->setEnabled(hasSF);
		ui.actionShowColorRamp->setChecked(cloud ? cloud->sfColorScaleShown() && cloud->sfShown() : false);

		unsigned sfCount = (cloud ? cloud->getNumberOfScalarFields() : 0);
		ui.menuSelectSF->setEnabled(hasSF && sfCount>1);
		if (hasSF && sfCount>1)
		{
			int currentSFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
			//ui.menuSelectSF->clear();
			for (unsigned i = 0; i < sfCount; ++i)
			{
				QAction* action = ui.menuSelectSF->addAction(cloud->getScalarFieldName(i));
				action->setData(i);
				action->setCheckable(true);
				if (currentSFIndex == static_cast<int>(i))
					action->setChecked(true);
				connect(action, &QAction::toggled, this, &ccViewer::changeCurrentScalarField);
			}
		}

		ui.menuSelected->setEnabled(true);

		ui.actionShowColors->blockSignals(false);
		ui.actionShowNormals->blockSignals(false);
		ui.actionShowMaterials->blockSignals(false);
		ui.actionShowScalarField->blockSignals(false);
		ui.actionShowColorRamp->blockSignals(false);

		m_selectedObject = toSelect;
	}

	m_glWindow->redraw();
}

bool ccViewer::checkForLoadedEntities()
{
	bool loadedEntities = true;
	m_glWindow->displayNewMessage(QString(),ccGLWindow::SCREEN_CENTER_MESSAGE); //clear (any) message in the middle area

	if (!m_glWindow->getSceneDB())
	{
		m_glWindow->displayNewMessage("Drag & drop files on the 3D window to load them!",ccGLWindow::SCREEN_CENTER_MESSAGE,true,3600);
		loadedEntities = false;
	}

	if (m_glWindow->getDisplayParameters().displayCross != loadedEntities)
	{
		ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		params.displayCross = loadedEntities;
		m_glWindow->setDisplayParameters(params);
	}

	return loadedEntities;
}

void ccViewer::updateDisplay()
{
	updateGLFrameGradient();

	m_glWindow->redraw();
}

void ccViewer::updateGLFrameGradient()
{
	//display parameters
	static const ccColor::Rgbub s_black(0, 0, 0);
	static const ccColor::Rgbub s_white(255, 255, 255);
	bool stereoModeEnabled = m_glWindow->stereoModeIsEnabled();
	const ccColor::Rgbub& bkgCol = stereoModeEnabled ? s_black : m_glWindow->getDisplayParameters().backgroundCol;
	const ccColor::Rgbub& forCol = stereoModeEnabled ? s_white : m_glWindow->getDisplayParameters().pointsDefaultCol;

	//QOpenGLFunctions_2_1* glFunc = m_glWindow->context()->versionFunctions<QOpenGLFunctions_2_1>();
	//if (!glFunc)
	//{
	//	return;
	//}

	//glFunc->glColor3ubv(bkgCol.rgb);
	//glFunc->glColor3ub(255-forCol.r,255-forCol.g,255-forCol.b);

	QString styleSheet = QString("QFrame{border: 2px solid white; border-radius: 10px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1,stop:0 rgb(%1,%2,%3), stop:1 rgb(%4,%5,%6));}")
								.arg(bkgCol.r)
								.arg(bkgCol.g)
								.arg(bkgCol.b)
								.arg(255-forCol.r)
								.arg(255-forCol.g)
								.arg(255-forCol.b);
	
	ui.GLframe->setStyleSheet(styleSheet);
}

void ccViewer::addToDB(QStringList filenames)
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		m_selectedObject = nullptr;
		m_glWindow->setSceneDB(nullptr);
		m_glWindow->redraw();
		delete currentRoot;
		currentRoot = nullptr;
	}

	bool scaleAlreadyDisplayed = false;

	FileIOFilter::LoadParameters parameters;
	parameters.alwaysDisplayLoadDialog = false;
	parameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
	parameters.parentWidget = this;

	const ccOptions& options = ccOptions::Instance();
	FileIOFilter::ResetSesionCounter();

	for (int i = 0; i < filenames.size(); ++i)
	{
		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
		ccHObject* newGroup = FileIOFilter::LoadFromFile(filenames[i], parameters, result);

		if (newGroup)
		{
			if (!options.normalsDisplayedByDefault)
			{
				//disable the normals on all loaded clouds!
				ccHObject::Container clouds;
				newGroup->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
				for (ccHObject* cloud : clouds)
				{
					if (cloud)
					{
						static_cast<ccGenericPointCloud*>(cloud)->showNormals(false);
					}
				}
			}

			addToDB(newGroup);

			if (!scaleAlreadyDisplayed)
			{
				for (unsigned i = 0; i < newGroup->getChildrenNumber(); ++i)
				{
					ccHObject* ent = newGroup->getChild(i);
					if (ent->isA(CC_TYPES::POINT_CLOUD))
					{
						ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
						if (pc->hasScalarFields())
						{
							pc->setCurrentDisplayedScalarField(0);
							pc->showSFColorsScale(true);
							scaleAlreadyDisplayed=true;
						}
					}
					else if (ent->isKindOf(CC_TYPES::MESH))
					{
						ccGenericMesh* mesh = static_cast<ccGenericMesh*>(ent);
						if (mesh->hasScalarFields())
						{
							mesh->showSF(true);
							scaleAlreadyDisplayed=true;
							ccPointCloud* pc = static_cast<ccPointCloud*>(mesh->getAssociatedCloud());
							pc->showSFColorsScale(true);
						}
					}
				}
			}
		}
		
		if (result == CC_FERR_CANCELED_BY_USER)
		{
			//stop importing the file if the user has cancelled the current process!
			break;
		}
	}

	checkForLoadedEntities();
}

void ccViewer::addToDB(ccHObject* entity)
{
	assert(entity && m_glWindow);

	entity->setDisplay_recursive(m_glWindow);

	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		//already a pure 'root'
		if (currentRoot->isA(CC_TYPES::HIERARCHY_OBJECT))
		{
			currentRoot->addChild(entity);
		}
		else
		{
			ccHObject* root = new ccHObject("root");
			root->addChild(currentRoot);
			root->addChild(entity);
			m_glWindow->setSceneDB(root);
		}
	}
	else
	{
		m_glWindow->setSceneDB(entity);
	}

	checkForLoadedEntities();
}

void ccViewer::showDisplayParameters()
{
	ccDisplayOptionsDlg clmDlg(this);

	connect(&clmDlg, &ccDisplayOptionsDlg::aspectHasChanged, this, &ccViewer::updateDisplay);

	clmDlg.exec();

	disconnect(&clmDlg, nullptr, nullptr, nullptr);
}

void ccViewer::doActionEditCamera()
{
	if (!s_cpeDlg)
	{
		s_cpeDlg = new ccCameraParamEditDlg(this, nullptr);
		s_cpeDlg->linkWith(m_glWindow);
	}
	s_cpeDlg->show();
}

void ccViewer::reflectPerspectiveState()
{
	if ( m_glWindow == nullptr )
		return;
	
	bool objectCentered = false;
	bool perspectiveEnabled = m_glWindow->getPerspectiveState(objectCentered);

	ui.actionSetOrthoView->setChecked(!perspectiveEnabled);
	ui.actionSetCenteredPerspectiveView->setChecked(perspectiveEnabled && objectCentered);
	ui.actionSetViewerPerspectiveView->setChecked(perspectiveEnabled && !objectCentered);
}

bool ccViewer::checkStereoMode()
{
	if (	m_glWindow
		&&	m_glWindow->getViewportParameters().perspectiveView
		&&	m_glWindow->stereoModeIsEnabled())
	{
		if (QMessageBox::question(this,"Stereo mode", "Stereo-mode only works in perspective mode. Do you want to disable it?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
		{
			return false;
		}
		else
		{
			toggleStereoMode(false);
		}
	}

	return true;
}


void ccViewer::setOrthoView()
{
	if (m_glWindow)
	{
		if (!checkStereoMode())
			return;
		m_glWindow->setPerspectiveState(false,true);
		m_glWindow->redraw();
	}
	reflectPerspectiveState();
}

void ccViewer::setCenteredPerspectiveView()
{
	if (m_glWindow)
	{
		m_glWindow->setPerspectiveState(true,true);
		m_glWindow->redraw();
	}
	reflectPerspectiveState();
}

void ccViewer::setViewerPerspectiveView()
{
	if (m_glWindow)
	{
		m_glWindow->setPerspectiveState(true,false);
		m_glWindow->redraw();
	}
	reflectPerspectiveState();
}

void ccViewer::reflectPivotVisibilityState()
{
	if ( m_glWindow == nullptr )
		return;
	
	ccGLWindow::PivotVisibility vis = m_glWindow->getPivotVisibility();

	ui.actionSetPivotAlwaysOn->setChecked(vis == ccGLWindow::PIVOT_ALWAYS_SHOW);
	ui.actionSetPivotRotationOnly->setChecked(vis == ccGLWindow::PIVOT_SHOW_ON_MOVE);
	ui.actionSetPivotOff->setChecked(vis == ccGLWindow::PIVOT_HIDE);
}

void ccViewer::setPivotAlwaysOn()
{
	if (m_glWindow)
	{
		m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_ALWAYS_SHOW);
		m_glWindow->redraw();
	}
	reflectPivotVisibilityState();
}

void ccViewer::setPivotRotationOnly()
{
	if (m_glWindow)
	{
		m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_SHOW_ON_MOVE);
		m_glWindow->redraw();
	}
	reflectPivotVisibilityState();
}

void ccViewer::setPivotOff()
{
	if (m_glWindow)
	{
		m_glWindow->setPivotVisibility(ccGLWindow::PIVOT_HIDE);
		m_glWindow->redraw();
	}
	reflectPivotVisibilityState();
}

void ccViewer::reflectLightsState()
{
	if ( m_glWindow == nullptr )
		return;
	
	ui.actionToggleSunLight->blockSignals(true);
	ui.actionToggleCustomLight->blockSignals(true);

	ui.actionToggleSunLight->setChecked(m_glWindow->sunLightEnabled());
	ui.actionToggleCustomLight->setChecked(m_glWindow->customLightEnabled());

	ui.actionToggleSunLight->blockSignals(false);
	ui.actionToggleCustomLight->blockSignals(false);
}

void ccViewer::toggleSunLight(bool state)
{
	if (m_glWindow)
		m_glWindow->setSunLight(state);
	reflectLightsState();
}

void ccViewer::toggleCustomLight(bool state)
{
	if (m_glWindow)
		m_glWindow->setCustomLight(state);
	reflectLightsState();
}

void ccViewer::toggleStereoMode(bool state)
{
	if (!m_glWindow)
		return;

	bool isActive = m_glWindow->stereoModeIsEnabled();
	if (isActive == state)
	{
		//nothing to do
		return;
	}

	if (isActive)
	{
		m_glWindow->disableStereoMode();
		if (	m_glWindow->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
			||	m_glWindow->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
		{
			//disable full screen
			ui.actionFullScreen->setChecked(false);
		}
	}
	else
	{
		//display a parameters dialog
		ccStereoModeDlg smDlg(this);
		smDlg.setParameters(m_glWindow->getStereoParams());
		if (!smDlg.exec())
		{
			//cancelled by the user
			ui.actionEnableStereo->blockSignals(true);
			ui.actionEnableStereo->setChecked(false);
			ui.actionEnableStereo->blockSignals(false);
			return;
		}

		ccGLWindow::StereoParams params = smDlg.getParameters();
#ifndef CC_GL_WINDOW_USE_QWINDOW
		if (!params.isAnaglyph())
		{
			ccLog::Error("This version doesn't handle stereo glasses and headsets.\nUse the 'Stereo' version instead.");
			//activation of the stereo mode failed: cancel selection
			ui.actionEnableStereo->blockSignals(true);
			ui.actionEnableStereo->setChecked(false);
			ui.actionEnableStereo->blockSignals(false);
			return;
		}
#endif

		//force perspective state!
		if (!m_glWindow->getViewportParameters().perspectiveView)
		{
			m_glWindow->setPerspectiveState(true, true);
			reflectPerspectiveState();
		}

		if (	params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION
			||	params.glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
		{
			//force full screen
			ui.actionFullScreen->setChecked(true);
		}

		if (!m_glWindow->enableStereoMode(params))
		{
			//activation of the stereo mode failed: cancel selection
			ui.actionEnableStereo->blockSignals(true);
			ui.actionEnableStereo->setChecked(false);
			ui.actionEnableStereo->blockSignals(false);
		}
	}

	updateDisplay();
}

void ccViewer::toggleFullScreen(bool state)
{
	if (m_glWindow)
	{
		if (	m_glWindow->stereoModeIsEnabled()
			&&	(	m_glWindow->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
				||	m_glWindow->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
			)
		{
			//auto disable stereo mode as NVidia Vision only works in full screen mode!
			ui.actionEnableStereo->setChecked(false);
		}

		m_glWindow->toggleExclusiveFullScreen(state);
	}
}

void ccViewer::onExclusiveFullScreenToggled(bool state)
{
	ui.actionFullScreen->blockSignals(true);
	ui.actionFullScreen->setChecked(m_glWindow ? m_glWindow->exclusiveFullScreen() : false);
	ui.actionFullScreen->blockSignals(false);

	if (	!state
		&&	m_glWindow
		&&	m_glWindow->stereoModeIsEnabled()
		&&	(	m_glWindow->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION
			||	m_glWindow->getStereoParams().glassType == ccGLWindow::StereoParams::GENERIC_STEREO_DISPLAY)
		)
	{
		//auto disable stereo mode as NVidia Vision only works in full screen mode!
		ui.actionEnableStereo->setChecked(false);
	}
}

void ccViewer::toggleRotationAboutVertAxis()
{
	if (!m_glWindow)
		return;

	bool wasLocked = m_glWindow->isRotationAxisLocked();
	bool isLocked = !wasLocked;

	m_glWindow->lockRotationAxis(isLocked, CCVector3d(0.0, 0.0, 1.0));

	ui.actionLockRotationVertAxis->blockSignals(true);
	ui.actionLockRotationVertAxis->setChecked(isLocked);
	ui.actionLockRotationVertAxis->blockSignals(false);

	if (isLocked)
	{
		m_glWindow->displayNewMessage(QString("[ROTATION LOCKED]"), ccGLWindow::UPPER_CENTER_MESSAGE, false, 24 * 3600, ccGLWindow::ROTAION_LOCK_MESSAGE);
	}
	else
	{
		m_glWindow->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false, 0, ccGLWindow::ROTAION_LOCK_MESSAGE);
	}
	m_glWindow->redraw();
}

void ccViewer::doActionDisplayShortcuts()
{
	QMessageBox msgBox;
	QString text;
	text += "Shortcuts:\n\n";
	text += "F2 : Set orthographic view\n";
	text += "F3 : Set object-centered perspective\n";
	text += "F4 : Set viewer-based perspective\n";
	text += "F6 : Toggle sun light\n";
	text += "F7 : Toggle custom light\n";
	text += "F8 : Toggle Console display\n";
	text += "F9 : Toggle full screen\n";
	text += "F11: Toggle exclusive full screen\n";
	text += "Z  : Zoom on selected entity\n";
	text += "L  : Lock rotation around Z\n";
	text += "B  : Enter/leave bubble view mode\n";
	text += "DEL: Delete selected entity\n";
	text += "+  : Zoom in\n";
	text += "-  : Zoom out\n";
	text += "\n";
	text += "Shift + C: Toggle color ramp visibility\n";
	text += "Shift + up arrow: activate previous SF\n";
	text += "Shift + down arrow: activate next SF\n";
	text += "\n";
	text += "Ctrl + D: Display parameters\n";
	text += "Ctrl + C: Camera parameters\n";
	text += "\n";
	text += "Left click: Select entity\n";
	//text += "Ctrl + left click: Select multiple entities (toggle)\n";
	//text += "Alt + left button hold: Select multiple entities (rectangular area)\n";
	text += "Shift + left click (on a point/triangle): spawn a label\n";
	text += "Right click (on a label): expand/collapse\n";
	msgBox.setText(text);
	msgBox.exec();
}

void ccViewer::setTopView()
{
	m_glWindow->setView(CC_TOP_VIEW);
}

void ccViewer::setBottomView()
{
	m_glWindow->setView(CC_BOTTOM_VIEW);
}

void ccViewer::setFrontView()
{
	m_glWindow->setView(CC_FRONT_VIEW);
}

void ccViewer::setBackView()
{
	m_glWindow->setView(CC_BACK_VIEW);
}

void ccViewer::setLeftView()
{
	m_glWindow->setView(CC_LEFT_VIEW);
}

void ccViewer::setRightView()
{
	m_glWindow->setView(CC_RIGHT_VIEW);
}

void ccViewer::setIsoView1()
{
	m_glWindow->setView(CC_ISO_VIEW_1);
}

void ccViewer::setIsoView2()
{
	m_glWindow->setView(CC_ISO_VIEW_2);
}

void ccViewer::toggleColorsShown(bool state)
{
	if (!m_selectedObject)
		return;

	m_selectedObject->showColors(state);
	m_glWindow->redraw();
}

void ccViewer::toggleNormalsShown(bool state)
{
	if (!m_selectedObject)
		return;

	m_selectedObject->showNormals(state);
	m_glWindow->redraw();
}

void ccViewer::toggleMaterialsShown(bool state)
{
	if (m_selectedObject && m_selectedObject->isKindOf(CC_TYPES::MESH))
	{
		static_cast<ccGenericMesh*>(m_selectedObject)->showMaterials(state);
		m_glWindow->redraw();
	}
}

void ccViewer::toggleScalarShown(bool state)
{
	if (!m_selectedObject)
		return;

	m_selectedObject->showSF(state);
	m_glWindow->redraw();
}

void ccViewer::toggleColorbarShown(bool state)
{
	if (!m_selectedObject)
		return;

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedObject);
	if (!cloud)
		return;
	cloud->showSFColorsScale(state);
	m_glWindow->redraw(true, false);
}

void ccViewer::changeCurrentScalarField(bool state)
{
	if (!m_selectedObject)
		return;

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedObject);
	if (!cloud)
		return;

	QAction* action = qobject_cast<QAction*>(QObject::sender());
	if (!action)
		return;

	//disable all other actions
	const QObjectList& children = ui.menuSelectSF->children();
	for (int i = 0; i < children.size(); ++i)
	{
		QAction* act = static_cast<QAction*>(children[i]);
		act->blockSignals(true);
		act->setChecked(act == action);
		act->blockSignals(false);
	}

	int sfIndex = action->data().toInt();
	if (sfIndex < static_cast<int>(cloud->getNumberOfScalarFields()))
	{
		cloud->setCurrentDisplayedScalarField(sfIndex);
		//when 'setCurrentDisplayedScalarField' is called, scalar field is automatically shown!
		ui.actionShowScalarField->blockSignals(true);
		ui.actionShowScalarField->setChecked(true);
		ui.actionShowScalarField->blockSignals(false);
		m_glWindow->redraw();
	}
}

void ccViewer::setGlobalZoom()
{
	if (m_glWindow)
		m_glWindow->zoomGlobal();
}

void ccViewer::zoomOnSelectedEntity()
{
	if (!m_glWindow || !m_selectedObject)
		return;

	ccBBox box = m_selectedObject->getDisplayBB_recursive(false, m_glWindow);
	m_glWindow->updateConstellationCenterAndZoom(&box);
	m_glWindow->redraw();
}

#include <ui_ccviewerAbout.h>

void ccViewer::doActionAbout()
{
	QDialog aboutDialog(this);

	Ui::AboutDialog ui;
	ui.setupUi(&aboutDialog);
	ui.textEdit->setHtml(ui.textEdit->toHtml().arg(ccApp->versionLongStr( true )));

	aboutDialog.exec();
}

/*** 3D MOUSE SUPPORT ***/

void ccViewer::release3DMouse()
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_3dMouseInput)
	{
		m_3dMouseInput->disconnect(); //disconnect from the driver
		disconnect(m_3dMouseInput); //disconnect from Qt ;)

		delete m_3dMouseInput;
		m_3dMouseInput = 0;
	}
#endif
}

void ccViewer::enable3DMouse(bool state)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_3dMouseInput)
		release3DMouse();

	if (state)
	{
		m_3dMouseInput = new Mouse3DInput(this);
		if (m_3dMouseInput->connect(this,"ccViewer"))
		{
			QObject::connect(m_3dMouseInput, &Mouse3DInput::sigMove3d,				this,	&ccViewer::on3DMouseMove);
			QObject::connect(m_3dMouseInput, &Mouse3DInput::sigReleased,			this,	&ccViewer::on3DMouseReleased);
			QObject::connect(m_3dMouseInput, &Mouse3DInput::sigOn3dmouseKeyDown,	this,	&ccViewer::on3DMouseKeyDown);
			QObject::connect(m_3dMouseInput, &Mouse3DInput::sigOn3dmouseKeyUp,		this,	&ccViewer::on3DMouseKeyUp);
			QObject::connect(m_3dMouseInput, &Mouse3DInput::sigOn3dmouseCMDKeyDown, this,	&ccViewer::on3DMouseCMDKeyDown);
			QObject::connect(m_3dMouseInput, &Mouse3DInput::sigOn3dmouseCMDKeyUp,	this,	&ccViewer::on3DMouseCMDKeyUp);
		}
		else
		{
			delete m_3dMouseInput;
			m_3dMouseInput = 0;
			
			ccLog::Warning("[3D Mouse] No device found");
			state = false;
		}
	}
	else
	{
		ccLog::Warning("[3D Mouse] Device has been disabled");
	}
#else
	state = false;
#endif

	ui.actionEnable3DMouse->blockSignals(true);
	ui.actionEnable3DMouse->setChecked(state);
	ui.actionEnable3DMouse->blockSignals(false);
}

void ccViewer::on3DMouseKeyUp(int)
{
	//nothing right now
}

// ANY CHANGE/BUG FIX SHOULD BE REFLECTED TO THE EQUIVALENT METHODS IN QCC "MainWindow.cpp" FILE!
void ccViewer::on3DMouseKeyDown(int key)
{
#ifdef CC_3DXWARE_SUPPORT

	switch(key)
	{
	case Mouse3DInput::V3DK_MENU:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_FIT:
		{
			if (m_selectedObject)
				zoomOnSelectedEntity();
			else
				setGlobalZoom();
		}
		break;
	case Mouse3DInput::V3DK_TOP:
		setTopView();
		break;
	case Mouse3DInput::V3DK_LEFT:
		setLeftView();
		break;
	case Mouse3DInput::V3DK_RIGHT:
		setRightView();
		break;
	case Mouse3DInput::V3DK_FRONT:
		setFrontView();
		break;
	case Mouse3DInput::V3DK_BOTTOM:
		setBottomView();
		break;
	case Mouse3DInput::V3DK_BACK:
		setBackView();
		break;
	case Mouse3DInput::V3DK_ROTATE:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_PANZOOM:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_ISO1:
		setIsoView1();
		break;
	case Mouse3DInput::V3DK_ISO2:
		setIsoView2();
		break;
	case Mouse3DInput::V3DK_PLUS:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_MINUS:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_DOMINANT:
		//should be handled by the driver now!
		break;
	case Mouse3DInput::V3DK_CW:
	case Mouse3DInput::V3DK_CCW:
		{
			if (m_glWindow)
			{
				CCVector3d axis(0,0,-1);
				CCVector3d trans(0,0,0);
				ccGLMatrixd mat;
				double angle = M_PI/2;
				if (key == Mouse3DInput::V3DK_CCW)
					angle = -angle;
				mat.initFromParameters(angle,axis,trans);
				m_glWindow->rotateBaseViewMat(mat);
				m_glWindow->redraw();
			}
		}
		break;
	case Mouse3DInput::V3DK_ESC:
	case Mouse3DInput::V3DK_ALT:
	case Mouse3DInput::V3DK_SHIFT:
	case Mouse3DInput::V3DK_CTRL:
	default:
		ccLog::Warning("[3D mouse] This button is not handled (yet)");
		//TODO
		break;
	}

#endif
}
void ccViewer::on3DMouseCMDKeyUp(int cmd)
{
	//nothing right now
}

void ccViewer::on3DMouseCMDKeyDown(int cmd)
{
#ifdef CC_3DXWARE_SUPPORT
	switch (cmd)
	{
		//ccLog::Print(QString("on3DMouseCMDKeyDown Cmd = %1").arg(cmd));
	case Mouse3DInput::V3DCMD_VIEW_FIT:
	{
		if (m_selectedObject)
			zoomOnSelectedEntity();
		else
			setGlobalZoom();
	}
	break;
	case Mouse3DInput::V3DCMD_VIEW_TOP:
		setTopView();
		break;
	case Mouse3DInput::V3DCMD_VIEW_LEFT:
		setLeftView();
		break;
	case Mouse3DInput::V3DCMD_VIEW_RIGHT:
		setRightView();
		break;
	case Mouse3DInput::V3DCMD_VIEW_FRONT:
		setFrontView();
		break;
	case Mouse3DInput::V3DCMD_VIEW_BOTTOM:
		setBottomView();
		break;
	case Mouse3DInput::V3DCMD_VIEW_BACK:
		setBackView();
		break;
	case Mouse3DInput::V3DCMD_VIEW_ISO1:
		setIsoView1();
		break;
	case Mouse3DInput::V3DCMD_VIEW_ISO2:
		setIsoView2();
		break;
	case Mouse3DInput::V3DCMD_VIEW_ROLLCW:
	case Mouse3DInput::V3DCMD_VIEW_ROLLCCW:
	{
		if (m_glWindow)
		{
			CCVector3d axis(0, 0, -1);
			CCVector3d trans(0, 0, 0);
			ccGLMatrixd mat;
			double angle = M_PI / 2;
			if (cmd == Mouse3DInput::V3DCMD_VIEW_ROLLCCW)
				angle = -angle;
			mat.initFromParameters(angle, axis, trans);
			m_glWindow->rotateBaseViewMat(mat);
			m_glWindow->redraw();
		}
	}
	break;
	case Mouse3DInput::V3DCMD_VIEW_SPINCW:
	case Mouse3DInput::V3DCMD_VIEW_SPINCCW:
	{
		if (m_glWindow)
		{
			CCVector3d axis(0, 1, 0);
			CCVector3d trans(0, 0, 0);
			ccGLMatrixd mat;
			double angle = M_PI / 2;
			if (cmd == Mouse3DInput::V3DCMD_VIEW_SPINCCW)
				angle = -angle;
			mat.initFromParameters(angle, axis, trans);
			m_glWindow->rotateBaseViewMat(mat);
			m_glWindow->redraw();
		}
	}
	case Mouse3DInput::V3DCMD_VIEW_TILTCW:
	case Mouse3DInput::V3DCMD_VIEW_TILTCCW:
	{
		if (m_glWindow)
		{
			CCVector3d axis(1, 0, 0);
			CCVector3d trans(0, 0, 0);
			ccGLMatrixd mat;
			double angle = M_PI / 2;
			if (cmd == Mouse3DInput::V3DCMD_VIEW_TILTCCW)
				angle = -angle;
			mat.initFromParameters(angle, axis, trans);
			m_glWindow->rotateBaseViewMat(mat);
			m_glWindow->redraw();
		}
	}
	break;
	default:
		ccLog::Warning("[3D mouse] This button is not handled (yet)");
		//TODO
		break;
	}
#endif
}

void ccViewer::on3DMouseMove(std::vector<float>& vec)
{
#ifdef CC_3DXWARE_SUPPORT
	if (m_glWindow)
		Mouse3DInput::Apply(vec,m_glWindow);
#endif
}

void ccViewer::on3DMouseReleased()
{
	//active window?
	if (m_glWindow && m_glWindow->getPivotVisibility() == ccGLWindow::PIVOT_SHOW_ON_MOVE)
	{
		//we have to hide the pivot symbol!
		m_glWindow->showPivotSymbol(false);
		m_glWindow->redraw();
	}
}
