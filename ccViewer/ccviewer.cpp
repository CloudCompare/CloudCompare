//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE LIGHT VIEWER                            #
//#                                                                        #
//#  This project has been initated under funding from ANR/CIFRE           #
//#  This program is free software; you can redistribute it and/or modify  #
//#                                                                        #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#      +++ COPYRIGHT: EDF R&D + TELECOM ParisTech (ENST-TSI) +++         #
//#                                                                        #
//##########################################################################

#include "ccviewer.h"

//Qt
#include <QVBoxLayout>
#include <QMessageBox>

//qCC
#include <ccGLWindow.h>
#include <fileIO/FileIOFilter.h>
#include <ccGuiParameters.h>

//dialogs
#include <ccDisplayOptionsDlg.h>
#include <ccCameraParamEditDlg.h>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccHObject.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>

//system
#include <assert.h>

//! Current version
const double CC_VIEWER_VERSION = 1.23;

//Camera parameters dialog
ccCameraParamEditDlg* s_cpeDlg = 0;

ccViewer::ccViewer(QWidget *parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags)
	, m_glWindow(0)
	, m_selectedObject(0)
{
	ui.setupUi(this);

	//insert GL window in a vertical layout
	QVBoxLayout* verticalLayout_2 = new QVBoxLayout(ui.GLframe);
	verticalLayout_2->setSpacing(0);
	const int margin = 10;
    verticalLayout_2->setContentsMargins(margin,margin,margin,margin);
    QGLFormat format;
    format.setSwapInterval(0);
	m_glWindow = new ccGLWindow(ui.GLframe,format);
	verticalLayout_2->addWidget(m_glWindow);

	updateGLFrameGradient();

	//UI/display synchronization
	ui.actionToggleSunLight->setChecked(m_glWindow->sunLightEnabled());
	ui.actionToggleCustomLight->setChecked(m_glWindow->customLightEnabled());
	bool perspective,centered;
	perspective = m_glWindow->getPerspectiveState(centered);
	ui.actionDisablePerspective->setChecked(!perspective);
	ui.actionToggleCenteredPerspective->setChecked(perspective && centered);
	ui.actionToggleViewerBasedPerspective->setChecked(perspective && !centered);
	ui.actionFullScreen->setChecked(false);

	ui.menuSelected->setEnabled(false);

	//Signals & slots connection
	connect(m_glWindow,								SIGNAL(filesDropped(const QStringList&)),	this,	SLOT(addToDB(const QStringList&)));
	connect(m_glWindow,								SIGNAL(entitySelectionChanged(int)),		this,	SLOT(selectEntity(int)));
	//connect(m_glWindow,								SIGNAL(newLabel(ccHObject*),				this,	SLOT(handleNewEntity(ccHObject*))); //nothing to do in ccViewer!

	//"Options" menu
	connect(ui.actionDisplayParameters,				SIGNAL(triggered()),						this,	SLOT(showDisplayParameters()));
    connect(ui.actionEditCamera,					SIGNAL(triggered()),    					this,	SLOT(doActionEditCamera()));
    //"Display > Standard views" menu
    connect(ui.actionSetViewTop,					SIGNAL(triggered()),						this,	SLOT(setTopView()));
    connect(ui.actionSetViewBottom,					SIGNAL(triggered()),						this,	SLOT(setBottomView()));
    connect(ui.actionSetViewFront,					SIGNAL(triggered()),						this,	SLOT(setFrontView()));
    connect(ui.actionSetViewBack,					SIGNAL(triggered()),						this,	SLOT(setBackView()));
    connect(ui.actionSetViewLeft,					SIGNAL(triggered()),						this,	SLOT(setLeftView()));
    connect(ui.actionSetViewRight,					SIGNAL(triggered()),						this,	SLOT(setRightView()));
    //"Display > Lights & Materials" menu
    connect(ui.actionToggleSunLight,				SIGNAL(toggled(bool)),    					this,	SLOT(toggleSunLight(bool)));
    connect(ui.actionToggleCustomLight,				SIGNAL(toggled(bool)),    					this,	SLOT(toggleCustomLight(bool)));
	//"Options > Selected" menu
	connect(ui.actionShowColors,					SIGNAL(toggled(bool)),    					this,	SLOT(toggleColorsShown(bool)));
	connect(ui.actionShowNormals,					SIGNAL(toggled(bool)),    					this,	SLOT(toggleNormalsShown(bool)));
	connect(ui.actionShowScalarField,				SIGNAL(toggled(bool)),    					this,	SLOT(toggleScalarShown(bool)));
	connect(ui.actionShowColorRamp,					SIGNAL(toggled(bool)),    					this,	SLOT(toggleColorbarShown(bool)));
    connect(ui.actionDelete,						SIGNAL(triggered()),						this,	SLOT(doActionDeleteSelectedEntity()));
	//"Options > Perspective" menu
    connect(ui.actionDisablePerspective,			SIGNAL(toggled(bool)),						this,	SLOT(togglePerspectiveOff(bool)));
    connect(ui.actionToggleCenteredPerspective,		SIGNAL(toggled(bool)),						this,	SLOT(toggleCenteredPerspective(bool)));
    connect(ui.actionToggleViewerBasedPerspective,	SIGNAL(toggled(bool)),						this,	SLOT(toggleViewerBasedPerspective(bool)));
	//"Options" menu
	connect(ui.actionFullScreen,					SIGNAL(toggled(bool)),						this,	SLOT(toggleFullScreen(bool)));

    //"Help" menu
    connect(ui.actionAbout,							SIGNAL(triggered()),						this,	SLOT(doActionAbout()));
    connect(ui.actionHelpShortctus,					SIGNAL(triggered()),						this,	SLOT(doActionDisplayShortcuts()));
}

ccViewer::~ccViewer()
{
	if (s_cpeDlg)
	{
		delete s_cpeDlg;
		s_cpeDlg=0;
	}

	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		m_glWindow->setSceneDB(0);
		m_glWindow->redraw();
		delete currentRoot;
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
				obj->getParent()->removeChild(obj);
			if (!obj->getFlagState(CC_FATHER_DEPENDANT))
				delete obj;
		}
		else
		{
			for (unsigned i=0;i<obj->getChildrenNumber();++i)
				toCheck.push_back(obj->getChild(i));
		}
	}

	m_glWindow->redraw();
}

void ccViewer::selectEntity(int uniqueID)
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (!currentRoot)
		return;

	currentRoot->setSelected_recursive(false);
	ui.menuSelectSF->clear();
	ui.menuSelected->setEnabled(false);

	ccHObject* toSelect = currentRoot->find(uniqueID);
	if (toSelect)
	{
		toSelect->setSelected(true);

		ui.actionShowColors->blockSignals(true);
		ui.actionShowNormals->blockSignals(true);
		ui.actionShowScalarField->blockSignals(true);
		ui.actionShowColorRamp->blockSignals(true);

		ui.actionShowColors->setEnabled(toSelect->hasColors());
		ui.actionShowColors->setChecked(toSelect->colorsShown());
		ui.actionShowNormals->setEnabled(toSelect->hasNormals());
		ui.actionShowNormals->setChecked(toSelect->normalsShown());

		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(toSelect);
		bool hasSF = (cloud ? cloud->hasScalarFields() : false);
		ui.actionShowScalarField->setEnabled(hasSF);
		ui.actionShowScalarField->setChecked(toSelect->sfShown());
		ui.actionShowColorRamp->setEnabled(hasSF);
		ui.actionShowColorRamp->setChecked(cloud ? cloud->sfColorScaleShown() : false);

		unsigned sfCount = (cloud ? cloud->getNumberOfScalarFields() : 0);
		ui.menuSelectSF->setEnabled(hasSF && sfCount>1);
		if (hasSF && sfCount>1)
		{
			int currentSFIndex = cloud->getCurrentDisplayedScalarFieldIndex();
			//ui.menuSelectSF->clear();
			for (unsigned i=0;i<sfCount;++i)
			{
				QAction* action = ui.menuSelectSF->addAction(cloud->getScalarFieldName(i));
				action->setData(i);
				action->setCheckable(true);
				if (currentSFIndex == (int)i)
					action->setChecked(true);
				connect(action, SIGNAL(toggled(bool)), this, SLOT(changeCurrentScalarField(bool)));
			}
		}

		ui.menuSelected->setEnabled(true);

		ui.actionShowColors->blockSignals(false);
		ui.actionShowNormals->blockSignals(false);
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

    if (ccGui::Parameters().displayCross != loadedEntities)
    {
        ccGui::ParamStruct params = ccGui::Parameters();
        params.displayCross = loadedEntities;
        ccGui::Set(params);
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
    const unsigned char* bkgCol = ccGui::Parameters().backgroundCol;
    const unsigned char* forCol = ccGui::Parameters().pointsDefaultCol;

    glColor3ubv(bkgCol);
    glColor3ub(255-forCol[0],255-forCol[1],255-forCol[2]);

	QString styleSheet = QString("QFrame{border: 2px solid white; border-radius: 10px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1,stop:0 rgb(%1,%2,%3), stop:1 rgb(%4,%5,%6));}").arg(bkgCol[0]).arg(bkgCol[1]).arg(bkgCol[2]).arg(255-forCol[0]).arg(255-forCol[1]).arg(255-forCol[2]);
	ui.GLframe->setStyleSheet(styleSheet);
}

void ccViewer::addToDB(const QStringList& filenames)
{
	ccHObject* currentRoot = m_glWindow->getSceneDB();
	if (currentRoot)
	{
		m_selectedObject = 0;
		m_glWindow->setSceneDB(0);
		m_glWindow->redraw();
		delete currentRoot;
		currentRoot=0;
	}

	bool scaleAlreadyDisplayed=false;

	for (int i=0;i<filenames.size();++i)
	{
		ccHObject* newEntities = FileIOFilter::LoadFromFile(filenames[i],UNKNOWN_FILE,false);

		if (newEntities)
		{
			addToDB(newEntities);

			if (!scaleAlreadyDisplayed)
			{
				for (unsigned i=0;i<newEntities->getChildrenNumber();++i)
				{
					ccHObject* ent = newEntities->getChild(i);
					if (ent->isA(CC_POINT_CLOUD))
					{
						ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
						if (pc->hasScalarFields())
						{
							pc->setCurrentDisplayedScalarField(0);
							pc->showSFColorsScale(true);
							scaleAlreadyDisplayed=true;
						}
					}
					else if (ent->isKindOf(CC_MESH))
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
		if (currentRoot->isA(CC_HIERARCHY_OBJECT))
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

	connect(&clmDlg, SIGNAL(aspectHasChanged()), this, SLOT(updateDisplay()));

	clmDlg.exec();

	disconnect(&clmDlg,0,0,0);
}

void ccViewer::setPerspective(bool enabled, bool centered)
{
	ui.actionDisablePerspective->blockSignals(true);
	ui.actionToggleCenteredPerspective->blockSignals(true);
	ui.actionToggleViewerBasedPerspective->blockSignals(true);

	ui.actionDisablePerspective->setChecked(!enabled);
	ui.actionToggleCenteredPerspective->setChecked(enabled && centered);
	ui.actionToggleViewerBasedPerspective->setChecked(enabled && !centered);

	ui.actionDisablePerspective->blockSignals(false);
	ui.actionToggleCenteredPerspective->blockSignals(false);
	ui.actionToggleViewerBasedPerspective->blockSignals(false);

	m_glWindow->setPerspectiveState(enabled,centered);
	m_glWindow->redraw();
}

void ccViewer::doActionEditCamera()
{
	if (!s_cpeDlg)
	{
		s_cpeDlg = new ccCameraParamEditDlg(0);
		s_cpeDlg->linkWith(m_glWindow);
	}
	s_cpeDlg->show();
}

void ccViewer::toggleSunLight(bool state)
{
	m_glWindow->setSunLight(state);
}

void ccViewer::toggleCustomLight(bool state)
{
	m_glWindow->setCustomLight(state);
}

void ccViewer::togglePerspectiveOff(bool)
{
	setPerspective(false,false);
}

void ccViewer::toggleCenteredPerspective(bool)
{
	setPerspective(true,true);
}

void ccViewer::toggleViewerBasedPerspective(bool)
{
	setPerspective(true,false);
}

void ccViewer::toggleFullScreen(bool state)
{
    if (state)
        showFullScreen();
    else
        showNormal();
    m_glWindow->redraw();
}

void ccViewer::doActionDisplayShortcuts()
{
	QMessageBox msgBox;
	QString text;
	text += "Shortcuts:\n\n";
	text += "F2 : Disable perspective\n";
	text += "F3 : Centered perspective\n";
	text += "F4 : Viewer based perspective\n";
	text += "F6 : Toggle sun light\n";
	text += "F7 : Toggle custom light\n";
	text += "F11: Toggle full screen\n";
	text += "DEL: Delete selected entity\n";
	text += "\n";
	text += "Ctrl+D: Display parameters\n";
	text += "Ctrl+C: Camera parameters\n";
	text += "\n";
	text += "Left click: Select entity\n";
	text += "Ctrl + left click: Select multiple entities (toggle)\n";
	text += "Shift + left click (on a 3D point): Display label\n";
	text += "Right click (on a label): (un)collapse\n";
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
	m_glWindow->updateGL();
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
	for (int i=0;i<children.size();++i)
	{
		QAction* act = static_cast<QAction*>(children[i]);
		act->blockSignals(true);
		act->setChecked(act == action);
		act->blockSignals(false);
	}

	int sfIndex = action->data().toInt();
	if (sfIndex < (int)cloud->getNumberOfScalarFields())
	{
		cloud->setCurrentDisplayedScalarField(sfIndex);
		//when 'setCurrentDisplayedScalarField' is called, scalar field is automatically shown!
		ui.actionShowScalarField->blockSignals(true);
		ui.actionShowScalarField->setChecked(true);
		ui.actionShowScalarField->blockSignals(false);
		m_glWindow->redraw();
	}
}

#include <ui_ccviewerAbout.h>

void ccViewer::doActionAbout()
{
	QDialog aboutDialog(this);

	Ui::AboutDialog ui;
	ui.setupUi(&aboutDialog);
	ui.textEdit->append(QString("Version: %1").arg(CC_VIEWER_VERSION,0,'f',2));

	aboutDialog.exec();
}
