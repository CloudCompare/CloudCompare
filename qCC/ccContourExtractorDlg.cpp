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

#include "ccContourExtractorDlg.h"

//CCLib
#include <CCPlatform.h>

//qCC_gl
#include <ccGLWidget.h>

//Qt
#include <QCoreApplication>

//system
#include <assert.h>
#if defined(CC_WINDOWS)
#include <windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif

ccContourExtractorDlg::ccContourExtractorDlg(QWidget* parent/*=0*/)
	: QDialog(parent, Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint)
	, Ui::ContourExtractorDlg()
	, m_skipped(false)
	, m_glWindow(nullptr)
{
	setupUi(this);
}

void ccContourExtractorDlg::init()
{
	if (m_glWindow)
	{
		//already initialized
		assert(false);
		return;
	}

	connect(nextPushButton, &QAbstractButton::clicked, &m_loop, &QEventLoop::quit);
	connect(skipPushButton, &QAbstractButton::clicked, this, &ccContourExtractorDlg::onSkipButtonClicked);
	nextPushButton->setFocus();

	//create 3D window
	{
		QWidget* glWidget = nullptr;
		CreateGLWindow(m_glWindow, glWidget, false, true);
		assert(m_glWindow && glWidget);

		ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		//black (text) & white (background) display by default
		params.backgroundCol = ccColor::white;
		params.textDefaultCol = ccColor::black;
		params.pointsDefaultCol = ccColor::black;
		params.drawBackgroundGradient = false;
		params.decimateMeshOnMove = false;
		params.displayCross = false;
		params.colorScaleUseShader = false;
		m_glWindow->setDisplayParameters(params,true);
		m_glWindow->setPerspectiveState(false,true);
		m_glWindow->setInteractionMode(ccGLWindow::INTERACT_PAN | ccGLWindow::INTERACT_ZOOM_CAMERA | ccGLWindow::INTERACT_CLICKABLE_ITEMS);
		m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
		m_glWindow->displayOverlayEntities(true);
		viewFrame->setLayout(new QHBoxLayout);
		viewFrame->layout()->addWidget(glWidget);
	}
}

void ccContourExtractorDlg::zoomOn(const ccBBox& box)
{
	if (!m_glWindow)
		return;

	float pixSize = std::max(box.getDiagVec().x / std::max(20, m_glWindow->glWidth() - 20), box.getDiagVec().y / std::max(20, m_glWindow->glHeight() - 20));
	m_glWindow->setPixelSize(pixSize);
	m_glWindow->setCameraPos(CCVector3d::fromArray(box.getCenter().u));
}

bool ccContourExtractorDlg::isSkipped() const
{
	return skipPushButton->isChecked();
}

void ccContourExtractorDlg::addToDisplay(ccHObject* obj, bool noDependency/*=true*/)
{
	if (m_glWindow && obj)
	{
		m_glWindow->addToOwnDB(obj,noDependency);
	}
}

void ccContourExtractorDlg::removFromDisplay(ccHObject* obj)
{
	if (m_glWindow && obj)
	{
		m_glWindow->removeFromOwnDB(obj);
	}
}

void ccContourExtractorDlg::refresh()
{
	if (m_skipped)
		return;
	if (m_glWindow)
	{
		m_glWindow->redraw();
		QCoreApplication::processEvents();
	}
}

void ccContourExtractorDlg::displayMessage(QString message, bool waitForUserConfirmation/*=false*/)
{
	if (m_skipped)
		return;
	messageLabel->setText(message);
	if (waitForUserConfirmation)
		waitForUser(20);
}

void ccContourExtractorDlg::onSkipButtonClicked()
{
	m_skipped = true;
	hide();
	QCoreApplication::processEvents();
}

void ccContourExtractorDlg::waitForUser(unsigned defaultDelay_ms/*=100*/)
{
	if (m_skipped)
		return;

	if (autoCheckBox->isChecked())
	{
		//simply wait a pre-determined time
#if defined(CC_WINDOWS)
		::Sleep(defaultDelay_ms);
#else
		usleep(defaultDelay_ms * 1000);
#endif
	}
	else
	{
		setModal(true);
		//wait for the user to click on the 'Next' button
		m_loop.exec();
		setModal(false);
		//exec();
	}
}
