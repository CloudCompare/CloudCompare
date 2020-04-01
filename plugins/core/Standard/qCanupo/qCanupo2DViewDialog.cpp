//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

#include "qCanupo2DViewDialog.h"

//local
#include "qCanupoTools.h"

//qCC_plugins
#include "ccMainAppInterface.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccSphere.h>

//qCC_gl
#include <ccGLWidget.h>

//Qt
#include <QSettings>
#include <QMainWindow>
#include <QFileInfo>
#include <QFileDialog>
#include <QPushButton>
#include <QApplication>
#include <QMessageBox>

//system
#include <limits>
#include <assert.h>

static bool s_firstDisplay = true;

qCanupo2DViewDialog::qCanupo2DViewDialog(	const CorePointDescSet* descriptors1,
											const CorePointDescSet* descriptors2,
											QString cloud1Name,
											QString cloud2Name,
											int class1/*=1*/,
											int class2/*=2*/,
											const CorePointDescSet* evaluationDescriptors/*=0*/,
											ccMainAppInterface* app/*=0*/)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::Canupo2DViewDialog()
	, m_app(app)
	, m_glWindow(nullptr)
	, m_classifierSaved(false)
	, m_descriptors1(descriptors1)
	, m_descriptors2(descriptors2)
	, m_evaluationDescriptors(evaluationDescriptors)
	, m_class1(class1)
	, m_cloud1Name(cloud1Name)
	, m_cloud2Name(cloud2Name)
	, m_class2(class2)
	, m_cloud(nullptr)
	, m_poly(nullptr)
	, m_polyVertices(nullptr)
	, m_selectedPointIndex(-1)
	, m_pickingRadius(5)
{
	setupUi(this);

	//update legend
	cloud1NameLabel->setText(QString("class %1: ").arg(m_class1) + m_cloud1Name); //blue points
	cloud2NameLabel->setText(QString("class %1: ").arg(m_class2) + m_cloud2Name); //red points
	
	s_firstDisplay = true;

	//setup 2D view
	{
		QWidget* glWidget = nullptr;
		m_app->createGLWindow(m_glWindow, glWidget);
		assert(m_glWindow && glWidget);

		ccGui::ParamStruct params = m_glWindow->getDisplayParameters();
		params.backgroundCol = ccColor::white;
		params.textDefaultCol = ccColor::black;
		params.pointsDefaultCol = ccColor::black;
		params.drawBackgroundGradient = false;
		params.displayCross = false;
		m_glWindow->setDisplayParameters(params,true);
		m_glWindow->setPerspectiveState(false,true);
		m_glWindow->setInteractionMode(ccGLWindow::PAN_ONLY() | ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		m_glWindow->setPickingMode(ccGLWindow::NO_PICKING);
		m_glWindow->displayOverlayEntities(false);
		//add window to the dedicated layout
		viewFrame->setLayout(new QHBoxLayout());
		viewFrame->layout()->addWidget(glWidget);

		connect(m_glWindow, SIGNAL(leftButtonClicked(int, int)),			this, SLOT(addOrSelectPoint(int, int)));
		connect(m_glWindow, SIGNAL(rightButtonClicked(int, int)),			this, SLOT(removePoint(int, int)));
		connect(m_glWindow, SIGNAL(mouseMoved(int, int, Qt::MouseButtons)),	this, SLOT(moveSelectedPoint(int, int, Qt::MouseButtons)));
		connect(m_glWindow, SIGNAL(buttonReleased()),						this, SLOT(deselectPoint()));
	}

	updateScalesList(true);

	//loadParamsFromPersistentSettings();
	connect(resetToolButton,		SIGNAL(clicked()),			this, SLOT(resetBoundary()));
	connect(statisticsToolButton,	SIGNAL(clicked()),			this, SLOT(computeStatistics()));
	connect(savePushButton,			SIGNAL(clicked()),			this, SLOT(saveClassifier()));
	connect(donePushButton,			SIGNAL(clicked()),			this, SLOT(checkBeforeAccept()));
	connect(pointSizeSpinBox,		SIGNAL(valueChanged(int)),	this, SLOT(setPointSize(int)));
	connect(scalesCountSpinBox,		SIGNAL(valueChanged(int)),	this, SLOT(onScalesCountSpinBoxChanged(int)));
}

qCanupo2DViewDialog::~qCanupo2DViewDialog()
{
	reset();

	if (m_app && m_glWindow)
	{
		m_app->destroyGLWindow(m_glWindow);
	}
}

void qCanupo2DViewDialog::updateScalesList(bool firstTime)
{
	//update list of scales
	if (!m_descriptors1 || !m_descriptors2)
	{
		scalesListLineEdit->setText("Invalid descriptors!");
		scalesCountSpinBox->setEnabled(false);
	}
	else
	{
		const std::vector<float>& allScales = m_descriptors1->scales();
		int maxScaleCount = static_cast<int>(allScales.size());
		scalesCountSpinBox->setRange(1,maxScaleCount);
		if (firstTime)
			scalesCountSpinBox->setValue(maxScaleCount);

		int currentScaleCount = scalesCountSpinBox->value();
		QStringList scalesList;
		for (int i=0; i<currentScaleCount; ++i)
			scalesList << QString::number(allScales[maxScaleCount - currentScaleCount + i]);

		scalesListLineEdit->setText(scalesList.join(" "));
		scalesCountSpinBox->setEnabled(true);
	}
}

void qCanupo2DViewDialog::reset()
{
	if (m_glWindow)
		m_glWindow->getOwnDB()->removeAllChildren();
	
	if (m_poly)
		delete m_poly;
	m_poly = nullptr;
	m_polyVertices = nullptr; //m_polyVertices is a child of m_poly
	
	if (m_cloud)
		delete m_cloud;
	m_cloud = nullptr;
}

void qCanupo2DViewDialog::getActiveScales(std::vector<float>& scales) const
{
	scales.clear();

	if (!m_descriptors1)
		return;

	//take only the right number (among the smallest)
	const std::vector<float>& allScales = m_descriptors1->scales();
	int maxScaleCount = static_cast<int>(allScales.size());
	int currentScaleCount = scalesCountSpinBox->value();
	assert(currentScaleCount >= 1 && currentScaleCount <= maxScaleCount);
	currentScaleCount = std::min<int>(currentScaleCount, maxScaleCount);
	scales.resize(currentScaleCount);
	for (int i=0; i<currentScaleCount; ++i)
	{
		//take only the smallest values (scales are sorted from the biggest to the smallest)
		scales[i] = allScales[maxScaleCount - currentScaleCount + i];
	}
}


//Training in progress
static bool s_training = false;
static int s_trainingValue = 0;

bool qCanupo2DViewDialog::trainClassifier()
{
	if (!m_descriptors1 || !m_descriptors2)
		return false;

	s_training = true;
	s_trainingValue = scalesCountSpinBox->value();
	statisticsToolButton->setEnabled(false);
	setEnabled(false);
	QApplication::processEvents();

	//take only the right number (among the smallest)
	std::vector<float> scales;
	getActiveScales(scales);

	//reset display
	reset();

	//Reset classifier and re-train it!
	m_classifier = Classifier();
	m_classifier.class1 = m_class1;
	m_classifier.class2 = m_class2;

	m_cloud = new ccPointCloud("CANUPO projections");
	if (!qCanupoTools::TrainClassifier(	m_classifier,
										*m_descriptors1,
										*m_descriptors2,
										scales,
										m_cloud,
										m_evaluationDescriptors,
										m_app))
	{
		delete m_cloud;
		m_cloud = nullptr;
		
		s_training = false;
		setEnabled(true);

		return false;
	}

	addObject(m_cloud);

	//show reference points as spheres
	{
		PointCoordinateType l = m_cloud->getOwnBB().getMaxBoxDim() / 100;
		ccGLMatrix matPos;
		matPos.setTranslation(CCVector3(m_classifier.refPointPos.x,m_classifier.refPointPos.y,0));
		ccSphere* spherePos = new ccSphere(l,&matPos);
		spherePos->setColor(ccColor::red);
		spherePos->showColors(true);
		spherePos->enableStippling(true);
		m_cloud->addChild(spherePos);
		addObject(spherePos);

		ccGLMatrix matNeg;
		matNeg.setTranslation(CCVector3(m_classifier.refPointNeg.x,m_classifier.refPointNeg.y,0));
		ccSphere* sphereNeg = new ccSphere(l,&matNeg);
		sphereNeg->setColor(ccColor::blue);
		sphereNeg->showColors(true);
		sphereNeg->enableStippling(true);
		m_cloud->addChild(sphereNeg);
		addObject(sphereNeg);
	}

	//update/create boundary representation
	resetBoundary();

	if (s_firstDisplay)
	{
		updateZoom();
		s_firstDisplay = false;
	}

	s_training = false;
	setEnabled(true);

	//we need a valid classifier to compute statistics!
	statisticsToolButton->setEnabled(true);

	return true;
}

void qCanupo2DViewDialog::onScalesCountSpinBoxChanged(int value)
{
	if (s_training)
	{
		scalesCountSpinBox->blockSignals(true);
		scalesCountSpinBox->setValue(s_trainingValue);
		scalesCountSpinBox->blockSignals(false);
	}
	else
	{
		//update list of scales
		updateScalesList(false);

		trainClassifier();
	}
}

void qCanupo2DViewDialog::computeStatistics()
{
	qCanupoTools::EvalParameters params;
	std::vector<float> scales;
	getActiveScales(scales);

	Classifier classifier = m_classifier;
	updateClassifierPath(classifier);

	if (qCanupoTools::EvaluateClassifier(	classifier,
											*m_descriptors1,
											*m_descriptors2,
											scales,
											params))
	{
		QStringList info;
		info << QString("Class %1 (%2)").arg(m_class1).arg(m_cloud1Name);
		info << QString("\tTotal: %1").arg(params.true1+params.false1);
		info << QString("\tTruly classified: %1").arg(params.true1);
		info << QString("\tFalsely classified: %1").arg(params.false1);
		info << QString("\tDist. to boundary: %1 +/- %2").arg(params.mu1).arg(sqrt(params.var1));
		info << QString("");
		info << QString("Class %1 (%2)").arg(m_class2).arg(m_cloud2Name);
		info << QString("\tTotal: %1").arg(params.true2+params.false2);
		info << QString("\tTruly classified: %1").arg(params.true2);
		info << QString("\tFalsely classified: %1").arg(params.false2);
		info << QString("\tDist. to boundary: %1 +/- %2").arg(params.mu2).arg(sqrt(params.var2));
		info << QString("");
		info << QString("Balanced accuracy (ba) = %1").arg(params.ba());
		info << QString("Fisher Discriminant Ratio (fdr) = %1").arg(params.fdr());

		QMessageBox::information(this,"Statistics",info.join("\n"),QMessageBox::Ok);
	}
}

void qCanupo2DViewDialog::setPointSize(int value)
{
	if (m_glWindow)
	{
		m_glWindow->setPointSize(static_cast<float>(value));
		m_glWindow->redraw();
	}
}

void qCanupo2DViewDialog::checkBeforeAccept()
{
	if (!m_classifierSaved)
	{
		if (QMessageBox::warning(this, "Classifier has not been saved!", "Do you really want to close the dialog before saving the classifier?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::No)
			return;
	}

	accept();
}

void qCanupo2DViewDialog::resetBoundary()
{
	assert(m_glWindow);

	if (!m_poly)
	{
		assert(!m_polyVertices);
		m_polyVertices = new ccPointCloud("vertices");
		m_poly = new ccPolyline(m_polyVertices);
		m_poly->addChild(m_polyVertices);
		m_poly->setColor(ccColor::magenta);
		m_poly->showColors(true);
		m_poly->setWidth(2);
		m_poly->showVertices(true);
		m_poly->setVertexMarkerWidth(4);
		addObject(m_poly);
	}

	m_poly->clear();
	m_polyVertices->clear();

	unsigned pathLength = static_cast<unsigned>(m_classifier.path.size());
	if (pathLength > 1)
	{
		if (	!m_poly->reserve(pathLength)
			||	!m_polyVertices->reserve(pathLength))
			return;

		for (unsigned i=0; i<pathLength; ++i)
			m_polyVertices->addPoint(CCVector3(m_classifier.path[i].x, m_classifier.path[i].y, -std::numeric_limits<PointCoordinateType>::epsilon()));
		
		m_poly->addPointIndex(0,pathLength);
	}

	m_glWindow->redraw();
}

void qCanupo2DViewDialog::saveClassifier()
{
	//select file to save
	QSettings settings("qCanupo");
	settings.beginGroup("Classif");
	QString currentPath = settings.value("MscCurrentPath",QApplication::applicationDirPath()).toString();

	QString filename = QFileDialog::getSaveFileName(this,"Save Classifier",currentPath,"*.prm");
	if (filename.isEmpty())
		return;

	//update the classifier's path
	Classifier classifier = m_classifier;
	updateClassifierPath(classifier);

	QString error;
	if (classifier.save(filename, error))
	{
		m_classifierSaved = true;
		if (m_app)
			m_app->dispToConsole(QString("Classifier file saved: '%1'").arg(filename));
	}
	else
	{
		if (m_app)
			m_app->dispToConsole(error,ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	//we update current file path
	currentPath = QFileInfo(filename).absolutePath();
	settings.setValue("MscCurrentPath",currentPath);
}

void qCanupo2DViewDialog::updateClassifierPath(Classifier& classifier) const
{
	//update the classifier's path
	if (m_poly)
	{
		classifier.path.resize(m_poly->size());
		for (unsigned i=0; i<m_poly->size(); ++i)
		{
			const CCVector3* P = m_poly->getPoint(i);
			classifier.path[i] = Classifier::Point2D(P->x,P->y);
		}
	}
}

void qCanupo2DViewDialog::addObject(ccHObject* obj)
{
	if (!obj)
		return;
	obj->setVisible(true);
	m_glWindow->addToOwnDB(obj);
}

void qCanupo2DViewDialog::updateZoom()
{
	ccBBox box = m_glWindow->getOwnDB()->getDisplayBB_recursive(false,m_glWindow);
	m_glWindow->updateConstellationCenterAndZoom(&box);
	m_glWindow->redraw();
}

void qCanupo2DViewDialog::setPickingRadius(int radius)
{
	m_pickingRadius = std::max(radius,1);
}

CCVector3 qCanupo2DViewDialog::getClickPos(int x, int y) const
{
	if (!m_glWindow)
	{
		assert(false);
		return CCVector3(0, 0, 0);
	}

	//we must convert the mouse click to the polyline coordinate system
	ccGLCameraParameters camera;
	m_glWindow->getGLCameraParameters(camera);

	QPointF pos2D = m_glWindow->toCornerGLCoordinates(x, y);
	CCVector3 P2D(pos2D.x(), pos2D.y(), 0);

	CCVector3d P3D;
	camera.unproject(P2D, P3D);

	//DGM: the Z value is meaningless!
	P3D.z = 0;

	return CCVector3::fromArray(P3D.u);
}

int qCanupo2DViewDialog::getClosestVertex(int x, int y, CCVector3& P) const
{
	if (!m_poly || !m_glWindow)
		return -1;

	P = getClickPos(x, y);

	//look for the closest vertex
	int closeIndex = -1;
	float closestSquareDist = 0;
	for (unsigned i=0; i<m_poly->size(); ++i)
	{
		float squareDist = (*m_poly->getPoint(i) - P).norm2();
		if (closeIndex < 0 || squareDist < closestSquareDist)
		{
			closestSquareDist = squareDist;
			closeIndex = static_cast<int>(i);
		}
	}

	return closeIndex;
}

void qCanupo2DViewDialog::addOrSelectPoint(int x, int y)
{
	if (!m_poly || !m_glWindow)
		return;

	CCVector3 P;
	int closeIndex = getClosestVertex(x, y, P);

	//B = closest vertex
	const CCVector3* B = (closeIndex >= 0 ? m_poly->getPoint(closeIndex) : nullptr);

	//picking radius
	double maxPickingDist = static_cast<double>(m_pickingRadius) * m_glWindow->computeActualPixelSize();

	//to allow 'mouse move" tracking event
	m_glWindow->setInteractionMode(ccGLWindow::INTERACT_SEND_ALL_SIGNALS);

	//is the closest point close enough?
	if (closeIndex >= 0)
	{
		assert(B);
		if ((P - *B).norm() <= maxPickingDist)
		{
			//we select the closest vertex
			m_selectedPointIndex = closeIndex;
			return;
		}
	}

	//let's look if the click fall 'inside' a segment
	double nearestProjectionDist = maxPickingDist;
	CCVector3 nearestProj = P;
	int nearestSegIndex = -1;
	for (unsigned i=0; i+1<m_poly->size(); ++i)
	{
		const CCVector3* A = m_poly->getPoint(i);
		const CCVector3* B = m_poly->getPoint(i+1);

		CCVector3 AB = (*B-*A);
		CCVector3 AP = ( P-*A);

		PointCoordinateType dot = AB.dot(AP);

		//projection falls inside the segment?
		dot /= AB.norm2();
		if (dot > 0 && dot < PC_ONE)
		{
			CCVector3 AH = AB*dot;
			CCVector3 HP = AP - AH;
			double dist = HP.norm();

			if (dist < nearestProjectionDist)
			{
				nearestProjectionDist = dist;
				nearestProj = AH + *A;
				nearestSegIndex = static_cast<int>(i);
			}
		}
	}

	if (nearestSegIndex >= 0)
	{
		//if we have a candidate
		closeIndex = nearestSegIndex;
		P = nearestProj; //shall we really replace P by its projection on the 
	}
	else
	{
		//we'll add the point to the nearest end (i.e. front or back)
		const CCVector3* A = m_poly->getPoint(0);
		const CCVector3* B = m_poly->getPoint(m_poly->size()-1);
		if ((P-*A).norm2() < (P-*B).norm2())
		{
			closeIndex = -1;
		}
		else
		{
			closeIndex = static_cast<int>(m_poly->size()-1);
		}
	}

	//add the point to the vertices cloud
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(m_poly->getAssociatedCloud());
	if (!vertices)
	{
		assert(false);
		return;
	}
	unsigned newIndexInCloud = vertices->size();
	vertices->reserve(newIndexInCloud+1);
	vertices->addPoint(P);

	//add the corresponding point index at the end of the polyline
	//(whatever the case to avoid complicated tests later)
	m_poly->reserve(m_poly->size()+1);
	m_poly->addPointIndex(newIndexInCloud);

	m_selectedPointIndex = closeIndex+1;

	//eventually we must shift the indexes starting from newIndexInPoly
	unsigned newIndexInPoly = static_cast<unsigned>(m_selectedPointIndex);
	while (newIndexInPoly < m_poly->size())
	{
		unsigned previousIndexInCloud = m_poly->getPointGlobalIndex(newIndexInPoly);
		m_poly->setPointIndex(newIndexInPoly, newIndexInCloud);
		newIndexInCloud = previousIndexInCloud;
		++newIndexInPoly;
	}

	m_glWindow->redraw();
}

void qCanupo2DViewDialog::removePoint(int x, int y)
{
	if (!m_poly || !m_glWindow)
		return;

	//we can't accept less than 2 vertices!
	unsigned polySize = m_poly->size();
	if (polySize < 3)
		return;

	CCVector3 P;
	int closeIndex = getClosestVertex(x, y, P);

	//nothing to do
	if (closeIndex < 0)
		return;

	//picking radius
	double maxPickingDist = static_cast<double>(m_pickingRadius) * m_glWindow->computeActualPixelSize();

	//B = closest vertex
	const CCVector3* B = (closeIndex >=0 ? m_poly->getPoint(closeIndex) : nullptr);
	if ((P-*B).norm() > maxPickingDist)
	{
		//too far
		return;
	}

	//shift all indexes on the polyline only (we don't bother with the vertices cloud!)
	for (unsigned i=static_cast<unsigned>(closeIndex); i<polySize-1; ++i)
		m_poly->setPointIndex(i,m_poly->getPointGlobalIndex(i+1));
	m_poly->resize(polySize-1);

	m_glWindow->redraw();
}

void qCanupo2DViewDialog::moveSelectedPoint(int x, int y, Qt::MouseButtons buttons)
{
	if (buttons != Qt::LeftButton)
		return;
	if (m_selectedPointIndex < 0)
		return;
	if (!m_poly || !m_glWindow)
		return;

	CCVector3 newP = getClickPos(x, y);

	assert(static_cast<int>(m_poly->size()) > m_selectedPointIndex);
	CCVector3* P = const_cast<CCVector3*>(m_poly->getPoint(m_selectedPointIndex));
	*P = newP;

	//TODO: in theory we should update the vertices cloud bounding-box
	//(but we as never use it...)

	m_glWindow->redraw();
}

void qCanupo2DViewDialog::deselectPoint()
{
	//to disable 'mouse move' event tracking
	m_glWindow->setInteractionMode(ccGLWindow::PAN_ONLY() | ccGLWindow::INTERACT_SEND_ALL_SIGNALS);

	m_selectedPointIndex = -1;
}
