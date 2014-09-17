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

#include "ccClippingBoxTool.h"

//Local
#include "ccGLWindow.h"
#include "mainwindow.h"
#include "ccClippingBoxRepeatDlg.h"
#include "ccBoundingBoxEditorDlg.h"

//qCC_db
#include <ccLog.h>
#include <ccHObject.h>
#include <ccClipBox.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccPolyline.h>
#include <ccProgressDialog.h>

//CCLib
#include <ReferenceCloud.h>
#include <Neighbourhood.h>

//Qt
#include <QProgressDialog>
#include <QMessageBox>
#include <QInputDialog>

//! Last contour unique ID
static std::vector<unsigned> s_lastContourUniqueIDs;
//! Max edge length parameter (contour extraction)
static double s_maxEdgeLength = -1.0;

ccClippingBoxTool::ccClippingBoxTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::ClippingBoxDlg()
	, m_clipBox(0)
{
	setupUi(this);

	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(editBoxToolButton,				SIGNAL(clicked()), this, SLOT(editBox()));
	connect(extractContourToolButton,		SIGNAL(clicked()), this, SLOT(extractContour()));
	connect(removeLastContourToolButton,	SIGNAL(clicked()), this, SLOT(removeLastContour()));
	connect(exportButton,					SIGNAL(clicked()), this, SLOT(exportCloud()));
	connect(exportMultButton,				SIGNAL(clicked()), this, SLOT(exportMultCloud()));
	connect(resetButton,					SIGNAL(clicked()), this, SLOT(reset()));
	connect(closeButton,					SIGNAL(clicked()), this, SLOT(closeDialog()));

	connect(showInteractorsCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleInteractors(bool)));

	connect(thickXDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(thicknessChanged(double)));
	connect(thickYDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(thicknessChanged(double)));
	connect(thickZDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(thicknessChanged(double)));

	connect(minusXShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftXMinus()));
	connect(plusXShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftXPlus()));
	connect(minusYShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftYMinus()));
	connect(plusYShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftYPlus()));
	connect(minusZShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftZMinus()));
	connect(plusZShiftToolButton,	SIGNAL(clicked()), this, SLOT(shiftZPlus()));

	viewButtonsFrame->setEnabled(true);
	connect(viewUpToolButton,		SIGNAL(clicked()),	this,	SLOT(setTopView()));
	connect(viewDownToolButton,		SIGNAL(clicked()),	this,	SLOT(setBottomView()));
	connect(viewFrontToolButton,	SIGNAL(clicked()),	this,	SLOT(setFrontView()));
	connect(viewBackToolButton,		SIGNAL(clicked()),	this,	SLOT(setBackView()));
	connect(viewLeftToolButton,		SIGNAL(clicked()),	this,	SLOT(setLeftView()));
	connect(viewRightToolButton,	SIGNAL(clicked()),	this,	SLOT(setRightView()));

	s_maxEdgeLength = -1.0;
	//s_lastContourUniqueIDs.clear();
	removeLastContourToolButton->setEnabled(false);
}

ccClippingBoxTool::~ccClippingBoxTool()
{
	if (m_clipBox)
		delete m_clipBox;
	m_clipBox = 0;
}

void ccClippingBoxTool::editBox()
{
	if (!m_clipBox)
		return;

	ccBBox box = m_clipBox->getBox();

	ccBoundingBoxEditorDlg bbeDlg(this);
	bbeDlg.setBaseBBox(box,false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Edit clipping box");

	if (!bbeDlg.exec())
		return;

	box = bbeDlg.getBox();
	m_clipBox->setBox(box);

	//onBoxModified(&box); //DGM: automatically called by 'm_clipBox'

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::toggleInteractors(bool state)
{
	if (m_clipBox)
		m_clipBox->setSelected(state);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

bool ccClippingBoxTool::setAssociatedEntity(ccHObject* entity)
{
	if (!m_associatedWin || !m_clipBox)
	{
		ccLog::Error(QString("[Clipping box] No associated 3D view or no valid clipping box!"));
		return false;
	}

	//we don't handle entities associated to another context
	if (entity->getDisplay() != m_associatedWin)
	{
		ccLog::Warning(QString("[Clipping box] Can't use entity '%1' cause it's not displayed in the active 3D view!").arg(entity->getName()));
		return false;
	}

	//we can't handle other entities than clouds for the moment
	if (!entity->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Warning(QString("[Clipping box] Only points clouds are handled! Entity '%1' will be ignored.").arg(entity->getName()));
		return false;
	}

	//force visibility
	entity->setVisible(true);
	entity->setEnabled(true);

	m_clipBox->setAssociatedEntity(entity);
	if (m_associatedWin)
		m_associatedWin->redraw();

	//set proper "steps" value for slice thickness editors
	{
		CCVector3 diag = entity->getBB().getDiagVec();
		thickXDoubleSpinBox->setSingleStep(static_cast<double>(diag.x) / 100.0);
		thickYDoubleSpinBox->setSingleStep(static_cast<double>(diag.y) / 100.0);
		thickZDoubleSpinBox->setSingleStep(static_cast<double>(diag.z) / 100.0);
	}

	s_maxEdgeLength = -1.0;
	s_lastContourUniqueIDs.clear();
	removeLastContourToolButton->setEnabled(false);

	return true;
}

bool ccClippingBoxTool::linkWith(ccGLWindow* win)
{
	if (m_associatedWin && m_clipBox)
	{
		//remove clipping box from previous window
		m_associatedWin->removeFromOwnDB(m_clipBox);
		m_clipBox->disconnect(this);
		delete m_clipBox;
		m_clipBox = 0;
		m_associatedWin->redraw();
	}

	if (!ccOverlayDialog::linkWith(win))
		return false;

	if (win)
	{
		if (!m_clipBox)
		{
			m_clipBox = new ccClipBox();
			m_clipBox->setVisible(true);
			m_clipBox->setEnabled(true);
			m_clipBox->setSelected(showInteractorsCheckBox->isChecked());
			connect(m_clipBox, SIGNAL(boxModified(const ccBBox*)), this, SLOT(onBoxModified(const ccBBox*)));
		}
		m_associatedWin->addToOwnDB(m_clipBox);
	}
	
	return true;
}

bool ccClippingBoxTool::start()
{
	assert(!m_processing);
	assert(m_associatedWin);
	if (!m_associatedWin || !m_clipBox)
		return false;

	m_clipBox->reset();

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	//m_associatedWin->displayNewMessage(QString(),ccGLWindow::UPPER_CENTER_MESSAGE); //clear the area
	//m_associatedWin->displayNewMessage("[Rotation/Translation mode]",ccGLWindow::UPPER_CENTER_MESSAGE,false,3600,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
	m_associatedWin->redraw();

	return ccOverlayDialog::start();
}

void ccClippingBoxTool::stop(bool state)
{
	if (m_associatedWin)
	{
		m_associatedWin->setUnclosable(false);
		//m_associatedWin->displayNewMessage("[Rotation/Translation mode OFF]",ccGLWindow::UPPER_CENTER_MESSAGE,false,2,ccGLWindow::MANUAL_TRANSFORMATION_MESSAGE);
		m_associatedWin->redraw();
	}

	ccOverlayDialog::stop(state);
}

void ccClippingBoxTool::removeLastContour()
{
	if (s_lastContourUniqueIDs.empty())
		return;

	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow)
	{
		for (size_t i=0; i<s_lastContourUniqueIDs.size(); ++i)
		{
			ccHObject* obj = mainWindow->db()->find(s_lastContourUniqueIDs[i]);
			if (obj)
			{
				mainWindow->removeFromDB(obj);
				ccGLWindow* win = mainWindow->getActiveGLWindow();
				if (win)
					win->redraw();
			}
		}
	}

	s_lastContourUniqueIDs.clear();
	removeLastContourToolButton->setEnabled(false);
}

void ccClippingBoxTool::exportCloud()
{
	if (!m_clipBox || !MainWindow::TheInstance())
		return;

	ccHObject* obj = m_clipBox->getAssociatedEntity();

	if (obj && obj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj)->createNewCloudFromVisibilitySelection(false);
		if (!cloud)
		{
			ccLog::Error("Not enough memory!");
			return;
		}
		
		cloud->setDisplay(obj->getDisplay());
		MainWindow::TheInstance()->addToDB(cloud);
	}
}

void ccClippingBoxTool::extractContour()
{
	extractSlicesAndContours(false, true, /*singleContourMode=*/true);
}

void ccClippingBoxTool::exportMultCloud()
{
	extractSlicesAndContours(true, true, /*singleContourMode=*/false);
}

void ccClippingBoxTool::extractSlicesAndContours(bool extractSlices, bool extractContours, bool singleContourMode)
{
	if (!m_clipBox)
		return;

	ccHObject* obj = m_clipBox->getAssociatedEntity();
	if (!obj || !obj->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Warning("Only works with point clouds!");
		return;
	}
	ccPointCloud* cloud = static_cast<ccPointCloud*>(obj);

	ccClippingBoxRepeatDlg repeatDlg(singleContourMode, MainWindow::TheInstance());
	
	//by default we set the 'flat/repeat' dimension to the smallest box dimension
	{
		CCVector3 diagVec = m_clipBox->getBB().getDiagVec();
		unsigned char flatDim = 0;
		if (diagVec.y < diagVec.x)
			flatDim = 1;
		if (diagVec.z < diagVec.u[flatDim])
			flatDim = 2;
		if (singleContourMode)
			repeatDlg.setFlatDim(flatDim);
		else
			repeatDlg.setRepeatDim(flatDim);
	}
	
	//random colors is only available for real point clouds!
	//(and only useful for mutliple slice/contour mode in fact...)
	repeatDlg.randomColorCheckBox->setEnabled(cloud->isA(CC_TYPES::POINT_CLOUD));
	
	//set default max edge length
	if (s_maxEdgeLength < 0)
		s_maxEdgeLength = static_cast<double>(cloud->getBB().getDiagNorm())/100.0;
	repeatDlg.maxEdgeLengthDoubleSpinBox->setValue(s_maxEdgeLength);
	
	if (!repeatDlg.exec())
		return;

	//repeat dimensions 
	bool processDim[3] = {	repeatDlg.xRepeatCheckBox->isChecked(),
							repeatDlg.yRepeatCheckBox->isChecked(),
							repeatDlg.zRepeatCheckBox->isChecked() };
	int processDimSum =	static_cast<int>(processDim[0])
					+	static_cast<int>(processDim[1])
					+	static_cast<int>(processDim[2]);

	if (processDimSum == 0)
	{
		ccLog::Error("No dimension selected to repeat the segmentation process?!");
		return;
	}

	//whether to use random colors for (multiple) generated slices
	bool generateRandomColors = repeatDlg.randomColorCheckBox->isChecked();

	//whether to extract contours or not
	if (!singleContourMode)
	{
		extractContours = repeatDlg.extractContoursGroupBox->isChecked();
	}
			
	//compute the cloud bounding box in the local clipping box ref.
	ccGLMatrix localTrans;
	{
		if (m_clipBox->isGLTransEnabled())
			localTrans = m_clipBox->getGLTransformation().inverse();
		else
			localTrans.toIdentity();
	}

	//compute 'grid' extents in the local clipping box ref.
	int indexMins[3] = { 0, 0, 0 };
	int indexMaxs[3] = { 0, 0, 0 };
	int gridDim[3] = { 1, 1, 1 };
	unsigned cellCount = 1;
	CCVector3 gridOrigin = m_clipBox->getBB().minCorner();
	CCVector3 cellSize = m_clipBox->getBB().getDiagVec();
	PointCoordinateType gap = (PointCoordinateType)repeatDlg.gapDoubleSpinBox->value();

	//for mutli-dimensional mode only!
	if (!singleContourMode)
	{
		ccBBox localBox;
		for (unsigned i=0; i<cloud->size(); ++i)
		{
			CCVector3 P = *cloud->getPoint(i);
			localTrans.apply(P);
			localBox.add(P);
		}

		for (unsigned char d=0; d<3; ++d)
		{
			if (processDim[d])
			{
				if (cellSize.u[d] + gap < ZERO_TOLERANCE)
				{
					ccLog::Error("Box size (plus gap) is null! Can't apply repetitive process!");
					return;
				}

				PointCoordinateType a = (localBox.minCorner().u[d] - gridOrigin.u[d])/(cellSize.u[d]+gap); //don't forget the user defined gap between 'cells'
				PointCoordinateType b = (localBox.maxCorner().u[d] - gridOrigin.u[d])/(cellSize.u[d]+gap);

				indexMins[d] = static_cast<int>(floor(a+static_cast<PointCoordinateType>(1.0e-6)));
				indexMaxs[d] = static_cast<int>(ceil(b-static_cast<PointCoordinateType>(1.0e-6)))-1;
				
				assert(indexMaxs[d] >= indexMins[d]);
				gridDim[d] = std::max(indexMaxs[d] - indexMins[d] + 1, 1);
				cellCount *= static_cast<unsigned>(gridDim[d]);
			}
		}
	}

	//apply process
	{
		//backup original clipping box
		//ccBBox originalBox = m_clipBox->getBox();

		//group to store all the resulting slices
		ccHObject* sliceGroup = 0;

		//slice(s) cloud(s)
		std::vector<ccGenericPointCloud*> clouds;
		try
		{
			clouds.resize(cellCount,0);
		}
		catch(std::bad_alloc)
		{
			ccLog::Error("Not enough memory!");
			return;
		}

		bool error = false;
		bool warningsIssued = false;
		unsigned subCloudsCount = 0;

		if (singleContourMode)
		{
			//single cloud: easy
			assert(clouds.size() == 1);
			clouds[0] = cloud->createNewCloudFromVisibilitySelection(false);
			if (!clouds[0])
			{
				ccLog::Error("Not enough memory!");
				return;
			}
			clouds[0]->setName(cloud->getName() + QString(".slice"));
			subCloudsCount = 1;
		}
		else
		{
			//we'll potentially create up to one (ref.) cloud per cell
			std::vector<CCLib::ReferenceCloud*> refClouds;
			try
			{
				refClouds.resize(cellCount,0);
			}
			catch(std::bad_alloc)
			{
				ccLog::Error("Not enough memory!");
				return;
			}
			CCVector3 cellSizePlusGap = cellSize + CCVector3(gap,gap,gap);

			//project points into grid
			{
				unsigned pointCount = cloud->size(); 

				ccProgressDialog pDlg(false,this);
				pDlg.setInfo(qPrintable(QString("Points: %1").arg(pointCount)));
				pDlg.start();
				pDlg.show();
				QApplication::processEvents();

				CCLib::NormalizedProgress nProgress(&pDlg,pointCount);
				for (unsigned i=0; i<pointCount; ++i)
				{
					CCVector3 P = *cloud->getPoint(i);
					localTrans.apply(P);

					//relative coordinates (between 0 and 1)
					P -= gridOrigin;
					P.x /= cellSizePlusGap.x;
					P.y /= cellSizePlusGap.y;
					P.z /= cellSizePlusGap.z;

					int xi = static_cast<int>(floor(P.x));
					xi = std::min( std::max(xi, indexMins[0]), indexMaxs[0]);
					int yi = static_cast<int>(floor(P.y));
					yi = std::min( std::max(yi, indexMins[1]), indexMaxs[1]);
					int zi = static_cast<int>(floor(P.z));
					zi = std::min( std::max(zi, indexMins[2]), indexMaxs[2]);

					if ( gap == 0 ||
						(	(P.x-static_cast<PointCoordinateType>(xi))*cellSizePlusGap.x <= cellSize.x
						&&	(P.y-static_cast<PointCoordinateType>(yi))*cellSizePlusGap.y <= cellSize.y
						&&	(P.z-static_cast<PointCoordinateType>(zi))*cellSizePlusGap.z <= cellSize.z) )
					{
						int cloudIndex = ((zi-indexMins[2]) * static_cast<int>(gridDim[1]) + (yi-indexMins[1])) * static_cast<int>(gridDim[0]) + (xi-indexMins[0]);
						assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex) < refClouds.size());

						if (!refClouds[cloudIndex])
						{
							refClouds[cloudIndex] = new CCLib::ReferenceCloud(cloud);
							++subCloudsCount;
						}

						if (!refClouds[cloudIndex]->addPointIndex(i))
						{
							ccLog::Error("Not enough memory!");
							error = true;
							break;
						}
					}
				}

				nProgress.oneStep();
			} //project points into grid

			//now create the real clouds
			{
				sliceGroup = new ccHObject(QString("%1.slices").arg(cloud->getName()));

				QProgressDialog pDlg(QString("Extract section(s): %1").arg(subCloudsCount),"Cancel",0,static_cast<int>(subCloudsCount),this);
				pDlg.show();
				QApplication::processEvents();

				for (int i=indexMins[0]; i<=indexMaxs[0]; ++i)
				{
					for (int j=indexMins[1]; j<=indexMaxs[1]; ++j)
					{
						for (int k=indexMins[2]; k<=indexMaxs[2]; ++k)
						{
							int cloudIndex = ((k-indexMins[2]) * static_cast<int>(gridDim[1]) + (j-indexMins[1])) * static_cast<int>(gridDim[0]) + (i-indexMins[0]);
							assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex) < refClouds.size());

							if (refClouds[cloudIndex]) //some slices can be empty due to rounding issues!
							{
								//generate slice from previous selection
								int warnings = 0;
								ccPointCloud* sliceCloud = cloud->partialClone(refClouds[cloudIndex],&warnings);
								warningsIssued |= (warnings != 0);
							
								if (sliceCloud)
								{
									if (generateRandomColors && cloud->isA(CC_TYPES::POINT_CLOUD))
									{
										colorType col[3];
										ccColor::Generator::Random(col);
										if (!sliceCloud->setRGBColor(col))
										{
											ccLog::Error("Not enough memory!");
											error = true;
											i = indexMaxs[0];
											j = indexMaxs[1];
											k = indexMaxs[2];
										}
										sliceCloud->showColors(true);
									}

									sliceCloud->setEnabled(true);
									sliceCloud->setVisible(true);
									sliceCloud->setDisplay(cloud->getDisplay());

									CCVector3 cellOrigin(	gridOrigin.x + static_cast<PointCoordinateType>(i) * cellSizePlusGap.x,
															gridOrigin.y + static_cast<PointCoordinateType>(j) * cellSizePlusGap.y,
															gridOrigin.z + static_cast<PointCoordinateType>(k) * cellSizePlusGap.z);
									QString slicePosStr = QString("(%1 ; %2 ; %3)").arg(cellOrigin.x).arg(cellOrigin.y).arg(cellOrigin.z);
									sliceCloud->setName(QString("slice @ ")+slicePosStr);

									//add slice to group
									sliceGroup->addChild(sliceCloud);
									//update 'real clouds' grid
									clouds[cloudIndex] = sliceCloud;
									++subCloudsCount;
								}

								if (pDlg.wasCanceled())
								{
									error = true;
									ccLog::Warning(QString("[ccClippingBoxTool::exportMultCloud] Process canceled by user"));
									//early stop
									i = indexMaxs[0];
									j = indexMaxs[1];
									k = indexMaxs[2];
									break;
								}
								pDlg.setValue(static_cast<int>(subCloudsCount));

							}
						}
					}
				}
			} //now create the real clouds

			//release memory
			{
				for (size_t i=0; i<refClouds.size(); ++i)
					if (refClouds[i])
						delete refClouds[i];
				refClouds.clear();
			}
		}

		//extract contour polylines (optionaly)
		if (!error && extractContours && subCloudsCount != 0)
		{
			//contour extraction parameter (max edge length)
			s_maxEdgeLength = repeatDlg.maxEdgeLengthDoubleSpinBox->value();
			bool splitContour = false;
			if (s_maxEdgeLength > 0)
			{
				splitContour = (QMessageBox::question(0,"Split contour","Do you want to split the contour(s) in multiple parts if necessary?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes);
			}
			ccHObject* contourGroup = new ccHObject(obj->getName() + QString(".contours"));

			QProgressDialog pDlg(QString("Extract contour(s): %1").arg(subCloudsCount),"Cancel",0,static_cast<int>(subCloudsCount),this);
			pDlg.show();
			QApplication::processEvents();

			//preferred dimension?
			int preferredDim = -1;
			if (processDimSum == 1 && !repeatDlg.projectOnBestFitCheckBox->isChecked())
			{
				for (int i=0; i<3; ++i)
					if (processDim[i])
						preferredDim = i;
			}
			ccGLMatrix invLocalTrans = localTrans.inverse();
			PointCoordinateType* preferredOrientation = (preferredDim != -1 ? invLocalTrans.getColumn(preferredDim) : 0);

			unsigned currentCloudCount = 0;
			for (int i=indexMins[0]; i<=indexMaxs[0]; ++i)
			{
				for (int j=indexMins[1]; j<=indexMaxs[1]; ++j)
				{
					for (int k=indexMins[2]; k<=indexMaxs[2]; ++k)
					{
						int cloudIndex = ((k-indexMins[2]) * static_cast<int>(gridDim[1]) + (j-indexMins[1])) * static_cast<int>(gridDim[0]) + (i-indexMins[0]);
						assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex) < clouds.size());

						if (clouds[cloudIndex]) //some slices can be empty due to rounding issues!
						{
							std::vector<ccPolyline*> polys;
							if (ccPolyline::ExtractFlatContour(	clouds[cloudIndex],
																static_cast<PointCoordinateType>(s_maxEdgeLength),
																polys,
																splitContour,
																preferredOrientation))
							{
								if (!polys.empty())
								{
									for (size_t p=0; p<polys.size(); ++p)
									{
										ccPolyline* poly = polys[p];
										poly->setColor(ccColor::green);
										poly->showColors(true);
										QString contourName = clouds[cloudIndex]->getName();
										contourName.replace("slice","contour");
										if (polys.size() != 1)
											contourName += QString(" (part %1)").arg(p+1);
										poly->setName(contourName);
										contourGroup->addChild(poly);
									}
								}
								else
								{
									ccLog::Warning(QString("%1: points are too far from each other! Increase the max edge length").arg(clouds[cloudIndex]->getName()));
									warningsIssued = true;
								}
							}
							else
							{
								ccLog::Warning(QString("%1: contour extraction failed!").arg(clouds[cloudIndex]->getName()));
								warningsIssued = true;
							}
						}
						
						++currentCloudCount;
						if (pDlg.wasCanceled())
						{
							error = true;
							ccLog::Warning(QString("[ccClippingBoxTool::exportMultCloud] Process canceled by user"));
							//early stop
							i = indexMaxs[0];
							j = indexMaxs[1];
							k = indexMaxs[2];
							break;
						}
						pDlg.setValue(static_cast<int>(currentCloudCount));
					}
				}
			}

			if (error || contourGroup->getChildrenNumber() == 0)
			{
				delete contourGroup;
				contourGroup = 0;
			}
			else
			{
				contourGroup->setDisplay_recursive(cloud->getDisplay());
				MainWindow::TheInstance()->addToDB(contourGroup);
			
				s_lastContourUniqueIDs.clear();
				s_lastContourUniqueIDs.push_back(contourGroup->getUniqueID());
				removeLastContourToolButton->setEnabled(true);
			}
		}

		//release memory
		if (error)
		{
			clouds.clear();
			delete sliceGroup;
			sliceGroup = 0;
		}
		else if (singleContourMode)
		{
			assert(!sliceGroup);
			assert(clouds.size() == 1);
			delete clouds[0]; //temporary cloud!
			clouds.clear();
		}

		if (!error)
		{
			if (warningsIssued)
			{
				ccLog::Warning("[ccClippingBoxTool::exportMultCloud] Warnings were issued during the process! (result may be incomplete)");
			}
			if (sliceGroup)
			{
				if (sliceGroup->getChildrenNumber() == 0)
				{
					ccLog::Warning("[ccClippingBoxTool] Repeat process generated no output!");
					delete sliceGroup;
					sliceGroup = 0;
				}
				else
				{
					QMessageBox::warning(0, "Process finished", QString("%1 clouds have been generated.\n(you may have to close the tool and hide the initial cloud to see them...)").arg(sliceGroup->getChildrenNumber()));
					sliceGroup->setDisplay_recursive(cloud->getDisplay());
					MainWindow::TheInstance()->addToDB(sliceGroup);
				}
			}
		}

		//m_clipBox->setBox(originalBox);
	}

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::onBoxModified(const ccBBox* box)
{
	if (box && box->isValid())
	{
		CCVector3 dd = box->maxCorner() - box->minCorner();
		thickXDoubleSpinBox->setValue(dd.x);
		thickYDoubleSpinBox->setValue(dd.y);
		thickZDoubleSpinBox->setValue(dd.z);
		thicknessGroupBox->setEnabled(true);
	}
	else
	{
		thicknessGroupBox->setEnabled(false);
	}
}

void ccClippingBoxTool::thicknessChanged(double)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	CCVector3 th(static_cast<PointCoordinateType>(thickXDoubleSpinBox->value()),
				 static_cast<PointCoordinateType>(thickYDoubleSpinBox->value()),
				 static_cast<PointCoordinateType>(thickZDoubleSpinBox->value()));

	ccBBox box = m_clipBox->getBox();
	CCVector3 boxCenter = (box.maxCorner() + box.minCorner()) / 2;

	box.minCorner() = boxCenter - th/2;
	box.maxCorner() = boxCenter + th/2;

	m_clipBox->setBox(box);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::shiftXMinus()
{
	shiftBox(0,true);
}

void ccClippingBoxTool::shiftXPlus()
{
	shiftBox(0,false);
}

void ccClippingBoxTool::shiftYMinus()
{
	shiftBox(1,true);
}

void ccClippingBoxTool::shiftYPlus()
{
	shiftBox(1,false);
}

void ccClippingBoxTool::shiftZMinus()
{
	shiftBox(2,true);
}

void ccClippingBoxTool::shiftZPlus()
{
	shiftBox(2,false);
}

void ccClippingBoxTool::shiftBox(unsigned char dim, bool minus)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	assert(dim<3);

	PointCoordinateType width = (m_clipBox->getBox().maxCorner() - m_clipBox->getBox().minCorner()).u[dim];
	CCVector3 shiftVec(0.0,0.0,0.0);
	shiftVec.u[dim] = (minus ? -width : width);
	m_clipBox->shift(shiftVec);

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::reset()
{
	if (m_clipBox)
		m_clipBox->reset();

	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::closeDialog()
{
	stop(true);
}

void ccClippingBoxTool::setTopView()
{
	setView(CC_TOP_VIEW);
}

void ccClippingBoxTool::setBottomView()
{
	setView(CC_BOTTOM_VIEW);
}

void ccClippingBoxTool::setFrontView()
{
	setView(CC_FRONT_VIEW);
}

void ccClippingBoxTool::setBackView()
{
	setView(CC_BACK_VIEW);
}

void ccClippingBoxTool::setLeftView()
{
	setView(CC_LEFT_VIEW);
}

void ccClippingBoxTool::setRightView()
{
	setView(CC_RIGHT_VIEW);
}

void ccClippingBoxTool::setView(CC_VIEW_ORIENTATION orientation)
{
	if (!m_associatedWin)
		return;

	//m_associatedWin->blockSignals(true);
	m_associatedWin->setView(orientation,false);
	if (m_clipBox && m_clipBox->isGLTransEnabled())
	{
		ccViewportParameters params = m_associatedWin->getViewportParameters();
		const ccGLMatrix& glMat = m_clipBox->getGLTransformation();

		ccGLMatrixd rotMat(glMat.data()); rotMat.clearTranslation();

		params.viewMat = params.viewMat * rotMat.inverse();
		m_associatedWin->setViewportParameters(params);
	}
	//m_associatedWin->blockSignals(false);
	m_associatedWin->redraw();
}
