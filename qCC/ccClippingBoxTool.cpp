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
#include "ccContourExtractor.h"
#include "ccCropTool.h"

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
#include <QMessageBox>
#include <QInputDialog>
#include <QElapsedTimer>

//Last contour unique ID
static std::vector<unsigned> s_lastContourUniqueIDs;

//Contour extraction parameters
static double s_maxEdgeLength = -1.0;
static bool s_splitContours = false;
static bool s_multiPass = false;
static double s_defaultGap = 0.0;

// persistent map of the previous box used for each entity
struct ccClipBoxParams
{
	ccBBox box;
	ccGLMatrix trans;
};
static QMap< unsigned, ccClipBoxParams > s_lastBoxParams;

ccClippingBoxTool::ccClippingBoxTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::ClippingBoxDlg()
	, m_clipBox(0)
{
	setupUi(this);

	connect(editBoxToolButton,				SIGNAL(clicked()),				this, SLOT(editBox()));
	connect(extractContourToolButton,		SIGNAL(clicked()),				this, SLOT(extractContour()));
	connect(removeLastContourToolButton,	SIGNAL(clicked()),				this, SLOT(removeLastContour()));
	connect(exportButton,					SIGNAL(clicked()),				this, SLOT(exportSlice()));
	connect(exportMultButton,				SIGNAL(clicked()),				this, SLOT(exportMultSlices()));
	connect(resetButton,					SIGNAL(clicked()),				this, SLOT(reset()));
	connect(restoreToolButton,				SIGNAL(clicked()),				this, SLOT(restoreLastBox()));
	connect(closeButton,					SIGNAL(clicked()),				this, SLOT(closeDialog()));

	connect(showBoxToolButton,				SIGNAL(toggled(bool)),			this, SLOT(toggleBox(bool)));
	connect(showInteractorsToolButton,		SIGNAL(toggled(bool)),			this, SLOT(toggleInteractors(bool)));

	connect(thickXDoubleSpinBox,			SIGNAL(valueChanged(double)),	this, SLOT(thicknessChanged(double)));
	connect(thickYDoubleSpinBox,			SIGNAL(valueChanged(double)),	this, SLOT(thicknessChanged(double)));
	connect(thickZDoubleSpinBox,			SIGNAL(valueChanged(double)),	this, SLOT(thicknessChanged(double)));

	connect(minusXShiftToolButton,			SIGNAL(clicked()),				this, SLOT(shiftXMinus()));
	connect(plusXShiftToolButton,			SIGNAL(clicked()),				this, SLOT(shiftXPlus()));
	connect(minusYShiftToolButton,			SIGNAL(clicked()),				this, SLOT(shiftYMinus()));
	connect(plusYShiftToolButton,			SIGNAL(clicked()),				this, SLOT(shiftYPlus()));
	connect(minusZShiftToolButton,			SIGNAL(clicked()),				this, SLOT(shiftZMinus()));
	connect(plusZShiftToolButton,			SIGNAL(clicked()),				this, SLOT(shiftZPlus()));

	viewButtonsFrame->setEnabled(true);
	connect(viewUpToolButton,				SIGNAL(clicked()),				this, SLOT(setTopView()));
	connect(viewDownToolButton,				SIGNAL(clicked()),				this, SLOT(setBottomView()));
	connect(viewFrontToolButton,			SIGNAL(clicked()),				this, SLOT(setFrontView()));
	connect(viewBackToolButton,				SIGNAL(clicked()),				this, SLOT(setBackView()));
	connect(viewLeftToolButton,				SIGNAL(clicked()),				this, SLOT(setLeftView()));
	connect(viewRightToolButton,			SIGNAL(clicked()),				this, SLOT(setRightView()));

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

	ccBBox box;
	ccGLMatrix transformation;
	m_clipBox->get(box, transformation);

	ccBoundingBoxEditorDlg bbeDlg(this);
	bbeDlg.setBaseBBox(box, false);
	bbeDlg.showInclusionWarning(false);
	bbeDlg.setWindowTitle("Edit clipping box");
	
	//show the box 'axes' (orientation)
	bbeDlg.showBoxAxes(true);
	//transformation.invert();
	bbeDlg.setBoxAxes(	transformation.getColumnAsVec3D(0),
						transformation.getColumnAsVec3D(1),
						transformation.getColumnAsVec3D(2) );

	if (!bbeDlg.exec())
		return;

	box = bbeDlg.getBox();
	m_clipBox->setBox(box);

	//construct the local box orientation matrix
	{
		CCVector3 X, Y, Z;
		bbeDlg.getBoxAxes(X, Y, Z);
		//make sure the vectors define an orthogonal basis
		Z = X.cross(Y);
		Y = Z.cross(X);

		X.normalize();
		Y.normalize();
		Z.normalize();
		ccGLMatrix rotMat;
		rotMat.setColumn(0, X);
		rotMat.setColumn(1, Y);
		rotMat.setColumn(2, Z);

		CCVector3 C = box.getCenter();
		ccGLMatrix transMat;
		transMat.setTranslation(-C);
		transMat = rotMat.inverse() * transMat;
		transMat.setTranslation(transMat.getTranslationAsVec3D() + C);

		m_clipBox->setGLTransformation(transMat.inverse());
	}

	//onBoxModified(&box); //DGM: automatically called by 'm_clipBox'

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}
}

void ccClippingBoxTool::toggleInteractors(bool state)
{
	if (m_clipBox)
		m_clipBox->setSelected(state);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

void ccClippingBoxTool::toggleBox(bool state)
{
	if (m_clipBox)
		m_clipBox->showBox(state);
	if (m_associatedWin)
		m_associatedWin->redraw();
}

bool ccClippingBoxTool::setAssociatedEntity(ccHObject* entity)
{
	restoreToolButton->setEnabled(false);
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

	if (!m_clipBox->setAssociatedEntity(entity))
	{
		//error message already issued
		ccLog::Error("An error occurred (see Console)");
		return false;
	}

	if (s_lastBoxParams.contains(entity->getUniqueID()))
	{
		restoreToolButton->setEnabled(true);
	}

	contourGroupBox->setEnabled(entity->isKindOf(CC_TYPES::POINT_CLOUD));

	//force visibility
	entity->setVisible(true);
	entity->setEnabled(true);

	if (m_associatedWin)
	{
		m_associatedWin->redraw();
	}

	//set proper "steps" value for slice thickness editors
	{
		CCVector3 diag = entity->getOwnBB().getDiagVec();
		thickXDoubleSpinBox->setSingleStep(diag.x / 100.0);
		thickYDoubleSpinBox->setSingleStep(diag.y / 100.0);
		thickZDoubleSpinBox->setSingleStep(diag.z / 100.0);
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
	{
		return false;
	}

	if (win)
	{
		if (!m_clipBox)
		{
			m_clipBox = new ccClipBox();
			m_clipBox->setVisible(true);
			m_clipBox->setEnabled(true);
			m_clipBox->setSelected(showInteractorsToolButton->isChecked());
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
	if (state && m_clipBox && m_clipBox->getAssociatedEntity())
	{
		//save clip box parameters
		ccClipBoxParams params;
		m_clipBox->get(params.box, params.trans);
		s_lastBoxParams[m_clipBox->getAssociatedEntity()->getUniqueID()] = params;
	}

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
				//obj->prepareDisplayForRefresh();
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

ccHObject* GetSlice(ccHObject* obj, ccClipBox* clipBox, bool silent)
{
	assert(clipBox);
	if (!obj)
	{
		assert(false);
		return 0;
	}

	if (obj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* inputCloud = ccHObjectCaster::ToGenericPointCloud(obj);

		ccGenericPointCloud::VisibilityTableType* selectionTable = new ccGenericPointCloud::VisibilityTableType;
		if (!selectionTable->resize(inputCloud->size()))
		{
			selectionTable->release();
			selectionTable = 0;
			
			if (!silent)
			{
				ccLog::Error("Not enough memory!");
			}
			return 0;
		}
		clipBox->flagPointsInside(inputCloud, selectionTable);
		
		ccGenericPointCloud* sliceCloud = inputCloud->createNewCloudFromVisibilitySelection(false, selectionTable);
		
		//we don't need the table anymore
		selectionTable->release();
		selectionTable = 0;

		if (!sliceCloud && !silent)
		{
			ccLog::Error("Not enough memory!");
		}
		return sliceCloud;
	}
	else if (obj->isKindOf(CC_TYPES::MESH))
	{
		const ccGLMatrix* _transformation = 0;
		ccGLMatrix transformation;
		if (clipBox->isGLTransEnabled())
		{
			transformation = clipBox->getGLTransformation().inverse();
			_transformation = &transformation;
		}

		const ccBBox& cropBox = clipBox->getBox();
		ccHObject* mesh = ccCropTool::Crop(obj, cropBox, true, _transformation);
		if (!mesh)
		{
			if (!silent)
				ccLog::Error("Failed to segment the mesh!");
			return 0;
		}
		return mesh;
	}

	return 0;
}

void ccClippingBoxTool::exportSlice()
{
	if (!m_clipBox || !MainWindow::TheInstance())
		return;

	ccHObject* obj = m_clipBox->getAssociatedEntity();
	if (!obj)
		return;

	ccHObject* result = GetSlice(obj, m_clipBox, false);

	if (result)
	{
		result->setName(obj->getName() + QString(".section"));
		result->setDisplay(obj->getDisplay());
		result->prepareDisplayForRefresh();
		if (obj->getParent())
			obj->getParent()->addChild(result);
		MainWindow::TheInstance()->addToDB(result);
	}
}

void ccClippingBoxTool::extractContour()
{
	extractSlicesAndContours(false, true, /*singleContourMode=*/true);
}

void ccClippingBoxTool::exportMultSlices()
{
	extractSlicesAndContours(true, true, /*singleContourMode=*/false);
}

void ccClippingBoxTool::extractSlicesAndContours(bool extractSlices, bool extractContours, bool singleContourMode)
{
	if (!m_clipBox)
		return;

	ccHObject* obj = m_clipBox->getAssociatedEntity();
	ccGenericPointCloud* cloud = obj ? ccHObjectCaster::ToGenericPointCloud(obj) : 0;
	if (!cloud)
	{
		ccLog::Warning("Only works with point clouds or meshes!");
		return;
	}

	QElapsedTimer eTimer;
	eTimer.start();

	ccClippingBoxRepeatDlg repeatDlg(singleContourMode, MainWindow::TheInstance());
	bool isCloud = obj->isKindOf(CC_TYPES::POINT_CLOUD);
	if (!isCloud)
	{
		repeatDlg.extractContoursGroupBox->setEnabled(false);
	}
	
	//by default we set the 'flat/repeat' dimension to the smallest box dimension
	{
		CCVector3 diagVec = m_clipBox->getOwnBB().getDiagVec();
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
	
	//random colors is only useful for mutliple slice/contour mode
	repeatDlg.randomColorCheckBox->setEnabled(!singleContourMode);
	
	//set default max edge length
	if (s_maxEdgeLength < 0)
		s_maxEdgeLength = static_cast<double>(obj->getOwnBB().getDiagNorm())/100.0;
	repeatDlg.maxEdgeLengthDoubleSpinBox->setValue(s_maxEdgeLength);
	repeatDlg.splitContourCheckBox->setChecked(s_splitContours);
	repeatDlg.multiPassCheckBox->setChecked(s_multiPass);
	repeatDlg.gapDoubleSpinBox->setValue(s_defaultGap);

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
	int gridDim[3]   = { 1, 1, 1 };
	unsigned cellCount = 1;
	CCVector3 gridOrigin = m_clipBox->getOwnBB().minCorner();
	CCVector3 cellSize = m_clipBox->getOwnBB().getDiagVec();
	s_defaultGap = repeatDlg.gapDoubleSpinBox->value();
	PointCoordinateType gap = static_cast<PointCoordinateType>(s_defaultGap);
	CCVector3 cellSizePlusGap = cellSize + CCVector3(gap,gap,gap);

	//for mutli-dimensional mode only!
	if (!singleContourMode)
	{
		ccBBox localBox;
		{
			for (unsigned i=0; i<cloud->size(); ++i)
			{
				CCVector3 P = *cloud->getPoint(i);
				localTrans.apply(P);
				localBox.add(P);
			}
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

				PointCoordinateType a = (localBox.minCorner().u[d] - gridOrigin.u[d]) / cellSizePlusGap.u[d]; //don't forget the user defined gap between 'cells'
				PointCoordinateType b = (localBox.maxCorner().u[d] - gridOrigin.u[d]) / cellSizePlusGap.u[d];

				indexMins[d] = static_cast<int>(floor(a + static_cast<PointCoordinateType>(1.0e-6)));
				indexMaxs[d] = static_cast<int>(ceil(b - static_cast<PointCoordinateType>(1.0e-6))) - 1;

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
		std::vector<ccHObject*> slices;
		try
		{
			slices.resize(cellCount,0);
		}
		catch (const std::bad_alloc&)
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
			assert(slices.size() == 1);

			slices[0] = GetSlice(obj, m_clipBox, false);
			if (!slices[0])
			{
				//error message already issued
				return;
			}
			slices[0]->setName(obj->getName() + QString(".slice"));
			subCloudsCount = 1;
		}
		else if (isCloud)
		{
			//we'll potentially create up to one (ref.) cloud per cell
			std::vector<CCLib::ReferenceCloud*> refClouds;
			{
				try
				{
					refClouds.resize(cellCount,0);
				}
				catch (const std::bad_alloc&)
				{
					ccLog::Error("Not enough memory!");
					return;
				}
			}

			//project points into grid
			{
				unsigned pointCount = cloud->size(); 

				ccProgressDialog pDlg(false, this);
				pDlg.setInfo(tr("Points: %1").arg(pointCount));
				pDlg.start();
				pDlg.show();
				QApplication::processEvents();

				CCLib::NormalizedProgress nProgress(&pDlg, pointCount);
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
						int cloudIndex = ((zi - indexMins[2]) * static_cast<int>(gridDim[1]) + (yi - indexMins[1])) * static_cast<int>(gridDim[0]) + (xi - indexMins[0]);
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

				ccProgressDialog pDlg(true, this);
				pDlg.setWindowTitle("Section extraction");
				pDlg.setInfo(tr("Section(s): %1").arg(subCloudsCount));
				pDlg.setMaximum(static_cast<int>(subCloudsCount));
				pDlg.show();
				QApplication::processEvents();
				
				//reset count
				subCloudsCount = 0;
				for (int i=indexMins[0]; i<=indexMaxs[0]; ++i)
				{
					for (int j=indexMins[1]; j<=indexMaxs[1]; ++j)
					{
						for (int k=indexMins[2]; k<=indexMaxs[2]; ++k)
						{
							int cloudIndex = ((k - indexMins[2]) * static_cast<int>(gridDim[1]) + (j - indexMins[1])) * static_cast<int>(gridDim[0]) + (i - indexMins[0]);
							assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex) < refClouds.size());

							if (refClouds[cloudIndex]) //some slices can be empty due to rounding issues!
							{
								//generate slice from previous selection
								int warnings = 0;
								ccPointCloud* sliceCloud = cloud->isA(CC_TYPES::POINT_CLOUD) ? static_cast<ccPointCloud*>(cloud)->partialClone(refClouds[cloudIndex], &warnings) : ccPointCloud::From(refClouds[cloudIndex], cloud);
								warningsIssued |= (warnings != 0);

								if (sliceCloud)
								{
									if (generateRandomColors)
									{
										ccColor::Rgb col = ccColor::Generator::Random();
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

									CCVector3 cellOrigin(	gridOrigin.x + i * cellSizePlusGap.x,
															gridOrigin.y + j * cellSizePlusGap.y,
															gridOrigin.z + k * cellSizePlusGap.z);
									QString slicePosStr = QString("(%1 ; %2 ; %3)").arg(cellOrigin.x).arg(cellOrigin.y).arg(cellOrigin.z);
									sliceCloud->setName(QString("slice @ ")+slicePosStr);

									//add slice to group
									sliceGroup->addChild(sliceCloud);
									//update 'real clouds' grid
									slices[cloudIndex] = sliceCloud;
									++subCloudsCount;
								}

								if (pDlg.wasCanceled())
								{
									error = true;
									ccLog::Warning(QString("[ccClippingBoxTool::extractSlicesAndContours] Process canceled by user"));
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
		else //mesh
		{
			const ccGLMatrix* _transformation = 0;
			ccGLMatrix transformation;
			if (m_clipBox->isGLTransEnabled())
			{
				transformation = m_clipBox->getGLTransformation().inverse();
				_transformation = &transformation;
			}

			sliceGroup = new ccHObject(QString("%1.slices").arg(obj->getName()));
			
			ccProgressDialog pDlg(true, this);
			pDlg.setWindowTitle("Section extraction");
			pDlg.setInfo(tr("Up to (%1 x %2 x %3) = %4 section(s)").arg(gridDim[0]).arg(gridDim[1]).arg(gridDim[2]).arg(cellCount));
			pDlg.setMaximum(static_cast<int>(cellCount));
			pDlg.show();
			QApplication::processEvents();
			
			//now extract the slices
			for (int i = indexMins[0]; i <= indexMaxs[0]; ++i)
			{
				for (int j = indexMins[1]; j <= indexMaxs[1]; ++j)
				{
					for (int k = indexMins[1]; k <= indexMaxs[2]; ++k)
					{
						int sliceIndex = ((k - indexMins[2]) * static_cast<int>(gridDim[1]) + (j - indexMins[1])) * static_cast<int>(gridDim[0]) + (i - indexMins[0]);

						CCVector3 C = gridOrigin + CCVector3(i*cellSizePlusGap.x, j*cellSizePlusGap.y, k*cellSizePlusGap.z);
						ccBBox cropBox(C, C + cellSize);

						ccHObject* croppedEnt = ccCropTool::Crop(obj, cropBox, true, _transformation);
						if (croppedEnt)
						{
							if (generateRandomColors)
							{
								ccPointCloud* croppedCloud = ccHObjectCaster::ToPointCloud(croppedEnt);
								if (croppedCloud)
								{
									ccColor::Rgb col = ccColor::Generator::Random();
									if (!croppedCloud->setRGBColor(col))
									{
										ccLog::Error("Not enough memory!");
										error = true;
										i = indexMaxs[0];
										j = indexMaxs[1];
										k = indexMaxs[2];
									}
									croppedCloud->showColors(true);
								}
							}

							croppedEnt->setEnabled(true);
							croppedEnt->setVisible(true);
							croppedEnt->setDisplay(cloud->getDisplay());

							QString slicePosStr = QString("(%1 ; %2 ; %3)").arg(C.x).arg(C.y).arg(C.z);
							croppedEnt->setName(QString("slice @ ") + slicePosStr);

							//add slice to group
							sliceGroup->addChild(croppedEnt);
							assert(sliceIndex >= 0 && static_cast<size_t>(sliceIndex) < slices.size());
							slices[sliceIndex] = croppedEnt;
						}

						if (pDlg.wasCanceled())
						{
							error = true;
							ccLog::Warning(QString("[ccClippingBoxTool::extractSlicesAndContours] Process canceled by user"));
							//early stop
							i = indexMaxs[0];
							j = indexMaxs[1];
							k = indexMaxs[2];
							break;
						}
						pDlg.setValue(sliceIndex);
					}
				}
			}
		}

		//extract contour polylines (optionaly) - for clouds only!
		if (!error && extractContours && subCloudsCount != 0)
		{
			//contour extraction parameter (max edge length)
			s_maxEdgeLength = repeatDlg.maxEdgeLengthDoubleSpinBox->value();
			s_multiPass = repeatDlg.multiPassCheckBox->isChecked();
			s_splitContours = repeatDlg.splitContourCheckBox->isChecked();
			bool visualDebugMode = repeatDlg.debugModeCheckBox->isChecked();

			ccHObject* contourGroup = new ccHObject(obj->getName() + QString(".contours"));

			ccProgressDialog pDlg(true, this);
			pDlg.setWindowTitle("Contour extraction");
			pDlg.setInfo(tr("Contour(s): %1").arg(subCloudsCount));
			pDlg.setMaximum(static_cast<int>(subCloudsCount));
			if (!visualDebugMode)
			{
				pDlg.show();
				QApplication::processEvents();
			}

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
						assert(cloudIndex >= 0 && static_cast<size_t>(cloudIndex) < slices.size());

						assert(!slices[cloudIndex] || slices[cloudIndex]->isA(CC_TYPES::POINT_CLOUD));
						ccPointCloud* sliceCloud = static_cast<ccPointCloud*>(slices[cloudIndex]);
						if (sliceCloud) //some slices can be empty due to rounding issues!
						{
							std::vector<ccPolyline*> polys;
							if (ccContourExtractor::ExtractFlatContour(	sliceCloud,
																		s_multiPass,
																		static_cast<PointCoordinateType>(s_maxEdgeLength),
																		polys,
																		s_splitContours,
																		preferredOrientation,
																		visualDebugMode))
							{
								if (!polys.empty())
								{
									for (size_t p=0; p<polys.size(); ++p)
									{
										ccPolyline* poly = polys[p];
										poly->setColor(ccColor::green);
										poly->showColors(true);
										poly->setGlobalScale(sliceCloud->getGlobalScale());
										poly->setGlobalShift(sliceCloud->getGlobalShift());
										QString contourName = sliceCloud->getName();
										contourName.replace("slice","contour");
										if (polys.size() != 1)
											contourName += QString(" (part %1)").arg(p+1);
										poly->setName(contourName);
										contourGroup->addChild(poly);
									}
								}
								else
								{
									ccLog::Warning(QString("%1: points are too far from each other! Increase the max edge length").arg(sliceCloud->getName()));
									warningsIssued = true;
								}
							}
							else
							{
								ccLog::Warning(QString("%1: contour extraction failed!").arg(sliceCloud->getName()));
								warningsIssued = true;
							}
						}
						
						++currentCloudCount;
						if (!visualDebugMode)
						{
							if (pDlg.wasCanceled())
							{
								error = true;
								ccLog::Warning(QString("[ccClippingBoxTool::extractSlicesAndContours] Process canceled by user"));
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
			slices.clear();
			delete sliceGroup;
			sliceGroup = 0;
		}
		else if (singleContourMode)
		{
			assert(!sliceGroup);
			assert(slices.size() == 1);
			delete slices[0]; //temporary cloud!
			slices.clear();
		}

		if (!error)
		{
			if (warningsIssued)
			{
				ccLog::Warning("[ccClippingBoxTool::extractSlicesAndContours] Warnings were issued during the process! (result may be incomplete)");
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
					QMessageBox::warning(0, "Process finished", QString("%1 slices have been generated.\n(you may have to close the tool and hide the initial cloud to see them...)").arg(sliceGroup->getChildrenNumber()));
					sliceGroup->setDisplay_recursive(cloud->getDisplay());
					MainWindow::TheInstance()->addToDB(sliceGroup);
				}
			}

			ccLog::Print("[ccClippingBoxTool] Processed finished in %.2f s.",eTimer.elapsed()/1.0e3);

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

void ccClippingBoxTool::shiftBox(unsigned char dim, bool minus)
{
	if (!m_clipBox || !m_clipBox->getBox().isValid())
		return;

	assert(dim<3);

	PointCoordinateType width = (m_clipBox->getBox().maxCorner() - m_clipBox->getBox().minCorner()).u[dim];
	CCVector3 shiftVec(0, 0, 0);
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

void ccClippingBoxTool::restoreLastBox()
{
	if (!m_clipBox || !m_clipBox->getAssociatedEntity())
	{
		assert(false);
		return;
	}
	
	unsigned uniqueID = m_clipBox->getAssociatedEntity()->getUniqueID();
	if (!s_lastBoxParams.contains(uniqueID))
	{
		assert(false);
		return;
	}

	const ccClipBoxParams& params = s_lastBoxParams[uniqueID];
	m_clipBox->set(params.box, params.trans);
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
