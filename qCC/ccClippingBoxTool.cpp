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
#include "ccConsole.h"
#include "mainwindow.h"
#include "ccClippingBoxRepeatDlg.h"

//qCC_db
#include <ccHObject.h>
#include <ccClipBox.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>

//CCLib
#include <ReferenceCloud.h>

//Qt
#include <QProgressDialog>
#include <QMessageBox>

ccClippingBoxTool::ccClippingBoxTool(QWidget* parent)
	: ccOverlayDialog(parent)
	, Ui::ClippingBoxDlg()
	, m_clipBox(0)
{
	setupUi(this);

	setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);

	connect(exportButton,		SIGNAL(clicked()), this, SLOT(exportCloud()));
	connect(exportMultButton,	SIGNAL(clicked()), this, SLOT(exportMultCloud()));
	connect(resetButton,		SIGNAL(clicked()), this, SLOT(reset()));
	connect(closeButton,		SIGNAL(clicked()), this, SLOT(closeDialog()));

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
}

ccClippingBoxTool::~ccClippingBoxTool()
{
	if (m_clipBox)
		delete m_clipBox;
	m_clipBox = 0;
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
		ccConsole::Error(QString("[Clipping box] No associated 3D view or no valid clipping box!"));
		return false;
	}

	//we don't handle entities associated to another context
	if (entity->getDisplay() != m_associatedWin)
	{
		ccConsole::Warning(QString("[Clipping box] Can't use entity '%1' cause it's not displayed in the active 3D view!").arg(entity->getName()));
		return false;
	}

	//we can't handle other entities than clouds for the moment
	if (!entity->isA(CC_POINT_CLOUD))
	{
		ccConsole::Warning(QString("[Clipping box] Only points clouds are handled! Entity '%1' will be ignored.").arg(entity->getName()));
		return false;
	}

	//force visibility
	entity->setVisible(true);
	entity->setEnabled(true);

	m_clipBox->setAssociatedEntity(entity);
	if (m_associatedWin)
		m_associatedWin->redraw();

	return true;
}

bool ccClippingBoxTool::linkWith(ccGLWindow* win)
{
	if (m_associatedWin && m_clipBox)
	{
		//remove clipping box from precedent window
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
			m_clipBox->setSelected(true);
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

void ccClippingBoxTool::exportCloud()
{
	if (!m_clipBox)
		return;

	ccHObject* obj = m_clipBox->getAssociatedEntity();

	if (obj && obj->isKindOf(CC_POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(obj)->createNewCloudFromVisibilitySelection(false);
		MainWindow::TheInstance()->addToDB(cloud);
	}
}

void ccClippingBoxTool::exportMultCloud()
{
	if (!m_clipBox)
		return;
	ccHObject* obj = m_clipBox->getAssociatedEntity();
	if (!obj || !obj->isA(CC_POINT_CLOUD))
	{
		ccLog::Warning("Only works with point clouds!");
		return;
	}
	ccPointCloud* cloud = static_cast<ccPointCloud*>(obj);

	ccClippingBoxRepeatDlg repeatDlg(MainWindow::TheInstance()/*this*/);
	repeatDlg.randomColorCheckBox->setEnabled(cloud->isA(CC_POINT_CLOUD)); //random colors is only available for real point clouds!
	
	if (!repeatDlg.exec())
		return;

	//compute the cloud bounding box in the local clipping box ref.
	ccBBox localBox;
	ccGLMatrix localTrans;
	if (m_clipBox->isGLTransEnabled())
		localTrans = m_clipBox->getGLTransformation().inverse();
	else
		localTrans.toIdentity();
	{
		for (unsigned i=0; i<cloud->size(); ++i)
		{
			CCVector3 P = *cloud->getPoint(i);
			localTrans.apply(P);
			localBox.add(P);
		}
	}

	//compute 'grid' extents in the local clipping box ref.
	bool processDim[3] = {	repeatDlg.xRepeatCheckBox->isChecked(),
							repeatDlg.yRepeatCheckBox->isChecked(),
							repeatDlg.zRepeatCheckBox->isChecked() };
	int indexMins[3] = { 0 , 0, 0 };
	int indexMaxs[3] = { 0 , 0, 0 };
	int gridDim[3] = { 0 , 0, 0 };
	unsigned cellCount = 1;
	CCVector3 gridOrigin = m_clipBox->getBB().minCorner();
	CCVector3 cellSize = m_clipBox->getBB().getDiagVec();
	PointCoordinateType gap = (PointCoordinateType)repeatDlg.gapDoubleSpinBox->value();
	{
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

				indexMins[d] = (int)floor(a);
				indexMaxs[d] = (int)ceil(b)-1;
				
				assert(indexMaxs[d] >= indexMins[d]);
				gridDim[d] = std::max(indexMaxs[d] - indexMins[d] + 1, 1);
				cellCount *= (unsigned)gridDim[d];
			}
		}
	}

	//apply process
	{
		bool generateRandomColors = repeatDlg.randomColorCheckBox->isChecked();

		//backup original clipping box
		ccBBox originalBox = m_clipBox->getBox();

		//group to store all the resulting slices
		ccHObject* group = new ccHObject(QString("%1.slices").arg(cloud->getName()));

		//grid of potential clouds
		CCLib::ReferenceCloud** clouds = new CCLib::ReferenceCloud*[cellCount];
		if (!clouds)
		{
			ccLog::Error("Not enough memory!");
			return;
		}
		memset(clouds, 0, sizeof(CCLib::ReferenceCloud*)*cellCount);

		//project points into grid
		bool error = false;
		{
			QProgressDialog pDlg(this);
			pDlg.show();
			QApplication::processEvents();

			unsigned pointCount = cloud->size(); 
			for (unsigned i=0; i<pointCount; ++i)
			{
				CCVector3 P = *cloud->getPoint(i);
				localTrans.apply(P);

				//relative coordinates (between 0 and 1)
				P -= gridOrigin;
				P.x /= (cellSize.x+gap);
				P.y /= (cellSize.y+gap);
				P.z /= (cellSize.z+gap);

				int xi = (int)P.x; if (xi == gridDim[0]) xi--;
				int yi = (int)P.y; if (yi == gridDim[1]) yi--;
				int zi = (int)P.z; if (zi == gridDim[2]) zi--;

				if (gap == 0 ||
					(	P.x-(PointCoordinateType)xi <= cellSize.x
					&&	P.y-(PointCoordinateType)yi <= cellSize.y
					&&	P.z-(PointCoordinateType)zi <= cellSize.z))
				{
					int cloudIndex = ((zi-indexMins[2]) * (int)gridDim[1] + (yi-indexMins[1])) * (int)gridDim[0] + (xi-indexMins[0]);
					if (!clouds[cloudIndex])
					{
						clouds[cloudIndex] = new CCLib::ReferenceCloud(cloud);
					}

					if (!clouds[cloudIndex]->addPointIndex(i))
					{
						ccLog::Error("Not enough memory!");
						error = true;
						break;
					}
				}

				pDlg.setValue((int)floor(100.0f*(float)i/(float)pointCount));
			}
		}

		if (!error)
		{
			QProgressDialog pDlg(this);
			pDlg.show();
			QApplication::processEvents();

			bool warningsIssued = false;

			unsigned currentCloudCount = 0;
			for (int i=indexMins[0]; i<=indexMaxs[0]; ++i)
			{
				for (int j=indexMins[1]; j<=indexMaxs[1]; ++j)
				{
					for (int k=indexMins[2]; k<=indexMaxs[2]; ++k)
					{
						int cloudIndex = ((k-indexMins[2]) * (int)gridDim[1] + (j-indexMins[1])) * (int)gridDim[0] + (i-indexMins[0]);

						if (clouds[cloudIndex]) //some slices can be empty!
						{
							int warnings = 0;
							ccPointCloud* sliceCloud = cloud->partialClone(clouds[cloudIndex],&warnings);
							warningsIssued |= (warnings != 0);
							
							if (sliceCloud)
							{
								if (generateRandomColors && cloud->isA(CC_POINT_CLOUD))
								{
									colorType col[3];
									col[0] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
									col[1] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
									col[2] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
									if (!static_cast<ccPointCloud*>(sliceCloud)->setRGBColor(col))
									{
										ccLog::Error("Not enough memory!");
										error = true;
										i = indexMaxs[0]; j = indexMaxs[1]; k = indexMaxs[2];
									}
									sliceCloud->showColors(true);
								}

								sliceCloud->setEnabled(true);
								sliceCloud->setVisible(true);

								CCVector3 cellOrigin(gridOrigin.x + (PointCoordinateType)i * (cellSize.x + gap),
													 gridOrigin.y + (PointCoordinateType)j * (cellSize.y + gap),
													 gridOrigin.z + (PointCoordinateType)k * (cellSize.z + gap));
								sliceCloud->setName(QString("slice @ (%1 ; %2 ; %3)").arg(cellOrigin.x).arg(cellOrigin.y).arg(cellOrigin.z));

								//add slice to group
								group->addChild(sliceCloud);
							}

							++currentCloudCount;
							pDlg.setValue((int)floor(100.0f*(float)currentCloudCount/(float)cellCount));
						}
					}
				}
			}

			if (warningsIssued)
			{
				ccLog::Warning("[ccClippingBoxTool::exportMultCloud] Warnings were issued during the process! Result may be incomplete!");
			}
		}

		if (error)
		{
			group->removeAllChildren();
		}

		//release memory
		{
			for (unsigned i=0; i<cellCount; ++i)
				if (clouds[i])
					delete clouds[i];
			delete[] clouds;
			clouds = 0;
		}

		//unsigned currentCell = 0;
		//for (int i=indexMins[0]; i<=indexMaxs[0]; ++i)
		//{
		//	ccBBox cellBox;
		//	cellBox.minCorner().x = gridOrigin.x + (PointCoordinateType)i * (cellSize.x + gap);
		//	cellBox.maxCorner().x = cellBox.minCorner().x + cellSize.x;
		//	for (int j=indexMins[1]; j<=indexMaxs[1]; ++j)
		//	{
		//		cellBox.minCorner().y = gridOrigin.y + (PointCoordinateType)j * (cellSize.y + gap);
		//		cellBox.maxCorner().y = cellBox.minCorner().y + cellSize.y;
		//		for (int k=indexMins[2]; k<=indexMaxs[2]; ++k, ++currentCell)
		//		{
		//			cellBox.minCorner().z = gridOrigin.z + (PointCoordinateType)k * (cellSize.z + gap);
		//			cellBox.maxCorner().z = cellBox.minCorner().z + cellSize.z;

		//			m_clipBox->setBox(cellBox);

		//			//count the number of visible slices
		//			unsigned visibleCount = 0;
		//			{
		//				ccGenericPointCloud::VisibilityTableType* visArray = cloud->getTheVisibilityArray();
		//				assert(visArray);
		//				if (visArray)
		//				{
		//					for (unsigned i=0; i<cloud->size(); ++i)
		//						if (visArray->getValue(i) == POINT_VISIBLE)
		//							++visibleCount;
		//				}
		//			}

		//			if (visibleCount) //some slices can be empty!
		//			{
		//				ccGenericPointCloud* sliceCloud = cloud->createNewCloudFromVisibilitySelection(false);
		//				if (sliceCloud)
		//				{
		//					if (generateRandomColors && cloud->isA(CC_POINT_CLOUD))
		//					{
		//						colorType col[3];
		//						col[0] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
		//						col[1] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
		//						col[2] = colorType(float(MAX_COLOR_COMP)*float(rand())/float(RAND_MAX));
		//						static_cast<ccPointCloud*>(sliceCloud)->setRGBColor(col);
		//						sliceCloud->showColors(true);
		//					}

		//					sliceCloud->setEnabled(true);
		//					sliceCloud->setVisible(true);
		//					sliceCloud->setName(QString("slice @ (%1 ; %2 ; %3)").arg(cellBox.minCorner().x).arg(cellBox.minCorner().y).arg(cellBox.minCorner().z));
		//					//add slice to group
		//					group->addChild(sliceCloud);
		//				}
		//			}

		//			pDlg.setValue((int)floor(100.0f*(float)currentCell/(float)cellCount));
		//		}
		//	}
		//}

		if (group->getChildrenNumber() == 0)
		{
			ccLog::Warning("[ccClippingBoxTool] Repeat process generated no output!");
			delete group;
			group = 0;
		}
		else
		{
			QMessageBox::warning(0, "Process finished", QString("%1 clouds have been generated.\n(you may have to close the tool and hide the initial cloud to see them...)").arg(group->getChildrenNumber()));
			group->setDisplay_recursive(cloud->getDisplay());
			MainWindow::TheInstance()->addToDB(group);
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

	CCVector3 th(thickXDoubleSpinBox->value(),
				 thickYDoubleSpinBox->value(),
				 thickZDoubleSpinBox->value());

	ccBBox box = m_clipBox->getBox();
	CCVector3 boxCenter = (box.maxCorner() + box.minCorner())/2.0;

	box.minCorner() = boxCenter - th/2.0;
	box.maxCorner() = boxCenter + th/2.0;

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

		ccGLMatrix rotMat = glMat; rotMat.setTranslation(CCVector3(0.0,0.0,0.0));

		//CCVector3 T = CCVector3(glMat.getTranslation()) - params.pivotPoint;
		//rotMat.inverse().apply(T);
		//T += params.pivotPoint;
		//params.viewMat.apply(T);
		//params.cameraCenter -= T;

		params.viewMat = params.viewMat * rotMat.inverse();
		m_associatedWin->setViewportParameters(params);

		//ccLog::Print(QString("X(%1,%2,%3)").arg(glMat.getColumn(0)[0]).arg(glMat.getColumn(0)[1]).arg(glMat.getColumn(0)[2]));
		//ccLog::Print(QString("Y(%1,%2,%3)").arg(glMat.getColumn(1)[0]).arg(glMat.getColumn(1)[1]).arg(glMat.getColumn(1)[2]));
		//ccLog::Print(QString("Z(%1,%2,%3)").arg(glMat.getColumn(2)[0]).arg(glMat.getColumn(2)[1]).arg(glMat.getColumn(2)[2]));
	}
    //m_associatedWin->blockSignals(false);
    m_associatedWin->redraw();
}
