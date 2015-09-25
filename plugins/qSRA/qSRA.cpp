//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
//#                           COPYRIGHT: EDF                               #
//#                                                                        #
//##########################################################################

#include "qSRA.h"

//Local
#include "profileImportDlg.h"
#include "distanceMapGenerationDlg.h"
#include "profileLoader.h"

//Qt
#include <QtGui>
#include <QSettings>
#include <QFileInfo>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QMainWindow>

//qCC_db
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <ccCone.h>
#include <ccScalarField.h>

//System
#include <string.h>
#include <algorithm>
#include <vector>

qSRA::qSRA(QObject* parent/*=0*/)
	: QObject(parent)
	, m_doLoadProfile(0)
	, m_doCompareCloudToProfile(0)
	, m_doProjectCloudDists(0)
{
}

void qSRA::getActions(QActionGroup& group)
{
	//actions
	if (!m_doLoadProfile)
	{
		m_doLoadProfile = new QAction("Load profile",this);
		m_doLoadProfile->setToolTip("Loads the 2D profile of a surface of revolution from a dedicated ASCII file");
		m_doLoadProfile->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSRA/loadProfileIcon.png")));
		//connect signal
		connect(m_doLoadProfile, SIGNAL(triggered()), this, SLOT(loadProfile()));
	}
	group.addAction(m_doLoadProfile);

	if (!m_doCompareCloudToProfile)
	{
		m_doCompareCloudToProfile = new QAction("Cloud-profile radial distance",this);
		m_doCompareCloudToProfile->setToolTip("Computes radial distances between a cloud and a surface of revolution (described by its profile polyline)");
		m_doCompareCloudToProfile->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSRA/distToProfileIcon.png")));
		//connect signal
		connect(m_doCompareCloudToProfile, SIGNAL(triggered()), this, SLOT(computeCloud2ProfileRadialDist()));
	}
	group.addAction(m_doCompareCloudToProfile);

	if (!m_doProjectCloudDists)
	{
		m_doProjectCloudDists = new QAction("2D distance map",this);
		m_doProjectCloudDists->setToolTip("Creates the surface or revolution 2D distance map (unroll)");
		m_doProjectCloudDists->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSRA/createMapIcon.png")));
		//connect signal
		connect(m_doProjectCloudDists, SIGNAL(triggered()), this, SLOT(projectCloudDistsInGrid()));
	}
	group.addAction(m_doProjectCloudDists);
}

void qSRA::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_doLoadProfile)
	{
		//always active
	}

	bool cloudAndPolylineSelected = (	selectedEntities.size() == 2
										&& (	(selectedEntities[0]->isA(CC_TYPES::POLY_LINE) && selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
											||	(selectedEntities[1]->isA(CC_TYPES::POLY_LINE) && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD))) );
	
	if (m_doCompareCloudToProfile)
	{
		m_doCompareCloudToProfile->setEnabled(cloudAndPolylineSelected);
	}

	if (m_doProjectCloudDists)
	{
		m_doProjectCloudDists->setEnabled(	cloudAndPolylineSelected 
											|| (selectedEntities.size() == 2 && 
												(  (selectedEntities[0]->isKindOf(CC_TYPES::CONE) && selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
												|| (selectedEntities[1]->isKindOf(CC_TYPES::CONE) && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD)))));
	}

	//backup selected entities
	m_selectedEntities = selectedEntities;
}

//return (and create if necessary) the plugin default destination container
const QString QSRA_DEFAULT_CONTAINER_NAME("Profile(s)");
ccHObject* GetDefaultContainer(ccMainAppInterface* app)
{
	assert(app);
	if (!app || !app->dbRootObject())
		return 0;

	//we look in qCC database for a group with the right name (i.e. if it has already been created)
	ccHObject::Container groups;
	app->dbRootObject()->filterChildren(groups,true,CC_TYPES::HIERARCHY_OBJECT);
	for (unsigned j=0; j<groups.size(); ++j)
	{
		if (groups[j]->getName() == QSRA_DEFAULT_CONTAINER_NAME)
			return groups[j];
	}

	//otherwise we create it
	ccHObject* defaultContainer = new ccHObject(QSRA_DEFAULT_CONTAINER_NAME);
	app->addToDB(defaultContainer);

	return defaultContainer;
}

void qSRA::loadProfile() const
{
	assert(m_app);
	if (!m_app)
		return;

	//persistent settings (default import path)
	QSettings settings;
	settings.beginGroup("qSRA");
	QString path = settings.value("importPath",QApplication::applicationDirPath()).toString();

	ProfileImportDlg piDlg(m_app->getMainWindow());
	piDlg.setDefaultFilename(path);

	if (!piDlg.exec())
		return;

	QString filename = piDlg.getFilename();
	if (filename.isEmpty())
		return;
	
	//save current import path to persistent settings
	settings.setValue("importPath",QFileInfo(filename).absolutePath());

	//get the user defined global axis
	int axisDim = piDlg.getAxisDimension();
	assert(axisDim >= 0 && axisDim <= 2);

	//load profile as a (2D) polyline
	CCVector3 origin(0,0,0);
	ccPolyline* polyline = ProfileLoader::Load(filename, origin, m_app);
	if (!polyline)
	{
		if (m_app)
			m_app->dispToConsole(QString("Failed to load file '%1'!").arg(filename),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//DGM: the following works only because the axis is colinear with X, Y or Z!
	PointCoordinateType heightShift = 0;
	if (!piDlg.absoluteHeightValues())
		heightShift = origin.u[axisDim]; //the profile height values are relative to the origin!
	else
		origin.u[axisDim] = 0;
	
	//apply a visual transformation to see the polyline in the right place
	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(polyline->getAssociatedCloud());
	if (vertices)
	{
		ccGLMatrix trans;
		trans.setTranslation(origin);
		float* mat = trans.data();
		switch (axisDim)
		{
		case 0: //X
			//invert X and Y
			mat[0] = 0;	mat[1] = 1;
			mat[4] = 1;	mat[5] = 0;
			break;
			//case 1: //Y
		case 2: //Z
			//invert Z and Y
			mat[5] = 0;	mat[6] = 1;
			mat[9] = 1;	mat[10] = 0;
			break;
		default:
			//nothing to do
			break;
		}
		polyline->set2DMode(false);
		polyline->setGLTransformation(trans);
	}

	//set meta-data
	DistanceMapGenerationTool::SetPoylineOrigin(polyline,origin);
	DistanceMapGenerationTool::SetPoylineRevolDim(polyline,axisDim);
	DistanceMapGenerationTool::SetPolylineHeightShift(polyline,heightShift);

	//default destination container
	ccHObject* defaultContainer = GetDefaultContainer(m_app);
	if (defaultContainer)
		defaultContainer->addChild(polyline);

	m_app->addToDB(polyline,true,false,true);

	m_app->dispToConsole(QString("[qSRA] File '%1' succesfully loaded").arg(filename),ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

void qSRA::computeCloud2ProfileRadialDist() const
{
	assert(m_app);

	if (m_selectedEntities.size() != 2)
		return;

	//retrieve input cloud and polyline
	ccPointCloud* cloud = 0;
	ccPolyline* polyline = 0;
	{
		for (unsigned i=0; i<2; ++i)
		{
			if (m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
				cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
			else if (m_selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
				polyline = static_cast<ccPolyline*>(m_selectedEntities[i]);
		}
	}

	if (!cloud || !polyline)
	{
		if (m_app)
			m_app->dispToConsole(QString("Select exactly one cloud and one polyline!"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (doComputeRadialDists(cloud, polyline))
	{
		//automatically ask the user if he wants to generate a 2D map
		if (QMessageBox::question(	m_app ? m_app->getMainWindow() : 0,
									"Generate map",
									"Do you want to generate a 2D map?",
									QMessageBox::Yes,
									QMessageBox::No) == QMessageBox::Yes)
		{
			doProjectCloudDistsInGrid(cloud,polyline);
		}
	}
}

bool qSRA::doComputeRadialDists(ccPointCloud* cloud, ccPolyline* polyline) const
{
	if (!cloud || !polyline)
	{
		assert(false);
		return false;
	}

	if (DistanceMapGenerationTool::ComputeRadialDist(cloud, polyline, false, m_app))
	{
		cloud->prepareDisplayForRefresh();
		if (m_app)
		{
			m_app->updateUI();
			m_app->refreshAll();
		}
		return true;
	}
	else
	{
		if (m_app)
			m_app->dispToConsole("An error ocurred while computing radial distances!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return false;
	}
}

void qSRA::projectCloudDistsInGrid() const
{
	size_t selectCount = m_selectedEntities.size();
	if (selectCount != 1 && selectCount != 2)
		return;

	//retrieve input cloud and polyline
	ccPointCloud* cloud = 0;
	ccPolyline* polyline = 0;
	bool tempPolyline = false;
	{
		for (size_t i=0; i<selectCount; ++i)
		{
			if (m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			{
				cloud = static_cast<ccPointCloud*>(m_selectedEntities[i]);
			}
			else if (m_selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
			{
				polyline = static_cast<ccPolyline*>(m_selectedEntities[i]);
			}
			else if (!polyline && m_selectedEntities[i]->isKindOf(CC_TYPES::CONE))
			{
				//special case: we can deduce the polyline from the cone/cylinder parameters
				ccCone* cone = static_cast<ccCone*>(m_selectedEntities[i]);
				{
					ccGLMatrix& mat = cone->getTransformation();

					CCVector3 axis = mat.getColumnAsVec3D(2);
					CCVector3 origin = mat.getTranslationAsVec3D();
					PointCoordinateType height = cone->getHeight();

					ccPointCloud* vertices = new ccPointCloud("vertices");
					vertices->reserve(2);
					vertices->addPoint(CCVector3(	cone->getBottomRadius(),
													-height/2,
													0 ));
					vertices->addPoint(CCVector3(	cone->getTopRadius(),
													height/2,
													0 ));


					polyline = new ccPolyline(vertices);
					polyline->addChild(vertices);
					polyline->reserve(2);
					polyline->addPointIndex(0,2);
					polyline->setClosed(false);
					tempPolyline = true;

					//set meta-data
					DistanceMapGenerationTool::SetPoylineOrigin(polyline, origin);
					DistanceMapGenerationTool::SetPoylineAxis(polyline, axis);
					DistanceMapGenerationTool::SetPolylineHeightShift(polyline, height/2);

					//we'll use the 'largest' axis dimension as 'revolution dimension'
					int revolDim = 0;
					for (int i=1; i<3; ++i)
						if (fabs(axis.u[i]) > fabs(axis.u[revolDim]))
							revolDim = i;
					DistanceMapGenerationTool::SetPoylineRevolDim(polyline, revolDim);

				}
			}
		}
	}

	if (cloud && polyline)
	{
		doProjectCloudDistsInGrid(cloud,polyline);
	}

	if (polyline && tempPolyline)
	{
		delete polyline;
		polyline = 0;
	}
}

void qSRA::doProjectCloudDistsInGrid(ccPointCloud* cloud, ccPolyline* polyline) const
{
	assert(cloud && m_app);
	if (!cloud)
		return;

	//get associated scalar field
	ccScalarField* sf = cloud->getCurrentDisplayedScalarField();
	if (!sf)
	{
		int sfIdx = cloud->getScalarFieldIndexByName(RADIAL_DIST_SF_NAME);
		if (sfIdx < 0)
		{
			QString message = QString("Cloud has no activated scalar field and no '%1' field!").arg(RADIAL_DIST_SF_NAME);
			if (QMessageBox::question(	m_app->getMainWindow(),
										"Distance field",
										message + QString("\nDo you want to compute it now?"),
										QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes )
			{
				if (doComputeRadialDists(cloud, polyline))
				{
					sfIdx = cloud->getScalarFieldIndexByName(RADIAL_DIST_SF_NAME);
					assert(sfIdx >= 0);
					
					//TEST
					//return;
				}
				else
				{
					//radial dist. computation failed
					return;
				}
			}
			else
			{
				//process is cancelled
				return;
			}
		}
		if (sfIdx >= 0)
		{
			sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
		}
		else
		{
			if (m_app)
				m_app->dispToConsole("Internal error: no radial distances SF found!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			return;
		}
	}
	assert(sf);

	DistanceMapGenerationDlg dmgDlg(cloud, sf, polyline, m_app);

	dmgDlg.exec();
}

QIcon qSRA::getIcon() const
{
	return QIcon(QString::fromUtf8(":/CC/plugin/qSRA/qSRA.png"));
}

#ifndef CC_QT5
Q_EXPORT_PLUGIN2(qSRA,qSRA);
#endif
