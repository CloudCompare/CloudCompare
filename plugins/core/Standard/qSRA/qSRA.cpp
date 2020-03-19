//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qSRA                         #
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
#include <ccFileUtils.h>
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
	, ccStdPluginInterface(":/CC/plugin/qSRA/info.json")
	, m_doLoadProfile(nullptr)
	, m_doCompareCloudToProfile(nullptr)
	, m_doProjectCloudDists(nullptr)
{
}

QList<QAction *> qSRA::getActions()
{
	//actions
	if (!m_doLoadProfile)
	{
		m_doLoadProfile = new QAction("Load profile",this);
		m_doLoadProfile->setToolTip("Loads the 2D profile of a Surface of Revolution (from a dedicated ASCII file)");
		m_doLoadProfile->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSRA/loadProfileIcon.png")));
		//connect signal
		connect(m_doLoadProfile, &QAction::triggered, this, &qSRA::loadProfile);
	}

	if (!m_doCompareCloudToProfile)
	{
		m_doCompareCloudToProfile = new QAction("Cloud-SurfRev radial distance",this);
		m_doCompareCloudToProfile->setToolTip("Computes the radial distances between a cloud and a Surface of Revolution (polyline/profile, cone or cylinder)");
		m_doCompareCloudToProfile->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSRA/distToProfileIcon.png")));
		//connect signal
		connect(m_doCompareCloudToProfile, &QAction::triggered, this, &qSRA::computeCloud2ProfileRadialDist);
	}

	if (!m_doProjectCloudDists)
	{
		m_doProjectCloudDists = new QAction("2D distance map",this);
		m_doProjectCloudDists->setToolTip("Creates the 2D deviation map (radial distances) from a Surface or Revolution (unroll)");
		m_doProjectCloudDists->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qSRA/createMapIcon.png")));
		//connect signal
		connect(m_doProjectCloudDists, &QAction::triggered, this, &qSRA::projectCloudDistsInGrid);
	}

	return QList<QAction *>{
				m_doLoadProfile,
				m_doCompareCloudToProfile,
				m_doProjectCloudDists,
	};
}

void qSRA::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_doLoadProfile)
	{
		//always active
	}

	bool validSelection = false;
	if (selectedEntities.size() == 2)
	{
		//we expect a cloud...
		int cloudIndex = selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD) ? 0 : selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD) ? 1 : -1;
		if (cloudIndex != -1)
		{
			//... and either a polyline or a cone/cylinder
			validSelection = (selectedEntities[1 - cloudIndex]->isA(CC_TYPES::POLY_LINE) || selectedEntities[1 - cloudIndex]->isKindOf(CC_TYPES::CONE));
		}
	}

	if (m_doCompareCloudToProfile)
	{
		m_doCompareCloudToProfile->setEnabled(validSelection);
	}

	if (m_doProjectCloudDists)
	{
		m_doProjectCloudDists->setEnabled(validSelection);
	}
}

//return (and create if necessary) the plugin default destination container
const QString QSRA_DEFAULT_CONTAINER_NAME("Profile(s)");
ccHObject* GetDefaultContainer(ccMainAppInterface* app)
{
	if (!app || !app->dbRootObject())
	{
		assert(false);
		return nullptr;
	}

	//we look in qCC database for a group with the right name (i.e. if it has already been created)
	ccHObject::Container groups;
	app->dbRootObject()->filterChildren(groups,true,CC_TYPES::HIERARCHY_OBJECT);
	for (size_t j = 0; j < groups.size(); ++j)
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
	if (!m_app)
	{
		assert(false);
		return;
	}

	//persistent settings (default import path)
	QSettings settings;
	settings.beginGroup("qSRA");
	QString path = settings.value("importPath", ccFileUtils::defaultDocPath()).toString();

	ProfileImportDlg piDlg(m_app->getMainWindow());
	piDlg.setDefaultFilename(path);

	if (!piDlg.exec())
		return;

	QString filename = piDlg.getFilename();
	if (filename.isEmpty())
		return;

	//save current import path to persistent settings
	settings.setValue("importPath", QFileInfo(filename).absolutePath());

	//get the user defined global axis
	int axisDim = piDlg.getAxisDimension();
	assert(axisDim >= 0 && axisDim <= 2);

	//load profile as a (2D) polyline
	CCVector3 origin(0, 0, 0);
	ccPolyline* polyline = ProfileLoader::Load(filename, origin, m_app);
	if (!polyline)
	{
		if (m_app)
			m_app->dispToConsole(QString("Failed to load file '%1'!").arg(filename), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//DGM: the following works only because the axis is colinear with X, Y or Z!
	PointCoordinateType heightShift = 0;
	if (piDlg.absoluteHeightValues())
	{
		heightShift = -origin.u[axisDim];
	}

	//apply a visual transformation to see the polyline in the right place
	{
		ccGLMatrix trans;
		CCVector3 T = origin;
		T.u[axisDim] += heightShift;
		trans.setTranslation(T);
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
	DistanceMapGenerationTool::SetPoylineOrigin(polyline, origin);
	DistanceMapGenerationTool::SetPoylineRevolDim(polyline, axisDim);
	DistanceMapGenerationTool::SetPolylineHeightShift(polyline, heightShift);

	//default destination container
	ccHObject* defaultContainer = GetDefaultContainer(m_app);
	if (defaultContainer)
	{
		defaultContainer->addChild(polyline);
	}

	m_app->addToDB(polyline, true, false, true);

	m_app->dispToConsole(QString("[qSRA] File '%1' successfully loaded").arg(filename), ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

//helper
static ccPolyline* GetConeProfile(ccCone* cone)
{
	if (!cone)
	{
		assert(false);
		return nullptr;
	}

	//we deduce the profile orientation and position from the cone 4x4 transformation
	ccGLMatrix& coneTrans = cone->getTransformation();

	CCVector3 axis = coneTrans.getColumnAsVec3D(2);
	CCVector3 origin = coneTrans.getTranslationAsVec3D();
	PointCoordinateType height = cone->getHeight();
	//we'll use the 'largest' axis dimension as 'revolution dimension'
	int revolDim = 0;
	for (int i = 1; i<3; ++i)
		if (fabs(axis.u[i]) > fabs(axis.u[revolDim]))
			revolDim = i;

	//the profile has only one segment
	ccPointCloud* vertices = new ccPointCloud("vertices");
	{
		if (!vertices->reserve(2))
		{
			delete vertices;
			ccLog::Error("Not enough memory");
			return nullptr;
		}

		vertices->addPoint(CCVector3(cone->getBottomRadius(), -height/2, 0));
		vertices->addPoint(CCVector3(cone->getTopRadius(),	   height/2, 0));
	}

	ccPolyline* polyline = new ccPolyline(vertices);
	{
		polyline->addChild(vertices);
		if (!polyline->reserve(2))
		{
			delete polyline;
			ccLog::Error("Not enough memory");
			return nullptr;
		}
		polyline->addPointIndex(0, 2);
		polyline->setClosed(false);
	}

	//apply a visual transformation to see the polyline in the right place
	{
		CCVector3 y(0, 1, 0);
		CCVector3 Z(0, 0, 0);
		Z.u[revolDim] = PC_ONE;
		ccGLMatrix axisTrans = ccGLMatrix::FromToRotation(y, Z);
		assert(((axisTrans * y) - Z).norm() < ZERO_TOLERANCE);
		ccGLMatrix polyMat = coneTrans * axisTrans;
		polyline->setGLTransformation(polyMat);
	}

	//set meta-data
	DistanceMapGenerationTool::SetPoylineOrigin(polyline, origin);
	DistanceMapGenerationTool::SetPoylineAxis(polyline, axis);
	DistanceMapGenerationTool::SetPolylineHeightShift(polyline, 0/*height / 2*/);
	DistanceMapGenerationTool::SetPoylineRevolDim(polyline, revolDim);

	return polyline;
}

void qSRA::computeCloud2ProfileRadialDist() const
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	if (selectedEntities.size() != 2)
	{
		assert(false);
		return;
	}

	//retrieve input cloud and polyline
	ccPointCloud* cloud = nullptr;
	ccPolyline* polyline = nullptr;
	bool tempPolyline = false;
	{
		for (unsigned i = 0; i < 2; ++i)
		{
			if (selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			{
				cloud = static_cast<ccPointCloud*>(selectedEntities[i]);
			}
			else if (selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
			{
				polyline = static_cast<ccPolyline*>(selectedEntities[i]);
			}
			else if (!polyline && selectedEntities[i]->isKindOf(CC_TYPES::CONE))
			{
				//special case: we can deduce the polyline from the cone/cylinder parameters
				ccCone* cone = static_cast<ccCone*>(selectedEntities[i]);
				polyline = GetConeProfile(cone);
				if (!polyline)
				{
					//the conversion failed?!
					return;
				}
				tempPolyline = true;

#ifdef _DEBUG
				//test: apply a visual transformation to see the polyline in the right place
				{
					polyline->set2DMode(false);
					polyline->setWidth(2);
					polyline->setColor(ccColor::green);
					polyline->showColors(true);
					cone->setVisible(false);
					cone->addChild(polyline);
					m_app->addToDB(polyline);
					tempPolyline = false;
				}
#endif
			}
		}
	}

	if (cloud && polyline)
	{
		if (doComputeRadialDists(cloud, polyline))
		{
			//automatically ask the user if he wants to generate a 2D map
			if (QMessageBox::question(	m_app ? m_app->getMainWindow() : nullptr,
										"Generate map",
										"Do you want to generate a 2D deviation map?",
										QMessageBox::Yes,
										QMessageBox::No) == QMessageBox::Yes)
			{
				doProjectCloudDistsInGrid(cloud, polyline);
			}
		}
	}
	else
	{
		if (m_app)
			m_app->dispToConsole(QString("Select exactly one cloud and one Surface of Revolution (polyline/profile, cone or cylinder)"),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}

	if (polyline && tempPolyline)
	{
		delete polyline;
		polyline = nullptr;
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
			m_app->dispToConsole("An error occurred while computing radial distances!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return false;
	}
}

void qSRA::projectCloudDistsInGrid() const
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selectCount = selectedEntities.size();
	if (selectCount != 1 && selectCount != 2)
	{
		assert(false);
		return;
	}

	//retrieve input cloud and polyline
	ccPointCloud* cloud = nullptr;
	ccPolyline* polyline = nullptr;
	bool tempPolyline = false;
	{
		for (size_t i = 0; i < selectCount; ++i)
		{
			if (selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
			{
				cloud = static_cast<ccPointCloud*>(selectedEntities[i]);
			}
			else if (selectedEntities[i]->isA(CC_TYPES::POLY_LINE))
			{
				polyline = static_cast<ccPolyline*>(selectedEntities[i]);
			}
			else if (!polyline && selectedEntities[i]->isKindOf(CC_TYPES::CONE))
			{
				//special case: we can deduce the polyline from the cone/cylinder parameters
				ccCone* cone = static_cast<ccCone*>(selectedEntities[i]);
				polyline = GetConeProfile(cone);
				if (!polyline)
				{
					//the conversion failed?!
					return;
				}
				tempPolyline = true;
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
		polyline = nullptr;
	}
}

void qSRA::doProjectCloudDistsInGrid(ccPointCloud* cloud, ccPolyline* polyline) const
{
	assert(cloud && m_app);
	if (!cloud)
		return;

	//get the scalar field to map
	ccScalarField* sf = nullptr;
	{
		int sfIdx = cloud->getScalarFieldIndexByName(RADIAL_DIST_SF_NAME);
		if (sfIdx < 0)
		{
			sf = cloud->getCurrentDisplayedScalarField();
			if (sf)
			{
				if (QMessageBox::question(	m_app ? m_app->getMainWindow() : nullptr,
											"Distance field",
											QString("Cloud has no '%1' field. Do you want to use the active scalar field instead?").arg(RADIAL_DIST_SF_NAME),
											QMessageBox::Yes,
											QMessageBox::No) == QMessageBox::No )
				{
					//we can stop already
					return;
				}
			}
			else
			{
				QString message = QString("Cloud has no no '%1' field and no active scalar field!").arg(RADIAL_DIST_SF_NAME);
				ccLog::Error(message);
				
				//additional indications
				if (m_doCompareCloudToProfile)
				{
					ccLog::Warning(QString("You can compute the radial distances with the '%1' method").arg(m_doCompareCloudToProfile->text()));
				}
				return;
			}
		}
		else
		{
			sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
		}
	}
	assert(sf);

	DistanceMapGenerationDlg dmgDlg(cloud, sf, polyline, m_app);

	dmgDlg.exec();
}
