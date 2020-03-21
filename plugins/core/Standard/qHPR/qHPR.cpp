//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "qHPR.h"
#include "ccHprDlg.h"

//Qt
#include <QtGui>
#include <QMainWindow>

//qCC_db
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccProgressDialog.h>
#include <cc2DViewportObject.h>

//qCC
#include <ccGLWindow.h>

//CCLib
#include <CloudSamplingTools.h>

//Qhull
extern "C"
{
#include <qhull_a.h>
}

qHPR::qHPR(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qHPR/info.json" )
	, m_action(nullptr)
{
}

QList<QAction *> qHPR::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, &QAction::triggered, this, &qHPR::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qHPR::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		//a single point cloud must be selected
		m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD));
	}
}

CCLib::ReferenceCloud* qHPR::removeHiddenPoints(CCLib::GenericIndexedCloudPersist* theCloud, const CCVector3d& viewPoint, double fParam)
{
	assert(theCloud);

	unsigned nbPoints = theCloud->size();
	if (nbPoints == 0)
		return nullptr;

	//less than 4 points? no need for calculation, we return the whole cloud
	if (nbPoints < 4)
	{
		CCLib::ReferenceCloud* visiblePoints = new CCLib::ReferenceCloud(theCloud);
		if (!visiblePoints->addPointIndex(0,nbPoints)) //well even for less than 4 points we never know ;)
		{
			//not enough memory!
			delete visiblePoints;
			visiblePoints = nullptr;
		}
		return visiblePoints;
	}

	double maxRadius = 0;

	//convert point cloud to an array of double triplets (for qHull)
	coordT* pt_array = new coordT[(nbPoints+1)*3];
	{
		coordT* _pt_array = pt_array;

		for (unsigned i=0; i<nbPoints; ++i)
		{
			CCVector3d P = CCVector3d::fromArray(theCloud->getPoint(i)->u) - viewPoint;
			*_pt_array++ = static_cast<coordT>(P.x);
			*_pt_array++ = static_cast<coordT>(P.y);
			*_pt_array++ = static_cast<coordT>(P.z);

			//we keep track of the highest 'radius'
			double r2 = P.norm2();
			if (maxRadius < r2)
				maxRadius = r2;
		}
		
		//we add the view point (Cf. HPR)
		*_pt_array++ = 0;
		*_pt_array++ = 0;
		*_pt_array++ = 0;

		maxRadius = sqrt(maxRadius);
	}

	//apply spherical flipping
	{
		maxRadius *= pow(10.0,fParam) * 2;
	
		coordT* _pt_array = pt_array;
		for (unsigned i=0; i<nbPoints; ++i)
		{
			CCVector3d P = CCVector3d::fromArray(theCloud->getPoint(i)->u) - viewPoint;

			double r = (maxRadius/P.norm()) - 1.0;
			*_pt_array++ *= r;
			*_pt_array++ *= r;
			*_pt_array++ *= r;
		}
	}

	//array to flag points on the convex hull
	std::vector<bool> pointBelongsToCvxHull;

	static char qHullCommand[] = "qhull QJ Qci";
	if (!qh_new_qhull(3,nbPoints+1,pt_array,False,qHullCommand,nullptr,stderr))
	{
		try
		{
			pointBelongsToCvxHull.resize(nbPoints+1,false);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory!
			delete[] pt_array;
			return nullptr;
		}

		vertexT *vertex = nullptr;
		vertexT **vertexp = nullptr;
		facetT *facet = nullptr;

		FORALLfacets
		{
			//if (!facet->simplicial)
			//	error("convhulln: non-simplicial facet"); // should never happen with QJ

			setT* vertices = qh_facet3vertex(facet);
			FOREACHvertex_(vertices)
			{
				pointBelongsToCvxHull[qh_pointid(vertex->point)] = true;
			}
			qh_settempfree(&vertices);
		}
	}

	delete[] pt_array;
	pt_array = nullptr;

	qh_freeqhull(!qh_ALL);
	//free long memory
	int curlong = 0;
	int totlong = 0;
	qh_memfreeshort (&curlong, &totlong);
	//free short memory and memory allocator

	if (!pointBelongsToCvxHull.empty())
	{
		//compute the number of points belonging to the convex hull
		unsigned cvxHullSize = 0;
		{
			for (unsigned i=0; i<nbPoints; ++i)
				if (pointBelongsToCvxHull[i])
					++cvxHullSize;
		}

		CCLib::ReferenceCloud* visiblePoints = new CCLib::ReferenceCloud(theCloud);
		if (cvxHullSize!=0 && visiblePoints->reserve(cvxHullSize))
		{
			for (unsigned i=0; i<nbPoints; ++i)
				if (pointBelongsToCvxHull[i])
					visiblePoints->addPointIndex(i); //can't fail, see above

			return visiblePoints;

		}
		else //not enough memory
		{
			delete visiblePoints;
			visiblePoints = nullptr;
		}
	}

	return nullptr;
}

void qHPR::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	if ( !m_app->haveOneSelection() || !selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* cloud = static_cast<ccPointCloud*>(selectedEntities[0]);

	ccGLWindow* win = m_app->getActiveGLWindow();
	if (!win)
	{
		m_app->dispToConsole("No active window!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//display parameters
	const ccViewportParameters& params =  win->getViewportParameters();
	if (!params.perspectiveView)
	{
		m_app->dispToConsole("Perspective mode only!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHprDlg dlg(m_app->getMainWindow());
	if (!dlg.exec())
		return;

	//progress dialog
	ccProgressDialog progressCb(false,m_app->getMainWindow());

	//unique parameter: the octree subdivision level
	int octreeLevel = dlg.octreeLevelSpinBox->value();
	assert(octreeLevel >= 0 && octreeLevel <= CCLib::DgmOctree::MAX_OCTREE_LEVEL);

	//compute octree if cloud hasn't any
	ccOctree::Shared theOctree = cloud->getOctree();
	if (!theOctree)
	{
		theOctree = cloud->computeOctree(&progressCb);
		if (theOctree && cloud->getParent())
		{
			m_app->addToDB(cloud->getOctreeProxy());
		}
	}

	if (!theOctree)
	{
		m_app->dispToConsole("Couldn't compute octree!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	CCVector3d viewPoint = params.cameraCenter;
	if (params.objectCenteredView)
	{
		CCVector3d PC = params.cameraCenter - params.pivotPoint;
		params.viewMat.inverse().apply(PC);
		viewPoint = params.pivotPoint + PC;
	}

	//HPR
	CCLib::ReferenceCloud* visibleCells = nullptr;
	{
		QElapsedTimer eTimer;
		eTimer.start();

		CCLib::ReferenceCloud* theCellCenters = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(	cloud,
																											static_cast<unsigned char>(octreeLevel),
																											CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
																											&progressCb,
																											theOctree.data());
		if (!theCellCenters)
		{
			m_app->dispToConsole("Error while simplifying point cloud with octree!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		visibleCells = removeHiddenPoints(theCellCenters,viewPoint,3.5);
	
		m_app->dispToConsole(QString("[HPR] Cells: %1 - Time: %2 s").arg(theCellCenters->size()).arg(eTimer.elapsed()/1.0e3));

		//warning: after this, visibleCells can't be used anymore as a
		//normal cloud (as it's 'associated cloud' has been deleted).
		//Only its indexes are valid! (they are corresponding to octree cells)
		delete theCellCenters;
		theCellCenters = nullptr;
	}

	if (visibleCells)
	{
		//DGM: we generate a new cloud now, instead of playing with the points visiblity! (too confusing for the user)
		/*if (!cloud->isVisibilityTableInstantiated() && !cloud->resetVisibilityArray())
		{
			m_app->dispToConsole("Visibility array allocation failed! (Not enough memory?)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		ccPointCloud::VisibilityTableType* pointsVisibility = cloud->getTheVisibilityArray();
		assert(pointsVisibility);
		pointsVisibility->fill(POINT_HIDDEN);
		*/

		CCLib::ReferenceCloud visiblePoints(theOctree->associatedCloud());

		unsigned visiblePointCount = 0;
		unsigned visibleCellsCount = visibleCells->size();

		CCLib::DgmOctree::cellIndexesContainer cellIndexes;
		if (!theOctree->getCellIndexes(static_cast<unsigned char>(octreeLevel), cellIndexes))
		{
			m_app->dispToConsole("Couldn't fetch the list of octree cell indexes! (Not enough memory?)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			delete visibleCells;
			return;
		}

		for (unsigned i=0; i<visibleCellsCount; ++i)
		{
			//cell index
			unsigned index = visibleCells->getPointGlobalIndex(i);

			//points in this cell...
			CCLib::ReferenceCloud Yk(theOctree->associatedCloud());
			theOctree->getPointsInCellByCellIndex(&Yk,cellIndexes[index],static_cast<unsigned char>(octreeLevel));
			//...are all visible
			/*unsigned count = Yk.size();
			for (unsigned j=0;j<count;++j)
				pointsVisibility->setValue(Yk.getPointGlobalIndex(j),POINT_VISIBLE);
			visiblePointCount += count;
			*/
			if (!visiblePoints.add(Yk))
			{
				m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				delete visibleCells;
				return;
			}
		}

		delete visibleCells;
		visibleCells = nullptr;

		m_app->dispToConsole(QString("[HPR] Visible points: %1").arg(visiblePointCount));

		if (visiblePoints.size() == cloud->size())
		{
			m_app->dispToConsole("No points were removed!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		}
		else
		{
			//create cloud from visibility selection
			ccPointCloud* newCloud = cloud->partialClone(&visiblePoints);
			if (newCloud)
			{			
				newCloud->setDisplay(newCloud->getDisplay());
				newCloud->setVisible(true);
				newCloud->setName(cloud->getName()+QString(".visible_points"));
				cloud->setEnabled(false);

				//add associated viewport object
				cc2DViewportObject* viewportObject = new cc2DViewportObject(QString("Viewport"));
				viewportObject->setParameters(params);
				newCloud->addChild(viewportObject);

				m_app->addToDB(newCloud);
				newCloud->redrawDisplay();
			}
			else
			{
				m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
		}
	}

	//currently selected entities appearance may have changed!
	m_app->refreshAll();
}
