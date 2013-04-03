//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################

#include "qHPR.h"
#include "ccHprDlg.h"

//Qt
#include <QtGui>
#include <QElapsedTimer>

//qCC_db
#include <ccPointCloud.h>
#include <ccOctree.h>
#include <ccProgressDialog.h>

//qCC
#include <ccGLWindow.h>

//CCLib
#include <CloudSamplingTools.h>
#include <ReferenceCloud.h>

//System
#include <string.h>

//Qhull
extern "C"
{
#include <qhull_a.h>
}

qHPR::qHPR(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qHPR::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

void qHPR::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1);
}

CCLib::ReferenceCloud* qHPR::removeHiddenPoints(CCLib::GenericIndexedCloudPersist* theCloud, const float viewPoint[], float fParam)
{
	assert(theCloud);

	unsigned i,nbPoints = theCloud->size();

	if (nbPoints==0)
		return 0;

	CCLib::ReferenceCloud* newCloud = new CCLib::ReferenceCloud(theCloud);

	//less than 4 points? no need for calculation, we return the whole cloud
	if (nbPoints<4)
	{
		if (!newCloud->addPointIndex(0,nbPoints)) //well, we never know ;)
		{
			//not enough memory!
			delete newCloud;
			return 0;
		}
		return newCloud;
	}

	//view point
	coordT Cx = viewPoint[0];
	coordT Cy = viewPoint[1];
	coordT Cz = viewPoint[2];

	float* radius = new float[nbPoints];
	if (!radius)
	{
		//not enough memory!
		delete newCloud;
		return 0;
	}
	float r,maxRadius = 0.0;

	//table of points
	coordT* pt_array = new coordT[(nbPoints+1)*3];
	coordT* _pt_array = pt_array;
	theCloud->placeIteratorAtBegining();

//#define BACKUP_PROJECTED_CLOUDS
#ifdef BACKUP_PROJECTED_CLOUDS
	FILE* fp = fopen("output_centered.asc","wt");
#endif
	double x,y,z;
	for (i=0;i<nbPoints;++i)
	{
		const CCVector3* P = theCloud->getNextPoint();
		*(_pt_array++)=x=coordT(P->x)-Cx;
		*(_pt_array++)=y=coordT(P->y)-Cy;
		*(_pt_array++)=z=coordT(P->z)-Cz;
		//we pre-compute the radius ...
		r = (float)sqrt(x*x+y*y+z*z);
		//in order to determine the max radius
		if (maxRadius<r)
			maxRadius = r;
		radius[i] = r;
#ifdef BACKUP_PROJECTED_CLOUDS
		fprintf(fp,"%f %f %f %f\n",x,y,z,r);
#endif
	}
	//we add the view point (Cf. HPR)
	*(_pt_array++)=0.0;
	*(_pt_array++)=0.0;
	*(_pt_array++)=0.0;
#ifdef BACKUP_PROJECTED_CLOUDS
	fprintf(fp,"%f %f %f %f\n",0,0,0,0);
	fclose(fp);
#endif

	maxRadius *= 2.0f*pow(10.0f,fParam);

	_pt_array = pt_array;
#ifdef BACKUP_PROJECTED_CLOUDS
	fp = fopen("output_transformed.asc","wt");
#endif
	for (i=0;i<nbPoints;++i)
	{
		//Spherical flipping
		r = maxRadius/radius[i]-1.0f;
#ifndef BACKUP_PROJECTED_CLOUDS
		*(_pt_array++) *= double(r);
		*(_pt_array++) *= double(r);
		*(_pt_array++) *= double(r);
#else
		x = *_pt_array * double(r);
		*(_pt_array++) = x;
		y = *_pt_array * double(r);
		*(_pt_array++) = y;
		z = *_pt_array * double(r);
		*(_pt_array++) = z;
		fprintf(fp,"%f %f %f %f\n",x,y,z,r);
#endif
	}
#ifdef BACKUP_PROJECTED_CLOUDS
	fclose(fp);
#endif

	//we re-use the radius to record if each point belongs to the convex hull
	//delete[] radius;
	//uchar* pointBelongsToCvxHull = new uchar[nbPoints];
	uchar* pointBelongsToCvxHull = (uchar*)radius;
	memset(pointBelongsToCvxHull,0,sizeof(uchar)*(nbPoints+1));

	if (!qh_new_qhull(3,nbPoints+1,pt_array,False,(char*)"qhull QJ s Qci Tcv",0,stderr))
	{
		vertexT *vertex,**vertexp;
		facetT *facet;
		setT *vertices;

		int j, i = 0;
		FORALLfacets
		{
			/*if (!facet->simplicial)
				error("convhulln: non-simplicial facet"); // should never happen with QJ
				*/

			j = 0;
			vertices = qh_facet3vertex (facet);
			FOREACHvertex_(vertices)
			{
				pointBelongsToCvxHull[qh_pointid(vertex->point)]=1;
				++j;
			}
			qh_settempfree(&vertices);

			if (j < 3)
				printf("Warning, facet %d only has %d vertices\n",i,j); // likewise but less fatal

			i++;
		}
	}

	delete[] pt_array;
	pt_array=0;

	qh_freeqhull(!qh_ALL);
	//free long memory
	int curlong, totlong;
	qh_memfreeshort (&curlong, &totlong);
	//free short memory and memory allocator

	//if (curlong || totlong)
	//{
	//	printf("convhulln: did not free %d bytes of long memory (%d pieces)",totlong, curlong);
	//}

	unsigned cvxHullSize = 0;
	for (i=0;i<nbPoints;++i)
		if (pointBelongsToCvxHull[i]>0)
			++cvxHullSize;

	if (cvxHullSize!=0 && newCloud->reserve(cvxHullSize))
	{
		for (i=0;i<nbPoints;++i)
			if (pointBelongsToCvxHull[i]>0)
				newCloud->addPointIndex(i); //can't fail, see above
	}
	else //not enough memory
	{
		delete newCloud;
		newCloud=0;
	}

	if (radius)
		delete[] radius;
	//delete[] pointBelongsToCvxHull;

	return newCloud;
}

void qHPR::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
    if (selNum!=1)
	{
		m_app->dispToConsole("Select only one cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

    ccHObject* ent = selectedEntities[0];
    if (!ent->isA(CC_POINT_CLOUD))
	{
		m_app->dispToConsole("Select a point cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
    ccPointCloud* cloud = static_cast<ccPointCloud*>(ent);

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
	assert(octreeLevel>=0 && octreeLevel<=255);

	//compute octree if cloud hasn't any
    ccOctree* theOctree = cloud->getOctree();
    if (!theOctree)
        theOctree = cloud->computeOctree(&progressCb);

	if (!theOctree)
	{
		m_app->dispToConsole("Couldn't compute octree!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        return;
	}

	CCVector3 viewPoint = params.cameraCenter;
	if (params.objectCenteredView)
	{
		CCVector3 PC = params.cameraCenter - params.pivotPoint;
		params.viewMat.inverse().apply(PC);
		viewPoint = params.pivotPoint + PC;
	}

    //HPR
	CCLib::ReferenceCloud* visibleCells = 0;
	{
		QElapsedTimer eTimer;
		eTimer.start();

		CCLib::ReferenceCloud* theCellCenters = CCLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(cloud,(uchar)octreeLevel,CCLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,&progressCb,theOctree);
		if (!theCellCenters)
		{
			m_app->dispToConsole("Error while simplifying point cloud with octree!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		visibleCells = removeHiddenPoints(theCellCenters,viewPoint.u,3.5);
	
		m_app->dispToConsole(QString("[HPR] Cells: %1 - Time: %2 s").arg(theCellCenters->size()).arg(eTimer.elapsed()/1.0e3));

		delete theCellCenters;
		theCellCenters = 0;
	}

    if (visibleCells)
    {
		if (!cloud->isVisibilityTableInstantiated() && !cloud->razVisibilityArray())
		{
			m_app->dispToConsole("Visibility array allocation failed! (Not enough memory?)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		ccPointCloud::VisibilityTableType* pointsVisibility = cloud->getTheVisibilityArray();

		assert(pointsVisibility);
		pointsVisibility->fill(POINT_HIDDEN);

        unsigned visiblePointCount = 0;
		unsigned visibleCellsCount = visibleCells->size();

		CCLib::DgmOctree::cellIndexesContainer cellIndexes;
		if (!theOctree->getCellIndexes((uchar)octreeLevel,cellIndexes))
		{
			m_app->dispToConsole("Couldn't fetch the list of octree cell indexes! (Not enough memory?)",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		CCLib::ReferenceCloud Yk(theOctree->associatedCloud());

        for (unsigned i=0; i<visibleCellsCount; ++i)
        {
			//cell index
            unsigned index = visibleCells->getPointGlobalIndex(i);
            
			//points in this cell...
			theOctree->getPointsInCellByCellIndex(&Yk,cellIndexes[index],(uchar)octreeLevel);
			//...are all visible
			unsigned count = Yk.size();
            for (unsigned j=0;j<count;++j)
                pointsVisibility->setValue(Yk.getPointGlobalIndex(j),POINT_VISIBLE);

            visiblePointCount += count;
        }

		m_app->dispToConsole(QString("[HPR] Visible points: %1").arg(visiblePointCount));
        cloud->redrawDisplay();

        delete visibleCells;
		visibleCells=0;
    }

    //currently selected entities appearance may have changed!
	m_app->refreshAll();
}

QIcon qHPR::getIcon() const
{
    return QIcon(QString::fromUtf8(":/CC/plugin/qHPR/cc_hpr.png"));
}

Q_EXPORT_PLUGIN2(qHPR,qHPR);
