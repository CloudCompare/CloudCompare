//##########################################################################
//#                                                                        #
//#                 CLOUDCOMPARE PLUGIN: qRANSAC_SD                        #
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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 1652                                                              $
//$LastChangedDate:: 2010-08-26 23:28:59 +0200 (jeu., 26 août 2010)       $
//**************************************************************************
//

#include "qRANSAC_SD.h"

//PrimitiveShapes/MiscLib
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <SpherePrimitiveShape.h>

//Dialog
#include "ccRansacSDDlg.h"

//Qt
#include <QtGui>
#include <QApplication>
#include <qtconcurrentrun.h>

//qCC_db
#include <ccProgressDialog.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPlane.h>
#include <ccSphere.h>
#include <ccCylinder.h>

//CCLib
#include <ScalarField.h>

void qRansacSDPlugin::getDescription(ccPluginDescription& desc)
{
    strcpy(desc.name,"Efficient RANSAC for Point-Cloud Shape Detection (Schnabel et al 2007)");
    strcpy(desc.menuName,"RansacSD");
    desc.hasAnIcon=true;
    desc.version=1;
}

bool qRansacSDPlugin::onNewSelection(const ccHObject::Container& selectedEntities)
{
    return (selectedEntities.size()==1 && selectedEntities[0]->isA(CC_POINT_CLOUD));
}

static MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >* s_shapes; // stores the detected shapes
static size_t s_remainingPoints = 0;
static RansacShapeDetector* s_detector = 0;
static PointCloud* s_cloud = 0;
void doDetection()
{
	if (!s_detector || !s_cloud || !s_shapes)
		return;

	s_remainingPoints = s_detector->Detect(*s_cloud, 0, s_cloud->size(), s_shapes);
}

int qRansacSDPlugin::doAction(ccHObject::Container& selectedEntities,
                        unsigned& uiModificationFlags,
                        ccProgressDialog* progressCb/*=NULL*/,
                        QWidget* parent/*=NULL*/)
{
    unsigned selNum = selectedEntities.size();
    if (selNum!=1)
        return -1;

    ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_POINT_CLOUD))
		return -2;

    ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	//point count inside input cloud
	unsigned i,j,count = pc->size();

	PointCloud cloud;
    cloud.reserve(count);

    //Convert CC point cloud to custom db
	bool hasNorms = pc->hasNormals();
	//default point & normal
    Point Pt;
	Pt.normal[0] = 0.0;
	Pt.normal[1] = 0.0;
	Pt.normal[2] = 0.0;
    for (i=0;i<count;++i)
    {
        const CCVector3* P = pc->getPoint(i);
        Pt.pos[0] = P->x;
        Pt.pos[1] = P->y;
        Pt.pos[2] = P->z;
		if (hasNorms)
		{
			const PointCoordinateType* N = pc->getPointNormal(i);
			Pt.normal[0] = N[0];
			Pt.normal[1] = N[1];
			Pt.normal[2] = N[2];
		}
        cloud.push_back(Pt);
    }

    //bounding box
    PointCoordinateType bbMin[3],bbMax[3];
    pc->getBoundingBox(bbMin,bbMax);
    Vec3f cbbMin,cbbMax;
    cbbMin[0] = bbMin[0];
    cbbMin[1] = bbMin[1];
    cbbMin[2] = bbMin[2];
    cbbMax[0] = bbMax[0];
    cbbMax[1] = bbMax[1];
    cbbMax[2] = bbMax[2];
    cloud.setBBox(cbbMin,cbbMax);

    //DO THE JOB HERE
	RansacShapeDetector::Options ransacOptions;
    ransacOptions.m_epsilon = .01f * cloud.getScale(); // set distance threshold to .01f of bounding box width
        // NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
    ransacOptions.m_bitmapEpsilon = .02f * cloud.getScale(); // set bitmap resolution to .02f of bounding box width
        // NOTE: This threshold is NOT multiplied internally!
    //ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
    //ransacOptions.m_minSupport = 500; // this is the minimal numer of points required for a primitive
    //ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	ccRansacSDDlg rsdDlg(parent);
	rsdDlg.epsilonDoubleSpinBox->setValue(ransacOptions.m_epsilon);
	rsdDlg.bitmapEpsilonDoubleSpinBox->setValue(ransacOptions.m_bitmapEpsilon);

	if (!rsdDlg.exec())
		return 0;

	//import parameters from dialog
	{
		ransacOptions.m_epsilon = rsdDlg.epsilonDoubleSpinBox->value();
		ransacOptions.m_bitmapEpsilon = rsdDlg.bitmapEpsilonDoubleSpinBox->value();
		ransacOptions.m_normalThresh = rsdDlg.normThreshDoubleSpinBox->value();
		ransacOptions.m_minSupport = rsdDlg.supportPointsSpinBox->value();
		ransacOptions.m_probability = rsdDlg.probaDoubleSpinBox->value();
	}

	if (progressCb)
	{
		progressCb->setCancelButton(0);
		progressCb->setRange(0,0);
	}

	if (!hasNorms)
	{
		if (progressCb)
		{
			progressCb->setInfo("Computing normals (please wait)");
			progressCb->start();
			QApplication::processEvents();
		}
		cloud.calcNormals(.01f * cloud.getScale());

		if (pc->reserveTheNormsTable())
		{
			for (i=0;i<count;++i)
			{
				CCVector3 N(cloud[i].normal);
				N.normalize();
				pc->addNorm(N.u);
			}
			pc->showNormals(true);
			
			//currently selected entities appearance may have changed!
			uiModificationFlags |= CC_PLUGIN_REFRESH_GL_WINDOWS;
		}
		else
		{
			return -4;
		}
	}

    // set which primitives are to be detected by adding the respective constructors
    RansacShapeDetector detector(ransacOptions); // the detector object

    detector.Add(new PlanePrimitiveShapeConstructor());
    detector.Add(new SpherePrimitiveShapeConstructor());
    detector.Add(new CylinderPrimitiveShapeConstructor());
    detector.Add(new ConePrimitiveShapeConstructor());
    detector.Add(new TorusPrimitiveShapeConstructor());

	size_t remaining = count;
	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes

    // run detection
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	if (progressCb)
	{
		progressCb->setInfo("Operation in progress");
		progressCb->setMethodTitle("Ransac Shape Detection");
		progressCb->start();

		//run in a separate thread
		s_detector = &detector;
		s_shapes = &shapes;
		s_cloud = &cloud;
		QFuture<void> future = QtConcurrent::run(doDetection);

		unsigned progress = 0;
		while (!future.isFinished())
		{
#if defined(_WIN32) || defined(WIN32)
			::Sleep(500);
#else
			sleep(500);
#endif
			progressCb->update(++progress);
			//Qtconcurrent::run can't be canceled!
			/*if (progressCb->isCancelRequested())
			{
				future.cancel();
				future.waitForFinished();
				s_remainingPoints = count;
				break;
			}
			//*/
		}

		remaining = s_remainingPoints;

		progressCb->stop();
		QApplication::processEvents();
	}
	else
	{
		remaining = detector.Detect(cloud, 0, cloud.size(), &shapes);
	}

#ifdef _DEBUG
	FILE* fp = fopen("RANS_SD_trace.txt","wt");

    fprintf(fp,"[Options]\n");
    fprintf(fp,"epsilon=%f\n",ransacOptions.m_epsilon);
    fprintf(fp,"bitmap epsilon=%f\n",ransacOptions.m_bitmapEpsilon);
    fprintf(fp,"normal thresh=%f\n",ransacOptions.m_normalThresh);
    fprintf(fp,"min support=%i\n",ransacOptions.m_minSupport);
    fprintf(fp,"probability=%f\n",ransacOptions.m_probability);

    fprintf(fp,"\n[Statistics]\n");
	fprintf(fp,"input points=%i\n",count);
	fprintf(fp,"segmented=%i\n",count-remaining);
	fprintf(fp,"remaining=%i\n",remaining);

    if (shapes.size()>0)
    {
        fprintf(fp,"\n[Shapes]\n");
        for (i=0;i<shapes.size();++i)
        {
            PrimitiveShape* shape = shapes[i].first;
            unsigned shapePointsCount = shapes[i].second;

            std::string desc;
            shape->Description(&desc);
            fprintf(fp,"#%i - %s - %i points\n",i+1,desc.c_str(),shapePointsCount);

    //        switch(shape->Identifier())
    //        {
    //            case 0:
    //                //plane (red)
    //                break;
    //
    //        }
        }
    }
	fclose(fp);
#endif

	if (remaining == count)
        return -5;

	if (shapes.size() != 0)
	{
		unsigned shiftCount=0;
		ccHObject* group = new ccHObject(QString("Ransac Detected Shapes (%1)").arg(ent->getName()));
		for (i=0;i<shapes.size();++i)
		{
			PrimitiveShape* shape = shapes[i].first;
			unsigned shapePointsCount = shapes[i].second;

			//too many points?!
			if (shapePointsCount > count)
				return -3;

			std::string desc;
			shape->Description(&desc);

			//new cloud for sub-part
			ccPointCloud* pcShape = new ccPointCloud(desc.c_str());

			//we fill cloud with sub-part points
			pcShape->reserve(shapePointsCount);
			pcShape->reserveTheNormsTable();
			for (j=0;j<shapePointsCount;++j)
			{
				CCVector3 P(cloud[count-1-j].pos);
				pcShape->addPoint(P);
				pcShape->addNorm(cloud[count-1-j].normal);

			}

			//random color
			unsigned char col[3];
			for (j=0;j<2;++j)
				col[j]=(unsigned char)(255.0*(float)rand()/(float)RAND_MAX);
			col[2]=255-(col[1]+col[2])/2;
			pcShape->setRGBColor(col);
			pcShape->showColors(true);
			pcShape->showNormals(true);

			group->addChild(pcShape);
			pcShape->setVisible(true);

			ccGenericPrimitive* prim = 0;

			switch(shape->Identifier())
			{
			case 0: //plane
				{
				PlanePrimitiveShape* plane = static_cast<PlanePrimitiveShape*>(shape);
				Vec3f G = plane->Internal().getPosition();
				Vec3f N = plane->Internal().getNormal();
				Vec3f X = plane->getXDim();
				Vec3f Y = plane->getYDim();

				//we look for real plane extents
				std::pair<float,float> param;
				float minX,maxX,minY,maxY;
				for (j=0;j<shapePointsCount;++j)
				{
					plane->Parameters(cloud[count-1-j].pos,&param);
					if (j!=0)
					{
						if (minX<param.first)
							minX=param.first;
						else if (maxX>param.first)
							maxX=param.first;
						if (minY<param.second)
							minY=param.second;
						else if (maxY>param.second)
							maxY=param.second;
					}
					else
					{
						minX=maxX=param.first;
						minY=maxY=param.second;
					}
				}

				//we recenter plane (as it is not always the case!)
				float dX = maxX-minX;
				float dY = maxY-minY;
				G += X * (minX+dX*0.5);
				G += Y * (minY+dY*0.5);

				//we build matrix from these vecctors
				ccGLMatrix glMat(CCVector3(X.getValue()),
								CCVector3(Y.getValue()),
								CCVector3(N.getValue()),
								CCVector3(G.getValue()));

				//plane primitive
				prim = new ccPlane(dX,dY,&glMat);
			
				}
				break;

			case 1: //sphere
				{
				SpherePrimitiveShape* sphere = static_cast<SpherePrimitiveShape*>(shape);
				float radius = sphere->Internal().Radius();
				Vec3f C = sphere->Internal().Center();

				pcShape->setName(QString("Sphere (r=%1)").arg(radius,0,'f'));

				//we build matrix from these vecctors
				ccGLMatrix glMat;
				glMat.setTranslation(CCVector3(C.getValue()));
				//sphere primitive
				prim = new ccSphere(radius,&glMat);
				//prim->showWired(true);
				prim->setEnabled(false);
			
				}
				break;

			case 2: //cylinder
				{
				CylinderPrimitiveShape* cyl = static_cast<CylinderPrimitiveShape*>(shape);
				Vec3f G = cyl->Internal().AxisPosition();
				Vec3f N = cyl->Internal().AxisDirection();
				Vec3f X = cyl->Internal().AngularDirection();
				Vec3f Y = N.cross(X);
				float r = cyl->Internal().Radius();
				float hMin = cyl->MinHeight();
				float hMax = cyl->MaxHeight();
				float h = hMax-hMin;
				G += N * (hMin+h*0.5);

				pcShape->setName(QString("Cylinder (r=%1/h=%2)").arg(r,0,'f').arg(h,0,'f'));

				//we look for real cylinder extents
				/*std::pair<float,float> param;
				float minAlpha,maxAlpha;
				for (j=0;j<shapePointsCount;++j)
				{
					plane->Parameters(cloud[count-1-j].pos,&param);
					if (j!=0)
					{
						if (minAlpha<param.second)
							minAlpha=param.second;
						else if (maxAlpha>param.second)
							maxAlpha=param.second;
					}
					else
					{
						minAlpha=maxAlpha=param.second;
					}
				}
				//*/

				//Vec3f X = plane->getXDim();
				//Vec3f Y = plane->getYDim();
				//we build matrix from these vecctors
				ccGLMatrix glMat(CCVector3(X.getValue()),
								CCVector3(Y.getValue()),
								CCVector3(N.getValue()),
								CCVector3(G.getValue()));

				//cylinder primitive
				prim = new ccCylinder(r,h,&glMat);
				//prim->showWired(true);
				prim->setEnabled(false);

				}
				break;

			}

			//is there a primitive to add to part cloud?
			if (prim)
			{
				prim->applyGLTransformation_recursive();
				pcShape->addChild(prim);
				prim->setDisplay(pcShape->getDisplay());
				prim->setColor(col);
				prim->showColors(true);
				prim->setVisible(true);
			}

			count -= shapePointsCount;

			QApplication::processEvents();
		}

		group->setDisplay_recursive(pc->getDisplay());
		group->prepareDisplayForRefresh_recursive();
		group->setVisible(true);
		selectedEntities.push_back(group);
		
		pc->setEnabled(false);
	}

    //currently selected entities parameters may have changed!
    uiModificationFlags |= CC_PLUGIN_REFRESH_ENTITY_BROWSER;
	//currently selected entities appearance may have changed!
	uiModificationFlags |= CC_PLUGIN_REFRESH_GL_WINDOWS;
    //new items have appeared below currently selected entity
	uiModificationFlags |= CC_PLUGIN_EXPAND_DB_TREE;

    return 1;
}

QString qRansacSDPlugin::getErrorMessage(int errorCode/*, LANGUAGE lang*/)
{
    QString errorMsg;
    switch(errorCode)
    {
        case -1:
            errorMsg=QString("Select only one cloud!");
            break;
        case -2:
            errorMsg=QString("Select a real point cloud!");
            break;
        case -3:
            errorMsg=QString("Inconsistent result!");
            break;
        case -4:
            errorMsg=QString("Not enough memory to compute normals!");
            break;
        case -5:
            errorMsg=QString("Segmentation failed...");
            break;
        default:
            errorMsg=QString("Undefined error!");
            break;
    }
    return errorMsg;
}

QIcon qRansacSDPlugin::getIcon() const
{
    return QIcon(QString::fromUtf8(":/CC/plugin/qRANSAC_SD/qRANSAC_SD.png"));
}

Q_EXPORT_PLUGIN2(qRansacSDPlugin,qRansacSDPlugin);
