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

#include "qRANSAC_SD.h"

//PrimitiveShapes/MiscLib
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

//Dialog
#include "ccRansacSDDlg.h"

//Qt
#include <QtGui>
#include <QApplication>
#include <QtConcurrentRun>
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPlane.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <ccTorus.h>

//CCLib
#include <ScalarField.h>
#include <CCPlatform.h>

//System
#include <algorithm>
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <time.h>
#endif

qRansacSD::qRansacSD(QObject* parent/*=0*/)
	: QObject(parent)
	, m_action(0)
{
}

void qRansacSD::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size()==1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

void qRansacSD::getActions(QActionGroup& group)
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

//for parameters persistence
static unsigned s_supportPoints    = 500;	// this is the minimal numer of points required for a primitive
static double   s_maxNormalDev_deg = 25.0;	// maximal normal deviation from ideal shape (in degrees)
static double   s_proba            = 0.01;	// probability that no better candidate was overlooked during sampling
static bool s_primEnabled[5] = {true,true,true,false,false};

void qRansacSD::doAction()
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
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	//input cloud
	unsigned count = pc->size();
	bool hasNorms = pc->hasNormals();
	CCVector3 bbMin, bbMax;
	pc->getBoundingBox(bbMin,bbMax);
	const CCVector3d& globalShift = pc->getGlobalShift();
	double globalScale = pc->getGlobalScale();

	//Convert CC point cloud to RANSAC_SD type
	PointCloud cloud;
	{
		try
		{
			cloud.reserve(count);
		}
		catch(...)
		{
			m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		//default point & normal
		Point Pt;
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* P = pc->getPoint(i);
			Pt.pos[0] = static_cast<float>(P->x);
			Pt.pos[1] = static_cast<float>(P->y);
			Pt.pos[2] = static_cast<float>(P->z);
			if (hasNorms)
			{
				const CCVector3& N = pc->getPointNormal(i);
				Pt.normal[0] = static_cast<float>(N.x);
				Pt.normal[1] = static_cast<float>(N.y);
				Pt.normal[2] = static_cast<float>(N.z);
			}
			cloud.push_back(Pt);
		}
		
		//manually set bounding box!
		Vec3f cbbMin,cbbMax;
		cbbMin[0] = static_cast<float>(bbMin.x);
		cbbMin[1] = static_cast<float>(bbMin.y);
		cbbMin[2] = static_cast<float>(bbMin.z);
		cbbMax[0] = static_cast<float>(bbMax.x);
		cbbMax[1] = static_cast<float>(bbMax.y);
		cbbMax[2] = static_cast<float>(bbMax.z);
		cloud.setBBox(cbbMin,cbbMax);
	}

	//cloud scale (useful for setting several parameters
	const float scale = cloud.getScale();

	//init dialog with default values
	ccRansacSDDlg rsdDlg(m_app->getMainWindow());
	rsdDlg.epsilonDoubleSpinBox->setValue(.005f * scale);		// set distance threshold to 0.5% of bounding box width
	rsdDlg.bitmapEpsilonDoubleSpinBox->setValue(.01f * scale);	// set bitmap resolution (= sampling resolution) to 1% of bounding box width
	rsdDlg.supportPointsSpinBox->setValue(s_supportPoints);
	rsdDlg.maxNormDevAngleSpinBox->setValue(s_maxNormalDev_deg);
	rsdDlg.probaDoubleSpinBox->setValue(s_proba);
	rsdDlg.planeCheckBox->setChecked(s_primEnabled[0]);
	rsdDlg.sphereCheckBox->setChecked(s_primEnabled[1]);
	rsdDlg.cylinderCheckBox->setChecked(s_primEnabled[2]);
	rsdDlg.coneCheckBox->setChecked(s_primEnabled[3]);
	rsdDlg.torusCheckBox->setChecked(s_primEnabled[4]);

	if (!rsdDlg.exec())
		return;

	//for parameters persistence
	{
		s_supportPoints = rsdDlg.supportPointsSpinBox->value();
		s_maxNormalDev_deg = rsdDlg.maxNormDevAngleSpinBox->value();
		s_proba = rsdDlg.probaDoubleSpinBox->value();

		//consistency check
		{
			unsigned char primCount = 0;
			for (unsigned char k=0;k<5;++k)
				primCount += (unsigned)s_primEnabled[k];
			if (primCount==0)
			{
				m_app->dispToConsole("No primitive type selected!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}

		s_primEnabled[0] = rsdDlg.planeCheckBox->isChecked();
		s_primEnabled[1] = rsdDlg.sphereCheckBox->isChecked();
		s_primEnabled[2] = rsdDlg.cylinderCheckBox->isChecked();
		s_primEnabled[3] = rsdDlg.coneCheckBox->isChecked();
		s_primEnabled[4] = rsdDlg.torusCheckBox->isChecked();
	}

	//import parameters from dialog
	RansacShapeDetector::Options ransacOptions;
	{
		ransacOptions.m_epsilon			= static_cast<float>(rsdDlg.epsilonDoubleSpinBox->value() * 3.0); //internally this threshold is multiplied by 3!
		ransacOptions.m_bitmapEpsilon	= static_cast<float>(rsdDlg.bitmapEpsilonDoubleSpinBox->value());
		ransacOptions.m_normalThresh	= static_cast<float>(cos(rsdDlg.maxNormDevAngleSpinBox->value() * CC_DEG_TO_RAD));
		assert( ransacOptions.m_normalThresh >= 0 );
		ransacOptions.m_probability		= static_cast<float>(rsdDlg.probaDoubleSpinBox->value());
		ransacOptions.m_minSupport		= static_cast<unsigned>(rsdDlg.supportPointsSpinBox->value());
	}

	if (!hasNorms)
	{
		QProgressDialog pDlg("Computing normals (please wait)",QString(),0,0,m_app->getMainWindow());
		pDlg.setWindowTitle("Ransac Shape Detection");
		pDlg.show();
		QApplication::processEvents();

		cloud.calcNormals(.01f * scale);

		if (pc->reserveTheNormsTable())
		{
			for (unsigned i=0; i<count; ++i)
			{
				Vec3f& Nvi = cloud[i].normal;
				CCVector3 Ni = CCVector3::fromArray(Nvi);
				//normalize the vector in case of
				Ni.normalize();
				pc->addNorm(Ni);
			}
			pc->showNormals(true);
			
			//currently selected entities appearance may have changed!
			pc->prepareDisplayForRefresh_recursive();
		}
		else
		{
			m_app->dispToConsole("Not enough memory to compute normals!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}

	// set which primitives are to be detected by adding the respective constructors
	RansacShapeDetector detector(ransacOptions); // the detector object

	if (rsdDlg.planeCheckBox->isChecked())
		detector.Add(new PlanePrimitiveShapeConstructor());
	if (rsdDlg.sphereCheckBox->isChecked())
		detector.Add(new SpherePrimitiveShapeConstructor());
	if (rsdDlg.cylinderCheckBox->isChecked())
		detector.Add(new CylinderPrimitiveShapeConstructor());
	if (rsdDlg.coneCheckBox->isChecked())
		detector.Add(new ConePrimitiveShapeConstructor());
	if (rsdDlg.torusCheckBox->isChecked())
		detector.Add(new TorusPrimitiveShapeConstructor());

	unsigned remaining = count;
	typedef std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > DetectedShape;
	MiscLib::Vector< DetectedShape > shapes; // stores the detected shapes

	// run detection
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	{
		//progress dialog (Qtconcurrent::run can't be canceled!)
		QProgressDialog pDlg("Operation in progress (please wait)",QString(),0,0,m_app->getMainWindow());
		pDlg.setWindowTitle("Ransac Shape Detection");
		pDlg.show();
		QApplication::processEvents();

		//run in a separate thread
		s_detector = &detector;
		s_shapes = &shapes;
		s_cloud = &cloud;
		QFuture<void> future = QtConcurrent::run(doDetection);

		while (!future.isFinished())
		{
#if defined(CC_WINDOWS)
			::Sleep(500);
#else
			usleep(500 * 1000);
#endif
			pDlg.setValue(pDlg.value()+1);
			QApplication::processEvents();
		}

		remaining = static_cast<unsigned>(s_remainingPoints);

		pDlg.hide();
		QApplication::processEvents();
	}
	//else
	//{
	//	remaining = detector.Detect(cloud, 0, cloud.size(), &shapes);
	//}

#if 0 //def _DEBUG
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
		for (unsigned i=0;i<shapes.size();++i)
		{
			PrimitiveShape* shape = shapes[i].first;
			size_t shapePointsCount = shapes[i].second;

			std::string desc;
			shape->Description(&desc);
			fprintf(fp,"#%i - %s - %i points\n",i+1,desc.c_str(),shapePointsCount);
		}
	}
	fclose(fp);
#endif

	if (remaining == count)
	{
		m_app->dispToConsole("Segmentation failed...",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	if (shapes.size() > 0)
	{
		ccHObject* group = 0;
		for (MiscLib::Vector<DetectedShape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
		{
			const PrimitiveShape* shape = it->first;
			unsigned shapePointsCount = static_cast<unsigned>(it->second);

			//too many points?!
			if (shapePointsCount > count)
			{
				m_app->dispToConsole("Inconsistent result!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				break;
			}

			std::string desc;
			shape->Description(&desc);

			//new cloud for sub-part
			ccPointCloud* pcShape = new ccPointCloud(desc.c_str());

			//we fill cloud with sub-part points
			if (!pcShape->reserve((unsigned)shapePointsCount))
			{
				m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				delete pcShape;
				break;
			}			
			bool saveNormals = pcShape->reserveTheNormsTable();

			for (unsigned j=0; j<shapePointsCount; ++j)
			{
				pcShape->addPoint(CCVector3::fromArray(cloud[count-1-j].pos));
				if (saveNormals)
					pcShape->addNorm(CCVector3::fromArray(cloud[count-1-j].normal));
			}

			//random color
			ccColor::Rgb col = ccColor::Generator::Random();
			pcShape->setRGBColor(col);
			pcShape->showColors(true);
			pcShape->showNormals(saveNormals);
			pcShape->setVisible(true);
			pcShape->setGlobalShift(globalShift);
			pcShape->setGlobalScale(globalScale);

			//convert detected primitive into a CC primitive type
			ccGenericPrimitive* prim = 0;
			switch(shape->Identifier())
			{
			case 0: //plane
				{
				const PlanePrimitiveShape* plane = static_cast<const PlanePrimitiveShape*>(shape);
				Vec3f G = plane->Internal().getPosition();
				Vec3f N = plane->Internal().getNormal();
				Vec3f X = plane->getXDim();
				Vec3f Y = plane->getYDim();

				//we look for real plane extents
				float minX,maxX,minY,maxY;
				for (unsigned j=0; j<shapePointsCount; ++j)
				{
					std::pair<float,float> param;
					plane->Parameters(cloud[count-1-j].pos,&param);
					if (j != 0)
					{
						if (minX < param.first)
							minX = param.first;
						else if (maxX > param.first)
							maxX = param.first;
						if (minY < param.second)
							minY = param.second;
						else if (maxY > param.second)
							maxY = param.second;
					}
					else
					{
						minX = maxX = param.first;
						minY = maxY = param.second;
					}
				}

				//we recenter plane (as it is not always the case!)
				float dX = maxX-minX;
				float dY = maxY-minY;
				G += X * (minX+dX/2);
				G += Y * (minY+dY/2);

				//we build matrix from these vectors
				ccGLMatrix glMat(	CCVector3::fromArray(X.getValue()),
									CCVector3::fromArray(Y.getValue()),
									CCVector3::fromArray(N.getValue()),
									CCVector3::fromArray(G.getValue()) );

				//plane primitive
				prim = new ccPlane(dX,dY,&glMat);
			
				}
				break;

			case 1: //sphere
				{
				const SpherePrimitiveShape* sphere = static_cast<const SpherePrimitiveShape*>(shape);
				float radius = sphere->Internal().Radius();
				Vec3f CC = sphere->Internal().Center();

				pcShape->setName(QString("Sphere (r=%1)").arg(radius,0,'f'));

				//we build matrix from these vecctors
				ccGLMatrix glMat;
				glMat.setTranslation(CC.getValue());
				//sphere primitive
				prim = new ccSphere(radius,&glMat);
				prim->setEnabled(false);
			
				}
				break;

			case 2: //cylinder
				{
				const CylinderPrimitiveShape* cyl = static_cast<const CylinderPrimitiveShape*>(shape);
				Vec3f G = cyl->Internal().AxisPosition();
				Vec3f N = cyl->Internal().AxisDirection();
				Vec3f X = cyl->Internal().AngularDirection();
				Vec3f Y = N.cross(X);
				float r = cyl->Internal().Radius();
				float hMin = cyl->MinHeight();
				float hMax = cyl->MaxHeight();
				float h = hMax-hMin;
				G += N * (hMin+h/2);

				pcShape->setName(QString("Cylinder (r=%1/h=%2)").arg(r,0,'f').arg(h,0,'f'));

				//we build matrix from these vecctors
				ccGLMatrix glMat(	CCVector3::fromArray(X.getValue()),
									CCVector3::fromArray(Y.getValue()),
									CCVector3::fromArray(N.getValue()),
									CCVector3::fromArray(G.getValue()) );

				//cylinder primitive
				prim = new ccCylinder(r,h,&glMat);
				prim->setEnabled(false);

				}
				break;

			case 3: //cone
				{
				const ConePrimitiveShape* cone = static_cast<const ConePrimitiveShape*>(shape);
				Vec3f CC = cone->Internal().Center();
				Vec3f CA = cone->Internal().AxisDirection();
				float alpha = cone->Internal().Angle();

				//compute max height
				CCVector3 maxP = CCVector3::fromArray(CC.getValue());
				float maxHeight = 0;
				for (size_t j=0; j<shapePointsCount; ++j)
				{
					float h = cone->Internal().Height(cloud[count-1-j].pos);
					if (h > maxHeight)
					{
						maxHeight = h;
						maxP = CCVector3::fromArray(cloud[count-1-j].pos);
					}
				}

				pcShape->setName(QString("Cone (alpha=%1/h=%2)").arg(alpha,0,'f').arg(maxHeight,0,'f'));

				float radius = tan(alpha)*maxHeight;
				CCVector3 Z = CCVector3::fromArray(CA.getValue());
				CCVector3 C = CCVector3::fromArray(CC.getValue()); //cone apex

				//construct remaining of base
				Z.normalize();
				CCVector3 X = maxP - (C + maxHeight * Z);
				X.normalize();
				CCVector3 Y = Z * X;

				//we build matrix from these vecctors
				ccGLMatrix glMat(X,Y,Z,C+(maxHeight/2)*Z);

				//cone primitive
				prim = new ccCone(0,radius,maxHeight,0,0,&glMat);
				prim->setEnabled(false);

				}
				break;

			case 4: //torus
				{
				const TorusPrimitiveShape* torus = static_cast<const TorusPrimitiveShape*>(shape);
				if (torus->Internal().IsAppleShaped())
				{
					m_app->dispToConsole("[qRansacSD] Apple-shaped torus are not handled by CloudCompare!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}
				else
				{
					Vec3f CC = torus->Internal().Center();
					Vec3f CA = torus->Internal().AxisDirection();
					float minRadius = torus->Internal().MinorRadius();
					float maxRadius = torus->Internal().MajorRadius();

					pcShape->setName(QString("Torus (r=%1/R=%2)").arg(minRadius,0,'f').arg(maxRadius,0,'f'));

					CCVector3 Z = CCVector3::fromArray(CA.getValue());
					CCVector3 C = CCVector3::fromArray(CC.getValue());
					//construct remaining of base
					CCVector3 X = Z.orthogonal();
					CCVector3 Y = Z * X;

					//we build matrix from these vecctors
					ccGLMatrix glMat(X,Y,Z,C);

					//torus primitive
					prim = new ccTorus(maxRadius-minRadius,maxRadius+minRadius,M_PI*2.0,false,0,&glMat);
					prim->setEnabled(false);
				}

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

			if (!group)
				group = new ccHObject(QString("Ransac Detected Shapes (%1)").arg(ent->getName()));
			group->addChild(pcShape);

			count -= shapePointsCount;

			QApplication::processEvents();
		}

		if (group)
		{
			assert(group->getChildrenNumber() != 0);
			
			//we hide input cloud
			pc->setEnabled(false);
			m_app->dispToConsole("[qRansacSD] Input cloud has been automtically hidden!",ccMainAppInterface::WRN_CONSOLE_MESSAGE);

			//we add new group to DB/display
			group->setVisible(true);
			group->setDisplay_recursive(pc->getDisplay());
			m_app->addToDB(group);

			m_app->refreshAll();		
		}
	}
}

QIcon qRansacSD::getIcon() const
{
	return QIcon(QString::fromUtf8(":/CC/plugin/qRANSAC_SD/qRANSAC_SD.png"));
}

#ifndef CC_QT5
Q_EXPORT_PLUGIN2(qRansacSD,qRansacSD);
#endif
