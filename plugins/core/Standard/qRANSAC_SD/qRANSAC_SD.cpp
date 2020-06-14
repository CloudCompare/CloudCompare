//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qRANSAC_SD                     #
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

#include "qRANSAC_SD.h"
#include "qRANSAC_SD_Commands.h"

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
#include "windows.h"
#else
#include <time.h>
#endif

static ccMainAppInterface* s_app = nullptr;

qRansacSD::qRansacSD(QObject* parent/*=nullptr*/)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qRANSAC_SD/info.json")
	, m_action(nullptr)
{
	s_app = m_app;
}

void qRansacSD::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

QList<QAction*> qRansacSD::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	return QList<QAction*>{ m_action };
}

void qRansacSD::registerCommands(ccCommandLineInterface* cmd)
{
	if (!cmd)
	{
		assert(false);
		return;
	}
	cmd->registerCommand(ccCommandLineInterface::Command::Shared(new CommandRANSAC));
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
static unsigned s_supportPoints = 500;	// this is the minimal numer of points required for a primitive
static double   s_maxNormalDev_deg = 25.0;	// maximal normal deviation from ideal shape (in degrees)
static double   s_proba = 0.01;	// probability that no better candidate was overlooked during sampling
static bool s_primEnabled[5] = { true,true,true,false,false };

void qRansacSD::doAction()
{
	assert(m_app);
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		ccLog::Error("[qRansacSD] Select only one cloud!");
		return;
	}

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		ccLog::Error("[qRansacSD] Select a real point cloud!");
		return;
	}

	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	//input cloud
	CCVector3 bbMin, bbMax;
	pc->getBoundingBox(bbMin, bbMax);
	CCVector3 diff = bbMax - bbMin;
	float scale = std::max(std::max(diff[0], diff[1]), diff[2]);

	
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

	RansacParams params;
	{
		params.epsilon = static_cast<float>(rsdDlg.epsilonDoubleSpinBox->value());
		params.bitmapEpsilon = static_cast<float>(rsdDlg.bitmapEpsilonDoubleSpinBox->value());
		params.maxNormalDev_deg = static_cast<float>(rsdDlg.maxNormDevAngleSpinBox->value());
		params.probability = static_cast<float>(rsdDlg.probaDoubleSpinBox->value());
		params.randomColor = true;
		params.supportPoints = static_cast<unsigned>(rsdDlg.supportPointsSpinBox->value());
		params.primEnabled[RPT_PLANE] = rsdDlg.planeCheckBox->isChecked();
		params.primEnabled[RPT_SPHERE] = rsdDlg.sphereCheckBox->isChecked();
		params.primEnabled[RPT_CYLINDER] = rsdDlg.cylinderCheckBox->isChecked();
		params.primEnabled[RPT_CONE] = rsdDlg.coneCheckBox->isChecked();
		params.primEnabled[RPT_TORUS] = rsdDlg.torusCheckBox->isChecked();
	}

	ccHObject* group = executeRANSAC(pc, params, false);
	

	if (group)
	{
		m_app->addToDB(group);
		m_app->refreshAll();
	}
}


ccHObject* qRansacSD::executeRANSAC(ccPointCloud* ccPC, const RansacParams& params, bool silent)
{
	//consistency check
	{
		unsigned char primCount = 0;
		for (unsigned char k = 0; k < 5; ++k)
		{
			primCount += static_cast<unsigned>(params.primEnabled[k]);
		}
		if (primCount == 0)
		{
			ccLog::Error("[qRansacSD] No primitive type selected!");
			return nullptr;
		}
	}
	for (unsigned char k = 0; k < 5; ++k)
	{
		s_primEnabled[k] = params.primEnabled[k];
	}
	s_supportPoints = params.supportPoints;
	s_maxNormalDev_deg = params.maxNormalDev_deg;
	s_proba = params.probability;
	unsigned count = ccPC->size();
	bool hasNorms = ccPC->hasNormals();
	CCVector3 bbMin, bbMax;
	ccPC->getBoundingBox(bbMin, bbMax);
	const CCVector3d& globalShift = ccPC->getGlobalShift();
	double globalScale = ccPC->getGlobalScale();
	PointCloud cloud;
	{
		try
		{
			cloud.reserve(count);
		}
		catch (...)
		{
			ccLog::Error("[qRansacSD] Could not create temporary cloud, Not enough memory!");
			return nullptr;
		}

		//default point & normal
		Point Pt;
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;
		for (unsigned i = 0; i < count; ++i)
		{
			const CCVector3* P = ccPC->getPoint(i);
			Pt.pos[0] = static_cast<float>(P->x);
			Pt.pos[1] = static_cast<float>(P->y);
			Pt.pos[2] = static_cast<float>(P->z);
			if (hasNorms)
			{
				const CCVector3& N = ccPC->getPointNormal(i);
				Pt.normal[0] = static_cast<float>(N.x);
				Pt.normal[1] = static_cast<float>(N.y);
				Pt.normal[2] = static_cast<float>(N.z);
			}
#ifdef POINTSWITHINDEX
			Pt.index = i;
#endif
			cloud.push_back(Pt);
		}

		//manually set bounding box!
		Vec3f cbbMin, cbbMax;
		cbbMin[0] = static_cast<float>(bbMin.x);
		cbbMin[1] = static_cast<float>(bbMin.y);
		cbbMin[2] = static_cast<float>(bbMin.z);
		cbbMax[0] = static_cast<float>(bbMax.x);
		cbbMax[1] = static_cast<float>(bbMax.y);
		cbbMax[2] = static_cast<float>(bbMax.z);
		cloud.setBBox(cbbMin, cbbMax);
	}

	RansacShapeDetector::Options ransacOptions;
	{
		ransacOptions.m_epsilon = params.epsilon;
		ransacOptions.m_bitmapEpsilon = params.bitmapEpsilon;
		ransacOptions.m_normalThresh = static_cast<float>(cos(params.maxNormalDev_deg * CC_DEG_TO_RAD));
		assert(ransacOptions.m_normalThresh >= 0);
		ransacOptions.m_probability = params.probability;
		ransacOptions.m_minSupport = params.supportPoints;
	}
	const float scale = cloud.getScale();

	if (!hasNorms)
	{
		QProgressDialog* pDlg = nullptr;
		if (!silent)
		{
			if (s_app)
			{
				pDlg = new QProgressDialog("Computing normals (please wait)", QString(), 0, 0, s_app->getMainWindow());
			}
			else
			{
				pDlg = new QProgressDialog("Computing normals (please wait)", QString(), 0, 0, nullptr);
			}
			pDlg->setWindowTitle("Ransac Shape Detection");
			pDlg->show();
		}
		QApplication::processEvents();

		cloud.calcNormals(.01f * scale);

		if (ccPC->reserveTheNormsTable())
		{
			for (unsigned i = 0; i < count; ++i)
			{
				Vec3f& Nvi = cloud[i].normal;
				CCVector3 Ni = CCVector3::fromArray(Nvi);
				//normalize the vector in case of
				Ni.normalize();
				ccPC->addNorm(Ni);
			}
			ccPC->showNormals(true);

			//currently selected entities appearance may have changed!
			ccPC->prepareDisplayForRefresh_recursive();
		}
		else
		{
			ccLog::Error("[qRansacSD] Not enough memory to compute normals!");
			if (pDlg)
			{
				pDlg->hide();
				delete pDlg;
			}
			return nullptr;
		}
		if (pDlg)
		{
			pDlg->hide();
			delete pDlg;
		}
	}

	RansacShapeDetector detector(ransacOptions); // the detector object

	if (params.primEnabled[RPT_PLANE])
		detector.Add(new PlanePrimitiveShapeConstructor());
	if (params.primEnabled[RPT_SPHERE])
		detector.Add(new SpherePrimitiveShapeConstructor());
	if (params.primEnabled[RPT_CYLINDER])
		detector.Add(new CylinderPrimitiveShapeConstructor());
	if (params.primEnabled[RPT_CONE])
		detector.Add(new ConePrimitiveShapeConstructor());
	if (params.primEnabled[RPT_TORUS])
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
		QProgressDialog* pDlg = nullptr;
		if (!silent)
		{
			if (s_app)
			{
				pDlg = new QProgressDialog("Operation in progress (please wait)", QString(), 0, 0, s_app->getMainWindow());
			}
			else
			{
				pDlg = new QProgressDialog("Operation in progress (please wait)", QString(), 0, 0, nullptr);
			}
			pDlg->setWindowTitle("Ransac Shape Detection");
			pDlg->show();
		}

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
			if (!silent && pDlg)
			{
				pDlg->setValue(pDlg->value() + 1);
			}
			QApplication::processEvents();
		}
		remaining = static_cast<unsigned>(s_remainingPoints);

		QApplication::processEvents();
		if (pDlg)
		{
			pDlg->hide();
			delete pDlg;
		}
	}

#if 0 //def _DEBUG
	FILE* fp = fopen("RANS_SD_trace.txt", "wt");

	fprintf(fp, "[Options]\n");
	fprintf(fp, "epsilon=%f\n", ransacOptions.m_epsilon);
	fprintf(fp, "bitmap epsilon=%f\n", ransacOptions.m_bitmapEpsilon);
	fprintf(fp, "normal thresh=%f\n", ransacOptions.m_normalThresh);
	fprintf(fp, "min support=%i\n", ransacOptions.m_minSupport);
	fprintf(fp, "probability=%f\n", ransacOptions.m_probability);

	fprintf(fp, "\n[Statistics]\n");
	fprintf(fp, "input points=%i\n", count);
	fprintf(fp, "segmented=%i\n", count - remaining);
	fprintf(fp, "remaining=%i\n", remaining);

	if (shapes.size() > 0)
	{
		fprintf(fp, "\n[Shapes]\n");
		for (unsigned i = 0; i < shapes.size(); ++i)
		{
			PrimitiveShape* shape = shapes[i].first;
			size_t shapePointsCount = shapes[i].second;

			std::string desc;
			shape->Description(&desc);
			fprintf(fp, "#%i - %s - %i points\n", i + 1, desc.c_str(), shapePointsCount);
		}
	}
	fclose(fp);
#endif


	if (remaining == count)
	{
		ccLog::Error("[qRansacSD] Segmentation failed...");
		return nullptr;
	}

	if (shapes.size() > 0)
	{
		unsigned planeCount = 1;
		unsigned sphereCount = 1;
		unsigned cylinderCount = 1;
		unsigned coneCount = 1;
		unsigned torusCount = 1;
		ccHObject* group = nullptr;
		for (MiscLib::Vector<DetectedShape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
		{
			const PrimitiveShape* shape = it->first;
			unsigned shapePointsCount = static_cast<unsigned>(it->second);

			//too many points?!
			if (shapePointsCount > count)
			{
				ccLog::Error("[qRansacSD] Inconsistent result!");
				break;
			}

			if (shapePointsCount < params.supportPoints)
			{
				ccLog::Warning("[qRansacSD] Skipping shape, did not meet minimum point requirement");
				count -= shapePointsCount;
				continue;
			}

			std::string desc;
			shape->Description(&desc);

			// points to current shapes last point in cloud
			const auto shapeCloudIndex = count - 1;

			//new cloud for sub-part
			ccPointCloud* pcShape = nullptr;
			bool saveNormals = true;
			{
#ifdef POINTSWITHINDEX
				CCLib::ReferenceCloud refPcShape(ccPC);
				//we fill cloud with sub-part points
				if (!refPcShape.reserve(static_cast<unsigned>(shapePointsCount)))
				{
					ccLog::Error("[qRansacSD] Not enough memory!");
					break;
				}

				for (unsigned j = 0; j < shapePointsCount; ++j)
				{
					refPcShape.addPointIndex(cloud[shapeCloudIndex - j].index);
				}
				int warnings = 0;
				pcShape = ccPC->partialClone(&refPcShape, &warnings);
				if (warnings != 0)
				{
					if ((warnings & ccPointCloud::WRN_OUT_OF_MEM_FOR_NORMALS) == ccPointCloud::WRN_OUT_OF_MEM_FOR_NORMALS)
					{
						saveNormals = false;
					}
				}
				
#else
				pcShape = new ccPointCloud(desc.c_str());
				if (!pcShape->reserve(static_cast<unsigned>(shapePointsCount)))
				{
					ccLog::Error("[qRansacSD] Not enough memory!");
					delete pcShape;
					break;
				}
				saveNormals = pcShape->reserveTheNormsTable();

				for (unsigned j = 0; j < shapePointsCount; ++j)
				{
					pcShape->addPoint(CCVector3::fromArray(cloud[shapeCloudIndex - j].pos));
					if (saveNormals)
					{
						pcShape->addNorm(CCVector3::fromArray(cloud[shapeCloudIndex - j].normal));
					}

				}
				pcShape->setGlobalShift(globalShift);
				pcShape->setGlobalScale(globalScale);
#endif
			}
			//random color
			ccColor::Rgb col = ccColor::Generator::Random();
			if (params.randomColor)
			{
				pcShape->setColor(col);
				pcShape->showSF(false);
				pcShape->showColors(true);
			}
			pcShape->showNormals(saveNormals);
			pcShape->setVisible(true);
			

			//convert detected primitive into a CC primitive type
			ccGenericPrimitive* prim = nullptr;
			switch (shape->Identifier())
			{
			case RPT_PLANE: //plane
			{
				const PlanePrimitiveShape* plane = static_cast<const PlanePrimitiveShape*>(shape);
				Vec3f G = plane->Internal().getPosition();
				Vec3f N = plane->Internal().getNormal();
				Vec3f X = plane->getXDim();
				Vec3f Y = plane->getYDim();

				//we look for real plane extents
				float minX, maxX, minY, maxY;
				for (unsigned j = 0; j < shapePointsCount; ++j)
				{
					std::pair<float, float> param;
					plane->Parameters(cloud[shapeCloudIndex - j].pos, &param);
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
				float dX = maxX - minX;
				float dY = maxY - minY;
				G += X * (minX + dX / 2);
				G += Y * (minY + dY / 2);

				//we build matrix from these vectors
				ccGLMatrix glMat(CCVector3::fromArray(X.getValue()),
					CCVector3::fromArray(Y.getValue()),
					CCVector3::fromArray(N.getValue()),
					CCVector3::fromArray(G.getValue()));

				//plane primitive
				//ccLog::Print(QString("dX: %1, dY: %2").arg(dX).arg(dY));
				prim = new ccPlane(std::abs(dX), std::abs(dY), &glMat);
				prim->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);
				prim->enableStippling(true);
				PointCoordinateType dip = 0.0f;
				PointCoordinateType dipDir = 0.0f;
				ccNormalVectors::ConvertNormalToDipAndDipDir(CCVector3::fromArray(N.getValue()), dip, dipDir);
				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
				prim->setName(dipAndDipDirStr);
				pcShape->setName(QString("Plane_%1").arg(planeCount, 4, 10, QChar('0')));
				planeCount++;
			}
			break;

			case RPT_SPHERE: //sphere
			{
				const SpherePrimitiveShape* sphere = static_cast<const SpherePrimitiveShape*>(shape);
				float radius = sphere->Internal().Radius();
				Vec3f CC = sphere->Internal().Center();		

				//we build matrix from these vecctors
				ccGLMatrix glMat;
				glMat.setTranslation(CC.getValue());
				//sphere primitive
				prim = new ccSphere(radius, &glMat);
				prim->setEnabled(false);
				prim->setName(QString("Sphere (r=%1)").arg(radius, 0, 'f'));
				pcShape->setName(QString("Sphere_%1").arg(sphereCount, 4, 10, QChar('0')));
				sphereCount++;
			}
			break;

			case RPT_CYLINDER: //cylinder
			{
				const CylinderPrimitiveShape* cyl = static_cast<const CylinderPrimitiveShape*>(shape);
				Vec3f G = cyl->Internal().AxisPosition();
				Vec3f N = cyl->Internal().AxisDirection();
				Vec3f X = cyl->Internal().AngularDirection();
				Vec3f Y = N.cross(X);
				float r = cyl->Internal().Radius();
				float hMin = cyl->MinHeight();
				float hMax = cyl->MaxHeight();
				float h = hMax - hMin;
				G += N * (hMin + h / 2);		

				//we build matrix from these vecctors
				ccGLMatrix glMat(CCVector3::fromArray(X.getValue()),
					CCVector3::fromArray(Y.getValue()),
					CCVector3::fromArray(N.getValue()),
					CCVector3::fromArray(G.getValue()));

				//cylinder primitive
				prim = new ccCylinder(r, h, &glMat);
				prim->setEnabled(false);
				prim->setName(QString("Cylinder (r=%1/h=%2)").arg(r, 0, 'f').arg(h, 0, 'f'));
				pcShape->setName(QString("Cylinder_%1").arg(cylinderCount, 4, 10, QChar('0')));
				cylinderCount++;
			}
			break;

			case RPT_CONE: //cone
			{
				const ConePrimitiveShape* cone = static_cast<const ConePrimitiveShape*>(shape);
				Vec3f CC = cone->Internal().Center();
				Vec3f CA = cone->Internal().AxisDirection();
				float alpha = cone->Internal().Angle();

				//compute max height
				Vec3f minP, maxP;
				float minHeight, maxHeight;
				minP = maxP = cloud[shapeCloudIndex].pos;
				minHeight = maxHeight = cone->Internal().Height(cloud[shapeCloudIndex].pos);
				for (size_t j = 1; j < shapePointsCount; ++j)
				{
					float h = cone->Internal().Height(cloud[shapeCloudIndex - j].pos);
					if (h < minHeight)
					{
						minHeight = h;
						minP = cloud[shapeCloudIndex - j].pos;
					}
					else if (h > maxHeight)
					{
						maxHeight = h;
						maxP = cloud[shapeCloudIndex - j].pos;
					}

				}


				float minRadius = tan(alpha) * minHeight;
				float maxRadius = tan(alpha) * maxHeight;

				//let's build the cone primitive
				{
					//the bottom should be the largest part so we inverse the axis direction
					CCVector3 Z = -CCVector3::fromArray(CA.getValue());
					Z.normalize();

					//the center is halfway between the min and max height
					float midHeight = (minHeight + maxHeight) / 2;
					CCVector3 C = CCVector3::fromArray((CC + CA * midHeight).getValue());

					//radial axis
					CCVector3 X = CCVector3::fromArray((maxP - (CC + maxHeight * CA)).getValue());
					X.normalize();

					//orthogonal radial axis
					CCVector3 Y = Z * X;

					//we build the transformation matrix from these vecctors
					ccGLMatrix glMat(X, Y, Z, C);

					//eventually create the cone primitive
					prim = new ccCone(maxRadius, minRadius, maxHeight - minHeight, 0, 0, &glMat);
					prim->setEnabled(false);
					prim->setName(QString("Cone (alpha=%1/h=%2)").arg(alpha, 0, 'f').arg(maxHeight - minHeight, 0, 'f'));
					pcShape->setName(QString("Cone_%1").arg(coneCount, 4, 10, QChar('0')));
					coneCount++;
				}

			}
			break;

			case RPT_TORUS: //torus
			{
				const TorusPrimitiveShape* torus = static_cast<const TorusPrimitiveShape*>(shape);
				if (torus->Internal().IsAppleShaped())
				{
					ccLog::Warning("[qRansacSD] Apple-shaped torus are not handled by CloudCompare!");
				}
				else
				{
					Vec3f CC = torus->Internal().Center();
					Vec3f CA = torus->Internal().AxisDirection();
					float minRadius = torus->Internal().MinorRadius();
					float maxRadius = torus->Internal().MajorRadius();

					CCVector3 Z = CCVector3::fromArray(CA.getValue());
					CCVector3 C = CCVector3::fromArray(CC.getValue());
					//construct remaining of base
					CCVector3 X = Z.orthogonal();
					CCVector3 Y = Z * X;

					//we build matrix from these vecctors
					ccGLMatrix glMat(X, Y, Z, C);

					//torus primitive
					prim = new ccTorus(maxRadius - minRadius, maxRadius + minRadius, M_PI * 2.0, false, 0, &glMat);
					prim->setEnabled(false);
					prim->setName(QString("Torus (r=%1/R=%2)").arg(minRadius, 0, 'f').arg(maxRadius, 0, 'f'));
					pcShape->setName(QString("Torus_%1").arg(torusCount, 4, 10, QChar('0')));
					torusCount++;
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
				group = new ccHObject(QString("Ransac Detected Shapes (%1)").arg(ccPC->getName()));
			group->addChild(pcShape);

			count -= shapePointsCount;

			QApplication::processEvents();
		}

		if (group)
		{
			assert(group->getChildrenNumber() != 0);

			//we hide input cloud
			ccPC->setEnabled(false);
			ccLog::Warning("[qRansacSD] Input cloud has been automtically hidden!");

			
			group->setVisible(true);
			group->setDisplay_recursive(ccPC->getDisplay());
			return group;
		}
	}

	return nullptr;
}