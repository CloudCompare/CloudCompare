// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
// #                                                                        #
// ##########################################################################

// Always first
#include "ccGenericMesh.h"

#include "ccIncludeGL.h"

// local
#include "ccColorScalesManager.h"
#include "ccGenericGLDisplay.h"
#include "ccGenericPointCloud.h"
#include "ccHObjectCaster.h"
#include "ccMaterialSet.h"
#include "ccNormalVectors.h"
#include "ccPointCloud.h"
#include "ccScalarField.h"

// CCCoreLib
#include <GenericProgressCallback.h>
#include <GenericTriangle.h>
#include <MeshSamplingTools.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>

// system
#include <cassert>

// QT
#include <QPainter>

#if defined(_OPENMP)
// OpenMP
#include <omp.h>
#endif

ccGenericMesh::ccGenericMesh(QString name /*=QString()*/, unsigned uniqueID /*=ccUniqueIDGenerator::InvalidUniqueID*/)
    : GenericIndexedMesh()
    , ccShiftedObject(name, uniqueID)
    , m_triNormsShown(false)
    , m_materialsShown(false)
    , m_showWired(false)
    , m_stippling(false)
    , m_forceSunLightOn(false)
{
	setVisible(true);
	lockVisibility(false);
}

void ccGenericMesh::showNormals(bool state)
{
	showTriNorms(state);
	ccHObject::showNormals(state);
}

// stipple mask (for semi-transparent display of meshes)
static const GLubyte s_byte0               = 1 | 4 | 16 | 64;
static const GLubyte s_byte1               = 2 | 8 | 32 | 128;
static const GLubyte s_stippleMask[4 * 32] = {s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1, s_byte0, s_byte0, s_byte0, s_byte0, s_byte1, s_byte1, s_byte1, s_byte1};

void ccGenericMesh::EnableGLStippleMask(QOpenGLContext* context, bool state)
{
	// get the set of OpenGL functions (version 2.1)
	auto* glFunc = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(context);
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (state)
	{
		glFunc->glPolygonStipple(s_stippleMask);
		glFunc->glEnable(GL_POLYGON_STIPPLE);
	}
	else
	{
		glFunc->glDisable(GL_POLYGON_STIPPLE);
	}
}

void ccGenericMesh::handleColorRamp(CC_DRAW_CONTEXT& context)
{
	if (MACRO_Draw2D(context))
	{
		if (MACRO_Foreground(context) && !context.sfColorScaleToDisplay)
		{
			if (sfShown())
			{
				ccGenericPointCloud* vertices = getAssociatedCloud();
				if (!vertices || !vertices->isA(CC_TYPES::POINT_CLOUD))
					return;

				ccPointCloud* cloud = static_cast<ccPointCloud*>(vertices);

				// we just need to 'display' the current SF scale if the vertices cloud is hidden
				//(otherwise, it will be taken in charge by the cloud itself)
				if (!cloud->sfColorScaleShown() || (cloud->sfShown() && cloud->isEnabled() && cloud->isVisible()))
					return;

				// we must also check that the parent is not a mesh itself with the same vertices! (in
				// which case it will also take that in charge)
				ccHObject* parent = getParent();
				if (parent && parent->isKindOf(CC_TYPES::MESH) && (ccHObjectCaster::ToGenericMesh(parent)->getAssociatedCloud() == vertices))
					return;

				cloud->addColorRampInfo(context);
				// cloud->drawScale(context);
			}
		}
	}
}

bool ccGenericMesh::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 29)
	{
		assert(false);
		return false;
	}

	if (!ccHObject::toFile_MeOnly(out, dataVersion))
	{
		return false;
	}

	//'show wired' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_showWired), sizeof(bool)) < 0)
		return WriteError();

	//'per-triangle normals shown' state (dataVersion>=29))
	if (out.write(reinterpret_cast<const char*>(&m_triNormsShown), sizeof(bool)) < 0)
		return WriteError();

	//'materials shown' state (dataVersion>=29))
	if (out.write(reinterpret_cast<const char*>(&m_materialsShown), sizeof(bool)) < 0)
		return WriteError();

	//'polygon stippling' state (dataVersion>=29))
	if (out.write(reinterpret_cast<const char*>(&m_stippling), sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool ccGenericMesh::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccHObject::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
	{
		return false;
	}

	//'show wired' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_showWired), sizeof(bool)) < 0)
	{
		return ReadError();
	}

	//'per-triangle normals shown' state (dataVersion>=29))
	if (dataVersion >= 29)
	{
		if (in.read(reinterpret_cast<char*>(&m_triNormsShown), sizeof(bool)) < 0)
		{
			return ReadError();
		}

		//'materials shown' state (dataVersion>=29))
		if (in.read(reinterpret_cast<char*>(&m_materialsShown), sizeof(bool)) < 0)
		{
			return ReadError();
		}

		//'polygon stippling' state (dataVersion>=29))
		if (in.read(reinterpret_cast<char*>(&m_stippling), sizeof(bool)) < 0)
		{
			return ReadError();
		}
	}

	return true;
}

short ccGenericMesh::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(29), ccHObject::minimumFileVersion_MeOnly());
}

ccPointCloud* ccGenericMesh::samplePoints(bool                                densityBased,
                                          double                              samplingParameter,
                                          bool                                withNormals,
                                          bool                                withRGB,
                                          bool                                withTexture,
                                          CCCoreLib::GenericProgressCallback* pDlg /*=nullptr*/)
{
	if (samplingParameter <= 0)
	{
		assert(false);
		return nullptr;
	}

	bool withFeatures = (withNormals || withRGB || withTexture);

	std::unique_ptr<std::vector<unsigned>> triIndices;
	if (withFeatures)
	{
		triIndices.reset(new std::vector<unsigned>);
	}

	CCCoreLib::PointCloud* sampledCloud = nullptr;
	if (densityBased)
	{
		sampledCloud = CCCoreLib::MeshSamplingTools::samplePointsOnMesh(this, samplingParameter, pDlg, triIndices.get());
	}
	else
	{
		sampledCloud = CCCoreLib::MeshSamplingTools::samplePointsOnMesh(this, static_cast<unsigned>(samplingParameter), pDlg, triIndices.get());
	}

	// convert to real point cloud
	ccPointCloud* cloud = nullptr;

	if (sampledCloud)
	{
		if (sampledCloud->size() == 0)
		{
			ccLog::Warning("[ccGenericMesh::samplePoints] No point was generated (sampling density is too low?)");
		}
		else
		{
			cloud = ccPointCloud::From(sampledCloud);
			if (!cloud)
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Not enough memory!");
			}
		}

		delete sampledCloud;
		sampledCloud = nullptr;
	}
	else
	{
		ccLog::Warning("[ccGenericMesh::samplePoints] Not enough memory!");
	}

	if (!cloud)
	{
		return nullptr;
	}

	if (withFeatures && triIndices && triIndices->size() >= cloud->size())
	{
		// generate normals
		if (withNormals && hasNormals())
		{
			if (cloud->reserveTheNormsTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned         triIndex = triIndices->at(i);
					const CCVector3* P        = cloud->getPoint(i);

					CCVector3 N(0, 0, 1);
					interpolateNormals(triIndex, *P, N);
					cloud->addNorm(N);
				}

				cloud->showNormals(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to interpolate normals (not enough memory?)");
			}
		}

		// generate colors
		if (withTexture && hasMaterials())
		{
			if (cloud->reserveTheRGBTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned         triIndex = triIndices->at(i);
					const CCVector3* P        = cloud->getPoint(i);

					ccColor::Rgba color;
					getColorFromMaterial(triIndex, *P, color, withRGB);
					cloud->addColor(color);
				}

				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to export texture colors (not enough memory?)");
			}
		}
		else if (withRGB && hasColors())
		{
			if (cloud->reserveTheRGBTable())
			{
				for (unsigned i = 0; i < cloud->size(); ++i)
				{
					unsigned         triIndex = triIndices->at(i);
					const CCVector3* P        = cloud->getPoint(i);

					ccColor::Rgb C;
					interpolateColors(triIndex, *P, C);
					cloud->addColor(C);
				}

				cloud->showColors(true);
			}
			else
			{
				ccLog::Warning("[ccGenericMesh::samplePoints] Failed to interpolate colors (not enough memory?)");
			}
		}
	}

	// we rename the resulting cloud
	cloud->setName(getName() + QString(".sampled"));
	cloud->setDisplay(getDisplay());
	cloud->prepareDisplayForRefresh();

	// import parameters from the source mesh
	cloud->copyGlobalShiftAndScale(*this);
	cloud->setGLTransformationHistory(getGLTransformationHistory());

	return cloud;
}

void ccGenericMesh::importParametersFrom(const ccGenericMesh* mesh)
{
	if (!mesh)
	{
		assert(false);
		return;
	}

	// original shift & scale
	copyGlobalShiftAndScale(*mesh);

	// stippling
	enableStippling(mesh->stipplingEnabled());
	// wired style
	showWired(mesh->isShownAsWire());

	// keep the transformation history!
	setGLTransformationHistory(mesh->getGLTransformationHistory());
	// and meta-data
	setMetaData(mesh->metaData());
}

void ccGenericMesh::computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const
{
	CCCoreLib::GenericTriangle* tri = const_cast<ccGenericMesh*>(this)->_getTriangle(triIndex);
	const CCVector3*            A   = tri->_getA();
	const CCVector3*            B   = tri->_getB();
	const CCVector3*            C   = tri->_getC();

	// barycentric interpolation weights
	weights.x = ((P - *B).cross(*C - *B)).normd() /*/2*/;
	weights.y = ((P - *C).cross(*A - *C)).normd() /*/2*/;
	weights.z = ((P - *A).cross(*B - *A)).normd() /*/2*/;

	// normalize weights
	double sum = weights.x + weights.y + weights.z;
	weights /= sum;
}

bool ccGenericMesh::trianglePicking(unsigned                    triIndex,
                                    const CCVector2d&           clickPos,
                                    const ccGLMatrix&           trans,
                                    bool                        noGLTrans,
                                    const ccGenericPointCloud&  vertices,
                                    const ccGLCameraParameters& camera,
                                    bool                        edgeOnly,
                                    CCVector3d&                 point,
                                    CCVector3d*                 barycentricCoords /*=nullptr*/) const
{
	assert(triIndex < size());

	CCVector3 A3D;
	CCVector3 B3D;
	CCVector3 C3D;
	getTriangleVertices(triIndex, A3D, B3D, C3D);

	CCVector3d A2D;
	CCVector3d B2D;
	CCVector3d C2D;
	bool       insideA = false;
	bool       insideB = false;
	bool       insideC = false;

	if (noGLTrans)
	{
		camera.project(A3D, A2D, &insideA);
		camera.project(B3D, B2D, &insideB);
		camera.project(C3D, C2D, &insideC);
	}
	else
	{
		CCVector3 A3Dp = trans * A3D;
		CCVector3 B3Dp = trans * B3D;
		CCVector3 C3Dp = trans * C3D;
		camera.project(A3Dp, A2D, &insideA);
		camera.project(B3Dp, B2D, &insideB);
		camera.project(C3Dp, C2D, &insideC);
	}

	// if none of the vertices fall inside the frustum, the triangle is (probably) not visible...
	if (!insideA && !insideB && !insideC)
	{
		// If there's one huge triangle or the user zoom in a lot, it's not true!
		// So we only use this if there are a lot of triangles
		if (size() > 10000)
		{
			return false;
		}
	}

	// barycentric coordinates
	GLdouble detT = (B2D.y - C2D.y) * (A2D.x - C2D.x) + (C2D.x - B2D.x) * (A2D.y - C2D.y);
	if (CCCoreLib::LessThanEpsilon(std::abs(detT)))
	{
		return false;
	}
	GLdouble l1 = ((B2D.y - C2D.y) * (clickPos.x - C2D.x) + (C2D.x - B2D.x) * (clickPos.y - C2D.y)) / detT;
	GLdouble l2 = ((C2D.y - A2D.y) * (clickPos.x - C2D.x) + (A2D.x - C2D.x) * (clickPos.y - C2D.y)) / detT;
	GLdouble l3 = 1.0 - l1 - l2;

	double l1l2 = l1 + l2;
	if (l1l2 > 1.0)
	{
		// we fall outside of the triangle!
		return false;
	}

	// does the point falls inside the triangle?
	if (edgeOnly)
	{
		double espilon = 15 / ((C2D - A2D).norm() + (B2D - A2D).norm() + (C2D - B2D).norm());
		bool   onEdge  = false;

		if (l1 > -espilon && l1 <= 1.0 && l2 > -espilon && l2 <= 1.0)
		{
			if (l1 < espilon)
			{
				l1     = 0;
				onEdge = true;
			}
			if (l2 < espilon)
			{
				l2     = 0;
				onEdge = true;
			}
			if (l3 < espilon)
			{
				l3     = 0;
				onEdge = true;
			}

			if (!onEdge)
			{
				// not on an edge
				return false;
			}

			// normalize
			double sum = l1 + l2 + l3;
			l1 /= sum;
			l2 /= sum;
			l3 /= sum;
		}
		else
		{
			return false;
		}
	}
	else if (l1 < 0 || l1 > 1.0 || l2 < 0.0 || l2 > 1.0)
	{
		return false;
	}

	// now deduce the 3D position
	point = CCVector3d(l1 * A3D.x + l2 * B3D.x + l3 * C3D.x,
	                   l1 * A3D.y + l2 * B3D.y + l3 * C3D.y,
	                   l1 * A3D.z + l2 * B3D.z + l3 * C3D.z);

	if (barycentricCoords)
	{
		*barycentricCoords = CCVector3d(l1, l2, l3);
	}

	return true;
}

bool ccGenericMesh::trianglePicking(const CCVector2d&           clickPos,
                                    const ccGLCameraParameters& camera,
                                    bool                        edgeOnly,
                                    int&                        nearestTriIndex,
                                    double&                     nearestSquareDist,
                                    CCVector3d&                 nearestPoint,
                                    CCVector3d*                 barycentricCoords /*=nullptr*/) const
{
	ccGLMatrix trans;
	bool       noGLTrans = !getAbsoluteGLTransformation(trans);

	// back project the clicked point in 3D
	CCVector3d clickPosd(clickPos.x, clickPos.y, 0);
	CCVector3d X(0, 0, 0);
	if (!camera.unproject(clickPosd, X))
	{
		return false;
	}

	nearestTriIndex   = -1;
	nearestSquareDist = -1.0;
	nearestPoint      = CCVector3d(0, 0, 0);
	if (barycentricCoords)
	{
		*barycentricCoords = CCVector3d(0, 0, 0);
	}

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

#if defined(_OPENMP) && !defined(_DEBUG) && !defined(TEST_PICKING)
#pragma omp parallel for num_threads(omp_get_max_threads())
#endif
	for (int i = 0; i < static_cast<int>(size()); ++i)
	{
		CCVector3d P;
		CCVector3d BC;
		if (!trianglePicking(i,
		                     clickPos,
		                     trans,
		                     noGLTrans,
		                     *vertices,
		                     camera,
		                     edgeOnly,
		                     P,
		                     barycentricCoords ? &BC : nullptr))
		{
			continue;
		}

		double squareDist = (X - P).norm2d();
		if (nearestTriIndex < 0 || squareDist < nearestSquareDist)
		{
			nearestSquareDist = squareDist;
			nearestTriIndex   = static_cast<int>(i);
			nearestPoint      = P;
			if (barycentricCoords)
			{
				*barycentricCoords = BC;
			}
		}
	}

	return (nearestTriIndex >= 0);
}

bool ccGenericMesh::trianglePicking(unsigned                    triIndex,
                                    const CCVector2d&           clickPos,
                                    const ccGLCameraParameters& camera,
                                    bool                        edgeOnly,
                                    CCVector3d&                 point,
                                    CCVector3d*                 barycentricCoords /*=nullptr*/) const
{
	if (triIndex >= size())
	{
		assert(false);
		return false;
	}

	ccGLMatrix trans;
	bool       noGLTrans = !getAbsoluteGLTransformation(trans);

	ccGenericPointCloud* vertices = getAssociatedCloud();
	if (!vertices)
	{
		assert(false);
		return false;
	}

	return trianglePicking(triIndex,
	                       clickPos,
	                       trans,
	                       noGLTrans,
	                       *vertices,
	                       camera,
	                       edgeOnly,
	                       point,
	                       barycentricCoords);
}

bool ccGenericMesh::computePointPosition(unsigned triIndex, const CCVector2d& uv, CCVector3& P, bool warningIfOutside /*=true*/) const
{
	if (triIndex >= size())
	{
		assert(false);
		ccLog::Warning("Index out of range");
		return true;
	}

	CCVector3 A;
	CCVector3 B;
	CCVector3 C;
	getTriangleVertices(triIndex, A, B, C);

	double z = 1.0 - uv.x - uv.y;
	if (warningIfOutside && ((z < -1.0e-6) || (z > 1.0 + 1.0e-6)))
	{
		ccLog::Warning("Point falls outside of the triangle");
	}

	P = CCVector3(static_cast<PointCoordinateType>(uv.x * A.x + uv.y * B.x + z * C.x),
	              static_cast<PointCoordinateType>(uv.x * A.y + uv.y * B.y + z * C.y),
	              static_cast<PointCoordinateType>(uv.x * A.z + uv.y * B.z + z * C.z));

	return true;
}

void ccGenericMesh::setGlobalShift(const CCVector3d& shift)
{
	if (getAssociatedCloud())
	{
		// auto transfer the global shift info to the vertices
		getAssociatedCloud()->setGlobalShift(shift);
	}
	else
	{
		// we normally don't want to store this information at
		// the mesh level as it won't be saved.
		assert(false);
		ccShiftedObject::setGlobalShift(shift);
	}
}

void ccGenericMesh::setGlobalScale(double scale)
{
	if (getAssociatedCloud())
	{
		// auto transfer the global scale info to the vertices
		getAssociatedCloud()->setGlobalScale(scale);
	}
	else
	{
		// we normally don't want to store this information at
		// the mesh level as it won't be saved.
		assert(false);
		ccShiftedObject::setGlobalScale(scale);
	}
}

const CCVector3d& ccGenericMesh::getGlobalShift() const
{
	return (getAssociatedCloud() ? getAssociatedCloud()->getGlobalShift() : ccShiftedObject::getGlobalShift());
}

double ccGenericMesh::getGlobalScale() const
{
	return (getAssociatedCloud() ? getAssociatedCloud()->getGlobalScale() : ccShiftedObject::getGlobalScale());
}

bool ccGenericMesh::IsCloudVerticesOfMesh(ccGenericPointCloud* cloud, ccGenericMesh** mesh /*=nullptr*/)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	// check whether the input point cloud acts as the vertices of a mesh
	{
		ccHObject* parent = cloud->getParent();
		if (parent && parent->isKindOf(CC_TYPES::MESH) && static_cast<ccGenericMesh*>(parent)->getAssociatedCloud() == cloud)
		{
			if (mesh)
			{
				*mesh = static_cast<ccGenericMesh*>(parent);
			}
			return true;
		}
	}

	// now check the children
	for (unsigned i = 0; i < cloud->getChildrenNumber(); ++i)
	{
		ccHObject* child = cloud->getChild(i);
		if (child && child->isKindOf(CC_TYPES::MESH) && static_cast<ccGenericMesh*>(child)->getAssociatedCloud() == cloud)
		{
			if (mesh)
			{
				*mesh = static_cast<ccGenericMesh*>(child);
			}
			return true;
		}
	}

	return false;
}
