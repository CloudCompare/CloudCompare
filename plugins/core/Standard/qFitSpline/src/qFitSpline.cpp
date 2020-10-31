//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qFitSpline                    #
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

#include "qFitSpline.h"

//qCC_db
#include <ccPointCloud.h>
#include <ccSpline.h>

//Qt
#include <QCoreApplication>
#include <QMainWindow>
#include <QProgressDialog>

//Eigen (for Levenberg-Marquardt)
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

//system
#include <cassert>

//! Spline "segment"
struct Segment
{
	typedef QSharedPointer<Segment> Shared;
	CCVector3 A, B;
	double posA = 0.0, posB = 0.0;
	Shared AC, CB;
};

static double DistanceToSegment(const CCVector3& P, Segment& seg)
{
	CCVector3 AB = seg.B - seg.A; //(unit) direction
	double lAB = AB.normd(); //length

	CCVector3 AP = P - seg.A;
	if (CCCoreLib::LessThanEpsilon(lAB))
	{
		//ignore too small segments
		return sqrt(std::min(AP.norm2d(), (P - seg.B).norm2d()));
	}

	CCVector3 uAB = AB / lAB;

	double dotProd = (P - seg.A).dot(uAB);

	if (dotProd <= 0)
	{
		return (P - seg.A).normd();
	}
	else if (dotProd >= lAB)
	{
		return (P - seg.B).normd();
	}
	else
	{
		return (AP - dotProd * uAB).normd();
	}
}

struct RecursiveSearch
{
	RecursiveSearch(const ccSpline& s,
					const CCVector3& p,
					double initialDistance,
					double convDelta)
		: spline(s)
		, P(p)
		, minDistance(initialDistance)
		, convergenceDelta(convDelta)
	{}
	
	void distanceToSegment(Segment& seg)
	{
		++depth;

		if (!seg.AC)
		{
			seg.AC.reset(new Segment);
			seg.AC->posA = seg.posA;
			seg.AC->A = seg.A;
			seg.AC->posB = (seg.posA + seg.posB) / 2;
			if (!spline.computePosition(seg.AC->posB, seg.AC->B))
			{
				assert(false);
				minDistance = std::numeric_limits<double>::quiet_NaN();
				return;
			}
		}
		double distToAC = DistanceToSegment(P, *seg.AC);

		if (!seg.CB)
		{
			seg.CB.reset(new Segment);
			seg.CB->posB = seg.posB;
			seg.CB->B = seg.B;
			seg.CB->posA = seg.AC->posB;
			seg.CB->A = seg.AC->B;
		}
		double distToCB = DistanceToSegment(P, *seg.CB);

		if (distToAC < distToCB)
		{
			double delta = minDistance - distToAC;
			minDistance = distToAC;
			if (delta > convergenceDelta)
			{
				//continue recursion
				distanceToSegment(*seg.AC);
			}
		}
		else  //(distToAC >= distToCB)
		{
			double delta = minDistance - distToCB;
			minDistance = distToCB;
			if (delta > convergenceDelta)
			{
				//continue recursion
				distanceToSegment(*seg.CB);
			}
		}
	}

	const ccSpline& spline;
	const CCVector3& P;
	double minDistance;
	double convergenceDelta;
	unsigned depth = 0;
};

static bool ComputeDistances(	const ccPointCloud& cloud,
								const ccSpline& spline,
								std::vector<double>& distances,
								double& rms,
								double precision,
								bool silent = true)
{
	rms = std::numeric_limits<double>::quiet_NaN();
	
	try
	{
		size_t pointCount = cloud.size();
		unsigned vertexCount = spline.size();
		if (pointCount == 0 || vertexCount < 2)
		{
			//not enough points or vertices
			ccLog::Warning("[qFitSpline::ComputeDistances] Not enough point or vertex");
			return false;
		}

		//check spline consistency
		if (vertexCount + spline.getOrder() != spline.nodes().size())
		{
			//malformed spline
			ccLog::Warning("[qFitSpline::ComputeDistances] Malformed spline");
			return false;
		}

		static const unsigned InitialSubdivision = 4;
		unsigned sampleSegCount = vertexCount * InitialSubdivision - 1;
		std::vector<Segment> initialSegments(sampleSegCount);
		double step = 1.0 / sampleSegCount;
		double s = 0.0;
		for (unsigned i = 0; i < sampleSegCount; ++i)
		{
			Segment& seg = initialSegments[i];
			seg.posA = s;
			if (i == 0)
			{
				if (!spline.computePosition(seg.posA, seg.A))
				{
					return false;
				}
			}
			else
			{
				seg.A = initialSegments[i - 1].B;
			}
			s += step;
			seg.posB = s;
			if (!spline.computePosition(seg.posB, seg.B))
			{
				return false;
			}
		}

		distances.resize(pointCount, std::numeric_limits<double>::quiet_NaN());

		double depthSum = 0.0;
		unsigned minDepth = 0;
		unsigned maxDepth = 0;
		rms = 0.0;
		for (unsigned iPt = 0; iPt < pointCount; ++iPt)
		{
			const CCVector3* P = cloud.getPoint(iPt);

			//find the first segment
			unsigned iClosestSeg = 0;
			double iClosestSegDist = std::numeric_limits<double>::quiet_NaN();
			for (unsigned iSeg = 0; iSeg < sampleSegCount; ++iSeg)
			{
				Segment& seg = initialSegments[iSeg];
				double dist = DistanceToSegment(*P, seg);
				if (std::isnan(iClosestSegDist) || dist < iClosestSegDist)
				{
					iClosestSegDist = dist;
					iClosestSeg = iSeg;
				}
			}

			//now start the recursion
			RecursiveSearch rs(spline, *P, iClosestSegDist, precision);

			rs.distanceToSegment(initialSegments[iClosestSeg]);

			distances[iPt] = rs.minDistance;

			rms += rs.minDistance * rs.minDistance;
			if (!silent)
			{
				if (iPt != 0)
				{
					if (rs.depth < minDepth)
					{
						minDepth = rs.depth;
					}
					else if (rs.depth > maxDepth)
					{
						maxDepth = rs.depth;
					}
				}
				else
				{
					minDepth = maxDepth = rs.depth;
				}
				depthSum += static_cast<double>(rs.depth);
			}
		}

		rms = sqrt(rms / pointCount);
		if (!silent)
		{
			ccLog::Print(QString("[qFitSpline::ComputeDistances] RMS = %1").arg(rms));
			double avgDepth = depthSum / pointCount;
			ccLog::Print(QString("[qFitSpline::ComputeDistances] Search depths in [%1 ; %2] (avg = %3)").arg(minDepth).arg(maxDepth).arg(avgDepth));
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}


	return true;
}

//! Levenberg Marquardt functor
template<int NX = Eigen::Dynamic, int NY = Eigen::Dynamic> struct LMFunctor
{
	typedef double Scalar;

	//! Eigen internal implementation
	enum
	{
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};

	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1>                   InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1>                   ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	//! Default constructor
	LMFunctor()
		: m_inputs(InputsAtCompileTime)
		, m_values(ValuesAtCompileTime)
	{
	}

	//! Constructor
	LMFunctor(int p_inputs, int p_values)
		: m_inputs(p_inputs)
		, m_values(p_values)
	{
	}

	int inputs() const
	{
		return m_inputs;
	}

	int values() const
	{
		return m_values;
	}

	virtual void initX(Eigen::VectorXd& x)
	{
		for (int i = 0; i < x.size(); ++i)
		{
			x(i) = 0.0;
		}
	}

	virtual void updateSpline(ccSpline& spline, const Eigen::VectorXd& x) const = 0;

	int m_inputs; //!< Number of inputs
	int m_values; //!< Number of values
};

struct FitSplineLMFunctor : LMFunctor<>
{
	//static const int ValueCount = ; //3D points (cloud)
	//static const int InputCount = ; //spline nodal values + 3D positions

	FitSplineLMFunctor(const ccSpline& spline,
		const ccPointCloud& cloud,
		double precision)
		: LMFunctor<>(static_cast<int>(spline.nodes().size() - 2 * spline.getOrder() + 3 * spline.size()), static_cast<int>(cloud.size()))
		, m_spline(spline)
		, m_cloud(cloud)
		, m_localSpline(spline)
		, m_distanceBuffer(cloud.size())
		, m_precision(precision)
	{
	}

	int operator()(const Eigen::VectorXd& p_x, Eigen::VectorXd& p_fvec) const
	{
		updateSpline(m_localSpline, p_x);

		size_t pointCount = m_cloud.size();
		assert(pointCount == p_fvec.size());

		double rms;
		if (false == ComputeDistances(	m_cloud,
										m_localSpline,
										m_distanceBuffer,
										rms,
										m_precision,
										true))
		{
			for (size_t i = 0; i < pointCount; ++i)
				p_fvec(i) = std::numeric_limits<float>::max();
			return -1;
		}

		for (size_t i = 0; i < pointCount; ++i)
		{
			double distToSpline = m_distanceBuffer[i];

			if (std::isfinite(distToSpline))
			{
				p_fvec(i) = distToSpline;
			}
			else
			{
				p_fvec(i) = std::numeric_limits<float>::max();
			}
		}

		return 0;
	}

	void updateSpline(ccSpline& spline, const Eigen::VectorXd& x) const override
	{
		size_t order = m_spline.getOrder();
		size_t nodeCount = m_spline.nodes().size() - 2 * order;
		size_t vertexCount = spline.size();
		assert(nodeCount + 3 * vertexCount == x.size());

		//load the new nodal parameters
		for (size_t i = 0; i < nodeCount; ++i)
		{
			double nodeVal = m_spline.nodes()[order + i] + x(i) / 100.0;
			//consistency check
			if (nodeVal < 0.0)
			{
				nodeVal = 0.0;
			}
			else if (nodeVal > 1.0)
			{
				nodeVal = 1.0;
			}
			if (i != 0 && nodeVal < spline.nodes()[order + i - 1])
			{
				nodeVal = spline.nodes()[order + i - 1];
			}
			spline.nodes()[order + i] = nodeVal;
		}
		//load the vertices positions
		size_t j = nodeCount;
		for (size_t i = 0; i < vertexCount; ++i)
		{
			const CCVector3* Po = m_spline.getPoint(static_cast<unsigned>(i));
			CCVector3* P = const_cast<CCVector3*>(spline.getPoint(static_cast<unsigned>(i)));
			P->x = Po->x + x(j++);
			P->y = Po->y + x(j++);
			P->z = Po->z + x(j++);
		}
	}

	const ccSpline& m_spline;
	const ccPointCloud& m_cloud;
	mutable ccSpline m_localSpline;
	mutable std::vector<double> m_distanceBuffer;
	double m_precision;
};

struct FitSplineLMFunctorNodes : LMFunctor<>
{
	//static const int ValueCount = ; //3D points (cloud)
	//static const int InputCount = ; //spline nodal values

	FitSplineLMFunctorNodes(const ccSpline& spline,
		const ccPointCloud& cloud,
		double precision)
		: LMFunctor<>(static_cast<int>(spline.nodes().size() - 2 * spline.getOrder()), static_cast<int>(cloud.size()))
		, m_spline(spline)
		, m_cloud(cloud)
		, m_localSpline(spline)
		, m_distanceBuffer(cloud.size())
		, m_precision(precision)
	{
	}

	int operator()(const Eigen::VectorXd& p_x, Eigen::VectorXd& p_fvec) const
	{
		updateSpline(m_localSpline, p_x);

		size_t pointCount = m_cloud.size();
		assert(pointCount == p_fvec.size());

		double rms;
		if (false == ComputeDistances(m_cloud,
			m_localSpline,
			m_distanceBuffer,
			rms,
			m_precision,
			true))
		{
			for (size_t i = 0; i < pointCount; ++i)
				p_fvec(i) = std::numeric_limits<float>::max();
			return -1;
		}

		for (size_t i = 0; i < pointCount; ++i)
		{
			double distToSpline = m_distanceBuffer[i];

			if (std::isfinite(distToSpline))
			{
				p_fvec(i) = distToSpline;
			}
			else
			{
				p_fvec(i) = std::numeric_limits<float>::max();
			}
		}

		return 0;
	}

	void updateSpline(ccSpline& spline, const Eigen::VectorXd& x) const override
	{
		size_t order = m_spline.getOrder();
		size_t nodeCount = m_spline.nodes().size() - 2 * order;
		assert(nodeCount == x.size());

		//load the new nodal parameters
		for (size_t i = 0; i < nodeCount; ++i)
		{
			double nodeVal = m_spline.nodes()[order + i] + x(i);
			//consistency check
			if (nodeVal < 0.0)
			{
				nodeVal = 0.0;
			}
			else if (nodeVal > 1.0)
			{
				nodeVal = 1.0;
			}
			if (i != 0 && nodeVal < spline.nodes()[order + i - 1])
			{
				nodeVal = spline.nodes()[order + i - 1];
			}
			spline.nodes()[order + i] = nodeVal;
		}
	}

	const ccSpline& m_spline;
	const ccPointCloud& m_cloud;
	mutable ccSpline m_localSpline;
	mutable std::vector<double> m_distanceBuffer;
	double m_precision;
};

struct FitSplineLMFunctorPos3D : LMFunctor<>
{
	//static const int ValueCount = ; //3D points (cloud)
	//static const int InputCount = ; //3D positions

	FitSplineLMFunctorPos3D(const ccSpline& spline,
		const ccPointCloud& cloud,
		double precision)
		: LMFunctor<>(static_cast<int>(3 * spline.size()), static_cast<int>(cloud.size()))
		, m_spline(spline)
		, m_cloud(cloud)
		, m_localSpline(spline)
		, m_distanceBuffer(cloud.size())
		, m_precision(precision)
	{
	}

	int operator()(const Eigen::VectorXd& p_x, Eigen::VectorXd& p_fvec) const
	{
		updateSpline(m_localSpline, p_x);

		size_t pointCount = m_cloud.size();
		assert(pointCount == p_fvec.size());

		double rms;
		if (false == ComputeDistances(m_cloud,
			m_localSpline,
			m_distanceBuffer,
			rms,
			m_precision,
			true))
		{
			for (size_t i = 0; i < pointCount; ++i)
				p_fvec(i) = std::numeric_limits<float>::max();
			return -1;
		}

		for (size_t i = 0; i < pointCount; ++i)
		{
			double distToSpline = m_distanceBuffer[i];

			if (std::isfinite(distToSpline))
			{
				p_fvec(i) = distToSpline;
			}
			else
			{
				p_fvec(i) = std::numeric_limits<float>::max();
			}
		}

		return 0;
	}

	void updateSpline(ccSpline& spline, const Eigen::VectorXd& x) const override
	{
		size_t vertexCount = spline.size();
		assert(3 * vertexCount == x.size());

		//load the vertices positions
		size_t j = 0;
		for (size_t i = 0; i < vertexCount; ++i)
		{
			const CCVector3* Po = m_spline.getPoint(static_cast<unsigned>(i));
			CCVector3* P = const_cast<CCVector3*>(spline.getPoint(static_cast<unsigned>(i)));
			P->x = Po->x + x(j++);
			P->y = Po->y + x(j++);
			P->z = Po->z + x(j++);
		}
	}

	const ccSpline& m_spline;
	const ccPointCloud& m_cloud;
	mutable ccSpline m_localSpline;
	mutable std::vector<double> m_distanceBuffer;
	double m_precision;
};

qFitSpline::qFitSpline(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qFitSpline/info.json" )
	, m_action( nullptr )
{
}

void qFitSpline::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		int cloudCount = 0;
		int polyLineOrSplineCount = 0;

		for (ccHObject* entity : selectedEntities)
		{
			if (!entity)
			{
				continue;
			}

			//if we have found at least one cloud
			if (entity->isA(CC_TYPES::POINT_CLOUD))
			{
				++cloudCount;
			}
			else if (entity->isKindOf(CC_TYPES::POLY_LINE))
			{
				++polyLineOrSplineCount;
			}
		}

		m_action->setEnabled(cloudCount == 1 && polyLineOrSplineCount == 1);
	}
}

QList<QAction *> qFitSpline::getActions()
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(),this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, &QAction::triggered, this, &qFitSpline::doAction);
	}

	return QList<QAction *>{ m_action };
}

void qFitSpline::doAction()
{
	if (!m_app)
	{
		assert(false);
		return;
	}

	if (!m_app->haveSelection())
	{
		assert(false);
		return;
	}

	ccPointCloud* cloud = nullptr;
	ccSpline* spline = nullptr;
	bool inputSpline = false;
	ccHObject* splineParent = nullptr;
	double precision = 1.0e-4;

	for (ccHObject* entity : m_app->getSelectedEntities())
	{
		if (!entity)
			continue;

		if (entity->isA(CC_TYPES::POINT_CLOUD))
		{
			if (cloud)
			{
				ccLog::Error("Only one cloud expected");
				return;
			}
			cloud = static_cast<ccPointCloud*>(entity);
		}
		else if (entity->isKindOf(CC_TYPES::POLY_LINE))
		{
			if (spline)
			{
				ccLog::Error("Only one spline or polyline expected");
				return;
			}

			if (entity->isA(CC_TYPES::POLY_LINE))
			{
				ccPolyline* poly = static_cast<ccPolyline*>(entity);
				spline = new ccSpline(*poly, 3, poly->isClosed() ? ccSpline::Uniform : ccSpline::OpenUniform);
				inputSpline = false;
			}
			else
			{
				spline = static_cast<ccSpline*>(entity);
				inputSpline = true;
			}
			splineParent = entity->getParent();
		}
	}

	//compute the distances before optimization
	{
		std::vector<double> distances;
		double rms;
		ccLog::Print("Computing spline/cloud distances... (before optimization)");
		if (!ComputeDistances(*cloud, *spline, distances, rms, precision, false))
		{
			ccLog::Error("Failed to compute distances (before optimization)");
			return;
		}
	}

	bool changePositionAndNodes = true;
	bool changePosition = false;
	bool changeNodes = false;

	//Lucky pass: change both the vertex positions and nodes
	if (changePositionAndNodes)
	{
		using CurrentLMFunctor = FitSplineLMFunctor;

		//LM routine functor
		CurrentLMFunctor functor(*spline, *cloud, precision);
		int inputCount = functor.inputs();

		//initial parameters for LM routine
		Eigen::VectorXd x(inputCount);
		functor.initX(x);

		//run LM routine
		Eigen::NumericalDiff<CurrentLMFunctor> numDiff(functor, precision);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CurrentLMFunctor>, double> lm(numDiff);

		//set LM algorithm convergence parameters
		lm.parameters.maxfev = 1000;
		//lm.parameters.ftol = 1.0e-8; //absolute convergence criteria
		//lm.parameters.xtol = 1.0e-6; //relative convergence criteria

		Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
		if (status <= 0)
		{
			ccLog::Error(QString("Levenberg Marquardt optimization failed to converge (status: %1)").arg(status));
			if (!inputSpline && spline)
				delete spline;
			return;
		}
		else
		{
			ccLog::Print(QString("Levenberg Marquardt optimization: iterations: %1 / func. evaluations: %2").arg(lm.iter).arg(lm.nfev));
		}

		//create the resulting spline (if not already)
		if (inputSpline)
		{
			//That's bold 8-o
			spline = new ccSpline(*spline);
			inputSpline = false;
		}

		//update the output spline
		functor.updateSpline(*spline, x);

		//compute the distances after optimization
		{
			std::vector<double> distances;
			double rms;
			ccLog::Print("Computing spline/cloud distances... (after optimization)");
			if (!ComputeDistances(*cloud, *spline, distances, rms, precision, false))
			{
				ccLog::Warning("Failed to compute distances (after optimization)");
				//return; //too late
			}
		}
	}

	//1st pass: change the vertex positions
	if (changePosition)
	{
		using CurrentLMFunctor = FitSplineLMFunctorPos3D;

		//LM routine functor
		CurrentLMFunctor functor(*spline, *cloud, precision);
		int inputCount = functor.inputs();

		//initial parameters for LM routine
		Eigen::VectorXd x(inputCount);
		functor.initX(x);

		//run LM routine
		Eigen::NumericalDiff<CurrentLMFunctor> numDiff(functor, precision);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CurrentLMFunctor>, double> lm(numDiff);

		//set LM algorithm convergence parameters
		lm.parameters.maxfev = 1000;
		//lm.parameters.ftol = 1.0e-8; //absolute convergence criteria
		//lm.parameters.xtol = 1.0e-6; //relative convergence criteria

		Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
		if (status <= 0)
		{
			ccLog::Error(QString("Levenberg Marquardt optimization failed to converge (status: %1)").arg(status));
			if (!inputSpline && spline)
				delete spline;
			return;
		}
		else
		{
			ccLog::Print(QString("Levenberg Marquardt optimization: iterations: %1 / func. evaluations: %2").arg(lm.iter).arg(lm.nfev));
		}

		//create the resulting spline (if not already)
		if (inputSpline)
		{
			//That's bold 8-o
			spline = new ccSpline(*spline);
			inputSpline = false;
		}

		//update the output spline
		functor.updateSpline(*spline, x);

		//compute the distances after optimization
		{
			std::vector<double> distances;
			double rms;
			ccLog::Print("Computing spline/cloud distances... (after optimization)");
			if (!ComputeDistances(*cloud, *spline, distances, rms, precision, false))
			{
				ccLog::Warning("Failed to compute distances (after optimization)");
				//return; //too late
			}
		}
	}

	//2nd pass: change the spline node values
	if (changeNodes)
	{
		using CurrentLMFunctor = FitSplineLMFunctorNodes;

		//LM routine functor
		CurrentLMFunctor functor(*spline, *cloud, 1.0e-5);
		int inputCount = functor.inputs();

		//initial parameters for LM routine
		Eigen::VectorXd x(inputCount);
		functor.initX(x);

		//run LM routine
		Eigen::NumericalDiff<CurrentLMFunctor> numDiff(functor, precision);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CurrentLMFunctor>, double> lm(numDiff);

		//set LM algorithm convergence parameters
		lm.parameters.maxfev = 1000;
		//lm.parameters.ftol = 1.0e-8; //absolute convergence criteria
		//lm.parameters.xtol = 1.0e-6; //relative convergence criteria

		Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
		if (status <= 0)
		{
			ccLog::Error(QString("Levenberg Marquardt optimization failed to converge (status: %1)").arg(status));
			if (!inputSpline && spline)
				delete spline;
			return;
		}
		else
		{
			ccLog::Print(QString("Levenberg Marquardt optimization: iterations: %1 / func. evaluations: %2").arg(lm.iter).arg(lm.nfev));
		}

		//create the resulting spline (if not already)
		if (inputSpline)
		{
			//That's bold 8-o
			spline = new ccSpline(*spline);
			inputSpline = false;
		}

		//update the output spline
		functor.updateSpline(*spline, x);

		//compute the distances after optimization
		{
			std::vector<double> distances;
			double rms;
			ccLog::Error("Computing spline/cloud distances... (after optimization)");
			if (!ComputeDistances(*cloud, *spline, distances, rms, precision, false))
			{
				ccLog::Warning("Failed to compute distances (after optimization)", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				//return; //too late
			}
		}
	}

#ifdef _DEBUG
	//show the final spline nodes
	for (size_t i = 0; i < spline->nodes().size(); ++i)
	{
		ccLog::Print(QString("Node %1 = %2").arg(i + 1).arg(spline->nodes()[i]));
	}
#endif

	if (splineParent)
	{
		splineParent->addChild(spline);
	}
	m_app->addToDB(spline);
}
