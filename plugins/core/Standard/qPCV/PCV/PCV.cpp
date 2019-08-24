//##########################################################################
//#                                                                        #
//#                                PCV                                     #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "PCV.h"
#include "PCVContext.h"

//Qt
#include <QString>

#ifdef USE_VLD
//VLD
#include <vld.h>
#endif

//System
#include <algorithm>
#include <cassert>
#include <cstring>

using namespace CCLib;

static int gcd(int num1, int num2)
{
	int remainder = (num2 % num1);

	return (remainder != 0 ? gcd(remainder, num1) : num1);
}

//! Sample points on the unit sphere
/** Transcripted from MATLAB's script "partsphere.m" by Paul Leopardi,
	2003-10-13, for UNSW School of Mathematics.

	As points are sampled on the unit sphere, they can also
	be considered as directions.
	WARNING: returned array is on the user responsibilty!

	\param N number of desired sampled directions
	\param[out] dirs set of N points
	\return success
**/
static bool SampleSphere(unsigned N, std::vector<CCVector3>& dirs)
{
	static const double c_eps = 2.2204e-16;
	static const double c_twist = 4.0;

	if (N == 0)
	{
		assert(false);
		return false;
	}

	try
	{
		dirs.resize(N, CCVector3(0, 0, 1));
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	if (N == 1)
	{
		return true;
	}

	try
	{
		double area = (4 * M_PI) / N;
		double beta = acos(1.0 - 2.0 / N); //return in [0,pi/2] as '1-2/N' goes from 0 to 1 when N --> inf
		double gamma = M_PI - 2 * beta; //in [0,pi]
		double fuzz = c_eps * 2 * N;

		int Ltemp = static_cast<int>(ceil(gamma / sqrt(area) - fuzz));
		int L = 2 + std::max(Ltemp, 1);

		//init mbar
		std::vector<double> mbar;
		mbar.resize(L, 0);
		assert(L >= 3);
		{
			mbar[0] = 1.0;
			double theta = gamma / (L - 2);
			for (int i = 1; i < L - 1; ++i)
			{
				mbar[i] = N * (cos(theta * (i - 1) + beta) - cos(theta * i + beta)) / 2;
			}
			mbar[L - 1] = 1.0;
		}

		//init m
		std::vector<int> m;
		m.resize(L, 0);
		{
			m[0] = 1;

			double alpha = 0.0;
			for (int i = 1; i < L; ++i)
			{
				double f = floor(mbar[i] + alpha + fuzz);
				if ((mbar[i] - f) >= 0.5)
					f = ceil(mbar[i] + alpha - fuzz);
				m[i] = static_cast<int>(f);

				alpha += mbar[i] - m[i];
			}
		}

		//now we can build the rays
		{
			std::vector<double> offset;
			offset.resize(L - 1, 0);

			double z = 1.0 - static_cast<double>(2 + m[1]) / N;

			unsigned int rayIndex = 1; //the first one is already (0,0,1)
			for (int i = 1; i < L - 1; ++i)
			{
				if (m[i - 1] != 0 && m[i] != 0)
				{
					offset[i] = offset[i - 1]
						+ static_cast<double>(gcd(m[i], m[i - 1])) / (2 * m[i] * m[i - 1])
						+ std::min<double>(c_twist, floor(m[i - 1] / c_twist)) / m[i - 1];
				}
				else
				{
					offset[i] = 0.0;
				}

				double temp = static_cast<double>(m[i]) / N;

				double h = cos((acos(z + temp) + acos(z - temp)) / 2);
				double r = sqrt(1.0 - h*h);

				for (int j = 0; j < m[i]; ++j)
				{
					double theta = 2.0*M_PI * (offset[i] + static_cast<double>(j) / m[i]);

					dirs[rayIndex++] = CCVector3(static_cast<PointCoordinateType>(r*cos(theta)),
						static_cast<PointCoordinateType>(r*sin(theta)),
						static_cast<PointCoordinateType>(h));
				}

				z -= static_cast<double>(m[i] + m[i + 1]) / N;
			}

			assert(rayIndex + 1 == N);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	dirs[N - 1] = CCVector3(0, 0, -1);

	return true;
}

bool PCV::GenerateRays(unsigned numberOfRays, std::vector<CCVector3>& rays, bool mode360/*=true*/)
{
	//generates light directions
	unsigned rayCount = numberOfRays * (mode360 ? 1 : 2);
	if (!SampleSphere(rayCount, rays))
	{
		return false;
	}

	//we keep only the light directions that meets input parameters (non predictible if not in 360° mode!)
	if (!mode360)
	{
		unsigned lastIndex = 0;
		for (size_t i = 0; i < rays.size(); ++i)
		{
			if (rays[i].z < 0)
			{
				if (lastIndex != i)
				{
					rays[lastIndex] = rays[i];
				}
				++lastIndex;
			}
		}
		rayCount = lastIndex;
		rays.resize(rayCount);
	}

	return true;
}

int PCV::Launch(unsigned numberOfRays,
				GenericCloud* vertices,
				GenericMesh* mesh/*=0*/,
				bool meshIsClosed/*=false*/,
				bool mode360/*=true*/,
				unsigned width/*=1024*/,
				unsigned height/*=1024*/,
				CCLib::GenericProgressCallback* progressCb/*=0*/,
				const QString& entityName/*=QString()*/)
{
	//generates light directions
	std::vector<CCVector3> rays;
	if (!GenerateRays(numberOfRays, rays, mode360))
	{
		return -2;
	}

	if (!Launch(rays, vertices, mesh, meshIsClosed, width, height, progressCb, entityName))
	{
		return -1;
	}

	return static_cast<int>(rays.size());
}

bool PCV::Launch(const std::vector<CCVector3>& rays,
				 CCLib::GenericCloud* vertices,
				 CCLib::GenericMesh* mesh/*=0*/,
				 bool meshIsClosed/*=false*/,
				 unsigned width/*=1024*/,
				 unsigned height/*=1024*/,
				 CCLib::GenericProgressCallback* progressCb/*=0*/,
				 const QString& entityName/*=QString()*/)
{
	if (rays.empty())
		return false;

	if (!vertices || !vertices->enableScalarField())
		return false;

	//vertices/points
	unsigned numberOfPoints = vertices->size();
	//rays
	unsigned numberOfRays = static_cast<unsigned>(rays.size());

	//for each vertex we keep count of the number of light directions for which it is "illuminated"
	std::vector<int> visibilityCount;
	try
	{
		visibilityCount.resize(numberOfPoints, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory?
		return false;
	}

	/*** Main illumination loop ***/

	CCLib::NormalizedProgress nProgress(progressCb, numberOfRays);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setMethodTitle("ShadeVis");
			QString infoStr;
			if (!entityName.isEmpty())
				infoStr = entityName + "\n";
			infoStr.append(QString("Rays: %1").arg(numberOfRays));
			if (mesh)
				infoStr.append(QString("\nFaces: %1").arg(mesh->size()));
			else
				infoStr.append(QString("\nVertices: %1").arg(numberOfPoints));
			progressCb->setInfo(qPrintable(infoStr));
		}
		progressCb->update(0);
		progressCb->start();
	}

	bool success = true;

	//must be done after progress dialog display!
	PCVContext win;
	if (win.init(width, height, vertices, mesh, meshIsClosed))
	{
		for (unsigned i = 0; i < numberOfRays; ++i)
		{
			//set current 'light' direction
			win.setViewDirection(rays[i]);

			//flag viewed vertices
			win.GLAccumPixel(visibilityCount);

			if (progressCb && !nProgress.oneStep())
			{
				success = false;
				break;
			}
		}

		if (success)
		{
			//we convert per-vertex accumulators to an 'intensity' scalar field
			for (unsigned j = 0; j < numberOfPoints; ++j)
			{
				ScalarType visValue = static_cast<ScalarType>(visibilityCount[j]) / numberOfRays;
				vertices->setPointScalarValue(j, visValue);
			}
		}
	}
	else
	{
		success = false;
	}

	return success;
}
