//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qGMMReg                       #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ccGMMRegWrapper.h"

//qCC_db
#include <ccHObjectCaster.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>

//qCC plugins
#include <ccMainAppInterface.h>

//GMMReg
#include <gmmreg_tps.h>
#include <gmmreg_grbf.h>

//Qt
#include <QtConcurrentRun>
#include <QMainWindow>
#include <QProgressDialog>
#include <QElapsedTimer>
#include <QCoreApplication>

//system
#if defined(CC_WINDOWS)
#include "windows.h"
#else
#include <time.h>
#include <unistd.h>
#endif

static bool CloudToVNLMatrix(CCLib::GenericCloud* cloud, vnl_matrix<double>& matrix, const CCVector3& C)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}

	unsigned pointCount = cloud->size();
	try
	{
		matrix.set_size(pointCount,3);
	}
	catch (...)
	{
		//not enough memory?
		return false;
	}

	cloud->placeIteratorAtBegining();
	for (unsigned i=0; i<pointCount; ++i)
	{
		CCVector3 P = *cloud->getNextPoint() - C;
		matrix(i,0) = P.x;
		matrix(i,1) = P.y;
		matrix(i,2) = P.z;
	}

	return true;
}

gmmreg::Base* s_reg = 0;
bool doPerformRegistration()
{
	if (s_reg)
	{
		return (s_reg->Run() >= 0);
	}
	else
	{
		assert(false);
		return false;
	}
}

bool ccGMMRegWrapper::PerformRegistration(	CCLib::GenericCloud* d,
											CCLib::GenericCloud* m,
											const std::vector<StepValues>& steps,
											std::vector<CCVector3>& displacedPoints,
											bool useTPS/*=true*/,
											size_t controlPointsCount/*=0*/,
											ccMainAppInterface* app/*=0*/,
											QWidget* parentWidget/*=0*/)
{
	if (!d || !m)
	{
		if (app)
			app->dispToConsole("Invalid input cloud(s)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		assert(false);
		return false;
	}

	size_t maxStepCount = steps.size();

	//check input parameters
	{
		if (maxStepCount > 3)
		{
			//too many steps
			if (app)
				app->dispToConsole("Too many steps defined (max: 3)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		size_t enabledSteps = 0;
		for (size_t i=0; i<maxStepCount; ++i)
		{
			if (steps[i].enabled)
				++enabledSteps;
		}

		if (enabledSteps == 0)
		{
			//nothing to do?!
			if (app)
				app->dispToConsole("No step enabled!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
	}

	//reserve memory for the output points
	try
	{
		displacedPoints.reserve(d->size());
	}
	catch (const std::bad_alloc&)
	{
		if (app)
			app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	//we can now prepare GMMReg
	gmmreg::TpsRegistration_L2 tpsReg;
	gmmreg::GrbfRegistration_L2 grbfReg;
	gmmreg::Base* reg = useTPS ? static_cast<gmmreg::Base*>(&tpsReg) : static_cast<gmmreg::Base*>(&grbfReg);

	CCVector3 C(0,0,0);
	{
		QProgressDialog progressDlg("Initialization in progress...", QString(), 0, 0, parentWidget ? parentWidget : app ? app->getMainWindow() : 0);
		progressDlg.setWindowTitle("Non-rigid registration");
		progressDlg.show();
		QCoreApplication::processEvents();

		QElapsedTimer timer;
		timer.start();

		ccBBox box;
		{
			//merged the data and model bounding-boxes
			CCVector3 bbMin, bbMax;
			d->getBoundingBox(bbMin, bbMax);
			box.add(bbMin);
			box.add(bbMax);
			m->getBoundingBox(bbMin, bbMax);
			box.add(bbMin);
			box.add(bbMax);
		}
		//C = box.getCenter();

		vnl_matrix<double> dataPts;
		if (!CloudToVNLMatrix(d, dataPts, C))
		{
			if (app)
				app->dispToConsole("Failed to convert data entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		vnl_matrix<double> modelPts;
		if (!CloudToVNLMatrix(m, modelPts, C))
		{
			if (app)
				app->dispToConsole("Failed to convert model entity to VNL matrix (not enough memory?)", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		vnl_matrix<double> ctrlPts;
		if (controlPointsCount != 0)
		{
			CCVector3 diag = box.getDiagVec();
			CCVector3 bbMin = box.minCorner();

			//try to get as close as possible with a regular grid
			PointCoordinateType l = pow((diag.x * diag.y * diag.z) / controlPointsCount, static_cast<PointCoordinateType>(1.0/3.0));
			unsigned nx = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.x / l)));
			unsigned ny = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.y / l)));
			unsigned nz = std::max<unsigned>(2,static_cast<unsigned>(ceil(diag.z / l)));
			unsigned realCount = nx * ny * nz;

			try
			{
				ctrlPts.set_size(realCount,3);
			}
			catch (...)
			{
				//not enough memory?
				if (app)
					app->dispToConsole("Not enough memory", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}

			ccPointCloud* test = 0;
#ifdef EXPORT_CONTROL_POINTS
			test = new ccPointCloud("grid");
			test->reserve(count);
#endif
			unsigned n = 0;
			for (unsigned k=0; k<nz; ++k)
			{
				CCVector3 P(0,0,bbMin.z + k * diag.z / (nz-1));
				for (unsigned j=0; j<ny; ++j)
				{
					P.y = bbMin.y + j * diag.y / (ny-1);
					for (unsigned i=0; i<nx; ++i, ++n)
					{
						P.x = bbMin.x + i * diag.x / (nx-1);

						assert(n < realCount);
						ctrlPts(n,0) = P.x;
						ctrlPts(n,1) = P.y;
						ctrlPts(n,2) = P.z;

#ifdef EXPORT_CONTROL_POINTS
						if (test)
							test->addPoint(P);
#endif
					}
				}
			}

#ifdef EXPORT_CONTROL_POINTS
			if (test)
			{
				if (app)
					app->addToDB(test);
				else
					delete test;
			}
#endif
		}
		
		//parameters
		int normalize = 1;
		
		unsigned level = 0; //<=3
		std::vector<float> v_scale(3);
		std::vector<int> v_func_evals(3);
		std::vector<float> v_lambda(3);
		std::vector<int> v_affine(3);

		for (size_t i=0; i<maxStepCount; ++i)
		{
			if (steps[i].enabled)
			{
				v_scale     [level] = static_cast<float>(steps[i].scale);
				v_func_evals[level] = steps[i].maxIter;
				v_lambda    [level] = static_cast<float>(steps[i].lambda);
				v_affine    [level] = 0;
				++level;
			}
		}

		assert(level >= 1 && level <= 3); //see previous tests

		//multi-scale options
		if (reg->MultiScaleOptions(level, v_scale, v_func_evals) < 0)
		{
			if (app)
				app->dispToConsole("Mutli-scale options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		if (reg == &tpsReg)
		{
			//TPS options
			if (tpsReg.PrepareOwnOptions(v_lambda, v_affine) < 0)
			{
				if (app)
					app->dispToConsole("TPS options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}
		}
		else if (reg == &grbfReg)
		{
			//GRBF options
			if (grbfReg.PrepareOwnOptions(v_lambda) < 0)
			{
				if (app)
					app->dispToConsole("GRBF options setting failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}
		}
  
		//initialization
		if (reg->Initialize(dataPts, modelPts, controlPointsCount != 0 ? &ctrlPts : 0, normalize) < 0)
		{
			if (app)
				app->dispToConsole("GMM initialization failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		qint64 intiTime_ms = timer.elapsed();
		if (app)
			app->dispToConsole(QString("[GMMReg] Initialization done (%1 msec)").arg(intiTime_ms), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		
		progressDlg.setLabelText("Registration in progress... please wait...");
		QCoreApplication::processEvents();

		try
		{
			//eventually we can run the registration (in a separate thread)
			s_reg = reg;
			QFuture<bool> future = QtConcurrent::run(doPerformRegistration);

			//wait until process is finished!
			while (!future.isFinished())
			{
#if defined(CC_WINDOWS)
				::Sleep(500);
#else
				usleep(500 * 1000);
#endif

				progressDlg.setValue(progressDlg.value()+1);
				QCoreApplication::processEvents();
			}

			s_reg = 0;

			if (!future.result())
			{
				if (app)
					app->dispToConsole("GMM registration failed", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return false;
			}
		}
		catch (...)
		{
			if (app)
				app->dispToConsole("Unknown exception caught", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}

		qint64 endTime_ms = timer.elapsed();
		if (app)
		{
			app->dispToConsole(QString("[GMMReg] Registration done (%1 msec)").arg(endTime_ms-intiTime_ms), ccMainAppInterface::STD_CONSOLE_MESSAGE);
			app->dispToConsole(QString("[GMMReg] Total time: %1 s.").arg(endTime_ms / 1000.0), ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
	}
	
	//prepare output points
	const vnl_matrix<double>& outputMatrix = reg->transformedModel();
	for (unsigned i = 0; i < outputMatrix.rows(); ++i)
	{
		CCVector3 P(static_cast<PointCoordinateType>(outputMatrix(i, 0)) + C.x,
					static_cast<PointCoordinateType>(outputMatrix(i, 1)) + C.y,
					static_cast<PointCoordinateType>(outputMatrix(i, 2)) + C.z);

		displacedPoints.push_back(P);
	}

	return true;
}
