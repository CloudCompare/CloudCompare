#include "../include/ccCloudLayersHelper.h"
#include "../include/ccMouseCircle.h"

//CC
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccMainAppInterface.h>

//QT
#include <QStringList>

//System
#include <thread>

ccCloudLayersHelper::ccCloudLayersHelper(ccMainAppInterface* app)
	: m_app ( app )
	, m_cloud( nullptr )
	, m_scalarFieldIndex(-1)
	, m_modified(false)
	, m_parameters{}
{
}

ccCloudLayersHelper::~ccCloudLayersHelper()
{
}

bool ccCloudLayersHelper::setCloud(ccPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return false;
	}
	m_cloud = cloud;

	// reserve some memory to project points in 2D
	try
	{
		m_projectedPoints.resize(m_cloud->size());
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error(QObject::tr("Not enough memory"));
		return false;
	}

	// backup visibility statuses
	m_originalCloudState.colorsWereShown = m_cloud->colorsShown();
	m_originalCloudState.sfWasShown = m_cloud->sfShown();
	m_originalCloudState.displayedSFIndex = m_cloud->getCurrentDisplayedScalarFieldIndex();
	m_originalCloudState.hadColors = m_cloud->hasColors();

	// backup colors
	if (m_originalCloudState.hadColors)
	{
		// store the original colors
		m_originalCloudState.colors.reset(m_cloud->rgbaColors()->clone());
		if (!m_originalCloudState.colors)
		{
			ccLog::Error(QObject::tr("Not enough memory to backup previous colors"));
			return false;
		}
	}
	else
	{
		// check memory for rgb colors
		if (!m_cloud->resizeTheRGBTable())
		{
			ccLog::Error(QObject::tr("Not enough memory"));
			return false;
		}
	}

	// backup the current scalar field
	if (m_cloud->hasScalarFields())
	{
		if (m_cloud->getCurrentDisplayedScalarFieldIndex() < 0)
		{
			// force the display of the first SF if none was active!
			m_cloud->setCurrentDisplayedScalarField(0);
		}

		if (!setScalarFieldIndexAndStoreValues(m_cloud->getCurrentDisplayedScalarFieldIndex()))
		{
			ccLog::Error(QObject::tr("Not enough memory"));
			return false;
		}
	}

	// we can eventually change the cloud state
	m_cloud->showColors(true);
	m_cloud->showSF(false);

	return true;
}

void ccCloudLayersHelper::keepCurrentSFVisible()
{
	if (m_cloud)
	{
		m_cloud->setCurrentDisplayedScalarField(m_scalarFieldIndex);
		if (m_cloud->getCurrentDisplayedScalarField())
		{
			m_cloud->getCurrentDisplayedScalarField()->computeMinAndMax();
		}
		m_cloud->showSF(true);
	}
}

void ccCloudLayersHelper::restoreCloud(bool restoreSFValues)
{
	if (!m_cloud)
	{
		return;
	}

	// restore the original colors (if any)
	if (m_originalCloudState.hadColors)
	{
		if (m_originalCloudState.colors)
		{
			// restore the saved colors
			if (m_cloud->rgbaColors())
			{
				// restore original colors
				m_originalCloudState.colors->copy(*m_cloud->rgbaColors());
				m_cloud->colorsHaveChanged();
			}
			else
			{
				assert(false);
			}
		}
	}
	else
	{
		m_cloud->unallocateColors();
	}

	// restore the color visibility status
	m_cloud->showColors(m_originalCloudState.colorsWereShown);

	if (restoreSFValues)
	{
		// restore the SF values
		restoreCurrentSFValues();

		// restore the SF visibility status (only in this case, as this means that the process has been cancelled)
		m_cloud->showSF(m_originalCloudState.sfWasShown);
		m_cloud->setCurrentDisplayedScalarField(m_originalCloudState.displayedSFIndex);
	}

	m_cloud->redrawDisplay();
}

QStringList ccCloudLayersHelper::getScalarFields()
{
	unsigned sfCount = m_cloud->getNumberOfScalarFields();
	QStringList scalarFields;
	if (m_cloud->hasScalarFields())
	{
		for (unsigned i = 0; i < sfCount; ++i)
		{
			scalarFields.append(QString::fromStdString(m_cloud->getScalarFieldName(i)));
		}
	}
	return scalarFields;
}

bool ccCloudLayersHelper::setScalarFieldIndexAndStoreValues(int index)
{
	if (saveCurrentSFValues(index))
	{
		m_scalarFieldIndex = index;
		m_modified = false;
		return true;
	}
	else
	{
		return false;
	}
}

void ccCloudLayersHelper::setVisible(bool value)
{
	unsigned pointCount = m_cloud->size();
	for (unsigned i = 0; i < pointCount; ++i)
	{
		ccColor::Rgba color = m_cloud->getPointColor(i);
		color.a = value ? ccColor::MAX : 0;
		m_cloud->setPointColor(i, color);
	}

	m_cloud->redrawDisplay();
}

void ccCloudLayersHelper::applyClassColors(QList<ccAsprsModel::AsprsItem>& items)
{
	m_cloud->setColor(ccColor::black);

	for (ccAsprsModel::AsprsItem& item : items)
	{
		item.count = applyClassColor(item);
	}
	
	m_cloud->redrawDisplay();
}

int ccCloudLayersHelper::applyClassColor(ccAsprsModel::AsprsItem& item, bool redrawDisplay/*=false*/)
{
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
	{
		return 0;
	}

	ccColor::Rgba ccColor = ccColor::FromQColora(item.color);
	ccColor.a = item.visible ? ccColor::MAX : 0;

	ScalarType code = static_cast<ScalarType>(item.code);
	int affected = 0;
	for (size_t i = 0; i < sf->size(); ++i)
	{
		if (sf->getValue(i) == code)
		{
			m_cloud->setPointColor(static_cast<unsigned>(i), ccColor);
			++affected;
		}
	}

	if (redrawDisplay)
	{
		m_cloud->redrawDisplay();
	}

	return affected;
}

void ccCloudLayersHelper::changeCode(const ccAsprsModel::AsprsItem& item, ScalarType oldCode)
{
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
	{
		return;
	}

	ScalarType code = static_cast<ScalarType>(item.code);
	for (size_t i = 0; i < sf->size(); ++i)
	{
		if (sf->getValue(i) == oldCode)
		{
			sf->setValue(static_cast<unsigned>(i), code);
		}
	}
}

int ccCloudLayersHelper::moveItem(const ccAsprsModel::AsprsItem& from, const ccAsprsModel::AsprsItem* to, bool redrawDisplay)
{
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
	{
		return 0;
	}

	ScalarType code = static_cast<ScalarType>(from.code);
	ScalarType emptyCode = static_cast<ScalarType>(nullptr != to ? to->code : 0);
	ccColor::Rgba color = (nullptr != to ? ccColor::FromQColora(to->color) : ccColor::black);

	int affected = 0;
	for (size_t i = 0; i < sf->size(); ++i)
	{
		if (sf->getValue(i) == code)
		{
			sf->setValue(static_cast<unsigned>(i), emptyCode);
			m_cloud->setPointColor(static_cast<unsigned>(i), color);
			++affected;
		}
	}

	if (redrawDisplay)
	{
		m_cloud->redrawDisplay();
	}

	return affected;
}

bool ccCloudLayersHelper::saveCurrentSFValues(int sfIndex)
{
	if (sfIndex < 0)
	{
		m_originalCloudState.scalarValues.clear();
		return true;
	}

	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(sfIndex);
	if (!sf)
	{
		return false;
	}

	unsigned cloudSize = m_cloud->size();
	try
	{
		
		m_originalCloudState.scalarValues.resize(cloudSize);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}

	for (unsigned i = 0; i < cloudSize; ++i)
	{
		m_originalCloudState.scalarValues[i] = sf->getValue(i);
	}

	return true;
}

void ccCloudLayersHelper::restoreCurrentSFValues()
{
	if (m_scalarFieldIndex < 0)
	{
		// nothing to do
		return;
	}

	CCCoreLib::ScalarField* sf = (nullptr != m_cloud ? m_cloud->getScalarField(m_scalarFieldIndex) : nullptr);
	if (!sf)
	{
		assert(false);
		return;
	}

	if (m_originalCloudState.scalarValues.size() != sf->size())
	{
		assert(false);
		return;
	}

	for (size_t i = 0; i < m_originalCloudState.scalarValues.size(); ++i)
	{
		sf->setValue(i, m_originalCloudState.scalarValues[i]);
	}
	sf->computeMinAndMax();
}

void ccCloudLayersHelper::project(const ccGLCameraParameters& camera, unsigned start, unsigned end)
{
	const double halfW = camera.viewport[2] / 2.0;
	const double halfH = camera.viewport[3] / 2.0;

	CCVector3d Q2D;
	bool pointInFrustum = false;
	for (unsigned i = start; i < end; ++i)
	{
		const CCVector3* P3D = m_cloud->getPoint(i);
		camera.project(*P3D, Q2D, &pointInFrustum);
		m_projectedPoints[i] = { CCVector2(static_cast<PointCoordinateType>(Q2D.x - halfW), static_cast<PointCoordinateType>(Q2D.y - halfH)), pointInFrustum };
	}
}

void ccCloudLayersHelper::mouseMove(const CCVector2& center2D, PointCoordinateType squareDist, std::map<ScalarType, int>& affected)
{
	if ( m_parameters.output == nullptr
		|| ((!m_parameters.anyPoints && !m_parameters.visiblePoints) && m_parameters.input == nullptr)
		)
	{
		return;
	}

	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
	{
		return;
	}

	ScalarType inputCode = static_cast<ScalarType>(nullptr != m_parameters.input ? m_parameters.input->code : 0);
	ScalarType outputCode = static_cast<ScalarType>(m_parameters.output->code);

	unsigned char alpha = (m_parameters.output->visible ? ccColor::MAX : 0);
	ccColor::Rgba outputColor = ccColor::Rgba(ccColor::FromQColor(m_parameters.output->color), alpha);

	unsigned cloudSize = m_cloud->size();
	for (unsigned i = 0; i < cloudSize; ++i)
	{
		// skip points outside of the frustum
		if (!m_projectedPoints[i].inFrustum)
		{
			continue;
		}

		// skip invisible points
		if (m_parameters.visiblePoints)
		{
			const auto& color = m_cloud->getPointColor(i);
			if (color.a != ccColor::MAX)
			{
				continue;
			}
		}

		ScalarType currentCode = sf->getValue(i);

		if (currentCode == outputCode)
		{
			// point already has the right code/class
			continue;
		}


		// if a specific code/class was input, skip the other codes/classes
		if (m_parameters.input && currentCode != inputCode)
		{
			continue;
		}

		// skip the points outside of the circle
		if ((m_projectedPoints[i].pos2D - center2D).norm2() > squareDist)
		{
			continue;
		}
		
		sf->setValue(i, outputCode);
		m_cloud->setPointColor(i, outputColor);

		--affected[currentCode];
		++affected[outputCode];

		m_modified = true;
	}
	
	m_cloud->redrawDisplay();
}

bool ccCloudLayersHelper::projectCloud(const ccGLCameraParameters& camera)
{
	// check if any camera parameter has changed (else, no need to reproject the points)
	if (m_cameraParameters == camera)
	{
		return true;
	}

	m_cameraParameters = camera;
	unsigned cloudSize = m_cloud->size();

	unsigned processorCount = std::thread::hardware_concurrency();
	if (processorCount == 0)
	{
		processorCount = 1;
	}

	const size_t chunkSize = cloudSize / processorCount;
	std::vector<std::thread*> threads;
	try
	{
		threads.resize(processorCount, nullptr);
	}
	catch (const::std::bad_alloc&)
	{
		ccLog::Warning(QObject::tr("Not enough memory to project the points"));
		return false;
	}

	for (unsigned i = 0; i < processorCount; ++i)
	{
		size_t start = i * chunkSize;
		size_t end = (i + 1 == processorCount ? cloudSize : start + chunkSize);

		threads[i] = new std::thread(&ccCloudLayersHelper::project, this, camera, start, end);
	}

	for (std::thread* thread : threads)
	{
		thread->join();
	}

	for (std::thread*& thread : threads)
	{
		delete thread;
		thread = nullptr;
	}

	return true;
}

ccCloudLayersHelper::Parameters& ccCloudLayersHelper::getParameters()
{
	return m_parameters;
}
