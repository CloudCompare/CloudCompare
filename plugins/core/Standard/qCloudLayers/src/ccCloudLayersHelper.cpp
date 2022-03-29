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

ccCloudLayersHelper::ccCloudLayersHelper(ccMainAppInterface* app, ccPointCloud* cloud)
	: m_app ( app )
	, m_cloud( cloud )
	, m_formerCloudColors( nullptr )
	, m_formerCloudColorsWereShown( false )
	, m_formerCloudSFWasShown( false )
	, m_scalarFieldIndex( 0 )
	, m_modified( false )
	, m_parameters{}
{
	m_projectedPoints.resize(m_cloud->size());
	m_pointInFrustum.resize(m_cloud->size());

	if (m_cloud)
	{
		m_formerCloudColorsWereShown = m_cloud->colorsShown();
		m_formerCloudSFWasShown = m_cloud->sfShown();

		if (m_cloud->hasColors())
		{
			// store the original colors
			m_formerCloudColors = m_cloud->rgbaColors()->clone();
			if (!m_formerCloudColors)
			{
				ccLog::Error("Not enough memory to backup previous colors");
			}
		}
		else
		{
			// check memory for rgb colors
			if (!m_cloud->resizeTheRGBTable())
			{
				ccLog::Error("Not enough memory to show colors");
			}
		}

		cloud->showColors(true);
		cloud->showSF(false);
	}
}

ccCloudLayersHelper::~ccCloudLayersHelper()
{
	if (m_cloud)
	{
		if (m_formerCloudColors)
		{
			if (m_cloud->rgbaColors())
			{
				// restore original colors
				m_formerCloudColors->copy(*m_cloud->rgbaColors());
			}

			delete m_formerCloudColors;
			m_formerCloudColors = nullptr;
		}
		else
		{
			m_cloud->unallocateColors();
		}
		
		m_cloud->showColors(m_formerCloudColorsWereShown);
		m_cloud->showSF(m_formerCloudSFWasShown);
		m_cloud->redrawDisplay();
	}
	m_cloud = nullptr;
}

QStringList ccCloudLayersHelper::getScalarFields()
{
	unsigned sfCount = m_cloud->getNumberOfScalarFields();
	QStringList scalarFields;
	if (m_cloud->hasScalarFields())
	{
		for (unsigned i = 0; i < sfCount; ++i)
		{
			scalarFields.append(QString(m_cloud->getScalarFieldName(i)));
		}
	}
	return scalarFields;
}

void ccCloudLayersHelper::setScalarFieldIndex(int index)
{
	m_scalarFieldIndex = index;
}

void ccCloudLayersHelper::keepCurrentSFVisible()
{
	m_formerCloudSFWasShown = true;
	m_cloud->setCurrentDisplayedScalarField(m_scalarFieldIndex);
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

void ccCloudLayersHelper::apply(QList<ccAsprsModel::AsprsItem>& items)
{
	if (m_scalarFieldIndex >= m_cloud->getNumberOfScalarFields())
		return;

	m_cloud->setColor(ccColor::black);

	for (int i = 0; i < items.size(); ++i)
	{
		items[i].count = apply(items[i]);
	}
	
	m_cloud->redrawDisplay();
}

int ccCloudLayersHelper::apply(ccAsprsModel::AsprsItem& item, bool redrawDisplay)
{
	ccColor::Rgba ccColor = ccColor::FromQColora(item.color);
	ccColor.a = item.visible ? ccColor::MAX : 0;
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
		return 0;

	ScalarType code = static_cast<ScalarType>(item.code);
	int affected = 0;
	int counter = 0;
	for (auto it = sf->begin(); it != sf->end(); ++it, ++counter)
	{
		if ((*it) == code)
		{
			m_cloud->setPointColor(counter, ccColor);
			++affected;
		}
	}

	if (redrawDisplay)
		m_cloud->redrawDisplay();

	return affected;
}

void ccCloudLayersHelper::changeCode(const ccAsprsModel::AsprsItem& item, ScalarType oldCode)
{
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
		return;

	ScalarType code = static_cast<ScalarType>(item.code);
	int counter = 0;
	for (auto it = sf->begin(); it != sf->end(); ++it, ++counter)
	{
		if ((*it) == oldCode)
		{
			sf->setValue(counter, code);
		}
	}
}

int ccCloudLayersHelper::moveItem(const ccAsprsModel::AsprsItem& from, const ccAsprsModel::AsprsItem* to, bool redrawDisplay)
{
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
		return 0;

	ScalarType code = static_cast<ScalarType>(from.code);
	ScalarType emptyCode = to != nullptr ? static_cast<ScalarType>(to->code) : static_cast<ScalarType>(0);
	const ccColor::Rgba color = to != nullptr ? ccColor::FromQColora(to->color) : ccColor::black;

	int affected = 0;
	int counter = 0;
	for (auto it = sf->begin(); it != sf->end(); ++it, ++counter)
	{
		if ((*it) == code)
		{
			sf->setValue(counter, emptyCode);
			m_cloud->setPointColor(counter, color);
			++affected;
		}
	}

	if (redrawDisplay)
		m_cloud->redrawDisplay();

	return affected;
}

void ccCloudLayersHelper::saveState()
{
	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
		return;

	unsigned cloudSize = m_cloud->size();
	m_cloudState.resize(cloudSize);
	for (unsigned i = 0; i < cloudSize; ++i)
	{
		m_cloudState[i].update(sf->getValue(i), m_cloud->getPointColor(i));
	}
}

void ccCloudLayersHelper::restoreState()
{
	CCCoreLib::ScalarField* sf = (m_cloud ? m_cloud->getScalarField(m_scalarFieldIndex) : nullptr);
	if (!sf)
	{
		assert(false);
		return;
	}

	unsigned cloudSize = m_cloud->size();
	if (m_cloudState.size() != cloudSize)
	{
		assert(false);
		return;
	}

	for (unsigned i = 0; i < cloudSize; ++i)
	{
		const CloudState& state = m_cloudState[i];
		sf->setValue(i, state.code);
		m_cloud->setPointColor(i, state.color);
	}
}

void ccCloudLayersHelper::project(ccGLCameraParameters camera, unsigned start, unsigned end)
{
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	CCVector3d Q2D;
	bool pointInFrustum = false;
	for (unsigned i = start; i < end; ++i)
	{
		const CCVector3* P3D = m_cloud->getPoint(i);
		camera.project(*P3D, Q2D, &pointInFrustum);
		m_projectedPoints[i] = CCVector2(static_cast<PointCoordinateType>(Q2D.x - half_w), static_cast<PointCoordinateType>(Q2D.y - half_h));
		m_pointInFrustum[i] = pointInFrustum;
	}
}

PointCoordinateType ccCloudLayersHelper::ComputeSquaredEuclideanDistance(const CCVector2& a, const CCVector2& b)
{
	return (b - a).norm2();
}

void ccCloudLayersHelper::mouseMove(const CCVector2& center, float squareDist, std::map<ScalarType, int>& affected)
{
	if (m_parameters.output == nullptr ||
		((!m_parameters.anyPoints && !m_parameters.visiblePoints) && m_parameters.input == nullptr))
	{
		return;
	}

	CCCoreLib::ScalarField* sf = m_cloud->getScalarField(m_scalarFieldIndex);
	if (!sf)
		return;

	ScalarType inputCode = m_parameters.input != nullptr ? static_cast<ScalarType>(m_parameters.input->code) : 0;
	ScalarType outputCode = static_cast<ScalarType>(m_parameters.output->code);

	unsigned char alpha = m_parameters.output->visible ? ccColor::MAX : 0;
	ccColor::Rgba outputColor = ccColor::Rgba(ccColor::FromQColor(m_parameters.output->color), alpha);

	unsigned cloudSize = m_cloud->size();
	for (unsigned i = 0; i < cloudSize; ++i)
	{
		// skip camera outside point
		if (!m_pointInFrustum[i])
			continue;

		const auto& color = m_cloud->getPointColor(i);

		// skip invisible points
		if (m_parameters.visiblePoints && color.a != ccColor::MAX)
			continue;

		ScalarType code = sf->getValue(i);

		// skip other codes
		if (m_parameters.input && code != inputCode)
			continue;

		// skip circle outside point
		if (ComputeSquaredEuclideanDistance(center, m_projectedPoints[i]) > squareDist)
			continue;
		
		if (code != outputCode)
		{
			sf->setValue(i, outputCode);
			m_cloud->setPointColor(i, outputColor);

			--affected[code];
			++affected[outputCode];
		}
	
		m_modified = true;
	}
	
	m_cloud->redrawDisplay();
}

void ccCloudLayersHelper::projectCloud(const ccGLCameraParameters& camera)
{
	// check camera parameters changes
	bool hasChanges = false;
	auto a = m_cameraParameters.modelViewMat.data();
	auto b = camera.modelViewMat.data();
	for (int i = 0; i < OPENGL_MATRIX_SIZE; ++i)
	{
		if (std::abs(a[i] - b[i]) > 1e-6)
		{
			hasChanges = true;
			break;
		}
	}

	if (!hasChanges)
		return;

	m_cameraParameters = camera;
	unsigned cloudSize = m_cloud->size();

	unsigned processorCount = std::thread::hardware_concurrency();
	if (processorCount == 0)
		processorCount = 1;

	const size_t part_size = cloudSize / processorCount;
	std::vector<std::thread*> threads;
	threads.resize(processorCount, nullptr);
	for (unsigned i = 0; i < processorCount; ++i)
	{
		size_t start = i * part_size;
		size_t end = start + part_size;

		if (i == processorCount - 1)
			end = cloudSize;

		threads[i] = new std::thread(&ccCloudLayersHelper::project, this, camera, start, end);
	}

	for (auto it = threads.begin(); it != threads.end(); ++it)
		(*it)->join();

	for (auto it = threads.begin(); it != threads.end(); ++it)
		delete (*it);
}

ccCloudLayersHelper::Parameters& ccCloudLayersHelper::getParameters()
{
	return m_parameters;
}
