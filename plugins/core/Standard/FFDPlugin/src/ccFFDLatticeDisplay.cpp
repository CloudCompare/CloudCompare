#include "ccFFDLatticeDisplay.h"

#include <ccColorTypes.h>

#include <QOpenGLFunctions_2_1>

ccFFDLatticeDisplay::ccFFDLatticeDisplay(const ccBBox& bbox,
	                                     const std::array<unsigned int, 3>& dims,
	                                     const std::vector<CCVector3d>& controlPoints)
	: ccHObject("FFD Lattice")
	, m_bbox(bbox)
	, m_dims(dims)
	, m_controlPoints(controlPoints)
{
	setVisible(true);
	setEnabled(true);
	updateBoundingBox();
}

ccBBox ccFFDLatticeDisplay::getOwnBB(bool /*withGLFeatures*/)
{
	return m_bbox;
}

void ccFFDLatticeDisplay::setControlPoints(const std::vector<CCVector3d>& controlPoints)
{
	m_controlPoints = controlPoints;
	updateBoundingBox();
}

void ccFFDLatticeDisplay::setSelectedIndices(const std::vector<int>& indices)
{
	m_selectedIndices = indices;
}

void ccFFDLatticeDisplay::updateBoundingBox()
{
	m_bbox.clear();
	for (const CCVector3d& cp : m_controlPoints)
	{
		m_bbox.add(CCVector3(static_cast<float>(cp.x), static_cast<float>(cp.y), static_cast<float>(cp.z)));
	}
}

const CCVector3d& ccFFDLatticeDisplay::getControlPoint(unsigned int x, unsigned int y, unsigned int z) const
{
	const unsigned int nx = m_dims[0];
	const unsigned int ny = m_dims[1];
	const unsigned int index = x + nx * (y + ny * z);
	return m_controlPoints[index];
}

void ccFFDLatticeDisplay::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
	{
		return;
	}

	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	if (!glFunc)
	{
		return;
	}

	if (m_controlPoints.empty())
	{
		return;
	}

	// Handle entity picking
	bool entityPickingMode = MACRO_EntityPicking(context);
	ccColor::Rgb pickingColor;
	if (entityPickingMode)
	{
		pickingColor = context.entityPicking.registerEntity(this);
	}

	const unsigned int nx = std::max(2u, m_dims[0]);
	const unsigned int ny = std::max(2u, m_dims[1]);
	const unsigned int nz = std::max(2u, m_dims[2]);

	glFunc->glPushAttrib(GL_LINE_BIT | GL_POINT_BIT | GL_COLOR_BUFFER_BIT | GL_ENABLE_BIT);
	glFunc->glDisable(GL_LIGHTING);
	glFunc->glLineWidth(1.0f);

	// Draw lattice grid lines (deforming with control points)
	if (entityPickingMode)
	{
		glFunc->glColor3ub(pickingColor.r, pickingColor.g, pickingColor.b);
	}
	else
	{
		glFunc->glColor3ub(180, 180, 180);
	}

	glFunc->glBegin(GL_LINES);

	// Lines along X
	for (unsigned int z = 0; z < nz; ++z)
	{
		for (unsigned int y = 0; y < ny; ++y)
		{
			for (unsigned int x = 0; x + 1 < nx; ++x)
			{
				const CCVector3d& p0 = getControlPoint(x, y, z);
				const CCVector3d& p1 = getControlPoint(x + 1, y, z);
				glFunc->glVertex3f(static_cast<float>(p0.x), static_cast<float>(p0.y), static_cast<float>(p0.z));
				glFunc->glVertex3f(static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z));
			}
		}
	}

	// Lines along Y
	for (unsigned int z = 0; z < nz; ++z)
	{
		for (unsigned int x = 0; x < nx; ++x)
		{
			for (unsigned int y = 0; y + 1 < ny; ++y)
			{
				const CCVector3d& p0 = getControlPoint(x, y, z);
				const CCVector3d& p1 = getControlPoint(x, y + 1, z);
				glFunc->glVertex3f(static_cast<float>(p0.x), static_cast<float>(p0.y), static_cast<float>(p0.z));
				glFunc->glVertex3f(static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z));
			}
		}
	}

	// Lines along Z
	for (unsigned int y = 0; y < ny; ++y)
	{
		for (unsigned int x = 0; x < nx; ++x)
		{
			for (unsigned int z = 0; z + 1 < nz; ++z)
			{
				const CCVector3d& p0 = getControlPoint(x, y, z);
				const CCVector3d& p1 = getControlPoint(x, y, z + 1);
				glFunc->glVertex3f(static_cast<float>(p0.x), static_cast<float>(p0.y), static_cast<float>(p0.z));
				glFunc->glVertex3f(static_cast<float>(p1.x), static_cast<float>(p1.y), static_cast<float>(p1.z));
			}
		}
	}

	glFunc->glEnd();

	// Draw control points
	glFunc->glPointSize(6.0f);
	if (entityPickingMode)
	{
		glFunc->glColor3ub(pickingColor.r, pickingColor.g, pickingColor.b);
	}
	else
	{
		glFunc->glColor3ub(255, 170, 0);
	}

	glFunc->glBegin(GL_POINTS);
	for (const CCVector3d& cp : m_controlPoints)
	{
		glFunc->glVertex3f(static_cast<float>(cp.x),
		                  static_cast<float>(cp.y),
		                  static_cast<float>(cp.z));
	}
	glFunc->glEnd();

	// Draw selected control points as highlighted spheres
	if (!m_selectedIndices.empty())
	{
		glFunc->glPointSize(12.0f);
		glFunc->glColor3ub(255, 128, 0); // Orange color for selected points

		glFunc->glBegin(GL_POINTS);
		for (int idx : m_selectedIndices)
		{
			if (idx >= 0 && idx < static_cast<int>(m_controlPoints.size()))
			{
				const CCVector3d& pt = m_controlPoints[idx];
				glFunc->glVertex3f(static_cast<float>(pt.x),
				                  static_cast<float>(pt.y),
				                  static_cast<float>(pt.z));
			}
		}
		glFunc->glEnd();
	}

	glFunc->glPopAttrib();
}
