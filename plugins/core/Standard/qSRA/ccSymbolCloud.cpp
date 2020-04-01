#include "ccSymbolCloud.h"

//qCC_db
#include <ccGenericGLDisplay.h>

ccSymbolCloud::ccSymbolCloud(QString name/*=QString()*/)
	: ccPointCloud(name)
	, m_symbolSize(10.0)
	, m_fontSize(12)
	, m_showSymbols(true)
	, m_showLabels(true)
	, m_labelAlignFlags(ccGenericGLDisplay::ALIGN_HMIDDLE | ccGenericGLDisplay::ALIGN_VBOTTOM)
{
}

bool ccSymbolCloud::reserve(unsigned numberOfPoints)
{
	if (!ccPointCloud::reserve(numberOfPoints))
		return false;
	
	if (m_labels.size())
		return reserveLabelArray(numberOfPoints);
	
	return true;
}

bool ccSymbolCloud::resize(unsigned numberOfPoints)
{
	if (!ccPointCloud::resize(numberOfPoints))
		return false;
	
	if (m_labels.size())
		return resizeLabelArray(numberOfPoints);
	
	return true;
}

bool ccSymbolCloud::reserveLabelArray(unsigned count)
{
	try
	{
		m_labels.reserve(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

void ccSymbolCloud::addLabel(QString label)
{
	try
	{
		m_labels.push_back(label);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		//TODO?
	}
}

bool ccSymbolCloud::resizeLabelArray(unsigned count)
{
	try
	{
		m_labels.resize(count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	return true;
}

void ccSymbolCloud::setLabel(unsigned index, QString label)
{
	if (m_labels.size() > index)
	{
		m_labels[index] = label;
	}
}

QString ccSymbolCloud::getLabel(unsigned index) const
{
	if (m_labels.size() > index)
	{
		return m_labels[index];
	}
	
	return QString();
}

void ccSymbolCloud::clearLabelArray()
{
	m_labels.clear();
}

void ccSymbolCloud::clear()
{
	ccPointCloud::clear();

	clearLabelArray();
}

template <class QOpenGLFunctions> void drawSymbolAt(QOpenGLFunctions* glFunc, double xp, double yp, double symbolRadius)
{
	if (!glFunc)
	{
		assert(false);
		return;
	}
	//diamong with cross
	glFunc->glBegin(GL_LINES);
	glFunc->glVertex2d(xp, yp - symbolRadius);
	glFunc->glVertex2d(xp, yp + symbolRadius);
	glFunc->glVertex2d(xp - symbolRadius, yp);
	glFunc->glVertex2d(xp + symbolRadius, yp);
	glFunc->glEnd();
	glFunc->glBegin(GL_LINE_LOOP);
	glFunc->glVertex2d(xp, yp - symbolRadius);
	glFunc->glVertex2d(xp + symbolRadius, yp);
	glFunc->glVertex2d(xp, yp + symbolRadius);
	glFunc->glVertex2d(xp - symbolRadius, yp);
	glFunc->glEnd();
}

void ccSymbolCloud::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (m_points.empty())
		return;

	//nothing to do?!
	if (!m_showSymbols && !m_showLabels)
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (MACRO_Draw3D(context))
	{
		//store the 3D camera parameters as we will need them for the 2D pass
		//(and we need the real ones, especially if the rendering zoom is != 1)
		context.display->getGLCameraParameters(m_lastCameraParams);
	}

	if (MACRO_Draw2D(context) && MACRO_Foreground(context))
	{
		//we get display parameters
		glDrawParams glParams;
		getDrawingParameters(glParams);

		//standard case: list names pushing
		bool pushName = MACRO_DrawEntityNames(context);
		bool hasLabels = !m_labels.empty();
		if (pushName)
		{
			//not fast at all!
			if (MACRO_DrawFastNamesOnly(context))
				return;

			glFunc->glPushName(getUniqueIDForDisplay());
			hasLabels = false; //no need to display labels in 'picking' mode
		}

		//we should already be in orthoprojective & centered omde
		//glFunc->glOrtho(-halfW, halfW, -halfH, halfH, -maxS, maxS);

		//default color
		const ccColor::Rgba* color = &context.pointsDefaultCol;
		if (isColorOverriden())
		{
			color = &m_tempColor;
			glParams.showColors = false;
		}
		
		unsigned numberOfPoints = size();

		//viewport parameters (will be used to project 3D positions to 2D)
		//ccGLCameraParameters camera;
		//context.display->getGLCameraParameters(camera);

		//only useful when displaying labels!
		QFont font(context.display->getTextDisplayFont()); //takes rendering zoom into account!
		font.setPointSize(static_cast<int>(m_fontSize * context.renderZoom));
		//font.setBold(true);
		QFontMetrics fontMetrics(font);

		double symbolSizeBackup = m_symbolSize;
		m_symbolSize *= static_cast<double>(context.renderZoom);

		double xpShift = 0.0;
		if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_HLEFT)
			xpShift = m_symbolSize / 2.0;
		else if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_HRIGHT)
			xpShift = -m_symbolSize / 2.0;

		double ypShift = 0.0;
		if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_VTOP)
			ypShift = m_symbolSize / 2.0;
		else if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_VBOTTOM)
			ypShift = -m_symbolSize / 2.0;

		//draw symbols + labels
		{
			for (unsigned i = 0; i < numberOfPoints; i++)
			{
				//symbol center
				const CCVector3* P = getPoint(i);

				//project it in 2D screen coordinates
				CCVector3d Q2D;
				m_lastCameraParams.project(*P, Q2D);

				//apply point color (if any)
				if (glParams.showColors)
				{
					color = &getPointColor(i);
				}
				//we must reset the color each time as the call to displayText may change the active color!
				glFunc->glColor4ubv(color->rgba);

				//draw associated symbol
				if (m_showSymbols && m_symbolSize > 0.0)
				{
					drawSymbolAt<QOpenGLFunctions_2_1>(glFunc, Q2D.x - context.glW / 2, Q2D.y - context.glH / 2, m_symbolSize / 2);
				}

				//draw associated label?
				if (m_showLabels && hasLabels && m_labels.size() > i && !m_labels[i].isNull())
				{
					//draw label
					context.display->displayText(	m_labels[i],
													static_cast<int>(Q2D.x + xpShift),
													static_cast<int>(Q2D.y + ypShift),
													m_labelAlignFlags,
													0,
													color,
													&font);
				}

			}
		}

		//restore original symbol size
		m_symbolSize = symbolSizeBackup;

		if (pushName)
		{
			glFunc->glPopName();
		}
	}
}
