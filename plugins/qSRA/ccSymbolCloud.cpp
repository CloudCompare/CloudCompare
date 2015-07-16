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

void ccSymbolCloud::drawSymbolAt(const double& xp, const double& yp) const
{
	//diamong with cross
	glBegin(GL_LINES);
	glVertex2d(xp, yp - m_symbolSize/2.0);
	glVertex2d(xp, yp + m_symbolSize/2.0);
	glVertex2d(xp - m_symbolSize/2.0, yp);
	glVertex2d(xp + m_symbolSize/2.0, yp);
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex2d(xp, yp - m_symbolSize/2.0);
	glVertex2d(xp + m_symbolSize/2.0, yp);
	glVertex2d(xp, yp + m_symbolSize/2.0);
	glVertex2d(xp - m_symbolSize/2.0, yp);
	glEnd();

}

void ccSymbolCloud::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!m_points->isAllocated())
		return;

	//nothing to do?!
	if (!m_showSymbols && !m_showLabels)
		return;

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

			glPushName(getUniqueID());
			hasLabels = false; //no need to display labels in 'picking' mode
		}

		//we should already be in orthoprojective & centered omde
		//glOrtho(-halfW,halfW,-halfH,halfH,-maxS,maxS);

		//default color
		const unsigned char* color = context.pointsDefaultCol.rgb;
		if (isColorOverriden())
		{
			color = m_tempColor.rgb;
			glParams.showColors = false;
		}
		
		if (!glParams.showColors)
			glColor3ubv(color);

		unsigned numberOfPoints = size();

		//viewport parameters (will be used to project 3D positions to 2D)
		int VP[4];
		context._win->getViewportArray(VP);
		const double* MM = context._win->getModelViewMatd(); //viewMat
		const double* MP = context._win->getProjectionMatd(); //projMat

		//only usefull when displaying labels!
		QFont font(context._win->getTextDisplayFont()); //takes rendering zoom into account!
		font.setPointSize(static_cast<int>(m_fontSize * context.renderZoom));
		//font.setBold(true);
		QFontMetrics fontMetrics(font);

		double symbolSizeBackup = m_symbolSize;
		m_symbolSize *= static_cast<double>(context.renderZoom);

		double xpShift = 0.0;
		if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_HLEFT)
			xpShift = m_symbolSize/2.0;
		else if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_HRIGHT)
			xpShift = -m_symbolSize/2.0;

		double ypShift = 0.0;
		if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_VTOP)
			ypShift = m_symbolSize/2.0;
		else if (m_labelAlignFlags & ccGenericGLDisplay::ALIGN_VBOTTOM)
			ypShift = -m_symbolSize/2.0;

		//draw symbols + labels
		{
			for (unsigned i=0;i<numberOfPoints;i++)
			{
				//symbol center
				const CCVector3* P = getPoint(i);

				//project it in 2D screen coordinates
				GLdouble xp,yp,zp;
				gluProject(P->x,P->y,P->z,MM,MP,VP,&xp,&yp,&zp);

				//apply point color (if any)
				if (glParams.showColors)
				{
					color = getPointColor(i);
					glColor3ubv(color);
				}
			
				//draw associated symbol
				if (m_showSymbols && m_symbolSize > 0.0)
				{
					drawSymbolAt(xp-static_cast<double>(context.glW/2),yp-static_cast<double>(context.glH/2));
				}

				//draw associated label?
				if (m_showLabels && hasLabels && m_labels.size() > i && !m_labels[i].isNull())
				{
					//draw label
					context._win->displayText(m_labels[i],static_cast<int>(xp+xpShift),static_cast<int>(yp+ypShift),m_labelAlignFlags,0,color,&font);
				}

			}
		}

		//restore original symbol size
		m_symbolSize = symbolSizeBackup;

		if (pushName)
			glPopName();
	}
}
