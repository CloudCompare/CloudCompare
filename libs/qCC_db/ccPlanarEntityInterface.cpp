#include "ccPlanarEntityInterface.h"

//Local
#include <ccCylinder.h>
#include <ccCone.h>

//Qt
#include <QSharedPointer>

ccPlanarEntityInterface::ccPlanarEntityInterface()
	: m_showNormalVector(false)
{
}

//unit normal representation
static QSharedPointer<ccCylinder> c_unitNormalSymbol(nullptr);
static QSharedPointer<ccCone> c_unitNormalHeadSymbol(nullptr);

void ccPlanarEntityInterface::glDrawNormal(CC_DRAW_CONTEXT& context, const CCVector3& pos, float scale, const ccColor::Rgb* color/*=0*/)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (!c_unitNormalSymbol)
	{
		c_unitNormalSymbol = QSharedPointer<ccCylinder>(new ccCylinder(0.02f, 0.9f, nullptr, "UnitNormal", 12));
		c_unitNormalSymbol->showColors(true);
		c_unitNormalSymbol->setVisible(true);
		c_unitNormalSymbol->setEnabled(true);
		c_unitNormalSymbol->setTempColor(ccColor::green);
	}
	if (!c_unitNormalHeadSymbol)
	{
		c_unitNormalHeadSymbol = QSharedPointer<ccCone>(new ccCone(0.05f, 0.0f, 0.1f, 0, 0, nullptr, "UnitNormalHead", 12));
		c_unitNormalHeadSymbol->showColors(true);
		c_unitNormalHeadSymbol->setVisible(true);
		c_unitNormalHeadSymbol->setEnabled(true);
		c_unitNormalHeadSymbol->setTempColor(ccColor::green);
	}

	//build-up the normal representation own 'context'
	CC_DRAW_CONTEXT normalContext = context;
	normalContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the primitives don't push their own!
	normalContext.display = nullptr;

	if (color)
	{
		c_unitNormalSymbol->setTempColor(*color, true);
		c_unitNormalHeadSymbol->setTempColor(*color, true);
	}
	else
	{
		c_unitNormalSymbol->enableTempColor(false);
		c_unitNormalHeadSymbol->enableTempColor(false);
	}

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	ccGL::Translate(glFunc, pos.x, pos.y, pos.z);
	ccGLMatrix mat = ccGLMatrix::FromToRotation(CCVector3(0, 0, PC_ONE), getNormal());
	glFunc->glMultMatrixf(mat.data());
	ccGL::Scale(glFunc, scale, scale, scale);
	glFunc->glTranslatef(0, 0, 0.45f);
	c_unitNormalSymbol->draw(normalContext);
	glFunc->glTranslatef(0, 0, 0.45f);
	c_unitNormalHeadSymbol->draw(normalContext);
	glFunc->glPopMatrix();
}