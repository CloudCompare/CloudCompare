#include "ccPlanarEntityInterface.h"

//Local
#include <ccCylinder.h>
#include <ccCone.h>
#include "ccSphere.h"
#include "ccTorus.h"

//Qt
#include <QSharedPointer>

ccPlanarEntityInterface::ccPlanarEntityInterface()
	: m_showNormalVector(false)
	, m_editable(false)
{
}

//unit normal representation
static QSharedPointer<ccCylinder> c_unitNormalSymbol(0);
static QSharedPointer<ccCone> c_unitNormalHeadSymbol(0);

void ccPlanarEntityInterface::glDrawNormal(CC_DRAW_CONTEXT& context, const CCVector3& pos, float scale, const ccColor::Rgb* color/*=0*/)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (!c_unitNormalSymbol)
	{
		c_unitNormalSymbol = QSharedPointer<ccCylinder>(new ccCylinder(0.02f, 0.9f, 0, "UnitNormal", 12));
		c_unitNormalSymbol->showColors(true);
		c_unitNormalSymbol->setVisible(true);
		c_unitNormalSymbol->setEnabled(true);
		c_unitNormalSymbol->setTempColor(ccColor::green);
	}
	if (!c_unitNormalHeadSymbol)
	{
		c_unitNormalHeadSymbol = QSharedPointer<ccCone>(new ccCone(0.05f, 0.0f, 0.1f, 0, 0, 0, "UnitNormalHead", 12));
		c_unitNormalHeadSymbol->showColors(true);
		c_unitNormalHeadSymbol->setVisible(true);
		c_unitNormalHeadSymbol->setEnabled(true);
		c_unitNormalHeadSymbol->setTempColor(ccColor::green);
	}

	//build-up the normal representation own 'context'
	CC_DRAW_CONTEXT normalContext = context;
	normalContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the primitives don't push their own!
	normalContext.display = 0;

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

//////////////////////////////////////////////////////////////////////////

//! editable normal

//Components geometry
static QSharedPointer<ccCylinder> c_arrowShaft(nullptr);
static QSharedPointer<ccCone> c_arrowHead(nullptr);
static QSharedPointer<ccSphere> c_centralSphere(nullptr);
static QSharedPointer<ccTorus> c_torus(nullptr);

static void DrawUnitArrow(int ID, const CCVector3& start, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
	{
		glFunc->glLoadName(ID);
	}

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, start.x, start.y, start.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0.0, 0.0, 1.0);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = static_cast<PointCoordinateType>(180.0);

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = acos(ps) * static_cast<PointCoordinateType>(CC_RAD_TO_DEG);

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_arrowShaft)
		c_arrowShaft = QSharedPointer<ccCylinder>(new ccCylinder(0.02f/*0.15f*/, /*0.9f*/0.6f, nullptr, "ArrowShaft", 12));
	if (!c_arrowHead)
		c_arrowHead = QSharedPointer<ccCone>(new ccCone(0.05f/*0.3f*/, 0, 0.1f/*0.4f*/, 0, 0, nullptr, "ArrowHead", 12/*24*/));

	glFunc->glTranslatef(0, 0, /*0.45f*/0.3f); // half of 0.9
	c_arrowShaft->setTempColor(col);
	c_arrowShaft->draw(context);
	glFunc->glTranslatef(0, 0, /*0.45f*/0.3f + 0.05f);
	c_arrowHead->setTempColor(col);
	c_arrowHead->draw(context);

	glFunc->glPopMatrix();
}

static void DrawUnitTorus(int ID, const CCVector3& center, const CCVector3& direction, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, scale, scale, scale);

	//we compute scalar prod between the two vectors
	CCVector3 Z(0, 0, 1);
	PointCoordinateType ps = Z.dot(direction);

	if (ps < 1)
	{
		CCVector3 axis(1, 0, 0);
		PointCoordinateType angle_deg = 180;

		if (ps > -1)
		{
			//we deduce angle from scalar prod
			angle_deg = acos(ps) * static_cast<PointCoordinateType>(CC_RAD_TO_DEG);

			//we compute rotation axis with scalar prod
			axis = Z.cross(direction);
		}

		ccGL::Rotate(glFunc, angle_deg, axis.x, axis.y, axis.z);
	}

	if (!c_torus)
		c_torus = QSharedPointer<ccTorus>(new ccTorus(0.05f/*0.2f*/, 0.10/*0.4f*/, 2.0*M_PI, false, 0, nullptr, "Torus", 12));

	glFunc->glTranslatef(0, 0, 0.45f);
	c_torus->setTempColor(col);
	c_torus->draw(context);

	glFunc->glPopMatrix();
}

//Unused function
static void DrawUnitSphere(int ID, const CCVector3& center, PointCoordinateType radius, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();

	ccGL::Translate(glFunc, center.x, center.y, center.z);
	ccGL::Scale(glFunc, radius, radius, radius);

	if (!c_centralSphere)
		c_centralSphere = QSharedPointer<ccSphere>(new ccSphere(0.05f, 0, "CentralSphere", 24));

	c_centralSphere->setTempColor(col);
	c_centralSphere->draw(context);

	glFunc->glPopMatrix();
}

static void DrawUnitCross(int ID, const CCVector3& center, PointCoordinateType scale, const ccColor::Rgb& col, CC_DRAW_CONTEXT& context)
{
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	if (ID > 0)
		glFunc->glLoadName(ID);

	scale /= 6;
	DrawUnitArrow(0, center, CCVector3(-1, 0, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3(1, 0, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3(0, -1, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3(0, 1, 0), scale, col, context);
	DrawUnitArrow(0, center, CCVector3(0, 0, -1), scale, col, context);
	DrawUnitArrow(0, center, CCVector3(0, 0, 1), scale, col, context);
}

void ccPlanarEntityInterface::glDrawNormal(CC_DRAW_CONTEXT& context, unsigned int id, const CCVector3& pos, float scale, const ccColor::Rgb* color/*=0*/)
{
// 	if (!m_editable) {
// 		glDrawNormal(context, pos, scale, color);
// 		return;
// 	}
	if (!MACRO_Draw3D(context))
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	//standard case: list names pushing (1st level)
	bool pushName = MACRO_DrawEntityNames(context) && m_editable;
	if (pushName)
	{
		glFunc->glPushName(id);
	}

	{
		//custom arrow 'context'
		CC_DRAW_CONTEXT componentContext = context;
		componentContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the arows don't push their own!
		componentContext.display = nullptr;

		if (pushName) //2nd level = sub-item
		{
			glFunc->glPushName(0); //fake ID, will be replaced by the arrows one if any
		}

		//force the light on
		glFunc->glPushAttrib(GL_LIGHTING_BIT);
		glFunc->glEnable(GL_LIGHT0);

		//! draw normal
		DrawUnitArrow(NORMAL_ARROW*pushName, pos, getNormal(), scale, color ? *color : ccColor::green, componentContext);

		//! draw interactors
		if (m_editable) {
			DrawUnitTorus(NORMAL_TORUS*pushName, pos, getNormal(), scale, ccColor::green, componentContext);
			DrawUnitSphere(CENTER_SPHERE*pushName, pos, scale, ccColor::yellow, componentContext);
			//DrawUnitCross(CROSS*pushName, pos, scale, ccColor::green, componentContext);
		}

		glFunc->glPopAttrib();

		if (pushName)
		{
			glFunc->glPopName();
		}
	}

	if (pushName)
	{
		glFunc->glPopName();
	}
}

static CCVector3d PointToVector(int x, int y, int screenWidth, int screenHeight)
{
	//convert mouse position to vector (screen-centered)
	CCVector3d v(static_cast<double>(2 * std::max(std::min(x, screenWidth - 1), -screenWidth + 1) - screenWidth) / static_cast<double>(screenWidth),
		static_cast<double>(screenHeight - 2 * std::max(std::min(y, screenHeight - 1), -screenHeight + 1)) / static_cast<double>(screenHeight),
		0);

	//square 'radius'
	double d2 = v.x*v.x + v.y*v.y;

	//projection on the unit sphere
	if (d2 > 1)
	{
		double d = sqrt(d2);
		v.x /= d;
		v.y /= d;
	}
	else
	{
		v.z = sqrt(1 - d2);
	}

	return v;
}

bool ccPlanarEntityInterface::move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight)
{
	//! rotation
	if (m_activeComponent != NORMAL_ARROW) {
		return false;
	}

	//convert mouse position to vector (screen-centered)
	CCVector3d currentOrientation = PointToVector(x, y, screenWidth, screenHeight);

	ccGLMatrixd rotMat = ccGLMatrixd::FromToRotation(m_lastOrientation, currentOrientation);

	CCVector3 C = getCenter();

	ccGLMatrixd transMat;
	transMat.setTranslation(-C);
	transMat = rotMat.inverse() * transMat; //rotMat * transMat;	//XYLIU 
	transMat.setTranslation(transMat.getTranslationAsVec3D() + CCVector3d::fromArray(C.u));
	
	//m_glTrans = ccGLMatrix(transMat.inverse().data()) * m_glTrans;
	//enableGLTransformation(true);
	notifyPlanarEntityChanged(ccGLMatrix(transMat.inverse().data()), false);
	emit planarEntityChanged();

	m_lastOrientation = currentOrientation;
	
	return true;
}

bool ccPlanarEntityInterface::move3D(const CCVector3d & u)
{
	if (m_activeComponent < NORMAL_TORUS) {
		return false;
	}

	return true;
}

void ccPlanarEntityInterface::setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd & viewMatrix)
{
	m_lastOrientation = PointToVector(x, y, screenWidth, screenHeight);
	m_viewMatrix = viewMatrix;
}

void ccPlanarEntityInterface::setActiveComponent(int id)
{
	switch (id)
	{
	case 1:
		m_activeComponent = NORMAL_ARROW;
		break;
	case 2:
		m_activeComponent = NORMAL_TORUS;
		break;
	case 3:
		m_activeComponent = CENTER_SPHERE;
		break;
	case 4:
		m_activeComponent = CROSS;
		break;
	default:
		m_activeComponent = NONE;
	}
}

CCVector3 ccPlanarEntityInterface::projectTo3DGlobal(CCVector3 pt_3d)
{
	return CCVector3();
}

CCVector2 ccPlanarEntityInterface::projectTo2DLocal(CCVector3 pt_3d)
{
	return CCVector2();
}

CCVector3 ccPlanarEntityInterface::backprojectTo3DGlobal(CCVector2 pt_2d)
{
	return CCVector3();
}

std::vector<CCVector3> ccPlanarEntityInterface::projectTo3DGlobal(std::vector<CCVector3> pt_3d)
{
	std::vector<CCVector3> prjs;
	for (auto & pt : pt_3d) {
		prjs.push_back(projectTo3DGlobal(pt));
	}
	return prjs;
}

std::vector<CCVector2> ccPlanarEntityInterface::projectTo2DLocal(std::vector<CCVector3> pt_3d)
{
	std::vector<CCVector2> prjs;
	for (auto & pt : pt_3d) {
		prjs.push_back(projectTo2DLocal(pt));
	}
	return prjs;
}

std::vector<CCVector3> ccPlanarEntityInterface::backprojectTo3DGlobal(std::vector<CCVector2> pt_2d)
{
	std::vector<CCVector3> prjs;
	for (auto & pt : pt_2d) {
		prjs.push_back(backprojectTo3DGlobal(pt));
	}
	return prjs;
}
