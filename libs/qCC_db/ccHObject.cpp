//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccHObject.h"

//Local
#include "ccIncludeGL.h"

//Objects handled by factory
#include "cc2DLabel.h"
#include "cc2DViewportLabel.h"
#include "ccBox.h"
#include "ccCameraSensor.h"
#include "ccCustomObject.h"
#include "ccCylinder.h"
#include "ccDish.h"
#include "ccExternalFactory.h"
#include "ccExtru.h"
#include "ccFacet.h"
#include "ccGBLSensor.h"
#include "ccImage.h"
#include "ccMaterialSet.h"
#include "ccMeshGroup.h"
#include "ccPlane.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccQuadric.h"
#include "ccSphere.h"
#include "ccSubMesh.h"
#include "ccTorus.h"

//Qt
#include <QIcon>

ccHObject::ccHObject(const QString& name, unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccObject(name, uniqueID)
	, ccDrawableObject()
	, m_parent(nullptr)
	, m_selectionBehavior(SELECTION_AA_BBOX)
	, m_isDeleting(false)
{
	setVisible(false);
	lockVisibility(true);
	
	m_glTransHistory.toIdentity();
}

ccHObject::ccHObject(const ccHObject& object)
	: ccObject(object)
	, ccDrawableObject(object)
	, m_parent(nullptr)
	, m_selectionBehavior(object.m_selectionBehavior)
	, m_isDeleting(false)
{
	m_glTransHistory.toIdentity();
}

ccHObject::~ccHObject()
{
	m_isDeleting = true;

	//process dependencies
	for (std::map<ccHObject*, int>::const_iterator it = m_dependencies.begin(); it != m_dependencies.end(); ++it)
	{
		assert(it->first);
		//notify deletion to other object?
		if ((it->second & DP_NOTIFY_OTHER_ON_DELETE) == DP_NOTIFY_OTHER_ON_DELETE)
		{
			it->first->onDeletionOf(this);
		}

		//delete other object?
		if ((it->second & DP_DELETE_OTHER) == DP_DELETE_OTHER)
		{
			it->first->removeDependencyFlag(this,DP_NOTIFY_OTHER_ON_DELETE); //in order to avoid any loop!
			//delete object
			if (it->first->isShareable())
				dynamic_cast<CCShareable*>(it->first)->release();
			else
				delete it->first;
		}
	}
	m_dependencies.clear();

	removeAllChildren();
}

void ccHObject::notifyGeometryUpdate()
{
	//the associated display bounding-box is (potentially) deprecated!!!
	if (m_currentDisplay)
	{
		m_currentDisplay->invalidateViewport();
		m_currentDisplay->deprecate3DLayer();
	}

	//process dependencies
	for (std::map<ccHObject*, int>::const_iterator it = m_dependencies.begin(); it != m_dependencies.end(); ++it)
	{
		assert(it->first);
		//notify deletion to other object?
		if ((it->second & DP_NOTIFY_OTHER_ON_UPDATE) == DP_NOTIFY_OTHER_ON_UPDATE)
		{
			it->first->onUpdateOf(this);
		}
	}
}

ccHObject* ccHObject::New(CC_CLASS_ENUM objectType, const char* name/*=0*/)
{
	switch(objectType)
	{
	case CC_TYPES::HIERARCHY_OBJECT:
		return new ccHObject(name);
	case CC_TYPES::POINT_CLOUD:
		return new ccPointCloud(name);
	case CC_TYPES::MESH:
		//warning: no associated vertices --> retrieved later
		return new ccMesh(nullptr);
	case CC_TYPES::SUB_MESH:
		//warning: no associated mesh --> retrieved later
		return new ccSubMesh(nullptr);
	case CC_TYPES::MESH_GROUP:
		//warning: deprecated
		ccLog::Warning("[ccHObject::New] Mesh groups are deprecated!");
		//warning: no associated vertices --> retrieved later
		return new ccMeshGroup();
	case CC_TYPES::POLY_LINE:
		//warning: no associated vertices --> retrieved later
		return new ccPolyline(nullptr);
	case CC_TYPES::FACET:
		return new ccFacet();
	case CC_TYPES::MATERIAL_SET:
		return new ccMaterialSet();
	case CC_TYPES::NORMALS_ARRAY:
		return new NormsTableType();
	case CC_TYPES::NORMAL_INDEXES_ARRAY:
		return new NormsIndexesTableType();
	case CC_TYPES::RGB_COLOR_ARRAY:
		return new ColorsTableType();
	case CC_TYPES::RGBA_COLOR_ARRAY:
		return new RGBAColorsTableType();
	case CC_TYPES::TEX_COORDS_ARRAY:
		return new TextureCoordsContainer();
	case CC_TYPES::IMAGE:
		return new ccImage();
	case CC_TYPES::CALIBRATED_IMAGE:
		return nullptr; //deprecated
	case CC_TYPES::GBL_SENSOR:
		//warning: default sensor type set in constructor (see CCLib::GroundBasedLidarSensor::setRotationOrder)
		return new ccGBLSensor();
	case CC_TYPES::CAMERA_SENSOR:
		return new ccCameraSensor();
	case CC_TYPES::LABEL_2D:
		return new cc2DLabel(name);
	case CC_TYPES::VIEWPORT_2D_OBJECT:
		return new cc2DViewportObject(name);
	case CC_TYPES::VIEWPORT_2D_LABEL:
		return new cc2DViewportLabel(name);
	case CC_TYPES::PLANE:
		return new ccPlane(name);
	case CC_TYPES::SPHERE:
		return new ccSphere(name);
	case CC_TYPES::TORUS:
		return new ccTorus(name);
	case CC_TYPES::CYLINDER:
	case CC_TYPES::OLD_CYLINDER_ID:
		return new ccCylinder(name);
	case CC_TYPES::BOX:
		return new ccBox(name);
	case CC_TYPES::CONE:
		return new ccCone(name);
	case CC_TYPES::DISH:
		return new ccDish(name);
	case CC_TYPES::EXTRU:
		return new ccExtru(name);
	case CC_TYPES::QUADRIC:
		return new ccQuadric(name);
	case CC_TYPES::TRANS_BUFFER:
		return new ccIndexedTransformationBuffer(name);
	case CC_TYPES::CUSTOM_H_OBJECT:
		return new ccCustomHObject(name);
	case CC_TYPES::CUSTOM_LEAF_OBJECT:
		return new ccCustomLeafObject(name);
	case CC_TYPES::POINT_OCTREE:
	case CC_TYPES::POINT_KDTREE:
		//construction this way is not supported (yet)
		ccLog::ErrorDebug("[ccHObject::New] This object (type %i) can't be constructed this way (yet)!",objectType);
		break;
	default:
		//unhandled ID
		ccLog::ErrorDebug("[ccHObject::New] Invalid object type (%i)!",objectType);
		break;
	}

	return nullptr;
}

ccHObject* ccHObject::New(const QString& pluginId, const QString& classId, const char* name)
{
	ccExternalFactory::Container::Shared externalFactories = ccExternalFactory::Container::GetUniqueInstance();
	if (!externalFactories)
	{
		return nullptr;
	}
	
	ccExternalFactory* factory = externalFactories->getFactoryByName(pluginId);
	if (!factory)
	{
		return nullptr;
	}
	
	ccHObject* obj = factory->buildObject(classId);

	if (name && obj)
	{
		obj->setName(name);
	}
	
	return obj;
}

QIcon ccHObject::getIcon() const
{
	return QIcon();
}

void ccHObject::addDependency(ccHObject* otherObject, int flags, bool additive/*=true*/)
{
	if (!otherObject || flags < 0)
	{
		ccLog::Error("[ccHObject::addDependency] Invalid arguments");
		assert(false);
		return;
	}
	else if (flags == 0)
	{
		return;
	}

	if (additive)
	{
		//look for already defined flags for this object
		std::map<ccHObject*,int>::iterator it = m_dependencies.find(otherObject);
		if (it != m_dependencies.end())
		{
			//nothing changes? we stop here (especially to avoid infinite
			//loop when setting  the DP_NOTIFY_OTHER_ON_DELETE flag below!)
			if ((it->second & flags) == flags)
				return;
			flags |= it->second;
		}
	}
	assert(flags != 0);

	m_dependencies[otherObject] = flags;

	//whenever we add a dependency, we must be sure to be notified
	//by the other object when its deleted! Otherwise we'll keep
	//bad pointers in the dependency list...
	otherObject->addDependency(this, DP_NOTIFY_OTHER_ON_DELETE);
}

int ccHObject::getDependencyFlagsWith(const ccHObject* otherObject) const
{
	std::map<ccHObject*, int>::const_iterator it = m_dependencies.find(const_cast<ccHObject*>(otherObject)); //DGM: not sure why erase won't accept a const pointer?! We try to modify the map here, not the pointer object!

	return (it != m_dependencies.end() ? it->second : 0);
}

void ccHObject::removeDependencyWith(ccHObject* otherObject)
{
	m_dependencies.erase(const_cast<ccHObject*>(otherObject)); //DGM: not sure why erase won't accept a const pointer?! We try to modify the map here, not the pointer object!
	if (!otherObject->m_isDeleting)
		otherObject->removeDependencyFlag(this, DP_NOTIFY_OTHER_ON_DELETE);
}

void ccHObject::removeDependencyFlag(ccHObject* otherObject, DEPENDENCY_FLAGS flag)
{
	int flags = getDependencyFlagsWith(otherObject);
	if ((flags & flag) == flag)
	{
		flags = (flags & (~flag));
		//either update the flags (if some bits remain)
		if (flags != 0)
			m_dependencies[otherObject] = flags;
		else //otherwise remove the dependency
			m_dependencies.erase(otherObject);
	}
}

void ccHObject::onDeletionOf(const ccHObject* obj)
{
	//remove any dependency declared with this object
	//and remove it from the children list as well (in case of)
	//DGM: we can't call 'detachChild' as this method will try to
	//modify the child contents!
	removeDependencyWith(const_cast<ccHObject*>(obj)); //this method will only modify the dependency flags of obj

	int pos = getChildIndex(obj);
	if (pos >= 0)
	{
		//we can't swap children as we want to keep the order!
		m_children.erase(m_children.begin() + pos);
	}
}

bool ccHObject::addChild(ccHObject* child, int dependencyFlags/*=DP_PARENT_OF_OTHER*/, int insertIndex/*=-1*/)
{
	if (!child)
	{
		assert(false);
		return false;
	}
	if (std::find(m_children.begin(), m_children.end(), child) != m_children.end())
	{
		ccLog::ErrorDebug("[ccHObject::addChild] Object is already a child!");
		return false;
	}

	if (isLeaf())
	{
		ccLog::ErrorDebug("[ccHObject::addChild] Leaf objects shouldn't have any child!");
		return false;
	}

	//insert child
	try
	{
		if (insertIndex < 0 || static_cast<size_t>(insertIndex) >= m_children.size())
			m_children.push_back(child);
		else
			m_children.insert(m_children.begin() + insertIndex, child);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory!
		return false;
	}

	//we want to be notified whenever this child is deleted!
	child->addDependency(this, DP_NOTIFY_OTHER_ON_DELETE); //DGM: potentially redundant with calls to 'addDependency' but we can't miss that ;)

	if (dependencyFlags != 0)
	{
		addDependency(child, dependencyFlags);
	}

	//the strongest link: between a parent and a child ;)
	if ((dependencyFlags & DP_PARENT_OF_OTHER) == DP_PARENT_OF_OTHER)
	{
		child->setParent(this);
		if (child->isShareable())
			dynamic_cast<CCShareable*>(child)->link();
		if (!child->getDisplay())
			child->setDisplay(getDisplay());
	}

	return true;
}

unsigned int ccHObject::getChildCountRecursive() const
{
	unsigned int	count = static_cast<unsigned>(m_children.size());
	
	for ( auto child : m_children )
	{
		count += child->getChildCountRecursive();
	}
	
	return count;
}

ccHObject* ccHObject::find(unsigned uniqueID) const
{
	//found the right item?
	if (getUniqueID() == uniqueID)
	{
		return const_cast<ccHObject *>(this);
	}
	
	//otherwise we are going to test all children recursively
	for (unsigned i=0; i<getChildrenNumber(); ++i)
	{
		ccHObject* match = getChild(i)->find(uniqueID);
		if (match)
		{
			return match;
		}
	}

	return nullptr;
}

unsigned ccHObject::filterChildren(	Container& filteredChildren,
									bool recursive/*=false*/,
									CC_CLASS_ENUM filter/*=CC_TYPES::OBJECT*/,
									bool strict/*=false*/,
									ccGenericGLDisplay* inDisplay/*=0*/) const
{
	for (auto child : m_children)
	{
		if (	(!strict && child->isKindOf(filter))
			||	( strict && child->isA(filter)))
		{
			if (!inDisplay || child->getDisplay() == inDisplay)
			{
				//warning: we have to handle unicity as a sibling may be in the same container as its parent!
				if (std::find(filteredChildren.begin(), filteredChildren.end(), child) == filteredChildren.end()) //not yet in output vector?
				{
					filteredChildren.push_back(child);
				}
			}
		}

		if (recursive)
		{
			child->filterChildren(filteredChildren, true, filter, strict, inDisplay);
		}
	}

	return static_cast<unsigned>(filteredChildren.size());
}

int ccHObject::getChildIndex(const ccHObject* child) const
{
	for (size_t i=0; i<m_children.size(); ++i)
		if (m_children[i] == child)
			return static_cast<int>(i);

	return -1;
}

void ccHObject::transferChild(ccHObject* child, ccHObject& newParent)
{
	assert(child);

	//remove link from old parent
	int childDependencyFlags = child->getDependencyFlagsWith(this);
	int parentDependencyFlags = getDependencyFlagsWith(child);
	
	detachChild(child); //automatically removes any dependency with this object

	newParent.addChild(child,parentDependencyFlags);
	child->addDependency(&newParent,childDependencyFlags);

	//after a successful transfer, either the parent is 'newParent' or a null pointer
	assert(child->getParent() == &newParent || child->getParent() == nullptr);
}

void ccHObject::transferChildren(ccHObject& newParent, bool forceFatherDependent/*=false*/)
{
	for (auto child : m_children)
	{
		//remove link from old parent
		int childDependencyFlags = child->getDependencyFlagsWith(this);
		int fatherDependencyFlags = getDependencyFlagsWith(child);
	
		//we must explicitely remove any dependency with the child as we don't call 'detachChild'
		removeDependencyWith(child);
		child->removeDependencyWith(this);

		newParent.addChild(child,fatherDependencyFlags);
		child->addDependency(&newParent,childDependencyFlags);

		//after a successful transfer, either the parent is 'newParent' or a null pointer
		assert(child->getParent() == &newParent || child->getParent() == nullptr);
	}
	m_children.clear();
}

void ccHObject::swapChildren(unsigned firstChildIndex, unsigned secondChildIndex)
{
	assert(firstChildIndex < m_children.size());
	assert(secondChildIndex < m_children.size());

	std::swap(m_children[firstChildIndex],m_children[secondChildIndex]);
}

int ccHObject::getIndex() const
{
	return (m_parent ? m_parent->getChildIndex(this) : -1);
}

bool ccHObject::isAncestorOf(const ccHObject *anObject) const
{
	assert(anObject);
	ccHObject* parent = anObject->getParent();
	if (!parent)
		return false;

	if (parent == this)
		return true;

	return isAncestorOf(parent);
}

bool ccHObject::getAbsoluteGLTransformation(ccGLMatrix& trans) const
{
	trans.toIdentity();
	bool hasGLTrans = false;
	
	//recurse among ancestors to get the absolute GL transformation
	const ccHObject* obj = this;
	while (obj)
	{
		if (obj->isGLTransEnabled())
		{
			trans = trans * obj->getGLTransformation();
			hasGLTrans = true;
		}
		obj = obj->getParent();
	}

	return hasGLTrans;
}

ccBBox ccHObject::getOwnBB(bool withGLFeatures/*=false*/)
{
	return ccBBox();
}

ccBBox ccHObject::getBB_recursive(bool withGLFeatures/*=false*/, bool onlyEnabledChildren/*=true*/)
{
	ccBBox box = getOwnBB(withGLFeatures);

	for (auto child : m_children)
	{
		if (!onlyEnabledChildren || child->isEnabled())
		{
			box += child->getBB_recursive(withGLFeatures,onlyEnabledChildren);
		}
	}

	return box;
}

ccBBox ccHObject::getDisplayBB_recursive(bool relative, const ccGenericGLDisplay* display/*=0*/)
{
	ccBBox box;

	if (!display || display == m_currentDisplay)
		box = getOwnBB(true);

	for (auto child : m_children)
	{
		if (child->isEnabled())
		{
			ccBBox childBox = child->getDisplayBB_recursive(true, display);
			if (child->isGLTransEnabled())
			{
				childBox = childBox * child->getGLTransformation();
			}
			box += childBox;
		}
	}

	if (!relative && box.isValid())
	{
		//get absolute bounding-box?
		ccGLMatrix trans;
		getAbsoluteGLTransformation(trans);
		box = box * trans;
	}

	return box;
}

bool ccHObject::isDisplayed() const
{
	return (getDisplay() != nullptr) && isVisible() && isBranchEnabled();
}

bool ccHObject::isDisplayedIn(const ccGenericGLDisplay* display) const
{
	return (getDisplay() == display) && isVisible() && isBranchEnabled();
}

bool ccHObject::isBranchEnabled() const
{
	if (!isEnabled())
		return false;
	
	if (m_parent)
		return m_parent->isBranchEnabled();

	return true;
}

void ccHObject::drawBB(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col)
{
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	switch (getSelectionBehavior())
	{
	case SELECTION_AA_BBOX:
		getDisplayBB_recursive(true, m_currentDisplay).draw(context, col);
		break;
	
	case SELECTION_FIT_BBOX:
		{
			//get the set of OpenGL functions (version 2.1)
			ccGLMatrix trans;
			ccBBox box = getOwnFitBB(trans);
			if (box.isValid())
			{
				glFunc->glMatrixMode(GL_MODELVIEW);
				glFunc->glPushMatrix();
				glFunc->glMultMatrixf(trans.data());
				box.draw(context, col);
				glFunc->glPopMatrix();
			}
		}
		break;
	
	case SELECTION_IGNORED:
		break;

	default:
		assert(false);
	}

	glFunc->glPopAttrib(); //GL_LINE_BIT
}

void ccHObject::drawNameIn3D(CC_DRAW_CONTEXT& context)
{
	if (!context.display)
		return;

	//we display it in the 2D layer in fact!
	//ccBBox bBox = getBB_recursive();
	//if (!bBox.isValid())
	//	return;
	
	//ccGLMatrix trans;
	//getAbsoluteGLTransformation(trans);

	//ccGLCameraParameters camera;
	//context.display->getGLCameraParameters(camera);

	//CCVector3 C = bBox.getCenter();
	//CCVector3d Q2D;
	//camera.project(C, Q2D);

	
	QFont font = context.display->getTextDisplayFont(); //takes rendering zoom into account!
	context.display->displayText(	getName(),
									static_cast<int>(m_nameIn3DPos.x),
									static_cast<int>(m_nameIn3DPos.y),
									ccGenericGLDisplay::ALIGN_HMIDDLE | ccGenericGLDisplay::ALIGN_VMIDDLE,
									0.75f,
									nullptr,
									&font);
}

void ccHObject::draw(CC_DRAW_CONTEXT& context)
{
	if (!isEnabled())
		return;
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );
	
	if ( glFunc == nullptr )
		return;

	//are we currently drawing objects in 2D or 3D?
	bool draw3D = MACRO_Draw3D(context);
	
	//the entity must be either visible or selected, and of course it should be displayed in this context
	bool drawInThisContext = ((m_visible || m_selected) && m_currentDisplay == context.display);

	if (draw3D)
	{
		//apply 3D 'temporary' transformation (for display only)
		if (m_glTransEnabled)
		{
			glFunc->glMatrixMode(GL_MODELVIEW);
			glFunc->glPushMatrix();
			glFunc->glMultMatrixf(m_glTrans.data());
		}

		//LOD for clouds is enabled?
		if (	context.decimateCloudOnMove
			&&	context.currentLODLevel > 0)
		{
			//only for real clouds
			drawInThisContext &= isA(CC_TYPES::POINT_CLOUD);
		}
	}

	//draw entity
	if (m_visible && drawInThisContext)
	{
		if (( !m_selected || !MACRO_SkipSelected(context) ) &&
			(  m_selected || !MACRO_SkipUnselected(context) ))
		{
			//apply default color (in case of)
			ccGL::Color4v(glFunc, context.pointsDefaultCol.rgba);

			//enable clipping planes (if any)
			bool useClipPlanes = (draw3D && !m_clipPlanes.empty());
			if (useClipPlanes)
			{
				toggleClipPlanes(context, true);
			}

			drawMeOnly(context);

			//disable clipping planes (if any)
			if (useClipPlanes)
			{
				toggleClipPlanes(context, false);
			}
		}
	}

	//draw name - container objects are not visible but can still show a name
	if (m_currentDisplay == context.display && m_showNameIn3D && !MACRO_DrawEntityNames(context))
	{
		if (MACRO_Draw3D(context))
		{
			//we have to comute the 2D position during the 3D pass!
			ccBBox bBox = getBB_recursive(true); //DGM: take the OpenGL features into account (as some entities are purely 'GL'!)
			if (bBox.isValid())
			{
				ccGLCameraParameters camera;
				glFunc->glGetIntegerv(GL_VIEWPORT, camera.viewport);
				glFunc->glGetDoublev(GL_PROJECTION_MATRIX, camera.projectionMat.data());
				glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, camera.modelViewMat.data());

				CCVector3 C = bBox.getCenter();
				camera.project(C, m_nameIn3DPos);
			}
		}
		else if (MACRO_Draw2D(context) && MACRO_Foreground(context))
		{
			//then we can display the name during the 2D pass
			drawNameIn3D(context);
		}
	}

	//draw entity's children
	for (auto child : m_children)
	{
		child->draw(context);
	}
	
	//if the entity is currently selected, we draw its bounding-box
	if (m_selected && draw3D && drawInThisContext && !MACRO_DrawEntityNames(context) && context.currentLODLevel == 0)
	{
		drawBB(context, context.bbDefaultCol);
	}

	if (draw3D && m_glTransEnabled)
		glFunc->glPopMatrix();
}

void ccHObject::applyGLTransformation(const ccGLMatrix& trans)
{
	m_glTransHistory = trans * m_glTransHistory;
}

void ccHObject::applyGLTransformation_recursive(const ccGLMatrix* transInput/*=nullptr*/)
{
	ccGLMatrix transTemp;
	const ccGLMatrix* transToApply = transInput;

	if (m_glTransEnabled)
	{
		if (!transInput)
		{
			//if no transformation is provided (by father)
			//we initiate it with the current one
			transToApply = &m_glTrans;
		}
		else
		{
			transTemp = *transInput * m_glTrans;
			transToApply = &transTemp;
		}
	}

	if (transToApply)
	{
		applyGLTransformation(*transToApply);
		notifyGeometryUpdate();
	}

	for (auto child : m_children)
		child->applyGLTransformation_recursive(transToApply);

	if (m_glTransEnabled)
		resetGLTransformation();
}

unsigned ccHObject::findMaxUniqueID_recursive() const
{
	unsigned id = getUniqueID();

	for (auto child : m_children)
	{
		unsigned childMaxID = child->findMaxUniqueID_recursive();
		if (id < childMaxID)
		{
			id = childMaxID;
		}
	}

	return id;
}

void ccHObject::detachChild(ccHObject* child)
{
	if (!child)
	{
		assert(false);
		return;
	}

	//remove any dependency (bilateral)
	removeDependencyWith(child);
	child->removeDependencyWith(this);

	if (child->getParent() == this)
	{
		child->setParent(nullptr);
	}
	
	int pos = getChildIndex(child);
	if (pos >= 0)
	{
		//we can't swap children as we want to keep the order!
		m_children.erase(m_children.begin()+pos);
	}
}

void ccHObject::detatchAllChildren()
{
	for (auto child : m_children)
	{
		//remove any dependency (bilateral)
		removeDependencyWith(child);
		child->removeDependencyWith(this);

		if (child->getParent() == this)
		{
			child->setParent(nullptr);
		}
	}
	m_children.clear();
}

void ccHObject::removeChild(ccHObject* child)
{
	int pos = getChildIndex(child);
	if (pos >= 0)
	{
		removeChild(pos);
	}
}

void ccHObject::removeChild(int pos)
{
	if (pos < 0 || static_cast<size_t>(pos) >= m_children.size())
	{
		assert(false);
		return;
	}

	ccHObject* child = m_children[pos];

	//we can't swap as we want to keep the order!
	//(DGM: do this BEFORE deleting the object (otherwise
	//the dependency mechanism can 'backfire' ;)
	m_children.erase(m_children.begin() + pos);

	//backup dependency flags
	int flags = getDependencyFlagsWith(child);

	//remove any dependency
	removeDependencyWith(child);
	//child->removeDependencyWith(this); //DGM: no, don't do this otherwise this entity won't be warned that the child has been removed!

	if ((flags & DP_DELETE_OTHER) == DP_DELETE_OTHER)
	{
		//delete object
		if (child->isShareable())
			dynamic_cast<CCShareable*>(child)->release();
		else/* if (!child->isA(CC_TYPES::POINT_OCTREE))*/
			delete child;
	}
	else if (child->getParent() == this)
	{
		child->setParent(nullptr);
	}
}

void ccHObject::removeAllChildren()
{
	while (!m_children.empty())
	{
		ccHObject* child = m_children.back();
		m_children.pop_back();

		int flags = getDependencyFlagsWith(child);
		if ((flags & DP_DELETE_OTHER) == DP_DELETE_OTHER)
		{
			if (child->isShareable())
				dynamic_cast<CCShareable*>(child)->release();
			else
				delete child;
		}
	}
}

bool ccHObject::isSerializable() const
{
	//we only handle pure CC_TYPES::HIERARCHY_OBJECT here (object groups)
	return (getClassID() == CC_TYPES::HIERARCHY_OBJECT);
}

bool ccHObject::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//write 'ccObject' header
	if (!ccObject::toFile(out))
		return false;

	//write own data
	if (!toFile_MeOnly(out))
		return false;

	//(serializable) child count (dataVersion >= 20)
	uint32_t serializableCount = 0;
	for (auto child : m_children)
	{
		if (child->isSerializable())
		{
			++serializableCount;
		}
	}
	
	if (out.write(reinterpret_cast<const char*>(&serializableCount), sizeof(uint32_t)) < 0)
		return WriteError();

	//write serializable children (if any)
	for (auto child : m_children)
	{
		if (child->isSerializable())
		{
			if (!child->toFile(out))
				return false;
		}
	}

	//write current selection behavior (dataVersion >= 23)
	if (out.write(reinterpret_cast<const char*>(&m_selectionBehavior), sizeof(SelectionBehavior)) < 0)
		return WriteError();

	//write transformation history (dataVersion >= 45)
	m_glTransHistory.toFile(out);

	return true;
}

bool ccHObject::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!fromFileNoChildren(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//(serializable) child count (dataVersion>=20)
	uint32_t serializableCount = 0;
	if (in.read(reinterpret_cast<char*>(&serializableCount), 4) < 0)
		return ReadError();

	//read serializable children (if any)
	for (uint32_t i = 0; i < serializableCount; ++i)
	{
		//read children class ID
		CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
		if (classID == CC_TYPES::OBJECT)
			return false;

		if (dataVersion >= 35 && dataVersion <= 47 && ((classID & CC_CUSTOM_BIT) != 0))
		{
			//bug fix: for a long time the CC_CAMERA_BIT and CC_QUADRIC_BIT were wrongly defined
			//with two bits instead of one! The additional and wrongly defined bit was the CC_CUSTOM_BIT :(
			if (	(classID & CC_TYPES::CAMERA_SENSOR) == CC_TYPES::CAMERA_SENSOR
				||	(classID & CC_TYPES::QUADRIC) == CC_TYPES::QUADRIC
				)
			{
				classID &= (~CC_CUSTOM_BIT);
			}
		}

		//create corresponding child object
		ccHObject* child = New(classID);

		//specifc case of custom objects (defined by plugins)
		if ((classID & CC_TYPES::CUSTOM_H_OBJECT) == CC_TYPES::CUSTOM_H_OBJECT)
		{
			//store current position
			size_t originalFilePos = in.pos();
			//we need to load the custom object as plain ccCustomHobject
			child->fromFileNoChildren(in, dataVersion, flags, oldToNewIDMap);
			//go back to original position
			in.seek(originalFilePos);
			//get custom object name and plugin name
			QString childName = child->getName();
			QString classId = child->getMetaData(ccCustomHObject::DefautMetaDataClassName()).toString();
			QString pluginId = child->getMetaData(ccCustomHObject::DefautMetaDataPluginName()).toString();
			//dont' need this instance anymore
			delete child;
			child = nullptr;

			// try to get a new object from external factories
			ccHObject* newChild = ccHObject::New(pluginId, classId);
			if (newChild) // found a plugin that can deserialize it
			{
				child = newChild;
			}
			else
			{
				ccLog::Warning(QString("[ccHObject::fromFile] Couldn't find a plugin able to deserialize custom object '%1' (class_ID = %2 / plugin_ID = %3)").arg(childName).arg(classID).arg(pluginId));
				return false; // FIXME: for now simply return false. We may want to skip it but I'm not sure if there is a simple way of doing that
			}
		}

		assert(child && child->isSerializable());
		if (child)
		{
			if (child->fromFile(in, dataVersion, flags, oldToNewIDMap))
			{
				//FIXME
				//addChild(child,child->getFlagState(CC_FATHER_DEPENDENT));
				addChild(child);
			}
			else
			{
				//delete child; //we can't do this as the object might be invalid
				return false;
			}
		}
		else
		{
			return CorruptError();
		}
	}

	//read the selection behavior (dataVersion>=23)
	if (dataVersion >= 23)
	{
		if (in.read(reinterpret_cast<char*>(&m_selectionBehavior), sizeof(SelectionBehavior)) < 0)
		{
			return ReadError();
		}
	}
	else
	{
		m_selectionBehavior = SELECTION_AA_BBOX;
	}

	//read transformation history (dataVersion >= 45)
	if (dataVersion >= 45)
	{
		if (!m_glTransHistory.fromFile(in, dataVersion, flags, oldToNewIDMap))
		{
			return false;
		}
	}

	return true;
}

bool ccHObject::fromFileNoChildren(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	//read 'ccObject' header
	if (!ccObject::fromFile(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//read own data
	return fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap);
}

bool ccHObject::toFile_MeOnly(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	/*** ccHObject takes in charge the ccDrawableObject properties (which is not a ccSerializableObject) ***/

	//'visible' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_visible), sizeof(bool)) < 0)
		return WriteError();
	//'lockedVisibility' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_lockedVisibility), sizeof(bool)) < 0)
		return WriteError();
	//'colorsDisplayed' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_colorsDisplayed), sizeof(bool)) < 0)
		return WriteError();
	//'normalsDisplayed' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_normalsDisplayed), sizeof(bool)) < 0)
		return WriteError();
	//'sfDisplayed' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_sfDisplayed), sizeof(bool)) < 0)
		return WriteError();
	//'colorIsOverriden' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_colorIsOverriden), sizeof(bool)) < 0)
		return WriteError();
	if (m_colorIsOverriden)
	{
		//'tempColor' (dataVersion>=20)
		if (out.write(reinterpret_cast<const char*>(m_tempColor.rgba), sizeof(ColorCompType)*3) < 0) //TODO: save the alpha channel?
		{
			return WriteError();
		}
	}
	//'glTransEnabled' state (dataVersion>=20)
	if (out.write(reinterpret_cast<const char*>(&m_glTransEnabled), sizeof(bool)) < 0)
		return WriteError();
	if (m_glTransEnabled)
	{
		if (!m_glTrans.toFile(out))
		{
			return false;
		}
	}

	//'showNameIn3D' state (dataVersion>=24)
	if (out.write(reinterpret_cast<const char*>(&m_showNameIn3D), sizeof(bool)) < 0)
		return WriteError();

	return true;
}

bool ccHObject::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	/*** ccHObject takes in charge the ccDrawableObject properties (which is not a ccSerializableObject) ***/

	//'visible' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_visible), sizeof(bool)) < 0)
		return ReadError();
	//'lockedVisibility' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_lockedVisibility), sizeof(bool)) < 0)
		return ReadError();
	//'colorsDisplayed' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_colorsDisplayed), sizeof(bool)) < 0)
		return ReadError();
	//'normalsDisplayed' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_normalsDisplayed), sizeof(bool)) < 0)
		return ReadError();
	//'sfDisplayed' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_sfDisplayed), sizeof(bool)) < 0)
		return ReadError();
	//'colorIsOverriden' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_colorIsOverriden), sizeof(bool)) < 0)
		return ReadError();
	if (m_colorIsOverriden)
	{
		//'tempColor' (dataVersion>=20)
		if (in.read(reinterpret_cast<char*>(m_tempColor.rgba), sizeof(ColorCompType)*3) < 0)
			return ReadError();
		m_tempColor.a = ccColor::MAX;
	}
	//'glTransEnabled' state (dataVersion>=20)
	if (in.read(reinterpret_cast<char*>(&m_glTransEnabled), sizeof(bool)) < 0)
		return ReadError();
	if (m_glTransEnabled)
	{
		if (!m_glTrans.fromFile(in, dataVersion, flags, oldToNewIDMap))
		{
			return false;
		}
	}

	//'showNameIn3D' state (dataVersion>=24)
	if (dataVersion >= 24)
	{
		if (in.read(reinterpret_cast<char*>(&m_showNameIn3D), sizeof(bool)) < 0)
		{
			return WriteError();
		}
	}
	else
	{
		m_showNameIn3D = false;
	}

	return true;
}
