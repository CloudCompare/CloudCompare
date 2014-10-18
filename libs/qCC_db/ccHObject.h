//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_HIERARCHY_OBJECT_HEADER
#define CC_HIERARCHY_OBJECT_HEADER

//Local
#include "qCC_db.h"
#include "ccObject.h"
#include "ccDrawableObject.h"

//System
#include <vector>

class QIcon;

//! Hierarchical CloudCompare Object
class QCC_DB_LIB_API ccHObject : public ccObject, public ccDrawableObject
{
public:

	//! Default constructor
	/** \param name object name (optional)
	**/
	ccHObject(QString name = QString());
	//! Copy constructor
	ccHObject(const ccHObject& object);

	//! Default destructor
	virtual ~ccHObject();

	//! Static factory
	/** Warning: objects depending on other structures (such as meshes 
		or polylines that should be linked with point clouds playing the
		role of vertices) are returned 'naked'.
		\param objectType object type
		\param name object name (optional)
		\return instantiated object (if type is valid) or 0
	**/
	static ccHObject* New(CC_CLASS_ENUM objectType, const char* name = 0);

	//! Static factory (version to be used by external plugin factories)
	/** Two strings are used as keys, one for the plugin name and one for the class name.
		Those strings will typically be saved as metadata of a custom object
	**/
	static ccHObject* New(QString pluginId, QString classId, const char* name = 0);

	//! Returns class ID
	/** \return class unique ID
	**/
	inline virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::HIERARCHY_OBJECT; }

	//! Returns the icon associated to this entity
	/** ccDBRoot will call this method: if an invalid icon is returned
		the default icon for that type will be used instead.
		\return invalid icon by default (to be re-implemented by child class)
	**/
	virtual QIcon getIcon() const;
	
	//! Returns parent object
	/** \return parent object (NULL if no parent)
	**/
	inline ccHObject* getParent() const { return m_parent; }

	/*** Inter-objects dependencies management ***/

	//! Dependency flags
	enum DEPENDENCY_FLAGS {	DP_NONE						= 0,	/**< no dependency **/
							DP_NOTIFY_OTHER_ON_DELETE	= 1,	/**< notify 'other' when deleted (will call ccHObject::onDeletionOf) **/
							DP_NOTIFY_OTHER_ON_UPDATE	= 2,	/**< notify 'other' when its geometry is modified (will call ccHObject::onUpdateOf) **/
							//DP_NOTIFY_XXX				= 4, 
							DP_DELETE_OTHER				= 8,	/**< delete 'other' before deleting itself **/
							DP_PARENT_OF_OTHER			= 24,	/**< same as DP_DELETE_OTHER + declares itself as parent of 'other' **/
	};

	//! Adds a new dependence (additive or not)
	/** \param otherObject other object
		\param flags dependency flags (see DEPENDENCY_FLAGS)
		\param additive whether we should 'add' the flag(s) if there's already a dependence with the other object or not
	**/
	void addDependency(ccHObject* otherObject, int flags, bool additive = true);

	//! Returns the dependency flags with a given object
	/** \param otherObject other object
	**/
	int getDependencyFlagsWith(const ccHObject* otherObject);

	//! Removes any dependency flags with a given object
	/** \param otherObject other object
	**/
	void removeDependencyWith(ccHObject* otherObject);

	//! Removes a given dependency flag
	/** \param otherObject other object
		\param flag dependency flag to remove (see DEPENDENCY_FLAGS)
	**/
	void removeDependencyFlag(ccHObject* otherObject, DEPENDENCY_FLAGS flag);

	/*** children management ***/

	//! Adds a child
	/** \warning by default (i.e. with the DP_PARENT_OF_OTHER flag) the child's parent
		will be automatically replaced by this instance. Moreover the child will be deleted

		\param child child instance
		\param dependencyFlags dependency flags
		\param insertIndex insertion index (if <0, child is simply appended to the children list)
		\return success
	**/
	virtual bool addChild(ccHObject* child, int dependencyFlags = DP_PARENT_OF_OTHER, int insertIndex = -1);

	//! Returns the number of children
	/** \return children number
	**/
	inline unsigned getChildrenNumber() const { return static_cast<unsigned>(m_children.size()); }

	//! Returns the ith child
	/** \param childPos child position
		\return child object (or NULL if wrong position)
	**/
	inline ccHObject* getChild(unsigned childPos) const { return (childPos < getChildrenNumber() ? m_children[childPos] : 0); }

	//! Finds an entity in this object hierarchy
	/** \param uniqueID child unique ID
		\return child (or NULL if not found)
	**/
	ccHObject* find(unsigned uniqueID);

	//! standard ccHObject container (for children, etc.)
	typedef std::vector<ccHObject*> Container;

	//! Collects the children corresponding to a certain pattern
	/** \param filteredChildren result container
		\param recursive specifies if the search should be recursive
		\param filter pattern for children selection
		\param strict whether the search is strict on the type (i.e 'isA') or not (i.e. 'isKindOf')
		\return number of collected children
	**/
	unsigned filterChildren(Container& filteredChildren,
							bool recursive = false,
							CC_CLASS_ENUM filter = CC_TYPES::OBJECT,
							bool strict = false) const;

	//! Detaches a specific child
	/** This method does not delete the child.
		Removes any dependency between the flag and this object
	**/
	void detachChild(ccHObject* child);
	//! Removes a specific child
	/** \warning This method may delete the child if the DP_PARENT_OF_OTHER
		dependency flag is set for this child (use detachChild if you
		want to avoid deletion).
	**/
	//! Detaches all children
	void detatchAllChildren();

	void removeChild(ccHObject* child);
	//! Removes a specific child given its index
	/** \warning This method may delete the child if the DP_PARENT_OF_OTHER
		dependency flag is set for this child (use detachChild if you
		want to avoid deletion).
	**/
	void removeChild(int pos);
	//! Removes all children
	void removeAllChildren();
	//! Returns child index
	int getChildIndex(const ccHObject* aChild) const;
	//! Swaps two children
	void swapChildren(unsigned firstChildIndex, unsigned secondChildIndex);
	//! Returns index relatively to its parent or -1 if no parent
	int getIndex() const;

	//! Transfer a given child to another parent
	void transferChild(ccHObject* child, ccHObject& newParent);
	//! Transfer all children to another parent
	void transferChildren(ccHObject& newParent, bool forceFatherDependent = false);

	//! Shortcut: returns first child
	ccHObject* getFirstChild() const { return (m_children.empty() ? 0 : m_children.front()); }
	//! Shortcut: returns last child
	ccHObject* getLastChild() const { return (m_children.empty() ? 0 : m_children.back()); }

	//! Returns true if the current object is an ancestor of the specified one
	bool isAncestorOf(const ccHObject *anObject) const;

	//Inherited from ccDrawableObject
	virtual ccBBox getBB(bool relative = true, bool withGLfeatures = false, const ccGenericGLDisplay* window = NULL);
	virtual void draw(CC_DRAW_CONTEXT& context);

	//! Returns whether the object is actually displayed (visible) or not
	virtual bool isDisplayed() const;

	//! Returns whether the object and all its ancestors are enabled
	virtual bool isBranchEnabled() const;

	/*** RECURSIVE CALL SCRIPTS ***/
	
	//0 parameter
	#define ccHObject_recursive_call0(baseName,recursiveName) \
	inline virtual void recursiveName() \
	{ \
		baseName(); \
		for (Container::iterator it = m_children.begin(); it != m_children.end(); ++it) \
			(*it)->recursiveName(); \
	} \

	//1 parameter
	#define ccHObject_recursive_call1(baseName,param1Type,recursiveName) \
	inline virtual void recursiveName(param1Type p) \
	{ \
		baseName(p); \
		for (Container::iterator it = m_children.begin(); it != m_children.end(); ++it) \
			(*it)->recursiveName(p); \
	} \

	/*****************************/

	//recursive equivalents of some of ccDrawableObject methods
	ccHObject_recursive_call1(setSelected,bool,setSelected_recursive);
	ccHObject_recursive_call1(setDisplay,ccGenericGLDisplay*,setDisplay_recursive);
	ccHObject_recursive_call1(removeFromDisplay,ccGenericGLDisplay*,removeFromDisplay_recursive);
	ccHObject_recursive_call0(prepareDisplayForRefresh,prepareDisplayForRefresh_recursive);
	ccHObject_recursive_call0(refreshDisplay,refreshDisplay_recursive);
	ccHObject_recursive_call0(resetGLTransformationHistory,resetGLTransformationHistory_recursive);
	ccHObject_recursive_call0(toggleActivation,toggleActivation_recursive);
	ccHObject_recursive_call0(toggleVisibility,toggleVisibility_recursive);
	ccHObject_recursive_call0(toggleColors,toggleColors_recursive);
	ccHObject_recursive_call0(toggleNormals,toggleNormals_recursive);
	ccHObject_recursive_call0(toggleSF,toggleSF_recursive);
	ccHObject_recursive_call0(toggleShowName,toggleShowName_recursive);
	ccHObject_recursive_call0(toggleMaterials,toggleMaterials_recursive);

	//! Returns the max 'unique ID' of this entity and its siblings
	unsigned findMaxUniqueID_recursive() const;

	//! Applies the active OpenGL transformation to the entity (recursive)
	/** The input ccGLMatrix should be left to 0, unless you want to apply
		a pre-transformation.
		\param trans a ccGLMatrix structure (reference to)
	**/
	void applyGLTransformation_recursive(ccGLMatrix* trans = 0);

	//! Returns the bounding-box center
	/** \return bounding-box center
	**/
	virtual CCVector3 getBBCenter();

	//! Notifies all dependent entities that the geometry of this entity has changed
	virtual void notifyGeometryUpdate();

	//! Returns the entity bounding-box only
	/** Children bboxes are ignored.
		\return bounding-box
	**/
	virtual ccBBox getMyOwnBB();

	//! Returns the entity GL display bounding-box
	/** Children bboxes are ignored. The bounding-box
		should take into account entity geometrical data
		and any other 3D displayed elements.
		\return bounding-box
	**/
	virtual ccBBox getDisplayBB();

	//inherited from ccSerializableObject
	virtual bool isSerializable() const;
	virtual bool toFile(QFile& out) const;
	inline virtual bool fromFile(QFile& in, short dataVersion, int flags) { return fromFile(in,dataVersion,flags,false); }

	//! Custom version of ccSerializableObject::fromFile
	/** This version is used to load only the object own part of a stream (an not its children's)
		\param in input file (already opened)
		\param dataVersion file version
		\param flags deserialization flags (see ccSerializableObject::DeserializationFlags)
		\param omitChildren to omit loading the children's part of the stream
		\return success
	**/
	virtual bool fromFile(QFile& in, short dataVersion, int flags, bool omitChildren);

	//! Returns whether object is shareable or not
	/** If object is father dependent and 'shared', it won't
		be deleted but 'released' instead.
	**/
	virtual bool isShareable() const { return false; }

	//! Behavior when selected
	enum SelectionBehavior { SELECTION_AA_BBOX,
							 SELECTION_FIT_BBOX,
							 SELECTION_IGNORED };

	//! Sets selection behavior (when displayed)
	/** WARNING: SELECTION_FIT_BBOX relies on the
		'ccDrawableObject::getFitBB' method (which
		is not supported by all entities).
	**/
	void setSelectionBehavior(SelectionBehavior mode) { m_selectionBehavior = mode; }

	//! Returns selection behavior
	SelectionBehavior getSelectionBehavior() const { return m_selectionBehavior; }

	//! Returns object unqiue ID used for display
	virtual unsigned getUniqueIDForDisplay() const { return getUniqueID(); }

	//! Returns the transformation 'history' matrix
	const ccGLMatrix& getGLTransformationHistory() const { return m_glTransHistory; }
	//! Sets the transformation 'history' matrix (handle with care!)
	void setGLTransformationHistory(const ccGLMatrix& mat) { m_glTransHistory = mat; }
	//! Resets the transformation 'history' matrix
	void resetGLTransformationHistory() { m_glTransHistory.toIdentity(); }

protected:

	//! Sets parent object
	virtual inline void setParent(ccHObject* anObject) { m_parent = anObject; }

	//! Draws the entity only (not its children)
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) { /*does nothing by default*/ }

	//! Applies a GL transformation to the entity
	/** this = rotMat*(this-rotCenter)+(rotCenter+trans)
		\param trans a ccGLMatrix structure
	**/
	virtual void applyGLTransformation(const ccGLMatrix& trans);

	//! Save own object data
	/** Called by 'toFile' (recursive scheme)
		To be overloaded (but still called;) by subclass.
	**/
	virtual bool toFile_MeOnly(QFile& out) const;

	//! Loads own object data
	/** Called by 'fromFile' (recursive scheme)
		To be overloaded (but still called;) by subclass.
		\param in input file
		\param dataVersion file version
		\param flags deserialization flags (see ccSerializableObject::DeserializationFlags)
	**/
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);

	//! Draws the entity name in 3D
	/** Names is displayed at the center of the bounding box by default.
	**/
	virtual void drawNameIn3D(CC_DRAW_CONTEXT& context);

	//! This method is called when another object is deleted
	/** For internal use.
	**/
	virtual void onDeletionOf(const ccHObject* obj);

	//! This method is called when another object (geometry) is updated
	/** For internal use.
	**/
	virtual void onUpdateOf(ccHObject* obj) { /*does nothing by default*/ }

	//! Object's parent
	ccHObject* m_parent;

	//! Object's children
	Container m_children;

	//! Selection behavior
	SelectionBehavior m_selectionBehavior;

	//! Dependencies map
	/** First parameter: other object
		Second parameter: dependency flags (see DEPENDENCY_FLAGS)
	**/
	std::map<ccHObject*,int> m_dependencies;

	//! Cumulative GL transformation
	/** History of all the applied transformations since the creation of the object
		as a single transformation.
	**/
	ccGLMatrix m_glTransHistory;

	//! Flag to safely handle dependencies when the object is being deleted
	bool m_isDeleting;
};

/*** Helpers ***/

//! Puts all entities inside a container in a group
/** Automatically removes siblings so as to get a valid hierarchy object.
	\param origin origin container
	\param dest destination group
	\param dependencyFlags default dependency link for the children added to the group
**/
inline void ConvertToGroup(const ccHObject::Container& origin, ccHObject& dest, int dependencyFlags = ccHObject::DP_NONE)
{
	size_t count = origin.size();
	for (size_t i=0; i<count; ++i)
	{
		//we don't take objects that are siblings of others
		bool isSiblingOfAnotherOne = false;
		for (size_t j=0; j<count; ++j)
		{
			if (i != j && origin[j]->isAncestorOf(origin[i]))
			{
				isSiblingOfAnotherOne = true;
				break;
			}
		}

		if (!isSiblingOfAnotherOne)
		{
			dest.addChild(origin[i],dependencyFlags);
		}
	}
}

#endif //CC_HIERARCHY_OBJECT_HEADER
