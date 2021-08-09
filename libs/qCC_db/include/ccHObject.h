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

#ifndef CC_HIERARCHY_OBJECT_HEADER
#define CC_HIERARCHY_OBJECT_HEADER

//Local
#include "ccObject.h"
#include "ccBBox.h"

class QIcon;

//! Hierarchical CloudCompare Object
class QCC_DB_LIB_API ccHObject : public ccObject, public ccDrawableObject
{
public: //construction

	//! Default constructor
	/** \param name object name (optional)
	    \param uniqueID unique ID (handle with care)
	**/
	ccHObject(const QString& name = QString(), unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);
	//! Copy constructor
	ccHObject(const ccHObject& object);

	//! Default destructor
	~ccHObject() override;

	//! Static factory
	/** \warning Objects depending on other structures (such as meshes
		or polylines that should be linked with point clouds playing the
		role of vertices) are returned 'naked'.
		\param objectType object type
		\param name object name (optional)
		\return instantiated object (if type is valid) or 0
	**/
	static ccHObject* New(CC_CLASS_ENUM objectType, const char* name = nullptr);

	//! Static factory (version to be used by external plugin factories)
	/** Two strings are used as keys, one for the plugin name and one for the class name.
		Those strings will typically be saved as metadata of a custom object
	**/
	static ccHObject* New(const QString& pluginId, const QString& classId, const char* name = nullptr);

public: //base members access

	//! Returns class ID
	/** \return class unique ID
	**/
	inline CC_CLASS_ENUM getClassID() const override { return CC_TYPES::HIERARCHY_OBJECT; }

	//! Returns whether the instance is a group
	inline bool isGroup() const { return getClassID() == static_cast<CC_CLASS_ENUM>(CC_TYPES::HIERARCHY_OBJECT); }

	//! Returns parent object
	/** \return parent object (nullptr if no parent)
	**/
	inline ccHObject* getParent() const { return m_parent; }

	//! Returns the icon associated to this entity
	/** ccDBRoot will call this method: if an invalid icon is returned
		the default icon for that type will be used instead.
		\return invalid icon by default (to be re-implemented by child class)
	**/
	virtual QIcon getIcon() const;
	
public: //dependencies management

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
	int getDependencyFlagsWith(const ccHObject* otherObject) const;

	//! Removes any dependency flags with a given object
	/** \param otherObject other object
	**/
	void removeDependencyWith(ccHObject* otherObject);

	//! Removes a given dependency flag
	/** \param otherObject other object
		\param flag dependency flag to remove (see DEPENDENCY_FLAGS)
	**/
	void removeDependencyFlag(ccHObject* otherObject, DEPENDENCY_FLAGS flag);

public: //children management

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
	
	//! Returns the total number of children under this object recursively
	/** \return Number of children
	**/
	unsigned int getChildCountRecursive() const;

	//! Returns the ith child
	/** \param childPos child position
		\return child object (or nullptr if wrong position)
	**/
	inline ccHObject* getChild(unsigned childPos) const { return (childPos < getChildrenNumber() ? m_children[childPos] : nullptr); }

	//! Finds an entity in this object hierarchy
	/** \param uniqueID child unique ID
		\return child (or nullptr if not found)
	**/
	ccHObject* find(unsigned uniqueID) const;

	//! Standard instances container (for children, etc.)
	using Container = std::vector<ccHObject *>;

	//! Shared pointer
	using Shared = QSharedPointer<ccHObject>;

	//! Shared instances container (for children, etc.)
	using SharedContainer = std::vector<Shared>;

	//! Collects the children corresponding to a certain pattern
	/** \param filteredChildren result container
		\param recursive specifies if the search should be recursive
		\param filter pattern for children selection
		\param strict whether the search is strict on the type (i.e 'isA') or not (i.e. 'isKindOf')
		\param inDisplay [optional] display in which the children are displayed
		\return number of collected children
	**/
	unsigned filterChildren(Container& filteredChildren,
							bool recursive = false,
							CC_CLASS_ENUM filter = CC_TYPES::OBJECT,
							bool strict = false,
							ccGenericGLDisplay* inDisplay = nullptr) const;

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
	void detachAllChildren();

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
	ccHObject* getFirstChild() const { return (m_children.empty() ? nullptr : m_children.front()); }
	//! Shortcut: returns last child
	ccHObject* getLastChild() const { return (m_children.empty() ? nullptr : m_children.back()); }

	//! Returns true if the current object is an ancestor of the specified one
	bool isAncestorOf(const ccHObject *anObject) const;

public: //bounding-box

	//! Returns the entity's own bounding-box (with local/shifted coordinates)
	/** Children bounding-boxes are ignored.
		\param withGLFeatures whether to take into account display-only elements (if any)
		\return bounding-box
	**/
	virtual ccBBox getOwnBB(bool withGLFeatures = false);

	//! Returns the local bounding-box of this entity and it's children
	/** \param withGLFeatures whether to take into account display-only elements (if any)
		\param onlyEnabledChildren only consider the 'enabled' children
		\return bounding-box
	**/
	virtual ccBBox getBB_recursive(bool withGLFeatures = false, bool onlyEnabledChildren = true);

	//! Global (non-shifted) bounding-box
	using GlobalBoundingBox = CCCoreLib::BoundingBoxTpl<double>;

	//! Returns the entity's own global bounding-box (with global/non-shifted coordinates - if relevant) 
	/** Children bounding-boxes are ignored.
		May differ from the (local) bounding-box if the entity is shifted
		\param withGLFeatures whether to take into account display-only elements (if any)
		\return global bounding-box
	**/
	virtual GlobalBoundingBox getOwnGlobalBB(bool withGLFeatures = false);

	//! Returns the entity's own global bounding-box (with global/non-shifted coordinates - if relevant) 
	/** Children bounding-boxes are ignored.
		By default this method returns the local bounding-box!
		But it may differ from the (local) bounding-box if the entity is shifted.
		\param[out] minCorner min global bounding-box corner
		\param[out] maxCorner max global bounding-box corner
		\return whether the bounding box is valid or not
	**/
	virtual bool getOwnGlobalBB(CCVector3d& minCorner, CCVector3d& maxCorner);

	//! Returns the global bounding-box of this entity and it's children
	/** \param withGLFeatures whether to take into account display-only elements (if any)
		\param onlyEnabledChildren only consider the 'enabled' children
		\return bounding-box
	**/
	virtual GlobalBoundingBox getGlobalBB_recursive(bool withGLFeatures = false, bool onlyEnabledChildren = true);

	//! Returns the bounding-box of this entity and it's children WHEN DISPLAYED
	/** Children's GL transformation is taken into account (if enabled).
		\param relative whether the bounding-box is relative (i.e. in the entity's local coordinate system) or absolute (in which case the parent's GL transformation will be taken into account)
		\param display if not null, this method will return the bounding-box of this entity (and its children) in the specified 3D view (i.e. potentially not visible)
		\return bounding-box
	**/
	virtual ccBBox getDisplayBB_recursive(bool relative, const ccGenericGLDisplay* display = nullptr);

	//! Returns best-fit bounding-box (if available)
	/** \warning Only suitable for leaf objects (i.e. without children)
		Therefore children bboxes are always ignored.
		\warning This method is not supported by all entities!
		(returns the axis-aligned bounding-box by default).
		\param[out] trans associated transformation (so that the bounding-box can be displayed in the right position!)
		\return fit bounding-box
	**/
	inline virtual ccBBox getOwnFitBB(ccGLMatrix& trans) { trans.toIdentity(); return getOwnBB(); }

	//! Draws the entity (and its children) bounding-box
	virtual void drawBB(CC_DRAW_CONTEXT& context, const ccColor::Rgb& col);

public: //display

	//Inherited from ccDrawableObject
	void draw(CC_DRAW_CONTEXT& context) override;

	//! Returns the absolute transformation (i.e. the actual displayed GL transformation) of an entity
	/** \param[out] trans absolute transformation
		\return whether a GL transformation is actually enabled or not
	**/
	bool getAbsoluteGLTransformation(ccGLMatrix& trans) const;

	//! Returns whether the object is actually displayed (visible) or not
	virtual bool isDisplayed() const;

	//! Returns whether the object is actually displayed (visible) in a given display or not
	virtual bool isDisplayedIn(const ccGenericGLDisplay* display) const;

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

	//recursive equivalents of some of ccDrawableObject methods
	ccHObject_recursive_call1(setSelected, bool, setSelected_recursive)
	ccHObject_recursive_call1(setDisplay, ccGenericGLDisplay*, setDisplay_recursive)
	ccHObject_recursive_call1(removeFromDisplay, ccGenericGLDisplay*, removeFromDisplay_recursive)
	ccHObject_recursive_call0(prepareDisplayForRefresh, prepareDisplayForRefresh_recursive)
	ccHObject_recursive_call1(refreshDisplay, bool, refreshDisplay_recursive)
	ccHObject_recursive_call0(resetGLTransformationHistory, resetGLTransformationHistory_recursive)
	ccHObject_recursive_call0(toggleActivation, toggleActivation_recursive)
	ccHObject_recursive_call0(toggleVisibility, toggleVisibility_recursive)
	ccHObject_recursive_call0(toggleColors, toggleColors_recursive)
	ccHObject_recursive_call0(toggleNormals, toggleNormals_recursive)
	ccHObject_recursive_call0(toggleSF, toggleSF_recursive)
	ccHObject_recursive_call0(toggleShowName, toggleShowName_recursive)
	ccHObject_recursive_call0(toggleMaterials, toggleMaterials_recursive)

	//! Transfers the entity from one display to the other
	inline virtual void transferDisplay(ccGenericGLDisplay* oldDisplay, ccGenericGLDisplay* newDisplay)
	{
		if (getDisplay() == oldDisplay)
		{
			setDisplay(newDisplay);
		}
	
		for (auto child : m_children)
		{
			child->transferDisplay(oldDisplay, newDisplay);
		}
	} 

	//! Returns the max 'unique ID' of this entity and its siblings
	unsigned findMaxUniqueID_recursive() const;

	//! Applies the active OpenGL transformation to the entity (recursive)
	/** The input ccGLMatrix should be left to 0, unless you want to apply
		a pre-transformation.
		\param trans a ccGLMatrix structure (reference to)
	**/
	void applyGLTransformation_recursive(const ccGLMatrix* trans = nullptr);

	//! Notifies all dependent entities that the geometry of this entity has changed
	virtual void notifyGeometryUpdate();

	//inherited from ccSerializableObject
	bool isSerializable() const override;
	bool toFile(QFile& out) const override;
	bool fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//! Custom version of ccSerializableObject::fromFile
	/** This is used to load only the object's part of a stream (and not its children)
		\param in input file (already opened)
		\param dataVersion file version
		\param flags deserialization flags (see ccSerializableObject::DeserializationFlags)
		\param oldToNewIDMap map to convert old IDs to new ones
		\return success
	**/
	bool fromFileNoChildren(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap);

	//! Returns whether object is shareable or not
	/** If object is father dependent and 'shared', it won't
		be deleted but 'released' instead.
	**/
	virtual inline bool isShareable() const { return false; }

	//! Behavior when selected
	enum SelectionBehavior { SELECTION_AA_BBOX,
							 SELECTION_FIT_BBOX,
							 SELECTION_IGNORED };

	//! Sets selection behavior (when displayed)
	/** \warning SELECTION_FIT_BBOX relies on the
		'ccDrawableObject::getFitBB' method (which
		is not supported by all entities).
	**/
	virtual inline void setSelectionBehavior(SelectionBehavior mode) { m_selectionBehavior = mode; }

	//! Returns selection behavior
	virtual inline SelectionBehavior getSelectionBehavior() const { return m_selectionBehavior; }

	//! Returns object unique ID used for display
	virtual inline unsigned getUniqueIDForDisplay() const { return getUniqueID(); }

	//! Returns the transformation 'history' matrix
	virtual inline const ccGLMatrix& getGLTransformationHistory() const { return m_glTransHistory; }
	//! Sets the transformation 'history' matrix (handle with care!)
	virtual inline void setGLTransformationHistory(const ccGLMatrix& mat) { m_glTransHistory = mat; }
	//! Resets the transformation 'history' matrix
	virtual inline void resetGLTransformationHistory() { m_glTransHistory.toIdentity(); }

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
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap);

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

	//! Parent
	ccHObject* m_parent;

	//! Children
	Container m_children;

	//! Selection behavior
	SelectionBehavior m_selectionBehavior;

	//! Dependencies map
	/** First parameter: other object
		Second parameter: dependency flags (see DEPENDENCY_FLAGS)
	**/
	std::map<ccHObject*, int> m_dependencies;

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
	for (size_t i = 0; i < count; ++i)
	{
		//we don't take objects that are siblings of others
		bool isSiblingOfAnotherOne = false;
		for (size_t j = 0; j < count; ++j)
		{
			if (i != j && origin[j]->isAncestorOf(origin[i]))
			{
				isSiblingOfAnotherOne = true;
				break;
			}
		}

		if (!isSiblingOfAnotherOne)
		{
			dest.addChild(origin[i], dependencyFlags);
		}
	}
}

#endif //CC_HIERARCHY_OBJECT_HEADER
