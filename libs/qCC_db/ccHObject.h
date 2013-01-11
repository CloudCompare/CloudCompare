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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2241                                                              $
//$LastChangedDate:: 2012-09-21 23:22:39 +0200 (ven., 21 sept. 2012)       $
//**************************************************************************
//

#ifndef CC_HIERARCHY_OBJECT_HEADER
#define CC_HIERARCHY_OBJECT_HEADER

//Local
#include "ccObject.h"
#include "ccDrawableObject.h"

//System
#include <vector>

//! Hierarchical CloudCompare Object
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccHObject : public ccObject, public ccDrawableObject
#else
class ccHObject : public ccObject, public ccDrawableObject
#endif
{
public:

    //! Default constructor
    /** \param name object name (optional)
    **/
    ccHObject(QString name=QString());

    //! Default destructor
    virtual ~ccHObject();

	//! Static factory
	/** Warning: objects depending on other structures (such as meshes 
		or polylines that should be linked with point clouds playing the
		role of vertices) are returned 'naked'.
		\param objectType object type (see CC_CLASS_ENUM)
		\param name object name (optional)
		\return instantiated object (if type is valid) or 0
	**/
	static ccHObject* New(unsigned objectType, const char* name=0);

    //! Returns class ID
    /** \return class unique ID
    **/
    virtual CC_CLASS_ENUM getClassID() const {return CC_HIERARCHY_OBJECT;};

    //! Returns parent object
    /** \return parent object (NULL if no parent)
    **/
	inline ccHObject* getParent() const { return m_parent; }

    /*** children management ***/

    //! Adds a child to object's children list
    /** \param anObject child
        \param dependant specifies if the child object should be deleted with its parent
		\param insertIndex insertion index (if <0, item is simply appended to the children list)
    **/
    virtual void addChild(ccHObject* anObject, bool dependant=true, int insertIndex=-1);

    //! Returns the number of children
    /** \return children number
    **/
	inline unsigned getChildrenNumber() const { return m_children.size(); }

    //! Returns the ith child
    /** \param childPos child position
        \return child object (or NULL if wrong position)
    **/
	inline ccHObject* getChild(unsigned childPos) const { return (childPos < m_children.size() ? m_children[childPos] : 0); }

	//! Finds an entity in this object hierarchy
	/** \param uniqueID child unique ID
		\return child (or NULL if not found)
	**/
	ccHObject* find(int uniqueID);

	//! standard ccHObject container (for children, etc.)
	typedef std::vector<ccHObject*> Container;

    //! Collects the children corresponding to a certain pattern
    /** \param filteredChildren result container
        \param recursive specifies if the search should be recursive
        \param filter pattern for children selection
        \return number of collected children
    **/
    unsigned filterChildren(Container& filteredChildren, bool recursive=false, CC_CLASS_ENUM filter = CC_OBJECT) const;

    //! Removes a specific child
    void removeChild(const ccHObject* anObject);
    //! Removes a specific child given its index
    void removeChild(int pos);
    //! Removes all children
    void removeAllChildren();
    //! Returns child index
    int getChildIndex(const ccHObject* aChild) const;
	//! Swaps two children
	void swapChildren(unsigned firstChildIndex, unsigned secondChildIndex);
    //! Returns index relatively to its parent or -1 if no parent
    int getIndex() const;

	//! Detaches entity from parent
	void detachFromParent();

	//! Transfer a given child to another parent
	void transferChild(unsigned index, ccHObject& newParent);
	//! Transfer all children to another parent
	void transferChildren(ccHObject& newParent, bool forceFatherDependent = false);

    //! Shortcut: returns first child
	ccHObject* getFirstChild() const { return (m_children.empty() ? 0 : m_children.front()); }
    //! Shortcut: returns last child
	ccHObject* getLastChild() const { return (m_children.empty() ? 0 : m_children.back()); }

    //! Returns true if the current object is an ancestor of the specified one
    bool isAncestorOf(const ccHObject *anObject) const;

    //Inherited from ccDrawableObject
    virtual ccBBox getBB(bool relative=true, bool withGLfeatures=false, const ccGenericGLDisplay* window=NULL);
    virtual void draw(CC_DRAW_CONTEXT& context);

	/*** RECURSIVE CALL SCRIPTS ***/
	
	//0 parameter
	#define recursive_call0(baseName,recursiveName) \
	inline virtual void recursiveName() \
	{ \
		baseName(); \
		for (Container::iterator it = m_children.begin(); it!=m_children.end(); ++it) \
			(*it)->recursiveName(); \
	} \

	//0 parameter (with exception: mesh groups already have a recursive behavior for some methods!)
	#define recursive_call0_ex(baseName,recursiveName) \
	inline virtual void recursiveName() \
	{ \
		baseName(); \
		if (getClassID() != CC_MESH_GROUP) \
			for (Container::iterator it = m_children.begin(); it!=m_children.end(); ++it) \
				(*it)->recursiveName(); \
	} \

	//1 parameter
	#define recursive_call1(baseName,param1Type,recursiveName) \
	inline virtual void recursiveName(param1Type p) \
	{ \
		baseName(p); \
		for (Container::iterator it = m_children.begin(); it!=m_children.end(); ++it) \
			(*it)->recursiveName(p); \
	} \

	/*****************************/

	//recursive equivalents of some of ccDrawableObject methods
	recursive_call1(setSelected,bool,setSelected_recursive);
	recursive_call1(setDisplay,ccGenericGLDisplay*,setDisplay_recursive);
	recursive_call1(removeFromDisplay,ccGenericGLDisplay*,removeFromDisplay_recursive);
	recursive_call0(prepareDisplayForRefresh,prepareDisplayForRefresh_recursive);
	recursive_call0(refreshDisplay,refreshDisplay_recursive);
	recursive_call0_ex(toggleVisibility,toggleVisibility_recursive);
	recursive_call0_ex(toggleColors,toggleColors_recursive);
	recursive_call0_ex(toggleNormals,toggleNormals_recursive);
	recursive_call0_ex(toggleSF,toggleSF_recursive);

    //! Applies the active OpenGL transformation to the entity (recursive)
    /** The input ccGLMatrix should be left to 0, unless you want to apply
        a pre-transformation.
        \param trans a ccGLMatrix structure (reference to)
    **/
    void applyGLTransformation_recursive(ccGLMatrix* trans = 0);

    //! Returns the bounding-box center
    /** \return bounding-box center
    **/
    virtual CCVector3 getCenter();

    //! Returns last modification time
    /** \return last modification time
    **/
	inline int getLastModificationTime() const { return m_lastModificationTime_ms; }

    //! Returns last modification time (recursive)
    /** \return last modification time
    **/
    int getLastModificationTime_recursive() const;

    //! Updates modification time
    void updateModificationTime();

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
	virtual bool fromFile(QFile& in, short dataVersion);

	//! Returns whether object is shareable or not
	/** If object is father dependent and 'shared', it won't
		be deleted but 'released' instead.
	**/
	virtual bool isShareable() const { return false; }

protected:

    //! Sets parent object
	virtual inline void setParent(ccHObject* anObject) { m_parent = anObject; }

    //! Draws the entity only (not its children)
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) { /*does nothing by default*/ };

    //! Applies a GL transformation to the entity
    /** this = rotMat*(this-rotCenter)+(rotCenter+trans)
        \param trans a ccGLMatrix structure
    **/
    virtual void applyGLTransformation(const ccGLMatrix& trans) { /*does nothing by default*/ };

	//! Save own object data
	/** Called by 'toFile' (recursive scheme)
		To be overloaded (but still called;) by subclass.
	**/
	virtual bool toFile_MeOnly(QFile& out) const;

	//! Loads own object data
	/** Called by 'fromFile' (recursive scheme)
		To be overloaded (but still called;) by subclass.
	**/
	virtual bool fromFile_MeOnly(QFile& out, short dataVersion);

    //! Object's parent
    ccHObject* m_parent;

    //! Object's children
    Container m_children;

    //! Last modification time (ms)
    int m_lastModificationTime_ms;
};

/*** Helpers ***/

//! standard ccHObject container (for children, etc.)
static void RemoveSiblings(const ccHObject::Container& origin, ccHObject::Container& dest)
{
	unsigned count = origin.size();
	for (unsigned i=0;i<count;++i)
	{
		//we don't take objects that are siblings of others
		bool isSiblingOfAnotherOne = false;
		for (unsigned j=0;j<count;++j)
		{
			if (i != j && origin[j]->isAncestorOf(origin[i]))
			{
				isSiblingOfAnotherOne = true;
				break;
			}
		}

		if (!isSiblingOfAnotherOne)
			dest.push_back(origin[i]);
	}
}


#endif
