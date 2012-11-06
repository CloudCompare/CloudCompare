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
//$Rev:: 1916                                                              $
//$LastChangedDate:: 2011-10-14 18:46:32 +0200 (ven., 14 oct. 2011)        $
//**************************************************************************
//

#ifndef CC_GRAPHICAL_TRANSFORMATION_TOOLS_HEADER
#define CC_GRAPHICAL_TRANSFORMATION_TOOLS_HEADER

#include <QDialog>

#include <ui_graphicalTransformationDlg.h>

#include <ccGLMatrix.h>

#include <vector>

class ccGLWindow;
class ccHObject;

//! Dialog + mechanism for graphical transformation of entities
/** Mouse driven rotation and translation of selected entities at screen.
**/
class ccGraphicalTransformationTool : public QDialog, public Ui::GraphicalTransformationDlg
{
    Q_OBJECT

public:

    //! Default constructor
	ccGraphicalTransformationTool(QWidget* parent);
	//! Default destructor
	virtual ~ccGraphicalTransformationTool();

	//! Links this dialog with a given 3D window
    void linkWith(ccGLWindow* win);

    //! Adds an entity to the 'selected' entities set
    /** Only the 'selected' entities are moved.
        \return success, if the entitiy is elligible for graphical transformation
    **/
	bool addEntity(ccHObject* anObject);

    //! Returns
    unsigned getNumberOfValidEntities();

    //! Shows dialog and starts graphical interaction
	bool start();

    //! Hides dialog and ends graphical interaction
	void stop();

protected slots:

    //! Applies transformation to selected entities
    void apply();

    //! Resets transformation
    void reset();

    //! Cancels (no transformation is applied)
    void cancel();

    //! applies translation (graphically) to selected entities
    void glTranslate(const CCVector3&);

    //! applies rotation (graphically) to selected entities
    void glRotate(const ccGLMatrix&);

signals:

    //! Signal emitted when the process ends
    void transformationFinished(bool);

protected:

    //! Clear all variables and 'unlink' dialog
    void clear();

    //! Updates all selected entities GL transformation matrices
    void updateAllGLTransformations();

    //! List of entities to be transformed
    ccHObject*  m_toTransform;

    //! Associated window
    ccGLWindow* m_associatedWin;

    //! Current rotation
    ccGLMatrix  m_rotation;

    //! Current translation
    CCVector3   m_translation;

    //! Rotation center
    /** The rotation center is actually the center of gravity of the selected 'entities'
    **/
    CCVector3   m_rotationCenter;

	//! Start state
	bool		m_started;
};

#endif
