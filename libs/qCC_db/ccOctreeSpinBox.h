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

#ifndef CC_OCTREE_SPIN_BOX_HEADER
#define CC_OCTREE_SPIN_BOX_HEADER

//Local
#include "qCC_db.h"

//CCLib
#include <DgmOctree.h>

//Qt
#include <QSpinBox>

class ccGenericPointCloud;

//! Octree level editor dialog
class QCC_DB_LIB_API ccOctreeSpinBox : public QSpinBox
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccOctreeSpinBox(QWidget* parent = 0);

	//! Inits the dialog with a cloud (on which the octree has been or will be computed)
	/** Alternative to ccOctreeSpinBox::setOctree
	**/
	void setCloud(ccGenericPointCloud* cloud);

	//! Inits the dialog with an octree
	/** Alternative to ccOctreeSpinBox::setCloud
	**/
	void setOctree(CCLib::DgmOctree* octree);

protected slots:

	//! Called each time the spinbox value changes
	void onValueChange(int);

protected:

	//! Corresponding octree base size
	double m_octreeBoxWidth;

};

#endif //CC_OCTREE_SPIN_BOX_HEADER
