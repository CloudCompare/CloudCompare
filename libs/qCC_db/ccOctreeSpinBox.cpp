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

#include "ccOctreeSpinBox.h"

//CCLib
#include <CCMiscTools.h>

//qCC_db
#include <ccBBox.h>
#include <ccOctree.h>
#include <ccGenericPointCloud.h>

ccOctreeSpinBox::ccOctreeSpinBox(QWidget* parent/*=0*/)
	: QSpinBox(parent)
	, m_octreeBoxWidth(0)
{
	setRange(0, CCLib::DgmOctree::MAX_OCTREE_LEVEL);
	
	//we'll catch any modification of the spinbox value and update the suffix consequently
	connect(this, static_cast<void (ccOctreeSpinBox::*)(int)>(&ccOctreeSpinBox::valueChanged), this, &ccOctreeSpinBox::onValueChange);
}

void ccOctreeSpinBox::setCloud(ccGenericPointCloud* cloud)
{
	if (!cloud)
	{
		assert(false);
		return;
	}

	if (cloud->getOctree())
	{
		setOctree(cloud->getOctree().data());
	}
	else
	{
		ccBBox box = cloud->getOwnBB(false);
		CCLib::CCMiscTools::MakeMinAndMaxCubical(box.minCorner(), box.maxCorner());
		m_octreeBoxWidth = box.getMaxBoxDim();
		onValueChange(value());
	}
}

void ccOctreeSpinBox::setOctree(CCLib::DgmOctree* octree)
{
	if (octree)
	{
		m_octreeBoxWidth = static_cast<double>(octree->getCellSize(0));
		onValueChange(value());
	}
	else
	{
		m_octreeBoxWidth = 0;
		setSuffix(QString());
	}
}

void ccOctreeSpinBox::onValueChange(int level)
{
	if (m_octreeBoxWidth > 0)
	{
		if (level >= 0/* && level <= CCLib::DgmOctree::MAX_OCTREE_LEVEL*/)
		{
			double cs = m_octreeBoxWidth / pow(2.0, static_cast<double>(level));
			setSuffix(QString(" (grid step = %1)").arg(cs));
		}
		else
		{
			//invalid level?!
			setSuffix(QString());
		}
	}
}
