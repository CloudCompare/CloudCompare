//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: FFDPlugin                       #
//#           Free Form Deformation - Non-rigid Transformation             #
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
//##########################################################################

#include "FFDAction.h"
#include "FFDLattice.h"
#include "FFDLatticeParamsDlg.h"
#include "ccFFDLatticeDisplay.h"
#include "ccFFDDeformationTool.h"

#include <ccMainAppInterface.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccGLWindowInterface.h>
#include <ccHObject.h>
#include <ReferenceCloud.h>

#include <QObject>
#include <algorithm>

namespace FFDAction
{
	void performDeformation( ccMainAppInterface *appInterface )
	{
		if ( appInterface == nullptr )
		{
			Q_ASSERT( false );
			return;
		}

		/*** HERE STARTS THE ACTION ***/

		// Get the selected cloud
		ccHObject::Container selectedEntities = appInterface->getSelectedEntities();

		if ( selectedEntities.empty() )
		{
			appInterface->dispToConsole( "[FFD] No entity selected", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
			return;
		}

		// Find the first point cloud
		ccPointCloud *cloud = nullptr;
		for ( ccHObject *entity : selectedEntities )
		{
			if ( entity->isA( CC_TYPES::POINT_CLOUD ) )
			{
				cloud = static_cast<ccPointCloud*>( entity );
				break;
			}
		}

		if ( cloud == nullptr )
		{
			appInterface->dispToConsole( "[FFD] No point cloud selected", ccMainAppInterface::ERR_CONSOLE_MESSAGE );
			return;
		}

		// Show dialog to get lattice parameters
		FFDLatticeParamsDlg paramsDlg(nullptr);
		if (paramsDlg.exec() != QDialog::Accepted)
		{
			appInterface->dispToConsole( "[FFD] Cancelled by user", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
			return;
		}

		// Create the FFD lattice with user-specified dimensions
		std::array<unsigned int, 3> latticeSize = paramsDlg.getLatticeSize();
		FFDLattice* lattice = new FFDLattice(latticeSize, cloud->getOwnBB());

		// Create a subsampled preview cloud for smooth interactive updates
		size_t fullSize = cloud->size();
		size_t targetPreviewSize = 50000; // stricter preview size for responsiveness
		size_t step = (fullSize > targetPreviewSize && targetPreviewSize > 0) ? (fullSize / targetPreviewSize) : 1;
		step = std::max<size_t>(step, 1);

		CCCoreLib::ReferenceCloud previewRef(cloud);
		previewRef.reserve(static_cast<unsigned>(fullSize / step + 1));
		for (size_t i = 0; i < fullSize; i += step)
		{
			previewRef.addPointIndex(static_cast<unsigned>(i));
		}

		ccPointCloud* previewCloud = cloud->partialClone(&previewRef);
		if (!previewCloud)
		{
			// Fallback: create a minimal preview cloud if cloning failed
			previewCloud = new ccPointCloud("FFD Preview");
			previewCloud->reserve(previewRef.size());
			for (unsigned i = 0; i < previewRef.size(); ++i)
			{
				const CCVector3* P = cloud->getPoint(previewRef.getPointGlobalIndex(i));
				previewCloud->addPoint(*P);
			}
		}
		previewCloud->setName(QString("%1 (FFD Preview)").arg(cloud->getName()));
		previewCloud->setVisible(true);
		previewCloud->setEnabled(true);
		previewCloud->setPointSize(cloud->getPointSize());

		// Visualize lattice
		ccFFDLatticeDisplay* latticeDisplay = new ccFFDLatticeDisplay(cloud->getOwnBB(), latticeSize, lattice->getAllControlPoints());
		latticeDisplay->setName("FFD Lattice");

		// Create a group entity to host the preview and lattice
		ccHObject* ffdGroup = new ccHObject("FFD");
		ffdGroup->addChild(previewCloud);
		ffdGroup->addChild(latticeDisplay);

		appInterface->addToDB(ffdGroup, false, true, false, true);
		appInterface->refreshAll();

		appInterface->dispToConsole(
			QString("[FFD] Preview cloud size: %1 / %2 points")
				.arg(previewCloud->size())
				.arg(fullSize),
			ccMainAppInterface::STD_CONSOLE_MESSAGE
		);

		appInterface->dispToConsole(
			QString( "[FFD] Created lattice with %1x%2x%3 control points" )
				.arg( latticeSize[0] )
				.arg( latticeSize[1] )
				.arg( latticeSize[2] ),
			ccMainAppInterface::STD_CONSOLE_MESSAGE
		);

		// Create and setup the interactive deformation tool
		ccFFDDeformationTool* tool = new ccFFDDeformationTool(cloud, previewCloud, appInterface);
		tool->setLattice(lattice, latticeDisplay);

		ccGLWindowInterface* win = appInterface->getActiveGLWindow();
		if (!win)
		{
			appInterface->dispToConsole( "[FFD] No active 3D window", ccMainAppInterface::ERR_CONSOLE_MESSAGE );
			delete tool;
			return;
		}

		tool->linkWith(win);
		tool->start();

		QObject::connect(tool, &ccOverlayDialog::processFinished, [tool](bool)
		{
			tool->deleteLater();
		});

		appInterface->dispToConsole( "[FFD] Interactive tool activated. Press X/Y/Z to constrain movement to an axis.",
									  ccMainAppInterface::STD_CONSOLE_MESSAGE );
		appInterface->dispToConsole( "[FFD] Press Enter to apply deformation, R to reset.",
									  ccMainAppInterface::STD_CONSOLE_MESSAGE );

		/*** HERE ENDS THE ACTION ***/
	}
}
