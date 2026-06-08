#include "ActionA.h"
#include "SeedPicker.h"
#include "SegmenterDlg.h"
#include "ccMainAppInterface.h"
#include "ccPointCloud.h"

#include <QMainWindow>

namespace Example
{
	static SeedPicker* g_seedPicker = nullptr;
	static SegmenterDlg* g_dialog   = nullptr;

	void performActionA(ccMainAppInterface* appInterface)
	{
		if (appInterface == nullptr) return;

		const ccHObject::Container& selected = appInterface->getSelectedEntities();
		for (ccHObject* obj : selected)
		{
			if (obj->isA(CC_TYPES::POINT_CLOUD))
			{
				ccPointCloud* cloud = static_cast<ccPointCloud*>(obj);
				if (!cloud->getOctree())
				{
					appInterface->dispToConsole("[Segmenter] Computing Octree...", ccMainAppInterface::STD_CONSOLE_MESSAGE);
					cloud->computeOctree();
				}
				break; 
			}
		}

		if (g_dialog == nullptr)
		{
			g_dialog = new SegmenterDlg(appInterface->getMainWindow());
			g_dialog->setAttribute(Qt::WA_DeleteOnClose);

            // Set initial state text
            g_dialog->setStatusMessage("Select a first point to start segmentation");

			QObject::connect(g_dialog, &QObject::destroyed, [appInterface]() {
                g_dialog = nullptr; 
                if (g_seedPicker) {
                    g_seedPicker->stopListening();
                    delete g_seedPicker;
                    g_seedPicker = nullptr;
                }
                appInterface->dispToConsole("[Segmenter] Dialog closed.", ccMainAppInterface::STD_CONSOLE_MESSAGE); 
            });
		}

		if (g_seedPicker == nullptr)
		{
			g_seedPicker = new SeedPicker(appInterface);
			g_seedPicker->startListening();
		}

		g_seedPicker->setPositiveMode(g_dialog->isAddingPositiveSeeds());

        // Master Lambda: Runs whenever a UI state changes or a point is clicked
        QObject::connect(g_dialog, &SegmenterDlg::stateChanged, [appInterface]()
        {
            if (g_seedPicker && g_dialog) {
                g_seedPicker->setPositiveMode(g_dialog->isAddingPositiveSeeds());
                
                double tau = g_dialog->getThreshold(); 
                double ws = g_dialog->getSpatialWeight();
                double wc = g_dialog->getChromaticWeight();
                double radius = 0.1; // Hardcoded radius for now (adjust as needed)
                
                int count = g_seedPicker->runRegionGrowing(radius, tau, ws, wc);
                
                if (count > 0) {
                    g_dialog->setStatusMessage("Segmentation updated.");
                    g_dialog->setPointCount(count);
                }
            } 
        });

		g_dialog->show();
		g_dialog->raise();          
		g_dialog->activateWindow(); 
	}
}