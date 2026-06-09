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
                if (!cloud->getOctree()) {
                    cloud->computeOctree();
                }
                break;
            }
        }

        if (g_dialog == nullptr)
        {
            g_dialog = new SegmenterDlg(appInterface->getMainWindow());
            g_dialog->setAttribute(Qt::WA_DeleteOnClose);
            g_dialog->setStatusMessage("Select a first point to start segmentation");

            QObject::connect(g_dialog, &QObject::destroyed, [appInterface]() {
                g_dialog = nullptr;
                if (g_seedPicker) {
                    g_seedPicker->stopListening();
                    delete g_seedPicker;
                    g_seedPicker = nullptr;
                }
            });
        }

        if (g_seedPicker == nullptr)
        {
            g_seedPicker = new SeedPicker(appInterface, g_dialog);
            g_seedPicker->startListening();
        }

        g_seedPicker->setPositiveMode(g_dialog->isAddingPositiveSeeds());

        // Connect changes to radio button mode toggles
        QObject::connect(g_dialog, &SegmenterDlg::stateChanged, []() {
            if (g_seedPicker && g_dialog) {
                g_seedPicker->setPositiveMode(g_dialog->isAddingPositiveSeeds());
            }
        });

        // Trigger the region growing execution pipeline whenever the Apply button is pressed
        QObject::connect(g_dialog, &SegmenterDlg::applyRequested, []() {
            if (g_seedPicker) {
                g_seedPicker->runRegionGrowing();
            }
        });

        g_dialog->show();
        g_dialog->raise();
        g_dialog->activateWindow();
    }
}