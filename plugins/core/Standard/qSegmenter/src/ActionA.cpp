#include "ActionA.h"
#include "SeedPicker.h"
#include "SegmenterDlg.h"
#include "ccMainAppInterface.h"
#include "ccPointCloud.h"

#include <QMainWindow>

namespace Example
{
    static SeedPicker*  g_seedPicker = nullptr;
    static SegmenterDlg* g_dialog    = nullptr;

    void performActionA(ccMainAppInterface* appInterface)
    {
        if (appInterface == nullptr) return;

        const ccHObject::Container& selected = appInterface->getSelectedEntities();
        for (ccHObject* obj : selected)
        {
            if (obj->isA(CC_TYPES::POINT_CLOUD))
            {
                ccPointCloud* cloud = static_cast<ccPointCloud*>(obj);
                if (!cloud->getOctree()) cloud->computeOctree();
                break;
            }
        }

        if (g_dialog == nullptr)
        {
            g_dialog     = new SegmenterDlg(appInterface->getMainWindow());
            g_seedPicker = new SeedPicker(appInterface, g_dialog);

            g_dialog->setAttribute(Qt::WA_DeleteOnClose);
            g_dialog->setStatusMessage("Click a point to start segmentation");
            g_seedPicker->startListening();

            // All connections are set up once here, avoiding duplicate signals
            // on repeated menu invocations while the dialog is already open.
            QObject::connect(g_dialog, &QObject::destroyed, [appInterface]() {
                g_dialog = nullptr;
                if (g_seedPicker) {
                    g_seedPicker->stopListening();
                    delete g_seedPicker;
                    g_seedPicker = nullptr;
                }
            });

            QObject::connect(g_dialog, &SegmenterDlg::stateChanged, []() {
                if (g_seedPicker && g_dialog)
                    g_seedPicker->setPositiveMode(g_dialog->isAddingPositiveSeeds());
            });
            QObject::connect(g_dialog, &SegmenterDlg::applyRequested, []() {
                if (g_seedPicker) g_seedPicker->runRegionGrowing();
            });
            QObject::connect(g_dialog, &SegmenterDlg::clearRequested, []() {
                if (g_seedPicker) g_seedPicker->clearAll();
            });
            QObject::connect(g_dialog, &SegmenterDlg::undoRequested, []() {
                if (g_seedPicker) g_seedPicker->undo();
            });
            QObject::connect(g_dialog, &SegmenterDlg::redoRequested, []() {
                if (g_seedPicker) g_seedPicker->redo();
            });
            QObject::connect(g_dialog, &SegmenterDlg::exportRequested, []() {
                if (g_seedPicker) g_seedPicker->exportSegmentation();
            });
        }

        g_seedPicker->setPositiveMode(g_dialog->isAddingPositiveSeeds());

        g_dialog->show();
        g_dialog->raise();
        g_dialog->activateWindow();
    }
}
