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

#ifndef CC_TRACE_POLY_LINE_TOOLS_HEADER
#define CC_TRACE_POLY_LINE_TOOLS_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//system
#include <set>

//GUI
#include <ui_tracePolyLineDlg.h>

class ccPolyline;
class ccPointCloud;
class ccGLWindow;

//! Graphical segmentation mechanism (with polyline)
class ccTracePolylineTool : public ccOverlayDialog, public Ui::TracePolyLineDlg {
    Q_OBJECT

public:
    //! Default constructor
    explicit ccTracePolylineTool(QWidget* parent);
    //! Destructor
    virtual ~ccTracePolylineTool();

    //! Get a pointer to the polyline that has been traced
    ccPolyline* getPolyLine() { return m_segmentationPoly; }

    //inherited from ccOverlayDialog
    virtual bool linkWith(ccGLWindow* win) override;
    virtual bool start() override;
    virtual void stop(bool accepted) override;

protected slots:
    void reset();

    //! do the actual polyline projection from 2d to 3d
    void projectPolyline(bool cpu = true);
    void apply();
    void cancel();

    void resetLine();

    void addPointToPolyline(int x, int y);
    void closePolyLine(int x = 0, int y = 0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
    void updatePolyLine(int x, int y, Qt::MouseButtons buttons);

    void linkSnapDimensions(const int status);

    //! To capture overridden shortcuts (pause button, etc.)
    void onShortcutTriggered(int);

protected:
    void doPolylineOverSampling(const int multiplicity);

    //! Whether something has changed or not (for proper 'cancel')
    bool m_somethingHasChanged;

    //! Segmentation polyline
    ccPolyline* m_segmentationPoly;
    //! Segmentation polyline vertices
    ccPointCloud* m_polyVertices;

    //! Current process state
    bool m_done = false;

};

#endif //CC_GRAPHICAL_SEGMENTATION_TOOLS_HEADER
