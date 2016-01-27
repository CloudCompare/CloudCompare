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

#ifndef CC_TRACE_POLY_LINE_TOOL_HEADER
#define CC_TRACE_POLY_LINE_TOOL_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//system
#include <set>

//GUI
#include <ui_tracePolylineDlg.h>

class ccPolyline;
class ccPointCloud;
class ccGLWindow;

//! Graphical Polyline Tracing tool
class ccTracePolylineTool : public ccOverlayDialog, public Ui::TracePolyLineDlg
{
    Q_OBJECT

public:
    //! Default constructor
    explicit ccTracePolylineTool(QWidget* parent);
    //! Destructor
    virtual ~ccTracePolylineTool();

    //inherited from ccOverlayDialog
    virtual bool linkWith(ccGLWindow* win) override;
    virtual bool start() override;
    virtual void stop(bool accepted) override;

protected slots:

    void apply();
    void cancel();
    void resetLine();
	void exportLine();

	void handlePickedItem(int, unsigned, int, int);
    //void addPointToPolyline(int x, int y);
    void closePolyLine(int x = 0, int y = 0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
    void updatePolyLineTip(int x, int y, Qt::MouseButtons buttons);

	void onSnapSizeChanged(int);
	void onWidthSizeChanged(int);

    //! To capture overridden shortcuts (pause button, etc.)
    void onShortcutTriggered(int);

protected:

	//! Oversamples a polyline
	static ccPolyline* PolylineOverSampling(ccPolyline* polyline, ccPointCloud* vertices, unsigned steps);

    //! 2D polyline (for the currently edited part)
    ccPolyline* m_polyTip;
    //! 2D polyline vertices
    ccPointCloud* m_polyTipVertices;

    //! 3D polyline
    ccPolyline* m_poly3D;
    //! 3D polyline vertices
    ccPointCloud* m_poly3DVertices;

    //! Current process state
    bool m_done;

};

#endif //CC_TRACE_POLY_LINE_TOOL_HEADER
