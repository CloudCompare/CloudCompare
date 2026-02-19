#pragma once

#include <ccOverlayDialog.h>
#include <ccHObject.h>
#include <ccGLWindowInterface.h>
#include <CCGeom.h>
#include <deque>

class ccPointCloud;
class ccMainAppInterface;
class FFDLattice;
class ccFFDLatticeDisplay;
class ccSelectionRectangle;
class ccFFDDeformationApplier;

class ccFFDDeformationTool : public ccOverlayDialog
{
    Q_OBJECT

public:
    ccFFDDeformationTool(ccPointCloud* originalCloud, ccPointCloud* previewCloud, ccMainAppInterface* appInterface);
    ~ccFFDDeformationTool();

    void setLattice(FFDLattice* lattice, ccFFDLatticeDisplay* display);

    bool start() override;
    void stop(bool accepted) override;

private Q_SLOTS:
    void onLeftButtonClicked(int x, int y);
    void onMouseMoved(int x, int y, Qt::MouseButtons buttons);
    void onButtonReleased();
    void onItemPicked(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P, const CCVector3d& uvw);
    void onShortcutTriggered(int key);

private:
    void updateControlPointCloud();
    CCVector3d applyAxisConstraint(const CCVector3d& delta) const;
    void selectPointsInRectangle(int x1, int y1, int x2, int y2);
    bool projectPointToScreen(const CCVector3d& point3D, int& screenX, int& screenY) const;
    void updateCloudDeformation();
    void pushLatticeHistory();           // Save current lattice state to history
    void undoLastTransformation();       // Restore previous lattice state

    ccPointCloud* m_originalCloud = nullptr;            // Original full-resolution point cloud
    ccPointCloud* m_previewCloud = nullptr;             // Subsampled preview point cloud
    ccMainAppInterface* m_appInterface = nullptr;
    FFDLattice* m_lattice = nullptr;
    ccFFDLatticeDisplay* m_latticeDisplay = nullptr;
    ccPointCloud* m_controlPointCloud = nullptr;        // Local picking cloud for control points
    ccSelectionRectangle* m_selectionRect = nullptr;    // Visual rectangle for multi-selection
    ccFFDDeformationApplier* m_previewApplier = nullptr;      // Applies FFD to preview cloud
    ccFFDDeformationApplier* m_fullApplier = nullptr;         // Applies FFD to full cloud on Apply

    int m_selectedPointIndex = -1;                      // Index of selected control point (-1 = none)
    std::vector<int> m_selectedPointIndices;            // Multiple selected control points
    std::vector<CCVector3d> m_dragStartPointPositions;  // Starting positions for all selected points
    CCVector3d m_dragStartPointPos;                     // Position of selected point at drag start
    CCVector3d m_dragStartClickPos;                     // 3D click position at drag start
    int m_lastMouseX = 0;
    int m_lastMouseY = 0;

    // Rectangle selection
    bool m_isDrawingRectangle = false;
    int m_rectStartX = 0;
    int m_rectStartY = 0;
    int m_rectEndX = 0;
    int m_rectEndY = 0;

    enum AxisConstraint { NONE, X_AXIS, Y_AXIS, Z_AXIS };
    AxisConstraint m_axisConstraint = NONE;            // Current axis constraint
    bool m_axisConstraintLocked = false;               // Axis constraint lock (Ctrl+Shift+X/Y/Z)

    bool m_isDragging = false;                          // Is user currently dragging

    // Undo history
    std::deque<std::vector<CCVector3d>> m_latticeHistory;   // Stack of previous lattice states
    static constexpr size_t MAX_HISTORY_SIZE = 50;          // Maximum undo levels

    ccGLWindowInterface::INTERACTION_FLAGS m_oldInteractionMode;
    ccGLWindowInterface::PICKING_MODE m_oldPickingMode = ccGLWindowInterface::DEFAULT_PICKING;
};
