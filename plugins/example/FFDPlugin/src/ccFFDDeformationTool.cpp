#include "ccFFDDeformationTool.h"
#include "FFDLattice.h"
#include "ccFFDLatticeDisplay.h"
#include "ccSelectionRectangle.h"
#include "ccFFDDeformationApplier.h"

#include <ccPointCloud.h>
#include <ccMainAppInterface.h>
#include <ccGLMatrix.h>
#include <ccGLWindowInterface.h>
#include <ccGLWindowSignalEmitter.h>

#include <Qt>
#include <QApplication>

#include "FFDDebug.h"

ccFFDDeformationTool::ccFFDDeformationTool(ccPointCloud* originalCloud, ccPointCloud* previewCloud, ccMainAppInterface* appInterface)
    : ccOverlayDialog()
    , m_originalCloud(originalCloud)
    , m_previewCloud(previewCloud)
    , m_appInterface(appInterface)
{
    FFD_DEBUG("CONSTRUCTOR: ccFFDDeformationTool created, this=" << this);
    FFD_DEBUG("  originalCloud=" << originalCloud << ", previewCloud=" << previewCloud);
    setWindowTitle("FFD Deformation Tool");
    setVisible(true);
    setAttribute(Qt::WA_DeleteOnClose, true);

    addOverriddenShortcut(Qt::Key_X);
    addOverriddenShortcut(Qt::Key_Y);
    addOverriddenShortcut(Qt::Key_Z);
    addOverriddenShortcut(Qt::Key_Return);
    addOverriddenShortcut(Qt::Key_R);
    addOverriddenShortcut(Qt::Key_C);
    addOverriddenShortcut(Qt::Key_F);

    connect(this, &ccOverlayDialog::shortcutTriggered, this, &ccFFDDeformationTool::onShortcutTriggered);
}

ccFFDDeformationTool::~ccFFDDeformationTool()
{
    FFD_DEBUG("DESTRUCTOR: ccFFDDeformationTool being destroyed, this=" << this);
    FFD_DEBUG("  m_selectionRect=" << m_selectionRect << ", m_lattice=" << m_lattice);
    FFD_DEBUG("  m_latticeDisplay=" << m_latticeDisplay << ", m_controlPointCloud=" << m_controlPointCloud);
    
    if (m_selectionRect)
    {
        FFD_DEBUG("  Deleting m_selectionRect...");
        delete m_selectionRect;
        m_selectionRect = nullptr;
        FFD_DEBUG("  m_selectionRect deleted");
    }
    if (m_previewApplier)
    {
        FFD_DEBUG("  Deleting m_previewApplier...");
        delete m_previewApplier;
        m_previewApplier = nullptr;
    }
    if (m_fullApplier)
    {
        FFD_DEBUG("  Deleting m_fullApplier...");
        delete m_fullApplier;
        m_fullApplier = nullptr;
    }
    if (m_controlPointCloud)
    {
        FFD_DEBUG("  Deleting m_controlPointCloud...");
        delete m_controlPointCloud;
        m_controlPointCloud = nullptr;
    }
    if (m_lattice)
    {
        FFD_DEBUG("  Deleting m_lattice...");
        delete m_lattice;
        m_lattice = nullptr;
    }
    FFD_DEBUG("DESTRUCTOR: ccFFDDeformationTool destroyed");
}

void ccFFDDeformationTool::setLattice(FFDLattice* lattice, ccFFDLatticeDisplay* display)
{
    FFD_DEBUG("setLattice: lattice=" << lattice << ", display=" << display);
    m_lattice = lattice;
    m_latticeDisplay = display;

    updateControlPointCloud();

    // Initialize deformation appliers
    if (m_previewApplier)
    {
        delete m_previewApplier;
    }
    if (m_fullApplier)
    {
        delete m_fullApplier;
    }

    m_previewApplier = new ccFFDDeformationApplier(m_previewCloud, m_lattice);
    m_fullApplier = new ccFFDDeformationApplier(m_originalCloud, m_lattice);

    if (m_previewApplier)
    {
        m_previewApplier->initializeOriginalPositions();
        m_previewApplier->setSubsampleRatio(1.0f); // preview cloud already subsampled
    }
    if (m_fullApplier)
    {
        m_fullApplier->initializeOriginalPositions();
        m_fullApplier->setSubsampleRatio(1.0f);
    }

    if (m_previewCloud && m_appInterface)
    {
        size_t previewSize = m_previewCloud->size();
        m_appInterface->dispToConsole(QString("[FFD] Preview cloud size: %1 points").arg(previewSize),
                                      ccMainAppInterface::STD_CONSOLE_MESSAGE);
    }
}

bool ccFFDDeformationTool::start()
{
    FFD_DEBUG("start: entering, this=" << this);
    if (!m_appInterface)
    {
        FFD_DEBUG("start: m_appInterface is null, returning false");
        return false;
    }

    ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : m_appInterface->getActiveGLWindow();
    if (!win)
    {
        FFD_DEBUG("start: win is null, returning false");
        return false;
    }
    FFD_DEBUG("start: win=" << win);

    if (m_previewCloud)
    {
        m_previewCloud->setVisible(true);
    }

    m_oldInteractionMode = win->getInteractionMode();
    m_oldPickingMode = win->getPickingMode();

    win->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
    win->setPickingMode(ccGLWindowInterface::POINT_PICKING, Qt::CrossCursor);
    win->setPickingRadius(14);

    FFD_DEBUG("start: connecting signals...");
    connect(win->signalEmitter(), &ccGLWindowSignalEmitter::leftButtonClicked, this, &ccFFDDeformationTool::onLeftButtonClicked);
    connect(win->signalEmitter(), &ccGLWindowSignalEmitter::mouseMoved, this, &ccFFDDeformationTool::onMouseMoved);
    connect(win->signalEmitter(), &ccGLWindowSignalEmitter::buttonReleased, this, &ccFFDDeformationTool::onButtonReleased);
    connect(win->signalEmitter(), &ccGLWindowSignalEmitter::itemPicked, this, &ccFFDDeformationTool::onItemPicked);

    // Note: m_controlPointCloud is kept invisible and is NOT added to the scene database
    // It is only used internally for point picking; the lattice display provides visual feedback

    // Create and add selection rectangle for visualization
    FFD_DEBUG("start: m_selectionRect before creation: " << m_selectionRect);
    if (!m_selectionRect)
    {
        m_selectionRect = new ccSelectionRectangle();
        m_selectionRect->setDrawing(false);
        FFD_DEBUG("start: created new m_selectionRect=" << m_selectionRect);
    }
    FFD_DEBUG("start: adding m_selectionRect to win->ownDB");
    win->addToOwnDB(m_selectionRect, false);
    FFD_DEBUG("start: m_selectionRect added to ownDB");

    const bool started = ccOverlayDialog::start();
    FFD_DEBUG("start: ccOverlayDialog::start() returned " << started);
    hide();
    return started;
}

void ccFFDDeformationTool::stop(bool accepted)
{
    FFD_DEBUG("stop: entering, accepted=" << accepted << ", this=" << this);
    ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
    if (win)
    {
        FFD_DEBUG("stop: win=" << win << ", removing selection rect from ownDB...");
        // Remove selection rectangle from window before stop
        if (m_selectionRect)
        {
            FFD_DEBUG("stop: removing m_selectionRect=" << m_selectionRect << " from ownDB");
            win->removeFromOwnDB(m_selectionRect);
            // removeFromOwnDB deletes the object, so null out the pointer
            // to prevent double-free in the destructor
            m_selectionRect = nullptr;
            FFD_DEBUG("stop: m_selectionRect removed from ownDB and nulled");
        }
        win->setInteractionMode(m_oldInteractionMode);
        win->setPickingMode(m_oldPickingMode);
        win->signalEmitter()->disconnect(this);
        // Note: m_controlPointCloud is not in the scene database, so no need to remove it
        win->redraw(false, false);
    }
    FFD_DEBUG("stop: calling ccOverlayDialog::stop");
    ccOverlayDialog::stop(accepted);
    FFD_DEBUG("stop: done");
}

void ccFFDDeformationTool::onLeftButtonClicked(int x, int y)
{
    FFD_DEBUG("onLeftButtonClicked: x=" << x << ", y=" << y << ", this=" << this);
    FFD_DEBUG("  m_lattice=" << m_lattice << ", m_latticeDisplay=" << m_latticeDisplay);
    
    if (!m_lattice || !m_latticeDisplay)
    {
        FFD_DEBUG("  early return: lattice or display is null");
        return;
    }

    m_lastMouseX = x;
    m_lastMouseY = y;

    const bool ctrlPressed = (QApplication::keyboardModifiers() & Qt::ControlModifier);
    FFD_DEBUG("  ctrlPressed=" << ctrlPressed);

    if (ctrlPressed)
    {
        FFD_DEBUG("  starting rectangle selection");
        // Start rectangle selection with Ctrl - disable camera controls
        ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
        FFD_DEBUG("  win=" << win);
        if (win)
        {
            win->setInteractionMode(ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
        }
        
        m_isDrawingRectangle = true;
        m_rectStartX = x;
        m_rectStartY = y;
        m_rectEndX = x;
        m_rectEndY = y;
        m_selectedPointIndices.clear();
        
        FFD_DEBUG("  m_selectionRect=" << m_selectionRect);
        if (m_selectionRect)
        {
            FFD_DEBUG("  setting m_selectionRect to drawing mode");
            m_selectionRect->setDrawing(true);
            m_selectionRect->setRectangle(x, y, x, y);
            FFD_DEBUG("  setRectangle done");
        }
        else
        {
            FFD_DEBUG("  WARNING: m_selectionRect is NULL!");
        }
    }
    else
    {
        // If points are already selected, start dragging them
        if (!m_selectedPointIndices.empty())
        {
            // Save current state to history before making changes
            pushLatticeHistory();
            
            m_isDragging = true;
            m_lastMouseX = x;
            m_lastMouseY = y;
        }
        // Otherwise, single point picking mode will be triggered by the window itself (POINT_PICKING mode)
    }
}

void ccFFDDeformationTool::onMouseMoved(int x, int y, Qt::MouseButtons buttons)
{
    if (!(buttons & Qt::LeftButton))
        return;

    // Handle rectangle drawing (with Ctrl pressed)
    if (m_isDrawingRectangle)
    {
        FFD_DEBUG("onMouseMoved: drawing rect, x=" << x << ", y=" << y);
        FFD_DEBUG("  m_selectionRect=" << m_selectionRect << ", this=" << this);
        m_rectEndX = x;
        m_rectEndY = y;
        
        if (m_selectionRect)
        {
            FFD_DEBUG("  calling m_selectionRect->setRectangle");
            m_selectionRect->setRectangle(m_rectStartX, m_rectStartY, m_rectEndX, m_rectEndY);
            FFD_DEBUG("  setRectangle done");
        }
        else
        {
            FFD_DEBUG("  WARNING: m_selectionRect is NULL during drawing!");
        }
        
        ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
        FFD_DEBUG("  win=" << win);
        if (win)
        {
            FFD_DEBUG("  calling win->redraw");
            win->redraw(false, false);
            FFD_DEBUG("  redraw done");
        }
        return;
    }

    // Handle point dragging (no Ctrl required)
    if (!m_isDragging || !m_lattice)
        return;
    
    // Must have either single point or multiple points selected
    if (m_selectedPointIndices.empty() && m_selectedPointIndex < 0)
        return;

    ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
    if (!win)
        return;

    const int dx = x - m_lastMouseX;
    const int dy = y - m_lastMouseY;
    m_lastMouseX = x;
    m_lastMouseY = y;

    const double pixelSize = win->computeActualPixelSize();

    const ccViewportParameters& vp = win->getViewportParameters();
    CCVector3d viewDir = vp.getViewDir();
    CCVector3d up = vp.getUpDir();
    CCVector3d right = viewDir.cross(up);
    right.normalize();
    up.normalize();

    // Screen coordinates: x grows to the right, y grows downward
    CCVector3d movement = right * (pixelSize * dx) + up * (-pixelSize * dy);
    movement = applyAxisConstraint(movement);

    if (movement.norm() > 1e-9)
    {
        auto dims = m_lattice->getLatticeSize();
        
        // Move either single selected point or all selected points
        if (!m_selectedPointIndices.empty())
        {
            // Move all selected points
            for (int idx : m_selectedPointIndices)
            {
                m_lattice->moveControlPoint(
                    idx % dims[0],
                    (idx / dims[0]) % dims[1],
                    idx / (dims[0] * dims[1]),
                    movement
                );
            }
        }
        else if (m_selectedPointIndex >= 0)
        {
            // Move single point
            m_lattice->moveControlPoint(
                m_selectedPointIndex % dims[0],
                (m_selectedPointIndex / dims[0]) % dims[1],
                m_selectedPointIndex / (dims[0] * dims[1]),
                movement
            );
        }

        if (m_latticeDisplay)
        {
            m_latticeDisplay->setControlPoints(m_lattice->getAllControlPoints());
        }

        updateControlPointCloud();
        updateCloudDeformation();

        if (m_associatedWin)
        {
            m_associatedWin->redraw(false, false);
        }
    }
}

void ccFFDDeformationTool::onButtonReleased()
{
    FFD_DEBUG("onButtonReleased: entering, this=" << this);
    FFD_DEBUG("  m_isDrawingRectangle=" << m_isDrawingRectangle << ", m_isDragging=" << m_isDragging);
    
    if (m_isDrawingRectangle)
    {
        FFD_DEBUG("  finishing rectangle selection");
        // Finish rectangle selection
        m_isDrawingRectangle = false;
        
        FFD_DEBUG("  m_selectionRect=" << m_selectionRect);
        if (m_selectionRect)
        {
            FFD_DEBUG("  setting m_selectionRect drawing=false");
            m_selectionRect->setDrawing(false);
        }
        else
        {
            FFD_DEBUG("  WARNING: m_selectionRect is NULL in onButtonReleased!");
        }
        
        FFD_DEBUG("  calling selectPointsInRectangle(" << m_rectStartX << "," << m_rectStartY << "," << m_rectEndX << "," << m_rectEndY << ")");
        selectPointsInRectangle(m_rectStartX, m_rectStartY, m_rectEndX, m_rectEndY);
        FFD_DEBUG("  selectPointsInRectangle done, selected " << m_selectedPointIndices.size() << " points");
        
        ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
        FFD_DEBUG("  win=" << win);
        
        if (!m_selectedPointIndices.empty())
        {
            FFD_DEBUG("  selected points not empty, count=" << m_selectedPointIndices.size());
            FFD_DEBUG("  m_appInterface=" << m_appInterface);
            if (!m_appInterface)
            {
                FFD_DEBUG("  ERROR: m_appInterface is NULL!");
                return;
            }
            m_appInterface->dispToConsole(QString("[FFD] Selected %1 control points").arg(m_selectedPointIndices.size()), 
                                         ccMainAppInterface::STD_CONSOLE_MESSAGE);
            FFD_DEBUG("  dispToConsole done");
            // Prepare for dragging - camera controls remain disabled until selection is cleared
            m_isDragging = false; // Will be set to true on next click
            m_dragStartPointPositions.clear();
            FFD_DEBUG("  m_lattice=" << m_lattice);
            if (!m_lattice)
            {
                FFD_DEBUG("  ERROR: m_lattice is NULL!");
                return;
            }
            const auto& points = m_lattice->getAllControlPoints();
            FFD_DEBUG("  getAllControlPoints done, points.size()=" << points.size());
            for (int idx : m_selectedPointIndices)
            {
                FFD_DEBUG("    checking idx=" << idx << " against points.size()=" << points.size());
                if (idx >= 0 && idx < static_cast<int>(points.size()))
                {
                    m_dragStartPointPositions.push_back(points[idx]);
                }
                else
                {
                    FFD_DEBUG("    ERROR: idx out of range!");
                }
            }
            FFD_DEBUG("  dragStartPointPositions populated");
            
            // Update lattice display to show selected points
            FFD_DEBUG("  m_latticeDisplay=" << m_latticeDisplay);
            if (m_latticeDisplay)
            {
                FFD_DEBUG("  calling setSelectedIndices");
                m_latticeDisplay->setSelectedIndices(m_selectedPointIndices);
                FFD_DEBUG("  setSelectedIndices done");
            }
            else
            {
                FFD_DEBUG("  WARNING: m_latticeDisplay is NULL, skipping setSelectedIndices");
            }
            
            // Keep camera controls disabled while points are selected
            if (win)
            {
                FFD_DEBUG("  setting interaction mode (send all signals)");
                win->setInteractionMode(ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
                FFD_DEBUG("  interaction mode set");
            }
        }
        else
        {
            FFD_DEBUG("  no points selected, re-enabling camera controls");
            // No points selected, re-enable camera controls
            if (win)
            {
                win->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
            }
        }
        
        FFD_DEBUG("  about to redraw, win=" << win);
        if (win)
        {
            FFD_DEBUG("  calling win->redraw");
            win->redraw(false, false);
            FFD_DEBUG("  redraw done");
        }
        FFD_DEBUG("  onButtonReleased (rectangle path) returning");
        return;
    }
    
    if (m_isDragging)
    {
        // Stop dragging but keep points selected - press Ctrl+C to deselect
        m_isDragging = false;
        m_appInterface->dispToConsole("[FFD] Released control point(s) - Press Ctrl+C to deselect or click to drag again", ccMainAppInterface::STD_CONSOLE_MESSAGE);

        ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
        if (win)
        {
            // Keep camera controls disabled while points remain selected
            win->redraw(false, false);
        }
    }
}

void ccFFDDeformationTool::onShortcutTriggered(int key)
{
    if (key == Qt::Key_C)
    {
        const Qt::KeyboardModifiers mods = QApplication::keyboardModifiers();
        if (mods & Qt::ControlModifier)
        {
            // Ctrl+C: Clear selection and re-enable camera controls
            m_isDragging = false;
            m_selectedPointIndex = -1;
            m_selectedPointIndices.clear();
            
            if (m_latticeDisplay)
            {
                m_latticeDisplay->setSelectedIndices({});
            }
            
            ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
            if (win)
            {
                win->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
                win->redraw(false, false);
            }
            
            m_appInterface->dispToConsole("[FFD] Selection cleared", ccMainAppInterface::STD_CONSOLE_MESSAGE);
            return;
        }
    }
    
    if (key == Qt::Key_F)
    {
        const Qt::KeyboardModifiers mods = QApplication::keyboardModifiers();
        if (mods & Qt::ControlModifier)
        {
            // Ctrl+F: Undo last transformation
            undoLastTransformation();
            return;
        }
    }
    
    if (key == Qt::Key_Return || key == Qt::Key_Enter)
    {
        // Apply deformation to full-resolution cloud
        if (m_fullApplier)
        {
            m_fullApplier->applyDeformation();
            m_appInterface->dispToConsole("[FFD] Deformation applied to full cloud", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        }
        // Keep preview in sync for continued editing
        if (m_previewApplier)
        {
            m_previewApplier->applyDeformation();
        }
        if (m_associatedWin)
        {
            m_associatedWin->redraw(false, false);
        }
        return;
    }
    if (key == Qt::Key_R)
    {
        // Reset deformation and lattice
        if (m_previewApplier)
        {
            m_previewApplier->resetDeformation();
        }
        if (m_fullApplier)
        {
            m_fullApplier->resetDeformation();
        }
        if (m_lattice)
        {
            m_lattice->reset();
            
            // Update the visual lattice display with reset control points
            if (m_latticeDisplay)
            {
                m_latticeDisplay->setControlPoints(m_lattice->getAllControlPoints());
                m_latticeDisplay->setSelectedIndices({});  // Clear selection
            }
        }
        
        // Clear selection and re-enable camera
        m_isDragging = false;
        m_selectedPointIndex = -1;
        m_selectedPointIndices.clear();
        
        // Update the control point cloud visualization
        updateControlPointCloud();

        ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
        if (win)
        {
            win->setInteractionMode(ccGLWindowInterface::MODE_TRANSFORM_CAMERA | ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
            win->redraw(false, false);
        }

        m_appInterface->dispToConsole("[FFD] Deformation and lattice reset", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        return;
    }

    AxisConstraint newConstraint = NONE;

    if (key == Qt::Key_X)
        newConstraint = X_AXIS;
    else if (key == Qt::Key_Y)
        newConstraint = Y_AXIS;
    else if (key == Qt::Key_Z)
        newConstraint = Z_AXIS;

    const Qt::KeyboardModifiers mods = QApplication::keyboardModifiers();
    const bool lockRequested = (mods & Qt::ShiftModifier) != 0;

    if (lockRequested)
    {
        if (m_axisConstraintLocked && m_axisConstraint == newConstraint)
        {
            m_axisConstraintLocked = false;
            m_axisConstraint = NONE;
            m_appInterface->dispToConsole("[FFD] Axis lock disabled", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        }
        else
        {
            m_axisConstraint = newConstraint;
            m_axisConstraintLocked = true;
            if (m_axisConstraint == X_AXIS)
                m_appInterface->dispToConsole("[FFD] X axis locked", ccMainAppInterface::STD_CONSOLE_MESSAGE);
            else if (m_axisConstraint == Y_AXIS)
                m_appInterface->dispToConsole("[FFD] Y axis locked", ccMainAppInterface::STD_CONSOLE_MESSAGE);
            else if (m_axisConstraint == Z_AXIS)
                m_appInterface->dispToConsole("[FFD] Z axis locked", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        }

        return;
    }

    if (m_axisConstraintLocked)
    {
        m_appInterface->dispToConsole("[FFD] Axis is locked (use Shift+X/Y/Z to unlock)", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        return;
    }

    if (m_axisConstraint == newConstraint)
    {
        m_axisConstraint = NONE;
        m_appInterface->dispToConsole("[FFD] Axis constraint disabled", ccMainAppInterface::STD_CONSOLE_MESSAGE);
    }
    else
    {
        m_axisConstraint = newConstraint;
        if (m_axisConstraint == X_AXIS)
            m_appInterface->dispToConsole("[FFD] Constrained to X axis", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        else if (m_axisConstraint == Y_AXIS)
            m_appInterface->dispToConsole("[FFD] Constrained to Y axis", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        else if (m_axisConstraint == Z_AXIS)
            m_appInterface->dispToConsole("[FFD] Constrained to Z axis", ccMainAppInterface::STD_CONSOLE_MESSAGE);
    }
}


void ccFFDDeformationTool::onItemPicked(ccHObject* entity, unsigned subEntityID, int x, int y, const CCVector3& P, const CCVector3d&)
{
    if (!m_lattice || !m_controlPointCloud)
        return;

    if (entity != m_controlPointCloud)
        return;

    auto points = m_lattice->getAllControlPoints();
    if (subEntityID >= points.size())
        return;

    // Save current state to history before making changes
    pushLatticeHistory();

    m_selectedPointIndex = static_cast<int>(subEntityID);
    m_isDragging = true;
    m_lastMouseX = x;
    m_lastMouseY = y;

    m_dragStartPointPos = points[m_selectedPointIndex];
    m_dragStartClickPos = CCVector3d(P.x, P.y, P.z);

    ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
    if (win)
    {
        win->setInteractionMode(ccGLWindowInterface::INTERACT_SEND_ALL_SIGNALS);
    }

    // Highlight the selected point
    if (m_latticeDisplay)
    {
        m_latticeDisplay->setSelectedIndices({m_selectedPointIndex});
    }

    m_appInterface->dispToConsole(QString("[FFD] Selected control point %1").arg(m_selectedPointIndex), ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

void ccFFDDeformationTool::updateControlPointCloud()
{
    if (!m_lattice)
        return;

    const auto& points = m_lattice->getAllControlPoints();

    if (!m_controlPointCloud)
    {
        m_controlPointCloud = new ccPointCloud("FFD Control Points");
        m_controlPointCloud->setVisible(false);  // Keep invisible - used only for picking, not display
        m_controlPointCloud->setEnabled(true);
        m_controlPointCloud->setPointSize(1);
    }

    m_controlPointCloud->clear();
    if (!m_controlPointCloud->reserve(points.size()))
        return;

    for (const CCVector3d& cp : points)
    {
        m_controlPointCloud->addPoint(CCVector3(static_cast<float>(cp.x), static_cast<float>(cp.y), static_cast<float>(cp.z)));
    }

    m_controlPointCloud->invalidateBoundingBox();
}

CCVector3d ccFFDDeformationTool::applyAxisConstraint(const CCVector3d& delta) const
{
    if (m_axisConstraint == X_AXIS)
        return CCVector3d(delta.x, 0, 0);
    if (m_axisConstraint == Y_AXIS)
        return CCVector3d(0, delta.y, 0);
    if (m_axisConstraint == Z_AXIS)
        return CCVector3d(0, 0, delta.z);

    return delta;
}

bool ccFFDDeformationTool::projectPointToScreen(const CCVector3d& point3D, int& screenX, int& screenY) const
{
    ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
    if (!win)
        return false;

    // Get camera parameters
    ccGLCameraParameters camera;
    win->getGLCameraParameters(camera);
    
    // Use CloudCompare's built-in projection
    CCVector3d projectedPoint;
    bool inFrustum = false;
    if (!camera.project(point3D, projectedPoint, &inFrustum))
        return false;
    
    if (!inFrustum)
        return false;
    
    // projectedPoint.x/y are in OpenGL window coordinates (origin at bottom-left)
    // convert to Qt screen coordinates (origin at top-left)
    const double winX = projectedPoint.x;
    const double winY = projectedPoint.y;
    screenX = static_cast<int>(winX);
    screenY = static_cast<int>(camera.viewport[3] - (winY - camera.viewport[1]) + camera.viewport[1]);
    
    return true;
}

void ccFFDDeformationTool::selectPointsInRectangle(int x1, int y1, int x2, int y2)
{
    FFD_DEBUG("selectPointsInRectangle: entering, this=" << this);
    FFD_DEBUG("  x1=" << x1 << ", y1=" << y1 << ", x2=" << x2 << ", y2=" << y2);
    FFD_DEBUG("  m_lattice=" << m_lattice);
    
    if (!m_lattice)
    {
        FFD_DEBUG("  early return: m_lattice is null");
        return;
    }

    ccGLWindowInterface* win = m_associatedWin ? m_associatedWin : (m_appInterface ? m_appInterface->getActiveGLWindow() : nullptr);
    FFD_DEBUG("  win=" << win);
    if (!win)
    {
        FFD_DEBUG("  early return: win is null");
        return;
    }

    // Scale mouse coords to device pixels to match projection results
    const double devicePixelRatio = win->getDevicePixelRatio();
    FFD_DEBUG("  devicePixelRatio=" << devicePixelRatio);
    const int sx1 = static_cast<int>(x1 * devicePixelRatio);
    const int sx2 = static_cast<int>(x2 * devicePixelRatio);
    const int sy1 = static_cast<int>(y1 * devicePixelRatio);
    const int sy2 = static_cast<int>(y2 * devicePixelRatio);

    // Normalize rectangle coordinates
    int minX = std::min(sx1, sx2);
    int maxX = std::max(sx1, sx2);
    int minY = std::min(sy1, sy2);
    int maxY = std::max(sy1, sy2);

    m_selectedPointIndices.clear();
    
    FFD_DEBUG("  getting control points from lattice");
    const auto& points = m_lattice->getAllControlPoints();
    FFD_DEBUG("  points.size()=" << points.size());
    
    for (size_t i = 0; i < points.size(); ++i)
    {
        int screenX, screenY;
        if (projectPointToScreen(points[i], screenX, screenY))
        {
            if (screenX >= minX && screenX <= maxX && screenY >= minY && screenY <= maxY)
            {
                m_selectedPointIndices.push_back(static_cast<int>(i));
            }
        }
    }
    FFD_DEBUG("  selected " << m_selectedPointIndices.size() << " points");
}

void ccFFDDeformationTool::updateCloudDeformation()
{
    if (!m_previewApplier)
        return;

    m_previewApplier->updateDeformedCloud();
}

void ccFFDDeformationTool::pushLatticeHistory()
{
    if (!m_lattice)
        return;
    
    // Save current lattice state
    m_latticeHistory.push_back(m_lattice->getAllControlPoints());
    
    // Limit history size
    if (m_latticeHistory.size() > MAX_HISTORY_SIZE)
    {
        m_latticeHistory.pop_front();
    }
}

void ccFFDDeformationTool::undoLastTransformation()
{
    if (!m_lattice || m_latticeHistory.empty())
    {
        m_appInterface->dispToConsole("[FFD] No history to undo", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
        return;
    }
    
    // Restore previous lattice state
    m_lattice->setAllControlPoints(m_latticeHistory.back());
    m_latticeHistory.pop_back();
    
    // Update visual display
    if (m_latticeDisplay)
    {
        m_latticeDisplay->setControlPoints(m_lattice->getAllControlPoints());
    }
    
    // Update control point cloud
    updateControlPointCloud();
    
    // Update deformed point clouds
    if (m_previewApplier)
    {
        m_previewApplier->updateDeformedCloud();
    }
    
    if (m_associatedWin)
    {
        m_associatedWin->redraw(false, false);
    }
    
    m_appInterface->dispToConsole("[FFD] Undo successful", ccMainAppInterface::STD_CONSOLE_MESSAGE);
}
