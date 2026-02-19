# FFD Plugin - Implementation Guide

## Current State Summary

**What's Complete:**
- ✅ Complete FFD lattice deformation algorithm
- ✅ Cubic B-spline basis functions for smooth interpolation
- ✅ Point cloud transformation engine
- ✅ Plugin infrastructure and Qt integration
- ✅ Full documentation and guides

**What's Missing:**
- ❌ Visual lattice rendering (GL code)
- ❌ Interactive point manipulation
- ❌ Real-time preview
- ❌ UI dialogs for parameters

---

## Phase 1: Lattice Visualization

### Goal
Make the lattice visible in the 3D viewport as a grid of points/lines.

### What to Create

**File:** `include/ccFFDLatticeDisplay.h`
```cpp
#ifndef CC_FFD_LATTICE_DISPLAY_HEADER
#define CC_FFD_LATTICE_DISPLAY_HEADER

#include "ccHObject.h"
#include "FFDLattice.h"

class ccFFDLatticeDisplay : public ccHObject
{
public:
    explicit ccFFDLatticeDisplay(const FFDLattice& lattice);
    ~ccFFDLatticeDisplay() override;
    
    // Override from ccHObject
    void drawMeOnly(CC_DRAW_CONTEXT& context) override;
    
    // Get bounding box
    ccBBox getMyOwnBB() override;
    
    // Return object type
    CC_CLASS_ENUM getClassID() const override { return CC_TYPES::CUSTOM; }
    
private:
    const FFDLattice& m_lattice;
    CCVector3 m_selectedCP;  // Selected control point (if any)
};

#endif
```

**File:** `src/ccFFDLatticeDisplay.cpp`
```cpp
#include "ccFFDLatticeDisplay.h"
#include "ccGLWindow.h"
#include "ccGL.h"

ccFFDLatticeDisplay::ccFFDLatticeDisplay(const FFDLattice& lattice)
    : ccHObject("FFD Lattice")
    , m_lattice(lattice)
{
    setVisible(true);
}

void ccFFDLatticeDisplay::drawMeOnly(CC_DRAW_CONTEXT& context)
{
    if (!MACRO_Draw3D(context))
        return;
        
    const auto& latticeSize = m_lattice.getLatticeSize();
    ccGL::glEnableClientState(GL_VERTEX_ARRAY);
    
    // Draw control points as spheres
    glPointSize(8.0f);
    glColor3f(1.0f, 0.0f, 0.0f);  // Red
    glBegin(GL_POINTS);
    
    for (unsigned z = 0; z < latticeSize[2]; ++z)
    {
        for (unsigned y = 0; y < latticeSize[1]; ++y)
        {
            for (unsigned x = 0; x < latticeSize[0]; ++x)
            {
                CCVector3d cp = m_lattice.getControlPoint(x, y, z);
                glVertex3dv(cp.u);
            }
        }
    }
    
    glEnd();
    ccGL::glDisableClientState(GL_VERTEX_ARRAY);
}

ccBBox ccFFDLatticeDisplay::getMyOwnBB()
{
    return m_lattice.getBoundingBox();
}
```

### Integration

In `FFDAction.cpp`, modify `performDeformation()`:
```cpp
// After creating the lattice:
FFDLattice lattice(latticeSize, cloud->getOwnBB());

// Create visual representation
auto* latticeDisplay = new ccFFDLatticeDisplay(lattice);
appInterface->addToDB(latticeDisplay);
appInterface->refreshAll();  // Refresh 3D view
```

### Testing
```bash
cmake .. -DPLUGIN_FFD_STANDARD=ON
make
# Load plugin, select cloud, run FFD
# You should now see 27 red points (3x3x3 grid) in the viewport
```

---

## Phase 2: Interactive Point Selection

### Goal
Allow clicking on control points to select them.

### What to Create

**File:** `include/ccFFDTool.h`
```cpp
#ifndef CC_FFD_TOOL_HEADER
#define CC_FFD_TOOL_HEADER

#include "ccOverlayDialog.h"
#include "FFDLattice.h"
#include <memory>

class ccFFDTool : public ccOverlayDialog
{
    Q_OBJECT

public:
    explicit ccFFDTool(ccGLWindow* glWindow, FFDLattice* lattice, ccPointCloud* cloud);
    ~ccFFDTool() override;
    
    // Mouse event handling
    bool eventFilter(QObject* obj, QEvent* event) override;

protected:
    // UI events
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void moveEvent(QMoveEvent* event) override;

private:
    // Helper methods
    void handleMouseMove(QMouseEvent* event);
    void handleMousePress(QMouseEvent* event);
    void handleMouseRelease(QMouseEvent* event);
    
    // Picking - find which control point was clicked
    int pickControlPoint(int x, int y);
    
    FFDLattice* m_lattice;
    ccPointCloud* m_cloud;
    ccGLWindow* m_glWindow;
    
    int m_selectedCPIndex = -1;  // -1 means no selection
    CCVector3d m_lastMousePos;
};

#endif
```

**File:** `src/ccFFDTool.cpp`
```cpp
#include "ccFFDTool.h"
#include "ccGLWindow.h"
#include <QMouseEvent>

ccFFDTool::ccFFDTool(ccGLWindow* glWindow, FFDLattice* lattice, ccPointCloud* cloud)
    : ccOverlayDialog(glWindow)
    , m_lattice(lattice)
    , m_cloud(cloud)
    , m_glWindow(glWindow)
{
    setWindowTitle("FFD - Lattice Editor");
    m_glWindow->installEventFilter(this);
}

ccFFDTool::~ccFFDTool()
{
    if (m_glWindow)
        m_glWindow->removeEventFilter(this);
}

bool ccFFDTool::eventFilter(QObject* obj, QEvent* event)
{
    if (obj == m_glWindow)
    {
        QMouseEvent* mouseEvent = dynamic_cast<QMouseEvent*>(event);
        if (mouseEvent)
        {
            switch (event->type())
            {
                case QEvent::MouseMove:
                    handleMouseMove(mouseEvent);
                    return true;
                case QEvent::MouseButtonPress:
                    handleMousePress(mouseEvent);
                    return true;
                case QEvent::MouseButtonRelease:
                    handleMouseRelease(mouseEvent);
                    return true;
                default:
                    break;
            }
        }
    }
    
    return ccOverlayDialog::eventFilter(obj, event);
}

void ccFFDTool::handleMousePress(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        m_selectedCPIndex = pickControlPoint(event->x(), event->y());
        m_lastMousePos = CCVector3d(event->x(), event->y(), 0);
    }
}

void ccFFDTool::handleMouseMove(QMouseEvent* event)
{
    if (m_selectedCPIndex >= 0 && (event->buttons() & Qt::LeftButton))
    {
        // Convert screen coordinates to 3D displacement
        // TODO: Use camera to transform mouse movement to world coordinates
        
        CCVector3d currentPos(event->x(), event->y(), 0);
        CCVector3d delta = currentPos - m_lastMousePos;
        
        // TODO: Move the control point in 3D space
        // This requires transforming 2D screen delta to 3D world delta
        
        m_lastMousePos = currentPos;
        m_glWindow->redraw();
    }
}

void ccFFDTool::handleMouseRelease(QMouseEvent* event)
{
    m_selectedCPIndex = -1;
}

int ccFFDTool::pickControlPoint(int x, int y)
{
    // TODO: Implement GL picking
    // Find which control point is closest to click position
    return -1;
}

void ccFFDTool::closeEvent(QCloseEvent* event)
{
    // Cleanup
    ccOverlayDialog::closeEvent(event);
}
```

---

## Phase 3: Real-time Deformation Preview

### Integration Point

After moving a control point, create a preview cloud:

```cpp
// In ccFFDTool after moving a control point:
ccPointCloud* previewCloud = m_cloud->cloneThis();

for (unsigned i = 0; i < previewCloud->size(); ++i)
{
    const CCVector3* pt = previewCloud->getPoint(i);
    CCVector3d deformed = m_lattice->deformPoint(
        CCVector3d(pt->x, pt->y, pt->z)
    );
    previewCloud->setPoint(i, CCVector3(deformed.x, deformed.y, deformed.z));
}

// Update visibility
previewCloud->setVisible(true);
m_glWindow->addToOwnDB(previewCloud);
m_glWindow->redraw();
```

---

## Phase 4: Application

### Final Apply Button

```cpp
// In ccFFDTool::applyDeformation():
if (m_lattice && m_cloud)
{
    ccPointCloud* result = m_cloud->cloneThis();
    
    for (unsigned i = 0; i < result->size(); ++i)
    {
        const CCVector3* pt = result->getPoint(i);
        CCVector3d deformed = m_lattice->deformPoint(
            CCVector3d(pt->x, pt->y, pt->z)
        );
        result->setPoint(i, CCVector3(deformed.x, deformed.y, deformed.z));
    }
    
    result->setName(m_cloud->getName() + " [FFD]");
    m_glWindow->addToOwnDB(result);
    
    // Remove preview cloud
    // Cleanup lattice display
    
    m_glWindow->redraw();
}
```

---

## Debugging Tips

### Print Lattice Information
```cpp
// In FFDAction.cpp:
auto& cps = lattice.getAllControlPoints();
qDebug() << "Lattice has" << cps.size() << "control points";
for (size_t i = 0; i < std::min(size_t(5), cps.size()); ++i)
{
    auto& cp = cps[i];
    qDebug() << QString("CP %1: (%2, %3, %4)")
        .arg(i).arg(cp.x).arg(cp.y).arg(cp.z);
}
```

### Test Deformation Algorithm
```cpp
// Create a simple test:
FFDLattice lattice({3, 3, 3}, cloud->getOwnBB());

// Move one control point
lattice.moveControlPoint(1, 1, 1, CCVector3d(1.0, 0.0, 0.0));

// Test a point at center
CCVector3 testPt = cloud->getPoint(0);
CCVector3d deformed = lattice.deformPoint(
    CCVector3d(testPt.x, testPt.y, testPt.z)
);

qDebug() << "Original:" << testPt.x << testPt.y << testPt.z;
qDebug() << "Deformed:" << deformed.x << deformed.y << deformed.z;
```

---

## Build Checklist

- [ ] Create `ccFFDLatticeDisplay` class
- [ ] Implement GL rendering of lattice points
- [ ] Register new class in CMakeLists.txt
- [ ] Test: Can see lattice in viewport
- [ ] Create `ccFFDTool` interactive tool
- [ ] Implement mouse picking
- [ ] Implement 2D-to-3D coordinate transformation
- [ ] Test: Can click and drag control points
- [ ] Implement deformation preview
- [ ] Test: See real-time cloud deformation
- [ ] Implement final "Apply" button
- [ ] Test: Get new deformed point cloud

---

## References for Implementation

**Qt/GL in CloudCompare:**
- Look at existing tools: `plugins/core/GL/ccClippingBoxTool`
- GL rendering: `ccGL.h`, `CC_DRAW_CONTEXT`
- Event handling: `ccOverlayDialog` base class

**Coordinate Systems:**
- Screen space → Normalized Device Coordinates (NDC)
- NDC → World space (use camera matrix inversion)
- Study: `ccGLWindow::getViewportParameters()`

**Point Picking:**
- CloudCompare uses `ccOctree` for efficient spatial queries
- Alternative: Simple distance check to all control points

---

Good luck with the implementation! The hardest part (the FFD algorithm) is already done. 🚀
