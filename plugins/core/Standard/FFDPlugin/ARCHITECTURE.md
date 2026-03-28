# FFD Plugin - Architecture & Data Flow

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                       CloudCompare                                   │
│                   (Main Application)                                 │
└────────────────────────┬────────────────────────────────────────────┘
                         │
                         │ Loads
                         ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    FFDPlugin                                         │
│  (ccGLPluginInterface Implementation)                               │
│                                                                      │
│  • Registers "Free Form Deformation" action                         │
│  • Validates selection (point cloud required)                       │
│  • Launches FFD workflow                                            │
└────────────────────────┬────────────────────────────────────────────┘
                         │
                         │ onNewSelection()
                         │ getActions()
                         │
              ┌──────────▼──────────┐
              │  User Interaction   │
              │                     │
              │ Selects cloud       │
              │ Clicks FFD action   │
              └──────────┬──────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────────┐
│               FFDAction::performDeformation()                        │
│                    (Entry Point)                                     │
└────────────────────────┬────────────────────────────────────────────┘
                         │
                         │ 1. Get selected cloud
                         │ 2. Create FFDLattice
                         │ 3. Create visual display (TODO)
                         │ 4. Activate interactive tool (TODO)
                         │
            ┌────────────┴────────────┐
            │                         │
            ▼                         ▼
    ┌──────────────────┐    ┌──────────────────────┐
    │  FFDLattice      │    │ ccFFDLatticeDisplay  │
    │                  │    │       (TODO)         │
    │ • 3D grid        │    │                      │
    │ • Control pts    │    │ • GL rendering       │
    │ • Deform alg     │    │ • Shows lattice      │
    │ • B-spline math  │    │ • Highlight points  │
    └──────┬───────────┘    └──────────┬───────────┘
           │                           │
           │ deformPoint()             │
           │                           │ drawMeOnly()
           │                           │
    ┌──────▼───────────────────────────▼──────────┐
    │  3D Viewport (OpenGL)                        │
    │                                              │
    │  Shows:                                      │
    │  • Original point cloud                      │
    │  • Lattice grid (control points)             │
    │  • Deformed preview (real-time)              │
    └──────────────────────────────────────────────┘
           ▲
           │
    ┌──────┴─────────────────────────┐
    │   ccFFDTool (TODO)              │
    │   Interactive Tool              │
    │                                 │
    │ • Mouse picking                 │
    │ • Point manipulation            │
    │ • Real-time preview             │
    │ • Apply/Cancel                  │
    └─────────────────────────────────┘
```

## Data Flow: Point Deformation

```
Original Point Cloud
        │
        │ User moves lattice node
        │
        ▼
┌──────────────────────────────────────┐
│   FFDLattice                         │
│                                      │
│   moveControlPoint(x,y,z, delta)    │
│                                      │
│   • Updates control point position  │
│   • Stores new position             │
└──────────────────────────────────────┘
        │
        │ For each point in cloud:
        │
        ▼
┌──────────────────────────────────────────────┐
│   deformPoint(originalPoint)                 │
│                                              │
│   1. Check if point in bounding box         │
│      │                                       │
│      ▼                                       │
│   2. For each control point:                │
│      • Compute weight using                │
│        cubicBSplineWeight(distance)        │
│        (smooth interpolation)              │
│      │                                       │
│      ▼                                       │
│   3. Weighted average of influences        │
│      deformed = Σ(weight × control_pt)    │
│                 / Σ(weight)               │
│      │                                       │
│      ▼                                       │
│   4. Return deformed point                 │
└──────────────────────────────────────────────┘
        │
        ▼
    Deformed Point Cloud
```

## Continuity Guarantee (The Key Innovation)

```
Without B-Spline (Hard Transitions - BAD ✗)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Weight
  │     ┌─────────┐
  │     │         │
1 ├─────┤         ├─────  Discontinuous jump!
  │     │         │
  │     │         └─────
  │     │               
0 ├─────┴─────────────── 
  │
  └────────────────────► Distance from control point


With B-Spline (Smooth Transitions - GOOD ✓)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Weight
  │          ╱╲
  │        ╱    ╲
1 ├──────╱────  ─╲────  Smooth curve!
  │    ╱          ╲
  │  ╱              ╲
  │╱                 ╲
0 ├───────────────────╲─ No jump!
  │
  └──────────────────────────► Distance from control point


Result: Point moves smoothly without popping/artifacts
```

## Control Point Grid Structure

```
3D Lattice (3×3×3 example)
━━━━━━━━━━━━━━━━━━━━━━━━

         z=2 layer
         ┌─────┬─────┬─────┐
         │ o───o───o │
         │ │   │   │ │
         o─o───o───o │
         │ │   │   │ │
         o─o───o───o │
         └─────┴─────┴─────┘

         z=1 layer
         ┌─────┬─────┬─────┐
         │ o───o───o │
         │ │   │   │ │
         o─o───o───o │
         │ │   │   │ │
         o─o───o───o │
         └─────┴─────┴─────┘

         z=0 layer
         ┌─────┬─────┬─────┐
         │ o───o───o │
         │ │   │   │ │
         o─o───o───o │
         │ │   │   │ │
         o─o───o───o │
         └─────┴─────┴─────┘

o = Control Point (can move)
─ = Lattice edges
│ = Lattice edges

Each point in cloud is influenced by nearby
control points with smooth weight falloff
```

## Deformation Process Step-by-Step

```
Step 1: Create Lattice
┌──────────────────┐
│  Point Cloud     │
│  ┌────────────┐  │
│  │            │  │
│  │    ☁ ☁     │  │
│  │  ☁   ☁   ☁ │  │
│  │    ☁ ☁     │  │
│  │            │  │
│  └────────────┘  │
│   Bounding Box   │
└──────────────────┘
          ↓
    ┌─────────────┐
    │   3×3×3     │
    │   Lattice   │
    │             │
    │   27 nodes  │
    └─────────────┘


Step 2: Move a Control Point
       ┌─────────────┐
       │   Lattice   │
       │             │
       │   •     •   │
       │             │
       │   •  ↑•  •  │  User drags top node up
       │      |      │
       └──────┼──────┘
              │
              ▼ Influence zone around node


Step 3: Points Are Deformed
       ┌─────────────┐
       │             │
       │ Original ▐  │▌ Deformed
       │     ▐  ▌    │
       │  ▐ ▌        │
       │             │
       └─────────────┘
       
       Points move in the influenced region
       Movement is smooth (no sudden changes)


Step 4: Create Result Cloud
       
       New Cloud = Deformed version
       (original unchanged)
       
       Ready to export or edit more
```

## B-Spline Basis Function

```
Cubic B-Spline: How Points Are Smoothly Influenced
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Influence by proximity:

Distance from control point:
0.0  0.5  1.0  1.5  2.0
│    │    │    │    │
├────┼────┼────┼────┤
│    │    │    │    │  <- Full influence (distance 0)
│  ╱ │ ╲  │    │    │  <- Smooth falloff
│ ╱  │  ╲ │    │    │  <- Curves smoothly to zero
├────┼────┼────┼────┤
0.0  0.5  1.0  1.5  2.0
^                   ^
Peak influence      No influence

Mathematical function: B³(t)
• t ∈ [0, 2]
• Smooth everywhere (C² continuous)
• Never jumps (prevents artifacts)
• Normalized (sums to 1.0)

Code: FFDLattice::cubicBSplineWeight(t)
```

## File Dependencies

```
FFDPlugin (Main Entry)
    │
    ├─→ FFDPlugin.h/cpp
    │   └─→ ccGLPluginInterface (CloudCompare)
    │       └─→ Qt5 Core
    │
    ├─→ FFDAction.h/cpp
    │   └─→ FFDLattice.h
    │
    └─→ FFDLattice.h/cpp (Core Algorithm)
        ├─→ ccPointCloud (CloudCompare)
        ├─→ CCGeom (CloudCompare)
        └─→ std::vector, std::array (C++)

UI Components (TODO):
    
    FFDTool.h/cpp
        ├─→ ccOverlayDialog (CloudCompare)
        ├─→ FFDLattice
        └─→ ccGLWindow (CloudCompare)
    
    ccFFDLatticeDisplay.h/cpp
        ├─→ ccHObject (CloudCompare)
        ├─→ FFDLattice
        └─→ GL functions (ccGL)
```

## Implementation Phases

```
Phase 1: Visualization
┌─────────────────────────────┐
│ ccFFDLatticeDisplay         │ ← Create this
│ • Draw lattice grid          │
│ • Show control points        │
│ • Highlight selected points  │
└──────────────────┬───────────┘
                   │
                   ▼ Result: See lattice in viewport


Phase 2: Interaction
┌──────────────────────────────┐
│ ccFFDTool                    │ ← Create this
│ • Mouse picking               │
│ • Drag control points         │
│ • Update lattice              │
└──────────────────┬────────────┘
                   │
                   ▼ Result: Move points with mouse


Phase 3: Preview
┌──────────────────────────────┐
│ Real-time Preview            │
│ • Clone point cloud           │ ← Enhance this
│ • Deform clone with lattice   │
│ • Show alongside original     │
└──────────────────┬────────────┘
                   │
                   ▼ Result: See deformation live


Phase 4: Apply
┌──────────────────────────────┐
│ Finalize                     │
│ • Create new deformed cloud   │ ← Complete this
│ • Save as new entity          │
│ • Undo/redo support           │
└──────────────────┬────────────┘
                   │
                   ▼ Result: Get deformed cloud
```

## Performance Characteristics

```
Operation Complexity:
━━━━━━━━━━━━━━━━━━━━

Create Lattice:        O(1)
- Fixed 3×3×3 grid

Move Control Point:    O(1)
- Just update one value

Deform One Point:      O(lattice_nodes)
- 27 control points = 27 multiplications

Deform Cloud:          O(num_points × lattice_nodes)
- 100k points × 27 nodes = 2.7M operations
- ~50-100ms on modern CPU

Optimization Options:
• Use spatial indexing (only consider nearby control points)
• GPU acceleration (compute many points in parallel)
• Use octree (faster point lookup)
```

## Error Handling

```
User selects non-cloud?
        │
        ▼
   FFDPlugin::onNewSelection()
        │
        ├─→ Check if point cloud
        │       │
        │       No? → Disable action
        │       │
        │       Yes → Enable action
        │
        ▼
User clicks FFD action
        │
        ▼
FFDAction::performDeformation()
        │
        ├─→ Validate input
        │   │
        │   ├─→ No cloud? → Error message
        │   │
        │   └─→ Valid? → Continue
        │
        └─→ Create lattice & proceed

Point outside bounding box?
        │
        ▼
deformPoint() checks bounds
        │
        ├─→ Outside? → Return unchanged
        │
        └─→ Inside? → Deform normally
```

---

This diagram shows how all the pieces fit together. The core algorithm (FFDLattice) is complete and handles all the math. The next steps focus on visualization and interaction!
