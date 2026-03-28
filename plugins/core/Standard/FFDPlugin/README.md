# FFD Plugin - Free Form Deformation for CloudCompare

## Overview

This plugin implements **Free Form Deformation (FFD)** for CloudCompare, allowing non-rigid transformations of point clouds using a controllable lattice structure.

### Key Concept

FFD works by:
1. Creating a 3D lattice (grid) of control points around the point cloud's bounding box
2. Allowing users to move individual lattice nodes in X, Y, Z directions
3. Using **smooth cubic B-spline interpolation** to ensure continuous, junction-free deformation
4. Transforming each point based on its weighted relationship to nearby control points

## Architecture

### Core Components

#### 1. **FFDLattice** (`FFDLattice.h` / `FFDLattice.cpp`)
The heart of the plugin. Manages:
- **Control Point Grid**: Stores a 3D array of control points
- **Point Deformation**: Uses cubic B-spline basis functions for smooth interpolation
- **Weight Computation**: Each point is influenced by nearby control points with smooth falloff
- **Continuity Guarantee**: B-spline functions provide C² continuity (smooth without discontinuities)

**Key Methods:**
- `moveControlPoint()`: Move a single lattice node
- `deformPoint()`: Transform a point based on current lattice state
- `reset()`: Undo all deformations
- `cubicBSplineWeight()`: Computes smooth interpolation weights

#### 2. **FFDPlugin** (`FFDPlugin.h` / `FFDPlugin.cpp`)
The main plugin interface that:
- Registers the action in CloudCompare's UI
- Manages plugin lifecycle
- Handles user selection validation

#### 3. **FFDAction** (`FFDAction.h` / `FFDAction.cpp`)
Action handler that:
- Gets the selected point cloud
- Creates the FFD lattice
- Acts as the entry point for the deformation workflow

## Building the Plugin

### Prerequisites
- CloudCompare source code
- CMake 3.1+
- Qt 5+ development libraries
- A C++11 compatible compiler

### Build Steps

1. **Enable the plugin in CMake:**
   ```bash
   cd build
   cmake .. -DPLUGIN_FFD_STANDARD=ON
   make
   ```

2. **Or enable via CMake GUI:**
   - Open CMake GUI
   - Find `PLUGIN_FFD_STANDARD` option
   - Check the box and configure

### Output
The compiled plugin will be placed in the CloudCompare plugins directory.

## Current Status

✅ **Completed:**
- Plugin skeleton and infrastructure
- FFD lattice data structure
- Point deformation engine with B-spline interpolation
- Smooth continuity implementation

⏳ **Next Steps (In Order of Priority):**

### Phase 1: Interactive Lattice Visualization (HIGH PRIORITY)
1. Create a new drawable entity class for the lattice (`ccFFDLatticeDisplay`)
2. Implement GL rendering of:
   - Lattice grid lines
   - Control point spheres
   - Highlight selected control points
3. Add to 3D view so user can see the lattice

### Phase 2: User Interaction (HIGH PRIORITY)
1. Create an interactive tool (`ccFFDTool` extending `ccOverlayDialog`)
2. Implement:
   - Mouse picking to select control points
   - Gizmo/manipulator for moving points
   - Real-time preview of deformation
   - Axis constraints (move only X, Y, or Z)
3. Add undo/redo support

### Phase 3: Deformation Application (HIGH PRIORITY)
1. Apply the deformation to selected point cloud
2. Create a new cloud with deformed points (preserve original)
3. Update color/scalar fields correctly
4. Handle meshes (if needed)

### Phase 4: Lattice Parameters (MEDIUM PRIORITY)
1. Create dialog to set lattice resolution (currently hardcoded 3x3x3)
2. Allow different resolution per axis
3. Save/load lattice configurations

### Phase 5: Advanced Features (LOW PRIORITY)
1. Non-uniform B-spline options
2. Radial Basis Function (RBF) deformation alternative
3. Symmetry planes for symmetric deformations
4. Animation/keyframe support

## File Structure

```
plugins/example/FFDPlugin/
├── CMakeLists.txt              # Plugin configuration
├── FFDPlugin.qrc               # Qt resource file
├── info.json                   # Plugin metadata
├── include/
│   ├── CMakeLists.txt
│   ├── FFDPlugin.h            # Main plugin class
│   ├── FFDLattice.h           # Lattice structure
│   └── FFDAction.h            # Action handler
├── src/
│   ├── CMakeLists.txt
│   ├── FFDPlugin.cpp
│   ├── FFDLattice.cpp
│   └── FFDAction.cpp
└── images/
    └── (icon.png - to be added)
```

## Technical Details

### Smooth Continuity Implementation

The key to smooth deformation without jumps is the **cubic B-spline basis function**:

```cpp
double cubicBSplineWeight(double t)
{
    // Provides C² continuity (twice continuously differentiable)
    // Ensures smooth transitions between control point influences
}
```

Each point's deformed position is computed as a weighted sum:
```
deformed_point = Σ(weight_i * control_point_i) / Σ(weight_i)
```

Where weights decay smoothly with distance, ensuring:
- ✅ No discontinuous jumps
- ✅ Smooth surface without artifacts
- ✅ Points outside influence radius return unchanged

### Weight Calculation

For each control point:
1. Compute normalized distance in lattice space
2. Apply B-spline basis function to each axis independently
3. Multiply axis weights for 3D weight
4. Only consider control points within 2-cell distance (efficiency)

## Usage (Once Complete)

1. Load a point cloud in CloudCompare
2. Select the cloud
3. Go to Tools → Free Form Deformation
4. A lattice grid will appear around the cloud
5. Click and drag control points to deform
6. Use axis constraints (X/Y/Z) for precise control
7. Apply to finalize or Cancel to discard

## Testing Recommendations

1. **Unit Tests:** Test FFDLattice deformation with known transformations
2. **Visual Tests:** Verify smooth deformation visually
3. **Edge Cases:** Test points outside bounding box, single-point clouds, etc.
4. **Performance:** Benchmark with large point clouds (100k+ points)

## References

- Sederberg & Parry, "Free-form deformation of solid geometric models," SIGGRAPH 1986
- Rueckert et al., "Nonrigid registration using free-form deformations," IEEE TMI 1999
- De Boor's algorithm for B-spline basis functions

## Contributing

To extend this plugin:
1. Follow CloudCompare's coding standards
2. Use consistent naming (FFD prefix for FFD-specific classes)
3. Add proper comments for complex algorithms
4. Test with various input sizes and configurations

---

**Status:** Work in Progress - Core algorithm complete, UI implementation pending
