# Simulation of Thin Surface Membranes for Virtual Surgery

<p align="center">
  <img src="Results/teaser_uncropped.gif" alt="Simulation Teaser" width="700"/>
</p>

A real-time physics simulation of deformable thin-surface membranes with interactive cutting and fracture mechanics, built for virtual surgery research and training applications.

<!-- ![C++17](https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus)
![OpenGL](https://img.shields.io/badge/OpenGL-Rendering-green?logo=opengl) -->

## Overview

This project simulates the deformation and cutting of thin membrane surfaces — a fundamental problem in virtual surgery. A deformable membrane is modeled as a mass-spring system on a triangular mesh, and a rigid surgical instrument (e.g., a scalpel or sphere) can interact with, deform, and cut through the membrane in real time.

The simulation features:

- **Half-edge mesh data structure** for efficient topological queries and dynamic re-meshing
- **Mass-spring dynamics** with configurable stiffness, damping, and gravity
- **Forward & backward Euler integration** (backward Euler via a Preconditioned Conjugate Gradient solver)
- **Collision detection** between a rigid instrument and the deformable membrane (triangle-triangle intersection, sphere proxy)
- **Mesh cutting & fracture** through vertex splitting, edge re-meshing, and crack propagation

---

## Features

### Physics Simulation
| Feature | Details |
|---|---|
| Deformable body | Mass-spring system over a triangular surface mesh |
| Time integration | Forward Euler and Backward Euler (implicit) |
| Implicit solver | Preconditioned Conjugate Gradient (PCG) with sparse matrices |
| Material model | Configurable mass, stiffness (`ks`), and damping (`kd`) constants |
| Constraints | Velocity constraints on boundary and contact vertices |

### Cutting & Fracture
- Interactive path-based cutting along the membrane surface
- Intersection point computation between cut path and mesh edges
- Dynamic re-triangulation and vertex splitting to open cuts
- Crack tip tracking and propagation
- Ghost springs to maintain structural continuity across cuts

### Collision Detection & Response
- Triangle-triangle intersection tests
- Sphere-based collision proxy for the rigid instrument
- Collision response via velocity constraints or penalty forces

### Rendering
- OpenGL-based rendering with lighting and materials
- Dual-window setup: simulation view + debug wireframe view
- Mesh import via [Assimp](https://github.com/assimp/assimp) (supports `.obj` and other formats)
- Interactive camera controls and instrument manipulation

---

## Dependencies

| Library | Purpose |
|---|---|
| [Eigen](https://eigen.tuxfamily.org/) | Linear algebra (vectors, matrices, sparse solvers) |
| [GLFW](https://www.glfw.org/) | Window management and input handling |
| [OpenGL](https://www.opengl.org/) | 3D rendering |
| [Assimp](https://www.assimp.org/) | 3D model importing (`.obj`, `.mtl`, etc.) |


> **Note:** The `Makefile` currently references Homebrew paths under `/opt/homebrew/`. If your installation paths differ, update the `CFLAGS` and `LDFLAGS` in the `Makefile` accordingly.

---

## Building & Running

### Build

```bash
make
```

### Run

```bash
# Default mode (no cutting)
make run

# With cutting enabled
make run M=1

# With cutting + debug mode + specific crack mode
make run M=1 DM=1 SM=1
```

Or run the executable directly:

```bash
./simulation [mode] [debugMode] [crackMode]
```

| Argument | Values | Description |
|---|---|---|
| `mode` | `0` (default), `1` | `0` = deformation only, `1` = cutting enabled |
| `debugMode` | `0` (default), `1` | Enables additional debug visualization |
| `crackMode` | `0` (default), `1` | Selects crack propagation strategy |

### Clean

```bash
make clean
```

---

## Controls

The simulation supports a comprehensive set of keyboard and mouse controls for interacting with the environment, the surgical instrument, and the cutting mechanics.

### Camera Controls
| Key / Input | Action |
|---|---|
| `Left Mouse Drag` | Rotate camera (yaw / pitch) |
| `W` / `S` | Zoom in / zoom out |
| `A` / `D` | Pan camera left / right |
| `Left Shift` / `Left Ctrl` | Pan camera up / down |

### Simulation State
| Key | Action |
|---|---|
| `Space` | Pause / resume simulation |
| `O` | Activate physics (start simulation) |
| `I` | Deactivate physics (stop simulation) |

### Rigid Instrument Controls
| Key | Action |
|---|---|
| `Up` / `Down` | Move instrument along the Y-axis |
| `Left` / `Right` | Move instrument along the X-axis |
| `Right Shift` / `Right Ctrl` | Move instrument along the Z-axis |
| `1` | Show instrument mesh |
| `2` | Hide instrument mesh |

### Cutting & Fracture Controls (Main Window)
| Key | Action |
|---|---|
| `M` | Execute an automated, pre-defined path cut |
| `P` | Initialize collision-induced cutting (construct cut graph) |
| `L` | Process the cut and remesh |
| `U` | Manually split vertex at the current intersection (demo mode) |
| `Y` | Stop cutting |

### Debugging (Debug Window)
| Key | Action |
|---|---|
| `1` | Visual Strain Debug Mode 1 (Visualize edge elongation) |
| `2` | Visual Strain Debug Mode 2 (Visualize face deformation) |

---

## Project Structure

```
.
├── main.cpp                 # Application entry point and main loop
├── Makefile                 # Build configuration
├── stdc++.h                 # Precompiled header (std library includes)
│
├── include/                 # Header files
│   ├── common.hpp           # Type definitions, constants, math utilities
│   ├── particle.hpp         # Particle, Spring, Edge, Face, Constraint structs
│   ├── half_edge.hpp        # Half-edge mesh data structure & mesh operations
│   ├── deformable_body.hpp  # Deformable membrane (cutting, physics, rendering)
│   ├── rigid_body.hpp       # Rigid surgical instrument
│   ├── collision.hpp        # Collision detection & response
│   ├── camera.hpp           # Camera controls
│   ├── lighting.hpp         # Scene lighting
│   ├── draw.hpp             # OpenGL draw utilities
│   ├── gui.hpp              # Window management (GLFW wrapper)
│   ├── curve.hpp            # Bézier / parametric curve utilities
│   ├── path.hpp             # Cut path representation
│   ├── intersect.hpp        # Ray / plane intersection utilities
│   ├── material.hpp         # Material properties
│   ├── shape.hpp            # Geometric shape primitives (Sphere)
│   ├── debug.hpp            # Debug helpers
│   ├── text.hpp             # Text rendering utilities
│   ├── PCGSolver.hpp        # Preconditioned Conjugate Gradient solver
│   ├── SparseMatrix.hpp     # Custom sparse matrix implementation
│   ├── BLAS.hpp             # Basic linear algebra subroutines
│   └── TriTri.hpp           # Triangle-triangle intersection tests
│
├── src/                     # Source files (implementations of the above)
│
├── objects/                 # 3D model assets
    ├── sphere.obj / .mtl    # Sphere instrument model
    └── scalpel.obj / .mtl   # Scalpel instrument model
```

---

## Technical Details

### Implicit Integration (Backward Euler)

For stability with stiff springs, the physical simulation supports **backward Euler** integration, solving the resulting linear system:

```
(M - dt² · ∂f/∂x - dt · ∂f/∂v) · Δv = dt · f(x, v)
```

using a **Preconditioned Conjugate Gradient (PCG)** method with a custom sparse matrix representation.

### Cutting & Remeshing Algorithm

For an in-depth explanation of the half-edge data structure and the dynamic cutting and progressive re-triangulation algorithms used in this project, please refer to the dedicated algorithm repository: 

🔗 [Progressive Remeshing for Simulation of Surgical Cuts](https://github.com/pshrey795/Progressive-Remeshing-for-Simulation-of-Surgical-Cuts)
