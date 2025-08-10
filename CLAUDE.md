# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Skeleposer is a cross-platform pose and transform management tool for Maya, Unity, and Unreal Engine. It's similar to Maya's Shape Editor but works with transforms and joints instead of blend shapes. Designed for character customization systems and complex facial rigs.

**Key Features:**
- Pose duplication, mirroring, and flipping
- Corrective and inbetween poses
- Two blend modes: additive (default) and replace
- Split poses tool for breaking complex poses into combinable chunks
- Layering capabilities with joint hierarchies
- Very lightweight single-node architecture
- Easily transferable between characters regardless of topology
- Optimized for complex facial rigs

**Documentation:** https://github.com/azagoruyko/skeleposer/wiki

## Build Commands

### Maya Plugin (C++)
```bash
# Build Maya plugin using CMake
mkdir build
cd build
cmake -DMAYA_VERSION=2024 ..
cmake --build . --config Release
```

### UE5 Plugin
- Build through Unreal Engine's build system
- Plugin location: `UE5Plugin/Skeleposer/`

### Unity Component
- No build required - Unity script in `Unity/Skeleposer.cs`

## Architecture

### Core Components

**Maya C++ Plugin (`source/`)**:
- `skeleposer.h/cpp` - Main MPxNode implementing pose blending system
- `blendMatrix.h/cpp` - Matrix blending node (version-aware for Maya 2020+)
- `stickyMatrix.h/cpp` - Matrix constraint utility
- `main.cpp` - Plugin registration/deregistration

**Maya Python Interface (`mayaModule/skeleposer/scripts/skeleposerEditor/`)**:
- `skeleposer.py` - Main Python API wrapper for the C++ node
- `ui.py` - PySide2-based UI for pose editing and management
- `utils.py` - Utility functions for Maya operations

**UE5 Integration (`UE5Plugin/Skeleposer/`)**:
- `RigUnit_Skeleposer.h/cpp` - Control Rig unit implementation
- Uses FLinearCurve for Maya RemapValue node functionality

**Unity Integration (`Unity/`)**:
- `Skeleposer.cs` - Unity component for pose-based skeletal animation

### Key Data Structures

**Directory System** - Hierarchical organization of poses:
- `Directory` class manages pose organization with recursive weight calculation
- Each directory has weight, parent index, and children indices
- Negative indices represent subdirectories, positive represent poses

**Pose System**:
- `Pose` struct contains weight, blend mode (ADDITIVE/REPLACE), and delta matrices
- `Joint` struct stores base matrix, joint orient, and associated poses
- Matrix-based transformations for skeletal animation

**Cross-Platform Consistency**:
- Same core data structures replicated in C++, C#, and Python
- JSON serialization for pose data exchange between platforms
- Consistent blend mode enums across all implementations

## Development Notes

- Maya plugin supports versions 2019-2024 with version-specific builds
- BlendMatrix node naming changes based on Maya version (2020+ uses "sblendMatrix")
- UI uses PySide2 for Maya integration
- All platforms support additive and replace blend modes for poses