# ImGui Robot Viewer — 6-DOF Robot Arm

A single-window C++ application that renders a 6-DOF robot arm in an ImGui
viewport with OpenGL 3.3. Every joint is driven from a slider.

Few dependencies on purpose: no glad, no glm, no scene graph library. On Linux
`libGL.so` already exports the GL 3.3 core entry points, Eigen covers the
matrix math, and the arm is a joint table rather than a model file.

## Dependencies (Ubuntu/Debian)

```bash
sudo apt install cmake g++ libglfw3-dev libeigen3-dev mesa-common-dev
```

## Build

```bash
cmake -S . -B build && cmake --build build -j$(nproc)
```

The first configure downloads Dear ImGui through FetchContent, pinned to
`v1.91.8-docking` because the panels use a dock space. The build type
defaults to `Release`; Eigen without an optimiser is an order of magnitude
slower, so an unset build type is worth avoiding. For a debug build with the
asserts active:

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug && cmake --build build-debug -j$(nproc)
```

## Run

```bash
./build/robot_viewer
```

## Controls

| Action | Control |
|--------|---------|
| Orbit | Left mouse button, drag |
| Pan | Right mouse button, drag |
| Zoom | Scroll wheel |
| Set a joint | Sliders in the right-hand panel |
| Reset one joint | `R` next to its slider |
| Reset everything | "Reset All Joints" |

A drag starts only over the 3D viewport, but continues until the button is
released, wherever the pointer goes. The viewport claims the pointer while it
is over it, so dragging inside it never moves the panel.

## Panels

Both panels live in a dock space: drag a tab to re-dock, split or tear one
loose. The first run lays the 3D view out on the left and the controls on the
right; after that the arrangement is restored from

```
$XDG_CONFIG_HOME/robot_viewer/imgui.ini    # or ~/.config/robot_viewer/imgui.ini
```

That path is fixed rather than relative to the working directory, so starting
from the source tree and from `build/` give the same layout. Delete the file to
get the default arrangement back.

Multi-viewport (panels as separate OS windows) is deliberately off: the 3D
scene is drawn into the main window's framebuffer under a scissor rectangle,
which a panel in its own OS window could not share.

## Layout

```
src/
├── main.cpp      — window, render loop, shaders, control panel
├── gl_core.h     — GL 3.3 entry points, no loader library
├── gl_math.h     — projection, view and transform matrices (Eigen)
├── gl_shader.*   — program building, uniform locations cached at link time
├── gl_mesh.*     — VAO/VBO ownership and the primitive generators
├── camera.*      — orbit camera
└── robot.*       — joint table, forward kinematics, arm rendering
```

## Extending

- **Different arm:** edit `s_arm_chain` in `src/robot.cpp`. One row per joint:
  axis, travel limits, offset from the parent, link radius, colour. The link
  length is the Y component of the offset, so there is only one place to
  change it.
- **More joints:** add rows. The builder wires each row as the child of the
  previous one and asserts that the chain stays a chain; a branching arm needs
  a parent index in the spec instead.
- **Loading a model:** add TinyGLTF and feed its meshes into `rbt_mesh_t` in
  place of the generated primitives.
