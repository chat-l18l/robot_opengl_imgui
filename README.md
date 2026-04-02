# ImGui Robot Viewer — 6-DOF Robot Arm

Een single-window C++ applicatie die een 6-DOF robotarm rendert in een ImGui viewport met OpenGL 3.3. Alle joints zijn via sliders instelbaar.

## Dependencies (Ubuntu/Debian)

```bash
sudo apt install cmake g++ libglfw3-dev libeigen3-dev mesa-common-dev
```

## Bouwen

```bash
cd imgui-robot-viewer
mkdir build && cd build
cmake ..
make -j$(nproc)
```

De eerste keer downloadt CMake automatisch ImGui via FetchContent. OpenGL functies komen uit de systeem Mesa headers (libGL.so).

## Runnen

```bash
./robot_viewer
```

## Besturing

| Actie | Control |
|-------|---------|
| Roteren | Linkermuis knop + sleep |
| Pannen | Rechtermuis knop + sleep |
| Zoomen | Scrollwiel |
| Joint aanpassen | Sliders in rechterpaneel |
| Alles resetten | "Reset All Joints" knop |

## Projectstructuur

```
src/
├── main.cpp       — ImGui setup, render loop, control panel UI
├── robot.h        — Robot joint/link definitie (header-only interface)
├── robot.cpp      — Forward kinematics, rendering, mesh generatie
├── camera.h       — Orbit camera (Eigen, geen glm)
├── gl_core.h      — GL header wrapper (GL/gl.h + GL/glext.h)
└── gl_utils.h     — Shaders, matrix math, primitieve meshes (header-only)
```

## Extensie

- **Eigen robot:** Wijzig de constructor in `robot.cpp` — pas offsets, link lengths, kleuren aan
- ** Meer/v minder joints:** Voeg `addChild()` toe of verwijder joints in de constructor
- **Model laden:** Voeg TinyGLTF toe en laad een `.glb` bestand als alternatief voor code-gebaseerde geometrie
