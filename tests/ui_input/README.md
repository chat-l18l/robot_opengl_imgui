# ui_input — pointer routing

Checks that mouse input reaches the camera exactly when the 3D viewport owns
the pointer, and never when the user is arranging panels.

Every case here started as a bug found by hand, in the running application, by
noticing the arm turning when it had no business turning. Dear ImGui decides
hover and active state without a window or a GL context, so the whole
interaction replays from synthetic events in a few milliseconds.

## What it covers

| Case | Pass criterion |
|------|----------------|
| Floating panel, drag the title bar | The panel moves; the camera does not turn |
| Floating panel, drag inside the view | The camera turns; the panel stays put |
| Docked panel, drag the tab strip | The camera does not turn |
| Docked panel, drag inside the view | The camera turns; the panel stays put |
| Wheel outside the view | No zoom |
| Wheel over the view | Zooms in |
| Release outside the panel | The orbit ends; later movement turns nothing |
| Floating controls panel, drag its body | It does not move |
| Floating controls panel, drag its title bar | It moves |

## Running

Built by default; `-DBUILD_TESTING=OFF` skips it.

```bash
cmake --build build -j$(nproc) && ctest --test-dir build --output-on-failure
```

The binary can also be run directly, and prints one line per check:

```bash
./build/tests/ui_input/ui_input_test
```

It exits non-zero when any check fails.

## What it exercises

The real thing, not a copy. It links `src/ui_layout.cpp` and `src/camera.cpp`
and drives `rbt_ui_configure_io()`, `rbt_ui_dockspace()`,
`rbt_viewport_begin()` and `rbt_viewport_camera_input()` — the same functions
`main.cpp` calls. The only thing the test supplies itself is the dock
arrangement, because it needs panels both docked and floating, which the
application never varies.

Mutating `src/ui_layout.cpp` fails it:

| Change | Check that fails |
|--------|------------------|
| `InvisibleButton` replaced by `IsWindowHovered()` | Floating title bar: camera does not turn |
| `ConfigWindowsMoveFromTitleBarOnly` cleared | Controls body: does not move it |
| Wheel no longer gated on `hovered` | Wheel outside the view does not zoom |
| Drag no longer gated on `active` | Both title bar cases |

## Two timing details it depends on

The camera is fed from inside the frame, before `Render()`, exactly where
`main.cpp` does it. `io.MouseWheel` only holds its value for the duration of a
frame, so reading it afterwards yields zero and the wheel checks pass while
testing nothing.

ImGui trickles its input queue, so a pointer move and a wheel notch submitted
in the same frame do not both land in that frame. `s_wheel_at()` separates
them, as real hardware does.
