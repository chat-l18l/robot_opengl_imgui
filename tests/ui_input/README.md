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

## What it does and does not exercise

It links the real `src/camera.cpp` and drives the real `rbt_camera_input()`, so
the camera is genuinely under test.

The ImGui calls in the test mirror `s_draw_viewport()` in `src/main.cpp`
instead of sharing code with it, because that function also renders. **If the
`Begin` / `InvisibleButton` / `End` sequence there changes, change it here
too**, or the test will keep passing while saying nothing about the
application.

Both fixes it guards were mutation-checked: replacing the `InvisibleButton`
with `IsWindowHovered()` fails the floating title bar case, and clearing
`ConfigWindowsMoveFromTitleBarOnly` fails the controls body case.
