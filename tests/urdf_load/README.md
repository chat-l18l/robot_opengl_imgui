# urdf_load — the URDF loader

Checks that `rbt_urdf_load()` turns a URDF into the right frames, visuals and
motion, and that it refuses what it should.

Several mistakes in a URDF loader still produce a robot that looks plausible:
rpy composed in the wrong order, a cylinder left on the wrong axis, degrees
taken for radians, a colour rule turned round, a package not found. So the
test runs against `test_pkg/urdf/test_robot.urdf`, small enough to work out by
hand, and the expected positions are derived in the test's comments rather
than computed by the code under test.

## What it covers

| File | Checks |
|------|--------|
| `test_pkg/urdf/test_robot.urdf` | One frame per link, named by link for the root and by joint otherwise; parents; a commented-out link left out; continuous and revolute joints, limits converted from radians; prismatic shown fixed; a mimic joint still movable |
| same | A box scaled to its size; a cylinder along Z and centred; a mesh's `scale`; named, inline, absent and mesh-own colours, and which wins |
| same | The root turning Z-up into Y-up; an origin's rpy; a joint turning what hangs off it; the joint's rotation applied after its origin's |
| `rpy.urdf` | rpy composed as Rz · Ry · Rx about fixed axes, which a turn about a single axis cannot show |
| `test_pkg/meshes/marker_y_up.dae` | A COLLADA file declaring Y_UP taken as written by default, turned to Z-up with `--honor-up-axis`; an STL never turned |
| missing, not XML, `empty.urdf`, `loop.urdf` | Refused, with the current robot left untouched |
| example-robot-data, when present | Panda and UR5 load with 7 and 6 movable joints and their meshes found |

`package://test_pkg/...` is resolved by walking up from the URDF to the
`test_pkg` directory, the same way a ROS workspace or an installed package is
found.

## Running

```bash
pixi run test
```

The binary prints one line per check and exits non-zero on any failure:

```bash
pixi run ./build/tests/urdf_load/urdf_load_test
```

Through pixi, `CONDA_PREFIX` points at example-robot-data and the real robots
are checked too; without it that part reports "skipped" and does not fail. The
loader reports the prismatic, mimic and broken joints in the test robot on
stderr while the test runs; that is expected.

## Mutation-checked

Each of these changes fails the test with at least one reported check, never
a crash:

| Change | Checks that fail |
|--------|------------------|
| rpy composed as Rx · Ry · Rz | rpy order |
| Root left Z-up | The four kinematics checks |
| Cylinder not turned onto Z | Cylinder placement |
| Limits left in radians | Limit conversion |
| URDF colour over a mesh's own | Mesh-own colour |
| `--honor-up-axis` ignored | Up axis honoured on request |
| Joint rotation before its origin, in `robot.cpp` | Rotation after origin |
| `package://` not resolved | The mesh visuals, their placement and the up-axis checks |
