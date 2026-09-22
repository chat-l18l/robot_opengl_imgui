#!/bin/sh
# Regenerate the images used by README.md.
#
# Run through pixi, which builds the viewer first:
#
#     pixi run shots
#
# or from the repository root against any existing build in build/.
# The viewer renders these off-screen from its own built-in layout, so the
# result does not depend on how the panels happen to be arranged locally.
set -eu

VIEWER=${VIEWER:-./build/robot_viewer}
OUT=docs/images

if [ ! -x "$VIEWER" ]; then
    echo "no viewer at $VIEWER; build it first" >&2
    exit 1
fi
mkdir -p "$OUT"

# The whole application: panels, sliders and the arm.
"$VIEWER" --shot "$OUT/viewer.png" --size 1280x800 \
          --view 40,18,4.3 --target 0,1.7,0 --pose 0,0,25,-70,-35,0,0

# The 3D view alone, at three angles and zoom levels.
"$VIEWER" --shot "$OUT/arm-close.png" --size 900x620 --bare \
          --view 20,14,3.2 --target 0,2.0,0 --pose 0,0,35,-80,-40,0,30
"$VIEWER" --shot "$OUT/arm-side.png" --size 900x620 --bare \
          --view 145,22,4.2 --target 0,1.5,0 --pose 0,-50,50,-95,-25,0,0
"$VIEWER" --shot "$OUT/arm-reach.png" --size 900x620 --bare \
          --view 55,16,4.8 --target 0,1.0,0 --pose 0,60,80,-110,-40,0,0

echo "wrote $OUT/*.png"
