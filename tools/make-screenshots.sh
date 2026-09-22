#!/bin/sh
# Regenerate the images used by README.md.
#
# Run from the repository root with the viewer already built:
#
#     cmake --build build -j$(nproc) && tools/make-screenshots.sh
#
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
          --view 40,18,4.6 --pose 0,0,25,-70,-35,0,0

# The 3D view alone, at three angles and zoom levels.
"$VIEWER" --shot "$OUT/arm-close.png" --size 900x620 --bare \
          --view 25,10,4.0 --pose 0,0,35,-80,-40,0,30
"$VIEWER" --shot "$OUT/arm-side.png" --size 900x620 --bare \
          --view 130,30,5.0 --pose 0,-50,50,-95,-25,0,0
"$VIEWER" --shot "$OUT/arm-reach.png" --size 900x620 --bare \
          --view 55,16,5.2 --pose 0,60,80,-110,-40,0,0

echo "wrote $OUT/*.png"
