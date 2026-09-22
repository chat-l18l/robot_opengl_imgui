#!/bin/sh
# Download a small set of third-party glTF models to look at in the viewer.
#
#     pixi run fetch-samples
#     pixi run viewer --model models/external/Buggy.glb
#
# They land in models/external/, which git ignores: the repository does not
# redistribute other people's models, it only says where to get them. Each
# model's source and licence is written to models/external/LICENSES.md.
#
# The Khronos CAD models (Buggy, 2CylinderEngine, GearboxAssy,
# ReciprocatingSaw) carry no explicit licence. Khronos publishes them for
# testing glTF viewers, which is what this is; do not redistribute them.
set -eu

OUT=models/external
OLD=https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Models/main/2.0
NEW=https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models
THREE=https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/models/gltf

mkdir -p "$OUT"
LICENSES="$OUT/LICENSES.md"
cat > "$LICENSES" <<'HEAD'
# Third-party models

Downloaded by `tools/fetch-sample-models.sh`. Not part of the repository.

| File | Source | Licence | Shows here |
|------|--------|---------|------------|
HEAD

# file | url | source | licence | how it looks in this viewer
while IFS='|' read -r file url source licence note; do
    [ -z "$file" ] && continue
    if [ -s "$OUT/$file" ]; then
        echo "have   $file"
    else
        echo "fetch  $file"
        curl -sfL --retry 2 --max-time 120 -o "$OUT/$file.part" "$url"
        mv "$OUT/$file.part" "$OUT/$file"
    fi
    echo "| \`$file\` | $source | $licence | $note |" >> "$LICENSES"
done <<LIST
Buggy.glb|$OLD/Buggy/glTF-Binary/Buggy.glb|Khronos glTF-Sample-Models|none stated, Khronos test asset|Deep node tree, many base colours: looks right
2CylinderEngine.glb|$OLD/2CylinderEngine/glTF-Binary/2CylinderEngine.glb|Khronos glTF-Sample-Models|none stated, Khronos test asset|CAD assembly, base colours: looks right
GearboxAssy.glb|$OLD/GearboxAssy/glTF-Binary/GearboxAssy.glb|Khronos glTF-Sample-Models|none stated, Khronos test asset|CAD assembly, base colours
ReciprocatingSaw.glb|$OLD/ReciprocatingSaw/glTF-Binary/ReciprocatingSaw.glb|Khronos glTF-Sample-Models|none stated, Khronos test asset|CAD assembly, base colours
MetalRoughSpheresNoTextures.glb|$NEW/MetalRoughSpheresNoTextures/glTF-Binary/MetalRoughSpheresNoTextures.glb|Khronos glTF-Sample-Assets|CC0-1.0|Test grid of spheres: looks right
CesiumMilkTruck.glb|$NEW/CesiumMilkTruck/glTF-Binary/CesiumMilkTruck.glb|Khronos glTF-Sample-Assets, by Cesium|CC-BY-4.0; Cesium logo is a trademark|Textured: comes out white
RobotExpressive.glb|$THREE/RobotExpressive/RobotExpressive.glb|three.js examples, model by Tomás Laulhé (Quaternius)|CC0-1.0|Skinned: parts drift apart until skinning is applied
LIST

echo "wrote $LICENSES"
